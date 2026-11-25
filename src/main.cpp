/*
 * ============================================================================
 * WEATHER STATION - ESP32 Dev Module
 * ============================================================================
 * Version: PRODUCTION 26-06-2026 - HARDWARE SIM800
 * Hardware: ESP32 Dev Module + SIM800 (Serial2) + DHT22 + MT6701/AS5600 (I2C) +
 *           SD card (VSPI) + anemómetro de pulsos + pluviómetro (interrupción)
 *
 * Mapa de pines:
 *   SIM800   : RX2=16 (recibe del TX del modem) / TX2=17 (transmite al RX del modem)
 *              Alimentación por GPIO15 (sim_control) -> NPN Q4 -> MOSFET canal P Q1:
 *              lógica NO invertida (HIGH=ON, LOW=OFF). Solo el modem.
 *   RS485    : Serial1 TX=32 / RX=33 (módulo TTL-RS485 auto-dirección en v_sensor).
 *              12V del sensor (elevador SX1308) controlado por GPIO14/D14
 *              (RS485_PWR_PIN, HIGH=ON), independiente del modem.
 *   Sensores : Alimentación controlada por MOSFET canal P en GPIO2/D2 (LOW=ON, HIGH=OFF)
 *              (v_sensor: DHT, viento, encoder, OLED, conector I2C, RS485 TTL)
 *   OLED     : 0.96" SSD1306 128x64 por I2C (0x3C, mismo bus SDA=21/SCL=22), VCC = v_sensor
 *   Botones  : BUT1 "ok" = GPIO35, BUT2 "up" = GPIO27 (pull-up externo 100k, activos LOW).
 *              "ok" además despierta del deep sleep (EXT0) para consultar el estado.
 *   SD card  : MISO=19 MOSI=23 SCK=18 CS=5 (coincide con el bus VSPI por defecto del ESP32)
 *   Lluvia   : GPIO39 (entrada solo, sin pull interno) - wakeup por EXT1
 *   Viento   : GPIO13/D13 (velocidad por pulsos), MT6701 o AS5600 por I2C (SDA=21/SCL=22)
 *              para dirección — se elige por software (dirSensorType, /dirSensor.txt)
 *   RTC      : DS3231 por I2C (mismo bus, SDA=21/SCL=22, dirección fija 0x68) —
 *              da la marca de tiempo real; mantiene la hora con su propia batería
 *              de respaldo aunque el equipo pierda alimentación por completo.
 *   DHT      : GPIO4 (DHT22)
 *   Batería  : GPIO34 (ADC), divisor R10=330k / R11=100k.
 *   Botón AP : GPIO36 (modo configuración si está en LOW al arrancar).
 *              Entrada solo-lectura SIN pull interno: requiere resistencia
 *              pull-up EXTERNA a 3.3V; el botón conecta a GND al presionarlo.
 */

#define ARDUINOJSON_STRING_LENGTH_SIZE 4

#include <driver/rtc_io.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>

#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"

/* ====================== I2C BUS (MT6701) ======================== */
#include <Wire.h>
#define I2C_SDA 21
#define I2C_SCL 22

/* ====================== WIND DIRECTION SENSOR (MT6701 or AS5600) ======================== */
// Ambos son encoders magnéticos I2C intercambiables para leer la dirección del viento.
// dirSensorType ("MT6701"/"AS5600", persistido en /dirSensor.txt) selecciona cuál se usa
// en tiempo de ejecución; solo el módulo elegido se inicializa/lee.
#include "MT6701.h"
#include <AS5600.h>
MT6701 encoder;
AS5600 as5600;
String dirSensorType = "MT6701";

/* ====================== RTC DS3231 (marca de tiempo real) ======================== */
// Comparte el bus I2C con el encoder de dirección de viento (SDA=21/SCL=22,
// dirección I2C fija 0x68). A diferencia del esquema de "ancla" por software
// de más abajo (que depende de que algún dispositivo haya cargado /settime o
// de haber tenido red alguna vez), el DS3231 mantiene la hora real por sí
// mismo gracias a su batería de respaldo, incluso si el equipo pierde
// alimentación por completo. getEpochNow() lo usa como fuente primaria y
// solo cae al esquema de ancla por software si el DS3231 no responde.
#include <RTClib.h>
RTC_DS3231 ds3231;
bool ds3231Ready = false;

/* ====================== OLED 0.96" SSD1306 (I2C) + BOTONES ======================== */
// Mismo bus I2C que el encoder y el DS3231 (sin conflicto de direcciones:
// OLED 0x3C, AS5600 0x36, MT6701 0x06, DS3231 0x68). Su VCC sale de v_sensor
// (MOSFET de GPIO2): solo funciona con el riel de sensores encendido, en deep
// sleep queda sin alimentación y hay que re-inicializarlo en cada despertar.
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_W     128
#define OLED_H     64
#define OLED_ROTATION 3   // vertical (64x128); 1 = vertical girado 180°
#define OLED_ADDR  0x3C
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);
bool oledReady = false;
bool oledOpEnabled = true;  // mostrar el progreso en modo operación (/oledOp.txt)

// Botones del esquemático: pull-up externo de 100k a 3.3V, activos en LOW.
// GPIO35 es solo-entrada (sin pull interno) y pin RTC: sirve como wakeup EXT0.
#define BTN_OK_PIN 35   // BUT1 "ok"
#define BTN_UP_PIN 27   // BUT2 "up"
#define BTN_DEBOUNCE_MS 30
#define BTN_LONG_MS     2000
#define OLED_CONFIG_TIMEOUT_MS 120000UL  // apaga la pantalla en modo config sin uso
#define STATUS_VIEW_TIMEOUT_MS 30000UL   // consulta de estado (wakeup por botón)

/* ====================== RS485 (Modbus RTU) ======================== */
// Módulo TTL-RS485 con control de dirección automático (sin pin DE/RE),
// alimentado por v_sensor (GPIO2). El sensor en sí toma 12V del elevador
// SX1308 (A2), habilitado por GPIO14 (RS485_PWR_PIN), independiente del
// modem (antes compartía sim_control/GPIO15 y cada lectura encendía el SIM800).
// GPIO14 era el TRIG del sensor ultrasónico, que ya no existe.
// Usa UART1 (Serial1) porque UART2 (Serial2) es del SIM800.
// rs485Type (persistido en /rs485.txt, elegible desde el portal AP o por
// MQTT) define qué sensor hay conectado:
//   "SOIL": suelo 4-en-1, 9600 baud, regs 0..3 = hum/10, temp/10, EC, pH/10
//   "RAD" : piranómetro, 4800 baud, reg 0 = radiación (W/m²)
//   "LEVEL": nivel ultrasónico ACLT600, 9600 baud, reg 0x0001 = nivel crudo
//            (int16, sin el offset +1000 del reg 0x0000). Protocolo confirmado
//            con hardware real en el proyecto levelSIM7000. Tarda ~10 s en
//            arrancar y sus lecturas se estabilizan antes de promediar.
#define RS485_TX_PIN      32   // net "TXD" del esquemático
#define RS485_RX_PIN      33   // net "RXD" del esquemático
#define RS485_SLAVE_ADDR  1
#define RS485_PWR_PIN     14   // habilita el elevador 12V del sensor RS485
#define RS485_PWR_ON      HIGH
#define RS485_PWR_OFF     LOW
#define RS485_WARMUP_MS   1500 // elevador 12V + arranque del sensor
#define RS485_TIMEOUT_MS  500
#define RS485_RETRIES     3
String rs485Type = "NONE";     // "NONE" | "SOIL" | "RAD" | "LEVEL"

// Valida un valor de rs485Type (portal, MQTT, SPIFFS): lo desconocido = NONE.
String rs485Norm(const String &v) {
  return (v == "SOIL" || v == "RAD" || v == "LEVEL") ? v : String("NONE");
}

// ACLT600 (rs485Type "LEVEL"): ver readLevel() más abajo.
#define LEVEL_REG              0x0001
#define LEVEL_WARMUP_MS        10000  // el sensor tarda ~10 s en iniciar tras energizarse
#define LEVEL_STABLE_WINDOW    8      // lecturas consecutivas que deben variar poco...
#define LEVEL_STABLE_THRESHOLD 5      // ...a lo sumo esto (unidades crudas)
#define LEVEL_STABLE_MAX_TRIES 30     // tope de lecturas buscando estabilidad
#define LEVEL_AVG_SAMPLES      5      // lecturas finales promediadas
#define LEVEL_SAMPLE_DELAY_MS  300
#define LEVEL_MAX_FAILS        3      // fallos seguidos que abortan (sensor caído)

/* ====================== DHT22 TEMP/HUM SETTINGS ======================== */
#include <DHT.h>
#define DHT_PIN  4
#define DHT_TYPE DHT21   // AM2301 = DHT21
DHT dht(DHT_PIN, DHT_TYPE);

/* ====================== SD CARD SETTINGS (VSPI) ======================== */
#include <SPI.h>
#include <SD.h>
// Backend oficial ESP-IDF (empaquetado en el core arduino-esp32) usado solo
// por formatSD() para poder formatear tarjetas sin filesystem FAT válido:
// la librería Arduino SD.h no expone una operación de formateo.
#include <esp_vfs_fat.h>
#include <driver/sdspi_host.h>
#include <sdmmc_cmd.h>
#define SD_MISO 19
#define SD_MOSI 23
#define SD_SCK  18
#define SD_CS   5

bool sdReady = false;
String sdCardType = "N/A";
uint64_t sdTotalMB = 0;
uint64_t sdUsedMB = 0;
uint64_t sdFreeMB = 0;

// v2: columnas RS485 (ST,SH,EC,PH,RAD) añadidas al final. v3: + LV (nivel).
// Archivo nuevo en cada cambio de columnas para no mezclar filas de distinto
// ancho bajo un mismo encabezado; los anteriores quedan en la tarjeta
// (descargables desde la lista de archivos).
#define SD_LOG_PATH "/datalog_v3.csv"

/* ================ MARCA DE TIEMPO (hora real, sin RTC chip) ================ */
// El ESP32 no tiene un RTC con batería propia ni GPS/NTP en este hardware, así
// que la única forma de tener hora real (no solo "segundos desde que
// arrancó") es que el dispositivo que se conecta al AP en modo configuración
// preste su reloj: al cargar la página se envía la hora actual del
// navegador/teléfono a /settime, que ancla epoch (segundos Unix) contra
// getRtcTimeSeconds() (el RTC slow-clock interno, que sí sobrevive el deep
// sleep). Cada medición posterior calcula su hora real sumando al ancla los
// segundos de RTC transcurridos desde que se guardó — sin depender de que la
// conexión de red esté disponible en cada ciclo.
uint64_t timeBaseEpoch = 0;  // epoch (s) reportado por el navegador al conectar al AP
uint64_t timeBaseRtc = 0;    // getRtcTimeSeconds() en el instante en que se ancló
bool timeBaseSet = false;

/* ====================== WEBSERVER SETTINGS ======================== */
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

volatile bool serverReady = false;

AsyncWebServer server(80);

const char* ssid = "WeatherStation_config";
const char* password = "12345678";

String RUNssid = "";
String RUNpass = "";

/* ============================== SIM800 CONFIGURATIONS ============================== */
#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024

#define SerialMon Serial
#define SerialAT Serial2
#define TINY_GSM_DEBUG SerialMon
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#define GSM_PIN ""

String RUNapn = "claro.pe";  // configurable desde el portal AP, ver /apn.txt en SPIFFS
const char gprsUser[] = "";
const char gprsPass[] = "";

#include <TinyGsmClient.h>
#include "FS.h"

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

// ESP32 Dev Module pinout
#define UART_BAUD           9600
#define PIN_RX2             16
#define PIN_TX2             17
// sim_control: GPIO15 -> R1 47k -> base NPN Q4 (R13 100k a GND); el colector
// baja el gate del MOSFET canal P Q1. Por el NPN la lógica queda HIGH=ON.
#define MODEM_PWR_PIN       15
#define MODEM_PWR_ON        HIGH
#define MODEM_PWR_OFF       LOW

// Arranque suave del riel del modem: en vez de cerrar el MOSFET de golpe
// (el condensador de entrada del SIM800 y del SX1308 se cargan con un pico
// de corriente que hunde VCC y resetea el ESP32 por brownout), se pulsa
// sim_control por PWM con ciclo útil creciente durante MODEM_SOFTSTART_MS.
// 0 = desactivado (encendido directo como antes).
#define MODEM_SOFTSTART_MS   0     // 0 como en PROD (a603f7a): encendido directo
#define MODEM_SOFTSTART_FREQ 500   // Hz: período de 2 ms
#define MODEM_SOFTSTART_CH   7     // canal LEDC libre

/* ================== GESTIÓN DE ENERGÍA (BATERÍA) ================== */
// Umbrales sobre vBat() medido en reposo (modem apagado). DEPENDEN de que
// vBat() esté calibrado contra un multímetro (factor batCalFactor, se ajusta desde el portal).
#define BAT_VALID_MIN_V      2.50f  // por debajo: lectura inválida (sin divisor / alimentación USB) → no se aplica ninguna regla
#define BAT_MODEM_MIN_V      3.55f  // por debajo: no se enciende el modem (los picos de 2 A tirarían el sistema)
#define BAT_CRITICAL_V       3.40f  // por debajo: período de sleep × LOW_BAT_SLEEP_FACTOR
#define LOW_BAT_SLEEP_FACTOR 4
#define NET_BACKOFF_MAX_SKIP 8

// Durante la sesión del modem (registro en red, GPRS, MQTT) los picos de TX
// del SIM800 pueden hundir VCC del ESP32 un instante por debajo del umbral
// de brownout (2.43 V, ya el mínimo) y resetearlo aunque el ESP32 seguiría
// funcionando. 1 = desactivar el detector solo mientras el modem está
// encendido y restaurarlo al apagarlo. Es un parche: la solución real es de
// hardware (LDO de baja caída para el ESP32, ver CLAUDE.md).
#define MODEM_BROWNOUT_RIDE_THROUGH 1

// Banda GSM forzada en el SIM800 (AT+CBAND). En 850/900 MHz el SIM800
// transmite a 2 W (picos ~2 A); en 1800/1900 MHz a 1 W (picos ~1.2 A).
// "PCS_MODE" = solo 1900 (Claro Perú usa 1900 y 850 según zona: verificar
// cobertura antes de forzarla). "" = no tocar la configuración del modem.
#define MODEM_BAND ""

// Tiempos de estabilización del enlace ESP32 <-> SIM800 antes de
// modem.init(). Mientras el SIM800 termina de arrancar (RDY, +CFUN, +CPIN,
// Call Ready, SMS Ready) y hace sus primeros picos de RF, las respuestas
// llegaban corruptas ("OK" + basura) y TinyGSM daba el init por fallido.
#define MODEM_BOOT_MS        5000   // arranque con el UART cerrado (PROD: 3000)
#define MODEM_SYNC_MS       15000   // máx. enviando "AT" hasta el primer OK (autobaud)
#define MODEM_READY_MS      15000   // máx. esperando "SMS Ready" / silencio de URC
#define MODEM_QUIET_MS       2000   // silencio en la línea que da por terminado el arranque
#define MODEM_STABLE_OK         3   // "AT"->"OK" limpios y seguidos exigidos antes del init
#define MODEM_STABLE_MS     12000   // máx. buscando esos OK seguidos
#define MODEM_INIT_TRIES        3   // init sin cortar alimentación antes de reiniciar
      // tras fallos seguidos, intentar envío como mucho cada 8 ciclos

#define SENS_PWR_PIN        2    // MOSFET canal P: LOW=ON, HIGH=OFF (ctrl-sensors, D2 en esquemático)


/* ============================== DEVICE CONFIGURATIONS ============================== */

uint64_t uS_TO_S_FACTOR = 1000000UL;
uint64_t TIME_TO_SLEEP = 60;
unsigned long previousTime = 0;
boolean status = false;

RTC_DATA_ATTR uint64_t cycle_start_rtc = 0;
RTC_DATA_ATTR bool cycle_active = false;

// Ticks RTC (no segundos, para resolución sub-segundo) del último pulso de
// lluvia contado como válido; persistido a través del deep sleep para
// exigir una separación física mínima entre vaciadas reales (ver
// runRainCounter()).
RTC_DATA_ATTR uint64_t lastRainPulseRtcTicks = 0;

// Marca de tiempo RTC (no persistida, se recaptura en cada boot/wake) del
// inicio de este ciclo despierto. Se usa en deepSleepSystem() para restar
// el tiempo de procesamiento (modem, sensores, MQTT) del próximo sleep y
// así evitar el desfase acumulado en el período configurado.
uint64_t bootRtcSeconds = 0;

// Resumen del último ciclo de operación, en memoria RTC (sobrevive al deep
// sleep) para mostrarlo en el OLED al despertar con el botón "ok", sin tener
// que volver a medir ni conectarse.
RTC_DATA_ATTR char lastDataJson[512] = "";
RTC_DATA_ATTR char lastNetStatus[24] = "";
RTC_DATA_ATTR char lastSignal[16] = "";

// Gestión de energía. sleepFactor vive en RTC para que sleepRemainingCycle()
// (wakes por lluvia/botón) use el mismo período alargado que deepSleepSystem().
RTC_DATA_ATTR uint8_t sleepFactor = 1;
RTC_DATA_ATTR uint32_t opCycleCount = 0;
RTC_DATA_ATTR int16_t lastModemMv = -1;
// Último paso del modem en curso. RTC_NOINIT (no RTC_DATA): el bootloader
// reinicializa .rtc.data en cualquier reset que no sea un wake de deep
// sleep, y lo que interesa es justamente sobrevivir a un brownout.
#define MODEM_STEP_MAGIC 0x5A11C0DEUL
RTC_NOINIT_ATTR uint32_t modemStepMagic;
RTC_NOINIT_ATTR char modemStep[24];   // tensión que mide el propio SIM800 (AT+CBC)
// Diagnóstico del encendido del modem: ver mdiagFeed()/modemDiagFail().
struct ModemDiag {
  float vBat0;       // batería justo antes de encender el modem (V)
  float vBatMin;     // mínima medida mientras el modem estuvo encendido (V)
  int8_t rxPre;      // RX con pull-down interno y modem APAGADO: 1 = pull-up externo / retroalimentación
  int8_t rxOn;       // RX con pull-down, 150 ms después de energizar (antes de abrir el UART)
  int8_t rxLevel;    // nivel de RX tras los 3 s de arranque: 1 = TX del SIM800 vivo (idle HIGH)
  int8_t txIdle;     // nivel real del pad TX del ESP32 en reposo: 0 = línea cargada/en corto a GND
  uint16_t rxBytes;  // bytes recibidos del modem (arranque + pruebas AT)
  uint32_t atBaud;   // baudrate al que respondió "OK" (0 = a ninguno)
  uint8_t attempts;  // encendidos en este ciclo (modemRestart() suma uno)
  int16_t vSimMax;   // máxima tensión reportada por el SIM800 (AT+CBC), mV
  uint16_t badBytes; // bytes no imprimibles recibidos (errores de trama / ruido en RX)
  uint8_t hexN;      // primeros bytes crudos recibidos (para ver basura)
  uint8_t hex[4];
  char step[12];     // paso en que falló (INIT/RED/GPRS/MQTT/PUBL)
  char urc[22];      // mensaje relevante del modem (OVER-VOLTAGE..., RDY, CPIN...)
  char tail[22];     // últimos caracteres imprimibles recibidos
};
RTC_DATA_ATTR ModemDiag mdiag;
// SIM800: VBAT máx. 4.4 V; por encima avisa OVER-VOLTAGE WARNNING y a ~4.5 V
// se apaga solo (OVER-VOLTAGE POWER DOWN), p.ej. alimentado desde el USB.
#define SIM800_VMAX_MV 4400

String rstModemStep = "";      // paso del modem en curso si el último reset fue por brownout/WDT
// Racha de envíos fallidos (en SPIFFS: un reset por brownout puede borrar la
// memoria RTC, y justamente esos resets son los que hay que contar).
int netFailStreak = 0;
bool brownoutBoot = false;
bool lastCycleSent = false;  // resultado del envío de este ciclo (resumen OLED)   // este arranque viene de un reset por brownout
float batCalFactor = 1.0f;   // calibración de vBat() (/batCal.txt)
float cycleBatV = -1;        // batería medida en este ciclo, en reposo
uint32_t sensRailOnMs = 0;   // millis() al encender v_sensor (0 = apagado): calentamiento del DHT
uint32_t modemRailOnMs = 0;  // millis() al encender sim_control: arranque del SIM800
uint32_t rs485RailOnMs = 0;  // millis() al encender el 12V del RS485 (GPIO14)

/* Device settings */
boolean A_TH = false;
boolean A_WS = false;
boolean A_PR = false;

const char* PARAM_SSID = "ssidString";
const char* PARAM_PASS = "passString";
const char* PARAM_SERVER = "server";
const char* PARAM_PORT = "port";
const char* PARAM_RESET = "reset";
const char* PARAM_CONN_TYPE = "connType";

/* ====================== MQTT ======================== */
const char *broker = "broker.hivemq.com"; //24.199.125.52
const int mqtt_port = 1883;
const char *topicSubscribe = "jhpAI/HIDROMET/settings";
const char *topicPublish = "jhpAI/HIDROMET/data";

#include <PubSubClient.h>
TinyGsmClient client(modem);
PubSubClient simclient(client);

WiFiClient espClient;
PubSubClient wificlient(espClient);

/* ====================== DEVICE SETTINGS ======================== */
#define pinBattery        34   // ADC1_CH6, divisor R10=330k / R11=100k
#define pinWindVelocity   13
#define pinConfigAP       36
#define RAINCOUNT         GPIO_NUM_39  // INPUT-ONLY PIN - sin pull interno; la placa trae R8=10k a 3.3V
#define RAINCOUNT_MASK    (1ULL << 39)

// Anemómetro: sensor Hall SS451A, 2 imanes a 180° = 2 pulsos/vuelta.
// V(m/s) = (pulsos/2) * 2*pi*radio / tiempo * factor_calibracion
#define WIND_RADIUS_M          0.079   // distancia centro-copa
#define WIND_CAL_FACTOR         2.64   // factor empírico (túnel de viento)
#define WIND_SPEED_PER_PULSE   (TWO_PI * WIND_RADIUS_M * WIND_CAL_FACTOR / 2.0)  // m/s por (pulso/segundo)
// Con las copas detenidas y un imán justo sobre el sensor, el campo puede
// quedar en el borde del umbral del comparador y generar parpadeo eléctrico
// (chatter) en vez de una salida LOW estable: se ven flancos de bajada
// repetidos sin que haya rotación real. MAX_WIND_SPEED_MS descarta como
// ruido cualquier lectura por encima de lo físicamente plausible para este
// anemómetro, y WIND_PULSE_DEBOUNCE_US exige una separación mínima entre
// pulsos válidos (permite hasta ~82 m/s, muy por encima de lo esperado).
#define MAX_WIND_SPEED_MS      75.0
#define WIND_PULSE_DEBOUNCE_US 8000UL
#define RAIN_MM_PER_TIP   0.0355  // pluviometro balancin: 4.5 ml/volcado, colector de 12.7 cm diametro

// El pluviómetro también usa un sensor Hall (igual que el viento): medido
// con multímetro, sin campo magnético da ~2.3V y con campo da ~21mV. El
// nivel "sin campo" (2.3V) queda peligrosamente cerca del umbral de
// entrada digital del ESP32 (~2.45-2.6V para lógica de 3.3V), así que
// leer el pin como digital (HIGH/LOW) dispara falsos positivos por ruido
// aun estando en reposo. En vez de depender de ese umbral fijo, se lee el
// voltaje real por ADC y se decide con histéresis propia, aprovechando
// que la separación real entre ambos niveles (2.3V vs 21mV) es enorme.
#define RAIN_ADC_FIELD_MV   500   // por debajo: campo magnético presente (imán encima)
#define RAIN_ADC_IDLE_MV   1500   // por encima: sin campo (reposo). Entre 500-1500mV: zona muerta, se ignora
#define RAIN_PULSE_MIN_INTERVAL_MS 300  // el balancín no puede vaciarse dos veces más rápido que esto

String connectionType = "";

// Contador de pulsos de lluvia en vivo (modo config); definido más abajo,
// declarado aquí para que setupServerRoutes() pueda reiniciarlo.
extern volatile unsigned long rainPulseCountISR;

/* ====================== SENSOR CACHE ======================== */
struct {
  String temp = "--", hum  = "--";
  String rain = "--", wvel = "--", wdir = "--", bat = "--";
  String rainStatus = "--", rainCount = "--";
  String rsT = "--", rsH = "--", rsEC = "--", rsPH = "--", rsRad = "--", rsLv = "--";
  String rs485 = "--";  // resumen de una línea para la UI web
} sensCache;

/* ====================== HTML PAGE ======================== */
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
<title>Weather Station</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta charset="UTF-8">
<link rel="icon" href="data:image/svg+xml;base64,PHN2ZyB4bWxucz0naHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmcnIHZpZXdCb3g9JzAgMCAxMDAgMTAwJz48dGV4dCB5PScuOWVtJyBmb250LXNpemU9JzkwJz7im4U8L3RleHQ+PC9zdmc+">
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Arial,sans-serif;background:linear-gradient(135deg,#667eea,#764ba2);padding:10px;min-height:100vh}
.c{max-width:700px;margin:0 auto;background:#fff;padding:20px;border-radius:12px;box-shadow:0 8px 30px rgba(0,0,0,.25)}
h2{color:#2c3e50;margin-bottom:4px;text-align:center;font-size:22px}
.offline{text-align:center;font-size:11px;color:#7f8c8d;margin-bottom:15px;letter-spacing:.3px}
.offline .dot{display:inline-block;width:7px;height:7px;border-radius:50%%;background:#28a745;margin-right:5px;box-shadow:0 0 0 3px rgba(40,167,69,.2)}
.i{background:linear-gradient(135deg,#667eea,#764ba2);color:#fff;padding:12px;border-radius:8px;margin-bottom:15px;text-align:center;font-size:13px;font-weight:500}
.tb{display:flex;gap:10px;margin-bottom:15px}
.tb>*{flex:1}
.btn{display:flex;align-items:center;justify-content:center;gap:6px;padding:12px 16px;border:none;border-radius:8px;cursor:pointer;font-size:14px;font-weight:600;transition:all .2s;text-transform:uppercase}
.br{background:linear-gradient(135deg,#28a745,#20c997);color:#fff;box-shadow:0 4px 15px rgba(40,167,69,.3)}
.br:hover{transform:translateY(-2px);box-shadow:0 6px 20px rgba(40,167,69,.4)}
.be{background:linear-gradient(135deg,#dc3545,#c82333);color:#fff;box-shadow:0 4px 15px rgba(220,53,69,.3)}
.be:hover{transform:translateY(-2px);box-shadow:0 6px 20px rgba(220,53,69,.4)}
.s{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-bottom:15px}
.sb{background:#fff;border:2px solid #e8e8e8;padding:12px 8px;border-radius:10px;text-align:center;transition:all .2s}
.sb:hover{border-color:#667eea;box-shadow:0 4px 12px rgba(102,126,234,.2)}
.sv{font-size:20px;font-weight:700;margin:6px 0;color:#2c3e50}
.sl{font-size:11px;color:#7f8c8d;text-transform:uppercase;letter-spacing:.5px}
.sc{background:#f8f9fa;padding:15px;margin:12px 0;border-radius:10px;border-left:4px solid #667eea}
.sc h3{color:#2c3e50;margin-bottom:10px;font-size:14px;display:flex;align-items:center;gap:6px}
label{display:block;margin:8px 0;cursor:pointer;font-size:13px;color:#2c3e50;padding:8px;background:#fff;border-radius:6px;transition:background .2s}
label:hover{background:#f0f0f0}
input[type=checkbox]{margin-right:8px;width:16px;height:16px;vertical-align:middle;accent-color:#667eea}
input[type=text],select{width:100%%;padding:10px;margin:4px 0 8px;border:2px solid #e0e0e0;border-radius:6px;font-size:13px;transition:border .2s}
input[type=text]:focus,select:focus{outline:none;border-color:#667eea}
input[type=submit]{background:linear-gradient(135deg,#667eea,#764ba2);color:#fff;padding:10px 15px;border:none;border-radius:6px;cursor:pointer;font-size:13px;font-weight:600;width:100%%;transition:all .2s}
input[type=submit]:hover{transform:translateY(-1px);box-shadow:0 4px 12px rgba(102,126,234,.3)}
.fr{margin:8px 0}
.fr label{background:0;padding:0;margin-bottom:4px;font-weight:600}
.fr label:hover{background:0}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.ps{font-size:11px;color:#888;margin-top:4px}
.rc{background:#fff;border:2px solid #e8e8e8;border-radius:10px;padding:12px;margin-bottom:8px;display:flex;justify-content:space-between;align-items:center}
.rc .sv{margin:0;font-size:24px}
@media(max-width:600px){.s{grid-template-columns:repeat(2,1fr)}.g2{grid-template-columns:1fr}}
.dt{width:100%%;overflow-x:auto;margin-top:8px;border-radius:6px;border:1px solid #e8e8e8}
.dt table{width:100%%;border-collapse:collapse;font-size:11px}
.dt th,.dt td{border-bottom:1px solid #eee;padding:6px 8px;text-align:left;white-space:nowrap}
.dt th{background:#f8f9fa;color:#2c3e50;position:sticky;top:0}
.dt tr:hover td{background:#f8f9fa}
</style>
</head>
<body>
<div class="c">
<h2>&#x26C5; Weather Station</h2>
<div class="offline"><span class="dot"></span>Modo Punto de Acceso — standalone, sin conexión a internet</div>
<div class="i"><b>MAC:</b> %MAC% | <b>Bat:</b> <span id="bat">--</span>V</div>

<div class="tb">
<button class="btn br" onclick="location.reload()">&#x21bb; Actualizar</button>
<form action="/get"><input type="hidden" name="reset" value="1"><button type="submit" class="btn be">&#x2714; Guardar y Salir</button></form>
</div>

<div class="sc">
<h3>&#x1F4CA; Lecturas en Tiempo Real</h3>
<div class="s">
<div class="sb"><div class="sl">&#x1F321; Temp</div><div class="sv" id="temp">--</div></div>
<div class="sb"><div class="sl">&#x1F4A7; Humedad</div><div class="sv" id="hum">--</div></div>
<div class="sb"><div class="sl">&#x1F327; Lluvia</div><div class="sv" id="rain">--</div></div>
<div class="sb"><div class="sl">&#x2601; Estado Lluvia</div><div class="sv" id="rainStatus">--</div></div>
<div class="sb"><div class="sl">&#x1F4A8; Vel. Viento</div><div class="sv" id="wvel">--</div></div>
<div class="sb"><div class="sl">&#x1F9ED; Dir. Viento</div><div class="sv" id="wdir">--</div></div>
<div class="sb" style="grid-column:1/-1"><div class="sl">&#x1F50C; RS485 (cada 30 s)</div><div class="sv" id="rs485" style="font-size:14px">--</div></div>
</div>
<div class="ps">Última actualización: <span id="upd">--</span> (cada 1 s, local)</div>
</div>
<div class="sc">
<h3>&#x1F6E0; Sensores</h3>
<form action="/get">
<label><input type="checkbox" name="A_TH" value="1" %A_TH_CHECKED%>Temperatura y Humedad (DHT22)</label>
<label><input type="checkbox" name="A_PR" value="1" %A_PR_CHECKED%>Precipitación</label>
<label><input type="checkbox" name="A_WS" value="1" %A_WS_CHECKED%>Velocidad y Dirección del Viento</label>
<input type="submit" value="Guardar Sensores">
</form>
<form action="/get">
<label>Módulo de Dirección (Viento)</label>
<select name="dirSensor">
<option value="MT6701" %MT6701_SELECTED%>MT6701 (encoder magnético)</option>
<option value="AS5600" %AS5600_SELECTED%>AS5600 (brújula magnética)</option>
</select>
<input type="submit" value="Guardar Módulo">
</form>
<form action="/get">
<label>Sensor RS485 (Modbus RTU, dirección 1)</label>
<select name="rs485">
<option value="NONE" %RS485_NONE%>Ninguno</option>
<option value="SOIL" %RS485_SOIL%>Suelo: temp, humedad, EC, pH (9600 baud)</option>
<option value="RAD" %RS485_RAD%>Radiación solar W/m² (4800 baud)</option>
<option value="LEVEL" %RS485_LEVEL%>Nivel ACLT600, crudo (9600 baud, ~15 s por lectura)</option>
</select>
<input type="submit" value="Guardar RS485">
</form>
<form action="/get">
<input type="hidden" name="oledForm" value="1">
<label><input type="checkbox" name="oledOp" value="1" %OLED_OP_CHECKED%>Mostrar progreso en OLED en modo operación (mantiene v_sensor encendido todo el ciclo, más consumo)</label>
<input type="submit" value="Guardar Pantalla">
</form>
<div class="rc">
<div><div class="sl">&#x1F327; Vaciadas acumuladas</div><div class="sv" id="rainCounterVal">--</div></div>
<form action="/get">
<input type="hidden" name="resetRain" value="1">
<button type="submit" class="btn be">&#x21bb; Reiniciar</button>
</form>
</div>
<div class="ps">Total persistente en memoria: no se reinicia solo en cada reporte, únicamente con el botón de arriba.</div>
</div>
<div class="sc">
<h3>&#x23F1; Período de Muestreo</h3>
<form action="/get">
<select name="period">
<option value="120" %P120%>2 min (120 seg)</option>
<option value="300" %P300%>5 min (300 seg)</option>
<option value="900" %P900%>15 min (900 seg)</option>
<option value="1800" %P1800%>30 min (1800 seg)</option>
<option value="3600" %P3600%>1 hora (3600 seg)</option>
<option value="7200" %P7200%>2 horas (7200 seg)</option>
</select>
<div class="ps">Tiempo actual: %PERIOD_SEC% segundos (%PERIOD_MIN% min)</div>
<input type="submit" value="Guardar Período">
</form>
</div>

<div class="sc">
<h3>&#x1F4BE; Tarjeta SD</h3>
<div class="s">
<div class="sb"><div class="sl">Estado</div><div class="sv">%SD_PRESENT%</div></div>
<div class="sb"><div class="sl">Tipo</div><div class="sv">%SD_TYPE%</div></div>
<div class="sb"><div class="sl">Total</div><div class="sv">%SD_TOTAL% MB</div></div>
<div class="sb"><div class="sl">Usado</div><div class="sv">%SD_USED% MB</div></div>
<div class="sb"><div class="sl">Libre</div><div class="sv">%SD_FREE% MB</div></div>
</div>
<div class="tb" style="margin-top:10px">
<button type="button" class="btn br heavy-btn" onclick="logNow()">&#x1F4DD; Guardar Lectura Ahora</button>
<button type="button" class="btn br" onclick="downloadSD()">&#x2B07; Descargar CSV</button>
</div>
<div class="tb" style="margin-top:8px">
<button type="button" class="btn br heavy-btn" onclick="loadSDFileList()">&#x1F4C2; Ver Archivos SD</button>
<button type="button" class="btn be heavy-btn" onclick="formatSD()">&#x1F5D1; Formatear Tarjeta</button>
</div>
<div id="sdResult" class="ps">"Guardar Lectura Ahora" agrega una fila real a la SD sin esperar el ciclo de sleep — útil para probar mientras se está en el portal. Formatear borra todos los datos de la tarjeta.</div>
<div class="dt" id="sdFileListWrap" style="margin-top:8px"></div>
</div>

<div class="sc">
<div class="ps">&#x1F551; Hora de referencia: <span id="timeBase">%TIME_BASE%</span></div>
</div>

<div class="sc">
<h3>&#x1F4CB; Datos Registrados (últimos 50)</h3>
<button type="button" class="btn br" style="width:100%%" onclick="loadDataTable()">&#x1F504; Cargar / Actualizar Tabla</button>
<div class="dt" id="dataTableWrap"><div class="ps" style="padding:8px">Sin cargar aún.</div></div>
</div>

<div class="sc">
<h3>&#x1F4E4; Datos Pendientes de Envío</h3>
<div class="ps">Lecturas que no se pudieron enviar y esperan en la memoria interna (se reenvían solas en el próximo envío exitoso): <b id="pendCount">%PENDING_COUNT%</b></div>
<button type="button" class="btn br heavy-btn" style="width:100%%;margin-bottom:10px" onclick="loadPending()">&#x1F441; Ver Pendientes</button>
<button type="button" class="btn be heavy-btn" style="width:100%%" onclick="clearPending()">&#x1F5D1; Limpiar Pendientes</button>
<div id="pendResult" class="ps"></div>
<div class="dt" id="pendTableWrap"></div>
</div>

<div class="sc">
<h3>&#x1F50B; Calibración de batería</h3>
<div class="ps">Lectura actual: %BAT_NOW% V (factor %BAT_CAL%). Mide la batería con un multímetro en el mismo momento y escribe el valor real; 0 = quitar calibración.</div>
<form action="/get"><div class="fr"><label>Voltaje real (V)</label><input type="number" step="0.01" min="0" max="6" name="batReal" placeholder="3.95"></div><input type="submit" value="Calibrar"></form>
</div>

<div class="sc" id="lteCard" style="%LTE_CARD_STYLE%">
<h3>&#x1F4E1; Diagnóstico LTE (SIM800)</h3>
<form action="/get"><div class="fr"><label>APN</label><input type="text" name="apn" value="%apn%" placeholder="claro.pe / internet.movistar.pe"></div><input type="submit" value="Guardar"></form>
<div class="ps">Ej: "claro.pe" para Claro, "internet.movistar.pe" para Movistar.</div>
<button type="button" class="btn br heavy-btn" style="width:100%%;margin-bottom:10px" onclick="lteTest()">&#x1F4E1; Probar Conexión</button>
<div id="lteResult" class="ps">Sin ejecutar aún. El modem se enciende solo al presionar el botón.</div>
</div>

<div class="sc">
<h3>&#x1F4F6; Conexión</h3>
<form action="/get">
<select name="connType">
<option value="WIFI" %WIFI_SELECTED%>WiFi</option>
<option value="LTE" %LTE_SELECTED%>LTE</option>
<option value="DATALOGER" %DATALOGER_SELECTED%>Dataloger local (solo SD, sin envío)</option>
</select>
<div class="ps">En "Dataloger local" el equipo no enciende WiFi ni el modem: cada lectura solo se guarda en la SD, con la hora ancla de abajo.</div>
<input type="submit" value="Guardar">
</form>
<div class="g2">
<form action="/get"><div class="fr"><label>SSID</label><input type="text" name="ssidString" value="%ssidString%"></div><input type="submit" value="Guardar"></form>
<form action="/get"><div class="fr"><label>Password</label><input type="text" name="passString" value="%passString%"></div><input type="submit" value="Guardar"></form>
</div>
</div>
<div class="sc">
<h3>&#x1F4E1; Servidor MQTT</h3>
<div class="g2">
<form action="/get"><div class="fr"><label>IP</label><input type="text" name="server" value="%server%"></div><input type="submit" value="Guardar"></form>
<form action="/get"><div class="fr"><label>Puerto</label><input type="text" name="port" value="%port%"></div><input type="submit" value="Guardar"></form>
</div>
<form action="/get"><div class="fr"><label>Topic Publish</label><input type="text" name="topicPub" value="%topicPub%"></div><input type="submit" value="Guardar"></form>
<form action="/get"><div class="fr"><label>Topic Subscribe</label><input type="text" name="topicSub" value="%topicSub%"></div><input type="submit" value="Guardar"></form>
</div>
</div>
<script>
// El equipo atiende una sola solicitud lenta a la vez (lectura de sensores,
// formateo, prueba LTE): mientras alguna esté en curso se pausa el polling
// de /sensors cada 1 s y se bloquean los demás botones "pesados", porque el
// AP del ESP32 solo soporta unas pocas conexiones TCP simultáneas — sin
// esto, el polling de fondo satura esas conexiones durante los ~15-20 s que
// tarda una lectura completa y las peticiones nuevas (de otro botón, o el
// mismo polling) terminan en "Failed to fetch" aunque el equipo siga vivo.
var busy=false;
function setBusy(b){busy=b;document.querySelectorAll('.heavy-btn').forEach(function(el){el.disabled=b;});}
function u(){if(busy)return;fetch('/sensors').then(r=>r.json()).then(d=>{document.getElementById('temp').innerText=d.temp!='NA'?d.temp+'°C':'--';document.getElementById('hum').innerText=d.hum!='NA'?d.hum+'%%':'--';document.getElementById('rain').innerText=d.rain!='NA'?d.rain+' mm':'--';document.getElementById('rainStatus').innerText=d.rainStatus!='NA'?d.rainStatus:'--';document.getElementById('wvel').innerText=d.wvel!='NA'?d.wvel+' m/s':'--';document.getElementById('wdir').innerText=d.wdir!='NA'?d.wdir+'°':'--';document.getElementById('bat').innerText=d.bat||'--';document.getElementById('rs485').innerText=d.rs485&&d.rs485!='NA'?d.rs485:'Deshabilitado';var rc=document.getElementById('rainCounterVal');if(rc)rc.innerText=d.rainCount!='NA'?d.rainCount:'--';document.getElementById('upd').innerText=new Date().toLocaleTimeString();}).catch(e=>console.log(e));}
u();setInterval(u,1000);
function syncTime(){fetch('/settime?epoch='+Math.floor(Date.now()/1000)).then(r=>r.json()).then(d=>{var tb=document.getElementById('timeBase');if(tb&&d.ok)tb.innerText=d.timestamp+' UTC (ancla)';}).catch(e=>console.log(e));}
syncTime();
function formatSD(){if(busy)return;if(!confirm('¿Formatear la tarjeta SD? Se borrarán todos los datos.'))return;setBusy(true);var r=document.getElementById('sdResult');r.innerText='Formateando...';fetch('/formatsd').then(r=>r.json()).then(d=>{r.innerText=d.msg;}).catch(e=>{r.innerText='Error de comunicación con el dispositivo: '+e;}).finally(()=>setBusy(false));}
function downloadSD(name){window.location.href=name?('/downloadsd?file='+encodeURIComponent(name)):'/downloadsd';}
function logNow(){if(busy)return;setBusy(true);var r=document.getElementById('sdResult');r.innerText='Leyendo sensores y guardando... (puede tardar unos segundos)';fetch('/lognow').then(r=>r.json()).then(d=>{r.innerText=d.msg;setBusy(false);loadDataTable();}).catch(e=>{r.innerText='Error de comunicación con el dispositivo: '+e;setBusy(false);});}
function loadSDFileList(){if(busy)return;setBusy(true);var w=document.getElementById('sdFileListWrap');w.innerHTML='<div class="ps" style="padding:8px">Cargando...</div>';fetch('/sdlist').then(r=>r.json()).then(d=>{if(!d.files||d.files.length==0){w.innerHTML='<div class="ps" style="padding:8px">Sin archivos en la SD.</div>';return;}var html='<table><thead><tr><th>Archivo</th><th>Tamaño</th><th></th></tr></thead><tbody>';d.files.forEach(function(f){var kb=(f.size/1024).toFixed(1);html+='<tr><td>'+f.name+'</td><td>'+kb+' KB</td><td><button type="button" class="btn br" style="padding:4px 10px" onclick="downloadSD(\''+f.name+'\')">&#x2B07;</button></td></tr>';});html+='</tbody></table>';w.innerHTML=html;}).catch(e=>{w.innerHTML='<div class="ps" style="padding:8px">Error de comunicación: '+e+'</div>';}).finally(()=>setBusy(false));}
function loadDataTable(){var w=document.getElementById('dataTableWrap');w.innerHTML='<div class="ps" style="padding:8px">Cargando...</div>';fetch('/viewdata').then(r=>r.text()).then(t=>{var lines=t.split('\n').filter(l=>l.length>0);if(lines.length<2){w.innerHTML='<div class="ps" style="padding:8px">Sin datos registrados aún.</div>';return;}var head=lines[0].split(',');var html='<table><thead><tr>';head.forEach(h=>html+='<th>'+h+'</th>');html+='</tr></thead><tbody>';for(var i=lines.length-1;i>=1;i--){var cols=lines[i].split(',');html+='<tr>';cols.forEach(c=>html+='<td>'+c+'</td>');html+='</tr>';}html+='</tbody></table>';w.innerHTML=html;}).catch(e=>{w.innerHTML='<div class="ps" style="padding:8px">Error de comunicación: '+e+'</div>';});}
function loadPending(){if(busy)return;setBusy(true);var w=document.getElementById('pendTableWrap');w.innerHTML='<div class="ps" style="padding:8px">Cargando...</div>';fetch('/pending').then(r=>r.text()).then(t=>{var lines=t.split('\n').filter(l=>l.trim().length>0);document.getElementById('pendCount').innerText=lines.length;if(lines.length==0){w.innerHTML='<div class="ps" style="padding:8px">No hay datos pendientes.</div>';return;}var cols=['timestamp','T','H','WV','WD','RA','vB','ST','SH','EC','PH','RAD'];var rows=[];lines.forEach(function(l){try{rows.push(JSON.parse(l));}catch(e){rows.push({timestamp:'(línea inválida)'});}});cols=cols.filter(function(c){return rows.some(function(o){return o[c]!=null;});});var MAX=100;var html=(rows.length>MAX?'<div class="ps" style="padding:8px">Mostrando los '+MAX+' más recientes de '+rows.length+'.</div>':'')+'<table><thead><tr>';cols.forEach(c=>html+='<th>'+c+'</th>');html+='</tr></thead><tbody>';for(var i=rows.length-1;i>=0&&i>=rows.length-MAX;i--){html+='<tr>';cols.forEach(c=>html+='<td>'+(rows[i][c]!=null?rows[i][c]:'')+'</td>');html+='</tr>';}html+='</tbody></table>';w.innerHTML=html;}).catch(e=>{w.innerHTML='<div class="ps" style="padding:8px">Error de comunicación: '+e+'</div>';}).finally(()=>setBusy(false));}
function clearPending(){if(busy)return;if(!confirm('¿Borrar todos los datos pendientes de envío? Se perderán los que no estén también en la SD.'))return;setBusy(true);var r=document.getElementById('pendResult');fetch('/pendingclear').then(r=>r.json()).then(d=>{r.innerText=d.msg;document.getElementById('pendCount').innerText=d.count;document.getElementById('pendTableWrap').innerHTML='';}).catch(e=>{r.innerText='Error de comunicación con el dispositivo: '+e;}).finally(()=>setBusy(false));}
function lteTest(){if(busy)return;setBusy(true);var r=document.getElementById('lteResult');r.innerText='Probando... (enciende y estabiliza el modem, puede tardar 1-2 min)';fetch('/ltetest').then(r=>r.json()).then(d=>{var lines=[];if(d.modemInfo)lines.push('Modem: '+d.modemInfo);if(d.signal!=null)lines.push('Señal (CSQ): '+d.signal+(d.signal==99?' (desconocida)':''));if(d.simReady!=null)lines.push('SIM lista: '+(d.simReady?'sí':'no'));if(d.networkRegistered!=null)lines.push('Registrado en red: '+(d.networkRegistered?'sí':'no'));if(d.operator)lines.push('Operador: '+d.operator);if(d.gprsConnected!=null)lines.push('GPRS/Internet: '+(d.gprsConnected?'sí':'no'));if(d.ip)lines.push('IP: '+d.ip);if(d.vBat0!=null)lines.push('Bateria: '+d.vBat0+' V -> min '+d.vBatMin+' V'+(d.vMod>0?' | SIM800: '+(d.vMod/1000).toFixed(2)+' V':''));if(d.rxLevel!=null)lines.push('RX modem: '+(d.rxLevel==1?'ALTO':d.rxLevel==0?'BAJO (sin energia?)':'--')+', '+d.rxBytes+' bytes, AT: '+d.atBaud);if(d.rxPre!=null)lines.push('RX apagado/encendido/3s: '+d.rxPre+' > '+d.rxOn+' > '+d.rxLevel+(d.rxPre==1?' (alto SIN modem: pull-up externo)':''));if(d.txIdle!=null)lines.push('TX ESP32 en reposo: '+(d.txIdle==1?'alto (OK)':d.txIdle==0?'BAJO (linea en corto/cargada)':'--'));if(d.hex)lines.push('Bytes crudos: '+d.hex);if(d.errBytes)lines.push('Bytes corruptos en RX: '+d.errBytes);if(d.cause)lines.push('Causa probable: '+d.cause);if(d.warning)lines.push('<b>'+d.warning+'</b>');if(d.urc)lines.push('Mensaje modem: '+d.urc);if(d.tail)lines.push('Ultimo RX: '+d.tail);lines.push(d.msg||(d.ok?'OK':'Error'));document.getElementById('lteResult').innerHTML=lines.join('<br>');}).catch(e=>{r.innerText='Error de comunicación con el dispositivo: '+e;}).finally(()=>setBusy(false));}
document.querySelectorAll('form').forEach(f=>{f.onsubmit=function(e){if(this.querySelector('input[name="resetRain"]')&&!confirm('¿Reiniciar el contador de lluvia a 0? Esta acción no se puede deshacer.')){e.preventDefault();return;}if(!this.querySelector('input[name="reset"]')){e.preventDefault();fetch('/get?'+new URLSearchParams(new FormData(this))).then(()=>location.href='/').catch(()=>location.href='/');}}});
</script>
</body>
</html>)rawliteral";

/*"*/

/* ====================== FUNCTION PROTOTYPES ======================== */
void notFound(AsyncWebServerRequest *request);
String readFile(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void saveSetting(String value, String fil);
String macA();
String processor(const String& var);
int countPendingRecords();
void deepSleepSystem();
void setting();
void setupServerRoutes();
String vBat();
float vBatRaw(int samples = 16);
void runRainCounter();
void readDHT22(float &temperature, float &humidity, bool &success, bool force = false);
float WindDirection();
float WindVelocity();
void modemPowerOn();
void rs485RailOff();
void mdiagReset();
String mdiagHexTxt();
void mdiagPump(uint32_t ms, bool live);
void mdiagTrackVsim(int16_t mv);
bool modemStabilize();
bool modemInitRetries();
void modemDiagProbeAT();
const char *modemDiagCause();
void drawModemDiagPage(const String &footer);
void modemPowerOff();
void modemApplyBand();
void modemRestart();
void modemLinesOff();
void callback(char *topic, byte *payload, unsigned int length);
void wifiReconnect();
void simReconnect();
bool checkMQTT();
void wakeup_reason();
void init_running();
void readConfig();
void initDHT22();
void initMT6701();
void initDS3231();
void initSD();
bool formatSD();
void logToSD(const String &line);
void loadTimeBase();
void setTimeBase(uint64_t epochSeconds);
uint64_t getEpochNow();
String formatTimestamp(uint64_t epochSeconds);
String formatTimestampISO8601(uint64_t epochSeconds);
void createDefaultConfigs();
String readSensors();
bool sendDataWiFi(String jsonData);
bool sendDataLTE(String jsonData);
uint64_t getRtcTimeSeconds();
void updateSensCache();
void waitForConfig(PubSubClient &mqttClient);
bool syncTimeFromInternetWiFi();
bool syncTimeFromInternetLTE();
void appendPendingRecord(const String &jsonLine);
bool flushPendingData(PubSubClient &mqttClient, const String &topicPub);
int countPendingRecords();
void armWakeSources(uint64_t sleepSeconds);
void sleepRemainingCycle();
void sensorsRailOn();
bool oledInit();
void oledPower(bool on);
void oledBegin(const String &header);
void oledLog(const String &line);
void setNetStatus(const char *msg);
void buttonsInit();
void configOledTask();
void runStatusViewer();
void modemRailOn();
uint64_t configuredSleepPeriod();
struct Rs485Data;
bool readRS485(Rs485Data &d, bool live);

/* ====================== HTML SCRIPTS ======================== */
void notFound(AsyncWebServerRequest *request) {
  Serial.print("NOT FOUND: ");
  Serial.println(request->url());
  request->send(404, "text/plain", "Not found");
}

/* ====================== FILE FUNCTIONS ======================== */
String readFile(fs::FS &fs, const char *path) {
  Serial.printf("[SPIFFS] Leyendo: %s\n", path);

  if (!SPIFFS.exists(path)) {
    Serial.printf("[SPIFFS] No existe: %s\n", path);
    return String("");
  }

  File file = fs.open(path, FILE_READ);
  if (!file) {
    Serial.printf("[SPIFFS] Error abriendo: %s\n", path);
    return String("");
  }

  if (file.isDirectory()) {
    file.close();
    return String("");
  }

  size_t size = file.size();
  if (size == 0) {
    file.close();
    return String("");
  }

  char* buf = (char*)malloc(size + 1);
  if (!buf) {
    file.close();
    return String("");
  }

  size_t bytesRead = file.readBytes(buf, size);
  buf[bytesRead] = '\0';
  file.close();

  String fileContent = String(buf);
  free(buf);
  fileContent.trim();

  return fileContent;
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, "w");
  if (!file) {
    Serial.printf("[SPIFFS] Error escribiendo: %s\n", path);
    return;
  }
  file.print(message);
  file.close();
}

void saveSetting(String value, String fil) {
  String orValue = readFile(SPIFFS, fil.c_str());
  if (orValue != value) {
    writeFile(SPIFFS, fil.c_str(), value.c_str());
  }
}

String macA() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

String processor(const String& var) {
  if (var == "ssidString") {
    return readFile(SPIFFS, "/ssidString.txt");
  } else if (var == "passString") {
    return readFile(SPIFFS, "/passString.txt");
  } else if (var == "server") {
    return readFile(SPIFFS, "/server.txt");
  } else if (var == "port") {
    return readFile(SPIFFS, "/port.txt");
  } else if (var == "topicPub") {
    return readFile(SPIFFS, "/topicPub.txt");
  } else if (var == "topicSub") {
    return readFile(SPIFFS, "/topicSub.txt");
  } else if (var == "BAT_NOW") {
    return vBat();
  } else if (var == "BAT_CAL") {
    return String(batCalFactor, 4);
  } else if (var == "apn") {
    String v = readFile(SPIFFS, "/apn.txt");
    return (v == "") ? "claro.pe" : v;
  } else if (var == "MAC") {
    return macA();
  } else if (var == "WIFI_SELECTED") {
    String connType = readFile(SPIFFS, "/connType.txt");
    return (connType == "WIFI" || connType == "") ? "selected" : "";
  } else if (var == "LTE_SELECTED") {
    String connType = readFile(SPIFFS, "/connType.txt");
    return (connType == "LTE") ? "selected" : "";
  } else if (var == "DATALOGER_SELECTED") {
    String connType = readFile(SPIFFS, "/connType.txt");
    return (connType == "DATALOGER") ? "selected" : "";
  } else if (var == "A_TH_CHECKED") {
    return readFile(SPIFFS, "/A_TH.txt") == "1" ? "checked" : "";
  } else if (var == "A_PR_CHECKED") {
    return readFile(SPIFFS, "/A_PR.txt") == "1" ? "checked" : "";
  } else if (var == "A_WS_CHECKED") {
    return readFile(SPIFFS, "/A_WS.txt") == "1" ? "checked" : "";
  } else if (var == "RS485_NONE") {
    return rs485Norm(readFile(SPIFFS, "/rs485.txt")) == "NONE" ? "selected" : "";
  } else if (var == "RS485_SOIL") {
    return readFile(SPIFFS, "/rs485.txt") == "SOIL" ? "selected" : "";
  } else if (var == "RS485_RAD") {
    return readFile(SPIFFS, "/rs485.txt") == "RAD" ? "selected" : "";
  } else if (var == "RS485_LEVEL") {
    return readFile(SPIFFS, "/rs485.txt") == "LEVEL" ? "selected" : "";
  } else if (var == "OLED_OP_CHECKED") {
    return readFile(SPIFFS, "/oledOp.txt") != "0" ? "checked" : "";
  } else if (var == "MT6701_SELECTED") {
    String dirSensor = readFile(SPIFFS, "/dirSensor.txt");
    return (dirSensor == "AS5600") ? "" : "selected";
  } else if (var == "AS5600_SELECTED") {
    String dirSensor = readFile(SPIFFS, "/dirSensor.txt");
    return (dirSensor == "AS5600") ? "selected" : "";
  } else if (var == "P120") {
    return readFile(SPIFFS, "/timesleep.txt") == "120" ? "selected" : "";
  } else if (var == "P300") {
    return readFile(SPIFFS, "/timesleep.txt") == "300" ? "selected" : "";
  } else if (var == "P900") {
    return readFile(SPIFFS, "/timesleep.txt") == "900" ? "selected" : "";
  } else if (var == "P1800") {
    return readFile(SPIFFS, "/timesleep.txt") == "1800" ? "selected" : "";
  } else if (var == "P3600") {
    return readFile(SPIFFS, "/timesleep.txt") == "3600" ? "selected" : "";
  } else if (var == "P7200") {
    return readFile(SPIFFS, "/timesleep.txt") == "7200" ? "selected" : "";
  } else if (var == "PERIOD_SEC") {
    String val = readFile(SPIFFS, "/timesleep.txt");
    return val != "" ? val : "300";
  } else if (var == "PERIOD_MIN") {
    String val = readFile(SPIFFS, "/timesleep.txt");
    int sec = val != "" ? val.toInt() : 300;
    return String(sec / 60);
  } else if (var == "SD_PRESENT") {
    return sdReady ? "Detectada" : "No detectada";
  } else if (var == "SD_TYPE") {
    return sdCardType;
  } else if (var == "SD_TOTAL") {
    return String((unsigned long)sdTotalMB);
  } else if (var == "SD_USED") {
    return String((unsigned long)sdUsedMB);
  } else if (var == "SD_FREE") {
    return String((unsigned long)sdFreeMB);
  } else if (var == "LTE_CARD_STYLE") {
    String connType = readFile(SPIFFS, "/connType.txt");
    return (connType == "LTE") ? "" : "display:none";
  } else if (var == "PENDING_COUNT") {
    return String(countPendingRecords());
  } else if (var == "TIME_BASE") {
    return timeBaseSet ? formatTimestamp(timeBaseEpoch) + " UTC (ancla)" : "Sin sincronizar";
  }
  return String();
}

/* ====================== SD CARD ======================== */
void initSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  // Reintento a 400 kHz: cableado marginal (protoboard/cables largos) suele
  // fallar a la velocidad por defecto (4 MHz) pero responde a baja velocidad.
  if (!SD.begin(SD_CS) && !SD.begin(SD_CS, SPI, 400000)) {
    Serial.println("[SD] Error al montar la tarjeta SD (probado a 4 MHz y 400 kHz)");
    sdReady = false;
    sdCardType = "N/A";
    sdTotalMB = sdUsedMB = sdFreeMB = 0;
    return;
  }

  sdReady = true;

  switch (SD.cardType()) {
    case CARD_MMC:  sdCardType = "MMC";  break;
    case CARD_SD:   sdCardType = "SDSC"; break;
    case CARD_SDHC: sdCardType = "SDHC"; break;
    default:        sdCardType = "Desconocido"; break;
  }

  sdTotalMB = SD.totalBytes() / (1024ULL * 1024ULL);
  sdUsedMB  = SD.usedBytes()  / (1024ULL * 1024ULL);
  sdFreeMB  = (sdTotalMB > sdUsedMB) ? (sdTotalMB - sdUsedMB) : 0;

  Serial.printf("[SD] Tarjeta SD lista: tipo=%s total=%llu MB usado=%llu MB libre=%llu MB\n",
                sdCardType.c_str(), sdTotalMB, sdUsedMB, sdFreeMB);
}

// Formatea la tarjeta SD como FAT (destructivo: borra todos los datos).
// Usa el driver oficial ESP-IDF esp_vfs_fat_sdspi_mount con
// format_if_mount_failed=true, ya que la librería Arduino SD.h no puede
// formatear: si SD.begin() nunca detecta la tarjeta, no hay filesystem
// FAT válido que montar y por tanto nada que la librería Arduino pueda leer.
bool formatSD() {
  Serial.println("[SD] Formateando tarjeta...");

  // Liberar el bus VSPI que usa la librería Arduino SD antes de remontar
  // con el driver ESP-IDF (ambos necesitan control exclusivo del CS/SPI).
  SD.end();
  SPI.end();

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  spi_bus_config_t busCfg = {
    .mosi_io_num = SD_MOSI,
    .miso_io_num = SD_MISO,
    .sclk_io_num = SD_SCK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
  };
  esp_err_t ret = spi_bus_initialize((spi_host_device_t)host.slot, &busCfg, SDSPI_DEFAULT_DMA);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    Serial.printf("[SD] Error iniciando bus SPI: %d\n", ret);
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    return false;
  }

  sdspi_device_config_t slotCfg = SDSPI_DEVICE_CONFIG_DEFAULT();
  slotCfg.gpio_cs = (gpio_num_t)SD_CS;
  slotCfg.host_id = (spi_host_device_t)host.slot;

  // esp_vfs_fat_sdspi_mount() con format_if_mount_failed=true SOLO formatea
  // cuando el montaje falla: si la tarjeta ya trae un FAT válido (el caso
  // normal, viene formateada de fábrica o de un ciclo anterior), el mount
  // simplemente reutiliza ese filesystem existente sin tocar nada — el botón
  // reportaba "Formateo OK" sin borrar un solo byte. Para forzar un
  // formateo real se inicializa la tarjeta a bajo nivel (sin pasar por
  // FatFs) y se borran los primeros sectores (boot sector FAT / tabla de
  // particiones): así el mount que sigue deja de encontrar un filesystem
  // válido, dispara la rama "mount falló" y esp_vfs_fat_sdspi_mount hace un
  // f_mkfs real.
  {
    sdspi_dev_handle_t wipeHandle;
    esp_err_t wipeRet = sdspi_host_init();
    if (wipeRet == ESP_OK || wipeRet == ESP_ERR_INVALID_STATE) {
      wipeRet = sdspi_host_init_device(&slotCfg, &wipeHandle);
    }
    if (wipeRet == ESP_OK) {
      sdmmc_host_t wipeHost = host;
      wipeHost.slot = wipeHandle;
      sdmmc_card_t rawCard;
      wipeRet = sdmmc_card_init(&wipeHost, &rawCard);
      if (wipeRet == ESP_OK) {
        uint8_t zero[512] = {0};
        for (int s = 0; s < 8 && wipeRet == ESP_OK; s++) {
          wipeRet = sdmmc_write_sectors(&rawCard, zero, s, 1);
        }
        if (wipeRet != ESP_OK) {
          Serial.printf("[SD] Advertencia: no se pudo borrar el sector de arranque: %s\n", esp_err_to_name(wipeRet));
        }
      } else {
        Serial.printf("[SD] Advertencia: no se pudo inicializar la tarjeta para borrado: %s\n", esp_err_to_name(wipeRet));
      }
      sdspi_host_remove_device(wipeHandle);
    } else {
      Serial.printf("[SD] Advertencia: no se pudo iniciar el host SPI para borrado: %s\n", esp_err_to_name(wipeRet));
    }
    sdspi_host_deinit();
  }

  esp_vfs_fat_sdmmc_mount_config_t mountCfg = {
    .format_if_mount_failed = true,
    .max_files = 2,
    .allocation_unit_size = 16 * 1024,
  };

  sdmmc_card_t *card = nullptr;
  ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slotCfg, &mountCfg, &card);

  bool ok = (ret == ESP_OK);
  if (ok) {
    Serial.println("[SD] Formateo OK");
    esp_vfs_fat_sdcard_unmount("/sdcard", card);
  } else {
    Serial.printf("[SD] Formateo falló: %s\n", esp_err_to_name(ret));
  }

  spi_bus_free((spi_host_device_t)host.slot);

  // Volver a montar con la librería Arduino SD para uso normal.
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  initSD();

  return ok;
}

// Respaldo en la tarjeta microSD física, en formato CSV con marca de tiempo.
// jsonLine es el mismo payload publicado por MQTT (ver readSensors()).
void logToSD(const String &jsonLine) {
  if (!sdReady) {
    Serial.println("[SD] Tarjeta no lista, se pierde este registro");
    return;
  }

  bool isNew = !SD.exists(SD_LOG_PATH);
  File f = SD.open(SD_LOG_PATH, FILE_APPEND);

  // Si el open falla, la tarjeta puede haber quedado en un estado
  // inconsistente (cableado marginal, ver initSD()) tras el ciclo anterior:
  // se remonta una vez desde cero y se reintenta antes de darse por vencido,
  // en vez de perder la lectura de este ciclo en silencio.
  if (!f) {
    Serial.println("[SD] Error abriendo " SD_LOG_PATH ", remontando y reintentando...");
    initSD();
    if (!sdReady) {
      Serial.println("[SD] Remontaje falló, se pierde este registro");
      return;
    }
    isNew = !SD.exists(SD_LOG_PATH);
    f = SD.open(SD_LOG_PATH, FILE_APPEND);
    if (!f) {
      Serial.println("[SD] Error abriendo " SD_LOG_PATH " tras reintento, se pierde este registro");
      return;
    }
  }

  if (isNew) {
    f.println("timestamp,epoch_s,mac,conn,T,H,WV,WD,RA,vB,ST,SH,EC,PH,RAD,LV");
  }

  // 256 se quedaba corto y descartaba el registro entero (DeserializationError
  // NoMemory): a diferencia de DynamicJsonDocument docOut(384) en
  // readSensors() (que solo tiene que ENSAMBLAR el JSON), este deserializeJson()
  // parte de un String inmutable y por eso duplica en el pool tanto las claves
  // como cada valor string (mac, timestamp ISO, "DATALOGER"/"error", etc.) —
  // ese payload real ronda ~300 bytes con los 10 campos actuales. 512 deja
  // margen para nombres de conexión o timestamps un poco más largos sin
  // volver a quedarse sin memoria.
  // 768: los 5 campos RS485 suman ~60 bytes de texto y 5 nodos más.
  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, jsonLine);
  if (err) {
    Serial.println("[SD] Error parseando JSON para CSV: " + String(err.c_str()));
    f.close();
    return;
  }

  uint64_t epochNow = getEpochNow();
  String row = formatTimestamp(epochNow);
  row += ","; row += String((unsigned long)epochNow);
  row += ","; row += doc["mac"].as<String>();
  row += ","; row += doc["conn"].as<String>();
  row += ","; row += doc["T"].as<String>();
  row += ","; row += doc["H"].as<String>();
  row += ","; row += doc["WV"].as<String>();
  row += ","; row += doc["WD"].as<String>();
  row += ","; row += doc["RA"].as<String>();
  row += ","; row += doc["vB"].as<String>();
  row += ","; row += doc["ST"].as<String>();
  row += ","; row += doc["SH"].as<String>();
  row += ","; row += doc["EC"].as<String>();
  row += ","; row += doc["PH"].as<String>();
  row += ","; row += doc["RAD"].as<String>();
  row += ","; row += doc["LV"].as<String>();

  // println() no reporta fallos de escritura de otra forma (disco lleno,
  // tarjeta caída a media escritura): comparar los bytes devueltos contra el
  // tamaño esperado de la fila es la única señal disponible de que el dato
  // realmente quedó grabado.
  size_t expected = row.length() + 1;  // +1 por el salto de línea de println()
  size_t written = f.println(row);
  f.close();

  if (written < expected) {
    Serial.printf("[SD] ADVERTENCIA: escritura incompleta (%u/%u bytes) — tarjeta llena o con error, revisar espacio libre\n",
                  (unsigned)written, (unsigned)expected);
  }
}

/* ================ MARCA DE TIEMPO (hora real) ================ */
void loadTimeBase() {
  timeBaseEpoch = (uint64_t)readFile(SPIFFS, "/timeBaseEpoch.txt").toInt();
  timeBaseRtc = (uint64_t)readFile(SPIFFS, "/timeBaseRtc.txt").toInt();
  timeBaseSet = timeBaseEpoch > 0;
  if (timeBaseSet) {
    Serial.printf("[TIME] Ancla cargada: epoch=%llu rtc=%llu\n", timeBaseEpoch, timeBaseRtc);
  } else {
    Serial.println("[TIME] Sin ancla de hora real guardada (aún no se conectó ningún dispositivo al AP)");
  }
}

// Ancla la hora real contra el RTC interno. Se llama desde /settime, con el
// epoch (segundos Unix, UTC) que envía el navegador/teléfono conectado al AP.
// También reajusta el DS3231 si está disponible, para que el reloj con
// batería de respaldo se mantenga alineado con la fuente de hora más
// reciente (NTP, hora de red LTE o el navegador conectado al AP).
void setTimeBase(uint64_t epochSeconds) {
  timeBaseEpoch = epochSeconds;
  timeBaseRtc = getRtcTimeSeconds();
  timeBaseSet = true;
  writeFile(SPIFFS, "/timeBaseEpoch.txt", String((unsigned long)timeBaseEpoch).c_str());
  writeFile(SPIFFS, "/timeBaseRtc.txt", String((unsigned long)timeBaseRtc).c_str());
  Serial.printf("[TIME] Ancla actualizada: epoch=%llu rtc=%llu\n", timeBaseEpoch, timeBaseRtc);

  if (ds3231Ready) {
    ds3231.adjust(DateTime((uint32_t)epochSeconds));
    Serial.println("[RTC] DS3231 realineado con la nueva ancla de hora");
  }
}

// Inicializa el DS3231 sobre el bus I2C (ya debe estar en marcha —
// Wire.begin()). La ajusta a la hora de compilación del firmware — es decir,
// la hora del PC donde se compiló/subió el sketch — en dos casos:
//   1) Perdió la hora (batería de respaldo nueva, agotada o ausente): se
//      ajusta en cada boot mientras eso siga así.
//   2) Es la primera vez que arranca ESTE binario (marcador en SPIFFS
//      ausente): fuerza un resync una única vez por firmware subido, aunque
//      el DS3231 ya tuviera hora válida de una batería previa — así una
//      subida nueva siempre corrige el reloj a la hora actual del PC en vez
//      de arrastrar la hora que se le haya puesto antes.
// Fuera de esos dos casos no se vuelve a tocar: el DS3231 sigue su marcha
// con su propio oscilador/batería hasta la próxima subida o pérdida de hora.
void initDS3231() {
  ds3231Ready = ds3231.begin();
  if (!ds3231Ready) {
    Serial.println("[RTC] DS3231 no detectado en el bus I2C");
    return;
  }

  bool firstBootOfThisBinary = !SPIFFS.exists("/ds3231Synced.txt");
  if (ds3231.lostPower() || firstBootOfThisBinary) {
    DateTime compileTime(F(__DATE__), F(__TIME__));
    Serial.printf("[RTC] Ajustando DS3231 a la hora del PC (compilacion): %s (motivo: %s)\n",
                  compileTime.timestamp().c_str(),
                  ds3231.lostPower() ? "sin hora valida" : "primer arranque de este firmware");
    ds3231.adjust(compileTime);
    writeFile(SPIFFS, "/ds3231Synced.txt", "1");
  }
  Serial.printf("[RTC] DS3231 listo, hora actual: %s\n", ds3231.now().timestamp().c_str());
}

// Hora real "ahora mismo". Fuente primaria: el DS3231 (hardware, con batería
// de respaldo). Si no responde, cae al esquema de ancla por software (ancla +
// segundos de RTC-slow-clock transcurridos desde que se guardó). Si tampoco
// hay ancla, devuelve 0 (el llamador debe usar formatTimestamp/logToSD con
// ese caso en mente).
uint64_t getEpochNow() {
  if (ds3231Ready) {
    return ds3231.now().unixtime();
  }
  if (!timeBaseSet) return 0;
  uint64_t nowRtc = getRtcTimeSeconds();
  uint64_t elapsed = (nowRtc > timeBaseRtc) ? (nowRtc - timeBaseRtc) : 0;
  return timeBaseEpoch + elapsed;
}

String formatTimestamp(uint64_t epochSeconds) {
  if (epochSeconds == 0) return "NA";
  time_t t = (time_t)epochSeconds;
  struct tm tmStruct;
  gmtime_r(&t, &tmStruct);
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           tmStruct.tm_year + 1900, tmStruct.tm_mon + 1, tmStruct.tm_mday,
           tmStruct.tm_hour, tmStruct.tm_min, tmStruct.tm_sec);
  return String(buf);
}

// Igual que formatTimestamp() pero en ISO 8601 (YYYY-MM-DDTHH:MM:SS, UTC),
// formato esperado por el backend (Postgres/Django) para el campo "timestamp".
String formatTimestampISO8601(uint64_t epochSeconds) {
  if (epochSeconds == 0) return "NA";
  time_t t = (time_t)epochSeconds;
  struct tm tmStruct;
  gmtime_r(&t, &tmStruct);
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
           tmStruct.tm_year + 1900, tmStruct.tm_mon + 1, tmStruct.tm_mday,
           tmStruct.tm_hour, tmStruct.tm_min, tmStruct.tm_sec);
  return String(buf);
}

/* ============ SINCRONIZACIÓN DE HORA POR INTERNET (NTP / red LTE) ============ */
// Se llama únicamente tras un publish MQTT exitoso (WiFi ya asociada o GPRS ya
// conectado), para no gastar tiempo/energía intentándolo si no hay red.
// Actualiza el ancla de hora real (setTimeBase) que ya usa formatTimestamp/SD.

bool syncTimeFromInternetWiFi() {
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  struct tm tmStruct;
  if (!getLocalTime(&tmStruct, 5000)) {
    Serial.println("[TIME] NTP (WiFi) sin respuesta, se mantiene ancla anterior");
    return false;
  }
  time_t epoch = mktime(&tmStruct) ; // mktime asume tm en tiempo local; configTime(0,0,...) => UTC
  setTimeBase((uint64_t)epoch);
  Serial.println("[TIME] Hora sincronizada por NTP (WiFi)");
  return true;
}

bool syncTimeFromInternetLTE() {
  int year, month, day, hour, minute, second;
  float timezoneOffset;
  if (!modem.getNetworkTime(&year, &month, &day, &hour, &minute, &second, &timezoneOffset)) {
    Serial.println("[TIME] Hora de red (LTE) no disponible, se mantiene ancla anterior");
    return false;
  }
  struct tm tmStruct = {};
  tmStruct.tm_year = year - 1900;
  tmStruct.tm_mon  = month - 1;
  tmStruct.tm_mday = day;
  tmStruct.tm_hour = hour;
  tmStruct.tm_min  = minute;
  tmStruct.tm_sec  = second;
  // getNetworkTime() devuelve hora LOCAL de red + offset horario (en horas).
  // mktime() interpreta tmStruct como tiempo local, pero el ESP32 no tiene TZ
  // configurada (por defecto UTC), así que su resultado ya equivale a tratar
  // los campos como si fueran UTC — igual que timegm(), que no está disponible
  // en este toolchain. Solo falta restar el offset horario para obtener UTC real.
  time_t localAsUtc = mktime(&tmStruct);
  time_t epoch = localAsUtc - (time_t)(timezoneOffset * 3600.0f);
  setTimeBase((uint64_t)epoch);
  Serial.println("[TIME] Hora sincronizada por red LTE");
  return true;
}

/* ====== ALMACENAMIENTO Y REENVÍO (SPIFFS, JSON Lines) ====== */
// Si el envío de una lectura falla (sin red o publish rechazado), el JSON
// completo (ya con su campo "timestamp") se guarda como una línea más en
// este archivo. En el próximo ciclo con conexión exitosa, flushPendingData()
// reenvía todo lo pendiente, en orden cronológico, antes del dato actual.
#define PENDING_LOG_PATH "/pending.jsonl"
#define PENDING_LOG_TMP  "/pending.tmp"

void appendPendingRecord(const String &jsonLine) {
  File f = SPIFFS.open(PENDING_LOG_PATH, FILE_APPEND);
  if (!f) {
    Serial.println("[PENDING] Error abriendo " PENDING_LOG_PATH " para escritura");
    return;
  }
  f.println(jsonLine);
  f.close();
  Serial.println("[PENDING] Lectura guardada para reenvío posterior");
}

// Reenvía, en orden, los registros pendientes por el mismo cliente MQTT ya
// conectado. Lectura/escritura secuencial línea por línea (sin cargar el
// archivo completo en RAM). Si una publicación falla, se detiene ahí mismo
// y conserva en el archivo esa línea y todas las siguientes para el próximo
// intento. Devuelve true si no quedó ningún pendiente (o no había ninguno).
bool flushPendingData(PubSubClient &mqttClient, const String &topicPub) {
  if (!SPIFFS.exists(PENDING_LOG_PATH)) return true;

  File in = SPIFFS.open(PENDING_LOG_PATH, FILE_READ);
  if (!in) {
    Serial.println("[PENDING] Error abriendo " PENDING_LOG_PATH " para lectura");
    return false;
  }

  SPIFFS.remove(PENDING_LOG_TMP);
  File out;
  bool outOpen = false;
  bool stoppedEarly = false;
  int sentCount = 0, keptCount = 0;

  while (in.available()) {
    String line = in.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (!stoppedEarly && mqttClient.connected() &&
        mqttClient.publish(topicPub.c_str(), line.c_str())) {
      sentCount++;
      mqttClient.loop();
      continue;
    }

    // No se pudo enviar esta línea: a partir de aquí se conserva todo tal cual,
    // sin volver a intentar publicar, para no reordenar los registros.
    stoppedEarly = true;
    if (!outOpen) {
      out = SPIFFS.open(PENDING_LOG_TMP, FILE_WRITE);
      outOpen = (bool)out;
    }
    if (outOpen) {
      out.println(line);
      keptCount++;
    }
  }
  in.close();
  if (outOpen) out.close();

  if (stoppedEarly && keptCount > 0) {
    SPIFFS.remove(PENDING_LOG_PATH);
    SPIFFS.rename(PENDING_LOG_TMP, PENDING_LOG_PATH);
    Serial.printf("[PENDING] %d reenviados, %d conservados para el próximo intento\n", sentCount, keptCount);
    return false;
  }

  // Todo se envió correctamente (o no quedó nada por conservar): limpiar.
  SPIFFS.remove(PENDING_LOG_PATH);
  SPIFFS.remove(PENDING_LOG_TMP);
  if (sentCount > 0) {
    Serial.printf("[PENDING] %d registros pendientes reenviados correctamente\n", sentCount);
  }
  return true;
}

// Período de sleep configurado (/timesleep.txt, mínimo 60 s) multiplicado
// por el factor de batería baja vigente.
uint64_t configuredSleepPeriod() {
  uint64_t period = readFile(SPIFFS, "/timesleep.txt").toInt();
  if (period < 60) period = 300;
  return period * (sleepFactor ? sleepFactor : 1);
}

// Número de lecturas en cola de reenvío (líneas no vacías de /pending.jsonl).
int countPendingRecords() {
  if (!SPIFFS.exists(PENDING_LOG_PATH)) return 0;
  File f = SPIFFS.open(PENDING_LOG_PATH, FILE_READ);
  if (!f) return 0;
  int count = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) count++;
  }
  f.close();
  return count;
}

/* ========================= DEEP SLEEP ========================= */

// Fuentes de wakeup comunes a todos los caminos que duermen.
void armWakeSources(uint64_t sleepSeconds) {
  esp_sleep_enable_timer_wakeup(sleepSeconds * uS_TO_S_FACTOR);

  // habilita solo si esta activado el sensor de lluvia
  if (A_PR) {
    esp_sleep_enable_ext1_wakeup(RAINCOUNT_MASK, ESP_EXT1_WAKEUP_ALL_LOW);
  }

  // Botón "ok" (GPIO35, activo LOW): despierta para consultar el estado en
  // el OLED (runStatusViewer()). Solo se arma si está suelto: atascado en
  // LOW (o sin su pull-up externo) despertaría en bucle sin parar.
  rtc_gpio_deinit((gpio_num_t)BTN_OK_PIN);
  pinMode(BTN_OK_PIN, INPUT);
  if (digitalRead(BTN_OK_PIN) == HIGH) {
    esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_OK_PIN, 0);
  } else {
    Serial.println("[SLEEP] Botón ok presionado/atascado: wake por botón DESHABILITADO");
  }
}

void deepSleepSystem() {
  Serial.println("\n[SLEEP] Preparando deep sleep...");

  // Apagar el panel antes de cortarle v_sensor
  oledPower(false);

  // ===== APAGAR SENSORES (MOSFET canal P, HIGH=OFF) =====
  gpio_hold_dis((gpio_num_t)SENS_PWR_PIN);
  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, HIGH);
  gpio_hold_en((gpio_num_t)SENS_PWR_PIN);

  // ===== APAGAR MODEM (sim_control, NPN+MOSFET: LOW=OFF) =====
  gpio_hold_dis((gpio_num_t)MODEM_PWR_PIN);
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, MODEM_PWR_OFF);
  gpio_hold_en((gpio_num_t)MODEM_PWR_PIN);

  // ===== APAGAR 12V RS485 (GPIO14) =====
  rs485RailOff();

  // ===== RX2/TX2 a LOW (no flotando): evita retroalimentar el SIM800 por
  // los pines UART mientras está sin alimentación. Incondicional: en modo
  // WiFi el modem nunca se enciende, así que sin esto estos pines
  // quedarían en su estado flotante por defecto durante todo el sleep.
  modemLinesOff();

  // Batería crítica: alargar el período para que la celda se recupere / el
  // panel la cargue antes del próximo ciclo. Se vuelve a evaluar cada ciclo.
  if (cycleBatV >= BAT_VALID_MIN_V) {
    sleepFactor = (cycleBatV < BAT_CRITICAL_V) ? LOW_BAT_SLEEP_FACTOR : 1;
    if (sleepFactor > 1) {
      Serial.printf("[POWER] Batería crítica (%.2f V): período × %d\n", cycleBatV, sleepFactor);
    }
  }

  // Leer tiempo de sleep
  TIME_TO_SLEEP = configuredSleepPeriod();

  // Compensar el tiempo despierto (encender modem, leer sensores, publicar
  // MQTT, esperar config) para que el período real entre wakeups por timer
  // sea TIME_TO_SLEEP y no TIME_TO_SLEEP + tiempo despierto (causaba un
  // desfase acumulado, p.ej. ~7 s cada 5 min). cycle_start_rtc marca el
  // inicio real del ciclo (bootRtcSeconds), no el instante justo antes de
  // dormir, para que runRainCounter() también calcule bien el restante si
  // el sleep siguiente se corta por un pulso de lluvia.
  uint64_t nowRtc = getRtcTimeSeconds();
  uint64_t awakeSeconds = (nowRtc > bootRtcSeconds) ? (nowRtc - bootRtcSeconds) : 0;

  cycle_start_rtc = bootRtcSeconds;
  cycle_active = true;

  uint64_t sleepSeconds = (awakeSeconds >= TIME_TO_SLEEP) ? 10 : (TIME_TO_SLEEP - awakeSeconds);
  if (sleepSeconds < 10) sleepSeconds = 10;

  Serial.printf("[SLEEP] Período: %d seg (%d min) - Despierto: %llu s - Durmiendo: %llu s\n",
                TIME_TO_SLEEP, TIME_TO_SLEEP / 60, awakeSeconds, sleepSeconds);

  // ===== HABILITAR HOLD DURANTE DEEP SLEEP =====
  gpio_deep_sleep_hold_en();

  // Configurar wake ups (timer + lluvia si A_PR + botón "ok")
  armWakeSources(sleepSeconds);
  Serial.printf("[SLEEP] Wake por lluvia: %s\n", A_PR ? "HABILITADO" : "DESHABILITADO");

  Serial.println("[SLEEP] Entrando en deep sleep...\n");
  Serial.flush();

  esp_deep_sleep_start();
}

/* ====================== SETUP SERVER ROUTES ======================== */
void setupServerRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("[HTTP] Root requested");
    request->send_P(200, "text/html", index_html, processor);
  });

  // Handler no bloqueante: sirve valores cacheados actualizados desde loop()
  server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<256> doc;
    doc["temp"] = sensCache.temp;
    doc["hum"]  = sensCache.hum;
    doc["rain"] = sensCache.rain;
    doc["rainStatus"] = sensCache.rainStatus;
    doc["rainCount"] = sensCache.rainCount;
    doc["wvel"] = sensCache.wvel;
    doc["wdir"] = sensCache.wdir;
    doc["bat"]  = sensCache.bat;
    doc["rs485"] = sensCache.rs485;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Diagnóstico LTE bajo demanda (bloqueante, botón manual): solo tiene
  // sentido si connType=LTE, ya que enciende el modem SIM800 vía MOSFET.
  server.on("/ltetest", HTTP_GET, [](AsyncWebServerRequest *request) {
    // En modo config init_running() entra a setting() antes de cargar
    // connectionType: leerlo siempre del archivo, que es la fuente real.
    connectionType = readFile(SPIFFS, "/connType.txt");
    if (connectionType != "LTE") {
      request->send(400, "application/json", "{\"ok\":false,\"msg\":\"Modo LTE no seleccionado\"}");
      return;
    }

    Serial.println("[LTE TEST] Iniciando diagnóstico...");
    StaticJsonDocument<768> doc;
    doc["ok"] = false;

    // Este handler corre en el hilo "async_tcp" (el mismo que atiende TODAS
    // las peticiones HTTP), que tiene su propio watchdog (~5 s por defecto).
    // modem.waitForNetwork()/gprsConnect() bloquean muchos segundos sin
    // devolver el control, así que sin esto el watchdog aborta y reinicia
    // el equipo a medias (ver el mismo problema documentado en /lognow más
    // abajo). Se desuscribe esta tarea del watchdog mientras dura el
    // diagnóstico bloqueante y se vuelve a suscribir antes de responder.
    esp_task_wdt_delete(NULL);

    // El OLED lo dibuja configOledTask() en el loop: este handler corre en
    // otro hilo, así que se silencian los oledLog() del modem (I2C no es
    // reentrante). El diagnóstico queda en mdiag (vista MODEM) y en el JSON.
    bool oledWas = oledReady;
    oledReady = false;
    mdiagReset();
    lastModemMv = -1;
    // Se agrega al JSON en cada respuesta (macro para no repetirlo).
    #define LTE_TEST_DIAG() do { \
      doc["vBat0"] = String(mdiag.vBat0, 2); doc["vBatMin"] = String(mdiag.vBatMin, 2); \
      doc["rxLevel"] = mdiag.rxLevel; doc["rxBytes"] = mdiag.rxBytes; \
      doc["atBaud"] = mdiag.atBaud ? String(mdiag.atBaud) : String("no"); \
      doc["rxPre"] = mdiag.rxPre; doc["rxOn"] = mdiag.rxOn; doc["txIdle"] = mdiag.txIdle; doc["hex"] = mdiagHexTxt(); \
      doc["cause"] = modemDiagCause(); \
      doc["urc"] = mdiag.urc; doc["tail"] = mdiag.tail; doc["vMod"] = mdiag.vSimMax > 0 ? mdiag.vSimMax : lastModemMv; doc["errBytes"] = mdiag.badBytes; \
      oledReady = oledWas; } while (0)

    modemPowerOn();

    // Misma secuencia que el ciclo: estabilizar → init (reintentos) → reinicio
    bool linkOk = modemStabilize();
    if (!linkOk || !modemInitRetries()) {
      modemRestart();
      linkOk = modemStabilize();
      if (!linkOk || !modemInitRetries()) {
        modemDiagProbeAT();
        doc["msg"] = String("El modem SIM800 no responde: ") + modemDiagCause();
        strlcpy(mdiag.step, "INIT", sizeof(mdiag.step));
        modemPowerOff();
        LTE_TEST_DIAG();
        esp_task_wdt_add(NULL);
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
        return;
      }
    }

    doc["modemInfo"] = modem.getModemInfo();
    modemApplyBand();
    lastModemMv = modem.getBattVoltage();
    mdiagTrackVsim(lastModemMv);
    if (lastModemMv >= SIM800_VMAX_MV)
      doc["warning"] = "SOBREVOLTAJE en el SIM800: " + String(lastModemMv) + " mV (max 4400). Se apagara solo (OVER-VOLTAGE POWER DOWN).";

    int16_t csq = modem.getSignalQuality();
    doc["signal"] = csq;
    doc["simReady"] = modem.getSimStatus() == SIM_READY;

    bool netOk = modem.waitForNetwork(15000L);
    doc["networkRegistered"] = netOk;

    if (!netOk) {
      doc["msg"] = "Sin registro en la red celular (revisar SIM/cobertura/antena)";
      strlcpy(mdiag.step, "RED", sizeof(mdiag.step));
      modemPowerOff();
      LTE_TEST_DIAG();
      esp_task_wdt_add(NULL);
      String response;
      serializeJson(doc, response);
      request->send(200, "application/json", response);
      return;
    }

    doc["operator"] = modem.getOperator();

    bool gprsOk = modem.gprsConnect(RUNapn.c_str());
    doc["gprsConnected"] = gprsOk;

    if (gprsOk) {
      doc["ip"] = modem.localIP().toString();
      doc["ok"] = true;
      doc["msg"] = "Conexión a internet OK";
      modem.gprsDisconnect();
    } else {
      doc["msg"] = "Registrado en red, pero falló GPRS (revisar APN: " + RUNapn + ")";
      strlcpy(mdiag.step, "GPRS", sizeof(mdiag.step));
    }

    modemPowerOff();
    LTE_TEST_DIAG();
    #undef LTE_TEST_DIAG
    esp_task_wdt_add(NULL);

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
    Serial.println("[LTE TEST] Fin diagnóstico");
  });

  // Captura manual bajo demanda (bloqueante, botón manual): lee sensores y
  // agrega una fila a la SD tal cual lo haría un ciclo real de operación,
  // sin esperar a que venza el período de sleep. En modo configuración
  // init_running()/logToSD() nunca se ejecutan solos (el equipo no duerme
  // mientras el portal AP está activo), así que sin este botón el archivo
  // de la SD no gana filas nuevas mientras se está probando desde el portal.
  server.on("/lognow", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("\n[LOG] Captura manual solicitada desde el portal AP");

    // readSensors() bloquea 8-20+ s (2.5 s de arranque del sensor + hasta
    // 10 s de reintentos del DHT + 3 s de muestreo de viento) corriendo
    // SOBRE el hilo "async_tcp" (el mismo que atiende esta petición HTTP).
    // Ese hilo tiene su propio watchdog (~5 s por defecto): sin esto el
    // watchdog abortaba el firmware A MITAD de readSensors() (se veía en el
    // log justo tras leer /ssidRainCounter.txt, el paso previo al de
    // batería) — el equipo se reiniciaba antes de llegar siquiera a
    // logToSD(), por eso datalog.csv nunca ganaba filas nuevas desde este
    // botón. Se desuscribe esta tarea del watchdog mientras dura la
    // captura y se vuelve a suscribir apenas termina.
    esp_task_wdt_delete(NULL);
    String sensorData = readSensors();
    logToSD(sensorData);
    esp_task_wdt_add(NULL);

    StaticJsonDocument<256> doc;
    doc["ok"] = sdReady;
    doc["msg"] = sdReady ? "Lectura guardada en la SD" : "Tarjeta SD no disponible";
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Formateo de SD bajo demanda (bloqueante, botón manual, destructivo).
  server.on("/formatsd", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Mismo problema de watchdog que /lognow y /ltetest: formatSD() (borrado
    // de sectores + montaje/f_mkfs) corre en el hilo "async_tcp" que atiende
    // esta petición.
    esp_task_wdt_delete(NULL);
    bool ok = formatSD();
    esp_task_wdt_add(NULL);
    StaticJsonDocument<256> doc;
    doc["ok"] = ok;
    doc["msg"] = ok ? "Formateo completado, tarjeta lista" : "Formateo falló (revisar cableado/tarjeta)";
    doc["sdPresent"] = sdReady;
    doc["sdType"] = sdCardType;
    doc["sdTotal"] = (unsigned long)sdTotalMB;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Lista los archivos en la raíz de la SD (nombre + tamaño) para que el
  // portal pueda ofrecer un selector de descarga en vez de solo el CSV
  // principal. No recursivo: esta firmware nunca crea subcarpetas en la SD.
  server.on("/sdlist", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(4096);
    JsonArray files = doc.createNestedArray("files");

    if (sdReady) {
      File root = SD.open("/");
      if (root && root.isDirectory()) {
        File entry = root.openNextFile();
        while (entry) {
          if (!entry.isDirectory()) {
            String name = String(entry.name());
            if (name.startsWith("/")) name = name.substring(1);
            JsonObject f = files.createNestedObject();
            f["name"] = name;
            f["size"] = (unsigned long)entry.size();
          }
          entry = root.openNextFile();
        }
        root.close();
      }
    }

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Descarga de un archivo de la tarjeta microSD física. Por compatibilidad,
  // sin parámetro ?file= descarga el CSV principal (SD_LOG_PATH); con
  // ?file=nombre descarga cualquier otro archivo listado por /sdlist.
  // NOTA: usar request->send(fs,...,download=true) en vez de beginResponse()+
  // addHeader("Content-Disposition",...) — ese patrón agregaba un SEGUNDO
  // header Content-Disposition encima del "inline" que ya pone la librería
  // cuando download=false, y los navegadores (Chrome:
  // ERR_RESPONSE_HEADERS_MULTIPLE_CONTENT_DISPOSITION) rechazan la respuesta
  // completa al ver dos headers Content-Disposition — por eso no descargaba
  // nada. El archivo se sirve en streaming (chunked) directo desde la SD, así
  // que el tamaño (varios MB) no es un problema.
  server.on("/downloadsd", HTTP_GET, [](AsyncWebServerRequest *request) {
    String path = SD_LOG_PATH;

    if (request->hasParam("file")) {
      String reqName = request->getParam("file")->value();
      // Solo nombres simples de archivo en la raíz: sin '/' interno ni '..',
      // así no hay forma de escapar de la SD con un path arbitrario en el
      // parámetro — esta firmware tampoco crea subcarpetas, así que una
      // ruta con '/' nunca es un archivo legítimo de todos modos.
      if (reqName.length() == 0 || reqName.indexOf('/') != -1 || reqName.indexOf("..") != -1) {
        request->send(400, "text/plain", "Nombre de archivo inválido");
        return;
      }
      path = "/" + reqName;
    }

    if (!sdReady || !SD.exists(path)) {
      request->send(404, "text/plain", "Archivo no encontrado en la SD");
      return;
    }

    File f = SD.open(path, FILE_READ);
    Serial.printf("[SD] Descargando %s (%lu bytes)\n", path.c_str(), (unsigned long)f.size());
    f.close();

    String contentType = path.endsWith(".csv") ? "text/csv" : "application/octet-stream";
    request->send(SD, path, contentType, true);
  });

  // Últimas filas del respaldo en la SD en texto plano (CSV) para pintar una
  // tabla en el portal AP, sin cargar el archivo completo en memoria: se
  // recorre línea a línea y solo se retiene un buffer circular de las
  // últimas MAX_ROWS.
  server.on("/viewdata", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!sdReady || !SD.exists(SD_LOG_PATH)) {
      request->send(200, "text/plain", "");
      return;
    }

    const int MAX_ROWS = 50;
    File f = SD.open(SD_LOG_PATH, FILE_READ);
    String header;
    String ring[MAX_ROWS];
    int count = 0;
    bool first = true;

    while (f.available()) {
      String line = f.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;
      if (first) {
        header = line;
        first = false;
        continue;
      }
      ring[count % MAX_ROWS] = line;
      count++;
    }
    f.close();

    String out = header + "\n";
    int shown = (count < MAX_ROWS) ? count : MAX_ROWS;
    int startCount = count - shown;
    for (int i = 0; i < shown; i++) {
      out += ring[(startCount + i) % MAX_ROWS] + "\n";
    }

    request->send(200, "text/plain", out);
  });

  // Datos pendientes de envío (/pending.jsonl en SPIFFS): se envía el
  // archivo tal cual, en streaming (puede ser grande; no se carga en RAM).
  // El JS del portal parsea cada línea JSON y arma la tabla.
  server.on("/pending", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!SPIFFS.exists(PENDING_LOG_PATH)) {
      request->send(200, "text/plain", "");
      return;
    }
    request->send(SPIFFS, PENDING_LOG_PATH, "text/plain");
  });

  // Borra la cola de pendientes (no toca la SD, que conserva su respaldo).
  server.on("/pendingclear", HTTP_GET, [](AsyncWebServerRequest *request) {
    int before = countPendingRecords();
    SPIFFS.remove(PENDING_LOG_PATH);
    SPIFFS.remove(PENDING_LOG_TMP);
    int after = countPendingRecords();
    Serial.printf("[PENDING] Cola borrada desde el portal (%d registros)\n", before);

    StaticJsonDocument<128> doc;
    doc["ok"] = after == 0;
    doc["count"] = after;
    doc["msg"] = after == 0 ? String(before) + " registro(s) pendiente(s) borrado(s)"
                            : String("No se pudo borrar la cola");
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Recibe la hora actual (epoch, segundos Unix UTC) del dispositivo
  // conectado al AP, para anclar la marca de tiempo real de las mediciones
  // (ver setTimeBase()). Lo llama el JS del portal automáticamente al cargar.
  server.on("/settime", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<128> doc;
    if (!request->hasParam("epoch")) {
      doc["ok"] = false;
      doc["msg"] = "Falta parámetro epoch";
    } else {
      uint64_t epoch = strtoull(request->getParam("epoch")->value().c_str(), nullptr, 10);
      if (epoch < 1000000000ULL) {  // sanity check: antes de 2001 => inválido
        doc["ok"] = false;
        doc["msg"] = "epoch inválido";
      } else {
        setTimeBase(epoch);
        doc["ok"] = true;
        doc["msg"] = "Hora sincronizada";
        doc["timestamp"] = formatTimestamp(epoch);
      }
    }
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("[HTTP] GET requested");
    String msg = "OK";

    if (request->hasParam(PARAM_CONN_TYPE)) {
      String val = request->getParam(PARAM_CONN_TYPE)->value();
      writeFile(SPIFFS, "/connType.txt", val.c_str());
      connectionType = val;  // /ltetest y demás handlers usan la global
      Serial.println("[SAVE] connType: " + val);
    }
    else if (request->hasParam(PARAM_SSID)) {
      String val = request->getParam(PARAM_SSID)->value();
      writeFile(SPIFFS, "/ssidString.txt", val.c_str());
      Serial.println("[SAVE] SSID: " + val);
    }
    else if (request->hasParam(PARAM_PASS)) {
      String val = request->getParam(PARAM_PASS)->value();
      writeFile(SPIFFS, "/passString.txt", val.c_str());
      Serial.println("[SAVE] Password saved");
    }
    else if (request->hasParam(PARAM_SERVER)) {
      String val = request->getParam(PARAM_SERVER)->value();
      writeFile(SPIFFS, "/server.txt", val.c_str());
      Serial.println("[SAVE] Server: " + val);
    }
    else if (request->hasParam(PARAM_PORT)) {
      String val = request->getParam(PARAM_PORT)->value();
      writeFile(SPIFFS, "/port.txt", val.c_str());
      Serial.println("[SAVE] Port: " + val);
    }
    else if (request->hasParam("topicPub")) {
      String val = request->getParam("topicPub")->value();
      writeFile(SPIFFS, "/topicPub.txt", val.c_str());
      Serial.println("[SAVE] Topic Publish: " + val);
    }
    else if (request->hasParam("topicSub")) {
      String val = request->getParam("topicSub")->value();
      writeFile(SPIFFS, "/topicSub.txt", val.c_str());
      Serial.println("[SAVE] Topic Subscribe: " + val);
    }
    else if (request->hasParam("batReal")) {
      // Factor = real / medido sin calibrar (promedio de varias lecturas).
      float real = request->getParam("batReal")->value().toFloat();
      float raw = 0;
      for (int i = 0; i < 8; i++) { raw += vBatRaw(); delay(5); }
      raw /= 8;
      if (real <= 0.1f) batCalFactor = 1.0f;
      else if (raw > 0.5f) batCalFactor = real / raw;
      if (batCalFactor < 0.5f || batCalFactor > 1.5f) {
        msg = "Factor fuera de rango (" + String(batCalFactor, 3) + "), no se guarda";
        batCalFactor = readFile(SPIFFS, "/batCal.txt").toFloat();
        if (batCalFactor <= 0) batCalFactor = 1.0f;
      } else {
        saveSetting(String(batCalFactor, 4), "/batCal.txt");
      }
      Serial.printf("[SAVE] Batería: real %.2f V, medido %.2f V -> factor %.4f\n", real, raw, batCalFactor);
    }
    else if (request->hasParam("apn")) {
      String val = request->getParam("apn")->value();
      writeFile(SPIFFS, "/apn.txt", val.c_str());
      RUNapn = val;
      Serial.println("[SAVE] APN: " + val);
    }
    else if (request->hasParam("dirSensor")) {
      String val = request->getParam("dirSensor")->value();
      writeFile(SPIFFS, "/dirSensor.txt", val.c_str());
      dirSensorType = val;
      Serial.println("[SAVE] dirSensor: " + val);
    }
    else if (request->hasParam("period")) {
      String period = request->getParam("period")->value();
      writeFile(SPIFFS, "/timesleep.txt", period.c_str());
      TIME_TO_SLEEP = period.toInt();
      Serial.println("[SAVE] Period: " + period + " seg");
    }
    else if (request->hasParam("rs485")) {
      String val = rs485Norm(request->getParam("rs485")->value());
      writeFile(SPIFFS, "/rs485.txt", val.c_str());
      rs485Type = val;
      Serial.println("[SAVE] rs485: " + val);
    }
    else if (request->hasParam("oledForm")) {
      writeFile(SPIFFS, "/oledOp.txt", request->hasParam("oledOp") ? "1" : "0");
      Serial.println("[SAVE] OLED en operación: " + String(request->hasParam("oledOp") ? "1" : "0"));
    }
    else if (request->hasParam("resetRain")) {
      writeFile(SPIFFS, "/ssidRainCounter.txt", "0");
      rainPulseCountISR = 0;
      Serial.println("[SAVE] Contador de lluvia reiniciado");
    }
    else if (request->hasParam(PARAM_RESET)) {
      request->send(200, "text/plain", "Reiniciando...");
      delay(500);
      ESP.restart();
      return;
    }
    else {
      writeFile(SPIFFS, "/A_TH.txt", request->hasParam("A_TH") ? "1" : "0");
      writeFile(SPIFFS, "/A_PR.txt", request->hasParam("A_PR") ? "1" : "0");
      writeFile(SPIFFS, "/A_WS.txt", request->hasParam("A_WS") ? "1" : "0");
      Serial.println("[SAVE] Configuración de sensores guardada");
    }

    request->redirect("/");
  });

  server.onNotFound(notFound);
}

/* ====================== OLED + BOTONES ======================== */
// Pantalla en vertical (rotada 90°): 64 x 128 px, 10 columnas x 16 filas
// con la fuente 6x8. Las cadenas van sin tildes: la fuente por defecto de
// Adafruit GFX no trae UTF-8.
//
// Dashboard (dashDraw()):
//   y   0-10  barra de estado: batería (icono + %) | tipo de enlace + señal
//   y  13-21  modo (OPER / CONFIG / ESTADO) invertido
//   y  24-95  solo los sensores HABILITADOS, una fila cada uno
//   y  98-125 proceso: paso actual + 2 líneas de detalle
//             (+ línea MODEM invertida encima cuando hay diagnóstico del
//             SIM800: el proceso sube 9 px y se recortan sensores si no caben)
//   y 126-127 barra de progreso de la secuencia

void sensorsRailOn() {
  gpio_hold_dis((gpio_num_t)SENS_PWR_PIN);
  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, LOW);  // MOSFET canal P: LOW = ON
  if (!sensRailOnMs) sensRailOnMs = millis() | 1;  // |1: nunca 0 (= apagado)
}

void sensorsRailOff() {
  gpio_hold_dis((gpio_num_t)SENS_PWR_PIN);
  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, HIGH);
  sensRailOnMs = 0;
}

// Espera hasta que hayan pasado ms desde el instante t0 (no espera si ya pasaron).
void waitSince(uint32_t t0, uint32_t ms) {
  uint32_t el = millis() - t0;
  if (el < ms) delay(ms - el);
}

// Requiere v_sensor encendido y Wire ya iniciado (SDA=21/SCL=22).
bool oledInit() {
  if (oledReady) return true;
  delay(100);  // arranque del SSD1306 tras energizar v_sensor

  // Adafruit_SSD1306::begin() no verifica el ACK del bus: sin este sondeo
  // reportaría éxito aunque el panel no esté conectado.
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[OLED] No detectado en 0x3C");
    return false;
  }
  // periphBegin=false: no re-inicializar Wire con los pines por defecto
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false)) {
    Serial.println("[OLED] Sin memoria para el buffer");
    return false;
  }
  oled.setRotation(OLED_ROTATION);
  oled.setTextWrap(false);
  oled.clearDisplay();
  oled.display();
  oledReady = true;
  Serial.println("[OLED] Listo");
  return true;
}

void oledPower(bool on) {
  if (oledReady) oled.ssd1306_command(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
}

/* ---------- Estado del dashboard ---------- */
enum DashVal { DV_T, DV_H, DV_WV, DV_WD, DV_RA, DV_ST, DV_SH, DV_EC, DV_PH, DV_RAD, DV_LV, DV_COUNT };

struct Dash {
  String mode = "";         // OPER / CONFIG / ESTADO
  String link = "";         // LTE / WIFI / SD
  float batV = 0;
  int8_t bars = -1;         // señal 0..4, -1 = sin dato
  String step = "";         // paso actual de la secuencia
  String detail[2];         // detalle del paso (o resultado)
  String modem = "";        // línea de diagnóstico del modem ("" = oculta)
  uint8_t stepIdx = 0, stepTotal = 0;
  String v[DV_COUNT];       // valores ya formateados, "--" = sin dato
} dash;

void dashDraw();

int csqToBars(int csq) {
  if (csq <= 1 || csq == 99) return 0;
  if (csq < 10) return 1;
  if (csq < 15) return 2;
  if (csq < 20) return 3;
  return 4;
}

int rssiToBars(int rssi) {
  if (rssi >= -55) return 4;
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  if (rssi >= -85) return 1;
  return 0;
}

// Li-ion 1S: 3.30 V = 0 %, 4.20 V = 100 % (aprox. lineal, suficiente aquí)
int batPercent(float v) {
  int p = (int)((v - 3.30f) * 100.0f / 0.90f);
  return p < 0 ? 0 : (p > 100 ? 100 : p);
}

String dashLinkName(const String &type) {
  if (type == "LTE") return "LTE";
  if (type == "WIFI") return "WIFI";
  if (type == "DATALOGER") return "SD";
  return type.substring(0, 4);
}

// Pasos de la secuencia del modo operación según el tipo de enlace
uint8_t dashStepsFor(const String &type) {
  if (type == "LTE") return 8;    // SENSORES SD MODEM RED INTERNET SERVIDOR ENVIANDO FIN
  if (type == "WIFI") return 6;   // SENSORES SD WIFI SERVIDOR ENVIANDO FIN
  return 3;                       // SENSORES SD FIN
}

void dashBegin(const String &mode, uint8_t totalSteps) {
  dash.mode = mode;
  dash.link = dashLinkName(connectionType);
  dash.batV = vBat().toFloat();
  dash.bars = -1;
  dash.step = "";
  dash.detail[0] = dash.detail[1] = "";
  // Si el último reset ocurrió con el modem encendido, se deja a la vista
  // el paso en que murió hasta que el modem vuelva a usarse.
  // (el paso técnico queda en la página de detalle del modem)
  dash.modem = rstModemStep.length() ? "!REINICIO" : "";
  dash.stepIdx = 0;
  dash.stepTotal = totalSteps;
  for (int i = 0; i < DV_COUNT; i++) dash.v[i] = "--";
  dashDraw();
}

// Detalle de 2 líneas: corta en un espacio si el texto no cabe en una.
void dashSetDetail(const String &text) {
  String t = text;
  t.trim();
  if (t.length() <= 10) {
    dash.detail[0] = t;
    dash.detail[1] = "";
    return;
  }
  int cut = t.lastIndexOf(' ', 10);
  if (cut <= 0) cut = 10;
  dash.detail[0] = t.substring(0, cut);
  String rest = t.substring(cut);
  rest.trim();
  dash.detail[1] = rest.substring(0, 10);
}

// Nuevo paso de la secuencia (avanza la barra de progreso)
void dashStep(const String &step, const String &detail = "") {
  if (!oledReady) return;
  dash.step = step;
  if (dash.stepIdx < dash.stepTotal) dash.stepIdx++;
  dashSetDetail(detail);
  dashDraw();
}

// Último paso (resultado): completa la barra aunque se hayan saltado pasos
void dashFinish(const String &result, const String &detail = "") {
  if (!oledReady) return;
  dash.step = result;
  dash.stepIdx = dash.stepTotal;
  dashSetDetail(detail);
  dashDraw();
}

void dashSet(DashVal idx, const String &value) {
  dash.v[idx] = value;
  if (oledReady) dashDraw();
}

void dashSignal(int bars) {
  dash.bars = bars;
  if (oledReady) dashDraw();
}

// Detalle del paso en curso (compatibilidad: todos los mensajes de
// progreso del firmware pasan por aquí).
void oledLog(const String &line) {
  if (!oledReady) return;
  dashSetDetail(line);
  dashDraw();
}

void oledBegin(const String &mode) {
  dashBegin(mode, dashStepsFor(connectionType));
}

// Valor de un sensor tal como llega del JSON / caché ("error", "NA"...)
String dashFmt(const String &raw, int decimals, const char *unit) {
  if (raw == "" || raw == "--" || raw == "NA" || raw == "null") return "--";
  if (raw == "error") return "ERR";
  return String(raw.toFloat(), decimals) + unit;
}

String dashFmtDir(const String &raw) {
  if (raw == "" || raw == "--" || raw == "NA" || raw == "null") return "--";
  if (raw == "error") return "ERR";
  static const char *pts[] = {"N", "NE", "E", "SE", "S", "SO", "O", "NO"};
  float d = raw.toFloat();
  int i = ((int)((d + 22.5f) / 45.0f)) & 7;
  return String((int)(d + 0.5f) % 360) + " " + pts[i];
}

// Carga los valores de un JSON de lectura (lastDataJson) en el dashboard
void dashLoadJson(JsonDocument &d) {
  dash.v[DV_T]   = dashFmt(d["T"] | "--", 1, "C");
  dash.v[DV_H]   = dashFmt(d["H"] | "--", 0, "%");
  dash.v[DV_WV]  = dashFmt(d["WV"] | "--", 1, "m/s");
  dash.v[DV_WD]  = dashFmtDir(d["WD"] | "--");
  dash.v[DV_RA]  = d.containsKey("RA") ? d["RA"].as<String>() : "--";
  dash.v[DV_ST]  = dashFmt(d["ST"] | "--", 1, "C");
  dash.v[DV_SH]  = dashFmt(d["SH"] | "--", 0, "%");
  dash.v[DV_EC]  = dashFmt(d["EC"] | "--", 0, "uS");
  dash.v[DV_PH]  = dashFmt(d["PH"] | "--", 1, "");
  dash.v[DV_RAD] = dashFmt(d["RAD"] | "--", 0, "W");
  dash.v[DV_LV]  = dashFmt(d["LV"] | "--", 0, "");
}

/* ---------- Dibujo ---------- */
void dashText(int x, int y, const String &s, bool inverse = false) {
  oled.setTextColor(inverse ? SSD1306_BLACK : SSD1306_WHITE);
  oled.setCursor(x, y);
  oled.print(s);
  oled.setTextColor(SSD1306_WHITE);
}

void dashTextRight(int y, const String &s, bool inverse = false) {
  dashText(oled.width() - 6 * (int)s.length(), y, s, inverse);
}

void dashDrawBattery(int x, int y) {
  // Cuerpo 13x7 + borne 2x3
  oled.drawRect(x, y, 13, 7, SSD1306_WHITE);
  oled.fillRect(x + 13, y + 2, 2, 3, SSD1306_WHITE);
  if (dash.batV < BAT_VALID_MIN_V) {
    dashText(x + 18, y, "USB");
    return;
  }
  int pct = batPercent(dash.batV);
  oled.fillRect(x + 2, y + 2, (9 * pct + 50) / 100, 3, SSD1306_WHITE);
  dashText(x + 18, y, String(dash.batV, 2) + "V");
}

void dashDrawBars(int x, int y) {
  // 4 barras de 2 px, alturas 2/4/6/8; sin dato = solo la base
  for (int i = 0; i < 4; i++) {
    int h = 2 + i * 2;
    int bx = x + i * 3;
    if (dash.bars > i) oled.fillRect(bx, y + 8 - h, 2, h, SSD1306_WHITE);
    else oled.drawFastHLine(bx, y + 7, 2, SSD1306_WHITE);
  }
}

void dashRow(int y, const char *label, const String &value) {
  dashText(0, y, String(label).substring(0, 3));
  dashTextRight(y, value.substring(0, 7));
}

void dashDraw() {
  if (!oledReady) return;
  const int W = oled.width();
  oled.clearDisplay();

  // Barra de estado
  dashDrawBattery(0, 1);
  if (dash.link != "SD") dashDrawBars(W - 11, 1);

  // Modo + enlace (invertido)
  oled.fillRect(0, 12, W, 10, SSD1306_WHITE);
  dashText(1, 13, dash.mode, true);
  dashTextRight(13, dash.link, true);

  // Sensores habilitados (con la línea MODEM visible el área baja a y<88)
  const int procY = dash.modem.length() ? 88 : 97;
  int y = 25;
  const int rowH = 8;
  // Nombres de 3 caracteres: TSU/HSU = temperatura/humedad del suelo
  if (A_TH) { dashRow(y, "TEM", dash.v[DV_T]); y += rowH; dashRow(y, "HUM", dash.v[DV_H]); y += rowH; }
  if (A_WS) { dashRow(y, "VEL", dash.v[DV_WV]); y += rowH; dashRow(y, "DIR", dash.v[DV_WD]); y += rowH; }
  if (A_PR) { dashRow(y, "LLU", dash.v[DV_RA]); y += rowH; }
  if (rs485Type == "SOIL") {
    dashRow(y, "TSU", dash.v[DV_ST]); y += rowH;
    dashRow(y, "HSU", dash.v[DV_SH]); y += rowH;
    dashRow(y, "EC", dash.v[DV_EC]); y += rowH;
    dashRow(y, "PH", dash.v[DV_PH]); y += rowH;
  } else if (rs485Type == "RAD") {
    dashRow(y, "RAD", dash.v[DV_RAD]); y += rowH;
  } else if (rs485Type == "LEVEL") {
    dashRow(y, "NIV", dash.v[DV_LV]); y += rowH;
  }
  if (y == 25) dashText(0, y, "Sin sens.");

  // Proceso (borra sensores que se hayan dibujado por debajo de procY)
  oled.fillRect(0, procY, W, 128 - procY, SSD1306_BLACK);
  oled.drawFastHLine(0, procY, W, SSD1306_WHITE);
  int py = procY + 2;
  if (dash.modem.length()) {
    oled.fillRect(0, py - 1, W, 9, SSD1306_WHITE);
    dashText(0, py, dash.modem.substring(0, 10), true);
    py += 9;
  }
  dashText(0, py, dash.step.substring(0, 10));
  dashText(0, py + 9, dash.detail[0].substring(0, 10));
  dashText(0, py + 17, dash.detail[1].substring(0, 10));
  if (dash.stepTotal > 0) {
    oled.drawFastHLine(0, 127, W, SSD1306_WHITE);
    oled.fillRect(0, 125, W * dash.stepIdx / dash.stepTotal, 3, SSD1306_WHITE);
  }
  oled.display();
}

// Página de información (vertical): título invertido + filas "clave/valor".
// Cada entrada ocupa 1 fila si cabe ("CSQ    14") o 2 si no (clave arriba,
// valor abajo, alineado a la derecha).
void oledInfoPage(const String &title, const String keys[], const String vals[], int n, const String &footer) {
  if (!oledReady) return;
  const int W = oled.width();
  oled.clearDisplay();
  oled.fillRect(0, 0, W, 10, SSD1306_WHITE);
  dashText(1, 1, title.substring(0, 10), true);
  int y = 13;
  const int yMax = footer.length() ? 110 : 120;
  for (int i = 0; i < n && y <= yMax; i++) {
    String v = vals[i].substring(0, 10);
    if (keys[i].length() + 1 + v.length() <= 10) {
      dashText(0, y, keys[i]);
      dashTextRight(y, v);
      y += 9;
    } else {
      if (y + 9 > yMax) break;
      dashText(0, y, keys[i]);
      dashTextRight(y + 8, v);
      y += 18;
    }
  }
  if (footer.length()) {
    oled.drawFastHLine(0, 118, W, SSD1306_WHITE);
    dashText(0, 120, footer.substring(0, 10));
  }
  oled.display();
}

// Traducción de lastNetStatus (técnico, va al JSON/SD/serie) a texto para un
// usuario básico en el OLED: rótulo corto (<=10), problema y qué hacer.
struct NetFriendly {
  const char *status;   // valor de lastNetStatus
  const char *shortTxt; // <= 10 caracteres
  const char *problem;
  const char *advice;
};

static const NetFriendly NET_FRIENDLY[] = {
  {"Modem sobrevoltaje", "VOLT ALTO", "Voltaje muy alto en el modem", "Revise el regulador del modem"},
  {"Modem bajo voltaje", "BAT BAJA", "Bateria debil para el modem", "Cargue o cambie la bateria"},
  {"Modem se apago", "BAT BAJA", "El modem se apago solo", "Cargue o cambie la bateria"},
  {"Modem sin energia", "SIN MODEM", "El modem no enciende", "Revise la conexion del modem"},
  {"Registro denegado", "ERROR SIM", "La red rechazo la SIM", "Revise la SIM y su saldo"},
  {"Sin cobertura", "SIN SENAL", "No hay senal celular", "Revise la antena o mueva el equipo"},
  {"Fallo GPRS", "SIN DATOS", "Sin internet movil", "Revise saldo de datos y APN"},
  {"Fallo MQTT", "ERROR SERV", "El servidor no responde", "Se volvera a intentar luego"},
  {"Fallo publicacion", "ERROR ENV", "No se pudo enviar el dato", "Se volvera a intentar luego"},
};
// Resto de causas "Modem ..." / "RX ..." / "TX ..." de modemDiagCause()
static const NetFriendly NET_FRIENDLY_MODEM = {"", "SIN MODEM", "El modem no responde", "Revise cables y antena del modem"};

const NetFriendly *netFriendly(const char *status) {
  if (!status || !status[0]) return nullptr;
  for (const NetFriendly &f : NET_FRIENDLY)
    if (strcmp(status, f.status) == 0) return &f;
  if (strncmp(status, "Modem", 5) == 0 || strncmp(status, "RX ", 3) == 0 || strncmp(status, "TX ", 3) == 0)
    return &NET_FRIENDLY_MODEM;
  return nullptr;  // éxito u otro texto ya legible ("Enviado OK (LTE)")
}

// Texto corto para el dashboard: el rótulo amigable si es un fallo conocido.
String netStatusShort(const char *status) {
  const NetFriendly *f = netFriendly(status);
  return f ? String(f->shortTxt) : String(status);
}

// Resultado de la conectividad de este ciclo: se muestra y queda guardado
// en memoria RTC para la consulta de estado posterior.
void setNetStatus(const char *msg) {
  strlcpy(lastNetStatus, msg, sizeof(lastNetStatus));
  oledLog(netStatusShort(msg));
}

// Dibuja text partido en palabras a 10 columnas desde y; devuelve la y
// siguiente. No pasa de yMax.
int oledWrap(int y, const String &text, int yMax) {
  String rest = text;
  rest.trim();
  while (rest.length() && y <= yMax) {
    int cut = rest.length() <= 10 ? rest.length() : rest.lastIndexOf(' ', 10);
    if (cut <= 0) cut = min((int)rest.length(), 10);
    dashText(0, y, rest.substring(0, cut));
    rest = rest.substring(cut);
    rest.trim();
    y += 9;
  }
  return y;
}

// Página amigable del resultado del modem: qué pasó y qué hacer, sin datos
// técnicos (esos están en drawModemDiagPage()).
void drawModemFriendlyPage(const String &footer) {
  if (!oledReady) return;
  const int W = oled.width();
  const NetFriendly *f = netFriendly(lastNetStatus);
  oled.clearDisplay();
  oled.fillRect(0, 0, W, 10, SSD1306_WHITE);
  dashText(1, 1, f ? "SIN ENVIO" : "MODEM", true);
  const int yMax = footer.length() ? 108 : 118;
  int y = 13;
  if (f) {
    y = oledWrap(y, f->problem, yMax) + 4;
    dashText(0, y, "Que hacer:");
    y = oledWrap(y + 9, f->advice, yMax) + 4;
    oledWrap(y, "Datos guardados", yMax);
  } else if (lastNetStatus[0]) {
    y = oledWrap(y, lastNetStatus, yMax) + 4;
    if (strncmp(lastSignal, "CSQ ", 4) == 0) {
      static const char *const quality[] = {"nula", "debil", "regular", "buena", "excelente"};
      oledWrap(y, String("Senal ") + quality[csqToBars(atoi(lastSignal + 4))], yMax);
    }
  } else {
    oledWrap(y, "Sin datos todavia", yMax);
  }
  if (footer.length()) {
    oled.drawFastHLine(0, 118, W, SSD1306_WHITE);
    dashText(0, 120, footer.substring(0, 10));
  }
  oled.display();
}

/* ---------- Botones (antirrebote + pulsación corta/larga) ---------- */
enum BtnEvent { BTN_NONE, BTN_SHORT, BTN_LONG };

struct Button {
  uint8_t pin;
  bool raw;                 // última lectura (true = presionado)
  bool stable;              // estado ya filtrado por antirrebote
  bool suppress;            // ignorar hasta soltar (ya disparó LONG o venía presionado)
  unsigned long rawChangedAt;
  unsigned long downAt;
};

Button btnOk = {BTN_OK_PIN, false, false, false, 0, 0};
Button btnUp = {BTN_UP_PIN, false, false, false, 0, 0};

void buttonInit(Button &b) {
  pinMode(b.pin, INPUT);  // pull-up externo (R18/R19 100k)
  b.raw = b.stable = b.suppress = (digitalRead(b.pin) == LOW);
  b.rawChangedAt = b.downAt = millis();
}

void buttonsInit() {
  // Si venimos de un wakeup EXT0, GPIO35 sigue en modo RTC IO
  rtc_gpio_deinit((gpio_num_t)BTN_OK_PIN);
  // Un botón que ya está presionado al iniciar (p.ej. el mismo toque que
  // despertó al equipo) no genera evento hasta que se suelte.
  buttonInit(btnOk);
  buttonInit(btnUp);
}

BtnEvent pollButton(Button &b) {
  bool raw = (digitalRead(b.pin) == LOW);
  unsigned long now = millis();
  if (raw != b.raw) {
    b.raw = raw;
    b.rawChangedAt = now;
  }
  if (now - b.rawChangedAt < BTN_DEBOUNCE_MS) return BTN_NONE;

  if (raw != b.stable) {
    b.stable = raw;
    if (raw) {
      b.downAt = now;
      return BTN_NONE;
    }
    bool wasSuppressed = b.suppress;
    b.suppress = false;
    return wasSuppressed ? BTN_NONE : BTN_SHORT;
  }
  if (raw && !b.suppress && (now - b.downAt) >= BTN_LONG_MS) {
    b.suppress = true;
    return BTN_LONG;
  }
  return BTN_NONE;
}

/* ---------- Vistas del modo configuración ---------- */
// 0 = dashboard en vivo (sensCache) | 1 = red | 2 = sistema
#define CONFIG_PAGES 3

void drawConfigPage(int page, bool fresh) {
  // Lo que sale de SPIFFS se relee solo al cambiar de página (readFile()
  // registra cada lectura en el monitor serie y se redibuja cada segundo).
  static String cType, cNetKey, cNet, cServer, cPeriod;
  static int cPending = 0;
  static unsigned long lastBat = 0;
  if (fresh) {
    cType = readFile(SPIFFS, "/connType.txt");
    dash.link = dashLinkName(cType);
    if (cType == "LTE") { cNetKey = "APN"; cNet = readFile(SPIFFS, "/apn.txt"); }
    else if (cType == "DATALOGER") { cNetKey = "Envio"; cNet = "no (SD)"; }
    else { cNetKey = "SSID"; cNet = readFile(SPIFFS, "/ssidString.txt"); }
    cServer = readFile(SPIFFS, "/server.txt");
    if (cServer == "") cServer = broker;
    cPeriod = String(readFile(SPIFFS, "/timesleep.txt").toInt() / 60) + "min";
    cPending = countPendingRecords();
  }

  const String footer = "OK2s:salir";
  if (page == 0) {
    if (fresh || millis() - lastBat >= 5000) {
      dash.batV = vBat().toFloat();
      lastBat = millis();
    }
    dash.v[DV_T]   = dashFmt(sensCache.temp, 1, "C");
    dash.v[DV_H]   = dashFmt(sensCache.hum, 0, "%");
    dash.v[DV_WV]  = dashFmt(sensCache.wvel, 1, "m/s");
    dash.v[DV_WD]  = dashFmtDir(sensCache.wdir);
    dash.v[DV_RA]  = (sensCache.rainCount == "NA") ? "--" : sensCache.rainCount;
    dash.v[DV_ST]  = dashFmt(sensCache.rsT, 1, "C");
    dash.v[DV_SH]  = dashFmt(sensCache.rsH, 0, "%");
    dash.v[DV_EC]  = dashFmt(sensCache.rsEC, 0, "uS");
    dash.v[DV_PH]  = dashFmt(sensCache.rsPH, 1, "");
    dash.v[DV_RAD] = dashFmt(sensCache.rsRad, 0, "W");
    dash.v[DV_LV]  = dashFmt(sensCache.rsLv, 0, "");
    // Alterna cada 3 s entre el estado del AP y la MAC del equipo (para
    // identificarlo: es el sufijo del topic de config). La MAC no cabe en
    // 10 columnas, va en dos mitades "AA:BB:CC" / "DD:EE:FF".
    if ((millis() / 3000) % 2) {
      String mac = macA();
      dash.step = "MAC";
      dash.detail[0] = mac.substring(0, 8);
      dash.detail[1] = mac.substring(9);
    } else {
      dash.step = serverReady ? "AP LISTO" : "INICIANDO";
      dash.detail[0] = WiFi.softAPIP().toString();
      dash.detail[1] = "Clientes " + String(WiFi.softAPgetStationNum());
    }
    dash.stepIdx = dash.stepTotal;
    dashDraw();
    return;
  }
  if (page == 1) {
    String k[] = {"AP", "Clave", "IP", "Enlace", cNetKey, "MQTT", "Periodo", "Pend."};
    String v[] = {String(ssid), String(password), WiFi.softAPIP().toString(), dashLinkName(cType),
                  cNet, cServer, cPeriod, String(cPending)};
    oledInfoPage("RED", k, v, 8, footer);
    return;
  }
  // MAC completa en dos filas (12 hex no caben en 10 columnas)
  String k[] = {"Fecha", "Hora", "SD", "MAC", "", "RS485"};
  String ts = formatTimestamp(getEpochNow());  // AAAA-MM-DDTHH:MM:SS
  String mac = macA();
  String v[] = {ts.substring(0, 10), ts.substring(11, 19),
                sdReady ? String((unsigned long)sdFreeMB) + "MB" : "no",
                mac.substring(0, 8), mac.substring(9), rs485Type};
  oledInfoPage("SISTEMA", k, v, 6, footer);
}

// Llamada en cada vuelta de loop() en modo configuración.
//   UP corto: vista siguiente   OK corto: vista anterior
//   OK largo (2 s): salir del modo config (reinicia en modo operación)
// La pantalla se apaga tras OLED_CONFIG_TIMEOUT_MS sin uso; cualquier botón
// la vuelve a encender (ese toque no cambia de vista).
void configOledTask() {
  static int page = 0;
  static bool screenOn = true;
  static bool fresh = true;
  static unsigned long lastDraw = 0;
  static unsigned long lastActivity = millis();

  if (!oledReady) return;

  BtnEvent up = pollButton(btnUp);
  BtnEvent ok = pollButton(btnOk);

  if (up != BTN_NONE || ok != BTN_NONE) {
    lastActivity = millis();
    if (!screenOn) {
      screenOn = true;
      oledPower(true);
      fresh = true;
      up = ok = BTN_NONE;
    }
  }

  if (up == BTN_SHORT) { page = (page + 1) % CONFIG_PAGES; fresh = true; }
  if (ok == BTN_SHORT) { page = (page + CONFIG_PAGES - 1) % CONFIG_PAGES; fresh = true; }
  if (ok == BTN_LONG) {
    dash.stepIdx = dash.stepTotal;
    dash.step = "SALIENDO";
    dash.detail[0] = "modo";
    dash.detail[1] = "operacion";
    dashDraw();
    Serial.println("[OLED] Salida del modo config por botón");
    delay(1500);
    ESP.restart();
  }

  if (screenOn && millis() - lastActivity > OLED_CONFIG_TIMEOUT_MS) {
    screenOn = false;
    oledPower(false);
  }

  if (screenOn && (fresh || millis() - lastDraw >= 1000)) {
    drawConfigPage(page, fresh);
    fresh = false;
    lastDraw = millis();
  }
}

/* ---------- Consulta de estado (wakeup EXT0 por botón "ok") ---------- */
// 0 = dashboard | 1 = conectividad | 2 = sistema | 3 = modem (amigable)
// 4 = modem (detalle técnico)
#define STATUS_PAGES 5

void drawStatusPage(int page, int pending) {
  const String footer = "OK2s:medir";
  uint64_t elapsed = getRtcTimeSeconds() - cycle_start_rtc;
  long next = (cycle_active && elapsed < TIME_TO_SLEEP) ? (long)(TIME_TO_SLEEP - elapsed) : 0;

  if (page == 0) {
    dash.step = "PROX " + String(next) + "s";
    dashSetDetail(lastNetStatus[0] ? netStatusShort(lastNetStatus) : String("Sin datos"));
    dash.stepTotal = 0;  // sin barra de progreso en esta vista
    dashDraw();
    return;
  }
  if (page == 1) {
    String k[] = {"Enlace", "Ultimo", "Senal", "Vmodem", "Pend.", "Prox."};
    String v[] = {dashLinkName(connectionType),
                  lastNetStatus[0] ? String(lastNetStatus) : "--",
                  lastSignal[0] ? String(lastSignal) : "--",
                  lastModemMv > 0 ? String(lastModemMv / 1000.0f, 2) + "V" : "--",
                  String(pending), String(next) + "s"};
    oledInfoPage("CONEXION", k, v, 6, footer);
    return;
  }
  if (page == 3) {
    drawModemFriendlyPage(footer);
    return;
  }
  if (page == 4) {
    drawModemDiagPage(footer);  // detalle técnico (para el técnico)
    return;
  }
  static String brownouts = readFile(SPIFFS, "/brownouts.txt");
  String k[] = {"Fecha", "Hora", "SD", "Periodo", "Ciclos", "Brownout"};
  String ts = formatTimestamp(getEpochNow());
  String v[] = {ts.substring(0, 10), ts.substring(11, 19),
                sdReady ? String((unsigned long)sdFreeMB) + "MB" : "no",
                String((unsigned long)(TIME_TO_SLEEP / 60)) + "min",
                String(opCycleCount), brownouts};
  oledInfoPage("SISTEMA", k, v, 6, footer);
}

// Despertar por el botón "ok" (EXT0): muestra el resumen del último ciclo
// sin medir ni conectarse, y vuelve a dormir solo lo que resta del período
// (sleepRemainingCycle()), así no se corre el horario de los reportes.
//   UP/OK corto: cambiar página   OK largo (2 s): medir y enviar ahora
void runStatusViewer() {
  Serial.println("\n[OLED] ===== CONSULTA DE ESTADO (botón ok) =====");

  connectionType = readFile(SPIFFS, "/connType.txt");
  TIME_TO_SLEEP = configuredSleepPeriod();

  sensorsRailOn();
  delay(50);
  Wire.begin(I2C_SDA, I2C_SCL);
  initDS3231();

  if (!oledInit()) {
    sleepRemainingCycle();  // No retorna
    return;
  }
  buttonsInit();

  dashBegin("ESTADO", 0);
  {
    StaticJsonDocument<1024> d;
    if (lastDataJson[0] && !deserializeJson(d, (const char *)lastDataJson)) dashLoadJson(d);
  }
  if (strncmp(lastSignal, "CSQ ", 4) == 0) dash.bars = csqToBars(atoi(lastSignal + 4));
  else if (lastSignal[0]) dash.bars = rssiToBars(atoi(lastSignal));

  int pending = countPendingRecords();
  int page = 0;
  bool redraw = true;
  unsigned long lastActivity = millis();
  unsigned long lastDraw = 0;

  while (millis() - lastActivity < STATUS_VIEW_TIMEOUT_MS) {
    BtnEvent up = pollButton(btnUp);
    BtnEvent ok = pollButton(btnOk);
    if (up != BTN_NONE || ok != BTN_NONE) lastActivity = millis();

    if (up == BTN_SHORT) { page = (page + 1) % STATUS_PAGES; redraw = true; }
    if (ok == BTN_SHORT) { page = (page + STATUS_PAGES - 1) % STATUS_PAGES; redraw = true; }
    if (ok == BTN_LONG) {
      Serial.println("[OLED] Ciclo manual solicitado por botón");
      init_running();  // No retorna, entra en deep sleep
      return;
    }

    if (redraw || millis() - lastDraw >= 1000) {
      drawStatusPage(page, pending);
      redraw = false;
      lastDraw = millis();
    }
    delay(20);
  }

  sleepRemainingCycle();  // No retorna
}

/* ====================== SETTING MODE ======================== */
void setting() {
  Serial.println("\n[SETTING MODE] Iniciando...");

  // OLED: necesita v_sensor y el bus I2C desde el inicio para mostrar el
  // progreso; updateSensCache() vuelve a encender el riel (sin efecto) y su
  // Wire.begin() posterior no reinicia un bus ya iniciado.
  sensorsRailOn();
  Wire.begin(I2C_SDA, I2C_SCL);
  if (oledInit()) {
    dashBegin("CONFIG", 3);   // SD, AP, LISTO
    dashStep("INICIO");
  }
  buttonsInit();

  // Montar SD (VSPI: MISO19/MOSI23/SCK18/CS5)
  initSD();
  dashStep("SD", sdReady ? "OK" : "no detectada");

  // Cargar el ancla de hora real (ver /settime, setTimeBase())
  loadTimeBase();

  // Desconectar WiFi completamente
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500);

  // Configurar modo AP
  Serial.println("[SETTING] Configurando Access Point...");
  dashStep("AP WIFI", "creando...");
  WiFi.mode(WIFI_AP);
  delay(500);

  // Configurar IP estática
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  delay(100);

  // Crear AP con reintentos
  int retries = 0;
  while (!WiFi.softAP(ssid, password) && retries < 5) {
    Serial.printf("[WARN] Intento %d de crear AP...\n", retries + 1);
    delay(1000);
    retries++;
  }

  if (retries >= 5) {
    Serial.println("[ERROR] No se pudo crear AP");
    dashFinish("ERROR AP", "reiniciando");
    delay(2000);
    ESP.restart();
    return;
  }

  // Esperar estabilización del AP
  delay(2000);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("[OK] AP activo en: ");
  Serial.println(IP);
  dashFinish("AP LISTO", IP.toString());

  // Configurar rutas HTTP (función separada)
  Serial.println("[SETTING] Configurando rutas HTTP...");
  setupServerRoutes();

  Serial.println("[SETTING] Rutas configuradas. Listo para iniciar servidor...");
  serverReady = true;
}

/* ====================== BATTERY VOLTAGE ======================== */
// Tensión de batería SIN calibrar: divisor R10=330k (arriba) / R11=100k
// (abajo) → V_bat = V_adc × (330+100)/100 = ×4.3. Con 4.2 V de batería
// el ADC ve ~0.98 V, zona en la que analogReadMilliVolts() (corregida con
// la calibración eFuse del chip) es bastante lineal. Se promedian varias
// muestras porque la impedancia del divisor (~78k) hace ruidosa cada lectura.
float vBatRaw(int samples) {
  const float BAT_DIV_RATIO = (330.0f + 100.0f) / 100.0f;
  uint32_t mvSum = 0;
  for (int i = 0; i < samples; i++) mvSum += analogReadMilliVolts(pinBattery);
  return (mvSum / (float)samples) / 1000.0f * BAT_DIV_RATIO;
}

// Batería calibrada: batCalFactor (/batCal.txt) se obtiene desde el portal
// con un multímetro (factor = real / medido) para corregir la tolerancia de
// las resistencias y el error del ADC. (Con la relación errónea 390k/100k
// se leían 4.67 V con ~4.0 V reales; con 330k/100k da ~4.10 V.)
String vBat() {
  return String(vBatRaw(16) * batCalFactor, 2);
}

/* ========================= RAIN COUNTER ========================= */
uint64_t getRtcTimeSeconds() {
  // El RTC clock corre a ~150kHz (RTC_SLOW_CLK)
  // rtc_time_get() retorna ticks del RTC
  uint64_t rtc_ticks = rtc_time_get();
  uint32_t rtc_freq = rtc_clk_slow_freq_get_hz();
  if (rtc_freq == 0) rtc_freq = 150000;  // Valor por defecto
  return rtc_ticks / rtc_freq;
}

// Aviso breve de una vaciada en el OLED (si /oledOp.txt lo permite). Pensado
// para el wake EXT1, que se repite con cada vaciada: ~0.1 s de arranque del
// panel + RAIN_OLED_MS en pantalla, a 80 MHz, y v_sensor se corta de nuevo en
// sleepRemainingCycle(). No se encienden sensores ni se leen archivos extra.
#define RAIN_OLED_MS 700
void rainOledFlash(int rain) {
  if (readFile(SPIFFS, "/oledOp.txt") == "0") return;
  sensorsRailOn();
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!oledInit()) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(14, 20);
  oled.print("LLUVIA");
  oled.setTextSize(3);
  oled.setCursor(14, 44);
  oled.print("+1");
  String n = String(rain);
  oled.setTextSize(n.length() <= 5 ? 2 : 1);
  int w = n.length() * 6 * (n.length() <= 5 ? 2 : 1);
  oled.setCursor((64 - w) / 2, 86);
  oled.print(n);
  oled.setTextSize(1);
  oled.setCursor(8, 106);
  oled.print("vaciadas");
  oled.display();
  delay(RAIN_OLED_MS);
  // El panel lo apaga sleepRemainingCycle(); si el período venció y se pasa
  // a init_running(), sigue encendido para el dashboard del ciclo.
}

void runRainCounter() {
  Serial.println("\n[RAIN] ===== PULSO DE LLUVIA =====");
  Serial.flush();
  setCpuFrequencyMhz(80);  // este wake solo cuenta (y avisa en el OLED)

  // El wakeup EXT1 (ESP_EXT1_WAKEUP_ALL_LOW) solo dispara si el pin ya
  // estaba en LOW (campo presente), así que el pulso es real por
  // definición: se cuenta directo, sin volver a confirmar el nivel por
  // ADC (esa verificación venía descartando vaciados reales por
  // problemas de lectura del pad justo tras el wake). Único filtro que
  // queda: no contar dos veces el mismo vaciado si el pin sigue
  // rebotando y se re-arma el wakeup muy rápido.
  rtc_gpio_deinit((gpio_num_t)RAINCOUNT);
  pinMode(RAINCOUNT, INPUT);

  uint64_t nowTicks = rtc_time_get();
  uint32_t rtcFreq = rtc_clk_slow_freq_get_hz();
  if (rtcFreq == 0) rtcFreq = 150000;
  uint64_t minIntervalTicks = (uint64_t)rtcFreq * RAIN_PULSE_MIN_INTERVAL_MS / 1000;
  bool intervalOk = (lastRainPulseRtcTicks == 0) ||
                     ((nowTicks - lastRainPulseRtcTicks) > minIntervalTicks);

  int rain = -1;
  if (intervalOk) {
    rain = readFile(SPIFFS, "/ssidRainCounter.txt").toInt();
    rain++;
    writeFile(SPIFFS, "/ssidRainCounter.txt", String(rain).c_str());
    lastRainPulseRtcTicks = nowTicks;
    Serial.printf("[RAIN] Contador: %d vaciadas\n", rain);
  } else {
    Serial.println("[RAIN] Descartado (muy seguido, <300 ms), no se cuenta");
  }

  // Esperar a que el campo deje de estar presente (máx. 100 ms) antes de
  // volver a dormir: si se rearma el EXT1 (nivel LOW = campo presente)
  // mientras el pin sigue ahí, el wakeup por nivel dispara de inmediato
  // otra vez. Se usa el mismo criterio por ADC que la detección — más
  // confiable que digitalRead() con un nivel de reposo (~2.3V) tan cerca
  // del umbral digital — y un cap más corto (100 ms en vez de 200 ms) ya
  // que el wake solo tiene que incrementar un contador.
  unsigned long start = millis();
  while (analogReadMilliVolts(RAINCOUNT) < RAIN_ADC_IDLE_MV && (millis() - start) < 100) {
    delayMicroseconds(500);
  }

  if (rain >= 0) rainOledFlash(rain);

  sleepRemainingCycle();  // No retorna, entra en deep sleep
}

// Vuelve a dormir solo lo que falta del período en curso, para que un
// despertar "extra" (pulso de lluvia por EXT1 o botón "ok" por EXT0) no
// corra el horario del próximo ciclo por TIMER. Si el período ya venció,
// ejecuta el ciclo de operación pendiente en vez de reprogramarlo.
void sleepRemainingCycle() {
  // Calcular tiempo restante
  TIME_TO_SLEEP = configuredSleepPeriod();

  uint64_t now_rtc = getRtcTimeSeconds();

  // Si no hay un ciclo en curso (primer wake por lluvia desde el último
  // reporte por TIMER, o el primero desde el flasheo), este mismo
  // despertar arranca el seguimiento del período: si no se hiciera esto,
  // cycle_start_rtc/cycle_active solo los fija deepSleepSystem() (ciclo
  // TIMER), y una cadena de wakeups por lluvia sin que medie un ciclo
  // TIMER de por medio nunca tendría contra qué medir el tiempo
  // transcurrido — remaining quedaba siempre en TIME_TO_SLEEP completo,
  // sin restar nada, en vez de ir descontando entre wakeups.
  if (!cycle_active || cycle_start_rtc == 0) {
    cycle_start_rtc = bootRtcSeconds;
    cycle_active = true;
  }

  uint64_t elapsed = now_rtc - cycle_start_rtc;
  int remaining;

  if (elapsed >= TIME_TO_SLEEP) {
    // El ciclo ya venció mientras dormíamos: si solo reprogramamos otro
    // período completo aquí, un GPIO39 ruidoso (sin pull-up, ver notas
    // arriba) que siga generando wakeups EXT1 más seguido que
    // TIME_TO_SLEEP puede postergar la lectura/envío real de sensores
    // indefinidamente — el dispositivo "nunca sale" del ciclo dormir/
    // despertar por lluvia y no llega a reportar. En vez de eso, se
    // ejecuta el ciclo de operación pendiente ahora mismo.
    Serial.println("[SLEEP] Período vencido durante un wake intermedio: ejecutando ciclo de operación pendiente");
    cycle_active = false;
    init_running();  // No retorna, entra en deep sleep
    return;
  }

  remaining = TIME_TO_SLEEP - elapsed;
  if (remaining < 10) remaining = 10;

  Serial.printf("[SLEEP] Restante: %d seg (%d min)\n", remaining, remaining / 60);

  oledPower(false);

  // ===== APAGAR TODO (modem: LOW=OFF / sensores: HIGH=OFF) =====
  gpio_hold_dis((gpio_num_t)MODEM_PWR_PIN);
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, MODEM_PWR_OFF);
  gpio_hold_en((gpio_num_t)MODEM_PWR_PIN);

  // RX2/TX2 a LOW: evita retroalimentar el SIM800 por los pines UART
  // mientras está sin alimentación (ver modemLinesOff()).
  modemLinesOff();
  rs485RailOff();

  gpio_hold_dis((gpio_num_t)SENS_PWR_PIN);
  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, HIGH);
  gpio_hold_en((gpio_num_t)SENS_PWR_PIN);

  // Habilitar hold global
  gpio_deep_sleep_hold_en();

  // Wake ups
  armWakeSources((uint64_t)remaining);

  Serial.println("[SLEEP] Volviendo a deep sleep...\n");
  Serial.flush();

  esp_deep_sleep_start();
}

/* ====================== READ DHT22 ======================== */
// Rango físico del DHT22/AM2301 según datasheet: la lectura se descarta
// (success=false) si cae fuera de estos límites, para que el llamador
// vuelva a pedir el dato en vez de propagar un valor imposible.
#define DHT22_TEMP_MIN -40.0f
#define DHT22_TEMP_MAX 80.0f
#define DHT22_HUM_MIN 0.0f
#define DHT22_HUM_MAX 100.0f

// force = true: la librería DHT devuelve el resultado anterior si se la
// llama antes de 2 s; forzar la trama permite muestrear más seguido
// (readHumidity() reutiliza la misma trama que acaba de leer readTemperature()).
void readDHT22(float &temperature, float &humidity, bool &success, bool force) {
  success = false;
  temperature = -999;
  humidity = -999;

  float t = dht.readTemperature(false, force);
  float h = dht.readHumidity();

  if (isnan(t) || isnan(h)) {
    Serial.println("[DHT22] Error de lectura");
    return;
  }

  if (t < DHT22_TEMP_MIN || t > DHT22_TEMP_MAX || h < DHT22_HUM_MIN || h > DHT22_HUM_MAX) {
    Serial.printf("[DHT22] Lectura fuera de rango, descartada (T=%.2f H=%.2f)\n", t, h);
    return;
  }

  temperature = t;
  humidity    = h;
  success     = true;
}

/* ====================== WIND FUNCTIONS ======================== */
// Contador de pulsos por interrupción, usado en modo config para calcular
// la velocidad de forma continua (no bloqueante) en vez del muestreo de
// polling de 3 s de WindVelocity().
volatile unsigned long windPulseCountISR = 0;
volatile unsigned long lastWindPulseMicrosISR = 0;
bool windISRAttached = false;

void IRAM_ATTR windPulseISR() {
  // Antirrebote: ver comentario de WIND_PULSE_DEBOUNCE_US arriba.
  unsigned long now = micros();
  if (now - lastWindPulseMicrosISR > WIND_PULSE_DEBOUNCE_US) {
    windPulseCountISR++;
    lastWindPulseMicrosISR = now;
  }
}

/* ====================== RAIN LIVE COUNTER (MODO CONFIG) ================ */
// runRainCounter() solo cuenta pulsos al despertar por EXT1 (deep sleep);
// en modo config el equipo no duerme, así que sin esta interrupción la
// precipitación nunca se actualizaba en la UI. Cuenta pulsos igual que el
// pluviómetro real: mismos dos filtros que runRainCounter() (ver ahí para
// el detalle) — confirmar el nivel LOW un par de ms y exigir 300 ms
// mínimo entre pulsos válidos.
// NOTA HW: GPIO39 es input-only y el ESP32 NO tiene pull-up/down interno
// disponible en ese pin en ningún modo (limitación de silicio, no de
// firmware, a diferencia de GPIO13 del viento que sí lo soporta). Sin una
// resistencia de pull externa en la placa, el pin queda flotando cuando
// el contacto del balancín está abierto; estos filtros por software
// reducen el ruido pero no lo eliminan del todo si el ruido persiste con
// separación >300 ms — para una solución completa hace falta esa
// resistencia externa (p.ej. 10k a 3.3V) en la placa.
volatile unsigned long rainPulseCountISR = 0;
volatile unsigned long lastRainPulseMillisISR = 0;
bool rainISRAttached = false;

void IRAM_ATTR rainPulseISR() {
  unsigned long now = millis();
  if (now - lastRainPulseMillisISR <= 300) return;

  // Confirmar el nivel LOW en una ventana corta antes de contar: un
  // glitch de ruido dura microsegundos, mucho menos que el cierre
  // mecánico real del balancín (igual que en runRainCounter()).
  for (int i = 0; i < 3; i++) {
    if (digitalRead(RAINCOUNT) == HIGH) return;
    delayMicroseconds(300);
  }

  rainPulseCountISR++;
  lastRainPulseMillisISR = now;
}

float WindDirection() {
  if (dirSensorType == "AS5600") {
    return as5600.rawAngle() * AS5600_RAW_TO_DEGREES;
  }
  return encoder.angleRead();
}

float WindVelocity() {
  // Sensor Hall: sin campo = 2.5V (HIGH), con campo = 0V (LOW). El
  // sensor es push-pull, así que el pull-up interno (~45k) no altera su
  // nivel activo (baja impedancia), pero define un HIGH sólido cuando el
  // sensor está desconectado/sin alimentar; sin él, el pin flota y el
  // ruido se contaba como pulsos de viento reales.
  // Se cuentan flancos descendentes (HIGH→LOW) = cada imán que pasa,
  // con el mismo antirrebote por tiempo que windPulseISR (ver
  // WIND_PULSE_DEBOUNCE_US): con las copas detenidas y un imán justo
  // sobre el sensor, el parpadeo eléctrico en el umbral se contaba como
  // rotación real.
  pinMode(pinWindVelocity, INPUT_PULLUP);

  int counter = 0;
  boolean lastState = digitalRead(pinWindVelocity);
  boolean curState;
  unsigned long lastPulseMicros = micros();

  // 3 s de ventana de medición (600 iter × 5 ms = 3000 ms)
  for (int i = 0; i < 600; i++) {
    curState = digitalRead(pinWindVelocity);
    if (lastState == HIGH && curState == LOW) {  // flanco descendente
      unsigned long now = micros();
      if (now - lastPulseMicros > WIND_PULSE_DEBOUNCE_US) {
        counter++;
        lastPulseMicros = now;
      }
    }
    lastState = curState;
    delay(5);
  }

  float speed = counter * (WIND_SPEED_PER_PULSE / 3.0);
  if (speed > MAX_WIND_SPEED_MS) {
    Serial.printf("[WIND] Lectura descartada por implausible: %.1f m/s (ruido del sensor Hall)\n", speed);
    return NAN;
  }
  return speed;
}

// Calibración del anemómetro (túnel de viento). La recta tiene ordenada
// al origen 0.53 m/s: aplicada a 0 pulsos (copas quietas o sensor
// desconectado) reportaba 0.5 m/s de viento inexistente. Sin pulsos = 0.
float windCal(float raw) {
  return raw > 0 ? 0.6448f * raw + 0.5343f : 0.0f;
}

/* ---------- Muestreo múltiple (modo operación) ---------- */
#define TH_SAMPLES            3
#define DHT_WARMUP_MS         4500   // riel ON -> lectura de descarte (2.5 s arranque frío + 2 s de initDHT22)
#define TH_INTERVAL_MS        1100   // lectura forzada (el DHT21 nominal es 2 s; ver readDHT22())
#define WIND_SAMPLES          2      // ventanas de velocidad + muestras de dirección
#define RS485_SAMPLES         2
#define WD_MAX_SPREAD_DEG     36.0f  // 10 % de 360°: dispersión máxima entre las muestras
#define WD_OSC_SPREAD_DEG     90.0f  // 25 % de 360°: rango ampliado con mucha oscilación
#define WD_EXTRA_SAMPLES      6      // muestras extra si las 3 primeras no son estables
#define WD_EXTRA_INTERVAL_MS  500

// Diferencia angular mínima entre dos rumbos (0..180), considerando el
// salto 359° -> 0°: 350° y 10° están a 20°, no a 340°.
float angleDiff(float a, float b) {
  float d = fabsf(fmodf(a - b, 360.0f));
  return d > 180.0f ? 360.0f - d : d;
}

// Promedio circular (vectorial): suma de vectores unitarios y atan2.
// El promedio aritmético de 350° y 10° daría 180° (opuesto); el vectorial
// da 0°.
float circularMean(const float *deg, int n) {
  float sx = 0, sy = 0;
  for (int i = 0; i < n; i++) {
    float r = deg[i] * DEG_TO_RAD;
    sx += cosf(r);
    sy += sinf(r);
  }
  float m = atan2f(sy, sx) * RAD_TO_DEG;
  if (m < 0) m += 360.0f;
  return m >= 359.95f ? 0.0f : m;
}

// Mayor diferencia angular entre cualquier par de muestras
float angularSpread(const float *deg, int n) {
  float maxD = 0;
  for (int i = 0; i < n; i++)
    for (int j = i + 1; j < n; j++) maxD = max(maxD, angleDiff(deg[i], deg[j]));
  return maxD;
}

// Grupo más grande de muestras que caben en un arco de maxSpread grados
// (todas a <= maxSpread/2 de una muestra central). Todo con angleDiff(), así
// 359° y 0° cuentan como vecinas (ambas apuntan al norte) y el grupo puede
// cruzar el norte sin problema. Deja los índices en idx y devuelve cuántas.
int largestAngularCluster(const float *deg, int n, float maxSpread, int *idx) {
  int best = 0;
  float bestSpread = 361.0f;
  int cand[WIND_SAMPLES + WD_EXTRA_SAMPLES];
  float sel[WIND_SAMPLES + WD_EXTRA_SAMPLES];
  for (int c = 0; c < n; c++) {
    int m = 0;
    for (int j = 0; j < n; j++)
      if (angleDiff(deg[j], deg[c]) <= maxSpread / 2.0f) cand[m++] = j;
    for (int k = 0; k < m; k++) sel[k] = deg[cand[k]];
    float sp = angularSpread(sel, m);
    // Más muestras gana; a igual cantidad, el grupo más compacto
    if (m > best || (m == best && sp < bestSpread)) {
      best = m;
      bestSpread = sp;
      memcpy(idx, cand, m * sizeof(int));
    }
  }
  return best;
}

// Viento: WIND_SAMPLES ventanas de 3 s de velocidad (cada una calibrada
// con windCal() y luego promediadas, así una ventana sin pulsos aporta 0
// y no la ordenada al origen) y una muestra de dirección al final de cada
// ventana. La dirección se decide en tres niveles:
//   1. ESTABLE: las WIND_SAMPLES últimas consecutivas caben en
//      WD_MAX_SPREAD_DEG (10 %); si no, se toman hasta WD_EXTRA_SAMPLES más.
//   2. OSCILANTE: si nunca se estabiliza, se amplía el rango a
//      WD_OSC_SPREAD_DEG (25 %) y se promedia el grupo más grande de
//      muestras que caben en ese arco (descarta las ráfagas aisladas que
//      apuntan a otro lado), si reúne más de la mitad de las muestras.
//   3. VARIABLE: si ni así, promedio circular de todas las muestras.
// Todas las comparaciones y promedios son circulares: 359° y 0° están a 1°.
void readWindAveraged(float &speed, bool &speedOk, float &dir, bool &dirStable) {
  float sum = 0;
  int nSpeed = 0;
  float dirs[WIND_SAMPLES + WD_EXTRA_SAMPLES];
  int nDir = 0;

  for (int i = 0; i < WIND_SAMPLES; i++) {
    float v = WindVelocity();
    dirs[nDir++] = WindDirection();
    if (!isnan(v)) { v = windCal(v); sum += v; nSpeed++; }
    Serial.printf("[WIND] Muestra %d: vel=%s dir=%.1f°\n", i + 1,
                  isnan(v) ? "descartada" : String(v, 2).c_str(), dirs[nDir - 1]);
  }
  speedOk = nSpeed > 0;
  speed = speedOk ? sum / nSpeed : 0;

  float spread = angularSpread(dirs, WIND_SAMPLES);
  dirStable = spread <= WD_MAX_SPREAD_DEG;
  while (!dirStable && nDir < WIND_SAMPLES + WD_EXTRA_SAMPLES) {
    delay(WD_EXTRA_INTERVAL_MS);
    dirs[nDir++] = WindDirection();
    spread = angularSpread(&dirs[nDir - WIND_SAMPLES], WIND_SAMPLES);
    dirStable = spread <= WD_MAX_SPREAD_DEG;
  }

  if (dirStable) {
    dir = circularMean(&dirs[nDir - WIND_SAMPLES], WIND_SAMPLES);
    Serial.printf("[WIND] Dirección estable (dispersión %.1f°): %.1f°\n", spread, dir);
    return;
  }

  // Nivel 2: rango ampliado al 25 %
  int idx[WIND_SAMPLES + WD_EXTRA_SAMPLES];
  int n = largestAngularCluster(dirs, nDir, WD_OSC_SPREAD_DEG, idx);
  // El grupo debe ser mayoría: 3 de 8 en un arco no definen un rumbo
  if (n >= WIND_SAMPLES && 2 * n > nDir) {
    float sel[WIND_SAMPLES + WD_EXTRA_SAMPLES];
    for (int k = 0; k < n; k++) sel[k] = dirs[idx[k]];
    dir = circularMean(sel, n);
    Serial.printf("[WIND] Dirección oscilante: %d/%d muestras dentro de %.0f° (dispersión %.1f°): %.1f°\n",
                  n, nDir, WD_OSC_SPREAD_DEG, angularSpread(sel, n), dir);
  } else {
    dir = circularMean(dirs, nDir);
    Serial.printf("[WIND] Dirección variable (%d muestras, dispersión >%.0f°): promedio vectorial %.1f°\n",
                  nDir, WD_OSC_SPREAD_DEG, dir);
  }
}

/* ====================== SIM800 FUNCTIONS ======================== */
// Recuerda si el modem quedó realmente encendido este ciclo, para no
// intentar un apagado AT (bloqueante hasta 10 s) cuando nunca se energizó
// (p.ej. al entrar a modo config en conexión WiFi).
bool modemPoweredOn = false;

// Fuerza RX2/TX2 a salida LOW (no solo alta impedancia) y bloquea ese
// estado con hold: dejarlos flotando (INPUT) o en HIGH (idle de UART)
// retroalimenta el SIM800 a través de los diodos de protección ESD de
// sus pines, energizándolo parcialmente (~2.7V medidos) incluso con el
// MOSFET de alimentación cortado. Debe llamarse siempre que el modem no
// esté en uso, incluido antes de cada deep sleep, sin importar el modo
// de conexión (WiFi nunca enciende el modem, así que sin esto estos
// pines quedarían en su estado flotante por defecto durante todo el sleep).
void modemLinesOff() {
  gpio_hold_dis((gpio_num_t)PIN_TX2);
  gpio_hold_dis((gpio_num_t)PIN_RX2);
  pinMode(PIN_TX2, OUTPUT);
  pinMode(PIN_RX2, OUTPUT);
  digitalWrite(PIN_TX2, LOW);
  digitalWrite(PIN_RX2, LOW);
  gpio_hold_en((gpio_num_t)PIN_TX2);
  gpio_hold_en((gpio_num_t)PIN_RX2);
}

// Enciende sim_control (modem + 12V RS485) con arranque suave: pulsos de
// ancho creciente (curva cuadrática, los primeros de ~2 µs) cargan los
// condensadores por escalones en vez de con un único pico de corriente.
// El MOSFET se apaga despacio (gate con pull-up de 100k), así que cada
// pulso conduce algo más que su ancho nominal; aun así el pico queda
// muy por debajo del encendido directo.
void modemRailOn() {
  gpio_hold_dis((gpio_num_t)MODEM_PWR_PIN);
#if MODEM_SOFTSTART_MS > 0
  const int steps = 40;
  ledcSetup(MODEM_SOFTSTART_CH, MODEM_SOFTSTART_FREQ, 10);
  ledcAttachPin(MODEM_PWR_PIN, MODEM_SOFTSTART_CH);
  for (int i = 1; i <= steps; i++) {
    ledcWrite(MODEM_SOFTSTART_CH, (uint32_t)(1023UL * i * i / (steps * steps)));
    delay(MODEM_SOFTSTART_MS / steps);
  }
  // Fijar el nivel del registro GPIO antes de soltar el LEDC: así no hay
  // un instante en LOW al volver a modo GPIO.
  digitalWrite(MODEM_PWR_PIN, MODEM_PWR_ON);
  ledcDetachPin(MODEM_PWR_PIN);
#endif
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, MODEM_PWR_ON);
  modemRailOnMs = millis();
}

void modemStepSet(const char *step) {
  strlcpy(modemStep, step, sizeof(modemStep));
  modemStepMagic = step[0] ? MODEM_STEP_MAGIC : 0;
}

#if MODEM_BROWNOUT_RIDE_THROUGH
static uint32_t savedBrownoutReg = 0;
#endif

void modemBrownoutRideThrough(bool enable) {
#if MODEM_BROWNOUT_RIDE_THROUGH
  if (enable) {
    if (!savedBrownoutReg) savedBrownoutReg = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG);
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  } else if (savedBrownoutReg) {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, savedBrownoutReg);
    savedBrownoutReg = 0;
  }
#endif
}

// Llamar tras modem.init(): fija la banda si MODEM_BAND no está vacío.
void modemApplyBand() {
  if (strlen(MODEM_BAND) == 0) return;
  modem.sendAT("+CBAND=\"" MODEM_BAND "\"");
  bool ok = modem.waitResponse(2000L) == 1;
  Serial.printf("[MODEM] Banda %s: %s\n", MODEM_BAND, ok ? "OK" : "rechazada");
}

/* ---------- Diagnóstico del modem (visible en el OLED) ---------- */
// En campo no hay monitor serie, así que lo que pasa al encender el SIM800
// se resume aquí y se muestra en una página del OLED cuando el envío LTE
// falla (y en la consulta de estado, vista "MODEM"). RTC_DATA_ATTR para que
// sobreviva al deep sleep y pueda verse con el botón "ok" más tarde.
static char mdiagLine[48];     // línea en construcción (para detectar URCs)
static uint8_t mdiagLineLen = 0;
static bool mdiagSawOK = false;
static bool mdiagSawReady = false;   // "SMS Ready": el SIM800 terminó de arrancar

// Batería rápida (4 muestras): para seguir la caída durante los bursts.
float vBatQuick() {
  return vBatRaw(4) * batCalFactor;
}

void mdiagTrackBat() {
  float v = vBatQuick();
  if (mdiag.vBatMin <= 0 || v < mdiag.vBatMin) mdiag.vBatMin = v;
}

void mdiagReset() {
  memset(&mdiag, 0, sizeof(mdiag));
  mdiag.rxLevel = mdiag.rxPre = mdiag.rxOn = mdiag.txIdle = -1;
  mdiagLineLen = 0;
  mdiagSawReady = false;
}

// Re-inicializa el SSD1306 sin tocar el bus: el OLED cuelga de v_sensor y
// la caída de tensión al energizar el modem puede resetearlo (queda negro
// o congelado aunque el ESP32 siga vivo). Reenviar la secuencia de init es
// inocuo si no se había reseteado.
void oledRecover() {
  if (!oledReady) return;
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("[OLED] Sin ACK tras encender el modem");
    return;
  }
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false);
  oled.setRotation(OLED_ROTATION);
  oled.setTextWrap(false);
}

// Paso de la secuencia del modem. code (p.ej. "M4 SYNC") y tech (valores
// técnicos) van solo al serie y a modemStep (RTC_NOINIT, guardado ANTES de
// ejecutar la acción: si el ESP32 se reinicia se recuerda el último paso).
// En el OLED solo se muestra msg, un texto simple para un usuario básico
// (<= 20 caracteres, palabras de <= 10).
void mdiagShow(const String &code, const String &msg, const String &tech = "") {
  Serial.printf("[MODEM] %s | %s | %s\n", code.c_str(), msg.c_str(), tech.c_str());
  modemStepSet(code.c_str());
  if (oledReady) {
    dashSetDetail(msg);
    dashDraw();
  }
}

// Espera del modem en el OLED: texto + segundos transcurridos.
void mdiagShowWait(const char *l0, const char *l1, uint32_t t0) {
  if (!oledReady) return;
  dash.detail[0] = l0;
  dash.detail[1] = String(l1) + " " + String((millis() - t0) / 1000) + "s";
  dashDraw();
}

// Procesa un byte recibido del modem: cuenta, guarda la cola imprimible y
// detecta líneas URC relevantes. Las de tensión tienen prioridad sobre las
// informativas (RDY/CPIN/Call Ready) y no se sobrescriben.
void mdiagFeed(char c) {
  if (mdiag.hexN < sizeof(mdiag.hex)) mdiag.hex[mdiag.hexN++] = (uint8_t)c;
  mdiag.rxBytes++;
  if (c == '\r' || c == '\n') {
    if (mdiagLineLen == 0) return;
    mdiagLine[mdiagLineLen] = '\0';
    mdiagLineLen = 0;
    Serial.printf("[MODEM<] %s\n", mdiagLine);
    if (strcmp(mdiagLine, "OK") == 0) { mdiagSawOK = true; return; }
    if (strstr(mdiagLine, "SMS Ready")) mdiagSawReady = true;
    bool volt = strstr(mdiagLine, "VOLT") || strstr(mdiagLine, "POWER DOWN");
    bool urcVolt = strstr(mdiag.urc, "VOLT") || strstr(mdiag.urc, "POWER DOWN");
    if (volt || (!urcVolt && strncmp(mdiagLine, "AT", 2) != 0))
      strlcpy(mdiag.urc, mdiagLine, sizeof(mdiag.urc));
    return;
  }
  if (c < 0x20 || c > 0x7E) { c = '?'; mdiag.badBytes++; }  // basura (baudrate distinto, ruido)
  if (mdiagLineLen < sizeof(mdiagLine) - 1) mdiagLine[mdiagLineLen++] = c;
  size_t n = strlen(mdiag.tail);
  if (n >= sizeof(mdiag.tail) - 1) {
    memmove(mdiag.tail, mdiag.tail + 1, n);
    n--;
  }
  mdiag.tail[n] = c;
  mdiag.tail[n + 1] = '\0';
}

// Primeros bytes crudos en hex ("FF 00 80"), para ver basura de baudrate/ruido.
String mdiagHexTxt() {
  String h;
  for (int i = 0; i < mdiag.hexN; i++) {
    char b[4];
    snprintf(b, sizeof(b), i ? " %02X" : "%02X", mdiag.hex[i]);
    h += b;
  }
  return h.length() ? h : "(nada)";
}

// Lee el UART del modem durante ms milisegundos, siguiendo la batería.
// live: refresca cada 500 ms las 2 líneas de detalle con la batería mínima,
// nivel de RX, bytes recibidos y segundos transcurridos.
void mdiagPump(uint32_t ms, bool live = false) {
  uint32_t t0 = millis(), lastBat = 0, lastDraw = 0;
  while (millis() - t0 < ms) {
    while (SerialAT.available()) mdiagFeed((char)SerialAT.read());
    if (millis() - lastBat >= 100) { mdiagTrackBat(); lastBat = millis(); }
    if (live && oledReady && millis() - lastDraw >= 500) {
      lastDraw = millis();
      dash.detail[0] = "Bmin " + String(mdiag.vBatMin, 2);
      dash.detail[1] = "RX" + String(digitalRead(PIN_RX2)) + " " + String(mdiag.rxBytes) + "B " +
                       String((millis() - t0) / 1000) + "s";
      dashDraw();
    }
    delay(5);
  }
}

// Envía "AT" hasta tries veces a la velocidad indicada; true si respondió OK.
bool mdiagProbe(const char *line, uint32_t baud, int tries) {
  Serial.printf("[MODEM] Probando AT a %lu baud\n", (unsigned long)baud);
  SerialAT.updateBaudRate(baud);
  delay(20);
  while (SerialAT.available()) mdiagFeed((char)SerialAT.read());
  mdiagSawOK = false;
  for (int i = 0; i < tries && !mdiagSawOK; i++) {
    mdiagShow(line, "Revisando modem",
              "intento " + String(i + 1) + "/" + String(tries) + " " + String(mdiag.rxBytes) + "B rx");
    SerialAT.print("AT\r\n");
    mdiagPump(600);
  }
  return mdiagSawOK;
}

// Prueba AT SOLO de diagnóstico, después de que modem.init() falló (la
// comunicación normal es la de PROD: init directo). 9600 y luego otros
// baudrates por si quedó fijado uno con AT+IPR; si responde a otro, lo
// devuelve a 9600. No se prueba TX/RX intercambiado: pondría la salida del
// ESP32 contra la salida TXD del SIM800.
String mdiagBaudTxt(uint32_t b) {
  if (!b) return "no";
  if (b % 1000 == 0) return String(b / 1000) + "k";
  return String(b / 1000.0f, 1) + "k";
}

bool mdiagTryBauds(const char *line, const uint32_t *bauds, int n, int tries) {
  for (int i = 0; i < n; i++) {
    String l = String(line) + " " + mdiagBaudTxt(bauds[i]);
    if (mdiagProbe(l.c_str(), bauds[i], tries)) { mdiag.atBaud = bauds[i]; return true; }
  }
  return false;
}

// Estabiliza el enlace antes de modem.init(), sin cortar la alimentación:
//   M4 SYNC    "AT" cada 500 ms hasta el primer OK (sincroniza el autobaud)
//   M5 ESPERA  deja terminar el arranque: hasta "SMS Ready" o MODEM_QUIET_MS
//              de silencio (máx. MODEM_READY_MS)
//   M6 ESTABLE MODEM_STABLE_OK respuestas "OK" seguidas sin bytes corruptos
// Devuelve false si nunca respondió (entonces sí conviene reiniciar).
bool modemStabilize() {
  // M4: sincronizar
  mdiagSawOK = false;
  uint32_t t0 = millis();
  int n = 0;
  while (!mdiagSawOK && millis() - t0 < MODEM_SYNC_MS) {
    Serial.printf("[MODEM] M4 SYNC AT #%d %lus %uB\n", ++n, (unsigned long)((millis() - t0) / 1000), mdiag.rxBytes);
    modemStepSet("M4 SYNC");
    mdiagShowWait("Conectando", "modem", t0);
    SerialAT.print("AT\r\n");
    mdiagPump(500);
  }
  if (!mdiagSawOK) {
    Serial.println("[MODEM] Sin OK durante la sincronización");
    return false;
  }
  mdiag.atBaud = UART_BAUD;

  // M5: esperar fin del arranque
  t0 = millis();
  uint32_t lastRx = millis();
  uint16_t prev = mdiag.rxBytes;
  while (!mdiagSawReady && millis() - t0 < MODEM_READY_MS) {
    mdiagPump(100);
    if (mdiag.rxBytes != prev) { prev = mdiag.rxBytes; lastRx = millis(); }
    if (millis() - lastRx >= MODEM_QUIET_MS && millis() - t0 >= MODEM_QUIET_MS) break;
    if ((millis() - t0) % 500 < 100) {
      modemStepSet("M5 ESPERA");
      mdiagShowWait("Preparando", "modem", t0);
    }
  }
  Serial.printf("[MODEM] Arranque terminado en %lu ms (%s)\n", (unsigned long)(millis() - t0),
                mdiagSawReady ? "SMS Ready" : "silencio/tiempo");

  // M6: OK limpios seguidos
  t0 = millis();
  int good = 0, total = 0;
  while (good < MODEM_STABLE_OK && millis() - t0 < MODEM_STABLE_MS) {
    while (SerialAT.available()) mdiagFeed((char)SerialAT.read());
    uint16_t badBefore = mdiag.badBytes;
    mdiagSawOK = false;
    SerialAT.print("AT\r\n");
    mdiagPump(400);
    total++;
    if (mdiagSawOK && mdiag.badBytes == badBefore) good++;
    else good = 0;
    mdiagShow("M6 ESTABLE", "Probando modem",
              "OK " + String(good) + "/" + String(MODEM_STABLE_OK) + " AT" + String(total) +
              " err" + String(mdiag.badBytes));
    delay(300);
  }
  Serial.printf("[MODEM] Enlace %s: %d OK seguidos en %d AT, %u bytes corruptos\n",
                good >= MODEM_STABLE_OK ? "estable" : "INESTABLE", good, total, mdiag.badBytes);
  // Vaciar lo que quede para que modem.init() empiece con la línea limpia
  mdiagPump(200);
  while (SerialAT.available()) SerialAT.read();
  return true;
}

// modem.init() con reintentos SIN cortar alimentación (vaciando la línea
// entre intentos). Devuelve true si alguno funcionó.
bool modemInitRetries() {
  for (int i = 1; i <= MODEM_INIT_TRIES; i++) {
    mdiagShow("M7 INIT " + String(i), "Activando modem", "err " + String(mdiag.badBytes));
    if (modem.init()) return true;
    Serial.printf("[MODEM] init %d/%d falló\n", i, MODEM_INIT_TRIES);
    mdiagPump(2000);
    while (SerialAT.available()) SerialAT.read();
  }
  return false;
}

void modemDiagProbeAT() {
  static const uint32_t first[] = {UART_BAUD};
  static const uint32_t others[] = {115200, 57600, 38400, 19200};
  mdiag.atBaud = 0;
  bool ok = mdiagTryBauds("D1", first, 1, 3) || mdiagTryBauds("D2", others, 4, 2);
  if (ok && mdiag.atBaud != UART_BAUD) {
    Serial.printf("[MODEM] Responde a %lu: se fija 9600 (AT+IPR=9600;&W)\n", (unsigned long)mdiag.atBaud);
    SerialAT.print("AT+IPR=9600;&W\r\n");
    mdiagPump(500);
  }
  SerialAT.updateBaudRate(UART_BAUD);
  delay(50);
  if (ok) {
    mdiagShow("D AT OK", "Revisando modem", mdiagBaudTxt(mdiag.atBaud) + " " + String(mdiag.rxBytes) + "B rx");
  } else {
    Serial.println("[MODEM] Sin respuesta AT a ningún baudrate");
    mdiagShow("D AT NO", "Modem no responde", String(mdiag.rxBytes) + "B rx " + mdiagHexTxt());
  }
}

void mdiagTrackVsim(int16_t mv) {
  if (mv > mdiag.vSimMax) mdiag.vSimMax = mv;
}


// Causa probable a partir del diagnóstico (texto corto para lastNetStatus).
const char *modemDiagCause() {
  if (strstr(mdiag.urc, "OVER") || mdiag.vSimMax >= SIM800_VMAX_MV) return "Modem sobrevoltaje";
  if (strstr(mdiag.urc, "UNDER")) return "Modem bajo voltaje";
  if (strstr(mdiag.urc, "POWER DOWN")) return "Modem se apago";
  if (mdiag.rxPre == 1) return "RX alto sin modem";      // pull-up/retroalimentación
  if (mdiag.rxBytes == 0 && mdiag.rxLevel == 0) return "Modem sin energia";
  if (mdiag.txIdle == 0) return "TX ESP32 en corto/GND";
  // Modem energizado (RX alto, LED de red parpadeando) pero ni un "OK" a
  // ningún baudrate ni orientación: con autobaud el SIM800 no emite nada
  // hasta sincronizar con un "AT", así que lo más probable es que no le
  // llegue el TX del ESP32 (pista GPIO17 -> RXD del SIM800, soldadura,
  // divisor/level shifter) o que su RXD esté dañado.
  if (mdiag.atBaud == 0 && mdiag.rxBytes <= 4) return "Modem no recibe TX";
  if (mdiag.atBaud == 0) return "Modem basura en RX";
  return "Modem no responde";
}

// Registro del diagnóstico en la SD (/modem_diag.txt) para revisarlo luego.
void modemDiagLogSD() {
  if (!sdReady) return;
  File f = SD.open("/modem_diag.txt", FILE_APPEND);
  if (!f) return;
  f.printf("%s paso=%s causa=%s vb0=%.2f vbmin=%.2f vmod=%d rx=%d>%d>%d tx=%d bytes=%u err=%u vsimmax=%d hex=%s at=%lu int=%u rst=%s urc=\"%s\" tail=\"%s\"\n",
           formatTimestamp(getEpochNow()).c_str(), mdiag.step, lastNetStatus,
           mdiag.vBat0, mdiag.vBatMin, lastModemMv, mdiag.rxPre, mdiag.rxOn, mdiag.rxLevel,
           mdiag.txIdle, mdiag.rxBytes, mdiag.badBytes, mdiag.vSimMax, mdiagHexTxt().c_str(), (unsigned long)mdiag.atBaud,
           mdiag.attempts, rstModemStep.c_str(), mdiag.urc, mdiag.tail);
  f.close();
}

void drawModemDiagPage(const String &footer) {
  String at = mdiagBaudTxt(mdiag.atBaud);
  // RX: apagado > recién energizado > tras 3 s  (1 = alto, 0 = bajo, - = sin dato)
  auto lv = [](int8_t l) { return l < 0 ? String("-") : String(l); };
  String rx = lv(mdiag.rxPre) + ">" + lv(mdiag.rxOn) + ">" + lv(mdiag.rxLevel);
  // Texto legible si lo hubo; si solo llegó basura, los bytes en hex
  bool legible = false;
  for (const char *c = mdiag.tail; *c; c++) if (isalnum((unsigned char)*c)) legible = true;
  String msg = mdiag.urc[0] ? String(mdiag.urc) : legible ? String(mdiag.tail) : mdiagHexTxt();
  String k[] = {"Paso", "Bat0", "Bmin", "Vsim", "RX", "TX/B", "AT", "Rst", "Msg"};
  String v[] = {mdiag.step[0] ? String(mdiag.step) : "--",
                mdiag.vBat0 > 0 ? String(mdiag.vBat0, 2) : "--",
                mdiag.vBatMin > 0 ? String(mdiag.vBatMin, 2) : "--",
                mdiag.vSimMax > 0 ? String(mdiag.vSimMax / 1000.0f, 2) + (mdiag.vSimMax >= SIM800_VMAX_MV ? "!" : "")
                                  : lastModemMv > 0 ? String(lastModemMv / 1000.0f, 2) : "--",
                rx, (mdiag.txIdle < 0 ? String("-") : String(mdiag.txIdle)) + "/" + String(mdiag.rxBytes),
                at + "/" + String(mdiag.attempts),
                rstModemStep.length() ? rstModemStep : "-", msg};
  oledInfoPage("MODEM", k, v, 9, footer);
}

// Fallo del envío LTE: guarda el paso, registra en SD y deja en pantalla
// hasta 20 s la página amigable (qué pasó / qué hacer). UP alterna con el
// detalle técnico; OK la cierra antes.
void modemDiagFail(const char *step) {
  strlcpy(mdiag.step, step, sizeof(mdiag.step));
  modemDiagLogSD();
  dash.modem = netStatusShort(lastNetStatus).substring(0, 10);
  modemStepSet("");
  if (!oledReady) return;
  buttonsInit();
  bool tech = false;
  drawModemFriendlyPage("UP:detalle");
  uint32_t t0 = millis();
  while (millis() - t0 < 20000) {
    if (pollButton(btnOk) != BTN_NONE) break;
    if (pollButton(btnUp) != BTN_NONE) {
      tech = !tech;
      t0 = millis();  // más tiempo para leer
      if (tech) drawModemDiagPage("OK:seguir");
      else drawModemFriendlyPage("UP:detalle");
    }
    delay(20);
  }
}

// Encendido en dos fases para poder aprovechar los MODEM_BOOT_MS de arranque
// (UART cerrado) en otra cosa, p.ej. leer el sensor RS485 que comparte riel:
//   modemPowerOnBegin()  riel ON + mediciones pasivas (vuelve enseguida)
//   modemPowerOnFinish() completa MODEM_BOOT_MS contados desde el riel ON y
//                        abre el UART. Misma secuencia eléctrica que antes.
void modemPowerOnBegin() {
  Serial.println("[MODEM] Encendiendo SIM800...");
  modemStepSet("Encendido");
  mdiag.attempts++;
  mdiagSawReady = false;  // cada encendido vuelve a esperar su "SMS Ready"
  mdiag.vBat0 = vBatQuick();
  mdiagTrackBat();
  mdiagShow("M1 BATERIA", "Revisando bateria",
            "Vb " + String(mdiag.vBat0, 2) + "V intento " + String(mdiag.attempts));
  modemBrownoutRideThrough(true);

  // Liberar el hold LOW de RX2/TX2 antes de que begin() los reconfigure
  // como pines UART, si no, el periférico no puede tomar control de ellos.
  // RX2 pasa a entrada ya: durante los 3 s de arranque el TX del SIM800
  // queda en reposo (HIGH) y no debe pelear contra un LOW forzado.
  gpio_hold_dis((gpio_num_t)PIN_TX2);
  gpio_hold_dis((gpio_num_t)PIN_RX2);
  // Nivel de RX con el modem APAGADO y pull-down interno (~45k): si aun así
  // lee 1, algo externo lo sube (pull-up del módulo alimentado por otro lado
  // o retroalimentación), y "RX alto" después no prueba que el modem viva.
  pinMode(PIN_RX2, INPUT_PULLDOWN);
  delay(5);
  mdiag.rxPre = digitalRead(PIN_RX2);

  modemPoweredOn = true;
  mdiagShow("M2 RIEL ON", "Prendiendo modem", "RX apag " + String(mdiag.rxPre));
  modemRailOn();
  delay(150);
  mdiag.rxOn = digitalRead(PIN_RX2);
  pinMode(PIN_RX2, INPUT);
  oledRecover();
}

void modemPowerOnFinish() {
  // Como PROD (a603f7a), pero MODEM_BOOT_MS: arranque con el UART CERRADO y recién
  // entonces SerialAT.begin(). Abrirlo antes captura basura/URC de arranque
  // que luego se mezclan con las respuestas de modem.init(). Durante la
  // espera solo se mide (batería, nivel de RX): nada se envía al modem.
  // Se cuenta desde el riel ON: si entretanto se leyó el RS485, solo se
  // espera lo que falta.
  uint32_t t0 = modemRailOnMs;
  while (millis() - t0 < MODEM_BOOT_MS) {
    mdiagTrackBat();
    modemStepSet("M3 ARRANQ");
    mdiagShowWait("Arrancando", "modem", t0);
    delay(250);
  }
  mdiag.rxLevel = digitalRead(PIN_RX2);  // idle HIGH = TX del modem energizado
  oledRecover();

  // La sesión con el modem corre a 240 MHz como PROD (a603f7a); el resto del
  // ciclo (sensores, SD) va a 80 MHz. Vaciar el log antes de cambiar.
  if (getCpuFrequencyMhz() != 240) {
    Serial.flush();
    setCpuFrequencyMhz(240);
  }

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
  // Pad TX en reposo (el UART lo deja en HIGH): 0 = línea cargada/en corto
  PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[PIN_TX2]);
  mdiag.txIdle = gpio_get_level((gpio_num_t)PIN_TX2);
  modemPoweredOn = true;

  Serial.printf("[MODEM] SIM800 encendido: Vbat %.2f->%.2f V, RX %d>%d>%d, TX %d\n",
                mdiag.vBat0, mdiag.vBatMin, mdiag.rxPre, mdiag.rxOn, mdiag.rxLevel, mdiag.txIdle);
  mdiagShow(mdiag.rxLevel ? "M3 RX ALTO" : "M3 RX BAJO",
            mdiag.rxLevel ? "Modem encendido" : "Modem sin energia?",
            "Bmin " + String(mdiag.vBatMin, 2) + " RX " + String(mdiag.rxPre) + ">" + String(mdiag.rxOn) +
            ">" + String(mdiag.rxLevel) + " TX" + String(mdiag.txIdle));
}

void modemPowerOn() {
  modemPowerOnBegin();
  modemPowerOnFinish();
}

// Inicio de la sesión LTE del ciclo: limpia el diagnóstico y enciende el riel
// (fase 1). Lo llama sendDataLTE().
void modemSessionBegin() {
  dashStep("MODEM");
  lastSignal[0] = '\0';
  mdiagReset();
  lastModemMv = -1;
  modemPowerOnBegin();
}

void modemPowerOff() {
  Serial.println("[MODEM] Apagando SIM800...");

  if (modemPoweredOn) {
    // Apagado ordenado (AT+CPOWD=1) antes de cortar la alimentación: si
    // se corta el MOSFET de golpe mientras el modem sigue registrado en
    // la red, puede quedar en un estado intermedio que sigue consumiendo
    // corriente (LED de red parpadeando) durante todo el deep sleep del
    // ESP32.
    modem.poweroff();
    delay(300);
  }

  SerialAT.end();
  modemLinesOff();

  gpio_hold_dis((gpio_num_t)MODEM_PWR_PIN);
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, MODEM_PWR_OFF);
  gpio_hold_en((gpio_num_t)MODEM_PWR_PIN);

  modemPoweredOn = false;
  modemBrownoutRideThrough(false);
  modemStepSet("");

  Serial.println("[MODEM] SIM800 apagado");
}

void modemRestart() {
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}

/* ====================== RS485 / MODBUS RTU ======================== */
// El esquemático nombra las líneas "TXD"/"RXD" sin dejar claro si es desde
// el lado del ESP32 o del módulo. Si el sensor no responde con la
// orientación actual se prueba la contraria, y se recuerda la que funcionó
// (memoria RTC) para los ciclos siguientes.
RTC_DATA_ATTR bool rs485Swapped = false;
volatile bool rs485Busy = false;  // /lognow (hilo async) vs updateSensCache() (loop)

struct Rs485Data {
  float soilT = 0, soilH = 0, ph = 0;
  int ec = 0;
  int rad = 0;
  float level = 0;  // ACLT600, crudo promediado
};

// CRC-16/MODBUS (polinomio 0xA001, inicial 0xFFFF). En la trama va el byte
// bajo primero.
uint16_t modbusCRC16(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

// Función 0x03 (Read Holding Registers). Valida dirección, función, número
// de bytes y CRC de la respuesta antes de aceptarla.
bool modbusReadRegs(uint32_t baud, uint8_t addr, uint16_t reg, uint16_t count, uint16_t *out) {
  if (count == 0 || count > 8) return false;

  uint8_t req[8] = {addr, 0x03, (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
                    (uint8_t)(count >> 8), (uint8_t)(count & 0xFF), 0, 0};
  uint16_t crc = modbusCRC16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  const size_t respLen = 5 + 2 * count;
  uint8_t resp[5 + 2 * 8];

  for (int attempt = 0; attempt < 2 * RS485_RETRIES; attempt++) {
    if (attempt == RS485_RETRIES) {
      rs485Swapped = !rs485Swapped;
      Serial.printf("[RS485] Sin respuesta, probando TX/RX invertidos (TX=%d RX=%d)\n",
                    rs485Swapped ? RS485_RX_PIN : RS485_TX_PIN,
                    rs485Swapped ? RS485_TX_PIN : RS485_RX_PIN);
    }
    int txPin = rs485Swapped ? RS485_RX_PIN : RS485_TX_PIN;
    int rxPin = rs485Swapped ? RS485_TX_PIN : RS485_RX_PIN;

    Serial1.begin(baud, SERIAL_8N1, rxPin, txPin);
    while (Serial1.available()) Serial1.read();
    Serial1.write(req, sizeof(req));
    Serial1.flush();

    // Se descarta lo que llegue antes de la dirección del esclavo (basura
    // del arranque del transceptor).
    size_t n = 0;
    unsigned long t0 = millis();
    while (n < respLen && millis() - t0 < RS485_TIMEOUT_MS) {
      if (!Serial1.available()) { delay(1); continue; }
      uint8_t b = Serial1.read();
      if (n == 0 && b != addr) continue;
      resp[n++] = b;
    }
    Serial1.end();
    // Sin UART, pines en alta impedancia: no alimentar el módulo por sus
    // entradas cuando se corte v_sensor.
    pinMode(RS485_TX_PIN, INPUT);
    pinMode(RS485_RX_PIN, INPUT);

    if (n == respLen && resp[1] == 0x03 && resp[2] == 2 * count &&
        modbusCRC16(resp, respLen - 2) == (uint16_t)(resp[respLen - 2] | (resp[respLen - 1] << 8))) {
      for (uint16_t i = 0; i < count; i++) {
        out[i] = (uint16_t)(resp[3 + 2 * i] << 8) | resp[4 + 2 * i];
      }
      return true;
    }
    Serial.printf("[RS485] Intento %d fallido (%u/%u bytes)\n", attempt + 1, (unsigned)n, (unsigned)respLen);
    delay(100);
  }
  return false;
}

// 12V del sensor RS485 por GPIO14, independiente del modem: leer el sensor
// ya no enciende el SIM800.
void rs485RailOn() {
  gpio_hold_dis((gpio_num_t)RS485_PWR_PIN);
  pinMode(RS485_PWR_PIN, OUTPUT);
  digitalWrite(RS485_PWR_PIN, RS485_PWR_ON);
  rs485RailOnMs = millis();
}

// Apaga el 12V y retiene el nivel (deep sleep incluido).
void rs485RailOff() {
  gpio_hold_dis((gpio_num_t)RS485_PWR_PIN);
  pinMode(RS485_PWR_PIN, OUTPUT);
  digitalWrite(RS485_PWR_PIN, RS485_PWR_OFF);
  gpio_hold_en((gpio_num_t)RS485_PWR_PIN);
  rs485RailOnMs = 0;
}

// Una lectura del registro de nivel del ACLT600 (int16 con signo).
bool readLevelOnce(int &raw) {
  uint16_t r;
  if (!modbusReadRegs(9600, RS485_SLAVE_ADDR, LEVEL_REG, 1, &r)) return false;
  raw = (int16_t)r;
  return true;
}

// Procedimiento del proyecto levelSIM7000 (probado con el ACLT600 real):
// lecturas sucesivas hasta que LEVEL_STABLE_WINDOW seguidas varíen como mucho
// LEVEL_STABLE_THRESHOLD (o se agote LEVEL_STABLE_MAX_TRIES), y luego el
// promedio de LEVEL_AVG_SAMPLES lecturas finales. Diferencia: si la primera
// lectura falla (sensor ausente) o fallan LEVEL_MAX_FAILS seguidas se aborta
// enseguida — cada fallo ya cuesta varios reintentos + inversión TX/RX, y
// seguir insistiendo dejaría el equipo despierto más de un minuto.
// live (modo config): sin la fase de estabilidad, solo el promedio final.
bool readLevel(float &avg, bool live) {
  int window[LEVEL_STABLE_WINDOW];
  int got = 0, fails = 0;
  bool stable = live;

  for (int i = 0; i < LEVEL_STABLE_MAX_TRIES && !stable; i++) {
    if (i > 0) delay(LEVEL_SAMPLE_DELAY_MS);
    int raw;
    if (!readLevelOnce(raw)) {
      if (got == 0 || ++fails >= LEVEL_MAX_FAILS) return false;
      continue;
    }
    fails = 0;
    window[got % LEVEL_STABLE_WINDOW] = raw;
    got++;
    Serial.printf("[RS485] Nivel crudo (estabilizando %d): %d\n", i + 1, raw);
    if (got >= LEVEL_STABLE_WINDOW) {
      int mn = window[0], mx = window[0];
      for (int k = 1; k < LEVEL_STABLE_WINDOW; k++) {
        mn = min(mn, window[k]);
        mx = max(mx, window[k]);
      }
      stable = (mx - mn) <= LEVEL_STABLE_THRESHOLD;
    }
  }
  if (!stable) Serial.println("[RS485] Nivel sin estabilizar en el tope de intentos, se promedia igual");

  long sum = 0;
  int n = 0;
  int samples = live ? RS485_SAMPLES : LEVEL_AVG_SAMPLES;
  for (int i = 0; i < samples; i++) {
    delay(LEVEL_SAMPLE_DELAY_MS);
    int raw;
    if (readLevelOnce(raw)) {
      sum += raw;
      n++;
    } else if (n == 0 && got == 0) {
      return false;  // modo live: sensor ausente
    }
  }
  if (n == 0) return false;
  avg = (float)sum / n;
  Serial.printf("[RS485] Nivel promedio: %.1f (crudo, %d muestras)\n", avg, n);
  return true;
}

// Lee el sensor elegido en rs485Type. Requiere v_sensor encendido (módulo
// TTL-RS485). live (modo config, cada 30 s): el 12V queda encendido entre
// lecturas para no repetir el calentamiento (se corta al dormir/reiniciar),
// y el nivel se lee sin la fase de estabilización.
bool readRS485(Rs485Data &d, bool live) {
  if (rs485Type == "NONE" || rs485Busy) return false;
  rs485Busy = true;

  // En modo config el 12V puede seguir encendido de la lectura anterior.
  if (!rs485RailOnMs) rs485RailOn();
  // Elevador 12V + arranque del sensor (el ACLT600 tarda ~10 s)
  if (rs485Type == "LEVEL") {
    if (!live) oledLog("nivel: arranque 10s");
    waitSince(rs485RailOnMs, LEVEL_WARMUP_MS);
  } else {
    waitSince(rs485RailOnMs, RS485_WARMUP_MS);
  }

  uint16_t r[4];
  bool ok = false;
  if (rs485Type == "SOIL") {
    // RS485_SAMPLES lecturas (300 ms entre ellas), se promedian las válidas
    float sH = 0, sT = 0, sPh = 0;
    uint32_t sEc = 0;
    int n = 0;
    for (int i = 0; i < RS485_SAMPLES; i++) {
      if (i > 0) delay(300);
      if (modbusReadRegs(9600, RS485_SLAVE_ADDR, 0x0000, 4, r)) {
        sH  += r[0] / 10.0f;
        sT  += (int16_t)r[1] / 10.0f;  // con signo: bajo cero en complemento a 2
        sEc += r[2];
        sPh += r[3] / 10.0f;
        n++;
        Serial.printf("[RS485] Suelo muestra %d: T=%.1f H=%.1f EC=%u pH=%.1f\n",
                      n, (int16_t)r[1] / 10.0f, r[0] / 10.0f, r[2], r[3] / 10.0f);
      } else if (n == 0) {
        break;  // ya agotó sus reintentos y la inversión TX/RX: sensor ausente
      }
    }
    ok = n > 0;
    if (ok) {
      d.soilH = sH / n;
      d.soilT = sT / n;
      d.ec    = (int)((sEc + n / 2) / n);
      d.ph    = sPh / n;
      Serial.printf("[RS485] Suelo promedio (%d/%d): T=%.1f°C H=%.1f%% EC=%d uS/cm pH=%.1f\n",
                    n, RS485_SAMPLES, d.soilT, d.soilH, d.ec, d.ph);
    }
  } else if (rs485Type == "RAD") {
    // RS485_SAMPLES lecturas (300 ms entre ellas), se promedian las válidas
    uint32_t sum = 0;
    int n = 0;
    for (int i = 0; i < RS485_SAMPLES; i++) {
      if (i > 0) delay(300);
      if (modbusReadRegs(4800, RS485_SLAVE_ADDR, 0x0000, 1, r)) {
        sum += r[0];
        n++;
        Serial.printf("[RS485] Radiación muestra %d: %u W/m2\n", n, r[0]);
      } else if (n == 0) {
        break;  // ya agotó sus reintentos y la inversión TX/RX: sensor ausente
      }
    }
    ok = n > 0;
    if (ok) {
      d.rad = (int)((sum + n / 2) / n);
      Serial.printf("[RS485] Radiación promedio (%d/%d): %d W/m2\n", n, RS485_SAMPLES, d.rad);
    }
  } else if (rs485Type == "LEVEL") {
    ok = readLevel(d.level, live);
  }
  if (!ok) Serial.println("[RS485] Sensor sin respuesta");

  if (!live) rs485RailOff();
  rs485Busy = false;
  return ok;
}

/* ====================== MQTT CALLBACK ======================== */
// Permite a waitForConfig() salir en cuanto llega la config, en vez de
// esperar siempre los 5 s completos con el modem/WiFi encendidos.
bool configReceivedFlag = false;

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.printf("[CONFIG] Topic recibido: %s\n", topic);

  // Extract MAC from end of topic: jhpAI/HIDROMET/settings/AA:BB:CC:DD:EE:FF
  String topicStr = String(topic);
  String macInTopic = topicStr.substring(topicStr.lastIndexOf('/') + 1);
  String deviceMac = macA();
  macInTopic.toUpperCase();
  deviceMac.toUpperCase();

  if (macInTopic != deviceMac) {
    Serial.printf("[CONFIG] MAC no coincide: topic=%s device=%s\n",
                  macInTopic.c_str(), deviceMac.c_str());
    return;
  }

  String msg_in = "";
  for (unsigned int i = 0; i < length; i++) msg_in += (char)payload[i];
  Serial.println("[CONFIG] Mensaje: " + msg_in);

  StaticJsonDocument<256> docIn;
  DeserializationError error = deserializeJson(docIn, msg_in);
  if (error) {
    Serial.printf("[CONFIG] JSON error: %s\n", error.c_str());
    return;
  }

  if (docIn.containsKey("status"))    status        = docIn["status"].as<bool>();
  if (docIn.containsKey("timesleep")) TIME_TO_SLEEP = (uint64_t)docIn["timesleep"].as<int>() * 60; // min → seg
  if (docIn.containsKey("A_TH"))      A_TH          = docIn["A_TH"].as<bool>();
  if (docIn.containsKey("A_WS"))      A_WS          = docIn["A_WS"].as<bool>();
  if (docIn.containsKey("A_PR"))      A_PR          = docIn["A_PR"].as<bool>();
  if (docIn.containsKey("dirSensor")) dirSensorType = docIn["dirSensor"].as<String>() == "AS5600" ? "AS5600" : "MT6701";
  if (docIn.containsKey("oled"))      oledOpEnabled = docIn["oled"].as<bool>();
  if (docIn.containsKey("rs485")) {
    String v = docIn["rs485"].as<String>();
    rs485Type = rs485Norm(v);
  }

  saveSetting(status ? "1" : "0",         "/status.txt");
  saveSetting(String((int)TIME_TO_SLEEP), "/timesleep.txt");
  saveSetting(A_TH ? "1" : "0",          "/A_TH.txt");
  saveSetting(A_WS ? "1" : "0",          "/A_WS.txt");
  saveSetting(A_PR ? "1" : "0",          "/A_PR.txt");
  saveSetting(dirSensorType,             "/dirSensor.txt");
  saveSetting(oledOpEnabled ? "1" : "0", "/oledOp.txt");
  saveSetting(rs485Type,                 "/rs485.txt");

  Serial.printf("[CONFIG] OK status=%d sleep=%llu seg (%llu min) TH=%d WS=%d PR=%d dirSensor=%s rs485=%s\n",
                status, TIME_TO_SLEEP, TIME_TO_SLEEP / 60, A_TH, A_WS, A_PR, dirSensorType.c_str(), rs485Type.c_str());

  configReceivedFlag = true;
}

/* ====================== MQTT RECONNECT ======================== */
void wifiReconnect() {
  int count = 0;
  while (!wificlient.connected()) {
    String client_id = "esp32-client-" + macA();

    if (wificlient.connect(client_id.c_str())) {
      String topiSub = String(topicSubscribe) + "/" + macA();
      wificlient.subscribe(topiSub.c_str());
      wificlient.setCallback(callback);
      Serial.println("[MQTT] Conectado via WiFi");
    } else {
      Serial.print("[MQTT] Error: ");
      Serial.println(wificlient.state());
      if (count >= 5) {
        ESP.restart();
      }
      delay(5000);
    }
    count++;
  }
}

void simReconnect() {
  int count = 0;
  while (!simclient.connected()) {
    String client_id = "esp32-client-" + macA();

    if (simclient.connect(client_id.c_str())) {
      String topiSub = String(topicSubscribe) + "/" + macA();
      simclient.subscribe(topiSub.c_str());
      simclient.setCallback(callback);
      Serial.println("[MQTT] Conectado via LTE");
    } else {
      Serial.print("[MQTT] Error: ");
      Serial.println(simclient.state());
      if (count >= 3) {
        ESP.restart();
      }
      delay(5000);
    }
    count++;
  }
}

bool checkMQTT() {
  if (connectionType == "WIFI") {
    return wificlient.connected();
  } else if (connectionType == "LTE") {
    return simclient.connected();
  }
  return false;
}

/* ========================= WAKE UP REASON ========================= */

void wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("[WAKEUP] Botón ok (EXT0) - consulta de estado");
      runStatusViewer();  // No retorna, entra en deep sleep
      break;

    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("[WAKEUP] Rain counter (EXT1)");
      // Actualizar tiempo transcurrido: tiempo dormido = tiempo configurado - restante
      // Como despertó por lluvia, el tiempo dormido es aproximadamente lo que faltaba
      // Esto se maneja dentro de runRainCounter()
      runRainCounter();  // No retorna, entra en deep sleep
      break;

    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("[WAKEUP] Timer - Ciclo completado");
      init_running();
      break;

    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("[WAKEUP] Touchpad");
      break;

    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("[WAKEUP] ULP");
      break;

    default:
      Serial.printf("[WAKEUP] Inicio normal (razón: %d)\n", wakeup_reason);
      init_running();
      break;
  }
}

/* ====================== LEER SENSORES ======================== */
String readSensors() {
  Serial.println("\n[SENSORS] Iniciando lectura de sensores...");

  // Encender sensores: MOSFET canal P gate LOW = ON. El calentamiento del
  // AM2301 (DHT_WARMUP_MS) se cuenta desde aquí (o desde antes, si el OLED
  // ya encendió el riel) y se solapa con la medición de viento, en vez de
  // esperarlo sin hacer nada.
  sensorsRailOn();
  const uint32_t railMs = sensRailOnMs;
  waitSince(railMs, 100);  // encoder / DS3231 / sensor Hall

  // Iniciar I2C una sola vez — lo usan el MT6701/AS5600 (si A_WS) y siempre
  // el DS3231 (marca de tiempo), así que ya no se condiciona a A_WS.
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);
  initDS3231();

  // Con los campos RS485 el JSON ensamblado ronda ~400 bytes de pool
  DynamicJsonDocument docOut(640);
  docOut["type"] = "weatherStationData";
  docOut["mac"] = macA();
  docOut["conn"] = connectionType;
  docOut["BO"] = readFile(SPIFFS, "/brownouts.txt").toInt();  // brownouts acumulados (diagnóstico de alimentación)
  docOut["timestamp"] = formatTimestampISO8601(getEpochNow());

  // ===== VIENTO: se mide primero, mientras el AM2301 se calienta =====
  float wv = 0, wd = 0;
  bool wvOk = false, wdStable = false;
  if (A_TH) dht.begin();
  if (A_WS) {
    Serial.println("[SENSOR] Leyendo viento...");
    oledLog("viento " + String(WIND_SAMPLES) + "x3s");
    initMT6701();
    readWindAveraged(wv, wvOk, wd, wdStable);  // ya calibrada (windCal)
  }

  // ===== TEMPERATURA Y HUMEDAD (DHT22) =====
  if (A_TH) {
    Serial.println("[SENSOR] Leyendo DHT22...");
    // Mismos tiempos que initDHT22() (2.5 s de arranque frío + 2 s, lectura
    // de descarte, 2 s), pero contados desde el encendido del riel: con el
    // viento activado la espera ya transcurrió.
    waitSince(railMs, DHT_WARMUP_MS);
    dht.readTemperature();  // la primera tras power-on trae valores de arranque
    dht.readHumidity();
    delay(2000);

    // TH_SAMPLES lecturas válidas promediadas (máx. el doble de intentos),
    // cada TH_INTERVAL_MS con lectura forzada.
    float temp = 0, hum = 0;
    int nValid = 0;
    for (int attempt = 0; attempt < 2 * TH_SAMPLES && nValid < TH_SAMPLES; attempt++) {
      if (attempt > 0) delay(TH_INTERVAL_MS);
      float t, h;
      bool okOne;
      readDHT22(t, h, okOne, true);
      if (okOne) {
        temp += t;
        hum += h;
        nValid++;
        Serial.printf("[DHT22] Muestra %d: T=%.2f H=%.2f\n", nValid, t, h);
        if (oledReady) oledLog("T/H " + String(nValid) + "/" + String(TH_SAMPLES));
      }
    }
    bool ok = nValid > 0;
    if (ok) {
      temp /= nValid;
      hum /= nValid;
      if (nValid < TH_SAMPLES) Serial.printf("[DHT22] Solo %d de %d lecturas válidas\n", nValid, TH_SAMPLES);
    }

    if (ok) {
      float tempCal = 0.5480f * temp + 9.0357f;
      float humCal  = 0.4499f * hum + 54.7577f;
      docOut["T"] = String(tempCal, 2);
      docOut["H"] = String(humCal, 2);
      Serial.printf("[SENSOR] Temp: %.2f°C, Hum: %.2f%%\n", tempCal, humCal);
      dash.v[DV_T] = String(tempCal, 1) + "C";
      dashSet(DV_H, String(humCal, 0) + "%");
    } else {
      docOut["T"] = "error";
      docOut["H"] = "error";
      Serial.println("[SENSOR] Error leyendo DHT22");
      dash.v[DV_T] = "ERR";
      dashSet(DV_H, "ERR");
    }
  }

  // ===== VELOCIDAD Y DIRECCIÓN DEL VIENTO (medidas arriba) =====
  if (A_WS) {
    float wvCal = wv;

    docOut["WV"] = wvOk ? String(wvCal, 2) : "error";
    docOut["WD"] = String(wd, 1);
    Serial.printf("[SENSOR] Vel: %.2f m/s, Dir: %.1f°\n", wvCal, wd);
    dash.v[DV_WV] = wvOk ? String(wvCal, 1) + "m/s" : "ERR";
    dashSet(DV_WD, dashFmtDir(String(wd, 1)));
  }

  // ===== PRECIPITACIÓN =====
  if (A_PR) {
    Serial.println("[SENSOR] Leyendo precipitación...");

    // Se reporta como número entero de vaciadas del balancín (no en mm):
    // dato persistente en SPIFFS, incrementado en runRainCounter() cada
    // vez que el deep sleep se interrumpe por un pulso de lluvia (EXT1).
    String rainStr = readFile(SPIFFS, "/ssidRainCounter.txt");
    int rain = rainStr != "" ? rainStr.toInt() : 0;

    docOut["RA"] = rain;
    Serial.printf("[SENSOR] Lluvia: %d vaciadas\n", rain);
    dashSet(DV_RA, String(rain));
    // No se resetea aquí: es un contador acumulado persistente, solo se
    // reinicia manualmente desde la vista de configuración (botón
    // "Reiniciar Contador de Lluvia" -> /get?resetRain=1).
  }

  // ===== SENSOR RS485 (suelo o radiación solar) =====
  if (rs485Type != "NONE") {
    Serial.printf("[SENSOR] Leyendo RS485 (%s)...\n", rs485Type.c_str());
    oledLog("RS485...");
    Rs485Data rs;
    bool ok = readRS485(rs, false);
    if (rs485Type == "SOIL") {
      docOut["ST"] = ok ? String(rs.soilT, 1) : "error";
      docOut["SH"] = ok ? String(rs.soilH, 1) : "error";
      docOut["EC"] = ok ? String(rs.ec) : "error";
      docOut["PH"] = ok ? String(rs.ph, 1) : "error";
      dash.v[DV_ST] = ok ? String(rs.soilT, 1) + "C" : "ERR";
      dash.v[DV_SH] = ok ? String(rs.soilH, 0) + "%" : "ERR";
      dash.v[DV_EC] = ok ? String(rs.ec) + "uS" : "ERR";
      dashSet(DV_PH, ok ? String(rs.ph, 1) : "ERR");
    } else if (rs485Type == "LEVEL") {
      docOut["LV"] = ok ? String(rs.level, 1) : "error";  // crudo promediado, sin escala
      dashSet(DV_LV, ok ? String(rs.level, 0) : "ERR");
    } else {
      docOut["RAD"] = ok ? String(rs.rad) : "error";
      dashSet(DV_RAD, ok ? String(rs.rad) + "W" : "ERR");
    }
    if (!ok) oledLog("RS485 sin respuesta");
  }

  // Apagar sensores: MOSFET canal P gate HIGH = OFF. Si el OLED está
  // mostrando el ciclo, v_sensor (que también lo alimenta) sigue encendido
  // hasta deepSleepSystem().
  if (!oledReady) sensorsRailOff();

  // ===== BATERÍA =====
  // En operación se usa la medida en reposo de init_running() (tomada antes
  // de encender el modem, que la hundiría); en el portal (/lognow), una nueva.
  String bat = cycleBatV > 0 ? String(cycleBatV, 2) : vBat();
  docOut["vB"] = bat;
  Serial.printf("[SENSOR] Batería: %s V\n", bat.c_str());

  // Serializar JSON
  String jsonString;
  serializeJson(docOut, jsonString);
  Serial.println("[DATA] " + jsonString);

  return jsonString;
}

/* ====================== WAIT FOR CONFIG ======================== */
// Solo espera 5 s procesando mensajes MQTT.
// La suscripción debe hacerse ANTES de publicar datos (en sendData*)
// para no perder la respuesta rápida del servidor.
void waitForConfig(PubSubClient &mqttClient) {
  Serial.println("[CONFIG] Esperando configuración (máx. 5 s)...");
  configReceivedFlag = false;
  unsigned long t = millis();
  // Cortar en cuanto llega la config en vez de esperar siempre los 5 s:
  // cada segundo de espera de más mantiene el modem LTE (o WiFi) encendido
  // sin necesidad, ciclo tras ciclo.
  while (millis() - t < 5000 && !configReceivedFlag) {
    mqttClient.loop();
    delay(50);
  }
  Serial.println("[CONFIG] Fin espera");
}

/* ============MODEM ========== ENVIO LTE (SIM800) ======================== */
// waitForNetwork() con diagnóstico: cada 3 s registra estado CREG, CSQ y
// la tensión que ve el SIM800 (AT+CBC), guarda la mínima en lastModemMv y
// corta antes si el modem deja de responder (se apagó por bajo voltaje en
// un burst de TX) o la red rechaza el registro. AT+CBC es un promedio: los
// valles de los bursts de 577 µs son más profundos que lo que muestra.
bool waitForNetworkMonitored(uint32_t timeoutMs) {
  uint32_t start = millis();
  int16_t minMv = lastModemMv > 0 ? lastModemMv : 9999;
  int noAnswer = 0;
  uint32_t lastDiag = 0;
  bool firstDiag = true;
  while (millis() - start < timeoutMs) {
    // CREG cada 1 s (antes 3 s): el registro se detecta hasta ~2 s antes, y
    // cada segundo con el modem buscando red cuesta ~100 mA. El diagnóstico
    // (CBC/CSQ/OLED) sigue cada 3 s.
    SIM800RegStatus st = modem.getRegistrationStatus();
    if (st == REG_OK_HOME || st == REG_OK_ROAMING) {
      if (minMv < 9999) lastModemMv = minMv;
      return true;
    }
    if (st == REG_DENIED) {
      Serial.println("[ERROR] Registro rechazado por la red (SIM/operador)");
      setNetStatus("Registro denegado");
      if (minMv < 9999) lastModemMv = minMv;
      return false;
    }
    if (!firstDiag && millis() - lastDiag < 3000) {
      delay(1000);
      continue;
    }
    firstDiag = false;
    lastDiag = millis();
    int16_t mv = modem.getBattVoltage();
    int csq = modem.getSignalQuality();
    mdiagTrackBat();
    mdiagTrackVsim(mv);
    if (mv > 0 && mv < minMv) minMv = mv;
    Serial.printf("[LTE] t=%2lus CREG=%d CSQ=%d V=%d mV%s\n",
                  (unsigned long)((millis() - start) / 1000), (int)st, csq, mv,
                  (mv > 0 && mv < 3500) ? "  <-- BAJA" : "");
    dash.bars = csqToBars(csq);  // las barras de señal ya informan la calidad
    modemStepSet("M8 CREG");
    mdiagShowWait("Buscando", "senal", start);

    if (st == REG_NO_RESULT && mv <= 0) {
      if (++noAnswer >= 2) {
        if (mdiag.vSimMax >= SIM800_VMAX_MV) {
          Serial.printf("[ERROR] El SIM800 se apagó por SOBREVOLTAJE (llegó a %d mV, máx 4400)\n", mdiag.vSimMax);
          setNetStatus("Modem sobrevoltaje");
        } else {
          Serial.println("[ERROR] El SIM800 dejó de responder buscando red (¿se apagó por bajo voltaje?)");
          setNetStatus("Modem se apago");
        }
        if (minMv < 9999) lastModemMv = minMv;
        return false;
      }
    } else {
      noAnswer = 0;
    }
    delay(1000);
  }
  if (minMv < 9999) lastModemMv = minMv;
  Serial.printf("[LTE] Tensión mínima del SIM800 buscando red: %d mV\n", lastModemMv);
  return false;
}

bool sendDataLTE(String jsonData) {
  Serial.println("\n[LTE] Iniciando envío de datos...");

  // Encender modem (si readSensors() ya lo encendió para el RS485, solo se
  // completa el tiempo de arranque que falte)
  if (!modemPoweredOn) modemSessionBegin();
  modemPowerOnFinish();

  // Inicializar modem: arranque largo + estabilización del enlace (ver
  // modemStabilize()) y hasta MODEM_INIT_TRIES init sin cortar alimentación.
  // Solo si todo eso falla se reinicia (el LED de red se apaga y vuelve a
  // parpadear) y se repite. Las pruebas AT de diagnóstico van al final.
  bool linkOk = modemStabilize();
  if (!linkOk || !modemInitRetries()) {
    mdiagShow("M7 CORTE", "Reinicio de modem", linkOk ? "init fallo" : "sin AT");
    modemRestart();
    linkOk = modemStabilize();
    if (!linkOk || !modemInitRetries()) {
      modemDiagProbeAT();
      Serial.println("[ERROR] Modem no responde");
      setNetStatus(modemDiagCause());
      modemPowerOff();
      modemDiagFail("INIT");
      return false;
    }
  }

  Serial.println("[LTE] Modem: " + modem.getModemInfo());
  modemApplyBand();

  // Tensión que ve el propio SIM800 en sus pines (AT+CBC): bajo 3.5 V el
  // módulo avisa UNDER-VOLTAGE y bajo ~3.4 V se apaga solo en plena
  // transmisión — es el diagnóstico directo de los reinicios.
  lastModemMv = modem.getBattVoltage();
  mdiagTrackVsim(lastModemMv);
  Serial.printf("[LTE] Tensión en el SIM800: %d mV%s\n", lastModemMv,
                (lastModemMv > 0 && lastModemMv < 3500) ? "  <-- BAJA" :
                lastModemMv >= SIM800_VMAX_MV ? "  <-- SOBREVOLTAJE (max 4.4 V)" : "");
  if (lastModemMv >= SIM800_VMAX_MV) {
    // Se sigue intentando (puede alcanzar a enviar), pero se avisa y se deja
    // ver 2 s: el SIM800 se apagará solo en cuanto pase su umbral.
    mdiagShow("M7 SOBREV!", "Voltaje muy alto!", "Vsim " + String(lastModemMv / 1000.0f, 2) + "V max 4.40V");
    delay(2000);
  }
  mdiagShow("M7 INIT OK", "Modem listo",
            "Vmod " + (lastModemMv > 0 ? String(lastModemMv / 1000.0f, 2) + "V" : String("--")) +
            " Bmin " + String(mdiag.vBatMin, 2));

  // Esperar red
  Serial.println("[LTE] Esperando red...");
  dashStep("RED");
  mdiagShow("M8 RED", "Buscando senal");
  if (!waitForNetworkMonitored(60000L)) {
    Serial.println("[ERROR] Sin registro en la red");
    if (strcmp(lastNetStatus, "Modem se apago") != 0 && strcmp(lastNetStatus, "Registro denegado") != 0 &&
        strcmp(lastNetStatus, "Modem sobrevoltaje") != 0)
      setNetStatus("Sin cobertura");
    modemPowerOff();
    modemDiagFail("RED");
    return false;
  }

  int csq = modem.getSignalQuality();
  snprintf(lastSignal, sizeof(lastSignal), "CSQ %d", csq);
  Serial.printf("[LTE] Red conectada (CSQ %d)\n", csq);
  dash.bars = csqToBars(csq);
  static const char *const quality[] = {"nula", "debil", "regular", "buena", "excelente"};
  oledLog(String("Senal ") + quality[csqToBars(csq)]);

  // Conectar GPRS
  Serial.println("[LTE] Conectando GPRS...");
  dashStep("INTERNET");
  mdiagShow("M9 GPRS", "Conectando", RUNapn);
  if (!modem.gprsConnect(RUNapn.c_str())) {
    Serial.println("[ERROR] Fallo conexión GPRS");
    setNetStatus("Fallo GPRS");
    modemPowerOff();
    modemDiagFail("GPRS");
    return false;
  }

  Serial.println("[LTE] GPRS conectado");

  // Configurar MQTT
  String serverIP = readFile(SPIFFS, "/server.txt");
  String topicPub = readFile(SPIFFS, "/topicPub.txt");

  if (serverIP == "") serverIP = broker;
  if (topicPub == "") topicPub = "jhpAI/HIDROMET/data";

  simclient.setServer(serverIP.c_str(), mqtt_port);
  // El buffer por defecto de PubSubClient (256 B, incluye topic y cabecera)
  // no alcanza para el JSON con campos RS485: publish() fallaría en silencio.
  simclient.setBufferSize(768);
  dashStep("SERVIDOR");
  mdiagShow("M10 MQTT", "Conectando", serverIP);

  // Conectar a MQTT
  int mqttRetries = 0;
  String clientId = "ws-" + macA();

  while (!simclient.connected() && mqttRetries < 5) {
    Serial.printf("[MQTT] Conectando a %s... ", serverIP.c_str());

    if (simclient.connect(clientId.c_str())) {
      Serial.println("OK");
    } else {
      Serial.printf("Error: %d\n", simclient.state());
      mqttRetries++;
      delay(2000);
    }
  }

  if (!simclient.connected()) {
    Serial.println("[ERROR] No se pudo conectar a MQTT");
    setNetStatus("Fallo MQTT");
    modemPowerOff();  // AT+CPOWD ya hace el detach de GPRS
    modemDiagFail("MQTT");
    return false;
  }

  // Suscribir ANTES de publicar: el servidor responde rápido
  String cfgTopic = readFile(SPIFFS, "/topicSub.txt");
  if (cfgTopic == "") cfgTopic = String(topicSubscribe);
  cfgTopic += "/" + macA();
  simclient.setCallback(callback);
  simclient.subscribe(cfgTopic.c_str());
  Serial.printf("[CONFIG] Suscrito a: %s\n", cfgTopic.c_str());

  // Reenviar primero cualquier lectura pendiente de ciclos anteriores sin
  // conexión, en orden cronológico, antes del dato de este ciclo.
  int pendingLTE = countPendingRecords();
  dashStep("ENVIANDO", pendingLTE > 0 ? "+" + String(pendingLTE) + " previos" : "");
  flushPendingData(simclient, topicPub);

  // Publicar datos
  bool published = simclient.publish(topicPub.c_str(), jsonData.c_str());

  if (published) {
    Serial.println("[MQTT] Datos publicados correctamente");
    setNetStatus("Enviado OK (LTE)");
    syncTimeFromInternetLTE();
  } else {
    Serial.println("[ERROR] Fallo al publicar");
    setNetStatus("Fallo publicacion");
  }

  simclient.loop();
  delay(100);

  if (published) {
    oledLog("Esperando ajustes");
    waitForConfig(simclient);
    if (configReceivedFlag) oledLog("Ajustes recibidos");
  }

  // Desconectar
  // Sin gprsDisconnect(): AT+CIPSHUT + AT+CGATT=0 tardan 1-3 s con el modem
  // encendido y AT+CPOWD (modemPowerOff) ya se desregistra de la red.
  simclient.disconnect();
  modemPowerOff();

  Serial.println("[LTE] Desconectado");
  dash.modem = "";
  oledLog("Modem apagado");

  return published;
}

/* ====================== ENVIO WIFI======================== */
bool sendDataWiFi(String jsonData) {
  Serial.println("\n[WIFI] Iniciando envío de datos...");

  RUNssid = readFile(SPIFFS, "/ssidString.txt");
  RUNpass = readFile(SPIFFS, "/passString.txt");

  lastSignal[0] = '\0';
  if (RUNssid == "") {
    Serial.println("[ERROR] SSID no configurado");
    setNetStatus("SSID no configurado");
    return false;
  }

  dashStep("WIFI", RUNssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(RUNssid.c_str(), RUNpass.c_str());

  int wifiRetries = 0;
  while (WiFi.status() != WL_CONNECTED && wifiRetries < 20) {
    delay(500);
    Serial.print(".");
    wifiRetries++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[ERROR] No se pudo conectar a WiFi");
    setNetStatus("Sin WiFi");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  Serial.println("\n[WIFI] Conectado: " + WiFi.localIP().toString());
  snprintf(lastSignal, sizeof(lastSignal), "%d dBm", (int)WiFi.RSSI());
  dash.bars = rssiToBars(WiFi.RSSI());
  oledLog(WiFi.localIP().toString());

  // Configurar MQTT
  String serverIP = readFile(SPIFFS, "/server.txt");
  String topicPub = readFile(SPIFFS, "/topicPub.txt");

  if (serverIP == "") serverIP = broker;
  if (topicPub == "") topicPub = "jhpAI/HIDROMET/data";

  wificlient.setServer(serverIP.c_str(), mqtt_port);
  // Ver nota en sendDataLTE(): buffer por defecto de 256 B insuficiente.
  wificlient.setBufferSize(768);
  dashStep("SERVIDOR", "Conectando");

  // Conectar a MQTT
  int mqttRetries = 0;
  String clientId = "ws-" + macA();

  while (!wificlient.connected() && mqttRetries < 5) {
    Serial.printf("[MQTT] Conectando a %s... ", serverIP.c_str());

    if (wificlient.connect(clientId.c_str())) {
      Serial.println("OK");
    } else {
      Serial.printf("Error: %d\n", wificlient.state());
      mqttRetries++;
      delay(1000);
    }
  }

  if (!wificlient.connected()) {
    Serial.println("[ERROR] No se pudo conectar a MQTT");
    setNetStatus("Fallo MQTT");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  // Suscribir ANTES de publicar: el servidor responde rápido
  String cfgTopic = readFile(SPIFFS, "/topicSub.txt");
  if (cfgTopic == "") cfgTopic = String(topicSubscribe);
  cfgTopic += "/" + macA();
  wificlient.setCallback(callback);
  wificlient.subscribe(cfgTopic.c_str());
  Serial.printf("[CONFIG] Suscrito a: %s\n", cfgTopic.c_str());

  // Reenviar primero cualquier lectura pendiente de ciclos anteriores sin
  // conexión, en orden cronológico, antes del dato de este ciclo.
  int pendingWiFi = countPendingRecords();
  dashStep("ENVIANDO", pendingWiFi > 0 ? "+" + String(pendingWiFi) + " previos" : "");
  flushPendingData(wificlient, topicPub);

  // Publicar datos
  bool published = wificlient.publish(topicPub.c_str(), jsonData.c_str());

  if (published) {
    Serial.println("[MQTT] Datos publicados correctamente");
    setNetStatus("Enviado OK (WiFi)");
    syncTimeFromInternetWiFi();
  } else {
    Serial.println("[ERROR] Fallo al publicar");
    setNetStatus("Fallo publicacion");
  }

  wificlient.loop();
  delay(100);

  if (published) {
    oledLog("Esperando ajustes");
    waitForConfig(wificlient);
    if (configReceivedFlag) oledLog("Ajustes recibidos");
  }

  // Desconectar
  wificlient.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  Serial.println("[WIFI] Desconectado");
  oledLog("wifi off");

  return published;
}

/* ====================== POLÍTICA DE ENVÍO / ENERGÍA ======================== */
// Devuelve "" si este ciclo debe intentar el envío, o el motivo (corto, se
// muestra en el OLED) para omitirlo. Un ciclo omitido no pierde datos: la
// lectura ya está en la SD y queda en /pending.jsonl para reenviarse.
String networkSkipReason() {
  // Tras un reset por brownout, reintentar enseguida reproduce el mismo
  // hundimiento de VCC: bucle de reinicios que vacía la batería.
  if (brownoutBoot) return "Brownout: sin envio";

  bool batValid = cycleBatV >= BAT_VALID_MIN_V;
  if (connectionType == "LTE" && batValid && cycleBatV < BAT_MODEM_MIN_V) {
    return "Bat baja " + String(cycleBatV, 2) + "V";
  }

  // Backoff exponencial: tras N fallos seguidos (sin cobertura, AP caído,
  // brownouts) intentar cada 2, 4, 8 ciclos en vez de gastar 60+ s de
  // modem en cada uno.
  if (netFailStreak >= 2) {
    uint32_t every = 1UL << min(netFailStreak - 1, 3);
    if (every > NET_BACKOFF_MAX_SKIP) every = NET_BACKOFF_MAX_SKIP;
    uint32_t phase = opCycleCount % every;
    if (phase != 0) return "Reintento en " + String(every - phase) + " cic";
  }
  return "";
}

/* ====================== INIT RUNNING ======================== */
// pinConfigAP (GPIO36) es un pin solo-entrada SIN resistencia pull interna en el ESP32
// (igual que 34/35/39) — requiere resistencia pull-up EXTERNA a 3.3V. El botón de
// configuración debe conectar el pin a GND al presionarlo (activo en LOW).
void init_running() {
  pinMode(pinConfigAP, INPUT);

  // GPIO36 no tiene pull-up interna; leer múltiples veces para confirmar
  // que el botón está realmente presionado (pin sostenido en GND) y no flotando.
  bool configPressed = true;
  for (int i = 0; i < 10; i++) {
    if (digitalRead(pinConfigAP)) { configPressed = false; break; }
    delay(10);
  }

  if (configPressed) {
    Serial.println("====> Modo Configuración");
    modemPowerOff();
    setting();
  } else {
    Serial.println("====> Modo Operación");

    connectionType = readFile(SPIFFS, "/connType.txt");

    if (connectionType == "") {
      connectionType = "WIFI";
      writeFile(SPIFFS, "/connType.txt", connectionType.c_str());
    }

    readConfig();
    opCycleCount++;

    // 80 MHz ahorra ~20-30 mA durante sensores/SD. La sesión con el SIM800
    // sube a 240 MHz como en PROD (a603f7a), que conectaba (tras bajar a
    // 80 MHz aparecieron bytes corruptos en su UART): ver modemPowerOnFinish().
    Serial.flush();  // cambiar la frecuencia con bytes en el FIFO del UART los corrompe
    setCpuFrequencyMhz(80);

    // Batería en reposo (modem apagado, sin carga de RF): decide si el modem
    // puede encenderse sin tirar el sistema y si hay que alargar el sleep.
    // Se mide antes de los sensores para decidir ya si este ciclo envía.
    cycleBatV = vBat().toFloat();
    String skip = connectionType == "DATALOGER" ? "" : networkSkipReason();

    // OLED del ciclo: si está habilitado (/oledOp.txt) o ya activo (ciclo
    // manual desde la consulta de estado). Mantiene v_sensor encendido
    // durante todo el ciclo, no solo durante la lectura de sensores.
    if (oledOpEnabled || oledReady) {
      sensorsRailOn();
      Wire.begin(I2C_SDA, I2C_SCL);
      if (oledInit()) {
        dashBegin("OPER", dashStepsFor(connectionType));
        dashStep("SENSORES");
      }
    }

    String sensorData = readSensors();
    strlcpy(lastDataJson, sensorData.c_str(), sizeof(lastDataJson));

    // Respaldo local en la SD, independientemente de si el envío MQTT funciona
    // (o si no hay envío en absoluto, modo DATALOGER). Usa la hora ancla
    // real-time cargada de /settime (ver logToSD()/getEpochNow()), no
    // depende de tener red disponible en este ciclo.
    logToSD(sensorData);
    dashStep("SD", sdReady ? "registrado" : "no detectada");

    if (cycleBatV >= BAT_VALID_MIN_V && cycleBatV < BAT_MODEM_MIN_V) {
      oledLog("bat baja " + String(cycleBatV, 2) + "V");
    }

    if (connectionType == "DATALOGER") {
      // Modo solo-registro local: no se enciende WiFi ni el modem, no hay
      // nada que enviar ni encolar como pendiente — el dato ya quedó en la SD.
      Serial.println("[MODE] Dataloger local (sin envío, solo respaldo en SD)");
      lastSignal[0] = '\0';
      setNetStatus("Solo SD (datalogger)");
    } else {
      bool success = false;

      if (skip != "") {
        Serial.println("[POWER] Envío omitido este ciclo: " + skip);
        setNetStatus(skip.c_str());
      } else if (connectionType == "WIFI") {
        Serial.println("[MODE] WiFi");
        success = sendDataWiFi(sensorData);
      } else if (connectionType == "LTE") {
        Serial.println("[MODE] LTE");
        success = sendDataLTE(sensorData);
      }

      // Solo los intentos reales cuentan para la racha (no los omitidos)
      if (skip == "") {
        netFailStreak = success ? 0 : netFailStreak + 1;
        saveSetting(String(netFailStreak), "/netFail.txt");
      }

      if (modemPoweredOn) modemPowerOff();  // por si algún camino lo dejó encendido
      Serial.flush();
      setCpuFrequencyMhz(80);  // resumen OLED / sleep: no hace falta 240 MHz

      lastCycleSent = success;
      if (success) {
        Serial.println("[OK] Datos enviados correctamente");
      } else {
        Serial.println("[ERROR] Fallo al enviar datos");
        appendPendingRecord(sensorData);
      }
    }

    if (oledReady) {
      int pending = countPendingRecords();
      dash.batV = cycleBatV;
      String sleepTxt = "zzz " + String((unsigned long)(configuredSleepPeriod() / 60)) + "min";
      if (connectionType == "DATALOGER") dashFinish("SOLO SD", sleepTxt);
      else dashFinish(lastCycleSent ? "ENVIADO" : "GUARDADO",
                      (pending > 0 ? "pend " + String(pending) + " " : "") + sleepTxt);
      delay(3000);  // dar tiempo a leer el resumen (se descuenta del sleep)
    }

    // Entrar en deep sleep
    deepSleepSystem();

    delay(500);
  }
}

/* ================== READ CONFIG ==================*/
void readConfig() {
  Serial.println("\n========== LEYENDO CONFIGURACIONES ==========\n");

  String ath = readFile(SPIFFS, "/A_TH.txt");
  String aws = readFile(SPIFFS, "/A_WS.txt");
  String apr = readFile(SPIFFS, "/A_PR.txt");
  String dirSensor = readFile(SPIFFS, "/dirSensor.txt");

  A_TH = (ath == "1");
  A_WS = (aws == "1");
  A_PR = (apr == "1");
  dirSensorType = (dirSensor == "AS5600") ? "AS5600" : "MT6701";
  oledOpEnabled = (readFile(SPIFFS, "/oledOp.txt") != "0");
  String rs485Cfg = readFile(SPIFFS, "/rs485.txt");
  rs485Type = rs485Norm(rs485Cfg);

  netFailStreak = readFile(SPIFFS, "/netFail.txt").toInt();

  String apnCfg = readFile(SPIFFS, "/apn.txt");
  RUNapn = (apnCfg == "") ? "claro.pe" : apnCfg;

  Serial.println("---- VALORES CARGADOS ----");
  Serial.printf("A_TH = %s\n", A_TH ? "true" : "false");
  Serial.printf("A_WS = %s\n", A_WS ? "true" : "false");
  Serial.printf("A_PR = %s\n", A_PR ? "true" : "false");
  Serial.printf("dirSensorType = %s\n", dirSensorType.c_str());
  Serial.printf("oledOp = %s\n", oledOpEnabled ? "true" : "false");
  Serial.printf("rs485Type = %s\n", rs485Type.c_str());
  Serial.printf("RUNapn = %s\n\n", RUNapn.c_str());
}

/* ================== INIT SENSORS ==================*/
// Wire.begin() se llama UNA SOLA VEZ en readSensors/updateSensCache
// para que el MT6701 use el bus I2C sin conflictos con otras llamadas.
void initDHT22() {
  dht.begin();
  delay(2000);
  // La primera lectura tras power-on devuelve siempre valores de arranque (1.0/1.0);
  // se descarta para que las siguientes lecturas sean válidas.
  dht.readTemperature();
  dht.readHumidity();
  delay(2000);
}

void initMT6701() {
  // Wire ya inicializado externamente; solo configurar el encoder elegido (MT6701/AS5600)
  delay(50);
  if (dirSensorType == "AS5600") {
    as5600.begin();
  } else {
    encoder.initializeI2C();
  }
  delay(50);
}

/* ====================== SENSOR CACHE UPDATE ======================== */
// En modo configuración el sensor rail se deja energizado de forma continua,
// DHT22/MT6701 se inicializan una sola vez, y el viento se cuenta por
// interrupción en segundo plano: así updateSensCache() nunca bloquea y puede
// llamarse en cada vuelta de loop() para reflejar lecturas casi en tiempo real.
void updateSensCache() {
  static bool sensorsInitialized = false;
  static unsigned long lastDHTRead = 0;
  static unsigned long lastWindCalc = 0;
  static unsigned long windPulsesAtLastCalc = 0;
  static unsigned long lastRainCheckTime = 0;
  static unsigned long rainPulsesAtLastCheck = 0;
  const unsigned long RAIN_CHECK_WINDOW_MS = 5000;  // ventana para detectar variacion de pulsos

  if (!sensorsInitialized) {
    Serial.println("[CACHE] Inicializando sensores (una sola vez)...");

    // MOSFET canal P: LOW = ON — se mantiene encendido mientras dure el modo config
    pinMode(SENS_PWR_PIN, OUTPUT);
    digitalWrite(SENS_PWR_PIN, LOW);
    delay(2500);  // AM2301 necesita ≥2.5 s de arranque frío tras encender v_sensor

    // Iniciar I2C una sola vez — lo usan el MT6701/AS5600 (si A_WS) y siempre
    // el DS3231 (marca de tiempo), así que ya no se condiciona a A_WS.
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(50);
    initDS3231();

    if (A_WS) {
      initMT6701();

      pinMode(pinWindVelocity, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(pinWindVelocity), windPulseISR, FALLING);
      windISRAttached = true;
    }

    if (A_TH) {
      initDHT22();
      lastDHTRead = millis() - 2000;  // permitir una lectura inmediata
    }

    if (A_PR) {
      // Sin interrupción: el nivel "sin campo" (~2.3V) queda demasiado
      // cerca del umbral digital del ESP32 para fiarse de un FALLING por
      // hardware. Se sondea el voltaje real por ADC en cada vuelta de
      // loop() más abajo (ver RAIN_ADC_FIELD_MV / RAIN_ADC_IDLE_MV).
      pinMode(RAINCOUNT, INPUT);
    }

    lastWindCalc = millis();
    lastRainCheckTime = millis();
    sensorsInitialized = true;
  }

  // DHT22: el sensor solo admite una lectura válida cada ~2 s
  if (A_TH) {
    if (millis() - lastDHTRead >= 2000) {
      float t, h; bool ok;
      readDHT22(t, h, ok);
      if (ok) {
        sensCache.temp = String(t, 1);
        sensCache.hum  = String(h, 1);
      }
      lastDHTRead = millis();
    }
  } else {
    sensCache.temp = "NA";
    sensCache.hum  = "NA";
  }

  if (A_WS) {
    // Velocidad: pulsos acumulados por ISR desde el último cálculo (ventana de 1 s)
    unsigned long now = millis();
    unsigned long elapsed = now - lastWindCalc;
    if (elapsed >= 1000) {
      // Lectura de 32 bits, atómica en ESP32: no hace falta deshabilitar
      // interrupciones (hacerlo aquí interfiere con el stack WiFi/AsyncTCP).
      unsigned long pulses = windPulseCountISR;
      unsigned long delta = pulses - windPulsesAtLastCalc;
      windPulsesAtLastCalc = pulses;
      lastWindCalc = now;

      float speed = delta * (WIND_SPEED_PER_PULSE / (elapsed / 1000.0));
      if (speed > MAX_WIND_SPEED_MS) {
        Serial.printf("[WIND] Lectura descartada por implausible: %.1f m/s (ruido del sensor Hall)\n", speed);
        speed = 0;
      }
      sensCache.wvel = String(windCal(speed), 1);
    }
    sensCache.wdir = String(WindDirection(), 1);
  } else {
    sensCache.wvel = "NA";
    sensCache.wdir = "NA";
  }

  if (A_PR) {
    // Conteo por sondeo de ADC con histéresis (no por interrupción, ver
    // nota en la inicialización): se muestrea cada ~20 ms y solo se
    // cuenta un pulso al pasar de "reposo confirmado" a "campo
    // confirmado", con 500-1500 mV de zona muerta entre ambos y un
    // mínimo de RAIN_PULSE_MIN_INTERVAL_MS entre pulsos contados.
    static bool rainIdleState = true;
    static unsigned long lastRainSampleMs = 0;
    static unsigned long lastRainPulseMs = 0;

    unsigned long nowR = millis();
    if (nowR - lastRainSampleMs >= 20) {
      lastRainSampleMs = nowR;
      uint32_t mv = analogReadMilliVolts(RAINCOUNT);

      if (mv < RAIN_ADC_FIELD_MV) {
        if (rainIdleState && (nowR - lastRainPulseMs > RAIN_PULSE_MIN_INTERVAL_MS)) {
          rainPulseCountISR++;
          lastRainPulseMs = nowR;
        }
        rainIdleState = false;
      } else if (mv > RAIN_ADC_IDLE_MV) {
        rainIdleState = true;
      }
      // Entre 500-1500 mV: zona muerta, se ignora la muestra (no cambia estado).
    }

    // Total acumulado del ciclo (pulsos persistidos en SPIFFS + los contados
    // en vivo desde que arrancó el modo config, aún no volcados a SPIFFS).
    unsigned long livePulses = rainPulseCountISR;

    // Diagnóstico: confirma en el monitor serie que cada pulso del sensor
    // Hall (~2.3V sin campo / ~21mV con campo) fue detectado por sondeo ADC.
    static unsigned long lastLoggedPulses = 0;
    if (livePulses != lastLoggedPulses) {
      Serial.printf("[RAIN] Pulso detectado en vivo. Total sesión: %lu (%lu mV)\n",
                    livePulses, (unsigned long)analogReadMilliVolts(RAINCOUNT));
      lastLoggedPulses = livePulses;
    }

    int savedPulses = readFile(SPIFFS, "/ssidRainCounter.txt").toInt();
    unsigned long totalPulses = savedPulses + livePulses;
    sensCache.rain = String(totalPulses * RAIN_MM_PER_TIP, 2);
    sensCache.rainCount = String(totalPulses);

    // Algoritmo de detección de tormenta: si el conteo de pulsos no varía
    // dentro de la ventana, no hay lluvia; si se incrementa, hay lluvia presente.
    unsigned long now = millis();
    if (now - lastRainCheckTime >= RAIN_CHECK_WINDOW_MS) {
      sensCache.rainStatus = (livePulses > rainPulsesAtLastCheck) ? "Lluvia detectada" : "Sin lluvia";
      rainPulsesAtLastCheck = livePulses;
      lastRainCheckTime = now;
    }
  } else {
    sensCache.rain = "NA";
    sensCache.rainStatus = "NA";
    sensCache.rainCount = "NA";
  }

  // RS485: lectura bloqueante (~2-3 s; el nivel ACLT600 ~10 s la primera vez,
  // luego el 12V queda encendido en modo config),
  // así que solo cada 30 s y no en cada vuelta de loop().
  static unsigned long lastRs485Read = 0;
  if (rs485Type == "NONE") {
    sensCache.rsT = sensCache.rsH = sensCache.rsEC = sensCache.rsPH = sensCache.rsRad = sensCache.rsLv = "NA";
    sensCache.rs485 = "NA";
    if (rs485RailOnMs && !rs485Busy) rs485RailOff();  // se deshabilitó con el 12V encendido
  } else if (lastRs485Read == 0 || millis() - lastRs485Read >= 30000) {
    Rs485Data rs;
    if (readRS485(rs, true)) {
      if (rs485Type == "SOIL") {
        sensCache.rsT  = String(rs.soilT, 1);
        sensCache.rsH  = String(rs.soilH, 1);
        sensCache.rsEC = String(rs.ec);
        sensCache.rsPH = String(rs.ph, 1);
        sensCache.rs485 = "T " + sensCache.rsT + "°C H " + sensCache.rsH + "% EC " + sensCache.rsEC + " pH " + sensCache.rsPH;
      } else if (rs485Type == "LEVEL") {
        sensCache.rsLv = String(rs.level, 1);
        sensCache.rs485 = "Nivel " + sensCache.rsLv + " (crudo)";
      } else {
        sensCache.rsRad = String(rs.rad);
        sensCache.rs485 = sensCache.rsRad + " W/m²";
      }
    } else {
      sensCache.rsT = sensCache.rsH = sensCache.rsEC = sensCache.rsPH = sensCache.rsRad = sensCache.rsLv = "error";
      sensCache.rs485 = "Sin respuesta";
    }
    lastRs485Read = millis();
  }

  sensCache.bat = vBat();
}

/* ================== CREATE DEFAULT CONFIGS ==================*/
void createDefaultConfigs() {
  Serial.println("[INIT] Verificando archivos de configuración...");

  struct ConfigFile {
    const char* path;
    const char* defaultValue;
  };

  ConfigFile configs[] = {
    {"/A_TH.txt", "0"},
    {"/A_WS.txt", "0"},
    {"/A_PR.txt", "0"},
    {"/dirSensor.txt", "MT6701"},
    {"/oledOp.txt", "1"},
    {"/rs485.txt", "NONE"},
    {"/netFail.txt", "0"},
    {"/connType.txt", "WIFI"},
    {"/apn.txt", "claro.pe"},
    {"/timesleep.txt", "300"},
    {"/ssidRainCounter.txt", "0"},
    {"/ssidString.txt", ""},
    {"/passString.txt", ""},
    {"/server.txt", ""},
    {"/port.txt", "8000"},
    {"/topicPub.txt", "jhpAI/HIDROMET/data"},
    {"/topicSub.txt", "jhpAI/HIDROMET/settings"},
    {"/status.txt", "1"},
    {"/timeBaseEpoch.txt", "0"},
    {"/timeBaseRtc.txt", "0"}
  };

  int numConfigs = sizeof(configs) / sizeof(configs[0]);

  for (int i = 0; i < numConfigs; i++) {
    if (!SPIFFS.exists(configs[i].path)) {
      Serial.printf("[INIT] Creando: %s = %s\n", configs[i].path, configs[i].defaultValue);
      writeFile(SPIFFS, configs[i].path, configs[i].defaultValue);
    }
  }

  // Migración: los topics por defecto pasaron de FAUNA a HIDROMET. Solo se
  // reemplaza el valor por defecto antiguo; un topic personalizado se respeta.
  if (readFile(SPIFFS, "/topicPub.txt") == "jhpAI/FAUNA/data")
    writeFile(SPIFFS, "/topicPub.txt", "jhpAI/HIDROMET/data");
  if (readFile(SPIFFS, "/topicSub.txt") == "jhpAI/FAUNA/settings")
    writeFile(SPIFFS, "/topicSub.txt", "jhpAI/HIDROMET/settings");

  Serial.println("[INIT] Archivos de configuración verificados\n");
}

/* ================== ARRANQUE TEMPRANO ==================*/
// GPIO15 (sim_control) es pin de arranque con pull-up interno activo durante
// el reset: a través de R1/R13 alcanza para saturar Q4/Q5 y el modem + el
// 12V del RS485 se encienden solos hasta que el firmware fuerce LOW. Este
// hook del core corre antes que setup() (y antes de Serial.begin + delay),
// así que acorta ese encendido espurio lo más posible. No lo elimina: los
// ~250 ms del bootloader quedan fuera del alcance del firmware (ver la
// recomendación de hardware: pull-down de 4.7k en GPIO15).
// En un wake desde deep sleep el pin sigue retenido (gpio_hold) en LOW y
// esto no tiene efecto.
extern "C" void initVariant() {
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, MODEM_PWR_OFF);
  // GPIO14 (12V RS485) también emite señal durante el arranque: apagarlo ya.
  pinMode(RS485_PWR_PIN, OUTPUT);
  digitalWrite(RS485_PWR_PIN, RS485_PWR_OFF);
}

/* ================== MAIN SETUP ==================*/
void setup() {
  Serial.begin(115200);
  delay(100);

  // Capturado lo antes posible: referencia para medir cuánto dura este
  // ciclo despierto (ver deepSleepSystem()).
  bootRtcSeconds = getRtcTimeSeconds();

  // ===== LIBERAR TODOS LOS GPIO HOLDS =====
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)MODEM_PWR_PIN);
  gpio_hold_dis((gpio_num_t)SENS_PWR_PIN);
  gpio_hold_dis((gpio_num_t)PIN_TX2);
  gpio_hold_dis((gpio_num_t)PIN_RX2);
  gpio_hold_dis((gpio_num_t)RS485_PWR_PIN);

  // ===== ASEGURAR MODEM Y SENSORES APAGADOS =====
  // Modem: MODEM_PWR_OFF (LOW, por el NPN Q4). Sensores: MOSFET canal P,
  // HIGH = OFF. No se requiere pulso de PWRKEY: toda la alimentación del
  // módulo se corta con el MOSFET.
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, MODEM_PWR_OFF);

  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, HIGH);

  pinMode(RS485_PWR_PIN, OUTPUT);
  digitalWrite(RS485_PWR_PIN, RS485_PWR_OFF);

  // RX2/TX2 a LOW por defecto hasta que modemPowerOn() los reconfigure
  // como UART: evita retroalimentar el SIM800 (ver modemLinesOff()).
  pinMode(PIN_TX2, OUTPUT);
  pinMode(PIN_RX2, OUTPUT);
  digitalWrite(PIN_TX2, LOW);
  digitalWrite(PIN_RX2, LOW);

  // Motivo del reset: "BROWNOUT" = VCC se hundió por debajo de ~2.43 V en
  // el ESP32 (típico al transmitir el SIM800 con batería floja o cableado
  // fino). Sin esta marca el equipo reintentaría el envío de inmediato.
  esp_reset_reason_t rst = esp_reset_reason();
  brownoutBoot = (rst == ESP_RST_BROWNOUT);
  Serial.printf("[BOOT] Motivo de reset: %d%s\n", (int)rst, brownoutBoot ? " (BROWNOUT)" : "");
  if (rst != ESP_RST_POWERON && rst != ESP_RST_DEEPSLEEP &&
      modemStepMagic == MODEM_STEP_MAGIC && modemStep[0]) {
    modemStep[sizeof(modemStep) - 1] = '\0';
    Serial.printf("[BOOT] El reset ocurrió con el modem en el paso: %s\n", modemStep);
    rstModemStep = String(modemStep).substring(0, 10);
  }
  modemStepMagic = 0;

  // Montar SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("Intentando formatear...");

    if (SPIFFS.format()) {
      Serial.println("[OK] SPIFFS formateado");
      if (SPIFFS.begin(true)) {
        Serial.println("[OK] SPIFFS montado después de formatear");
      } else {
        Serial.println("[ERROR CRÍTICO] No se pudo montar SPIFFS");
        return;
      }
    } else {
      Serial.println("[ERROR CRÍTICO] No se pudo formatear SPIFFS");
      return;
    }
  } else {
    Serial.println("[OK] SPIFFS montado correctamente");
  }

  // ===== CAMINO RÁPIDO: wake por lluvia (EXT1) =====
  // Si despertamos por un pulso del pluviómetro, saltarse el montaje de
  // SD, createDefaultConfigs() y el resto de readConfig(): nada de eso
  // hace falta solo para incrementar el contador, y el tiempo que suman
  // (el montaje de SD en particular, con la tarjeta ausente o con fallo,
  // puede tardar varios cientos de ms en fallar) hacía que para cuando el
  // código llegaba a leer el pin ya se hubiera reabierto el contacto de
  // un vaciado breve, descartándolo por error como "ruido".
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
    A_PR = readFile(SPIFFS, "/A_PR.txt") == "1";
    Serial.println("[WAKEUP] Rain counter (EXT1) - camino rápido");
    runRainCounter();  // No retorna, entra en deep sleep
    return;
  }

  Serial.printf("Total: %d bytes\n", SPIFFS.totalBytes());
  Serial.printf("Usado: %d bytes\n\n", SPIFFS.usedBytes());

  // Montar SD (VSPI: MISO19/MOSI23/SCK18/CS5)
  initSD();

  // Cargar el ancla de hora real (ver /settime, setTimeBase())
  loadTimeBase();

  createDefaultConfigs();
  {
    float f = readFile(SPIFFS, "/batCal.txt").toFloat();
    batCalFactor = (f >= 0.5f && f <= 1.5f) ? f : 1.0f;
    Serial.printf("[BAT] Factor de calibración: %.4f\n", batCalFactor);
  }

  // Leer configuración
  readConfig();

  // Encendido en frío (batería reconectada, botón EN, flasheo por USB):
  // el operador está presente, probablemente tras corregir la alimentación.
  // Limpiar el backoff para que este ciclo intente enviar de inmediato en
  // vez de heredar la racha de fallos/brownouts de antes del corte.
  if (rst == ESP_RST_POWERON || rst == ESP_RST_EXT || rst == ESP_RST_SW) {
    if (netFailStreak != 0) {
      Serial.printf("[POWER] Arranque en frío: backoff reiniciado (racha previa %d)\n", netFailStreak);
      netFailStreak = 0;
      saveSetting("0", "/netFail.txt");
    }
  }

  if (brownoutBoot) {
    // Contar el brownout como un envío fallido (alimenta el backoff) y
    // llevar un total acumulado para diagnóstico remoto (campo "BO").
    netFailStreak++;
    saveSetting(String(netFailStreak), "/netFail.txt");
    saveSetting(String(readFile(SPIFFS, "/brownouts.txt").toInt() + 1), "/brownouts.txt");
  }

  if (A_PR) {
    pinMode(RAINCOUNT, INPUT);
    esp_sleep_enable_ext1_wakeup(RAINCOUNT_MASK, ESP_EXT1_WAKEUP_ALL_LOW);
    Serial.println("[INIT] Wake por lluvia GPIO39: HABILITADO");
  } else {
    Serial.println("[INIT] Wake por lluvia GPIO39: DESHABILITADO");
  }

  // Verificar razón de wake up
  wakeup_reason();
}

/* ================== MAIN LOOP ==================*/
void loop() {
  // Modo configuración (Access Point)
  if (serverReady) {
    static bool serverStarted = false;
    static unsigned long serverStartTime = 0;

    if (!serverStarted) {
      if (serverStartTime == 0) {
        serverStartTime = millis();
        Serial.println("[LOOP] Esperando estabilización del AP...");
      }

      if (millis() - serverStartTime < 3000) {
        delay(100);
        return;
      }

      if (WiFi.getMode() != WIFI_AP) {
        Serial.println("[ERROR] WiFi AP no está activo!");
        ESP.restart();
        return;
      }

      server.begin();
      serverStarted = true;

      Serial.println("================================");
      Serial.printf("   SSID: %s\n", ssid);
      Serial.printf("   Pass: %s\n", password);
      Serial.println("   URL: http://192.168.4.1");
      Serial.println("================================\n");

    }

    // updateSensCache() ya no bloquea (viento por interrupción, DHT por temporizador
    // interno): se llama en cada vuelta de loop() para lecturas casi en tiempo real.
    updateSensCache();
    configOledTask();

    delay(20);
    return;
  }

  // Modo operación normal
  static unsigned long lastCheck = 0;

  if (!checkMQTT()) {
    if (millis() - lastCheck > 5000) {
      if (connectionType == "WIFI") {
        wifiReconnect();
      } else if (connectionType == "LTE") {
        simReconnect();
      }
      lastCheck = millis();
    }
  }

  if (connectionType == "WIFI") {
    if (!wificlient.connected()) {
      wifiReconnect();
    }
    wificlient.loop();
  } else if (connectionType == "LTE") {
    if (!simclient.connected()) {
      simReconnect();
    }
    simclient.loop();
  }

  // Timeout - entrar en deep sleep si no hay actividad
  if ((millis() - previousTime) > 90000) {
    Serial.println("[TIMEOUT] Entrando en deep sleep");
    deepSleepSystem();
  }

  delay(50);
}
