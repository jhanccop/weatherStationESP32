/*
 * ============================================================================
 * WEATHER STATION - LilyGO TTGO T-SIM7000G
 * ============================================================================
 * Version: main-sim7000 - HARDWARE T-SIM7000G (esquemático rev 2.1, 2025-12-04)
 * Hardware: LilyGO T-SIM7000G (ESP32 + SIM7000G integrado) + DHT21 + MT6701/AS5600
 *           (I2C) + radiación solar RS485 + SD card integrada + anemómetro de
 *           pulsos + pluviómetro (EXT1)
 *
 * Mapa de pines:
 *   SIM7000G : interno de la placa — TX=27 / RX=26 (Serial1), PWRKEY=4.
 *              El modem se alimenta directo de la batería (sin MOSFET): se
 *              enciende/apaga por pulso de PWRKEY y AT+CPOWD.
 *   vext1    : sensores (DHT, viento, encoder I2C, ADS1115), MOSFET canal P
 *              AO3401 controlado por GPIO18 (vext_ctrl_1, LOW=ON, HIGH=OFF)
 *   vext2    : boost 3.7→5V del sensor de radiación + módulo RS485, MOSFET
 *              canal P AO3401 controlado por GPIO19 (vext_ctrl_2, LOW=ON, HIGH=OFF)
 *   RS485    : TXD=GPIO32 / RXD=GPIO33 (Serial2, UART propio, ya no compartido con el modem)
 *   SD card  : integrada en la placa — MISO=2 MOSI=15 SCK=14 CS=13
 *   Lluvia   : GPIO34 (entrada solo, pull-up externo 10k a 3V3) - wakeup por EXT1
 *   Viento   : GPIO35 (velocidad por pulsos, entrada solo, pull-up externo 10k a vext1),
 *              MT6701 o AS5600 por I2C (SDA=21/SCL=22) para dirección — se elige
 *              por software (dirSensorType, /dirSensor.txt)
 *   DHT      : GPIO23 (DHT21/AM2301, pull-up 10k a vext1)
 *   Batería  : GPIO39 (ADC_IN), divisor 390k/100k desde vBat.
 *   Botón AP : GPIO36 (modo configuración si está en LOW al arrancar).
 *              Entrada solo-lectura SIN pull interno: pull-up EXTERNO 10k a
 *              3.3V; el botón conecta a GND al presionarlo.
 */

#define ARDUINOJSON_STRING_LENGTH_SIZE 4

#include <driver/rtc_io.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>

#include "soc/rtc.h"

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

/* ====================== DHT22 TEMP/HUM SETTINGS ======================== */
#include <DHT.h>
#define DHT_PIN  23
#define DHT_TYPE DHT21   // AM2301 = DHT21
DHT dht(DHT_PIN, DHT_TYPE);

/* ====================== STH30 SETTINGS ======================== */
// Alternativa I2C al DHT22: comparte el mismo bus que el MT6701/AS5600
// (SDA=21/SCL=22). tempSensorType ("DHT22"/"SHT30", persistido en
// /tempSensor.txt) selecciona cuál se usa en tiempo de ejecución.
#include "ClosedCube_SHT31D.h"
ClosedCube_SHT31D sht3xd;
#define SHT30_I2C_ADDRESS 0x44
String tempSensorType = "DHT22";

/* ====================== RS485 SETTINGS (RADIACIÓN) ======================== */
// Sensor de radiación solar por RS485 en su propio UART (Serial2, pines
// 32/33): el SIM7000G usa Serial1 en 26/27, así que ya no comparten pines.
// El sensor y el módulo RS485 se alimentan de vext2 (boost 3.7→5V), cuyo
// MOSFET canal P se controla con GPIO19 (vext_ctrl_2), activo en BAJO.
#define RS485_PWR_PIN 19
#define RS485_PWR_ON  LOW
#define RS485_PWR_OFF HIGH
#define RX_PIN 33   // red "RXD" del esquemático
#define TX_PIN 32   // red "TXD" del esquemático
#define BAUD_RATE 4800
#define SERIAL_MODE SERIAL_8N1
#define MAX_BUFFER 64
uint8_t rs485Buffer[MAX_BUFFER];

/* ====================== SD CARD SETTINGS (integrada T-SIM7000G) ======================== */
#include <SPI.h>
#include <SD.h>
// Backend oficial ESP-IDF (empaquetado en el core arduino-esp32) usado solo
// por formatSD() para poder formatear tarjetas sin filesystem FAT válido:
// la librería Arduino SD.h no expone una operación de formateo.
#include <esp_vfs_fat.h>
#include <driver/sdspi_host.h>
#include <sdmmc_cmd.h>
#define SD_MISO 2
#define SD_MOSI 15
#define SD_SCK  14
#define SD_CS   13

bool sdReady = false;
String sdCardType = "N/A";
uint64_t sdTotalMB = 0;
uint64_t sdUsedMB = 0;
uint64_t sdFreeMB = 0;

#define SD_LOG_PATH "/datalog.csv"

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
const char* password = "1234567890";

String RUNssid = "";
String RUNpass = "";

/* ============================== SIM7000G CONFIGURATIONS ============================== */
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 4096

#define SerialMon Serial
#define SerialAT Serial1
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

// LilyGO T-SIM7000G: pines internos de la placa hacia el SIM7000G
#define UART_BAUD           115200
#define PIN_RX2             26   // RX del ESP32 (recibe del TX del modem)
#define PIN_TX2             27   // TX del ESP32 (transmite al RX del modem)
#define MODEM_DTR_PIN       25
#define MODEM_PWRKEY_PIN    4    // reposo HIGH, pulso LOW (ver modemPwrKeyPulse())
#define MODEM_PWRKEY_ON_MS  200  // SIM7000G: encendido ≥72 ms
#define MODEM_PWRKEY_OFF_MS 1500 // SIM7000G: apagado ≥1.2 s

#define SENS_PWR_PIN        18   // vext_ctrl_1 → vext1. MOSFET canal P: LOW=ON, HIGH=OFF


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

/* Device settings */
boolean A_TH = false;
boolean A_WS = false;
boolean A_PR = false;
boolean A_RAD = false;

const char* PARAM_SSID = "ssidString";
const char* PARAM_PASS = "passString";
const char* PARAM_SERVER = "server";
const char* PARAM_PORT = "port";
const char* PARAM_RESET = "reset";
const char* PARAM_CONN_TYPE = "connType";

/* ====================== MQTT ======================== */
const char *broker = "broker.hivemq.com"; //24.199.125.52
const int mqtt_port = 1883;
const char *topicSubscribe = "jhpAI/FAUNA/settings";
const char *topicPublish = "jhpAI/FAUNA/data";

#include <PubSubClient.h>
TinyGsmClient client(modem);
PubSubClient simclient(client);

WiFiClient espClient;
PubSubClient wificlient(espClient);

/* ====================== DEVICE SETTINGS ======================== */
#define pinBattery        39   // ADC1_CH3 (ADC_IN), divisor 390k/100k desde vBat
#define pinWindVelocity   35   // entrada solo, pull-up externo 10k a vext1
#define pinConfigAP       36
#define RAINCOUNT         GPIO_NUM_34  // INPUT-ONLY PIN - pull-up externo 10k a 3V3
#define RAINCOUNT_MASK    (1ULL << 34)

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
// En esta placa la señal del Hall en GPIO35 no alcanza niveles lógicos
// válidos (medido: ~0.9 V con campo, ~1.6 V sin campo, ambos bajo el
// umbral digital del ESP32 ~2.5 V), así que digitalRead() nunca ve un
// flanco. Se lee por ADC con histéresis, igual que el pluviómetro: entre
// ambos umbrales es zona muerta y no cambia el estado.
#define WIND_ADC_FIELD_MV  1100  // por debajo: imán presente (pulso)
#define WIND_ADC_IDLE_MV   1400  // por encima: sin campo (reposo)
#define WIND_SAMPLE_US      500  // período de muestreo del ADC (2 kHz)
// Imprime min/max mV y pulsos de cada WindVelocity() por el monitor serie.
#define WIND_DEBUG true
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
  String rad = "--";
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
<div class="sb"><div class="sl">&#x2600; Radiación</div><div class="sv" id="rad">--</div></div>
</div>
<div class="ps">Última actualización: <span id="upd">--</span> (cada 1 s, local)</div>
</div>
<div class="sc">
<h3>&#x1F6E0; Sensores</h3>
<form action="/get">
<label><input type="checkbox" name="A_TH" value="1" %A_TH_CHECKED%>Temperatura y Humedad</label>
<label><input type="checkbox" name="A_PR" value="1" %A_PR_CHECKED%>Precipitación</label>
<label><input type="checkbox" name="A_WS" value="1" %A_WS_CHECKED%>Velocidad y Dirección del Viento</label>
<label><input type="checkbox" name="A_RAD" value="1" %A_RAD_CHECKED%>Radiación Solar (RS485)</label>
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
<label>Módulo de Temperatura/Humedad</label>
<select name="tempSensor">
<option value="DHT22" %DHT22_SELECTED%>DHT22/AM2301 (digital)</option>
<option value="SHT30" %SHT30_SELECTED%>SHT30 (I2C)</option>
</select>
<input type="submit" value="Guardar Módulo">
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

<div class="sc" id="lteCard" style="%LTE_CARD_STYLE%">
<h3>&#x1F4E1; Diagnóstico LTE (SIM7000G)</h3>
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
function u(){if(busy)return;fetch('/sensors').then(r=>r.json()).then(d=>{document.getElementById('temp').innerText=d.temp!='NA'?d.temp+'°C':'--';document.getElementById('hum').innerText=d.hum!='NA'?d.hum+'%%':'--';document.getElementById('rain').innerText=d.rain!='NA'?d.rain+' mm':'--';document.getElementById('rainStatus').innerText=d.rainStatus!='NA'?d.rainStatus:'--';document.getElementById('wvel').innerText=d.wvel!='NA'?d.wvel+' m/s':'--';document.getElementById('wdir').innerText=d.wdir!='NA'?d.wdir+'°':'--';document.getElementById('rad').innerText=d.rad!='NA'?d.rad:'--';document.getElementById('bat').innerText=d.bat||'--';var rc=document.getElementById('rainCounterVal');if(rc)rc.innerText=d.rainCount!='NA'?d.rainCount:'--';document.getElementById('upd').innerText=new Date().toLocaleTimeString();}).catch(e=>console.log(e));}
u();setInterval(u,1000);
function syncTime(){fetch('/settime?epoch='+Math.floor(Date.now()/1000)).then(r=>r.json()).then(d=>{var tb=document.getElementById('timeBase');if(tb&&d.ok)tb.innerText=d.timestamp+' UTC (ancla)';}).catch(e=>console.log(e));}
syncTime();
function formatSD(){if(busy)return;if(!confirm('¿Formatear la tarjeta SD? Se borrarán todos los datos.'))return;setBusy(true);var r=document.getElementById('sdResult');r.innerText='Formateando...';fetch('/formatsd').then(r=>r.json()).then(d=>{r.innerText=d.msg;}).catch(e=>{r.innerText='Error de comunicación con el dispositivo: '+e;}).finally(()=>setBusy(false));}
function downloadSD(name){window.location.href=name?('/downloadsd?file='+encodeURIComponent(name)):'/downloadsd';}
function logNow(){if(busy)return;setBusy(true);var r=document.getElementById('sdResult');r.innerText='Leyendo sensores y guardando... (puede tardar unos segundos)';fetch('/lognow').then(r=>r.json()).then(d=>{r.innerText=d.msg;setBusy(false);loadDataTable();}).catch(e=>{r.innerText='Error de comunicación con el dispositivo: '+e;setBusy(false);});}
function loadSDFileList(){if(busy)return;setBusy(true);var w=document.getElementById('sdFileListWrap');w.innerHTML='<div class="ps" style="padding:8px">Cargando...</div>';fetch('/sdlist').then(r=>r.json()).then(d=>{if(!d.files||d.files.length==0){w.innerHTML='<div class="ps" style="padding:8px">Sin archivos en la SD.</div>';return;}var html='<table><thead><tr><th>Archivo</th><th>Tamaño</th><th></th></tr></thead><tbody>';d.files.forEach(function(f){var kb=(f.size/1024).toFixed(1);html+='<tr><td>'+f.name+'</td><td>'+kb+' KB</td><td><button type="button" class="btn br" style="padding:4px 10px" onclick="downloadSD(\''+f.name+'\')">&#x2B07;</button></td></tr>';});html+='</tbody></table>';w.innerHTML=html;}).catch(e=>{w.innerHTML='<div class="ps" style="padding:8px">Error de comunicación: '+e+'</div>';}).finally(()=>setBusy(false));}
function loadDataTable(){var w=document.getElementById('dataTableWrap');w.innerHTML='<div class="ps" style="padding:8px">Cargando...</div>';fetch('/viewdata').then(r=>r.text()).then(t=>{var lines=t.split('\n').filter(l=>l.length>0);if(lines.length<2){w.innerHTML='<div class="ps" style="padding:8px">Sin datos registrados aún.</div>';return;}var head=lines[0].split(',');var html='<table><thead><tr>';head.forEach(h=>html+='<th>'+h+'</th>');html+='</tr></thead><tbody>';for(var i=lines.length-1;i>=1;i--){var cols=lines[i].split(',');html+='<tr>';cols.forEach(c=>html+='<td>'+c+'</td>');html+='</tr>';}html+='</tbody></table>';w.innerHTML=html;}).catch(e=>{w.innerHTML='<div class="ps" style="padding:8px">Error de comunicación: '+e+'</div>';});}
function lteTest(){if(busy)return;setBusy(true);var r=document.getElementById('lteResult');r.innerText='Probando... (enciende el modem, el registro en red puede tardar varios minutos)';fetch('/ltetest').then(r=>r.json()).then(d=>{var lines=[];if(d.modemInfo)lines.push('Modem: '+d.modemInfo);if(d.signal!=null)lines.push('Señal (CSQ): '+d.signal+(d.signal==99?' (desconocida)':''));if(d.simReady!=null)lines.push('SIM lista: '+(d.simReady?'sí':'no'));if(d.networkRegistered!=null)lines.push('Registrado en red: '+(d.networkRegistered?'sí':'no'));if(d.operator)lines.push('Operador: '+d.operator);if(d.gprsConnected!=null)lines.push('GPRS/Internet: '+(d.gprsConnected?'sí':'no'));if(d.ip)lines.push('IP: '+d.ip);lines.push(d.msg||(d.ok?'OK':'Error'));document.getElementById('lteResult').innerHTML=lines.join('<br>');}).catch(e=>{r.innerText='Error de comunicación con el dispositivo: '+e;}).finally(()=>setBusy(false));}
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
void deepSleepSystem();
void setting();
void setupServerRoutes();
String vBat();
void runRainCounter();
void readDHT22(float &temperature, float &humidity, bool &success);
void readSHT30(float &temperature, float &humidity, bool &success);
void initSHT30();
void initRS485();
int radRS485();
int readRS485Radiation();
void deinitRS485();
float WindDirection();
float WindVelocity();
void modemPowerOn();
void modemPowerOff();
void modemRestart();
void modemLinesOff();
bool waitForLTENetwork();
void rs485LinesOff();
void callback(char *topic, byte *payload, unsigned int length);
void wifiReconnect();
void simReconnect();
bool checkMQTT();
void wakeup_reason();
void init_running();
void readConfig();
void initDHT22();
void initMT6701();
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
  } else if (var == "A_RAD_CHECKED") {
    return readFile(SPIFFS, "/A_RAD.txt") == "1" ? "checked" : "";
  } else if (var == "DHT22_SELECTED") {
    String tempSensor = readFile(SPIFFS, "/tempSensor.txt");
    return (tempSensor == "SHT30") ? "" : "selected";
  } else if (var == "SHT30_SELECTED") {
    String tempSensor = readFile(SPIFFS, "/tempSensor.txt");
    return (tempSensor == "SHT30") ? "selected" : "";
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

  // Liberar el bus SPI que usa la librería Arduino SD antes de remontar
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
    f.println("timestamp,epoch_s,mac,conn,T,H,WV,WD,RA,vB");
  }

  // 256 se quedaba corto y descartaba el registro entero (DeserializationError
  // NoMemory): a diferencia de DynamicJsonDocument docOut(384) en
  // readSensors() (que solo tiene que ENSAMBLAR el JSON), este deserializeJson()
  // parte de un String inmutable y por eso duplica en el pool tanto las claves
  // como cada valor string (mac, timestamp ISO, "DATALOGER"/"error", etc.) —
  // ese payload real ronda ~300 bytes con los 10 campos actuales. 512 deja
  // margen para nombres de conexión o timestamps un poco más largos sin
  // volver a quedarse sin memoria.
  StaticJsonDocument<512> doc;
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
void setTimeBase(uint64_t epochSeconds) {
  timeBaseEpoch = epochSeconds;
  timeBaseRtc = getRtcTimeSeconds();
  timeBaseSet = true;
  writeFile(SPIFFS, "/timeBaseEpoch.txt", String((unsigned long)timeBaseEpoch).c_str());
  writeFile(SPIFFS, "/timeBaseRtc.txt", String((unsigned long)timeBaseRtc).c_str());
  Serial.printf("[TIME] Ancla actualizada: epoch=%llu rtc=%llu\n", timeBaseEpoch, timeBaseRtc);
}

// Hora real estimada "ahora mismo": ancla + segundos de RTC transcurridos
// desde que se guardó. Si nunca se estableció el ancla, devuelve 0 (el
// llamador debe usar formatTimestamp/logToSD con ese caso en mente).
uint64_t getEpochNow() {
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

/* ========================= DEEP SLEEP ========================= */

void deepSleepSystem() {
  Serial.println("\n[SLEEP] Preparando deep sleep...");

  // ===== APAGAR SENSORES (MOSFET canal P, HIGH=OFF) =====
  gpio_hold_dis((gpio_num_t)SENS_PWR_PIN);
  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, HIGH);
  gpio_hold_en((gpio_num_t)SENS_PWR_PIN);

  // ===== APAGAR SENSOR DE RADIACIÓN / vext2 (MOSFET canal P, HIGH=OFF) =====
  gpio_hold_dis((gpio_num_t)RS485_PWR_PIN);
  pinMode(RS485_PWR_PIN, OUTPUT);
  digitalWrite(RS485_PWR_PIN, RS485_PWR_OFF);
  gpio_hold_en((gpio_num_t)RS485_PWR_PIN);

  // ===== PWRKEY en reposo y UART del modem en alta impedancia =====
  // El SIM7000G no tiene MOSFET: ya quedó apagado por AT+CPOWD en
  // modemPowerOff() (si se encendió este ciclo). Aquí solo se asegura que
  // PWRKEY no genere un pulso espurio durante el sleep.
  modemLinesOff();

  // TX/RX del RS485 en LOW: vext2 está cortado (ver deinitRS485()).
  rs485LinesOff();

  // Leer tiempo de sleep
  TIME_TO_SLEEP = readFile(SPIFFS, "/timesleep.txt").toInt();
  if (TIME_TO_SLEEP < 60) TIME_TO_SLEEP = 300;

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

  // Configurar wake ups
  esp_sleep_enable_timer_wakeup(sleepSeconds * uS_TO_S_FACTOR);

  // habilita solo si esta activado el sensor de lluvia
  if (A_PR) {
    esp_sleep_enable_ext1_wakeup(RAINCOUNT_MASK, ESP_EXT1_WAKEUP_ALL_LOW);
    Serial.println("[SLEEP] Wake por lluvia: HABILITADO");
  } else {
    Serial.println("[SLEEP] Wake por lluvia: DESHABILITADO");
  }

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
    doc["rad"]  = sensCache.rad;
    doc["bat"]  = sensCache.bat;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Diagnóstico LTE bajo demanda (bloqueante, botón manual): solo tiene
  // sentido si connType=LTE, ya que enciende el modem SIM7000G por PWRKEY.
  server.on("/ltetest", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (connectionType != "LTE") {
      request->send(400, "application/json", "{\"ok\":false,\"msg\":\"Modo LTE no seleccionado\"}");
      return;
    }

    Serial.println("[LTE TEST] Iniciando diagnóstico...");
    StaticJsonDocument<384> doc;
    doc["ok"] = false;

    // Este handler corre en el hilo "async_tcp" (el mismo que atiende TODAS
    // las peticiones HTTP), que tiene su propio watchdog (~5 s por defecto).
    // modem.waitForNetwork()/gprsConnect() bloquean muchos segundos sin
    // devolver el control, así que sin esto el watchdog aborta y reinicia
    // el equipo a medias (ver el mismo problema documentado en /lognow más
    // abajo). Se desuscribe esta tarea del watchdog mientras dura el
    // diagnóstico bloqueante y se vuelve a suscribir antes de responder.
    esp_task_wdt_delete(NULL);

    modemPowerOn();

    if (!modem.init()) {
      modemRestart();
      delay(2000);
      if (!modem.init()) {
        doc["msg"] = "El modem SIM7000G no responde";
        modemPowerOff();
        esp_task_wdt_add(NULL);
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
        return;
      }
    }

    doc["modemInfo"] = modem.getModemInfo();

    int16_t csq = modem.getSignalQuality();
    doc["signal"] = csq;
    doc["simReady"] = modem.getSimStatus() == SIM_READY;

    bool netOk = waitForLTENetwork();
    doc["networkRegistered"] = netOk;

    if (!netOk) {
      doc["msg"] = "Sin registro en la red celular (revisar SIM/cobertura/antena)";
      modemPowerOff();
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
    }

    modemPowerOff();
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
    else if (request->hasParam("tempSensor")) {
      String val = request->getParam("tempSensor")->value();
      writeFile(SPIFFS, "/tempSensor.txt", val.c_str());
      tempSensorType = val;
      Serial.println("[SAVE] tempSensor: " + val);
    }
    else if (request->hasParam("period")) {
      String period = request->getParam("period")->value();
      writeFile(SPIFFS, "/timesleep.txt", period.c_str());
      TIME_TO_SLEEP = period.toInt();
      Serial.println("[SAVE] Period: " + period + " seg");
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
      writeFile(SPIFFS, "/A_RAD.txt", request->hasParam("A_RAD") ? "1" : "0");
      Serial.println("[SAVE] Configuración de sensores guardada");
    }

    request->redirect("/");
  });

  server.onNotFound(notFound);
}

/* ====================== SETTING MODE ======================== */
void setting() {
  Serial.println("\n[SETTING MODE] Iniciando...");

  // Detener I2C
  Wire.end();

  // Montar SD integrada (MISO2/MOSI15/SCK14/CS13)
  initSD();

  // Cargar el ancla de hora real (ver /settime, setTimeBase())
  loadTimeBase();

  // Desconectar WiFi completamente
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500);

  // Configurar modo AP
  Serial.println("[SETTING] Configurando Access Point...");
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
    delay(2000);
    ESP.restart();
    return;
  }

  // Esperar estabilización del AP
  delay(2000);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("[OK] AP activo en: ");
  Serial.println(IP);

  // Configurar rutas HTTP (función separada)
  Serial.println("[SETTING] Configurando rutas HTTP...");
  setupServerRoutes();

  Serial.println("[SETTING] Rutas configuradas. Listo para iniciar servidor...");
  serverReady = true;
}

/* ====================== BATTERY VOLTAGE ======================== */
String vBat() {
  // Divisor: R_top=390k (R8), R_bottom=100k (R10) → V_bat = V_adc × (390+100)/100
  // analogReadMilliVolts() aplica la calibración de fábrica (eFuse) del
  // ADC, más lineal que la conversión cruda raw*3.3/4095.
  float v_adc = analogReadMilliVolts(pinBattery) / 1000.0f;

  // Calibración de 2 puntos contra multímetro, pendiente de medir en este
  // hardware: SLOPE=4.9 (relación teórica del divisor) y OFFSET=0 hasta
  // entonces. Con 390k la impedancia del divisor es alta para el ADC,
  // así que es esperable que haga falta ajustar ambos valores.
  const float BAT_CAL_SLOPE  = 4.9f;
  const float BAT_CAL_OFFSET = 0.0f;
  float voltage = v_adc * BAT_CAL_SLOPE + BAT_CAL_OFFSET;

  return String(voltage, 2);
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

void runRainCounter() {
  Serial.println("\n[RAIN] ===== PULSO DE LLUVIA =====");

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

  if (intervalOk) {
    int rain = readFile(SPIFFS, "/ssidRainCounter.txt").toInt();
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

  // Calcular tiempo restante
  TIME_TO_SLEEP = readFile(SPIFFS, "/timesleep.txt").toInt();
  if (TIME_TO_SLEEP < 60) TIME_TO_SLEEP = 300;

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
    // período completo aquí, un pin de lluvia ruidoso (ver notas
    // arriba) que siga generando wakeups EXT1 más seguido que
    // TIME_TO_SLEEP puede postergar la lectura/envío real de sensores
    // indefinidamente — el dispositivo "nunca sale" del ciclo dormir/
    // despertar por lluvia y no llega a reportar. En vez de eso, se
    // ejecuta el ciclo de operación pendiente ahora mismo.
    Serial.println("[RAIN] Período vencido durante el wake por lluvia: ejecutando ciclo de operación pendiente");
    cycle_active = false;
    init_running();  // No retorna, entra en deep sleep
    return;
  }

  remaining = TIME_TO_SLEEP - elapsed;
  if (remaining < 10) remaining = 10;

  Serial.printf("[RAIN] Restante: %d seg (%d min)\n", remaining, remaining / 60);

  // ===== APAGAR TODO (MOSFET canal P, HIGH=OFF) =====
  // PWRKEY en reposo (ver modemLinesOff()); el modem no se enciende en
  // este camino, así que no hace falta AT+CPOWD.
  modemLinesOff();

  gpio_hold_dis((gpio_num_t)SENS_PWR_PIN);
  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, HIGH);
  gpio_hold_en((gpio_num_t)SENS_PWR_PIN);

  gpio_hold_dis((gpio_num_t)RS485_PWR_PIN);
  pinMode(RS485_PWR_PIN, OUTPUT);
  digitalWrite(RS485_PWR_PIN, RS485_PWR_OFF);
  gpio_hold_en((gpio_num_t)RS485_PWR_PIN);

  // Habilitar hold global
  gpio_deep_sleep_hold_en();

  // Wake ups
  esp_sleep_enable_timer_wakeup((uint64_t)remaining * uS_TO_S_FACTOR);
  if (A_PR) {
    esp_sleep_enable_ext1_wakeup(RAINCOUNT_MASK, ESP_EXT1_WAKEUP_ALL_LOW);
  }

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

void readDHT22(float &temperature, float &humidity, bool &success) {
  success = false;
  temperature = -999;
  humidity = -999;

  float t = dht.readTemperature();
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

/* ====================== READ SHT30 ======================== */
// Alternativa I2C al DHT22 (mismo bus que el MT6701/AS5600, SDA=21/SCL=22).
// Rango físico del SHT30 según datasheet, mismo criterio de descarte que
// DHT22_TEMP_MIN/MAX arriba.
#define SHT30_TEMP_MIN -40.0f
#define SHT30_TEMP_MAX 125.0f
#define SHT30_HUM_MIN 0.0f
#define SHT30_HUM_MAX 100.0f

void readSHT30(float &temperature, float &humidity, bool &success) {
  success = false;
  temperature = -999;
  humidity = -999;

  SHT31D result = sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_LOW, SHT3XD_MODE_CLOCK_STRETCH, 50);

  if (result.error != SHT3XD_NO_ERROR) {
    Serial.printf("[SHT30] Error de lectura: %d\n", result.error);
    return;
  }

  if (result.t < SHT30_TEMP_MIN || result.t > SHT30_TEMP_MAX ||
      result.rh < SHT30_HUM_MIN || result.rh > SHT30_HUM_MAX) {
    Serial.printf("[SHT30] Lectura fuera de rango, descartada (T=%.2f H=%.2f)\n", result.t, result.rh);
    return;
  }

  temperature = result.t;
  humidity    = result.rh;
  success     = true;
}

/* ====================== READ LEVEL RS485 (RADIACIÓN) ======================== */
// El sensor de radiación es esclavo Modbus RTU (dirección 0x01): no envía
// nada por su cuenta, hay que interrogarlo con una trama de consulta
// (function code 0x03, leer 4 registros desde 0x0000) y leer su respuesta.
// Esto es lo que causaba "RAD":"error" siempre: la implementación anterior
// solo escuchaba, sin escribir la consulta primero.
// Enciende vext2 (GPIO19 en BAJO, MOSFET canal P) y abre Serial2 sobre
// los pines RS485 (32/33), independientes del UART del SIM7000G.
// El manual del CWT-SI especifica "Response time: <=10S": tras energizarlo
// en frío, la medición óptica/ADC interna del sensor tarda hasta 10 s en
// estabilizarse. Se confirmó que 10 s FIJOS de espera sí funcionan, pero en
// la práctica el sensor suele estabilizarse antes — para un equipo a
// batería con deep sleep, cada segundo despierto de más cuenta, así que en
// vez de esperar siempre el máximo del datasheet, se sondea temprano y
// seguido y se corta en cuanto la lectura deja de cambiar (ver
// readRS485Radiation() más abajo). RS485_POWERUP_SETTLE_MS queda solo como
// tope de seguridad por si el sensor nunca llega a estabilizarse.
#define RS485_POWERUP_SETTLE_MS 10000
// Cada cuánto se sondea mientras se espera que la lectura se estabilice.
#define RS485_POLL_INTERVAL_MS 500
// Nº de lecturas válidas consecutivas e iguales para darlas por estables.
#define RS485_STABLE_READS 2

// Activa el volcado detallado (bytes crudos TX/RX, detección de eco,
// verificación de CRC) por el monitor serie. Ya diagnosticado y
// confirmado funcionando — se deja en false por defecto para no gastar
// tiempo de CPU/UART imprimiendo en cada ciclo normal; poner en true solo
// si hace falta volver a depurar.
#define RS485_DEBUG false

void rs485LinesOff() {
  gpio_hold_dis((gpio_num_t)TX_PIN);
  gpio_hold_dis((gpio_num_t)RX_PIN);
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);
  digitalWrite(RX_PIN, LOW);
  gpio_hold_en((gpio_num_t)TX_PIN);
  gpio_hold_en((gpio_num_t)RX_PIN);
}

void initRS485() {
  gpio_hold_dis((gpio_num_t)RS485_PWR_PIN);
  pinMode(RS485_PWR_PIN, OUTPUT);
  digitalWrite(RS485_PWR_PIN, RS485_PWR_ON);

  gpio_hold_dis((gpio_num_t)TX_PIN);
  gpio_hold_dis((gpio_num_t)RX_PIN);

  Serial2.begin(BAUD_RATE, SERIAL_MODE, RX_PIN, TX_PIN);
}

// CRC16 Modbus RTU estándar (polinomio 0xA001), usado para verificar que la
// trama recibida no esté corrompida por ruido en el bus antes de confiar en
// su contenido — clave para diferenciar "el sensor respondió basura" de
// "hubo un error de cableado/timing y esto ni siquiera es una trama real".
uint16_t modbusCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void rs485HexDump(const char *label, const uint8_t *buf, int len) {
  if (!RS485_DEBUG) return;
  String hex;
  for (int i = 0; i < len; i++) {
    char b[4];
    snprintf(b, sizeof(b), "%02X ", buf[i]);
    hex += b;
  }
  Serial.printf("[RS485][DEBUG] %s (%d bytes): %s\n", label, len, hex.c_str());
}

int radRS485() {
  while (Serial2.available()) Serial2.read();  // limpiar cualquier byte residual

  // Consulta Modbus RTU documentada en el manual del sensor (CWT-SI):
  // esclavo 0x01, function 0x03 (read holding registers), registro 0000H
  // (irradiancia solar), 1 solo registro, CRC16 0x0A84.
  uint8_t command[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
  rs485HexDump("TX", command, sizeof(command));
  Serial2.write(command, sizeof(command));
  Serial2.flush();

  int len = 0;
  unsigned long start = millis();

  // Ventana esperando la respuesta del sensor: a 4800 baudios una trama de
  // 7 bytes tarda ~15 ms en el aire, 300 ms deja margen de sobra para el
  // turnaround del esclavo sin desperdiciar tiempo despierto en cada sondeo.
  while (millis() - start < 300 && len < MAX_BUFFER) {
    if (Serial2.available()) {
      rs485Buffer[len++] = Serial2.read();
    }
  }

  rs485HexDump("RX crudo", rs485Buffer, len);

  if (len == 0) {
    Serial.println("[RS485] Sin respuesta del sensor de radiación (revisar cableado 485+/485- o alimentación 10-30V del sensor)");
    return -1;
  }

  // Eco del propio bus: en RS485 de 2 hilos sin control de dirección por
  // hardware, lo que el ESP32 transmite reaparece primero en la línea RX
  // (es el mismo par de cables). Si los primeros bytes recibidos coinciden
  // exactamente con la consulta enviada, hay que descartarlos y quedarse
  // con lo que venga después — si no, se termina "leyendo" la propia
  // consulta como si fuera la respuesta del sensor (esto es lo que
  // probablemente está pasando: la consulta empieza con 01 03 00 00 00,
  // y tomar los bytes 3-4 de ESO da 00 00 = 0, exactamente el síntoma
  // reportado de "siempre lee 0" sin importar la luz que reciba el sensor).
  uint8_t *frame = rs485Buffer;
  int frameLen = len;
  bool echoStripped = false;
  if (len >= (int)sizeof(command) && memcmp(rs485Buffer, command, sizeof(command)) == 0) {
    frame = rs485Buffer + sizeof(command);
    frameLen = len - sizeof(command);
    echoStripped = true;
    Serial.println("[RS485] Eco de la consulta detectado y descartado");
    rs485HexDump("RX tras quitar eco", frame, frameLen);
  }

  if (frameLen == 0) {
    Serial.println(echoStripped
      ? "[RS485] Solo se recibió el eco, el sensor no respondió (revisar 485+/485- o alimentación)"
      : "[RS485] Sin datos utilizables");
    return -1;
  }

  // Excepción Modbus: el esclavo devuelve function|0x80 + un código de error
  // de 1 byte en vez de los datos pedidos (p.ej. dirección de registro
  // inválida). Distinto de un fallo de comunicación: aquí SÍ hay diálogo,
  // pero el sensor está rechazando la consulta.
  if (frameLen >= 3 && (frame[1] & 0x80)) {
    Serial.printf("[RS485] El sensor respondió una excepción Modbus: función=0x%02X código=0x%02X\n",
                  frame[1], frame[2]);
    return -1;
  }

  if (frameLen < 5 || frame[0] != 0x01 || frame[1] != 0x03) {
    Serial.printf("[RS485] Respuesta inválida (%d bytes, addr=0x%02X func=0x%02X)\n",
                  frameLen, frame[0], frame[1]);
    return -1;
  }

  // Verificar CRC16 de la trama de respuesta: [addr][func][byteCount][data...][crcLo][crcHi]
  int byteCount = frame[2];
  int expectedFrameLen = 3 + byteCount + 2;
  if (frameLen < expectedFrameLen) {
    Serial.printf("[RS485] Trama incompleta (%d bytes, se esperaban %d)\n", frameLen, expectedFrameLen);
    return -1;
  }
  uint16_t crcCalc = modbusCRC16(frame, 3 + byteCount);
  uint16_t crcRecv = frame[3 + byteCount] | (frame[3 + byteCount + 1] << 8);
  if (crcCalc != crcRecv) {
    Serial.printf("[RS485] CRC inválido (calculado=0x%04X recibido=0x%04X) — trama corrupta por ruido en el bus\n",
                  crcCalc, crcRecv);
    return -1;
  }

  int value = (frame[3] << 8) | frame[4];
  Serial.printf("[RS485] Radiación leída: %d W/m²\n", value);
  return value;
}

// Sondea el sensor cada RS485_POLL_INTERVAL_MS y corta en cuanto obtiene
// RS485_STABLE_READS lecturas válidas seguidas e IGUALES: eso es indicio de
// que el sensor ya terminó su arranque en frío y está reportando su valor
// real, en vez de esperar siempre el máximo de 10 s del datasheet. Si nunca
// se estabiliza (o el bus falla), corta por seguridad a los
// RS485_POWERUP_SETTLE_MS y devuelve la última lectura válida que haya
// tenido (o -1 si nunca hubo ninguna).
int readRS485Radiation() {
  unsigned long t0 = millis();
  int lastValue = -1;
  int stableCount = 0;

  while (millis() - t0 < RS485_POWERUP_SETTLE_MS) {
    int value = radRS485();

    if (value >= 0) {
      if (value == lastValue) {
        stableCount++;
        if (stableCount >= RS485_STABLE_READS) {
          Serial.printf("[RS485] Estable tras %lu ms: %d W/m²\n", millis() - t0, value);
          return value;
        }
      } else {
        stableCount = 1;
        lastValue = value;
      }
    } else {
      stableCount = 0;
    }

    delay(RS485_POLL_INTERVAL_MS);
  }

  if (lastValue >= 0) {
    Serial.printf("[RS485] Tope de calentamiento (%d ms) sin estabilizar, se usa la última lectura válida: %d W/m²\n",
                  RS485_POWERUP_SETTLE_MS, lastValue);
  } else {
    Serial.println("[RS485] Tope de calentamiento agotado sin ninguna lectura válida");
  }
  return lastValue;
}

// Apaga vext2 y deja TX/RX del RS485 en LOW con hold: con el módulo sin
// alimentación, un HIGH (idle de UART) lo retroalimentaría por sus pines.
void deinitRS485() {
  Serial2.end();
  digitalWrite(RS485_PWR_PIN, RS485_PWR_OFF);

  rs485LinesOff();
}

/* ====================== WIND FUNCTIONS ======================== */
// Detección de pulso por ADC con histéresis (ver WIND_ADC_FIELD_MV). Se
// cuenta un pulso solo al pasar de "reposo confirmado" a "campo
// confirmado", con el antirrebote por tiempo de WIND_PULSE_DEBOUNCE_US.
// Devuelve true si esta muestra completó un pulso válido.
bool windSamplePulse(uint32_t mv, bool &idleState, unsigned long &lastPulseMicros) {
  if (mv < WIND_ADC_FIELD_MV) {
    bool pulse = false;
    if (idleState) {
      unsigned long now = micros();
      if (now - lastPulseMicros > WIND_PULSE_DEBOUNCE_US) {
        pulse = true;
        lastPulseMicros = now;
      }
    }
    idleState = false;
    return pulse;
  }
  if (mv > WIND_ADC_IDLE_MV) idleState = true;
  return false;
}

// Contador continuo para modo config: un esp_timer muestrea el ADC cada
// WIND_SAMPLE_US (no se puede usar attachInterrupt, la señal no tiene
// niveles digitales válidos) y updateSensCache() calcula la velocidad.
volatile unsigned long windPulseCountISR = 0;
bool windISRAttached = false;
esp_timer_handle_t windSampleTimer = nullptr;

void windSampleTimerCb(void *) {
  static bool idleState = false;
  static unsigned long lastPulseMicros = 0;
  if (windSamplePulse(analogReadMilliVolts(pinWindVelocity), idleState, lastPulseMicros)) {
    windPulseCountISR++;
  }
}

/* ====================== RAIN LIVE COUNTER (MODO CONFIG) ================ */
// runRainCounter() solo cuenta pulsos al despertar por EXT1 (deep sleep);
// en modo config el equipo no duerme, así que sin esta interrupción la
// precipitación nunca se actualizaba en la UI. Cuenta pulsos igual que el
// pluviómetro real: mismos dos filtros que runRainCounter() (ver ahí para
// el detalle) — confirmar el nivel LOW un par de ms y exigir 300 ms
// mínimo entre pulsos válidos.
// NOTA HW: GPIO34 es input-only y el ESP32 NO tiene pull-up/down interno
// en ese pin; en esta placa lo provee R5 (10k a 3V3).
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
  // Ventana de 3 s muestreando el voltaje del Hall por ADC (ver
  // WIND_ADC_FIELD_MV): cada paso del imán (campo → reposo) es un pulso.
  // idleState arranca en false: si la ventana empieza con el imán encima,
  // primero hay que ver el reposo antes de contar el siguiente pulso.
  pinMode(pinWindVelocity, INPUT);  // GPIO35: sin pull interno, R1 10k a vext1

  int counter = 0;
  bool idleState = false;
  unsigned long lastPulseMicros = micros();
#if WIND_DEBUG
  uint32_t mvMin = 5000, mvMax = 0;
#endif

  unsigned long start = millis();
  while (millis() - start < 3000) {
    uint32_t mv = analogReadMilliVolts(pinWindVelocity);
#if WIND_DEBUG
    if (mv < mvMin) mvMin = mv;
    if (mv > mvMax) mvMax = mv;
#endif
    if (windSamplePulse(mv, idleState, lastPulseMicros)) counter++;
    delayMicroseconds(WIND_SAMPLE_US);
  }

#if WIND_DEBUG
  Serial.printf("[WIND][DEBUG] GPIO%d ADC: min=%lu mV max=%lu mV, pulsos=%d\n", pinWindVelocity,
                (unsigned long)mvMin, (unsigned long)mvMax, counter);
#endif

  float speed = counter * (WIND_SPEED_PER_PULSE / 3.0);
  if (speed > MAX_WIND_SPEED_MS) {
    Serial.printf("[WIND] Lectura descartada por implausible: %.1f m/s (ruido del sensor Hall)\n", speed);
    speed = 0;
  }
  return speed;
}

/* ====================== SIM7000G FUNCTIONS ======================== */
// Secuencias de PWRKEY/DTR, tiempos de espera y reintentos tomados del
// firmware levelSIM7000, que sí logra registrarse en la red con esta placa.
// Recuerda si el modem quedó realmente encendido este ciclo, para no
// intentar un apagado AT (bloqueante hasta 10 s) cuando nunca se encendió
// (p.ej. al entrar a modo config en conexión WiFi).
bool modemPoweredOn = false;

// En la T-SIM7000G el modem se alimenta directo de la batería (no hay
// MOSFET que lo corte): "apagado" es el estado tras AT+CPOWD o un pulso
// largo de PWRKEY. En reposo PWRKEY queda en HIGH con hold, para que no se
// genere un pulso espurio durante el deep sleep. RX/TX/DTR del modem quedan
// como entradas: no se fuerzan a LOW porque la placa tiene adaptadores de
// nivel con pull-ups hacia el modem, y un LOW sostenido consumiría
// corriente a través de ellos todo el sleep.
void modemLinesOff() {
  gpio_hold_dis((gpio_num_t)MODEM_PWRKEY_PIN);
  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
  gpio_hold_en((gpio_num_t)MODEM_PWRKEY_PIN);

  pinMode(PIN_TX2, INPUT);
  pinMode(PIN_RX2, INPUT);
  pinMode(MODEM_DTR_PIN, INPUT);
}

// Pulso LOW en PWRKEY. SIM7000G: ≥72 ms enciende, ≥1.2 s apaga. Un pulso
// corto de encendido no apaga un modem que ya esté encendido.
void modemPwrKeyPulse(uint32_t lowMs) {
  gpio_hold_dis((gpio_num_t)MODEM_PWRKEY_PIN);
  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
  delay(100);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);
  delay(lowMs);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
}

void modemPowerOn() {
  Serial.println("[MODEM] Encendiendo SIM7000G...");

  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);  // DTR LOW: modem despierto

  modemPwrKeyPulse(MODEM_PWRKEY_ON_MS);
  delay(5000);  // esperar arranque completo

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
  if (!modem.testAT(5000)) {
    Serial.println("[MODEM] Sin respuesta AT tras el encendido");
  }
  modemPoweredOn = true;

  Serial.println("[MODEM] SIM7000G encendido");
}

void modemPowerOff() {
  Serial.println("[MODEM] Apagando SIM7000G...");

  if (modemPoweredOn) {
    // Apagado ordenado (AT+CPOWD=1): el modem sigue alimentado por la
    // batería, así que si no se apaga aquí queda registrado en la red y
    // consumiendo corriente durante todo el deep sleep del ESP32. Solo si
    // no responde se usa el pulso largo de PWRKEY: si CPOWD ya lo apagó,
    // ese pulso lo volvería a encender.
    if (!modem.poweroff()) {
      Serial.println("[MODEM] AT+CPOWD sin respuesta, apagando por PWRKEY");
      modemPwrKeyPulse(MODEM_PWRKEY_OFF_MS);
    }
    delay(3000);  // esperar apagado completo
  }

  SerialAT.end();
  modemLinesOff();

  modemPoweredOn = false;

  Serial.println("[MODEM] SIM7000G apagado");
}

void modemRestart() {
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}

// Reinicio "suave" del modem (AT+CFUN=1,1) sin pasar por PWRKEY: primer
// intento de recuperación cuando no hay registro de red.
void modemATReset() {
  Serial.println("[MODEM] Reset por comando AT (AT+CFUN=1,1)...");
  modem.sendAT("+CFUN=1,1");
  modem.waitResponse(10000L);
  delay(8000);  // esperar reinicio interno del modem tras el CFUN

  if (!modem.init()) {
    Serial.println("[MODEM] Sin respuesta tras reset AT");
  }
}

// El registro de red del SIM7000G puede tardar varios minutos (búsqueda de
// banda/operador): 60 s aborta la búsqueda antes de tiempo. 2 intentos de
// hasta 4 min, con reset entre ambos (AT+CFUN) si el primero falla.
bool waitForLTENetwork() {
  const int maxAttempts = 2;
  const uint32_t networkTimeoutMs = 240000L;

  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    int csq = modem.getSignalQuality();
    Serial.printf("[LTE] Intento %d/%d - Señal (CSQ): %d - esperando hasta %lu s...\n",
                  attempt, maxAttempts, csq, networkTimeoutMs / 1000);

    if (modem.waitForNetwork(networkTimeoutMs)) {
      return true;
    }

    Serial.printf("[ERROR] Sin cobertura de red (intento %d/%d)\n", attempt, maxAttempts);
    if (attempt < maxAttempts) modemATReset();
  }
  return false;
}

/* ====================== MQTT CALLBACK ======================== */
// Permite a waitForConfig() salir en cuanto llega la config, en vez de
// esperar siempre los 5 s completos con el modem/WiFi encendidos.
bool configReceivedFlag = false;

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.printf("[CONFIG] Topic recibido: %s\n", topic);

  // Extract MAC from end of topic: jhpAI/FAUNA/settings/AA:BB:CC:DD:EE:FF
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

  StaticJsonDocument<320> docIn;
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
  if (docIn.containsKey("A_RAD"))     A_RAD         = docIn["A_RAD"].as<bool>();
  if (docIn.containsKey("dirSensor")) dirSensorType = docIn["dirSensor"].as<String>() == "AS5600" ? "AS5600" : "MT6701";
  if (docIn.containsKey("tempSensor")) tempSensorType = docIn["tempSensor"].as<String>() == "SHT30" ? "SHT30" : "DHT22";

  saveSetting(status ? "1" : "0",         "/status.txt");
  saveSetting(String((int)TIME_TO_SLEEP), "/timesleep.txt");
  saveSetting(A_TH ? "1" : "0",          "/A_TH.txt");
  saveSetting(A_WS ? "1" : "0",          "/A_WS.txt");
  saveSetting(A_PR ? "1" : "0",          "/A_PR.txt");
  saveSetting(A_RAD ? "1" : "0",         "/A_RAD.txt");
  saveSetting(dirSensorType,             "/dirSensor.txt");
  saveSetting(tempSensorType,            "/tempSensor.txt");

  Serial.printf("[CONFIG] OK status=%d sleep=%llu seg (%llu min) TH=%d WS=%d PR=%d RAD=%d dirSensor=%s tempSensor=%s\n",
                status, TIME_TO_SLEEP, TIME_TO_SLEEP / 60, A_TH, A_WS, A_PR, A_RAD, dirSensorType.c_str(), tempSensorType.c_str());

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
      Serial.println("[WAKEUP] External 0");
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

  DynamicJsonDocument docOut(384);
  docOut["type"] = "weatherStationData";
  docOut["mac"] = macA();
  docOut["conn"] = connectionType;
  docOut["timestamp"] = formatTimestampISO8601(getEpochNow());

  // Encender sensores: MOSFET canal P gate LOW = ON
  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, LOW);
  delay(2500);  // AM2301 necesita ≥2.5 s de arranque frío tras encender v_sensor

  // Iniciar I2C una sola vez — lo usa el MT6701/AS5600 y, si está
  // seleccionado, el SHT30 (comparten el mismo bus SDA=21/SCL=22)
  bool needsI2C = A_WS || (A_TH && tempSensorType == "SHT30");
  if (needsI2C) {
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(50);
  }

  // ===== TEMPERATURA Y HUMEDAD (DHT22 o SHT30) =====
  if (A_TH) {
    float temp, hum;
    bool ok;
    int retries = 0;

    if (tempSensorType == "SHT30") {
      Serial.println("[SENSOR] Leyendo SHT30...");
      initSHT30();

      do {
        readSHT30(temp, hum, ok);
        if (!ok) {
          retries++;
          delay(500);
        }
      } while (!ok && retries < 5);
    } else {
      Serial.println("[SENSOR] Leyendo DHT22...");
      initDHT22();

      do {
        readDHT22(temp, hum, ok);
        if (!ok) {
          retries++;
          delay(2000);  // el DHT22 necesita ~2 s entre lecturas
        }
      } while (!ok && retries < 5);
    }

    if (ok) {
      float tempCal = 0.5480f * temp + 9.0357f;
      float humCal  = 0.4499f * hum + 54.7577f;
      docOut["T"] = String(tempCal, 2);
      docOut["H"] = String(humCal, 2);
      Serial.printf("[SENSOR] Temp: %.2f°C, Hum: %.2f%%\n", tempCal, humCal);
    } else {
      docOut["T"] = "error";
      docOut["H"] = "error";
      Serial.printf("[SENSOR] Error leyendo %s\n", tempSensorType.c_str());
    }
  }

  // ===== VELOCIDAD Y DIRECCIÓN DEL VIENTO =====
  if (A_WS) {
    Serial.println("[SENSOR] Leyendo viento...");

    initMT6701();
    float wv = WindVelocity();
    float wd = WindDirection();

    float wvCal = 0.6448f * wv + 0.5343f;

    docOut["WV"] = String(wvCal, 2);
    docOut["WD"] = String(wd, 1);
    Serial.printf("[SENSOR] Vel: %.2f m/s, Dir: %.1f°\n", wvCal, wd);
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
    // No se resetea aquí: es un contador acumulado persistente, solo se
    // reinicia manualmente desde la vista de configuración (botón
    // "Reiniciar Contador de Lluvia" -> /get?resetRain=1).
  }

  // ===== RADIACIÓN SOLAR (RS485) =====
  // El sensor y el módulo RS485 TTL se alimentan de vext2 (boost 5V),
  // independiente de vext1: initRS485()/deinitRS485() lo encienden y apagan.
  if (A_RAD) {
    Serial.println("[SENSOR] Leyendo radiación (RS485)...");
    initRS485();
    int rad = readRS485Radiation();
    deinitRS485();

    if (rad >= 0) {
      docOut["RAD"] = rad;
      Serial.printf("[SENSOR] Radiación: %d\n", rad);
    } else {
      docOut["RAD"] = "error";
      Serial.println("[SENSOR] Error leyendo radiación");
    }
  }

  // Apagar sensores: MOSFET canal P gate HIGH = OFF
  digitalWrite(SENS_PWR_PIN, HIGH);

  // ===== BATERÍA =====
  String bat = vBat();
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

/* ============MODEM ========== ENVIO LTE (SIM7000G) ======================== */
bool sendDataLTE(String jsonData) {
  Serial.println("\n[LTE] Iniciando envío de datos...");

  // Encender modem
  modemPowerOn();

  // Inicializar modem
  if (!modem.init()) {
    modemRestart();
    delay(2000);
    if (!modem.init()) {
      Serial.println("[ERROR] Modem no responde");
      modemPowerOff();
      return false;
    }
  }

  Serial.println("[LTE] Modem: " + modem.getModemInfo());

  // Recibir la hora de la red (NITZ) al registrarse, para
  // syncTimeFromInternetLTE(); &W la guarda en el modem.
  modem.sendAT(GF("+CLTS=1"));
  modem.waitResponse();
  modem.sendAT(GF("&W"));
  modem.waitResponse();

  // Esperar red (con reintentos y reset si no hay registro)
  Serial.println("[LTE] Esperando red...");
  if (!waitForLTENetwork()) {
    Serial.println("[ERROR] Sin cobertura de red (tras reintentos y reset del modem)");
    modemPowerOff();
    return false;
  }

  Serial.println("[LTE] Red conectada");

  // Conectar GPRS
  Serial.println("[LTE] Conectando GPRS...");
  if (!modem.gprsConnect(RUNapn.c_str())) {
    Serial.println("[ERROR] Fallo conexión GPRS");
    modemPowerOff();
    return false;
  }

  Serial.println("[LTE] GPRS conectado");

  // Configurar MQTT
  String serverIP = readFile(SPIFFS, "/server.txt");
  String topicPub = readFile(SPIFFS, "/topicPub.txt");

  if (serverIP == "") serverIP = broker;
  if (topicPub == "") topicPub = "jhpAI/FAUNA/data";

  simclient.setServer(serverIP.c_str(), mqtt_port);
  simclient.setBufferSize(512);  // el payload puede superar los 256 bytes por defecto

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
    modem.gprsDisconnect();
    modemPowerOff();
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
  flushPendingData(simclient, topicPub);

  // Publicar datos
  bool published = simclient.publish(topicPub.c_str(), jsonData.c_str());

  if (published) {
    Serial.println("[MQTT] Datos publicados correctamente");
    syncTimeFromInternetLTE();
  } else {
    Serial.println("[ERROR] Fallo al publicar");
  }

  simclient.loop();
  delay(100);

  if (published) waitForConfig(simclient);

  // Desconectar
  simclient.disconnect();
  modem.gprsDisconnect();
  modemPowerOff();

  Serial.println("[LTE] Desconectado");

  return published;
}

/* ====================== ENVIO WIFI======================== */
bool sendDataWiFi(String jsonData) {
  Serial.println("\n[WIFI] Iniciando envío de datos...");

  RUNssid = readFile(SPIFFS, "/ssidString.txt");
  RUNpass = readFile(SPIFFS, "/passString.txt");

  if (RUNssid == "") {
    Serial.println("[ERROR] SSID no configurado");
    return false;
  }

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
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  Serial.println("\n[WIFI] Conectado: " + WiFi.localIP().toString());

  // Configurar MQTT
  String serverIP = readFile(SPIFFS, "/server.txt");
  String topicPub = readFile(SPIFFS, "/topicPub.txt");

  if (serverIP == "") serverIP = broker;
  if (topicPub == "") topicPub = "jhpAI/FAUNA/data";

  wificlient.setServer(serverIP.c_str(), mqtt_port);
  wificlient.setBufferSize(512);  // el payload puede superar los 256 bytes por defecto

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
  flushPendingData(wificlient, topicPub);

  // Publicar datos
  bool published = wificlient.publish(topicPub.c_str(), jsonData.c_str());

  if (published) {
    Serial.println("[MQTT] Datos publicados correctamente");
    syncTimeFromInternetWiFi();
  } else {
    Serial.println("[ERROR] Fallo al publicar");
  }

  wificlient.loop();
  delay(100);

  if (published) waitForConfig(wificlient);

  // Desconectar
  wificlient.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  Serial.println("[WIFI] Desconectado");

  return published;
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

    String sensorData = readSensors();

    // Respaldo local en la SD, independientemente de si el envío MQTT funciona
    // (o si no hay envío en absoluto, modo DATALOGER). Usa la hora ancla
    // real-time cargada de /settime (ver logToSD()/getEpochNow()), no
    // depende de tener red disponible en este ciclo.
    logToSD(sensorData);

    if (connectionType == "DATALOGER") {
      // Modo solo-registro local: no se enciende WiFi ni el modem, no hay
      // nada que enviar ni encolar como pendiente — el dato ya quedó en la SD.
      Serial.println("[MODE] Dataloger local (sin envío, solo respaldo en SD)");
    } else {
      bool success = false;

      if (connectionType == "WIFI") {
        Serial.println("[MODE] WiFi");
        success = sendDataWiFi(sensorData);
      } else if (connectionType == "LTE") {
        Serial.println("[MODE] LTE");
        success = sendDataLTE(sensorData);
      }

      if (success) {
        Serial.println("[OK] Datos enviados correctamente");
      } else {
        Serial.println("[ERROR] Fallo al enviar datos");
        appendPendingRecord(sensorData);
      }
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
  String arad = readFile(SPIFFS, "/A_RAD.txt");
  String dirSensor = readFile(SPIFFS, "/dirSensor.txt");
  String tempSensor = readFile(SPIFFS, "/tempSensor.txt");

  A_TH = (ath == "1");
  A_WS = (aws == "1");
  A_PR = (apr == "1");
  A_RAD = (arad == "1");
  dirSensorType = (dirSensor == "AS5600") ? "AS5600" : "MT6701";
  tempSensorType = (tempSensor == "SHT30") ? "SHT30" : "DHT22";

  String apnCfg = readFile(SPIFFS, "/apn.txt");
  RUNapn = (apnCfg == "") ? "claro.pe" : apnCfg;

  Serial.println("---- VALORES CARGADOS ----");
  Serial.printf("A_TH = %s\n", A_TH ? "true" : "false");
  Serial.printf("A_WS = %s\n", A_WS ? "true" : "false");
  Serial.printf("A_PR = %s\n", A_PR ? "true" : "false");
  Serial.printf("A_RAD = %s\n", A_RAD ? "true" : "false");
  Serial.printf("dirSensorType = %s\n", dirSensorType.c_str());
  Serial.printf("tempSensorType = %s\n", tempSensorType.c_str());
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

void initSHT30() {
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);
  sht3xd.begin(SHT30_I2C_ADDRESS);
  delay(50);
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

    if (A_WS) {
      Wire.begin(I2C_SDA, I2C_SCL);
      delay(50);
      initMT6701();

      pinMode(pinWindVelocity, INPUT);  // GPIO35: sin pull interno, R1 10k a vext1
      esp_timer_create_args_t timerArgs = {};
      timerArgs.callback = windSampleTimerCb;
      timerArgs.name = "windSample";
      esp_timer_create(&timerArgs, &windSampleTimer);
      esp_timer_start_periodic(windSampleTimer, WIND_SAMPLE_US);
      windISRAttached = true;
    }

    if (A_TH) {
      if (tempSensorType == "SHT30") {
        initSHT30();
      } else {
        initDHT22();
      }
      lastDHTRead = millis() - 2000;  // permitir una lectura inmediata
    }

    if (A_RAD) {
      // Se abre una sola vez y se deja activo mientras dure el modo config:
      // el RS485 tiene su propio UART (Serial2), independiente del modem.
      initRS485();
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

  // Temp/Hum (DHT22 o SHT30): el sensor solo admite una lectura válida cada ~2 s
  if (A_TH) {
    if (millis() - lastDHTRead >= 2000) {
      float t, h; bool ok;
      if (tempSensorType == "SHT30") {
        readSHT30(t, h, ok);
      } else {
        readDHT22(t, h, ok);
      }
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
      sensCache.wvel = String(speed, 1);
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

  // Radiación (RS485): trama lenta, se sondea cada 15 s (no en cada vuelta
  // de loop()) para no bloquear el resto del refresco de la cache tan seguido.
  static unsigned long lastRadRead = 0;
  if (A_RAD) {
    if (millis() - lastRadRead >= 15000) {
      // El módulo ya lleva rato energizado (se abrió una sola vez arriba),
      // así que aquí no hace falta la espera de calentamiento: una consulta
      // directa basta.
      int rad = radRS485();
      sensCache.rad = (rad >= 0) ? String(rad) : "error";
      lastRadRead = millis();
    }
  } else {
    sensCache.rad = "NA";
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
    {"/A_RAD.txt", "0"},
    {"/dirSensor.txt", "MT6701"},
    {"/tempSensor.txt", "DHT22"},
    {"/connType.txt", "WIFI"},
    {"/apn.txt", "claro.pe"},
    {"/timesleep.txt", "300"},
    {"/ssidRainCounter.txt", "0"},
    {"/ssidString.txt", ""},
    {"/passString.txt", ""},
    {"/server.txt", ""},
    {"/port.txt", "8000"},
    {"/topicPub.txt", "jhpAI/FAUNA/data"},
    {"/topicSub.txt", "jhpAI/FAUNA/settings"},
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

  Serial.println("[INIT] Archivos de configuración verificados\n");
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
  gpio_hold_dis((gpio_num_t)MODEM_PWRKEY_PIN);
  gpio_hold_dis((gpio_num_t)SENS_PWR_PIN);
  gpio_hold_dis((gpio_num_t)RS485_PWR_PIN);
  gpio_hold_dis((gpio_num_t)TX_PIN);
  gpio_hold_dis((gpio_num_t)RX_PIN);

  // ===== ASEGURAR SENSORES APAGADOS Y PWRKEY EN REPOSO =====
  // vext1/vext2: MOSFET canal P, HIGH = OFF. PWRKEY en reposo (HIGH); solo
  // modemPowerOn()/modemPowerOff() generan pulsos LOW.
  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);

  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, HIGH);

  pinMode(RS485_PWR_PIN, OUTPUT);
  digitalWrite(RS485_PWR_PIN, RS485_PWR_OFF);

  // TX/RX del RS485 a LOW por defecto hasta que initRS485() los reconfigure
  // como UART: evita retroalimentar el módulo con vext2 cortado.
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);
  digitalWrite(RX_PIN, LOW);

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

  // Montar SD integrada (MISO2/MOSI15/SCK14/CS13)
  initSD();

  // Cargar el ancla de hora real (ver /settime, setTimeBase())
  loadTimeBase();

  createDefaultConfigs();

  // Leer configuración
  readConfig();

  if (A_PR) {
    pinMode(RAINCOUNT, INPUT);
    esp_sleep_enable_ext1_wakeup(RAINCOUNT_MASK, ESP_EXT1_WAKEUP_ALL_LOW);
    Serial.println("[INIT] Wake por lluvia GPIO34: HABILITADO");
  } else {
    Serial.println("[INIT] Wake por lluvia GPIO34: DESHABILITADO");
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
