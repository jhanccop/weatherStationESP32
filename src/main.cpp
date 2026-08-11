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
 *              Alimentación controlada por MOSFET canal P en GPIO15 (LOW=ON, HIGH=OFF)
 *   Sensores : Alimentación controlada por MOSFET canal P en GPIO2/D2 (LOW=ON, HIGH=OFF)
 *   SD card  : MISO=19 MOSI=23 SCK=18 CS=5 (coincide con el bus VSPI por defecto del ESP32)
 *   Lluvia   : GPIO39 (entrada solo, sin pull interno) - wakeup por EXT1
 *   Viento   : GPIO13/D13 (velocidad por pulsos), MT6701 o AS5600 por I2C (SDA=21/SCL=22)
 *              para dirección — se elige por software (dirSensorType, /dirSensor.txt)
 *   RTC      : DS3231 por I2C (mismo bus, SDA=21/SCL=22, dirección fija 0x68) —
 *              da la marca de tiempo real; mantiene la hora con su propia batería
 *              de respaldo aunque el equipo pierda alimentación por completo.
 *   DHT      : GPIO4 (DHT22)
 *   Batería  : GPIO34 (ADC), divisor 100k/100k.  
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
#define MODEM_PWR_PIN       15   // MOSFET canal P: LOW=ON, HIGH=OFF

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
const char *topicSubscribe = "jhpAI/FAUNA/settings";
const char *topicPublish = "jhpAI/FAUNA/data";

#include <PubSubClient.h>
TinyGsmClient client(modem);
PubSubClient simclient(client);

WiFiClient espClient;
PubSubClient wificlient(espClient);

/* ====================== DEVICE SETTINGS ======================== */
#define pinBattery        34   // ADC1_CH6, divisor 100k/100k
#define pinWindVelocity   13
#define pinConfigAP       36
#define RAINCOUNT         GPIO_NUM_39  // INPUT-ONLY PIN - NO INTERNAL PULLUP
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
<button type="button" class="btn be heavy-btn" onclick="deleteLogFile()">&#x1F5D1; Borrar Archivo de Datos</button>
</div>
<div class="tb" style="margin-top:8px">
<button type="button" class="btn be heavy-btn" onclick="formatSD()">&#x1F5D1; Formatear Tarjeta (completo)</button>
</div>
<div id="sdResult" class="ps">"Guardar Lectura Ahora" agrega una fila real a la SD sin esperar el ciclo de sleep — útil para probar mientras se está en el portal. "Borrar Archivo de Datos" solo borra datalog.csv. "Formatear Tarjeta" borra TODO el contenido de la tarjeta (usar solo si la tarjeta quedó corrupta).</div>
<div class="dt" id="sdFileListWrap" style="margin-top:8px"></div>
</div>

<div class="sc">
<div class="ps">&#x1F551; Hora de referencia: <span id="timeBase">%TIME_BASE%</span></div>
<button type="button" class="btn br heavy-btn" style="width:100%%;margin-top:8px" onclick="syncTime(true)">&#x1F553; Actualizar Reloj RTC (DS3231)</button>
<div id="rtcResult" class="ps">Sincroniza la hora ancla y el RTC DS3231 con el reloj de este PC/celular. También se hace automáticamente al cargar esta página.</div>
</div>

<div class="sc">
<h3>&#x1F4CB; Datos Registrados (últimos 50)</h3>
<button type="button" class="btn br" style="width:100%%" onclick="loadDataTable()">&#x1F504; Cargar / Actualizar Tabla</button>
<div class="dt" id="dataTableWrap"><div class="ps" style="padding:8px">Sin cargar aún.</div></div>
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
function u(){if(busy)return;fetch('/sensors').then(r=>r.json()).then(d=>{document.getElementById('temp').innerText=d.temp!='NA'?d.temp+'°C':'--';document.getElementById('hum').innerText=d.hum!='NA'?d.hum+'%%':'--';document.getElementById('rain').innerText=d.rain!='NA'?d.rain+' mm':'--';document.getElementById('rainStatus').innerText=d.rainStatus!='NA'?d.rainStatus:'--';document.getElementById('wvel').innerText=d.wvel!='NA'?d.wvel+' m/s':'--';document.getElementById('wdir').innerText=d.wdir!='NA'?d.wdir+'°':'--';document.getElementById('bat').innerText=d.bat||'--';var rc=document.getElementById('rainCounterVal');if(rc)rc.innerText=d.rainCount!='NA'?d.rainCount:'--';document.getElementById('upd').innerText=new Date().toLocaleTimeString();}).catch(e=>console.log(e));}
u();setInterval(u,1000);
function syncTime(manual){if(manual){if(busy)return;setBusy(true);}var rr=document.getElementById('rtcResult');if(manual&&rr)rr.innerText='Sincronizando...';fetch('/settime?epoch='+Math.floor(Date.now()/1000)).then(r=>r.json()).then(d=>{var tb=document.getElementById('timeBase');if(tb&&d.ok)tb.innerText=d.timestamp+' UTC (ancla)';if(rr){if(!d.ok)rr.innerText='Error: '+(d.msg||'desconocido');else rr.innerText=(d.ds3231?'RTC DS3231 actualizado a '+d.timestamp+' UTC.':'Ancla de hora actualizada, pero el DS3231 no respondió (revisar cableado I2C).');}}).catch(e=>{if(rr)rr.innerText='Error de comunicación con el dispositivo: '+e;}).finally(()=>{if(manual)setBusy(false);});}
syncTime(false);
function formatSD(){if(busy)return;if(!confirm('¿Formatear TODA la tarjeta SD? Se borrarán todos los datos y archivos.'))return;setBusy(true);var r=document.getElementById('sdResult');r.innerText='Formateando...';fetch('/formatsd').then(r=>r.json()).then(d=>{r.innerText=d.msg;}).catch(e=>{r.innerText='Error de comunicación con el dispositivo: '+e;}).finally(()=>setBusy(false));}
function deleteLogFile(){if(busy)return;if(!confirm('¿Borrar el archivo de datos (datalog.csv)? Se perderán los registros guardados.'))return;setBusy(true);var r=document.getElementById('sdResult');r.innerText='Borrando...';fetch('/deletelog').then(r=>r.json()).then(d=>{r.innerText=d.msg;loadDataTable();}).catch(e=>{r.innerText='Error de comunicación con el dispositivo: '+e;}).finally(()=>setBusy(false));}
function downloadSD(name){window.location.href=name?('/downloadsd?file='+encodeURIComponent(name)):'/downloadsd';}
function logNow(){if(busy)return;setBusy(true);var r=document.getElementById('sdResult');r.innerText='Leyendo sensores y guardando... (puede tardar unos segundos)';fetch('/lognow').then(r=>r.json()).then(d=>{r.innerText=d.msg;setBusy(false);loadDataTable();}).catch(e=>{r.innerText='Error de comunicación con el dispositivo: '+e;setBusy(false);});}
function loadSDFileList(){if(busy)return;setBusy(true);var w=document.getElementById('sdFileListWrap');w.innerHTML='<div class="ps" style="padding:8px">Cargando...</div>';fetch('/sdlist').then(r=>r.json()).then(d=>{if(!d.files||d.files.length==0){w.innerHTML='<div class="ps" style="padding:8px">Sin archivos en la SD.</div>';return;}var html='<table><thead><tr><th>Archivo</th><th>Tamaño</th><th></th></tr></thead><tbody>';d.files.forEach(function(f){var kb=(f.size/1024).toFixed(1);html+='<tr><td>'+f.name+'</td><td>'+kb+' KB</td><td><button type="button" class="btn br" style="padding:4px 10px" onclick="downloadSD(\''+f.name+'\')">&#x2B07;</button></td></tr>';});html+='</tbody></table>';w.innerHTML=html;}).catch(e=>{w.innerHTML='<div class="ps" style="padding:8px">Error de comunicación: '+e+'</div>';}).finally(()=>setBusy(false));}
function loadDataTable(){var w=document.getElementById('dataTableWrap');w.innerHTML='<div class="ps" style="padding:8px">Cargando...</div>';fetch('/viewdata').then(r=>r.text()).then(t=>{var lines=t.split('\n').filter(l=>l.length>0);if(lines.length<2){w.innerHTML='<div class="ps" style="padding:8px">Sin datos registrados aún.</div>';return;}var head=lines[0].split(',');var html='<table><thead><tr>';head.forEach(h=>html+='<th>'+h+'</th>');html+='</tr></thead><tbody>';for(var i=lines.length-1;i>=1;i--){var cols=lines[i].split(',');html+='<tr>';cols.forEach(c=>html+='<td>'+c+'</td>');html+='</tr>';}html+='</tbody></table>';w.innerHTML=html;}).catch(e=>{w.innerHTML='<div class="ps" style="padding:8px">Error de comunicación: '+e+'</div>';});}
function lteTest(){if(busy)return;setBusy(true);var r=document.getElementById('lteResult');r.innerText='Probando... (enciende el modem, puede tardar hasta 20 s)';fetch('/ltetest').then(r=>r.json()).then(d=>{var lines=[];if(d.modemInfo)lines.push('Modem: '+d.modemInfo);if(d.signal!=null)lines.push('Señal (CSQ): '+d.signal+(d.signal==99?' (desconocida)':''));if(d.simReady!=null)lines.push('SIM lista: '+(d.simReady?'sí':'no'));if(d.networkRegistered!=null)lines.push('Registrado en red: '+(d.networkRegistered?'sí':'no'));if(d.operator)lines.push('Operador: '+d.operator);if(d.gprsConnected!=null)lines.push('GPRS/Internet: '+(d.gprsConnected?'sí':'no'));if(d.ip)lines.push('IP: '+d.ip);lines.push(d.msg||(d.ok?'OK':'Error'));document.getElementById('lteResult').innerHTML=lines.join('<br>');}).catch(e=>{r.innerText='Error de comunicación con el dispositivo: '+e;}).finally(()=>setBusy(false));}
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
float WindDirection();
float WindVelocity();
void modemPowerOn();
void modemPowerOff();
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

/* ====================== CALIBRACIÓN POR DISPOSITIVO (MAC) ======================== */
// Coeficientes de calibración (Oficial = slope·lectura_cruda + offset) para T/H/viento,
// usados en readSensors(). Dos equipos concretos ("Sotomayor 1"/"Sotomayor 2") fueron
// calibrados cada uno contra su propia referencia oficial en su ubicación, así que sus
// coeficientes reemplazan (no se suman a) los genéricos por defecto. Cualquier otro MAC
// no listado aquí sigue usando los coeficientes genéricos (comportamiento previo, sin
// calibración por equipo).
struct CalCoeffs {
  float tempSlope, tempOffset;
  float humSlope, humOffset;
  float windSlope, windOffset;
};

CalCoeffs getCalCoeffs() {
  String mac = macA();

  if (mac == "30:76:F5:F7:6E:7C") {  // Sotomayor 1
    return CalCoeffs{ 0.582f, 8.020f, 0.588f, 31.972f, 1.585f, 0.539f };
  }
  if (mac == "30:76:F5:F8:E5:E8") {  // Sotomayor 2
    return CalCoeffs{ 0.90f, 1.63f, 1.09f, -20.37f, 2.38f, -0.62f };
  }

  // Genéricos por defecto (comportamiento previo, sin cambios)
  return CalCoeffs{ 0.5480f, 9.0357f, 0.4499f, 54.7577f, 0.6448f, 0.5343f };
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

// Ventana de fechas plausible para aceptar la lectura del DS3231: no antes de
// que este firmware se haya compilado (con un pequeño margen, por si el reloj
// del equipo aún no se ajustó justo tras flashear) ni más de MAX_YEARS_AHEAD
// después. Visto en campo: el DS3231 a veces devuelve una fecha disparatada
// (epoch de ~2043 en una lectura y ~2028 en la siguiente, sin relación con la
// hora real) por una lectura I2C corrupta o la batería de respaldo fallando.
// Igual que se descartan lecturas de DHT22/viento fuera de su rango físico
// (ver DHT22_TEMP_MIN/MAX, MAX_WIND_SPEED_MS), una fecha implausible del
// DS3231 se descarta en vez de propagarse al log, y se cae al ancla por
// software en su lugar.
bool isPlausibleEpoch(uint64_t epochSeconds) {
  static uint64_t compileEpoch = 0;
  if (compileEpoch == 0) {
    DateTime compileTime(F(__DATE__), F(__TIME__));
    compileEpoch = compileTime.unixtime();
  }
  const uint64_t SLACK_BEFORE_S     = 24ULL * 3600;          // 1 día de margen hacia atrás
  const uint64_t MAX_YEARS_AHEAD_S  = 10ULL * 365 * 24 * 3600;  // 10 años hacia adelante

  uint64_t lowerBound = (compileEpoch > SLACK_BEFORE_S) ? (compileEpoch - SLACK_BEFORE_S) : 0;
  uint64_t upperBound = compileEpoch + MAX_YEARS_AHEAD_S;
  return epochSeconds >= lowerBound && epochSeconds <= upperBound;
}

// Hora real "ahora mismo". Fuente primaria: el DS3231 (hardware, con batería
// de respaldo), validada con isPlausibleEpoch() antes de confiar en ella. Si
// no responde o su lectura es implausible, cae al esquema de ancla por
// software (ancla + segundos de RTC-slow-clock transcurridos desde que se
// guardó). Si tampoco hay ancla, devuelve 0 (el llamador debe usar
// formatTimestamp/logToSD con ese caso en mente).
uint64_t getEpochNow() {
  if (ds3231Ready) {
    uint64_t t = ds3231.now().unixtime();
    if (isPlausibleEpoch(t)) {
      return t;
    }
    Serial.printf("[RTC] DS3231 devolvió una fecha implausible (epoch=%llu), se usa el ancla de respaldo\n",
                  (unsigned long long)t);
  }
  if (!timeBaseSet) return 0;
  uint64_t nowRtc = getRtcTimeSeconds();
  uint64_t elapsed = (nowRtc > timeBaseRtc) ? (nowRtc - timeBaseRtc) : 0;
  return timeBaseEpoch + elapsed;
}

// Descompone epochSeconds (UTC) en fecha/hora calendario sin pasar por
// time_t/gmtime_r: en este framework time_t es de 32 bits con signo, así que
// cualquier epoch por encima de ~2^31 (año 2038, o un valor corrupto/futuro
// que el DS3231 llegue a devolver pese al filtro de isPlausibleEpoch) se
// desbordaba a un timestamp absurdo como "1907-03-24" en vez de mostrar el
// valor real — lo cual hacía imposible distinguir "RTC mal puesto" de "bug de
// formato". Algoritmo de Howard Hinnant (civil_from_days), aritmética entera
// pura, válido para cualquier epoch no negativo.
static void epochToCivil(uint64_t epochSeconds, int &year, int &month, int &day,
                          int &hour, int &minute, int &second) {
  int64_t days = (int64_t)(epochSeconds / 86400ULL);
  int64_t rem  = (int64_t)(epochSeconds % 86400ULL);
  hour   = (int)(rem / 3600); rem %= 3600;
  minute = (int)(rem / 60);
  second = (int)(rem % 60);

  int64_t z   = days + 719468;
  int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  int64_t doe = z - era * 146097;                                    // [0, 146096]
  int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365; // [0, 399]
  int64_t y   = yoe + era * 400;
  int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);              // [0, 365]
  int64_t mp  = (5 * doy + 2) / 153;                                  // [0, 11]
  day   = (int)(doy - (153 * mp + 2) / 5 + 1);                        // [1, 31]
  month = (int)(mp + (mp < 10 ? 3 : -9));                             // [1, 12]
  year  = (int)(y + (month <= 2));
}

String formatTimestamp(uint64_t epochSeconds) {
  if (epochSeconds == 0) return "NA";
  int y, mo, d, h, mi, s;
  epochToCivil(epochSeconds, y, mo, d, h, mi, s);
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", y, mo, d, h, mi, s);
  return String(buf);
}

// Igual que formatTimestamp() pero en ISO 8601 (YYYY-MM-DDTHH:MM:SS, UTC),
// formato esperado por el backend (Postgres/Django) para el campo "timestamp".
String formatTimestampISO8601(uint64_t epochSeconds) {
  if (epochSeconds == 0) return "NA";
  int y, mo, d, h, mi, s;
  epochToCivil(epochSeconds, y, mo, d, h, mi, s);
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d", y, mo, d, h, mi, s);
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

  // ===== APAGAR MODEM (MOSFET canal P, HIGH=OFF) =====
  gpio_hold_dis((gpio_num_t)MODEM_PWR_PIN);
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, HIGH);
  gpio_hold_en((gpio_num_t)MODEM_PWR_PIN);

  // ===== RX2/TX2 a LOW (no flotando): evita retroalimentar el SIM800 por
  // los pines UART mientras está sin alimentación. Incondicional: en modo
  // WiFi el modem nunca se enciende, así que sin esto estos pines
  // quedarían en su estado flotante por defecto durante todo el sleep.
  modemLinesOff();

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
    doc["bat"]  = sensCache.bat;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Diagnóstico LTE bajo demanda (bloqueante, botón manual): solo tiene
  // sentido si connType=LTE, ya que enciende el modem SIM800 vía MOSFET.
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
        doc["msg"] = "El modem SIM800 no responde";
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

    bool netOk = modem.waitForNetwork(15000L);
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

  // Borra solo el archivo de datos (datalog.csv), sin tocar el resto de la
  // tarjeta. Deliberadamente NO reusa formatSD(): ese formateo baja al driver
  // ESP-IDF sdmmc de bajo nivel (borrado de sectores + f_mkfs) sobre el mismo
  // bus/pines que la librería Arduino SD usa para cada logToSD() — si algo
  // sale mal ahí, deja el subsistema SD en un estado del que initSD() ya no
  // se recupera, y entonces ni siquiera las lecturas normales vuelven a
  // grabarse. Este botón usa solo SD.remove() con la misma librería Arduino
  // que ya funciona para leer/escribir en cada ciclo, así que es mucho menos
  // riesgoso para el caso común de "quiero vaciar mis datos".
  server.on("/deletelog", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!sdReady) initSD();  // igual que logToSD(): reintentar un remontaje antes de rendirse

    StaticJsonDocument<256> doc;
    if (!sdReady) {
      doc["ok"] = false;
      doc["msg"] = "Tarjeta SD no disponible";
    } else if (!SD.exists(SD_LOG_PATH)) {
      doc["ok"] = true;
      doc["msg"] = "No había archivo de datos que borrar";
    } else {
      bool removed = SD.remove(SD_LOG_PATH);
      doc["ok"] = removed;
      doc["msg"] = removed ? "Archivo de datos borrado" : "No se pudo borrar el archivo (revisar tarjeta)";
    }
    doc["sdPresent"] = sdReady;
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
        // ds3231Ready refleja si el chip respondió en este ciclo (ver setting(),
        // que ahora lo inicializa al entrar en modo AP) — se lo devolvemos al
        // portal para que el botón de sincronización pueda distinguir "ancla
        // actualizada" de "DS3231 realmente reajustado".
        doc["ds3231"] = ds3231Ready;
        doc["msg"] = ds3231Ready ? "Hora y RTC DS3231 sincronizados" : "Hora sincronizada (DS3231 no detectado en el bus I2C)";
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

  // Montar SD (VSPI: MISO19/MOSI23/SCK18/CS5)
  initSD();

  // Cargar el ancla de hora real (ver /settime, setTimeBase())
  loadTimeBase();

  // Encender el riel de sensores (mismo MOSFET que alimenta el DS3231, GPIO2)
  // e iniciar I2C para que el RTC quede listo (ds3231Ready) ANTES de que el
  // portal cargue en el navegador/celular. Así el /settime automático que
  // dispara el JS al conectar (ver syncTime() en index_html) llega a
  // setTimeBase() con el DS3231 ya detectado y lo reajusta con la hora real
  // del PC/celular — sin esto, quedaba a merced de que updateSensCache() lo
  // inicializara más tarde en el primer loop(), lo cual es una carrera con
  // qué tan rápido el dispositivo se conecta y carga la página.
  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, LOW);  // MOSFET canal P: LOW = ON
  delay(100);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);
  initDS3231();

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
  int rawValue = analogRead(pinBattery);
  // Divisor: R_top=100k, R_bottom=100k → V_bat_teórico = V_adc × (100+100)/100
  float v_adc = rawValue * (3.3f / 4095.0f);

  // El ADC del ESP32 (atenuación 11dB) no es lineal en este rango: la
  // fórmula teórica (x2) subestima la tensión real de forma consistente.
  // Calibración de 2 puntos contra multímetro:
  //   3.70 V real → v_adc 1.67 V (teórico daba 3.34 V)
  //   4.20 V real → v_adc 1.93 V (teórico daba 3.86 V)
  const float BAT_CAL_SLOPE  = 1.9231f;
  const float BAT_CAL_OFFSET = 0.4885f;
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
    // período completo aquí, un GPIO39 ruidoso (sin pull-up, ver notas
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
  gpio_hold_dis((gpio_num_t)MODEM_PWR_PIN);
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, HIGH);
  gpio_hold_en((gpio_num_t)MODEM_PWR_PIN);

  // RX2/TX2 a LOW: evita retroalimentar el SIM800 por los pines UART
  // mientras está sin alimentación (ver modemLinesOff()).
  modemLinesOff();

  gpio_hold_dis((gpio_num_t)SENS_PWR_PIN);
  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, HIGH);
  gpio_hold_en((gpio_num_t)SENS_PWR_PIN);

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
    speed = 0;
  }
  return speed;
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

void modemPowerOn() {
  Serial.println("[MODEM] Encendiendo SIM800...");

  // Liberar el hold LOW de RX2/TX2 antes de que begin() los reconfigure
  // como pines UART, si no, el periférico no puede tomar control de ellos.
  gpio_hold_dis((gpio_num_t)PIN_TX2);
  gpio_hold_dis((gpio_num_t)PIN_RX2);

  gpio_hold_dis((gpio_num_t)MODEM_PWR_PIN);
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, LOW);  // MOSFET canal P: LOW = ON
  delay(3000);  // esperar arranque del SIM800

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
  modemPoweredOn = true;

  Serial.println("[MODEM] SIM800 encendido");
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
  digitalWrite(MODEM_PWR_PIN, HIGH);  // MOSFET canal P: HIGH = OFF
  gpio_hold_en((gpio_num_t)MODEM_PWR_PIN);

  modemPoweredOn = false;

  Serial.println("[MODEM] SIM800 apagado");
}

void modemRestart() {
  modemPowerOff();
  delay(1000);
  modemPowerOn();
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

  saveSetting(status ? "1" : "0",         "/status.txt");
  saveSetting(String((int)TIME_TO_SLEEP), "/timesleep.txt");
  saveSetting(A_TH ? "1" : "0",          "/A_TH.txt");
  saveSetting(A_WS ? "1" : "0",          "/A_WS.txt");
  saveSetting(A_PR ? "1" : "0",          "/A_PR.txt");
  saveSetting(dirSensorType,             "/dirSensor.txt");

  Serial.printf("[CONFIG] OK status=%d sleep=%llu seg (%llu min) TH=%d WS=%d PR=%d dirSensor=%s\n",
                status, TIME_TO_SLEEP, TIME_TO_SLEEP / 60, A_TH, A_WS, A_PR, dirSensorType.c_str());

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

  // Encender sensores: MOSFET canal P gate LOW = ON
  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, LOW);
  delay(2500);  // AM2301 necesita ≥2.5 s de arranque frío tras encender v_sensor

  // Iniciar I2C una sola vez — lo usan el MT6701/AS5600 (si A_WS) y siempre
  // el DS3231 (marca de tiempo), así que ya no se condiciona a A_WS.
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);
  initDS3231();

  // Coeficientes de calibración de este equipo en particular (ver getCalCoeffs()).
  CalCoeffs cal = getCalCoeffs();

  DynamicJsonDocument docOut(384);
  docOut["type"] = "weatherStationData";
  docOut["mac"] = macA();
  docOut["conn"] = connectionType;
  docOut["timestamp"] = formatTimestampISO8601(getEpochNow());

  // ===== TEMPERATURA Y HUMEDAD (DHT22) =====
  if (A_TH) {
    Serial.println("[SENSOR] Leyendo DHT22...");
    initDHT22();

    float temp, hum;
    bool ok;
    int retries = 0;

    do {
      readDHT22(temp, hum, ok);
      if (!ok) {
        retries++;
        delay(2000);  // el DHT22 necesita ~2 s entre lecturas
      }
    } while (!ok && retries < 5);

    if (ok) {
      float tempCal = cal.tempSlope * temp + cal.tempOffset;
      float humCal  = cal.humSlope * hum + cal.humOffset;
      docOut["T"] = String(tempCal, 2);
      docOut["H"] = String(humCal, 2);
      Serial.printf("[SENSOR] Temp: %.2f°C, Hum: %.2f%%\n", tempCal, humCal);
    } else {
      docOut["T"] = "error";
      docOut["H"] = "error";
      Serial.println("[SENSOR] Error leyendo DHT22");
    }
  }

  // ===== VELOCIDAD Y DIRECCIÓN DEL VIENTO =====
  if (A_WS) {
    Serial.println("[SENSOR] Leyendo viento...");

    initMT6701();
    float wv = WindVelocity();
    float wd = WindDirection();

    float wvCal = cal.windSlope * wv + cal.windOffset;

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

/* ============MODEM ========== ENVIO LTE (SIM800) ======================== */
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

  // Esperar red
  Serial.println("[LTE] Esperando red...");
  if (!modem.waitForNetwork(60000L)) {
    Serial.println("[ERROR] Sin cobertura de red");
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
  String dirSensor = readFile(SPIFFS, "/dirSensor.txt");

  A_TH = (ath == "1");
  A_WS = (aws == "1");
  A_PR = (apr == "1");
  dirSensorType = (dirSensor == "AS5600") ? "AS5600" : "MT6701";

  String apnCfg = readFile(SPIFFS, "/apn.txt");
  RUNapn = (apnCfg == "") ? "claro.pe" : apnCfg;

  Serial.println("---- VALORES CARGADOS ----");
  Serial.printf("A_TH = %s\n", A_TH ? "true" : "false");
  Serial.printf("A_WS = %s\n", A_WS ? "true" : "false");
  Serial.printf("A_PR = %s\n", A_PR ? "true" : "false");
  Serial.printf("dirSensorType = %s\n", dirSensorType.c_str());
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
  gpio_hold_dis((gpio_num_t)MODEM_PWR_PIN);
  gpio_hold_dis((gpio_num_t)SENS_PWR_PIN);
  gpio_hold_dis((gpio_num_t)PIN_TX2);
  gpio_hold_dis((gpio_num_t)PIN_RX2);

  // ===== ASEGURAR MODEM Y SENSORES APAGADOS =====
  // MOSFET canal P: HIGH = OFF. A diferencia del SIM7000 no se requiere
  // pulso de PWRKEY: toda la alimentación del módulo se corta con el MOSFET.
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, HIGH);

  pinMode(SENS_PWR_PIN, OUTPUT);
  digitalWrite(SENS_PWR_PIN, HIGH);

  // RX2/TX2 a LOW por defecto hasta que modemPowerOn() los reconfigure
  // como UART: evita retroalimentar el SIM800 (ver modemLinesOff()).
  pinMode(PIN_TX2, OUTPUT);
  pinMode(PIN_RX2, OUTPUT);
  digitalWrite(PIN_TX2, LOW);
  digitalWrite(PIN_RX2, LOW);

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

  // Leer configuración
  readConfig();

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
