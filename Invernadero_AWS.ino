/*
  INVERNADERO IBERO + AWS + RIEGO MANUAL
  XIAO ESP32S3 Sense + camara + BME688/BME680 + valvula de 12 V mediante MOSFET

  FUNCIONAMIENTO PRINCIPAL:
  - Se conecta a la red IBERO Primavera26.
  - Usa la IP fija 172.22.85.38, igual que el sistema original.
  - La interfaz muestra camara, datos ambientales, historial y control de la valvula.
  - Se registra una lectura local cada 20 segundos.
  - Se envian datos a AWS cada 30 segundos en una tarea independiente.
  - Si no logra conectarse a la IBERO, crea la red de respaldo INVERNADERO-RIEGO
    y la interfaz queda disponible en http://192.168.4.1, pero sin envio a AWS.

  CONEXIONES BME688:
  - VCC/VIN -> 3V3
  - GND     -> GND
  - SDA/SDI -> D4 (GPIO5)
  - SCL/SCK -> D5 (GPIO6)

  CONEXIONES DE CONTROL DE LA VALVULA:
  - Gate del MOSFET, por medio de resistencia de 220 ohms -> D3 (GPIO4)
  - Resistencia pull-down de 10 kohms entre Gate y GND
  - GND del XIAO unido al GND de la fuente de 12 V
  - Diodo flyback en paralelo con la valvula

  IMPORTANTE:
  - D4 NO se usa para la valvula porque ya es SDA del BME688.
  - La valvula utiliza su fuente independiente de 12 V.
  - En Arduino IDE activa PSRAM para la camara.
*/

// Evita el choque de nombres entre esp_camera y Adafruit Unified Sensor.
#define sensor_t camera_sensor_t
#include "esp_camera.h"
#undef sensor_t

#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_http_server.h"
#include <Wire.h>
#include <Adafruit_BME680.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// =====================================================
// CONFIGURACION GENERAL
// =====================================================

const char* DEVICE_ID = "invernadero_01";

// =====================================================
// AWS: misma configuracion del sistema original
// =====================================================
const char* AWS_DEVICE_ID = "huerto_01";
const char* DEVICE_SECRET = "da66cd4b5b22af5915ad6c039f9eaae6c0057ba67f12efc7";
const char* AWS_HUERTO_URL = "http://54.175.23.46/huerto";
const unsigned long AWS_PUSH_INTERVAL_MS = 30000;
const unsigned long AWS_HTTP_TIMEOUT_MS = 3000;

// =====================================================
// WIFI IBERO CON IP FIJA
// =====================================================
const char* WIFI_SSID = "Primavera26";
const char* WIFI_PASSWORD = "Ib3r02026pR1m";

IPAddress LOCAL_IP(172, 22, 85, 38);
IPAddress GATEWAY(172, 22, 87, 254);
IPAddress SUBNET(255, 255, 252, 0);
IPAddress PRIMARY_DNS(172, 22, 87, 254);
IPAddress SECONDARY_DNS(8, 8, 8, 8);

const unsigned long WIFI_CONNECT_TIMEOUT_MS = 45000;

// Red propia de respaldo si la IBERO no esta disponible.
const char* AP_SSID = "INVERNADERO-RIEGO";
const char* AP_PASSWORD = "Invernadero26";

// Lectura y registro local del sensor cada 20 segundos.
const unsigned long SENSOR_READ_INTERVAL_MS = 20000;

// Ultimas 30 lecturas: 30 x 20 s = 10 minutos de historial.
const size_t HISTORY_SIZE = 30;

// Camara: capturas rapidas para no dejar una conexion MJPEG infinita.
const framesize_t CAMERA_FRAME_SIZE = FRAMESIZE_VGA;
const int CAMERA_JPEG_QUALITY = 14;
const int VIEW_REFRESH_MS = 250;

// I2C externo del XIAO ESP32S3 Sense.
const int BME_SDA_PIN = 5;  // D4 = GPIO5
const int BME_SCL_PIN = 6;  // D5 = GPIO6

// D3 corresponde a GPIO4 y queda libre en este sistema.
const int PIN_VALVULA = D3;
const bool VALVE_ACTIVE_HIGH = true;

// Seguridad: la valvula nunca permanece abierta mas de 2 minutos.
const unsigned long VALVE_MAX_OPEN_MS = 120000;

// =====================================================
// PINES DE CAMARA XIAO ESP32S3 SENSE
// =====================================================

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10

#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15

#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// =====================================================
// DATOS GLOBALES
// =====================================================

httpd_handle_t web_httpd = NULL;  // Pagina y camara, puerto 80
httpd_handle_t api_httpd = NULL;  // Sensor, historial y valvula, puerto 81

Adafruit_BME680 bme;
bool usingAccessPoint = false;
bool bmeDetected = false;
uint8_t bmeAddress = 0x00;
unsigned long lastSensorReadMillis = 0;

struct SensorData {
  bool ok = false;
  float temperature = NAN;
  float humidity = NAN;
  float pressure = NAN;
  float gasResistance = NAN;
  unsigned long millisTime = 0;
  unsigned long sampleNumber = 0;
};

SensorData lastData;
SensorData historyData[HISTORY_SIZE];
size_t historyCount = 0;
size_t historyWriteIndex = 0;
unsigned long totalSamples = 0;

SemaphoreHandle_t cameraMutex = NULL;
SemaphoreHandle_t sensorDataMutex = NULL;
SemaphoreHandle_t valveMutex = NULL;
TaskHandle_t awsTaskHandle = NULL;

bool valveOpen = false;
unsigned long valveOpenedAt = 0;
unsigned long valveOpenDurationMs = 0;

// =====================================================
// FUNCIONES AUXILIARES
// =====================================================

String floatToString(float value, int decimals) {
  if (isnan(value)) return "null";
  return String(value, decimals);
}

String floatToText(float value, int decimals) {
  if (isnan(value)) return "--";
  return String(value, decimals);
}

String getCurrentIP() {
  if (usingAccessPoint) return WiFi.softAPIP().toString();
  return WiFi.localIP().toString();
}

String getCurrentSSID() {
  if (usingAccessPoint) return String(AP_SSID);
  if (WiFi.status() == WL_CONNECTED) return WiFi.SSID();
  return "SIN_RED";
}

String getCurrentBSSID() {
  if (usingAccessPoint) return WiFi.softAPmacAddress();
  if (WiFi.status() == WL_CONNECTED) return WiFi.BSSIDstr();
  return "SIN_BSSID";
}

String getConnectionMode() {
  if (usingAccessPoint) return "Red propia de respaldo";
  if (WiFi.status() == WL_CONNECTED) return "IBERO con IP fija";
  return "Sin conexion";
}

String wifiStatusToText(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "esperando";
    case WL_NO_SSID_AVAIL: return "SSID no encontrado";
    case WL_CONNECTED: return "conectado";
    case WL_CONNECT_FAILED: return "fallo de conexion/password";
    case WL_CONNECTION_LOST: return "conexion perdida";
    case WL_DISCONNECTED: return "desconectado";
    default: return "estado " + String((int)status);
  }
}

void addCorsHeaders(httpd_req_t* req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");
  httpd_resp_set_hdr(req, "Connection", "close");
}

// =====================================================
// WIFI IBERO CON IP FIJA + RED DE RESPALDO
// =====================================================

void startFallbackAccessPoint() {
  Serial.println("Creando red propia de respaldo...");

  WiFi.disconnect(true, true);
  delay(500);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  usingAccessPoint = true;

  IPAddress apIP(192, 168, 4, 1);
  IPAddress apGateway(192, 168, 4, 1);
  IPAddress apSubnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);

  bool apOk = WiFi.softAP(AP_SSID, AP_PASSWORD, 6, 0, 4);

  if (apOk) {
    Serial.println("Red propia creada correctamente.");
    Serial.print("Nombre: ");
    Serial.println(AP_SSID);
    Serial.print("Abre: http://");
    Serial.println(WiFi.softAPIP());
    Serial.println("En este modo la interfaz funciona, pero AWS no tiene internet.");
  } else {
    Serial.println("ERROR: no se pudo crear la red propia de respaldo.");
  }
}

void startWiFiFixed() {
  Serial.println();
  Serial.println("========== WIFI IBERO IP FIJA ==========");
  Serial.print("SSID: ");
  Serial.println(WIFI_SSID);
  Serial.print("IP solicitada: ");
  Serial.println(LOCAL_IP);

  usingAccessPoint = false;
  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("MAC del XIAO: ");
  Serial.println(WiFi.macAddress());

  bool configOk = WiFi.config(LOCAL_IP, GATEWAY, SUBNET, PRIMARY_DNS, SECONDARY_DNS);
  if (!configOk) {
    Serial.println("Advertencia: WiFi.config fallo; se intentara conectar de todos modos.");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - startAttemptTime < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    usingAccessPoint = false;
    Serial.println("WiFi IBERO conectado correctamente.");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("BSSID: ");
    Serial.println(WiFi.BSSIDstr());
    Serial.print("Interfaz: http://");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("Mascara: ");
    Serial.println(WiFi.subnetMask());
    Serial.print("RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

    if (WiFi.localIP() != LOCAL_IP) {
      Serial.println("AVISO: la IP final no coincide con la IP fija solicitada.");
    }
  } else {
    Serial.print("No se pudo conectar a la IBERO. Estado: ");
    Serial.println(wifiStatusToText(WiFi.status()));
    startFallbackAccessPoint();
  }

  Serial.println("========================================");
  Serial.println();
}

// =====================================================
// BME688 / BME680
// =====================================================

bool startBME688() {
  Serial.println("========== BME688 ==========");
  Serial.print("SDA: D4/GPIO");
  Serial.print(BME_SDA_PIN);
  Serial.print(" | SCL: D5/GPIO");
  Serial.println(BME_SCL_PIN);

  Wire.begin(BME_SDA_PIN, BME_SCL_PIN);
  Wire.setClock(100000);

  if (bme.begin(0x76, &Wire)) {
    bmeDetected = true;
    bmeAddress = 0x76;
  } else if (bme.begin(0x77, &Wire)) {
    bmeDetected = true;
    bmeAddress = 0x77;
  } else {
    bmeDetected = false;
    bmeAddress = 0x00;
  }

  if (!bmeDetected) {
    Serial.println("ERROR: BME688/BME680 no detectado en 0x76 ni 0x77.");
    Serial.println("Revisa 3V3, GND, D4/SDA y D5/SCL.");
    Serial.println("============================");
    return false;
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);

  Serial.print("Sensor detectado en 0x");
  Serial.println(bmeAddress, HEX);
  Serial.println("============================");
  return true;
}

SensorData readBME688() {
  SensorData data;
  data.millisTime = millis();
  data.sampleNumber = ++totalSamples;

  if (!bmeDetected) {
    data.ok = false;
    return data;
  }

  if (!bme.performReading()) {
    data.ok = false;
    return data;
  }

  data.ok = true;
  data.temperature = bme.temperature;
  data.humidity = bme.humidity;
  data.pressure = bme.pressure / 100.0;
  data.gasResistance = bme.gas_resistance;
  return data;
}

void printSensorData(const SensorData& data) {
  Serial.println("========== LECTURA BME688 ==========");
  Serial.print("Muestra: ");
  Serial.println(data.sampleNumber);
  Serial.print("Estado: ");
  Serial.println(data.ok ? "OK" : "ERROR");

  if (data.ok) {
    Serial.print("Temperatura: ");
    Serial.print(data.temperature, 2);
    Serial.println(" C");
    Serial.print("Humedad: ");
    Serial.print(data.humidity, 2);
    Serial.println(" %");
    Serial.print("Presion: ");
    Serial.print(data.pressure, 2);
    Serial.println(" hPa");
    Serial.print("Gas: ");
    Serial.print(data.gasResistance, 0);
    Serial.println(" ohms");
  }

  Serial.println("====================================");
}

void saveSensorData(const SensorData& data) {
  if (sensorDataMutex != NULL &&
      xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
    lastData = data;
    historyData[historyWriteIndex] = data;
    historyWriteIndex = (historyWriteIndex + 1) % HISTORY_SIZE;
    if (historyCount < HISTORY_SIZE) historyCount++;
    xSemaphoreGive(sensorDataMutex);
    return;
  }

  // Respaldo si el mutex no estuviera disponible.
  lastData = data;
}

SensorData getLastSensorDataCopy() {
  SensorData copy;

  if (sensorDataMutex != NULL &&
      xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    copy = lastData;
    xSemaphoreGive(sensorDataMutex);
    return copy;
  }

  return lastData;
}

void updateSensorIfNeeded() {
  unsigned long now = millis();

  if ((unsigned long)(now - lastSensorReadMillis) >= SENSOR_READ_INTERVAL_MS) {
    SensorData data = readBME688();
    saveSensorData(data);
    printSensorData(data);
    lastSensorReadMillis = now;
  }
}

// =====================================================
// ENVIO A AWS EN SEGUNDO PLANO
// =====================================================

bool enviarDatosAWS(const SensorData& data, const char* origen) {
  Serial.println("========== AWS HUERTO ==========");
  Serial.printf("[AWS] Origen: %s\n", origen);

  if (usingAccessPoint || WiFi.status() != WL_CONNECTED || !data.ok) {
    Serial.println("[AWS] NO ENVIADO: sin internet o lectura no valida.");
    Serial.println("===============================\n");
    return false;
  }

  String jsonBody = "{";
  jsonBody += "\"device_id\":\"" + String(AWS_DEVICE_ID) + "\",";
  jsonBody += "\"temperature\":" + floatToString(data.temperature, 2) + ",";
  jsonBody += "\"humidity\":" + floatToString(data.humidity, 2) + ",";
  jsonBody += "\"pressure\":" + floatToString(data.pressure, 2) + ",";
  jsonBody += "\"gas\":" + floatToString(data.gasResistance, 0);
  jsonBody += "}";

  WiFiClient client;
  HTTPClient http;

  if (!http.begin(client, AWS_HUERTO_URL)) {
    Serial.println("[AWS] ERROR: no se pudo iniciar HTTPClient.");
    Serial.println("===============================\n");
    return false;
  }

  http.setTimeout(AWS_HTTP_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-device-secret", DEVICE_SECRET);

  int httpCode = http.POST(jsonBody);
  bool success = false;

  if (httpCode > 0) {
    String response = http.getString();
    success = httpCode >= 200 && httpCode < 300;

    if (success) {
      Serial.printf("[AWS] ENVIADO CORRECTAMENTE. HTTP %d\n", httpCode);
    } else {
      Serial.printf("[AWS] RECHAZADO. HTTP %d\n", httpCode);
      if (response.length() > 0) {
        Serial.print("[AWS] Respuesta: ");
        Serial.println(response);
      }
    }
  } else {
    Serial.printf("[AWS] ERROR DE RED: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
  Serial.println("===============================\n");
  return success;
}

void awsPushTask(void* parameter) {
  Serial.println("[AWS] Tarea de envio iniciada.");

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(AWS_PUSH_INTERVAL_MS));
    SensorData data = getLastSensorDataCopy();
    enviarDatosAWS(data, "ENVIO_PERIODICO_30S");
  }
}

void startAWSPushTask() {
  if (awsTaskHandle != NULL) return;

  BaseType_t result = xTaskCreate(
    awsPushTask,
    "AWS Push",
    8192,
    NULL,
    1,
    &awsTaskHandle
  );

  if (result == pdPASS) {
    Serial.println("[AWS] Tarea creada correctamente.");
  } else {
    Serial.println("[AWS] ERROR: no se pudo crear la tarea.");
  }
}

void probarConexionAWSInicial() {
  if (usingAccessPoint || WiFi.status() != WL_CONNECTED) {
    Serial.println("[AWS] Prueba inicial omitida: no hay internet.");
    return;
  }

  SensorData data = getLastSensorDataCopy();
  if (!data.ok) {
    Serial.println("[AWS] Prueba inicial omitida: lectura BME688 no valida.");
    return;
  }

  enviarDatosAWS(data, "PRUEBA_INICIAL_SETUP");
}

// =====================================================
// VALVULA MEDIANTE MOSFET
// =====================================================

void writeValvePin(bool openState) {
  bool pinHigh = VALVE_ACTIVE_HIGH ? openState : !openState;
  digitalWrite(PIN_VALVULA, pinHigh ? HIGH : LOW);
}

void closeValve(const char* reason) {
  if (valveMutex != NULL &&
      xSemaphoreTake(valveMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    writeValvePin(false);
    valveOpen = false;
    valveOpenedAt = 0;
    valveOpenDurationMs = 0;
    xSemaphoreGive(valveMutex);
  } else {
    writeValvePin(false);
    valveOpen = false;
    valveOpenedAt = 0;
    valveOpenDurationMs = 0;
  }

  Serial.print("[VALVULA] CERRADA. Motivo: ");
  Serial.println(reason);
}

void openValve(unsigned long requestedDurationMs) {
  // 0 significa apertura manual, pero igualmente se aplica el limite de seguridad.
  unsigned long duration = requestedDurationMs;
  if (duration == 0 || duration > VALVE_MAX_OPEN_MS) {
    duration = VALVE_MAX_OPEN_MS;
  }

  if (valveMutex != NULL &&
      xSemaphoreTake(valveMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    writeValvePin(true);
    valveOpen = true;
    valveOpenedAt = millis();
    valveOpenDurationMs = duration;
    xSemaphoreGive(valveMutex);
  } else {
    writeValvePin(true);
    valveOpen = true;
    valveOpenedAt = millis();
    valveOpenDurationMs = duration;
  }

  Serial.print("[VALVULA] ABIERTA durante maximo ");
  Serial.print(duration / 1000);
  Serial.println(" segundos.");
}

void updateValveSafety() {
  bool shouldClose = false;

  if (valveMutex != NULL &&
      xSemaphoreTake(valveMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (valveOpen && valveOpenDurationMs > 0 &&
        (unsigned long)(millis() - valveOpenedAt) >= valveOpenDurationMs) {
      shouldClose = true;
    }
    xSemaphoreGive(valveMutex);
  } else if (valveOpen && valveOpenDurationMs > 0 &&
             (unsigned long)(millis() - valveOpenedAt) >= valveOpenDurationMs) {
    shouldClose = true;
  }

  if (shouldClose) closeValve("tiempo terminado / seguridad");
}

void getValveStateCopy(bool& isOpen, unsigned long& remainingMs) {
  isOpen = valveOpen;
  remainingMs = 0;

  if (valveMutex != NULL &&
      xSemaphoreTake(valveMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    isOpen = valveOpen;
    if (valveOpen && valveOpenDurationMs > 0) {
      unsigned long elapsed = millis() - valveOpenedAt;
      remainingMs = elapsed >= valveOpenDurationMs ? 0 : valveOpenDurationMs - elapsed;
    }
    xSemaphoreGive(valveMutex);
    return;
  }

  if (valveOpen && valveOpenDurationMs > 0) {
    unsigned long elapsed = millis() - valveOpenedAt;
    remainingMs = elapsed >= valveOpenDurationMs ? 0 : valveOpenDurationMs - elapsed;
  }
}

// =====================================================
// CAMARA
// =====================================================

bool startCamera() {
  Serial.println();
  Serial.println("========== CAMARA ==========");

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = CAMERA_FRAME_SIZE;
    config.jpeg_quality = CAMERA_JPEG_QUALITY;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    Serial.println("PSRAM detectada.");
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 16;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    Serial.println("PSRAM no detectada; se usara QVGA.");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("ERROR CAMARA: 0x%x\n", err);
    Serial.println("Revisa el flex y la opcion PSRAM del Arduino IDE.");
    return false;
  }

  camera_sensor_t* cameraSensor = esp_camera_sensor_get();
  if (cameraSensor != NULL) {
    cameraSensor->set_brightness(cameraSensor, 1);
    cameraSensor->set_contrast(cameraSensor, 1);
    cameraSensor->set_saturation(cameraSensor, 0);
  }

  Serial.println("Camara iniciada correctamente.");
  Serial.println("============================");
  return true;
}

// =====================================================
// SERVIDOR WEB
// =====================================================

static esp_err_t indexHandler(httpd_req_t* req) {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Invernadero - Riego IBERO</title>
  <style>
    * { box-sizing: border-box; }
    body {
      font-family: Arial, sans-serif;
      background: #07140d;
      color: #edf7f0;
      margin: 0;
      padding: 10px;
      text-align: center;
    }
    .wrap { max-width: 980px; margin: 0 auto; }
    h1 { font-size: 22px; margin: 8px 0 3px; }
    .subtitle { color: #a5b9aa; font-size: 13px; margin-bottom: 10px; }
    .box {
      background: #102219;
      border: 1px solid #2c4936;
      border-radius: 14px;
      padding: 11px;
      margin-bottom: 10px;
    }
    .camBox {
      min-height: 210px;
      display: flex;
      align-items: center;
      justify-content: center;
      background: #020704;
      border-radius: 11px;
      overflow: hidden;
    }
    #cam { width: 100%; max-width: 850px; display: block; }
    .grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 8px;
    }
    .card {
      background: #0b1911;
      border: 1px solid #294333;
      border-radius: 11px;
      padding: 10px 5px;
    }
    .label { color: #a5b9aa; font-size: 12px; }
    .value { font-size: 22px; font-weight: bold; margin-top: 5px; }
    .unit { color: #a5b9aa; font-size: 11px; }
    .valveState {
      font-size: 28px;
      font-weight: bold;
      margin: 5px 0 2px;
    }
    .open { color: #56e88b; }
    .closed { color: #ff7777; }
    .buttons {
      display: grid;
      grid-template-columns: repeat(5, 1fr);
      gap: 8px;
      margin-top: 10px;
    }
    button {
      border: 0;
      border-radius: 10px;
      padding: 12px 6px;
      font-weight: bold;
      font-size: 14px;
      cursor: pointer;
    }
    .openButton { background: #2fc56a; color: #041109; }
    .closeButton { background: #e85858; color: white; }
    .timeButton { background: #d9a938; color: #171005; }
    button:disabled { opacity: 0.5; cursor: wait; }
    .info { color: #b7c8bb; font-size: 12px; line-height: 1.55; margin-top: 8px; }
    .ok { color: #56e88b; font-weight: bold; }
    .bad { color: #ff7777; font-weight: bold; }
    .historyWrap { overflow-x: auto; }
    table { width: 100%; border-collapse: collapse; font-size: 12px; }
    th, td { border-bottom: 1px solid #294333; padding: 7px 5px; white-space: nowrap; }
    th { color: #b9ccbe; }
    .notice {
      background: #2b2512;
      border: 1px solid #6d5a22;
      color: #f3dda2;
      border-radius: 10px;
      padding: 8px;
      font-size: 12px;
      margin-bottom: 10px;
    }
    @media (max-width: 720px) {
      .grid { grid-template-columns: repeat(2, 1fr); }
      .buttons { grid-template-columns: repeat(2, 1fr); }
      .buttons button:last-child { grid-column: span 2; }
      .value { font-size: 20px; }
      .camBox { min-height: 180px; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <h1>Invernadero - control de riego</h1>
    <div class="subtitle">IBERO con IP fija · AWS cada 30 s · lectura local cada 20 s</div>

    <div class="notice">
      Acceso local: <b>http://%IP%</b> · Modo actual: <b>%MODE%</b>.
      Si arranca la red de respaldo, conéctate a <b>INVERNADERO-RIEGO</b>.
    </div>

    <div class="box">
      <div class="camBox"><img id="cam" alt="Cámara del invernadero"></div>
    </div>

    <div class="box">
      <div class="grid">
        <div class="card">
          <div class="label">Temperatura</div>
          <div class="value"><span id="temp">%TEMP%</span></div>
          <div class="unit">°C</div>
        </div>
        <div class="card">
          <div class="label">Humedad ambiental</div>
          <div class="value"><span id="hum">%HUM%</span></div>
          <div class="unit">%</div>
        </div>
        <div class="card">
          <div class="label">Presión</div>
          <div class="value"><span id="press">%PRESS%</span></div>
          <div class="unit">hPa</div>
        </div>
        <div class="card">
          <div class="label">Resistencia de gas</div>
          <div class="value"><span id="gas">%GAS%</span></div>
          <div class="unit">Ω</div>
        </div>
      </div>
      <div class="info">
        BME688: <span id="bmeStatus">%BME_STATUS%</span> ·
        Muestra: <span id="sample">%SAMPLE%</span> ·
        Actualización: 20 s
      </div>
    </div>

    <div class="box">
      <div class="label">Estado de la válvula</div>
      <div id="valveState" class="valveState closed">CERRADA</div>
      <div class="info">Tiempo restante: <span id="remaining">0</span> s · Seguridad máxima: 120 s</div>
      <div class="buttons">
        <button class="openButton" onclick="controlValve('open', 0)">ABRIR</button>
        <button class="closeButton" onclick="controlValve('close', 0)">CERRAR</button>
        <button class="timeButton" onclick="controlValve('open', 5000)">REGAR 5 s</button>
        <button class="timeButton" onclick="controlValve('open', 10000)">REGAR 10 s</button>
        <button class="timeButton" onclick="controlValve('open', 30000)">REGAR 30 s</button>
      </div>
      <div id="commandMessage" class="info"></div>
    </div>

    <div class="box">
      <div class="label" style="margin-bottom:8px;">Historial guardado en el ESP32</div>
      <div class="historyWrap">
        <table>
          <thead>
            <tr>
              <th>Muestra</th><th>Tiempo</th><th>Temp.</th><th>Humedad</th><th>Presión</th><th>Gas</th>
            </tr>
          </thead>
          <tbody id="historyBody"></tbody>
        </table>
      </div>
      <div class="info">Se conservan las últimas 30 lecturas en RAM. Se borran al reiniciar el XIAO.</div>
    </div>

    <div class="box info">
      Red: <b>%SSID%</b> · IP: <b>%IP%</b><br>
      BSSID: <b>%BSSID%</b> · AWS: <b>cada 30 s</b><br>
      Señal MOSFET: <b>D3 / GPIO4</b> · BME688: <b>D4 y D5</b>
    </div>
  </div>

  <script>
    const apiBase = 'http://%IP%:81';
    const refreshMs = %REFRESH_MS%;
    const cam = document.getElementById('cam');

    function fmt(value, decimals) {
      if (value === null || value === undefined || isNaN(value)) return '--';
      return Number(value).toFixed(decimals);
    }

    function setText(id, value) {
      const el = document.getElementById(id);
      if (el) el.textContent = value;
    }

    function cameraLoop() {
      const frame = new Image();
      frame.onload = () => {
        cam.src = frame.src;
        setTimeout(cameraLoop, refreshMs);
      };
      frame.onerror = () => setTimeout(cameraLoop, 1000);
      frame.src = '/capture?t=' + Date.now();
    }

    async function updateSensor() {
      try {
        const response = await fetch(apiBase + '/sensor?t=' + Date.now(), { cache: 'no-store' });
        const data = await response.json();
        setText('temp', fmt(data.temperature_c, 2));
        setText('hum', fmt(data.humidity_percent, 2));
        setText('press', fmt(data.pressure_hpa, 2));
        setText('gas', fmt(data.gas_resistance_ohms, 0));
        setText('sample', data.sample_number);

        const status = document.getElementById('bmeStatus');
        status.textContent = data.bme_ok ? 'OK' : 'ERROR';
        status.className = data.bme_ok ? 'ok' : 'bad';
      } catch (error) {
        const status = document.getElementById('bmeStatus');
        status.textContent = 'SIN RESPUESTA';
        status.className = 'bad';
      }
    }

    function formatUptime(ms) {
      const totalSeconds = Math.floor(ms / 1000);
      const minutes = Math.floor(totalSeconds / 60);
      const seconds = totalSeconds % 60;
      return minutes + ':' + String(seconds).padStart(2, '0');
    }

    async function updateHistory() {
      try {
        const response = await fetch(apiBase + '/history?t=' + Date.now(), { cache: 'no-store' });
        const payload = await response.json();
        const rows = payload.readings.slice().reverse().map(item => `
          <tr>
            <td>${item.sample}</td>
            <td>${formatUptime(item.millis)}</td>
            <td>${fmt(item.temperature_c, 2)} °C</td>
            <td>${fmt(item.humidity_percent, 2)} %</td>
            <td>${fmt(item.pressure_hpa, 2)}</td>
            <td>${fmt(item.gas_resistance_ohms, 0)}</td>
          </tr>`).join('');
        document.getElementById('historyBody').innerHTML = rows;
      } catch (error) {
        document.getElementById('historyBody').innerHTML = '<tr><td colspan="6">No se pudo cargar el historial.</td></tr>';
      }
    }

    function applyValveState(data) {
      const state = document.getElementById('valveState');
      state.textContent = data.valve_open ? 'ABIERTA' : 'CERRADA';
      state.className = 'valveState ' + (data.valve_open ? 'open' : 'closed');
      setText('remaining', Math.ceil(data.remaining_ms / 1000));
    }

    async function updateValveStatus() {
      try {
        const response = await fetch(apiBase + '/valve?t=' + Date.now(), { cache: 'no-store' });
        const data = await response.json();
        applyValveState(data);
      } catch (error) {
        setText('commandMessage', 'Sin respuesta del control de válvula.');
      }
    }

    async function controlValve(action, durationMs) {
      const buttons = document.querySelectorAll('button');
      buttons.forEach(button => button.disabled = true);
      setText('commandMessage', 'Enviando orden...');

      try {
        const url = apiBase + '/valve?state=' + action + '&duration_ms=' + durationMs + '&t=' + Date.now();
        const response = await fetch(url, { cache: 'no-store' });
        const data = await response.json();
        applyValveState(data);
        setText('commandMessage', data.message || 'Orden ejecutada.');
      } catch (error) {
        setText('commandMessage', 'No se pudo enviar la orden.');
      } finally {
        buttons.forEach(button => button.disabled = false);
      }
    }

    cameraLoop();
    updateSensor();
    updateHistory();
    updateValveStatus();

    setInterval(updateSensor, 20000);
    setInterval(updateHistory, 20000);
    setInterval(updateValveStatus, 1000);
  </script>
</body>
</html>
)rawliteral";

  SensorData data = getLastSensorDataCopy();

  html.replace("%IP%", getCurrentIP());
  html.replace("%SSID%", getCurrentSSID());
  html.replace("%BSSID%", getCurrentBSSID());
  html.replace("%MODE%", getConnectionMode());
  html.replace("%TEMP%", floatToText(data.temperature, 2));
  html.replace("%HUM%", floatToText(data.humidity, 2));
  html.replace("%PRESS%", floatToText(data.pressure, 2));
  html.replace("%GAS%", floatToText(data.gasResistance, 0));
  html.replace("%BME_STATUS%", data.ok ? "OK" : "ERROR");
  html.replace("%SAMPLE%", String(data.sampleNumber));
  html.replace("%REFRESH_MS%", String(VIEW_REFRESH_MS));

  httpd_resp_set_type(req, "text/html; charset=utf-8");
  addCorsHeaders(req);
  return httpd_resp_send(req, html.c_str(), html.length());
}

static esp_err_t captureHandler(httpd_req_t* req) {
  if (cameraMutex != NULL &&
      xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_sendstr(req, "Camara ocupada");
  }

  camera_fb_t* frame = esp_camera_fb_get();
  if (frame == NULL) {
    if (cameraMutex != NULL) xSemaphoreGive(cameraMutex);
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  addCorsHeaders(req);

  esp_err_t result = httpd_resp_send(req, (const char*)frame->buf, frame->len);
  esp_camera_fb_return(frame);
  if (cameraMutex != NULL) xSemaphoreGive(cameraMutex);
  return result;
}

static esp_err_t sensorHandler(httpd_req_t* req) {
  SensorData data = getLastSensorDataCopy();

  String json = "{";
  json += "\"device_id\":\"" + String(DEVICE_ID) + "\",";
  json += "\"mode\":\"" + getConnectionMode() + "\",";
  json += "\"wifi_ssid\":\"" + getCurrentSSID() + "\",";
  json += "\"wifi_bssid\":\"" + getCurrentBSSID() + "\",";
  json += "\"ip_address\":\"" + getCurrentIP() + "\",";
  json += "\"wifi_connected\":" + String((!usingAccessPoint && WiFi.status() == WL_CONNECTED) ? "true" : "false") + ",";
  json += "\"using_fallback_ap\":" + String(usingAccessPoint ? "true" : "false") + ",";
  json += "\"wifi_rssi_dbm\":" + String((!usingAccessPoint && WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0) + ",";
  json += "\"aws_push_interval_s\":" + String(AWS_PUSH_INTERVAL_MS / 1000) + ",";
  json += "\"sensor_interval_s\":" + String(SENSOR_READ_INTERVAL_MS / 1000) + ",";
  json += "\"bme_ok\":" + String(data.ok ? "true" : "false") + ",";
  json += "\"bme_detected\":" + String(bmeDetected ? "true" : "false") + ",";
  json += "\"bme_i2c_address\":\"";
  if (bmeDetected) {
    json += "0x";
    json += String(bmeAddress, HEX);
  } else {
    json += "NO_DETECTADO";
  }
  json += "\",";
  json += "\"sample_number\":" + String(data.sampleNumber) + ",";
  json += "\"millis\":" + String(data.millisTime) + ",";
  json += "\"temperature_c\":" + floatToString(data.temperature, 2) + ",";
  json += "\"humidity_percent\":" + floatToString(data.humidity, 2) + ",";
  json += "\"pressure_hpa\":" + floatToString(data.pressure, 2) + ",";
  json += "\"gas_resistance_ohms\":" + floatToString(data.gasResistance, 0);
  json += "}";

  httpd_resp_set_type(req, "application/json");
  addCorsHeaders(req);
  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t historyHandler(httpd_req_t* req) {
  SensorData historyCopy[HISTORY_SIZE];
  size_t copyCount = 0;

  if (sensorDataMutex != NULL &&
      xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
    copyCount = historyCount;
    size_t oldestIndex = (historyCount < HISTORY_SIZE) ? 0 : historyWriteIndex;

    for (size_t i = 0; i < copyCount; i++) {
      size_t sourceIndex = (oldestIndex + i) % HISTORY_SIZE;
      historyCopy[i] = historyData[sourceIndex];
    }

    xSemaphoreGive(sensorDataMutex);
  }

  String json;
  json.reserve(7000);
  json = "{\"count\":" + String(copyCount) + ",\"readings\":[";

  for (size_t i = 0; i < copyCount; i++) {
    const SensorData& data = historyCopy[i];
    if (i > 0) json += ",";

    json += "{";
    json += "\"sample\":" + String(data.sampleNumber) + ",";
    json += "\"ok\":" + String(data.ok ? "true" : "false") + ",";
    json += "\"millis\":" + String(data.millisTime) + ",";
    json += "\"temperature_c\":" + floatToString(data.temperature, 2) + ",";
    json += "\"humidity_percent\":" + floatToString(data.humidity, 2) + ",";
    json += "\"pressure_hpa\":" + floatToString(data.pressure, 2) + ",";
    json += "\"gas_resistance_ohms\":" + floatToString(data.gasResistance, 0);
    json += "}";
  }

  json += "]}";

  httpd_resp_set_type(req, "application/json");
  addCorsHeaders(req);
  return httpd_resp_send(req, json.c_str(), json.length());
}

static esp_err_t valveHandler(httpd_req_t* req) {
  String requestedState = "status";
  unsigned long requestedDurationMs = 0;
  bool commandExecuted = false;
  String message = "Estado actualizado.";

  size_t queryLength = httpd_req_get_url_query_len(req);
  if (queryLength > 0) {
    char* query = (char*)malloc(queryLength + 1);
    if (query != NULL) {
      if (httpd_req_get_url_query_str(req, query, queryLength + 1) == ESP_OK) {
        char value[24];

        if (httpd_query_key_value(query, "state", value, sizeof(value)) == ESP_OK) {
          requestedState = String(value);
          requestedState.toLowerCase();
        }

        if (httpd_query_key_value(query, "duration_ms", value, sizeof(value)) == ESP_OK) {
          requestedDurationMs = strtoul(value, NULL, 10);
        }
      }
      free(query);
    }
  }

  if (requestedState == "open" || requestedState == "on") {
    openValve(requestedDurationMs);
    commandExecuted = true;
    message = "Valvula abierta.";
  } else if (requestedState == "close" || requestedState == "off") {
    closeValve("orden desde la interfaz");
    commandExecuted = true;
    message = "Valvula cerrada.";
  } else if (requestedState != "status") {
    message = "Orden no reconocida. Usa open, close o status.";
  }

  bool isOpen = false;
  unsigned long remainingMs = 0;
  getValveStateCopy(isOpen, remainingMs);

  String json = "{";
  json += "\"ok\":true,";
  json += "\"command_executed\":" + String(commandExecuted ? "true" : "false") + ",";
  json += "\"message\":\"" + message + "\",";
  json += "\"valve_open\":" + String(isOpen ? "true" : "false") + ",";
  json += "\"pin\":\"D3/GPIO4\",";
  json += "\"remaining_ms\":" + String(remainingMs) + ",";
  json += "\"max_open_ms\":" + String(VALVE_MAX_OPEN_MS);
  json += "}";

  httpd_resp_set_type(req, "application/json");
  addCorsHeaders(req);
  return httpd_resp_send(req, json.c_str(), json.length());
}

void startWebServers() {
  // Puerto 80: pagina y capturas de camara.
  httpd_config_t webConfig = HTTPD_DEFAULT_CONFIG();
  webConfig.server_port = 80;
  webConfig.ctrl_port = 32768;
  webConfig.max_uri_handlers = 6;
  webConfig.stack_size = 10240;
  webConfig.max_open_sockets = 6;
  webConfig.lru_purge_enable = true;

  httpd_uri_t indexUri = {};
  indexUri.uri = "/";
  indexUri.method = HTTP_GET;
  indexUri.handler = indexHandler;

  httpd_uri_t captureUri = {};
  captureUri.uri = "/capture";
  captureUri.method = HTTP_GET;
  captureUri.handler = captureHandler;

  if (httpd_start(&web_httpd, &webConfig) == ESP_OK) {
    httpd_register_uri_handler(web_httpd, &indexUri);
    httpd_register_uri_handler(web_httpd, &captureUri);
    Serial.println("Servidor web/camara iniciado en puerto 80.");
  } else {
    Serial.println("ERROR al iniciar servidor web/camara.");
  }

  // Puerto 81: API para evitar que las capturas bloqueen sensor o valvula.
  httpd_config_t apiConfig = HTTPD_DEFAULT_CONFIG();
  apiConfig.server_port = 81;
  apiConfig.ctrl_port = 32769;
  apiConfig.max_uri_handlers = 8;
  apiConfig.stack_size = 8192;
  apiConfig.max_open_sockets = 5;
  apiConfig.lru_purge_enable = true;

  httpd_uri_t sensorUri = {};
  sensorUri.uri = "/sensor";
  sensorUri.method = HTTP_GET;
  sensorUri.handler = sensorHandler;

  httpd_uri_t historyUri = {};
  historyUri.uri = "/history";
  historyUri.method = HTTP_GET;
  historyUri.handler = historyHandler;

  httpd_uri_t valveUri = {};
  valveUri.uri = "/valve";
  valveUri.method = HTTP_GET;
  valveUri.handler = valveHandler;

  if (httpd_start(&api_httpd, &apiConfig) == ESP_OK) {
    httpd_register_uri_handler(api_httpd, &sensorUri);
    httpd_register_uri_handler(api_httpd, &historyUri);
    httpd_register_uri_handler(api_httpd, &valveUri);
    Serial.println("API de sensor/valvula iniciada en puerto 81.");
  } else {
    Serial.println("ERROR al iniciar API de sensor/valvula.");
  }
}

// =====================================================
// SETUP Y LOOP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("INICIANDO INVERNADERO IBERO + AWS + RIEGO");

  // La valvula se inicializa primero y siempre cerrada.
  pinMode(PIN_VALVULA, OUTPUT);
  writeValvePin(false);
  valveOpen = false;
  Serial.println("Valvula inicializada CERRADA en D3/GPIO4.");

  cameraMutex = xSemaphoreCreateMutex();
  sensorDataMutex = xSemaphoreCreateMutex();
  valveMutex = xSemaphoreCreateMutex();

  startWiFiFixed();
  startBME688();

  // Primera lectura inmediata para que la pagina y AWS tengan datos validos.
  SensorData firstData = readBME688();
  saveSensorData(firstData);
  printSensorData(firstData);
  lastSensorReadMillis = millis();

  bool cameraOk = startCamera();
  startWebServers();

  // Prueba inmediata y despues envios periodicos sin bloquear la camara.
  probarConexionAWSInicial();
  startAWSPushTask();

  Serial.println();
  Serial.println("========== SISTEMA LISTO ==========");
  Serial.print("Camara: ");
  Serial.println(cameraOk ? "OK" : "ERROR");
  Serial.print("BME688: ");
  Serial.println(bmeDetected ? "OK" : "ERROR");
  Serial.print("Modo de red: ");
  Serial.println(getConnectionMode());
  Serial.print("SSID: ");
  Serial.println(getCurrentSSID());
  Serial.print("BSSID: ");
  Serial.println(getCurrentBSSID());
  Serial.print("Interfaz: http://");
  Serial.println(getCurrentIP());
  Serial.print("Sensor JSON: http://");
  Serial.print(getCurrentIP());
  Serial.println(":81/sensor");
  Serial.print("Historial JSON: http://");
  Serial.print(getCurrentIP());
  Serial.println(":81/history");
  Serial.println("Valvula: D3/GPIO4");
  Serial.print("AWS: ");
  Serial.println((!usingAccessPoint && WiFi.status() == WL_CONNECTED) ? "ACTIVO" : "SIN INTERNET");
  Serial.println("===================================");
}

void loop() {
  updateSensorIfNeeded();
  updateValveSafety();

  // Los servidores HTTP y AWS trabajan en tareas propias.
  delay(20);
}
