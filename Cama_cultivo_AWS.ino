/*
  ESP32 + Sensor de Humedad FC-28 + BME688
  Proyecto: Camas de cultivo / Huerto IoT

  Version optimizada basada en la arquitectura rapida del proyecto de camara:
  - IP fija en Primavera26.
  - Servidor HTTP rapido en puerto 80.
  - Pagina estatica ligera: no se construye HTML gigante con String.replace().
  - Endpoint JSON principal: http://IP/sensor
  - Endpoint JSON extra de respaldo: http://IP:81/sensor
  - Lectura de sensores cacheada cada 30 segundos.
  - Soporte multiusuario con sockets LRU.
  - Red propia de respaldo si no conecta a Primavera26.
  - Reconexión WiFi si se cae la red.
  - Sin cambiar el funcionamiento de sensores ni rutas principales.

  Librerias necesarias en Arduino IDE:
  - Adafruit BME680 Library
  - Adafruit Unified Sensor

  Conexiones recomendadas:

  FC-28 / Higrometro analogico:
  - VCC -> 3V3
  - GND -> GND
  - AO  -> GPIO34

  BME688:
  - VCC/VIN -> 3V3
  - GND     -> GND
  - SDA/SDI -> GPIO21
  - SCL/SCK -> GPIO22

  Nota importante de red IBERO:
  Para la IP fija 172.22.32.115 se usa mascara /22.
  Rango aproximado: 172.22.32.1 - 172.22.35.254
  Gateway esperado: 172.22.35.254
*/

#include <WiFi.h>
#include "esp_http_server.h"
#include <Wire.h>
#include <Adafruit_BME680.h>
#include <HTTPClient.h> // Librería de peticiones de red salientes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Configuración de la firma segura de AWS
const char* AWS_HUERTO_URL = "http://54.175.23.46/huerto";
const char* DEVICE_SECRET = "da66cd4b5b22af5915ad6c039f9eaae6c0057ba67f12efc7";
TaskHandle_t awsTaskHandle = NULL;

// =====================================================
// CONFIGURACION DEL USUARIO
// =====================================================

const char* DEVICE_ID = "cama_cultivo_01";

// Red IBERO
const char* WIFI_SSID = "Primavera26";
const char* WIFI_PASSWORD = "Ib3r02026pR1m";

// IP fija asignada para cama de cultivo.
// IMPORTANTE: basada en el codigo rapido del otro proyecto, usamos /22 y gateway del bloque.
IPAddress LOCAL_IP(172, 22, 32, 115);
IPAddress GATEWAY(172, 22, 35, 254);
IPAddress SUBNET(255, 255, 252, 0);
IPAddress PRIMARY_DNS(172, 22, 35, 254);
IPAddress SECONDARY_DNS(8, 8, 8, 8);

// Red propia de respaldo si no conecta a la IBERO
const char* AP_SSID = "CAMA-CULTIVO-01";
const char* AP_PASSWORD = "12345678";

const unsigned long WIFI_CONNECT_TIMEOUT_MS = 45000;
const unsigned long SENSOR_READ_INTERVAL_MS = 30000;        // 30 segundos
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 10000;     // 10 segundos

// Pines
const int SOIL_ANALOG_PIN = 34;

// Si quieres usar tambien D0 del sensor, cambia -1 por un pin, por ejemplo 35.
const int SOIL_DIGITAL_PIN = -1;

// I2C del BME688 en ESP32 DevKit
const int BME_SDA_PIN = 21;
const int BME_SCL_PIN = 22;

// Calibracion inicial del sensor de humedad.
// Segun tus pruebas anteriores: seco aprox 4095, mojado aprox 1700.
int SOIL_DRY_ADC = 4095;
int SOIL_WET_ADC = 1700;

// =====================================================
// OBJETOS Y VARIABLES GLOBALES
// =====================================================

httpd_handle_t web_httpd = NULL;     // Pagina + /sensor en puerto 80
httpd_handle_t sensor_httpd = NULL;  // /sensor extra en puerto 81

Adafruit_BME680 bme;

bool usingAccessPoint = false;
bool bmeDetected = false;
uint8_t bmeAddress = 0x00;

unsigned long lastSensorReadMillis = 0;
unsigned long lastWifiReconnectAttempt = 0;

SemaphoreHandle_t dataMutex = NULL;

struct SystemData {
  bool bmeOk = false;
  bool soilOk = false;

  float temperature = NAN;
  float humidityAir = NAN;
  float pressure = NAN;
  float gasResistance = NAN;

  int soilRaw = 0;
  float soilVoltage = NAN;
  float soilPercent = NAN;
  int soilDigital = -1;

  const char* soilStatus = "SIN DATOS";
  const char* recommendation = "Esperando primera lectura...";

  unsigned long millisTime = 0;
};

SystemData lastData;

// =====================================================
// PAGINA WEB ESTATICA EN FLASH
// =====================================================
//
// Esta pagina carga rapido porque no tiene placeholders ni String.replace().
// Al abrir la IP, el navegador recibe HTML fijo y despues pide los datos a /sensor.

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta http-equiv="Cache-Control" content="no-store">
  <title>Cama de Cultivo IoT</title>
  <style>
    *{box-sizing:border-box}
    body{
      margin:0;
      padding:12px;
      font-family:Arial,Helvetica,sans-serif;
      background:#0f1f13;
      color:#f7fff3;
    }
    .wrap{width:100%;max-width:980px;margin:0 auto}
    .hero,.card,.footer{
      background:#14291a;
      border:1px solid #2f5736;
      border-radius:14px;
      padding:12px;
      margin-bottom:10px;
      box-shadow:0 8px 18px rgba(0,0,0,.22);
    }
    h1{font-size:22px;margin:0 0 4px}
    .sub{color:#c8e8bd;font-size:13px;line-height:1.35}
    .statusbar,.grid{
      display:grid;
      gap:8px;
    }
    .statusbar{grid-template-columns:repeat(4,1fr);margin-top:10px}
    .grid{grid-template-columns:repeat(3,1fr)}
    .mini{
      background:#0e1d12;
      border:1px solid #29482e;
      border-radius:10px;
      padding:8px;
      min-height:54px;
    }
    .mini-label,.card-title,.unit{color:#accb9e;font-size:12px}
    .mini-value{font-size:13px;font-weight:bold;margin-top:4px;word-break:break-word}
    .card-title{margin-bottom:6px}
    .value{font-size:30px;font-weight:800;line-height:1.05}
    .unit{margin-top:3px}
    .wide{grid-column:span 3}
    .soil-box{
      margin-top:10px;
      background:#243324;
      border-radius:999px;
      height:16px;
      overflow:hidden;
      border:1px solid #3f5e3f;
    }
    .soil-fill{
      height:100%;
      width:0%;
      background:linear-gradient(90deg,#d9822b,#e9c46a,#6ab04c);
      transition:width .25s ease;
    }
    .badge{
      display:inline-block;
      margin-top:8px;
      padding:6px 9px;
      border-radius:999px;
      font-weight:bold;
      font-size:12px;
      background:#263627;
      border:1px solid #496349;
    }
    .recommendation{font-size:16px;line-height:1.4;margin-top:3px}
    .ok{color:#7cff9b;font-weight:bold}
    .bad{color:#ff8585;font-weight:bold}
    .footer{color:#cfe8c3;font-size:12px;line-height:1.5;word-break:break-word}
    a{color:#b9ff8a;text-decoration:none;font-weight:bold}
    @media(max-width:760px){
      body{padding:8px}
      h1{font-size:19px}
      .statusbar,.grid{grid-template-columns:repeat(2,1fr)}
      .wide{grid-column:span 2}
      .value{font-size:26px}
    }
    @media(max-width:480px){
      .statusbar,.grid{grid-template-columns:1fr}
      .wide{grid-column:span 1}
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="hero">
      <h1>🌱 Cama de Cultivo IoT</h1>
      <div class="sub">Humedad del suelo, temperatura, humedad ambiental, presión y gas. Actualización cada 30 segundos.</div>
      <div class="statusbar">
        <div class="mini"><div class="mini-label">Equipo</div><div class="mini-value" id="deviceId">--</div></div>
        <div class="mini"><div class="mini-label">IP</div><div class="mini-value" id="ip">--</div></div>
        <div class="mini"><div class="mini-label">WiFi</div><div class="mini-value" id="ssid">--</div></div>
        <div class="mini"><div class="mini-label">BSSID</div><div class="mini-value" id="bssid">--</div></div>
      </div>
    </div>

    <div class="grid">
      <div class="card">
        <div class="card-title">Humedad de suelo</div>
        <div class="value"><span id="soilPercent">--</span></div>
        <div class="unit">%</div>
        <div class="soil-box"><div id="soilFill" class="soil-fill"></div></div>
        <div class="badge" id="soilStatus">SIN DATOS</div>
      </div>

      <div class="card">
        <div class="card-title">Temperatura ambiente</div>
        <div class="value"><span id="temp">--</span></div>
        <div class="unit">°C</div>
      </div>

      <div class="card">
        <div class="card-title">Humedad ambiental</div>
        <div class="value"><span id="humAir">--</span></div>
        <div class="unit">%</div>
      </div>

      <div class="card">
        <div class="card-title">Presión barométrica</div>
        <div class="value"><span id="pressure">--</span></div>
        <div class="unit">hPa</div>
      </div>

      <div class="card">
        <div class="card-title">Gas / VOC relativo</div>
        <div class="value"><span id="gas">--</span></div>
        <div class="unit">Ω</div>
      </div>

      <div class="card">
        <div class="card-title">Lectura cruda suelo</div>
        <div class="value"><span id="soilRaw">--</span></div>
        <div class="unit">ADC 0-4095</div>
      </div>

      <div class="card wide">
        <div class="card-title">Recomendación</div>
        <div class="recommendation" id="recommendation">Cargando datos...</div>
      </div>

      <div class="card">
        <div class="card-title">BME688</div>
        <div class="value"><span id="bmeStatus">--</span></div>
        <div class="unit">estado</div>
      </div>

      <div class="card">
        <div class="card-title">Última lectura</div>
        <div class="value"><span id="lastMillis">--</span></div>
        <div class="unit">millis</div>
      </div>

      <div class="card">
        <div class="card-title">Señal WiFi</div>
        <div class="value"><span id="rssi">--</span></div>
        <div class="unit">dBm</div>
      </div>
    </div>

    <div class="footer">
      Modo de conexión: <b id="mode">--</b><br>
      Dirección BME688: <b id="bmeAddr">--</b><br>
      JSON del sistema: <a href="/sensor" target="_blank">/sensor</a>
    </div>
  </div>

<script>
  const SENSOR_URL = "/sensor";
  const REFRESH_MS = 30000;

  function setText(id, value) {
    const el = document.getElementById(id);
    if (el) el.textContent = value;
  }

  function fmt(value, decimals) {
    if (value === null || value === undefined || Number.isNaN(Number(value))) return "--";
    return Number(value).toFixed(decimals);
  }

  function updateSoilBar(percent) {
    const fill = document.getElementById("soilFill");
    if (!fill) return;
    let p = Number(percent);
    if (Number.isNaN(p)) p = 0;
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    fill.style.width = p + "%";
  }

  async function updateSensor() {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 4500);

    try {
      const res = await fetch(SENSOR_URL + "?t=" + Date.now(), {
        cache: "no-store",
        signal: controller.signal
      });

      const data = await res.json();
      clearTimeout(timer);

      setText("deviceId", data.device_id || "--");
      setText("ip", "http://" + (data.ip_address || location.host));
      setText("ssid", data.wifi_ssid || "--");
      setText("bssid", data.wifi_bssid || "--");
      setText("mode", data.connection_mode || "--");

      setText("soilPercent", fmt(data.soil_moisture_percent, 1));
      setText("soilRaw", data.soil_raw_adc ?? "--");
      setText("soilStatus", data.soil_status || "SIN DATOS");

      setText("temp", fmt(data.temperature_c, 2));
      setText("humAir", fmt(data.air_humidity_percent, 2));
      setText("pressure", fmt(data.pressure_hpa, 2));
      setText("gas", fmt(data.gas_resistance_ohms, 0));

      setText("recommendation", data.recommendation || "--");
      setText("lastMillis", data.millis ?? "--");
      setText("rssi", data.rssi_dbm ?? "--");
      setText("bmeAddr", data.bme_i2c_address || "--");

      const bmeStatus = document.getElementById("bmeStatus");
      if (bmeStatus) {
        bmeStatus.textContent = data.bme_ok ? "OK" : "ERROR";
        bmeStatus.className = data.bme_ok ? "ok" : "bad";
      }

      updateSoilBar(data.soil_moisture_percent);
    } catch (err) {
      clearTimeout(timer);
      const bmeStatus = document.getElementById("bmeStatus");
      if (bmeStatus) {
        bmeStatus.textContent = "SIN RESPUESTA";
        bmeStatus.className = "bad";
      }
      setText("recommendation", "No hubo respuesta del ESP32. Revisa señal WiFi, IP fija o alimentación.");
    }
  }

  updateSensor();
  setInterval(updateSensor, REFRESH_MS);
</script>
</body>
</html>
)rawliteral";

// =====================================================
// FUNCIONES AUXILIARES
// =====================================================

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float mapSoilPercent(int raw) {
  // En este sensor normalmente:
  // seco = valor alto
  // mojado = valor bajo
  float denom = (float)SOIL_DRY_ADC - (float)SOIL_WET_ADC;
  if (denom == 0) return NAN;

  float percent = ((float)SOIL_DRY_ADC - (float)raw) * 100.0 / denom;
  return clampFloat(percent, 0.0, 100.0);
}

void floatJson(char* out, size_t outSize, float value, int decimals) {
  if (isnan(value)) {
    snprintf(out, outSize, "null");
    return;
  }

  if (decimals == 0) {
    snprintf(out, outSize, "%.0f", value);
  } else if (decimals == 1) {
    snprintf(out, outSize, "%.1f", value);
  } else if (decimals == 2) {
    snprintf(out, outSize, "%.2f", value);
  } else {
    snprintf(out, outSize, "%.3f", value);
  }
}

String getCurrentIP() {
  if (usingAccessPoint) return WiFi.softAPIP().toString();
  return WiFi.localIP().toString();
}

String getCurrentSSID() {
  if (usingAccessPoint) return String(AP_SSID);
  return WiFi.SSID();
}

String getCurrentBSSID() {
  if (!usingAccessPoint && WiFi.status() == WL_CONNECTED) {
    return WiFi.BSSIDstr();
  }

  if (usingAccessPoint) {
    return WiFi.softAPmacAddress();
  }

  return "SIN_BSSID";
}

String getConnectionMode() {
  if (usingAccessPoint) return "Red propia del ESP32/AP";
  if (WiFi.status() == WL_CONNECTED) return "Conectado a WiFi con IP fija";
  return "Sin conexion";
}

String wifiStatusToText(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS: esperando";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL: no se encontro el SSID";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED: escaneo completado";
    case WL_CONNECTED: return "WL_CONNECTED: conectado";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED: fallo de conexion/password";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST: conexion perdida";
    case WL_DISCONNECTED: return "WL_DISCONNECTED: desconectado";
    default: return "Estado WiFi desconocido: " + String((int)status);
  }
}

String encryptionTypeToText(wifi_auth_mode_t type) {
  switch (type) {
    case WIFI_AUTH_OPEN: return "Abierta";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Enterprise";
    default: return "Seguridad tipo " + String((int)type);
  }
}

// =====================================================
// WIFI
// =====================================================

bool scanForConfiguredSSID() {
  Serial.println();
  Serial.println("Escaneando redes WiFi visibles...");

  int networkCount = WiFi.scanNetworks(false, true);

  if (networkCount <= 0) {
    Serial.println("No se detectaron redes WiFi.");
    return false;
  }

  bool foundTarget = false;

  Serial.print("Redes encontradas: ");
  Serial.println(networkCount);

  for (int i = 0; i < networkCount; i++) {
    String ssid = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    uint8_t channel = WiFi.channel(i);
    wifi_auth_mode_t enc = WiFi.encryptionType(i);

    Serial.print(i + 1);
    Serial.print(") SSID: ");
    Serial.print(ssid.length() ? ssid : "<oculta>");
    Serial.print(" | RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm | Canal: ");
    Serial.print(channel);
    Serial.print(" | Seguridad: ");
    Serial.println(encryptionTypeToText(enc));

    if (ssid == WIFI_SSID) {
      foundTarget = true;
    }
  }

  if (foundTarget) {
    Serial.print("La red configurada SI fue detectada: ");
    Serial.println(WIFI_SSID);
  } else {
    Serial.print("La red configurada NO fue detectada: ");
    Serial.println(WIFI_SSID);
    Serial.println("Puede estar fuera de rango, ser solo 5 GHz o tener senal debil.");
  }

  WiFi.scanDelete();
  Serial.println();
  return foundTarget;
}

void startFallbackAccessPoint() {
  Serial.println("Creando red propia del ESP32 como respaldo...");

  WiFi.disconnect(true, true);
  delay(500);

  WiFi.mode(WIFI_AP);
  delay(500);

  usingAccessPoint = true;

  IPAddress apIP(192, 168, 4, 1);
  IPAddress apGateway(192, 168, 4, 1);
  IPAddress apSubnet(255, 255, 255, 0);

  WiFi.softAPConfig(apIP, apGateway, apSubnet);

  bool apOk = WiFi.softAP(AP_SSID, AP_PASSWORD, 6, 0, 4);

  if (apOk) {
    Serial.println("Red propia creada correctamente.");
    Serial.print("Nombre de red: ");
    Serial.println(AP_SSID);
    Serial.print("Contrasena: ");
    Serial.println(AP_PASSWORD);
    Serial.print("Abre en navegador: http://");
    Serial.println(WiFi.softAPIP());
    Serial.println("Si el celular dice 'sin internet', selecciona mantener conexion.");
  } else {
    Serial.println("Error: no se pudo crear la red propia.");
  }
}

void startWiFiFixed() {
  Serial.println();
  Serial.println("========== WIFI IP FIJA ==========");
  Serial.print("Equipo: ");
  Serial.println(DEVICE_ID);
  Serial.print("SSID configurado: ");
  Serial.println(WIFI_SSID);

  usingAccessPoint = false;

  WiFi.persistent(false);
  WiFi.disconnect(true, true);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("MAC WiFi del ESP32: ");
  Serial.println(WiFi.macAddress());

  scanForConfiguredSSID();

  Serial.println("Configurando IP fija...");
  Serial.print("LOCAL_IP: ");
  Serial.println(LOCAL_IP);
  Serial.print("GATEWAY: ");
  Serial.println(GATEWAY);
  Serial.print("SUBNET: ");
  Serial.println(SUBNET);

  bool configOk = WiFi.config(LOCAL_IP, GATEWAY, SUBNET, PRIMARY_DNS, SECONDARY_DNS);

  if (!configOk) {
    Serial.println("Advertencia: WiFi.config fallo. Intentare conectar de todos modos.");
  }

  Serial.println("Conectando a WiFi con IP fija...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");

    if ((millis() - startAttemptTime) % 5000 < 600) {
      Serial.print(" [");
      Serial.print(wifiStatusToText(WiFi.status()));
      Serial.print("] ");
    }
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    usingAccessPoint = false;

    Serial.println("WiFi conectado correctamente con IP fija.");
    Serial.print("SSID conectado: ");
    Serial.println(WiFi.SSID());
    Serial.print("BSSID conectado: ");
    Serial.println(WiFi.BSSIDstr());
    Serial.print("IP fija: http://");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway/router: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("Mascara de subred: ");
    Serial.println(WiFi.subnetMask());
    Serial.print("DNS: ");
    Serial.println(WiFi.dnsIP());
    Serial.print("RSSI/senal: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

    if (WiFi.localIP() != LOCAL_IP) {
      Serial.println("Aviso: la IP final no coincide con LOCAL_IP. Revisa gateway/subnet o conflicto de red.");
    }
  } else {
    Serial.println("No se pudo conectar a Primavera26 con IP fija.");
    Serial.print("Estado final: ");
    Serial.println(wifiStatusToText(WiFi.status()));
    Serial.println("Se inicia red propia para poder entrar a la pagina.");
    startFallbackAccessPoint();
  }

  Serial.println("==================================");
  Serial.println();
}

void maintainWiFi() {
  if (usingAccessPoint) return;
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - lastWifiReconnectAttempt < WIFI_RECONNECT_INTERVAL_MS) return;

  lastWifiReconnectAttempt = now;

  Serial.println("WiFi desconectado. Intentando reconectar...");
  Serial.print("Estado actual: ");
  Serial.println(wifiStatusToText(WiFi.status()));
  WiFi.disconnect(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// =====================================================
// BME688
// =====================================================

bool startBME688() {
  Serial.println();
  Serial.println("========== BME688 ==========");
  Serial.print("Iniciando I2C en SDA GPIO");
  Serial.print(BME_SDA_PIN);
  Serial.print(" / SCL GPIO");
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
    Serial.println("Error: no se detecto el BME688 en 0x76 ni 0x77.");
    Serial.println("Revisa VCC, GND, SDA y SCL.");
    Serial.println("============================");
    Serial.println();
    return false;
  }

  Serial.print("BME688 detectado en direccion I2C 0x");
  Serial.println(bmeAddress, HEX);

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);

  // Calentador interno para lectura de gas.
  bme.setGasHeater(320, 150);

  Serial.println("BME688 iniciado correctamente.");
  Serial.println("============================");
  Serial.println();

  return true;
}

// =====================================================
// LECTURA DE SENSORES
// =====================================================

SystemData readAllSensors() {
  SystemData data;
  data.millisTime = millis();

  // ---------- Sensor humedad suelo ----------
  data.soilRaw = analogRead(SOIL_ANALOG_PIN);
  data.soilVoltage = (data.soilRaw * 3.3) / 4095.0;
  data.soilPercent = mapSoilPercent(data.soilRaw);
  data.soilOk = true;

  if (SOIL_DIGITAL_PIN >= 0) {
    data.soilDigital = digitalRead(SOIL_DIGITAL_PIN);
  } else {
    data.soilDigital = -1;
  }

  if (data.soilPercent >= 70.0) {
    data.soilStatus = "HUMEDAD ALTA";
    data.recommendation = "La cama de cultivo tiene buena humedad. No se recomienda riego.";
  } else if (data.soilPercent >= 35.0) {
    data.soilStatus = "HUMEDAD MEDIA";
    data.recommendation = "La humedad es aceptable. Vigilar antes de activar riego.";
  } else {
    data.soilStatus = "SUELO SECO";
    data.recommendation = "La cama de cultivo esta seca. Se recomienda revisar riego.";
  }

  // ---------- BME688 ----------
  if (!bmeDetected) {
    data.bmeOk = false;
    return data;
  }

  if (bme.performReading()) {
    data.bmeOk = true;
    data.temperature = bme.temperature;
    data.humidityAir = bme.humidity;
    data.pressure = bme.pressure / 100.0;       // Pa a hPa
    data.gasResistance = bme.gas_resistance;    // Ohms
  } else {
    data.bmeOk = false;
  }

  return data;
}

void printSystemData(const SystemData& data) {
  Serial.println("========== LECTURA CAMAS CULTIVO ==========");
  Serial.print("Millis: ");
  Serial.println(data.millisTime);

  Serial.print("Humedad suelo raw ADC: ");
  Serial.println(data.soilRaw);

  Serial.print("Voltaje AO suelo: ");
  Serial.print(data.soilVoltage, 3);
  Serial.println(" V");

  Serial.print("Humedad suelo estimada: ");
  Serial.print(data.soilPercent, 1);
  Serial.println(" %");

  Serial.print("Estado suelo: ");
  Serial.println(data.soilStatus);

  Serial.print("BME688: ");
  Serial.println(data.bmeOk ? "OK" : "ERROR");

  if (data.bmeOk) {
    Serial.print("Temperatura ambiente: ");
    Serial.print(data.temperature, 2);
    Serial.println(" C");

    Serial.print("Humedad ambiente: ");
    Serial.print(data.humidityAir, 2);
    Serial.println(" %");

    Serial.print("Presion: ");
    Serial.print(data.pressure, 2);
    Serial.println(" hPa");

    Serial.print("Gas: ");
    Serial.print(data.gasResistance, 0);
    Serial.println(" Ohms");
  }

  Serial.print("Recomendacion: ");
  Serial.println(data.recommendation);

  Serial.println("===========================================");
  Serial.println();
}

// =====================================================
// ENVIO PUSH A AWS ASINCRONO
// =====================================================

void enviarDatosAWS(const SystemData& data) {
  if (usingAccessPoint || WiFi.status() != WL_CONNECTED || !data.bmeOk) return;

  WiFiClient client;
  HTTPClient http;

  if (!http.begin(client, AWS_HUERTO_URL)) {
    Serial.println("[AWS] Error al inicializar HTTPClient");
    return;
  }

  http.setTimeout(3000); // Timeout de 3 segundos no bloqueante
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-device-secret", DEVICE_SECRET);

  // Convertir floats a texto limpio para el JSON saliente
  char tStr[16], hStr[16], pStr[16], gStr[16], smStr[16];
  floatJson(tStr, sizeof(tStr), data.temperature, 2);
  floatJson(hStr, sizeof(hStr), data.humidityAir, 2);
  floatJson(pStr, sizeof(pStr), data.pressure, 2);
  floatJson(gStr, sizeof(gStr), data.gasResistance, 0);
  floatJson(smStr, sizeof(smStr), data.soilPercent, 1);

  String jsonBody = "{";
  jsonBody += "\"device_id\":\"cama_cultivo_01\",";
  jsonBody += "\"temperature\":" + String(tStr) + ",";
  jsonBody += "\"humidity\":" + String(hStr) + ",";
  jsonBody += "\"pressure\":" + String(pStr) + ",";
  jsonBody += "\"gas\":" + String(gStr) + ",";
  jsonBody += "\"soil_raw\":" + String(data.soilRaw) + ",";
  jsonBody += "\"soil_percent\":" + String(smStr);
  jsonBody += "}";

  int httpCode = http.POST(jsonBody);
  if (httpCode > 0) {
    Serial.printf("[AWS Push] Servidor respondió: %d\n", httpCode);
  } else {
    Serial.printf("[AWS Push] Error de red: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

// Hilo asíncrono periódico en segundo plano
void awsPushTask(void *parameter) {
  Serial.println("[FreeRTOS] Tarea asíncrona de envío a AWS iniciada.");
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(30000)); // Esperar exactamente 30 segundos
    SystemData dataCopy;
    if (dataMutex != NULL && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      dataCopy = lastData;
      xSemaphoreGive(dataMutex);
      enviarDatosAWS(dataCopy);
    }
  }
}

void updateSensorsIfNeeded() {
  unsigned long now = millis();

  if (now - lastSensorReadMillis >= SENSOR_READ_INTERVAL_MS) {
    SystemData newData = readAllSensors();

    if (dataMutex != NULL) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      lastData = newData;
      xSemaphoreGive(dataMutex);
    } else {
      lastData = newData;
    }

    printSystemData(newData);
    lastSensorReadMillis = now;
  }
}

// =====================================================
// SERVIDOR WEB HTTPD
// =====================================================

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Connection", "close");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t favicon_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400");
  return httpd_resp_send(req, "", 0);
}

static esp_err_t health_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Connection", "close");
  return httpd_resp_sendstr(req, "OK");
}

static esp_err_t sensor_handler(httpd_req_t *req) {
  SystemData data;

  if (dataMutex != NULL) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    data = lastData;
    xSemaphoreGive(dataMutex);
  } else {
    data = lastData;
  }

  char soilVoltage[20];
  char soilPercent[20];
  char temp[20];
  char humAir[20];
  char pressure[20];
  char gas[24];

  floatJson(soilVoltage, sizeof(soilVoltage), data.soilVoltage, 3);
  floatJson(soilPercent, sizeof(soilPercent), data.soilPercent, 1);
  floatJson(temp, sizeof(temp), data.temperature, 2);
  floatJson(humAir, sizeof(humAir), data.humidityAir, 2);
  floatJson(pressure, sizeof(pressure), data.pressure, 2);
  floatJson(gas, sizeof(gas), data.gasResistance, 0);

  char bmeAddr[18];
  if (bmeDetected) {
    snprintf(bmeAddr, sizeof(bmeAddr), "0x%02X", bmeAddress);
  } else {
    snprintf(bmeAddr, sizeof(bmeAddr), "NO_DETECTADO");
  }

  String connectionMode = getConnectionMode();
  String ssid = getCurrentSSID();
  String bssid = getCurrentBSSID();
  String ip = getCurrentIP();
  String gateway = usingAccessPoint ? WiFi.softAPIP().toString() : WiFi.gatewayIP().toString();
  String subnetText = usingAccessPoint ? String("255.255.255.0") : WiFi.subnetMask().toString();

  char json[1600];

  int len = snprintf(
    json,
    sizeof(json),
    "{"
      "\"device_id\":\"%s\","
      "\"connection_mode\":\"%s\","
      "\"wifi_ssid\":\"%s\","
      "\"wifi_bssid\":\"%s\","
      "\"ip_address\":\"%s\","
      "\"dhcp_enabled\":false,"
      "\"gateway\":\"%s\","
      "\"subnet\":\"%s\","
      "\"rssi_dbm\":%ld,"
      "\"sensor_interval_s\":%lu,"
      "\"soil_ok\":%s,"
      "\"soil_raw_adc\":%d,"
      "\"soil_voltage_v\":%s,"
      "\"soil_moisture_percent\":%s,"
      "\"soil_digital\":%d,"
      "\"soil_status\":\"%s\","
      "\"recommendation\":\"%s\","
      "\"bme_ok\":%s,"
      "\"bme_detected\":%s,"
      "\"bme_i2c_address\":\"%s\","
      "\"millis\":%lu,"
      "\"temperature_c\":%s,"
      "\"air_humidity_percent\":%s,"
      "\"pressure_hpa\":%s,"
      "\"gas_resistance_ohms\":%s,"
      "\"heap_free\":%u"
    "}",
    DEVICE_ID,
    connectionMode.c_str(),
    ssid.c_str(),
    bssid.c_str(),
    ip.c_str(),
    gateway.c_str(),
    subnetText.c_str(),
    (long)((WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0),
    SENSOR_READ_INTERVAL_MS / 1000,
    data.soilOk ? "true" : "false",
    data.soilRaw,
    soilVoltage,
    soilPercent,
    data.soilDigital,
    data.soilStatus,
    data.recommendation,
    data.bmeOk ? "true" : "false",
    bmeDetected ? "true" : "false",
    bmeAddr,
    data.millisTime,
    temp,
    humAir,
    pressure,
    gas,
    ESP.getFreeHeap()
  );

  if (len < 0 || len >= (int)sizeof(json)) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Connection", "close");

  return httpd_resp_send(req, json, len);
}

void registerCommonRoutes(httpd_handle_t server, bool includeIndex) {
  if (includeIndex) {
    httpd_uri_t index_uri = {};
    index_uri.uri = "/";
    index_uri.method = HTTP_GET;
    index_uri.handler = index_handler;
    index_uri.user_ctx = NULL;
    httpd_register_uri_handler(server, &index_uri);

    httpd_uri_t favicon_uri = {};
    favicon_uri.uri = "/favicon.ico";
    favicon_uri.method = HTTP_GET;
    favicon_uri.handler = favicon_handler;
    favicon_uri.user_ctx = NULL;
    httpd_register_uri_handler(server, &favicon_uri);

    httpd_uri_t health_uri = {};
    health_uri.uri = "/health";
    health_uri.method = HTTP_GET;
    health_uri.handler = health_handler;
    health_uri.user_ctx = NULL;
    httpd_register_uri_handler(server, &health_uri);
  }

  httpd_uri_t sensor_uri = {};
  sensor_uri.uri = "/sensor";
  sensor_uri.method = HTTP_GET;
  sensor_uri.handler = sensor_handler;
  sensor_uri.user_ctx = NULL;
  httpd_register_uri_handler(server, &sensor_uri);
}

void startWebServers() {
  // Puerto 80: pagina principal + /sensor.
  httpd_config_t web_config = HTTPD_DEFAULT_CONFIG();
  web_config.server_port = 80;
  web_config.ctrl_port = 32768;
  web_config.max_uri_handlers = 8;
  web_config.stack_size = 8192;
  web_config.max_open_sockets = 6;
  web_config.lru_purge_enable = true;
  web_config.recv_wait_timeout = 3;
  web_config.send_wait_timeout = 3;

  if (httpd_start(&web_httpd, &web_config) == ESP_OK) {
    registerCommonRoutes(web_httpd, true);
    Serial.println("Servidor web rapido iniciado en puerto 80.");
  } else {
    Serial.println("Error iniciando servidor web en puerto 80.");
  }

  // Puerto 81: /sensor extra, igual que en el proyecto rapido de camara.
  // No cambia el funcionamiento existente porque /sensor en puerto 80 sigue disponible.
  httpd_config_t sensor_config = HTTPD_DEFAULT_CONFIG();
  sensor_config.server_port = 81;
  sensor_config.ctrl_port = 32769;
  sensor_config.max_uri_handlers = 4;
  sensor_config.stack_size = 6144;
  sensor_config.max_open_sockets = 3;
  sensor_config.lru_purge_enable = true;
  sensor_config.recv_wait_timeout = 3;
  sensor_config.send_wait_timeout = 3;

  if (httpd_start(&sensor_httpd, &sensor_config) == ESP_OK) {
    registerCommonRoutes(sensor_httpd, false);
    Serial.println("Servidor JSON extra iniciado en puerto 81.");
  } else {
    Serial.println("Aviso: no se pudo iniciar servidor extra en puerto 81. El /sensor de puerto 80 sigue activo.");
  }
}

// =====================================================
// SETUP Y LOOP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("====================================");
  Serial.println("INICIANDO CAMA DE CULTIVO IoT");
  Serial.println("Version rapida basada en arquitectura multiusuario");
  Serial.println("====================================");

  Serial.print("Equipo: ");
  Serial.println(DEVICE_ID);

  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    Serial.println("Advertencia: no se pudo crear mutex de datos.");
  }

  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_ANALOG_PIN, ADC_11db);

  if (SOIL_DIGITAL_PIN >= 0) {
    pinMode(SOIL_DIGITAL_PIN, INPUT);
  }

  startBME688();

  // Primera lectura antes de abrir la pagina.
  SystemData firstData = readAllSensors();
  if (dataMutex != NULL) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    lastData = firstData;
    xSemaphoreGive(dataMutex);
  } else {
    lastData = firstData;
  }
  printSystemData(firstData);
  lastSensorReadMillis = millis();

  startWiFiFixed();

  startWebServers();

  // Lanzar la tarea asíncrona a la pila del procesador
  xTaskCreate(awsPushTask, "AWS_Push", 8192, NULL, 1, &awsTaskHandle);
  Serial.println("Transmisión paralela hacia AWS activada.");

  Serial.println();
  Serial.println("========== SISTEMA LISTO ==========");
  Serial.print("Abre esta direccion en el navegador: http://");
  Serial.println(getCurrentIP());
  Serial.print("JSON sensores principal: http://");
  Serial.print(getCurrentIP());
  Serial.println("/sensor");
  Serial.print("JSON sensores extra: http://");
  Serial.print(getCurrentIP());
  Serial.println(":81/sensor");
  Serial.print("Health check: http://");
  Serial.print(getCurrentIP());
  Serial.println("/health");
  Serial.println("===================================");
  Serial.println();
}

void loop() {
  maintainWiFi();
  updateSensorsIfNeeded();

  // esp_http_server corre en su propia tarea.
  // No usamos server.handleClient().
  delay(50);
}
