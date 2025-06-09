#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#define DEBUG_SERIAL Serial

const char* ssid = "ScantoolHonda"; // SSID dari ESP32 sebagai server (Lokal tidak terkoneksi internet)
const char* password = ""; // Sengaja tidak diberi Password
IPAddress local_IP(192, 168, 4, 1); // IP Adress bisa di setting sendiri
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

byte WAKEUP[]    = { 0xFE, 0x04, 0x72, 0x8C };
byte TABLE[]     = { 0x72, 0x05, 0x71, 0x17, 0x01 };
byte buff[29];
byte buffCount = 0;

int   OBD_RPM = 0;
float OBD_TPS1_MV = 0.0;
float OBD_TPS2_PCT = 0.0;
float OBD_ECT1_MV = 0.0;
int   OBD_ECT2_C = 0;
float OBD_IAT1_MV = 0.0;
int   OBD_IAT2_C = 0;
float OBD_MAP1_MV = 0.0;
int   OBD_MAP2_KPA = 0;
float OBD_BAT_V = 0.0;
float OBD_INJ_MS = 0.0;
float OBD_IGT_DEG = 0.0;
int   OBD_SPEED_KMH = 0;

HardwareSerial obdHonda(2);
WebServer server(80);

struct SensorData {
  int obd_rpm;
  float obd_tps1_mv;
  float obd_tps2_pct;
  float obd_ect1_mv;
  int obd_ect2_c;
  float obd_iat1_mv;
  int obd_iat2_c;
  float obd_map1_mv;
  int obd_map2_kpa;
  float obd_bat_v;
  float obd_inj_ms;
  float obd_igt_deg;
  int obd_speed_kmh;
  String timestamp;
};

#define MAX_DATA_POINTS 20
SensorData dataHistory[MAX_DATA_POINTS];
int currentDataIndex = 0;
bool dataAvailable = false;

unsigned long lastDataUpdateTime = 0;
const unsigned long dataUpdateInterval = 300;

void handleRoot();
void handleData();
void scanTool();
void processAndStoreOBDData();

const char HTML_PAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="id">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>ScanTool</title>
  <style>
    body { 
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif, "Apple Color Emoji", "Segoe UI Emoji", "Segoe UI Symbol";
      margin: 0; 
      padding: 20px; 
      background: linear-gradient(135deg, #2D3436 0%, #1E272E 100%);
      color: #E0E0E0; 
      line-height: 1.6;
    }
    .header { 
      text-align: center; 
      margin-bottom: 30px; 
      padding: 25px 20px; 
      background-color: #1E272E;
      border-radius: 12px; 
      color: white; 
      box-shadow: 0 5px 20px rgba(0,0,0,0.4);
      border: 1px solid #4A4A4A;
    }
    .title { font-size: 26px; margin: 0 0 5px 0; font-weight: 600; }
    .subtitle { font-size: 18px; margin: 0 0 10px 0; color: #B0B0B0; font-weight: 300; }
    .author { margin-top: 15px; font-style: normal; font-size: 14px; color: #888888;}
    .special-title { color:#FDCB6E; margin-top:10px; font-size: 20px; font-weight: 400; }

    .nav { 
      margin-bottom: 25px; 
      background-color: #3B3B3B;
      padding: 15px; 
      border-radius: 8px; 
      text-align: center; 
      box-shadow: 0 3px 10px rgba(0,0,0,0.3);
    }
    .nav a { 
      margin: 0 12px; 
      text-decoration: none; 
      color: #E0E0E0; 
      font-weight: 500; 
      font-size: 15px; 
      transition: color 0.2s ease-in-out;
      display: inline-block;
    }
    .nav a:hover { color: #FDCB6E; }

    .readings-container, .data-container { 
      background: #2C2C2E; 
      padding: 20px; 
      border-radius: 12px; 
      color: #E0E0E0; 
      margin-bottom: 25px; 
      box-shadow: 0 4px 15px rgba(0,0,0,0.3);
      border: 1px solid #4A4A4A;
    }
    .readings-container h2 { margin-top:0; color: #A29BFE; font-size: 20px; border-bottom: 1px solid #444; padding-bottom: 10px; margin-bottom: 20px; text-align:center; }
    
    .data-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 15px;
    }
    .data-card {
      background-color: #3a3a3c;
      border-radius: 10px;
      padding: 15px;
      text-align: center;
      display: flex;
      flex-direction: column;
      justify-content: center;
      min-height: 100px;
    }
    .data-card .label {
      font-size: 14px;
      color: #b0b0b0;
      margin-bottom: 8px;
    }
    .data-card .value {
      font-size: 2.2rem; /* Ukuran font besar untuk nilai */
      font-weight: bold;
      color: #ffffff;
      line-height: 1.1;
    }
    .data-card .unit {
      font-size: 1rem;
      color: #b0b0b0;
      margin-left: 4px;
    }

    .table-responsive-wrapper {
      overflow-x: auto;
      width: 100%;
    }
    table { 
      width: 100%; 
      border-collapse: collapse; 
      margin-top: 0;
      background: transparent; 
      color: #E0E0E0; 
      font-size: 14px;
    }
    th, td { 
      border: 1px solid #454d55; 
      padding: 10px 12px;
      text-align: left; 
      white-space: nowrap;
    }
    th { 
      background-color: #343a40;
      color: #F8F9FA;
      font-weight: 600;
      font-size: 13px;
    }
    tr:nth-child(even) { background-color: #31353a; }

    @media (max-width: 480px) {
      .data-grid {
        grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      }
      .data-card .value { font-size: 1.8rem; }
    }
  </style>
</head>
<body>
  <div class="header">
    <h1 class="title">SMK GAJAH MADA BANYUWANGI</h1>
    <h2 class="subtitle">PROGRAM KEAHLIAN TEKNIK ELEKTRONIKA</h2>
  </div>
  <div class="nav">
    <h1 class="special-title">SCANTOOL HONDA</h1>
  </div>
  
  <div class="readings-container">
    <div class="data-grid" id="data-grid-container">
      <div class="data-card"><div class="label">RPM</div><div class="value" id="val-rpm">0</div></div>
      <div class="data-card"><div class="label">TPS VOLT</div><div class="value" id="val-tps-mv">0.0<span class="unit">mV</span></div></div>
      <div class="data-card"><div class="label">TPS</div><div class="value" id="val-tps">0.00<span class="unit">%</span></div></div>
      <div class="data-card"><div class="label">ECT VOLT</div><div class="value" id="val-ect-mv">0.0<span class="unit">mV</span></div></div>
      <div class="data-card"><div class="label">ECT</div><div class="value" id="val-ect">0<span class="unit">°C</span></div></div>
      <div class="data-card"><div class="label">IAT VOLT</div><div class="value" id="val-iat-mv">0.0<span class="unit">mV</span></div></div>
      <div class="data-card"><div class="label">IAT</div><div class="value" id="val-iat">0<span class="unit">°C</span></div></div>
      <div class="data-card"><div class="label">MAP VOLT</div><div class="value" id="val-map-mv">0.0<span class="unit">mV</span></div></div>
      <div class="data-card"><div class="label">MAP</div><div class="value" id="val-map">0<span class="unit">kPa</span></div></div>
      <div class="data-card"><div class="label">BATTERY</div><div class="value" id="val-bat">0.0<span class="unit">V</span></div></div>
      <div class="data-card"><div class="label">INJECTOR</div><div class="value" id="val-inj">0.00<span class="unit">ms</span></div></div>
      <div class="data-card"><div class="label">IGNITION</div><div class="value" id="val-igt">0.0<span class="unit">°</span></div></div>
      <div class="data-card"><div class="label">SPEED</div><div class="value" id="val-speed">0<span class="unit">km/h</span></div></div>
      <div class="data-card"><div class="label"></div><div class="value" id=""><span class="unit"></span></div></div>
      <div class="data-card"><div class="label"></div><div class="value" id=""><span class="unit"></span></div></div>
      <div class="data-card"><div class="label"></div><div class="value" id=""><span class="unit"></span></div></div>
      <div class="data-card"><div class="label"></div><div class="value" id=""><span class="unit"></span></div></div>
      <div class="data-card"><div class="label"></div><div class="value" id=""><span class="unit"></span></div></div>
    </div>
  </div>

  <div class="data-container">
    <h2>History Data</h2>
    <div class="table-responsive-wrapper">
      <table id="dataTable">
        <thead>
          <tr>
            <th>Timestamp</th>
            <th>RPM</th>
            <th>TPS (mV)</th>
            <th>TPS (%)</th>
            <th>ECT (mV)</th>
            <th>ECT (°C)</th>
            <th>IAT (mV)</th>
            <th>IAT (°C)</th>
            <th>MAP (mV)</th>
            <th>MAP (kPa)</th>
            <th>BAT (V)</th>
            <th>INJ (mS)</th>
            <th>IGT (°)</th>
            <th>SPEED (km/h)</th>
          </tr>
        </thead>
        <tbody id="tableBody"></tbody>
      </table>
    </div>
  </div>
  <script>
    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          if (data && data.length > 0) {
            const latest = data[0];
            
            document.getElementById('val-rpm').textContent = latest.obd_rpm;
            document.getElementById('val-speed').innerHTML = latest.obd_speed_kmh + '<span class="unit">km/h</span>';
            document.getElementById('val-ect').innerHTML = latest.obd_ect2_c + '<span class="unit">°C</span>';
            document.getElementById('val-bat').innerHTML = latest.obd_bat_v.toFixed(1) + '<span class="unit">V</span>';
            document.getElementById('val-tps').innerHTML = latest.obd_tps2_pct.toFixed(2) + '<span class="unit">%</span>';
            document.getElementById('val-map').innerHTML = latest.obd_map2_kpa + '<span class="unit">kPa</span>';
            document.getElementById('val-iat').innerHTML = latest.obd_iat2_c + '<span class="unit">°C</span>';
            document.getElementById('val-inj').innerHTML = latest.obd_inj_ms.toFixed(2) + '<span class="unit">ms</span>';
            document.getElementById('val-igt').innerHTML = latest.obd_igt_deg.toFixed(1) + '<span class="unit">°</span>';
            document.getElementById('val-tps-mv').innerHTML = latest.obd_tps1_mv.toFixed(1) + '<span class="unit">mV</span>';
            document.getElementById('val-ect-mv').innerHTML = latest.obd_ect1_mv.toFixed(1) + '<span class="unit">mV</span>';
            document.getElementById('val-iat-mv').innerHTML = latest.obd_iat1_mv.toFixed(1) + '<span class="unit">mV</span>';
            document.getElementById('val-map-mv').innerHTML = latest.obd_map1_mv.toFixed(1) + '<span class="unit">mV</span>';

            const tableBody = document.getElementById('tableBody');
            tableBody.innerHTML = '';
            data.forEach(row => {
              const tr = document.createElement('tr');
              tr.innerHTML = `
                <td>${row.timestamp}</td>
                <td>${row.obd_rpm}</td>
                <td>${row.obd_tps1_mv.toFixed(1)}</td>
                <td>${row.obd_tps2_pct.toFixed(2)}</td>
                <td>${row.obd_ect1_mv.toFixed(1)}</td>
                <td>${row.obd_ect2_c}</td>
                <td>${row.obd_iat1_mv.toFixed(1)}</td>
                <td>${row.obd_iat2_c}</td>
                <td>${row.obd_map1_mv.toFixed(1)}</td>
                <td>${row.obd_map2_kpa}</td>
                <td>${row.obd_bat_v.toFixed(1)}</td>
                <td>${row.obd_inj_ms.toFixed(2)}</td>
                <td>${row.obd_igt_deg.toFixed(1)}</td>
                <td>${row.obd_speed_kmh}</td>
              `;
              tableBody.appendChild(tr);
            });
          }
        })
        .catch(error => console.error('Error fetching data:', error));
    }
    document.addEventListener('DOMContentLoaded', updateData);
    setInterval(updateData, 300);
  </script>
</body>
</html>
)=====";

void setup() {
  DEBUG_SERIAL.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  
  DEBUG_SERIAL.print("[SETUP] IP address: ");
  DEBUG_SERIAL.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  
  server.begin();

  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH); delay(100);
  digitalWrite(17, LOW);  delay(70);
  digitalWrite(17, HIGH); delay(120);
  digitalWrite(17, HIGH); delay(100);
  digitalWrite(17, LOW);  delay(70);
  digitalWrite(17, HIGH); delay(120);
  obdHonda.begin(10400, SERIAL_8N1, 16, 17);
  obdHonda.write(WAKEUP, sizeof(WAKEUP));
  delay(100);
}

void loop() {
  server.handleClient();
  unsigned long currentMillis = millis();
  if (currentMillis - lastDataUpdateTime >= dataUpdateInterval) {
    lastDataUpdateTime = currentMillis;
    processAndStoreOBDData();
  }
}

void handleRoot() {
  server.send_P(200, "text/html", HTML_PAGE);
}

void handleData() {
  DynamicJsonDocument doc(8192); 
  JsonArray array = doc.to<JsonArray>();
  
  int points_to_send = 0;
  if (dataAvailable) {
    bool is_full_circular_buffer = (dataHistory[currentDataIndex].timestamp != "");
     if (is_full_circular_buffer) {
      points_to_send = MAX_DATA_POINTS;
    } else {
      points_to_send = currentDataIndex;
    }
  }

  for (int i = 0; i < points_to_send; i++) {
    int index = (currentDataIndex - 1 - i + MAX_DATA_POINTS) % MAX_DATA_POINTS;
    JsonObject dp = array.createNestedObject();
    dp["timestamp"] = dataHistory[index].timestamp;
    dp["obd_rpm"] = dataHistory[index].obd_rpm;
    dp["obd_tps1_mv"] = float(int(dataHistory[index].obd_tps1_mv * 10)) / 10.0; 
    dp["obd_tps2_pct"] = float(int(dataHistory[index].obd_tps2_pct * 100)) / 100.0; 
    dp["obd_ect1_mv"] = float(int(dataHistory[index].obd_ect1_mv * 10)) / 10.0;
    dp["obd_ect2_c"] = dataHistory[index].obd_ect2_c;
    dp["obd_iat1_mv"] = float(int(dataHistory[index].obd_iat1_mv * 10)) / 10.0;
    dp["obd_iat2_c"] = dataHistory[index].obd_iat2_c;
    dp["obd_map1_mv"] = float(int(dataHistory[index].obd_map1_mv * 10)) / 10.0;
    dp["obd_map2_kpa"] = dataHistory[index].obd_map2_kpa;
    dp["obd_bat_v"] = float(int(dataHistory[index].obd_bat_v * 10)) / 10.0;
    dp["obd_inj_ms"] = float(int(dataHistory[index].obd_inj_ms * 100)) / 100.0;
    dp["obd_igt_deg"] = float(int(dataHistory[index].obd_igt_deg * 10)) / 10.0;
    dp["obd_speed_kmh"] = dataHistory[index].obd_speed_kmh;
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void scanTool() {
  obdHonda.flush();
  obdHonda.write(TABLE, sizeof(TABLE));
  delay(100); 
  
  buffCount = 0;
  while ((obdHonda.available() > 0) && (buffCount < 29)) {
    buff[buffCount++] = obdHonda.read();
  }
    OBD_RPM = (buff[9] * 256) + buff[10];
    OBD_TPS1_MV = buff[11] * 19.6;
    OBD_TPS2_PCT = buff[12] * 1.57;
    OBD_ECT1_MV = buff[13] * 19.6;
    OBD_ECT2_C = buff[14] - 40;

    if (buff[15] == 255) { OBD_IAT1_MV = 0; OBD_IAT2_C = -40; }
    else { OBD_IAT1_MV = buff[15] * 19.6; OBD_IAT2_C = buff[16] - 40; }

    if (buff[17] == 255) { OBD_MAP1_MV = 0; OBD_MAP2_KPA = 0; }
    else { OBD_MAP1_MV = buff[17] * 19.6; OBD_MAP2_KPA = buff[18]; }

    OBD_BAT_V = buff[19] / 10.0;
    OBD_INJ_MS = ((buff[20] * 256) + buff[21]) * 3.92; 
    OBD_IGT_DEG = -64 + buff[22] * 0.5;
    OBD_SPEED_KMH = buff[24];
}

void processAndStoreOBDData() {
  scanTool();
  SensorData newData;
  newData.obd_rpm = OBD_RPM;
  newData.obd_tps1_mv = OBD_TPS1_MV;
  newData.obd_tps2_pct = OBD_TPS2_PCT;
  newData.obd_ect1_mv = OBD_ECT1_MV;
  newData.obd_ect2_c = OBD_ECT2_C;
  newData.obd_iat1_mv = OBD_IAT1_MV;
  newData.obd_iat2_c = OBD_IAT2_C;
  newData.obd_map1_mv = OBD_MAP1_MV;
  newData.obd_map2_kpa = OBD_MAP2_KPA;
  newData.obd_bat_v = OBD_BAT_V;
  newData.obd_inj_ms = OBD_INJ_MS;
  newData.obd_igt_deg = OBD_IGT_DEG;
  newData.obd_speed_kmh = OBD_SPEED_KMH;

  unsigned long currentMillis = millis();
  unsigned long seconds = currentMillis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  newData.timestamp = String(hours % 24) + ":" + String(minutes % 60) + ":" + String(seconds % 60);

  dataHistory[currentDataIndex] = newData;
  currentDataIndex = (currentDataIndex + 1) % MAX_DATA_POINTS;
  if (!dataAvailable && currentDataIndex > 0) {
    dataAvailable = true;
  }
}
