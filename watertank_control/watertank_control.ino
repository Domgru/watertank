#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <time.h>

#include "credentials.h"

const char* ssid = SECRET_SSID;
const char* password = SECRET_PASSWORD;

const int RELAY_PIN = 15;
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;
const int FLASH_LED_PIN = 4;

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define RELAY_AN HIGH
#define RELAY_AUS LOW

WebServer server(80);
Preferences preferences;

// sensor height at: 92 cm, real max fill level = 76 cm
// limits for normal auto mode
float limitEmpty = 76.0;  // level at which the tank is considered to be empty measured from the height of the sensor, e.g. if the tank is considered to empty at 16 cm then we caculate this value as 92-16.
float limitFull = 30.0;   // level at which the tank is considered to be full. The sensor has a blind zone of 20-25cm. So we take 30 to be safe.
int fillCounter = 0;

const float limitEmpty = 76.0;  // Pump starts at 76 cm distance
const float limitFull = 30.0;   // Post-run triggers at 30 cm distance

int fillCounter = 0;

// time management and "security" max-time
unsigned long pumpStartTime = 0;
const unsigned long maxRunTime = 600000;  // 10 minutes in milli-sec

bool isPostRunning = false;
unsigned long postRunStartTime = 0;
/* 
 * In the automatich mode already 62 cm (92-30) have been filled and 14 cm are missing to max level 76 cm. 
 * The pump fills constantly 46,6 cm in 301 seconds which was measured manually. Final tests showed there is a minor overflow with 90 seconds post run duration so we take 80.
*/
const unsigned long postRunDuration = 80000;

unsigned long lastMeasurement = 0;
unsigned long measureInterval = 60000;  // 60 seconds
float smoothedDistance = 0.0;
bool relayState = false;

// Wifi reconnect timer
unsigned long lastWifiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 30000;  // check every 30 seconds

// ring buffer for smoothened sensor value
const int FILTER_SAMPLES = 5;
float measurements[FILTER_SAMPLES] = { 0 };
int filterIndex = 0;

bool testModeActive = false;
float simulatedDistance = 80.0;

struct LogEntry {
  String timestamp;
  float levelPercent;
  float distanceCm;
};
const int HIST_SIZE = 5;
LogEntry history[HIST_SIZE];
int historyIndex = 0;
int totalHistoryCount = 0;

String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[NTP] Locale time not available yet!");
    return "Syncing...";
  }
  char timeStringBuff[30];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%d.%m.%Y %H:%M:%S", &timeinfo);
  return String(timeStringBuff);
}


void addHistoryEntry(float distance, float percent) {
  history[historyIndex].timestamp = getFormattedTime();
  history[historyIndex].distanceCm = distance;
  history[historyIndex].levelPercent = percent;

  historyIndex = (historyIndex + 1) % HIST_SIZE;
  if (totalHistoryCount < HIST_SIZE) totalHistoryCount++;
}

float getFilteredDistance() {
  if (testModeActive) {
    if (relayState) {
      simulatedDistance -= 2.0;
      if (simulatedDistance < 10.0) simulatedDistance = 10.0;
    } else {
      simulatedDistance += 0.5;
      if (simulatedDistance > 180.0) simulatedDistance = 180.0;
    }
    return simulatedDistance;
  }

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return smoothedDistance;

  float currentMeasure = (duration * 0.0343) / 2.0;

  if (currentMeasure < 25.0 || currentMeasure > 400.0) {
    return smoothedDistance;
  }

  measurements[filterIndex] = currentMeasure;
  filterIndex = (filterIndex + 1) % FILTER_SAMPLES;

  float sum = 0;
  int validCount = 0;
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    if (measurements[i] > 0) {
      sum += measurements[i];
      validCount++;
    }
  }

  if (validCount > 0) {
    smoothedDistance = sum / validCount;
  }
  return smoothedDistance;
}

// HTML
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta charset="utf-8">
    <title>Water Tank Control</title>
    <style>
        body { font-family: Arial, sans-serif; text-align: center; background-color: #f4f4f4; margin: 0; padding: 20px; color: #333; }
        .container { max-width: 500px; background: white; margin: 0 auto; padding: 20px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
        .btn { display: inline-block; padding: 12px 20px; font-size: 16px; color: white; border: none; border-radius: 5px; cursor: pointer; text-decoration: none; margin: 5px; }
        .btn-capture { background-color: #3498db; width: 90%; }
        .status-box { font-size: 18px; margin: 15px 0; padding: 15px; border-radius: 5px; background: #eee; text-align: left; }
        .tank-wrapper { background: #ddd; border-radius: 8px; height: 30px; width: 100%; margin: 15px 0; position: relative; overflow: hidden; border: 1px solid #bbb; }
        .tank-fill { background: #2980b9; height: 100%; width: 0%; transition: width 0.4s ease-out; }
        .tank-text { position: absolute; width: 100%; top: 4px; left: 0; font-weight: bold; text-align: center; color: #000; text-shadow: 0 0 3px #fff; }
        .cfg-group { margin: 15px 0; text-align: left; background: #fdfdfd; padding: 10px; border: 1px solid #ddd; border-radius: 5px; }
        .cfg-group label { display: inline-block; width: 190px; font-weight: bold; }
        .cfg-group input { width: 80px; padding: 5px; font-size: 14px; text-align: center; }
        .btn-save { background-color: #27ae60; padding: 6px 12px; font-size: 14px; float: right; margin-top: -3px; }
        img { max-width: 100%; height: auto; border-radius: 5px; margin-top: 15px; border: 2px solid #ddd; }
        .highlight { font-weight: bold; color: #2980b9; float: right; }
        .test-badge { background: #e67e22; color: white; padding: 5px; border-radius: 3px; font-size: 12px; font-weight: bold; display: none; margin-bottom: 10px; }
        .history-box { margin: 20px 0; padding: 10px; background: #fff; border: 1px solid #ddd; border-radius: 5px; text-align: left; }
        .history-table { width: 100%; border-collapse: collapse; margin-top: 10px; font-size: 14px; }
        .history-table th, .history-table td { border-bottom: 1px solid #ddd; padding: 8px; text-align: left; }
        .history-table th { background-color: #f2f2f2; font-weight: bold; }
        .history-table tr:nth-child(even) { background-color: #fdfdfd; }
    </style>
</head>
<body>
    <div class="container">
        <h2>Water Tank Control</h2>
        <div id="test_indicator" class="test-badge">SIMULATION MODE ACTIVE</div>
        
        <div class="tank-wrapper">
            <div id="tank_bar" class="tank-fill"></div>
            <div id="tank_percent" class="tank-text">Starting measurement...</div>
        </div>

        <div class="status-box">
            Sensor Distance: <span id="dist" class="highlight">-- cm</span><br>
            Pump Status: <span id="status" class="highlight">--</span><br>
            Fill Cycles (Since Boot): <span id="counter" class="highlight">--</span>
        </div>

        <div class="history-box">
            <h3 style="margin-top:0;">Last 5 Measurements</h3>
            <table class="history-table">
                <thead>
                    <tr>
                        <th>Timestamp</th>
                        <th>Distance</th>
                        <th>Water Level</th>
                    </tr>
                </thead>
                <tbody id="history_body">
                    <tr><td colspan="3" style="text-align:center; color:#888;">Waiting for data...</td></tr>
                </tbody>
            </table>
        </div>

        <div class="cfg-group">
            <h3>Active Configuration (Locked)</h3>
            <label>Pump ON (Empty Limit):</label>
            <input type="number" id="inp_empty" readonly style="background:#eee; color:#666; cursor:not-allowed;"> cm<br><br>
            <label>Pump OFF (Full Limit):</label>
            <input type="number" id="inp_full" readonly style="background:#eee; color:#666; cursor:not-allowed;"> cm<br><br>
            <label>Pump Post-Run Time:</label>
            <input type="number" id="inp_postrun" readonly style="background:#eee; color:#666; cursor:not-allowed;"> sec.<br><br>
            <label>Max Run Time (Timeout):</label>
            <input type="number" id="inp_maxrun" readonly style="background:#eee; color:#666; cursor:not-allowed;"> min.
            <!-- Save Button removed since settings are permanent -->
            <div style="clear:both;"></div>
        </div>
        
        <button class="btn btn-capture" onclick="capturePhoto()">Take Live Photo</button>
        <br>
        <img id="stream" src="" style="display:none;">
    </div>

    <script>
        var initialFieldsLoaded = false; 

        function updateData() {
            fetch('/api/data')
            .then(res => res.json())
            .then(data => {
                document.getElementById('dist').innerText = data.isPostRunning ? "Blind Zone (Post-Run)" : data.distance.toFixed(1) + " cm";
                document.getElementById('counter').innerText = data.counter;
                document.getElementById('test_indicator').style.display = data.isTest ? "block" : "none";
                
                if (data.isPostRunning) {
                    document.getElementById('status').innerText = "POST-RUN (Filling...)";
                    document.getElementById('status').style.color = "orange";
                    document.getElementById('tank_bar').style.width = "100%";
                    document.getElementById('tank_percent').innerText = "Level: Filling remaining volume...";
                } else {
                    document.getElementById('status').innerText = data.relay ? "ACTIVE (Filling)" : "STANDBY / OFF";
                    document.getElementById('status').style.color = data.relay ? "green" : "red";
                    
                    var percent = ((data.limitEmpty - data.distance) / (data.limitEmpty - data.limitFull)) * 100;
                    if (percent > 100) percent = 100;
                    if (percent < 0) percent = 0;
                    
                    document.getElementById('tank_bar').style.width = percent.toFixed(0) + "%";
                    document.getElementById('tank_percent').innerText = "Level: " + percent.toFixed(0) + "%";
                }

                if(!initialFieldsLoaded) {
                    document.getElementById('inp_empty').value = data.limitEmpty;
                    document.getElementById('inp_full').value = data.limitFull;
                    document.getElementById('inp_postrun').value = data.postRunSec;
                    document.getElementById('inp_maxrun').value = data.maxRunMin;
                    initialFieldsLoaded = true; 
                }

                if (data.history && data.history.length > 0) {
                    let html = "";
                    data.history.forEach(row => {
                        let displayTime = row.time;
                        if (displayTime === "Syncing time...") {
                            displayTime = "Syncing time...";
                        }
                        html += `<tr>
                            <td>${displayTime}</td>
                            <td>${row.dist.toFixed(1)} cm</td>
                            <td><strong>${row.pct.toFixed(0)}%</strong></td>
                        </tr>`;
                    });
                    document.getElementById('history_body').innerHTML = html;
                }
            });
        }

        function capturePhoto() {
            var img = document.getElementById('stream');
            img.src = '/capture?t=' + new Date().getTime();
            img.style.display = 'block';
        }

        updateData();
        setInterval(updateData, 1500); 
    </script>
</body>
</html>
)=====";

void handleCapture() {
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(350);
  camera_fb_t* dummy_fb = esp_camera_fb_get();
  if (dummy_fb) esp_camera_fb_return(dummy_fb);

  camera_fb_t* fb = esp_camera_fb_get();
  digitalWrite(FLASH_LED_PIN, LOW);

  if (!fb) {
    server.send(500, "text/plain", "Camera Error");
    return;
  }
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void setup() {
  digitalWrite(RELAY_PIN, RELAY_AUS);
  pinMode(RELAY_PIN, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  Serial.begin(115200);

  fillCounter = 0;

  camera_config_t config;
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) Serial.printf("Camera init failed: 0x%x\n", err);

  // ask for a fixed ip - address
  IPAddress local_IP(192, 168, 29, 19);  // desired IP
  IPAddress gateway(192, 168, 29, 1);    // address of Jio router
  IPAddress subnet(255, 255, 255, 0);    // standard sub net mask
  IPAddress dns(192, 168, 29, 1);        // DNS server

  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("[WiFi] Fixed IP-Configuration failed!");
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nWiFi connected! IP-Address: ");
  Serial.println(WiFi.localIP());

  configTime(5.5 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("[NTP] Initiated time sync via pool.ntp.org.");

  if (MDNS.begin("watertank")) Serial.println("Accessible at http://watertank.local");

  server.on("/", [&]() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  // URL to START the simulation mode
  server.on("/test/on", [&]() {
    testModeActive = true;
    simulatedDistance = limitEmpty - 10.0;

    // set back timer so the simulation starts immediately
    lastMeasurement = 0;
    measureInterval = 1000;  // force immediately fast intervals

    server.send(200, "text/plain", "Simulation mode ENABLED. Go back to start page.");
  });

  // URL to END the simulation mode
  server.on("/test/off", [&]() {
    testModeActive = false;
    isPostRunning = false;

    // reset timer to return to real sensor
    lastMeasurement = 0;
    measureInterval = 60000;  // go back to normal standby-mode

    server.send(200, "text/plain", "Simulation mode DISABLED. Go back to start page.");
  });

  server.on("/api/data", [&]() {
    if (!isPostRunning) {
      smoothedDistance = getFilteredDistance();
    }

    String json = "{";
    json += "\"distance\":" + String(smoothedDistance) + ",";
    json += "\"relay\":" + String(relayState ? "true" : "false") + ",";
    json += "\"limitEmpty\":" + String(limitEmpty) + ",";
    json += "\"limitFull\":" + String(limitFull) + ",";
    json += "\"postRunSec\":" + String(postRunDuration / 1000) + ",";
    json += "\"maxRunMin\":" + String(maxRunTime / 60000) + ",";
    json += "\"counter\":" + String(fillCounter) + ",";
    json += "\"isTest\":" + String(testModeActive ? "true" : "false") + ",";
    json += "\"isPostRunning\":" + String(isPostRunning ? "true" : "false") + ",";

    // include the history
    json += "\"history\": [";
    if (totalHistoryCount == 0) {
      json += "";
    } else {
      for (int i = 0; i < totalHistoryCount; i++) {
        int targetIdx = (historyIndex - 1 - i + HIST_SIZE) % HIST_SIZE;

        // security fallback: should the timestamp still be empty, send a dummy text
        String displayTime = history[targetIdx].timestamp;
        if (displayTime == "" || displayTime.length() < 2) {
          displayTime = "Syncing time...";
        }

        json += "{";
        json += "\"time\":\"" + displayTime + "\",";
        json += "\"dist\":" + String(history[targetIdx].distanceCm) + ",";
        json += "\"pct\":" + String(history[targetIdx].levelPercent);
        json += "}";
        if (i < totalHistoryCount - 1) json += ",";
      }
    }
    json += "]";
    json += "}";
    server.send(200, "application/json", json);
  });

  // Flash writing completely disabled
  server.on("/api/set", [&]() {
    server.send(403, "text/plain", "Error: Configuration is hard-coded and locked via firmware.");
  });

  server.on("/capture", handleCapture);
  server.begin();
  MDNS.addService("http", "tcp", 80);

  Serial.println("[SYS] Warming up ultrasonic sensor buffer over cable lines...");
  for (int i = 0; i < 15; i++) {
    getFilteredDistance();
    delay(150);  // Give ambient noise and electrical echoes room to dissipate
  }

  float initialDist = getFilteredDistance();
  for (int i = 0; i < FILTER_SAMPLES; i++) measurements[i] = initialDist;
  smoothedDistance = initialDist;

  lastMeasurement = millis();
  measureInterval = 5000;  // First loop-bound automation execution begins in 5 seconds
  Serial.printf("[SYS] Safe operational baseline established: %.1f cm\n", smoothedDistance);
}

// --- MAIN LOOP ---
void loop() {

  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  unsigned long currentMillis = millis();

  // --- Asynchronous WiFi reconnect check each 30 seconds
  if (currentMillis - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
    lastWifiCheck = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Connection lost! Initiating reconnect...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }
  }

  // 1. check of post-run timer
  if (relayState && isPostRunning) {
    if (currentMillis - postRunStartTime >= postRunDuration) {
      relayState = false;
      isPostRunning = false;
      digitalWrite(RELAY_PIN, RELAY_AUS);
      measureInterval = 60000;
      Serial.println("Post-run complete. Pump OFF.");
    }
  }

  // 2. safety - keep the pump running at maxRunTime
  if (relayState && (currentMillis - pumpStartTime >= maxRunTime)) {
    relayState = false;
    isPostRunning = false;
    digitalWrite(RELAY_PIN, RELAY_AUS);
    measureInterval = 60000;
    Serial.println("SAFETY CUT-OFF: Maximum pump runtime exceeded!");
  }

  // 3. "normal" auto mode
  if (!isPostRunning && (currentMillis - lastMeasurement >= measureInterval)) {
    lastMeasurement = currentMillis;
    smoothedDistance = getFilteredDistance();

    // historization
    float currentPercent = ((limitEmpty - smoothedDistance) / (limitEmpty - limitFull)) * 100.0;
    if (currentPercent > 100.0) currentPercent = 100.0;
    if (currentPercent < 0.0) currentPercent = 0.0;
    addHistoryEntry(smoothedDistance, currentPercent);

    if (smoothedDistance >= limitEmpty && !relayState) {
      relayState = true;
      pumpStartTime = currentMillis;
      digitalWrite(RELAY_PIN, RELAY_AN);
      measureInterval = 1000;

      fillCounter++;
      Serial.println("Tank empty. Pump ON.");
    } else if (smoothedDistance <= limitFull && relayState) {  // pump continues in post-run mode
      isPostRunning = true;
      postRunStartTime = currentMillis;
      // set interval to very high value (10 minutes) to avoid measurement by the sensor in its blindzone during the post-run
      measureInterval = 600000;  // 10 minutes!
      Serial.printf("Sensor \"full\" limit (%.1f cm) reached. Sensor sleeping. Post-run of %lu sec. running...", smoothedDistance, postRunDuration / 1000);
    }
  }
  delay(2);
}
