#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
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

// measured max water level from ground: 78,5 cm

const float SENSOR_HEIGHT_FROM_GROUND = 93.7;
const float PHYSICAL_EMPTY_DISTANCE = 77.7;  // Water level 16 cm is considered empty which is equivalent to a sensor distance of 77,7 cm (93,7 - 16)
const float PHYSICAL_FULL_DISTANCE = 30.0;   // Minimum sensor distance to water (Full Tank = 100%)

// for demo only:
float pumpOnTriggerDistance = 77.7;                           // Modifiable in video to trigger pump!
const float pumpOffTriggerDistance = PHYSICAL_FULL_DISTANCE;  // Locked at physical full
int fillCounter = 0;

// time management and "security" max-time
unsigned long pumpStartTime = 0;
const unsigned long maxRunTime = 600000;  // 10 minutes in milli-sec

bool isPostRunning = false;
unsigned long postRunStartTime = 0;

// In the automatich mode already 63,7 cm (93,7-30) have been filled and 14,8 cm are missing to max level 78,5 cm. However, even if the pump is switching off the water still is flowing.
unsigned long postRunDuration = 60000;

unsigned long lastMeasurement = 0;
unsigned long measureInterval;
float smoothedDistance = 0.0;
bool relayState = false;

unsigned long lastHistoryLog = 0;
float lastLoggedPercent = -99.0;

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
  byte day;
  byte month;
  int year;
  byte hour;
  byte minute;
  byte second;
  float levelPercent;
  float distanceCm;
};
const int HIST_SIZE = 5;
LogEntry history[HIST_SIZE];
int historyIndex = 0;
int totalHistoryCount = 0;

void addHistoryEntry(float distance, float percent) {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    history[historyIndex].day = timeinfo.tm_mday;
    history[historyIndex].month = timeinfo.tm_mon + 1;
    history[historyIndex].year = timeinfo.tm_year + 1900;
    history[historyIndex].hour = timeinfo.tm_hour;
    history[historyIndex].minute = timeinfo.tm_min;
    history[historyIndex].second = timeinfo.tm_sec;
  } else {
    // Fallback falls NTP noch nicht synchronisiert ist
    history[historyIndex].day = 0;
    history[historyIndex].month = 0;
    history[historyIndex].year = 0;
    history[historyIndex].hour = 99;
    history[historyIndex].minute = 99;
    history[historyIndex].second = 99;
  }
  history[historyIndex].distanceCm = distance;
  history[historyIndex].levelPercent = percent;

  historyIndex = (historyIndex + 1) % HIST_SIZE;
  if (totalHistoryCount < HIST_SIZE) totalHistoryCount++;
}

float getFilteredDistance() {
  if (testModeActive) {
    if (relayState) {
      simulatedDistance -= 3.0;
      if (simulatedDistance < PHYSICAL_FULL_DISTANCE) {
        simulatedDistance = PHYSICAL_FULL_DISTANCE;
      }
    } else {
      simulatedDistance += 1.0;
      if (simulatedDistance > PHYSICAL_EMPTY_DISTANCE) {
        simulatedDistance = PHYSICAL_EMPTY_DISTANCE;
      }
    }
    return simulatedDistance;
  }

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1.0;

  float currentMeasure = (duration * 0.0343) / 2.0;

  if (currentMeasure > 400.0) {
    return -1.0;
  }

  if (currentMeasure < PHYSICAL_FULL_DISTANCE) {
    currentMeasure = PHYSICAL_FULL_DISTANCE;
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

float calculateTankPercentage(float currentDistance) {
  if (currentDistance < 0) return 0.0;

  float percent = ((PHYSICAL_EMPTY_DISTANCE - currentDistance) / (PHYSICAL_EMPTY_DISTANCE - PHYSICAL_FULL_DISTANCE)) * 100.0;

  if (percent > 100.0) percent = 100.0;
  if (percent < 0.0) percent = 0.0;
  return percent;
}

float calculateWaterHeight(float currentDistance) {
  if (currentDistance < 0) return -1.0;

  float height = SENSOR_HEIGHT_FROM_GROUND - currentDistance;
  float maxHeight = SENSOR_HEIGHT_FROM_GROUND - PHYSICAL_FULL_DISTANCE;  // limit for maxHeight is 63.7 cm since PHYSICAL_FULL_DISTANCE is 30 cm, hence 93.7 - 30.0 = 63.7 cm
  if (height > maxHeight) return maxHeight;
  if (height < 0.0) return 0.0;
  return height;
}

unsigned long getPostRunRemainingSeconds() {
  if (!relayState || !isPostRunning) {
    return 0;
  }
  unsigned long timeElapsed = millis() - postRunStartTime;
  if (timeElapsed < postRunDuration) {
    // Add 999ms to round up to full seconds before the division
    return ((postRunDuration - timeElapsed) + 999) / 1000;
  }
  return 0;
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
            Actual Water Height: <span id="height" class="highlight">-- cm</span><br>
            <div id="postrun_time_row" style="display: none;">
                Post-Run Remaining: <span id="postrun_time" class="highlight" style="color: orange;">-- sec.</span><br>
            </div>
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
            <h3>Configuration Values</h3>
            <label>Pump ON (Empty Limit):</label>
            <input type="number" id="inp_empty" step="1" style="background:#fff; color:#000; cursor:text;"> cm<br><br>
            <label>Pump Post-Run Time:</label>
            <input type="number" id="inp_postrun" min="1" step="1" style="background:#fff; color:#000; cursor:text;"> sec.<br><br>
            <button class="btn btn-save" onclick="saveLimits()">Apply configuration</button>
            <br><br>
            
            <label>Pump OFF (Full Limit):</label>
            <input type="number" id="inp_full" readonly style="background:#eee; color:#666; cursor:not-allowed;"> cm<br><br>
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
              if (data.distance < 0) {
                  document.getElementById('postrun_time_row').style.display = "none";
                  document.getElementById('height').innerText = "Unknown";
                  document.getElementById('height').style.color = "red";
                  document.getElementById('dist').innerText = "Hardware Connection Error";
                  document.getElementById('dist').style.color = "red";
                  document.getElementById('status').innerText = "SENSOR ERROR";
                  document.getElementById('status').style.color = "red";
                  document.getElementById('tank_bar').style.width = "0%";
                  document.getElementById('tank_percent').innerText = "Level: Unknown (Check Cable)";
              }
              else if (data.isPostRunning) {
                  document.getElementById('height').innerText = "≥ " + data.waterHeight.toFixed(1) + " cm";
                  document.getElementById('height').style.color = "orange";
                  document.getElementById('postrun_time_row').style.display = "block";
                  document.getElementById('postrun_time').innerText = data.postRunRemainingSec + " sec.";
                  document.getElementById('dist').innerText = "Blind Zone (Post-Run)";
                  document.getElementById('dist').style.color = "#2980b9";
                  document.getElementById('status').innerText = "POST-RUN (Filling...)";
                  document.getElementById('status').style.color = "orange";
                  document.getElementById('tank_bar').style.width = "100%";
                  document.getElementById('tank_percent').innerText = "Level: Filling remaining volume...";
              }
              else {
                  if (data.isMaxFilled) {
                      document.getElementById('dist').innerText = "≤ " + data.distance.toFixed(1) + " cm";
                      document.getElementById('height').innerText = "≥ " + data.waterHeight.toFixed(1) + " cm";
                  } else {
                      document.getElementById('dist').innerText = data.distance.toFixed(1) + " cm";
                      document.getElementById('height').innerText = data.waterHeight.toFixed(1) + " cm";
                  }
                  document.getElementById('postrun_time_row').style.display = "none";
                  document.getElementById('dist').style.color = "#2980b9";
                  document.getElementById('height').style.color = "#2980b9";
                  document.getElementById('status').innerText = data.relay ? "ACTIVE (Filling)" : "STANDBY / OFF";
                  document.getElementById('status').style.color = data.relay ? "green" : "red";
                  
                  var percent = data.percent;
                  document.getElementById('tank_bar').style.width = percent.toFixed(0) + "%";
                  document.getElementById('tank_percent').innerText = "Level: " + percent.toFixed(0) + "%";
              }

              document.getElementById('counter').innerText = data.counter;
              document.getElementById('test_indicator').style.display = data.isTest ? "block" : "none";

              if(!initialFieldsLoaded) {
                  document.getElementById('inp_empty').value = data.pumpOnTrigger;
                  document.getElementById('inp_full').value = data.pumpOffTrigger;
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

        function parseAndValidateInput(elementId, fieldName, isInteger = false) {
            var rawValue = document.getElementById(elementId).value.replace(',', '.');
            var parsedValue = Number(rawValue);
            
            if (isNaN(parsedValue) || parsedValue <= 0) {
                alert("Please provide a valid numeric value for field '" + fieldName + "'!");
                return null;
            }
            if (isInteger && parsedValue % 1 !== 0) {
              alert("Field '" + fieldName + "' must be a whole number (no decimals allowed)!");
              return null;
            }
            return parsedValue;
        }

        function saveLimits() {
            var emp = parseAndValidateInput('inp_empty', 'empty limit');
            if (emp === null) return;
            var postrun = parseAndValidateInput('inp_postrun', 'postrun time', true);
            if (postrun === null) return;

            var full = parseFloat(document.getElementById('inp_full').value);

            if (emp < (full + 2.0)) {
                alert("Invalid input! The Pump-ON distance (" + emp + " cm) must be greater than the Pump-OFF distance (" + full + " cm) plus 2 cm safety margin.");
                return;
            }
            
            fetch('/api/set?empty=' + emp + '&postrun=' + postrun)
            .then(res => res.text())
            .then(txt => { 
                if (txt === "Saved all settings!") {
                    alert("Pump-ON trigger distance and PostrunTime updated successfully in RAM!");
                } else {
                    alert(txt);
                }
                initialFieldsLoaded = false; 
                updateData(); 
            });
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
  // 1. Hardware-Pins sofort initialisieren
  digitalWrite(RELAY_PIN, RELAY_AUS);
  pinMode(RELAY_PIN, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  Serial.begin(115200);
  delay(100); // Kurze Pause zur Stabilisierung der Spannungsversorgung
  Serial.println("\n[SYS] Booting secure firmware...");
  fillCounter = 0;

  // 2. WLAN SOFORT STARTEN (Verhindert Boot-Loops, falls der Router nach Stromausfall bootet)
  IPAddress local_IP(192, 168, 29, 199);  // Neue, krisensichere IP
  IPAddress gateway(192, 168, 29, 1);    // Jio Router
  IPAddress subnet(255, 255, 255, 0);    
  IPAddress dns(192, 168, 29, 1);        

  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("[WiFi] Fixed IP-Configuration failed!");
  }

  WiFi.begin(ssid, password);
  Serial.println("[WiFi] Connecting to Network...");
  
  int attempts = 0;
  // Maximal 15 Sekunden (30 * 500ms) beim Start versuchen, um Hänger zu vermeiden
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi connected! IP-Address: ");
    Serial.println(WiFi.localIP());
    configTime(5.5 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    Serial.println("[NTP] Initiated time sync.");
    if (MDNS.begin("watertank")) {
      Serial.println("Accessible at http://watertank.local");
      MDNS.addService("http", "tcp", 80);
    }
  } else {
    Serial.println("\n[WiFi] Router not ready yet. Continuing boot, will auto-connect in background.");
  }

  // 3. WEBSERVER-ROUTEN REGISTRIEREN
  server.on("/", [&]() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/test/on", [&]() {
    testModeActive = true;
    simulatedDistance = pumpOnTriggerDistance + 5.0;
    lastMeasurement = 0;
    measureInterval = 1000;
    server.send(200, "text/plain", "Simulation mode ENABLED. Go back to start page.");
  });

  server.on("/test/off", [&]() {
    testModeActive = false;
    isPostRunning = false;
    lastMeasurement = 0;
    measureInterval = 60000;
    server.send(200, "text/plain", "Simulation mode DISABLED. Go back to start page.");
  });

  server.on("/api/data", [&]() {
    String json = "{";
    json += "\"distance\":" + String(smoothedDistance) + ",";
    json += "\"percent\":" + String(calculateTankPercentage(smoothedDistance)) + ",";
    json += "\"waterHeight\":" + String(calculateWaterHeight(smoothedDistance)) + ",";
    json += "\"relay\":" + String(relayState ? "true" : "false") + ",";
    json += "\"pumpOnTrigger\":" + String(pumpOnTriggerDistance) + ",";
    json += "\"pumpOffTrigger\":" + String(pumpOffTriggerDistance) + ",";
    json += "\"postRunSec\":" + String(postRunDuration / 1000) + ",";
    json += "\"postRunRemainingSec\":" + String(getPostRunRemainingSeconds()) + ",";
    json += "\"maxRunMin\":" + String(maxRunTime / 60000) + ",";
    json += "\"counter\":" + String(fillCounter) + ",";
    json += "\"isTest\":" + String(testModeActive ? "true" : "false") + ",";
    json += "\"isPostRunning\":" + String(isPostRunning ? "true" : "false") + ",";
    json += "\"isMaxFilled\":" + String(smoothedDistance <= (PHYSICAL_FULL_DISTANCE + 0.1) ? "true" : "false") + ",";

    json += "\"history\": [";
    if (totalHistoryCount == 0) {
      json += "";
    } else {
      for (int i = 0; i < totalHistoryCount; i++) {
        int targetIdx = (historyIndex - 1 - i + HIST_SIZE) % HIST_SIZE;
        String displayTime;
        if (history[targetIdx].hour == 99) {
          displayTime = "Syncing time...";
        } else {
          char buff[25];
          sprintf(buff, "%02d.%02d.%04d %02d:%02d:%02d",
                  history[targetIdx].day, history[targetIdx].month, history[targetIdx].year,
                  history[targetIdx].hour, history[targetIdx].minute, history[targetIdx].second);
          displayTime = String(buff);
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

  server.on("/api/set", [&]() {
    if (server.hasArg("empty") && server.hasArg("postrun")) {
      float emptyChangedValue = server.arg("empty").toFloat();
      if (emptyChangedValue < (PHYSICAL_FULL_DISTANCE + 2.0)) {
        server.send(400, "text/plain", "Error: Pump-ON distance must be at least " + String(PHYSICAL_FULL_DISTANCE + 2.0) + " cm!");
        return;
      }
      String postrunRaw = server.arg("postrun");
      unsigned long postrunChangedValue = strtoul(postrunRaw.c_str(), NULL, 10);
      if (postrunChangedValue == 0) {
        server.send(400, "text/plain", "Error: Postrun must be greater than 0!");
        return;
      }
      pumpOnTriggerDistance = emptyChangedValue;
      lastMeasurement = 0;  
      postRunDuration = postrunChangedValue * 1000;
      server.send(200, "text/plain", "Saved all settings!");
    } else {
      server.send(400, "text/plain", "Error. Missing empty parameter.");
    }
  });

  server.on("/capture", handleCapture);
  server.begin();

  // 4. KAMERA-INITIALISIERUNG (Mit Schutz vor komplettem Systemabsturz)
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
  if (err != ESP_OK) {
    Serial.printf("[CAMERA] Init failed: 0x%x. System keeps running without video!\n", err);
  }

    Serial.println("[SYS] Pre-filling ultrasonic sensor buffer with empty baseline...");
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    measurements[i] = PHYSICAL_EMPTY_DISTANCE; // Puffer sicher mit "Leer-Abstand" vorfüllen
  }

  Serial.println("[SYS] Warming up ultrasonic sensor over cable lines...");
  for (int i = 0; i < 15; i++) {
    getFilteredDistance();
    delay(150); 
  }

  float initialDist = getFilteredDistance();
  // Wenn der frisch gereinigte Tank Müllwerte liefert oder out-of-range ist, auf leer setzen
  if (initialDist <= 0 || initialDist > PHYSICAL_EMPTY_DISTANCE) {
    initialDist = PHYSICAL_EMPTY_DISTANCE;
    Serial.println("[SYS] Sensor values unstable or tank empty. Set to PHYSICAL_EMPTY_DISTANCE.");
  }

  for (int i = 0; i < FILTER_SAMPLES; i++) measurements[i] = initialDist;
  smoothedDistance = initialDist;

  lastMeasurement = millis();
  measureInterval = 2000;  // Erste Automatik-Messung startet nach 2 Sekunden
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
    float currentPercent = calculateTankPercentage(smoothedDistance);

    if (abs(currentPercent - lastLoggedPercent) >= 2.0 || (currentMillis - lastHistoryLog >= 300000) || lastLoggedPercent < 0) {
      addHistoryEntry(smoothedDistance, currentPercent);
      lastLoggedPercent = currentPercent;
      lastHistoryLog = currentMillis;
    }

    if (smoothedDistance >= pumpOnTriggerDistance && !relayState) {
      relayState = true;
      pumpStartTime = currentMillis;
      digitalWrite(RELAY_PIN, RELAY_AN);
      measureInterval = 1000;

      fillCounter++;
      Serial.println("Tank empty. Pump ON.");
      addHistoryEntry(smoothedDistance, currentPercent);
    } else if (smoothedDistance <= pumpOffTriggerDistance && relayState) {  // pump continues in post-run mode
      isPostRunning = true;
      postRunStartTime = currentMillis;
      // set interval to very high value (10 minutes) to avoid measurement by the sensor in its blindzone during the post-run
      measureInterval = 600000;  // 10 minutes!
      Serial.printf("Sensor \"full\" limit (%.1f cm) reached. Sensor sleeping. Post-run of %lu sec. running...", smoothedDistance, postRunDuration / 1000);
      addHistoryEntry(smoothedDistance, currentPercent);
    }
  }
  delay(2);
}
