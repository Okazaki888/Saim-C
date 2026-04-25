/*
  ============================================================
  กระบอกเซียมซี MPU6050 — ESP32 Edition  v2.0
  ============================================================
  แก้ไขจากเวอร์ชันก่อน:
  - ใช้ millis() แทน delay() ใน loop → web ไม่ค้าง
  - เพิ่ม NTP sync → timestamp จริง
  - แก้ RMS calculation ที่ผิด → เก็บ sumSq แยกต่างหาก
  - แก้ gyroBuffer ไม่ reset → ไม่นับค่าเก่า
  - เพิ่ม JSON buffer ให้ใหญ่พอ (DynamicJsonDocument 2048)
  - ฟอนต์ system stack ใน CSS แทน Google Fonts → ใช้ได้โดยไม่มีอินเทอร์เน็ต
  - เพิ่ม CORS + Cache-Control header
  - esp_random() เป็น entropy เพิ่มเติมสำหรับเลข
  - Wire.setClock(400kHz) fast mode
  - AP fallback password ยาวขึ้น (8+ ตัว)

  การเชื่อมต่อ:
    MPU6050 VCC  ->  ESP32 3.3V
    MPU6050 GND  ->  ESP32 GND
    MPU6050 SDA  ->  ESP32 GPIO 21
    MPU6050 SCL  ->  ESP32 GPIO 22

  Library (ติดตั้งผ่าน Arduino Library Manager):
    1. "MPU6050_light"  by rfetick
    2. "ArduinoJson"    by Benoit Blanchon
    WebServer + WiFi มาพร้อม ESP32 board package แล้ว

  Board: "ESP32 Dev Module"  (Espressif Systems)
  ============================================================
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <MPU6050_light.h>
#include <time.h>

// ============================================================
//  ตั้งค่า — แก้ตรงนี้ก่อนอัปโหลด
// ============================================================
const char* WIFI_SSID        = "Okazaki";
const char* WIFI_PASS        = "00000000";

// NTP
const char* NTP_SERVER       = "pool.ntp.org";
const long  GMT_OFFSET_SEC   = 7 * 3600;  // UTC+7 (ไทย)
const int   DAYLIGHT_OFFSET  = 0;

// ค่า sensor
const float SHAKE_THRESHOLD  = 80.0f;   // deg/s
const int   SHAKE_DURATION_S = 5;     // วินาที (5 นาที)
const int   COOLDOWN_S       = 3;

// Shake detection
const int   SHAKE_BUF_SIZE   = 20;
const int   SHAKE_BUF_MIN    = 10;  // sample ขั้นต่ำก่อนตัดสิน
const int   SHAKE_POSITIVE   = 5;   // ต้องมี shaking >= ค่านี้ใน buffer

// History RAM
const int   MAX_HISTORY      = 10;
// ============================================================

MPU6050   mpu(Wire);
WebServer server(80);

// ===== State =====
enum State { WAIT_SHAKE, RECORDING, COMPUTING, COOLDOWN };
State currentState = WAIT_SHAKE;

// ===== Shake circular buffer =====
bool shakeBuf[SHAKE_BUF_SIZE];
int  shakeBufHead   = 0;
int  shakeBufFilled = 0;

// ===== Recording =====
float recSumX = 0, recSumY = 0;
float recSumSqX = 0, recSumSqY = 0;   // สำหรับ RMS
int   recCount = 0, recShakeN = 0;
unsigned long recStartMs  = 0;
unsigned long cooldownMs  = 0;

// ===== Live values =====
float curGX = 0, curGY = 0;
bool  curShaking   = false;
float recProgress  = 0.0f;

// ===== History =====
struct LotteryEntry {
  char  number[8];
  char  datetime[24];
  float avgX, avgY, rmsX, rmsY;
  int   shakeCount;
};
LotteryEntry histBuf[MAX_HISTORY];
int histHead = 0, histCount = 0;

// ===== Last result =====
char   lastNumber[8]  = "--";
char   lastTime[24]   = "--";
String sysStatus      = "รอการเขย่า";
bool   ntpSynced      = false;

// ============================================================
//  Helpers
// ============================================================
void getTimeStr(char* buf, size_t len) {
  if (ntpSynced) {
    struct tm ti;
    if (getLocalTime(&ti, 500)) {
      strftime(buf, len, "%d/%m/%Y %H:%M:%S", &ti);
      return;
    }
  }
  unsigned long s = millis() / 1000;
  snprintf(buf, len, "uptime %02lu:%02lu:%02lu",
           s / 3600, (s % 3600) / 60, s % 60);
}

uint64_t fnv1a64(const uint8_t* d, size_t n) {
  uint64_t h = 14695981039346656037ULL;
  for (size_t i = 0; i < n; i++) {
    h ^= d[i];
    h *= 1099511628211ULL;
  }
  return h;
}

String computeLottery(float avgX, float avgY,
                       float rmsX, float rmsY,
                       float ssX,  float ssY, int cnt) {
  uint64_t us  = esp_timer_get_time();
  uint32_t rng = esp_random();

  char seed[200];
  int n = snprintf(seed, sizeof(seed),
    "%.6f|%.6f|%.6f|%.6f|%.6f|%.6f|%d|%llu|%lu|%u",
    avgX, avgY, rmsX, rmsY, ssX, ssY,
    cnt, (unsigned long long)us, millis(), rng);

  uint64_t h1 = fnv1a64((uint8_t*)seed, n);
  // reverse pass
  for (int i = 0; i < n / 2; i++) {
    char tmp = seed[i]; seed[i] = seed[n-1-i]; seed[n-1-i] = tmp;
  }
  uint64_t h2      = fnv1a64((uint8_t*)seed, n);
  uint64_t combined = h1 ^ (h2 << 19) ^ (h2 >> 45);
  uint32_t num     = (uint32_t)(combined % 1000000ULL);

  char out[8];
  snprintf(out, sizeof(out), "%06u", num);
  return String(out);
}

void pushHistory(const char* num, const char* dt,
                 float ax, float ay, float rx, float ry, int sc) {
  LotteryEntry& e = histBuf[histHead];
  strlcpy(e.number,   num, sizeof(e.number));
  strlcpy(e.datetime, dt,  sizeof(e.datetime));
  e.avgX = ax; e.avgY = ay;
  e.rmsX = rx; e.rmsY = ry;
  e.shakeCount = sc;
  histHead = (histHead + 1) % MAX_HISTORY;
  if (histCount < MAX_HISTORY) histCount++;
}

// ===== Shake buffer =====
void shakeBufReset() {
  shakeBufHead = shakeBufFilled = 0;
  memset(shakeBuf, 0, sizeof(shakeBuf));
}

void shakeBufPush(bool v) {
  shakeBuf[shakeBufHead] = v;
  shakeBufHead = (shakeBufHead + 1) % SHAKE_BUF_SIZE;
  if (shakeBufFilled < SHAKE_BUF_SIZE) shakeBufFilled++;
}

bool isShaking() {
  if (shakeBufFilled < SHAKE_BUF_MIN) return false;
  int pos = 0;
  for (int i = 0; i < shakeBufFilled; i++) if (shakeBuf[i]) pos++;
  return pos >= SHAKE_POSITIVE;
}

// ===== State transitions =====
void enterWait() {
  currentState = WAIT_SHAKE;
  sysStatus    = "รอการเขย่า";
  shakeBufReset();
  recProgress  = 0.0f;
  Serial.println("[WAIT]");
}

void enterRecording() {
  currentState = RECORDING;
  sysStatus    = "กำลังบันทึก...";
  recSumX = recSumY = recSumSqX = recSumSqY = 0;
  recCount = recShakeN = 0;
  recStartMs   = millis();
  recProgress  = 0.0f;
  Serial.println("[REC] เริ่มบันทึก 5 นาที");
}

void enterComputing() {
  currentState = COMPUTING;
  sysStatus    = "กำลังคำนวณ...";

  float avgX = recCount > 0 ? recSumX   / recCount : 0.0f;
  float avgY = recCount > 0 ? recSumY   / recCount : 0.0f;
  float rmsX = recCount > 0 ? sqrtf(recSumSqX / recCount) : 0.0f;
  float rmsY = recCount > 0 ? sqrtf(recSumSqY / recCount) : 0.0f;

  String lottery = computeLottery(avgX, avgY, rmsX, rmsY,
                                   recSumSqX, recSumSqY, recCount);
  strlcpy(lastNumber, lottery.c_str(), sizeof(lastNumber));
  getTimeStr(lastTime, sizeof(lastTime));

  pushHistory(lastNumber, lastTime, avgX, avgY, rmsX, rmsY, recShakeN);

  Serial.printf("[LOTTERY] %s  avg(%.2f,%.2f) rms(%.2f,%.2f) n=%d shake=%d\n",
                lastNumber, avgX, avgY, rmsX, rmsY, recCount, recShakeN);

  sysStatus    = String("ผลเลข: ") + lastNumber;
  currentState = COOLDOWN;
  cooldownMs   = millis();
}

// ============================================================
//  HTML (PROGMEM — ไม่ต้องอินเทอร์เน็ต)
// ============================================================
const char HTML_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="th">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>&#x0E01;&#x0E23;&#x0E30;&#x0E1A;&#x0E2D;&#x0E01;&#x0E40;&#x0E0B;&#x0E35;&#x0E22;&#x0E21;&#x0E0B;&#x0E35;</title>
<style>
:root{--g:#C9A84C;--gl:#F0D080;--r:#8B1A1A;--dk:#0E0A04;--s1:#1A1208;--s2:#241A0A;--tx:#F5EDD0;--mu:#9A8A6A}
*{margin:0;padding:0;box-sizing:border-box}
body{background:var(--dk);color:var(--tx);font-family:'Noto Sans Thai',Sarabun,sans-serif;min-height:100vh;
  background-image:radial-gradient(ellipse 60% 40% at 20% 20%,rgba(201,168,76,.07),transparent 70%),
  radial-gradient(ellipse 60% 40% at 80% 80%,rgba(139,26,26,.09),transparent 70%)}
.c{max-width:460px;margin:0 auto;padding:1.5rem 1rem 3rem}
.hd{text-align:center;padding:2rem 0 1.5rem}
.hd-i{font-size:2.8rem;display:block;margin-bottom:.4rem}
.hd h1{font-size:1.55rem;font-weight:700;color:var(--gl);letter-spacing:.06em}
.hd p{font-size:.75rem;color:var(--mu);margin-top:.2rem}
.card{background:var(--s1);border:1px solid rgba(201,168,76,.18);border-radius:16px;
  padding:1.25rem;margin-bottom:.9rem;position:relative;overflow:hidden}
.card::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;
  background:linear-gradient(90deg,transparent,var(--g),transparent)}
.lbl{font-size:.65rem;color:var(--mu);letter-spacing:.18em;text-transform:uppercase;margin-bottom:.45rem}
.st{font-size:1.05rem;color:var(--gl)}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;
  background:var(--mu);margin-right:6px;transition:background .25s}
.dot.on{background:#4CAF50;box-shadow:0 0 8px #4CAF50}
.sl{font-size:.78rem;color:var(--mu)}
.gg{display:grid;grid-template-columns:1fr 1fr;gap:.7rem;margin-bottom:.9rem}
.gc{background:var(--s2);border:1px solid rgba(201,168,76,.1);border-radius:12px;padding:.9rem;text-align:center}
.ga{font-size:.6rem;color:var(--mu);letter-spacing:.1em;margin-bottom:.35rem}
.gv{font-size:1.55rem;font-weight:600;color:var(--g);font-variant-numeric:tabular-nums}
.gu{font-size:.6rem;color:var(--mu)}
.pw{margin-bottom:.9rem;display:none}
.pw.show{display:block}
.pl{font-size:.72rem;color:var(--mu);margin-bottom:.45rem;display:flex;justify-content:space-between}
.pb{height:5px;border-radius:3px;background:rgba(201,168,76,.15);overflow:hidden}
.pf{height:100%;border-radius:3px;background:linear-gradient(90deg,var(--r),var(--g));transition:width .6s}
.lc{background:var(--s1);border:1px solid rgba(201,168,76,.38);border-radius:16px;
  padding:1.5rem 1.25rem;text-align:center;margin-bottom:.9rem;position:relative;overflow:hidden}
.lc::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;
  background:linear-gradient(90deg,transparent,var(--gl),transparent)}
.ln{font-size:3.4rem;font-weight:700;color:var(--gl);letter-spacing:.35em;padding-left:.35em;
  text-shadow:0 0 28px rgba(201,168,76,.35)}
.lt{font-size:.72rem;color:var(--mu);margin-top:.45rem}
.hi{display:flex;justify-content:space-between;align-items:center;
  padding:.55rem 0;border-bottom:1px solid rgba(201,168,76,.08);font-size:.82rem}
.hi:last-child{border-bottom:none}
.hn{color:var(--g);font-weight:600;letter-spacing:.12em}
.hd2{color:var(--mu);font-size:.68rem}
.nh{color:var(--mu);font-size:.82rem;text-align:center;padding:.9rem 0}
</style>
</head>
<body>
<div class="c">
  <div class="hd">
    <span class="hd-i">&#x1F38B;</span>
    <h1>&#x0E01;&#x0E23;&#x0E30;&#x0E1A;&#x0E2D;&#x0E01;&#x0E40;&#x0E0B;&#x0E35;&#x0E22;&#x0E21;&#x0E0B;&#x0E35;</h1>
    <p>MPU6050 &middot; ESP32 &middot; &#x0E40;&#x0E25;&#x0E02;&#x0E2B;&#x0E27;&#x0E22; 6 &#x0E2B;&#x0E25;&#x0E31;&#x0E01;</p>
  </div>
  <div class="card">
    <div class="lbl">&#x0E2A;&#x0E16;&#x0E32;&#x0E19;&#x0E30;&#x0E23;&#x0E30;&#x0E1A;&#x0E1A;</div>
    <div class="st" id="ST">&#x0E01;&#x0E33;&#x0E25;&#x0E31;&#x0E07;&#x0E40;&#x0E0A;&#x0E37;&#x0E48;&#x0E2D;&#x0E21;&#x0E15;&#x0E48;&#x0E2D;...</div>
    <div style="margin-top:.7rem">
      <span class="dot" id="DD"></span>
      <span class="sl" id="SL">&#x0E44;&#x0E21;&#x0E48;&#x0E21;&#x0E35;&#x0E01;&#x0E32;&#x0E23;&#x0E40;&#x0E02;&#x0E22;&#x0E48;&#x0E32;</span>
    </div>
  </div>
  <div class="gg">
    <div class="gc">
      <div class="ga">GYRO X</div><div class="gv" id="GX">&mdash;</div><div class="gu">&deg;/s</div>
    </div>
    <div class="gc">
      <div class="ga">GYRO Y</div><div class="gv" id="GY">&mdash;</div><div class="gu">&deg;/s</div>
    </div>
  </div>
  <div class="pw" id="PW">
    <div class="pl">
      <span id="PL">&#x0E01;&#x0E33;&#x0E25;&#x0E31;&#x0E07;&#x0E1A;&#x0E31;&#x0E19;&#x0E17;&#x0E36;&#x0E01;...</span>
      <span id="PP">0%</span>
    </div>
    <div class="pb"><div class="pf" id="PF" style="width:0%"></div></div>
  </div>
  <div class="lc">
    <div class="lbl">&#x0E1C;&#x0E25;&#x0E40;&#x0E25;&#x0E02;&#x0E2B;&#x0E27;&#x0E22;</div>
    <div class="ln" id="LN">&#x2014;&#x2014;</div>
    <div class="lt" id="LT">&#x0E22;&#x0E31;&#x0E07;&#x0E44;&#x0E21;&#x0E48;&#x0E21;&#x0E35;&#x0E1C;&#x0E25;</div>
  </div>
  <div class="card">
    <div class="lbl">&#x0E1B;&#x0E23;&#x0E30;&#x0E27;&#x0E31;&#x0E15;&#x0E34; 10 &#x0E23;&#x0E32;&#x0E22;&#x0E01;&#x0E32;&#x0E23;&#x0E25;&#x0E48;&#x0E32;&#x0E2A;&#x0E38;&#x0E14;</div>
    <div id="HL"><div class="nh">&#x0E22;&#x0E31;&#x0E07;&#x0E44;&#x0E21;&#x0E48;&#x0E21;&#x0E35;&#x0E1B;&#x0E23;&#x0E30;&#x0E27;&#x0E31;&#x0E15;&#x0E34;</div></div>
  </div>
</div>
<script>
async function poll(){
  try{
    var d=await(await fetch('/api/status',{cache:'no-store'})).json();
    document.getElementById('ST').textContent=d.status||'';
    document.getElementById('GX').textContent=(+d.gx).toFixed(1);
    document.getElementById('GY').textContent=(+d.gy).toFixed(1);
    var dd=document.getElementById('DD'),sl=document.getElementById('SL');
    if(d.shaking){dd.className='dot on';sl.textContent='\u0E15\u0E23\u0E27\u0E08\u0E1E\u0E1A\u0E01\u0E32\u0E23\u0E40\u0E02\u0E22\u0E48\u0E32!';}
    else{dd.className='dot';sl.textContent='\u0E44\u0E21\u0E48\u0E21\u0E35\u0E01\u0E32\u0E23\u0E40\u0E02\u0E22\u0E48\u0E32';}
    var pw=document.getElementById('PW');
    if(d.recording){
      pw.className='pw show';
      var p=Math.round(d.progress*100);
      document.getElementById('PP').textContent=p+'%';
      document.getElementById('PF').style.width=p+'%';
      var rem=Math.round((1-d.progress)*300);
      document.getElementById('PL').textContent='\u0E1A\u0E31\u0E19\u0E17\u0E36\u0E01... \u0E40\u0E2B\u0E25\u0E37\u0E2D '+rem+' \u0E27\u0E34';
    }else{pw.className='pw';}
    document.getElementById('LN').textContent=d.lottery||'\u2014\u2014';
    document.getElementById('LT').textContent=d.lottery_time||'\u0E22\u0E31\u0E07\u0E44\u0E21\u0E48\u0E21\u0E35\u0E1C\u0E25';
    var hl=document.getElementById('HL'),hist=d.history||[];
    hl.innerHTML=hist.length?hist.map(function(h){
      return '<div class="hi"><span class="hn">'+h.number+'</span><span class="hd2">'+h.datetime+'</span></div>';
    }).join(''):'<div class="nh">\u0E22\u0E31\u0E07\u0E44\u0E21\u0E48\u0E21\u0E35\u0E1B\u0E23\u0E30\u0E27\u0E31\u0E15\u0E34</div>';
  }catch(e){}
}
setInterval(poll,900);poll();
</script>
</body>
</html>)HTML";

// ============================================================
//  Web handlers
// ============================================================
void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html; charset=utf-8", HTML_PAGE);
}

void handleStatus() {
  DynamicJsonDocument doc(2048);
  doc["gx"]           = String(curGX, 1);
  doc["gy"]           = String(curGY, 1);
  doc["shaking"]      = curShaking;
  doc["status"]       = sysStatus;
  doc["lottery"]      = lastNumber;
  doc["lottery_time"] = lastTime;
  doc["recording"]    = (currentState == RECORDING);
  doc["progress"]     = recProgress;

  JsonArray hist = doc.createNestedArray("history");
  for (int i = 0; i < histCount; i++) {
    int idx = ((histHead - 1 - i) + MAX_HISTORY) % MAX_HISTORY;
    JsonObject h = hist.createNestedObject();
    h["number"]   = histBuf[idx].number;
    h["datetime"] = histBuf[idx].datetime;
  }

  String json;
  serializeJson(doc, json);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", json);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== กระบอกเซียมซี v2.0 ===");

  Wire.begin(21, 22);
  Wire.setClock(400000);


    enterWait();
  Serial.println("READY");  // ← เพิ่มบรรทัดนี้

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("เชื่อมต่อ WiFi: %s", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(400); Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, NTP_SERVER);
    struct tm ti;
    if (getLocalTime(&ti, 6000)) {
      ntpSynced = true;
      Serial.println("NTP sync ✓");
    } else {
      Serial.println("NTP timeout — ใช้ uptime แทน");
    }
  } else {
    Serial.println("\nWiFi ล้มเหลว → AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("SiamsiAP", "siamsi1234");
    Serial.printf("SSID: SiamsiAP  Password: siamsi1234\n");
    Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());
  }

  // MPU6050
  byte s = mpu.begin();
  for (int i = 0; s != 0 && i < 10; i++) {
    Serial.printf("MPU6050 ไม่พบ (status=%d) ลองใหม่...\n", s);
    delay(1000);
    s = mpu.begin();
  }
  if (s != 0) {
    Serial.println("[FATAL] MPU6050 ไม่ตอบสนอง — ตรวจสอบสาย SDA/SCL");
    while (true) delay(1000);
  }
  Serial.println("MPU6050 พบแล้ว — Calibrate (วางนิ่ง 3 วิ)...");
  delay(3000);
  mpu.calcOffsets(true, true);
  Serial.println("Calibrate ✓");

  server.on("/",           handleRoot);
  server.on("/api/status", handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Web server พร้อม ✓");

  enterWait();
}

// ============================================================
//  Loop — ไม่มี blocking delay
// ============================================================
void loop() {
  server.handleClient();

  static unsigned long lastRead = 0;
  unsigned long now = millis();
  if (now - lastRead < 100) return;
  lastRead = now;

  mpu.update();
  float gx = mpu.getGyroX();
  float gy = mpu.getGyroY();
  bool  sh = (fabsf(gx) > SHAKE_THRESHOLD || fabsf(gy) > SHAKE_THRESHOLD);

  curGX = gx; curGY = gy; curShaking = sh;

  switch (currentState) {

    case WAIT_SHAKE:
      shakeBufPush(sh);
      if (isShaking()) enterRecording();
      break;

    case RECORDING: {
      recSumX   += gx;
      recSumY   += gy;
      recSumSqX += gx * gx;
      recSumSqY += gy * gy;
      recCount++;
      if (sh) recShakeN++;

      unsigned long elapsed = now - recStartMs;
      unsigned long total   = (unsigned long)SHAKE_DURATION_S * 1000UL;
      recProgress = constrain((float)elapsed / (float)total, 0.0f, 1.0f);

      if (elapsed >= total) enterComputing();
      break;
    }

    case COMPUTING:
      break;   // enterComputing() จบใน 1 tick แล้วเปลี่ยน state เอง

    case COOLDOWN:
      sysStatus = String("พักระบบ — ผลเลข: ") + lastNumber;
      if (now - cooldownMs >= (unsigned long)COOLDOWN_S * 1000UL)
        enterWait();
      break;
  }
}
