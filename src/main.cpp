#include <Arduino.h>
#include <Modbus.h>
#include <ModbusRTU.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"

// ==========================================
// 1. 운용 모드 스위치
// ==========================================
//  true  = 웹 스트리밍 디버깅 모드 (Wi-Fi 연결, 브라우저에서 영상 확인)
//  false = Modbus RTU 485 운용 모드 (PLC와 통신)
#define ENABLE_DEBUG_MODE true

// ==========================================
// 2. Wi-Fi 및 Modbus RTU 설정
// ==========================================
const char *ssid = "1실습실_2.4G";
const char *password = "tank5000";

#define SLAVE_ID        1
#define MODBUS_BAUD     9600
#define RS485_DE_RE_PIN 13

// Modbus 홀딩 레지스터 주소 정의 (0-based, Modbus 40001번지부터)
#define REG_COLOR_CODE  0       // 40001: 색상 코드 (1:흰, 2:검, 3:빨, 4:노, 5:초, 6:파)
#define REG_R_VAL       1       // 40002: R 값
#define REG_G_VAL       2       // 40003: G 값
#define REG_B_VAL       3       // 40004: B 값
#define REG_DETECT_CNT  4       // 40005: 새 물체 감지 누적 횟수
#define REG_SIZE_CODE   5       // 40006: 크기 코드 (1:소, 2:중, 3:대)
#define REG_RATIO_X100  6       // 40007: 긴변/짧은변 비율 x 100 (예: 157 = 1.57배)

// ==========================================
// 3. 스테인리스 배경 차분 임계값
// ==========================================
// 배경과 물체의 명도(V) 차이 임계값 — HSV 명도 기반 감지
//   - 은색판 위 물체는 색상 차이가 적어도 명도 차이로 감지 가능
//   - 너무 낮으면 노이즈, 너무 높으면 물체 미감지
const int BG_DIFF_THRESH = 15;   // 명도(V) 차이 임계값 (0~255)

// 물체로 인정할 최소 픽셀 수 (QVGA 160x120 기준)
const int MIN_OBJ_PIXELS = 100;

// 가장 작은 변(35mm) 대비 가장 긴 변의 비율 기준
//   이론값: 소(35/35=1.00), 중(55/35=1.57), 대(85/35=2.43)
//   실측 여유분: ±0.3~0.4
const float RATIO_SMALL_MAX  = 1.32f; // < 1.32 → 소 (35mm)
const float RATIO_MEDIUM_MAX = 1.98f; // 1.32~1.98 → 중 (55mm), > 1.98 → 대 (85mm)

// ==========================================
// 4. 안정화 대기 시간 (크기별 차등)
// ==========================================
// 물체가 은색판 위에 놓인 후 진동/반사가 가라앉을 때까지 대기
//   소(35mm) = 가벼워서 빨리 안정 → 300ms
//   중(55mm) = 보통 → 500ms
//   대(85mm) = 무거워서 느리게 안정 → 700ms
const unsigned long STABILIZE_SMALL = 300;
const unsigned long STABILIZE_MEDIUM = 500;
const unsigned long STABILIZE_LARGE = 700;

// ==========================================
// 5. AI-Thinker ESP32-CAM 핀 정의
// ==========================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

ModbusRTU mb;
httpd_handle_t stream_httpd = NULL;
volatile bool isStreaming = false;

// 부팅 시 학습할 빈 스테인리스판 기준 배경 버퍼 (QQVGA: 160x120)
uint8_t bgR[160 * 120];
uint8_t bgG[160 * 120];
uint8_t bgB[160 * 120];
bool isBgCaptured = false;

// 확정된 최신 측정 결과
int curR = 0, curG = 0, curB = 0;
uint16_t curColorCode = 0;
uint16_t curSizeCode = 0;
float curRatio = 0.0f;
uint16_t detectCount = 0;

// 상태 추적 변수
bool isObjectRemoved = true;
bool hasNewDetection = false;
unsigned long objectEnterTime = 0;
bool isWaitingStabilize = false;
uint16_t pendingSizeCode = 0;  // 안정화 대기 중 잠정 크기

// ==========================================
// 6. 6가지 색상 정밀 판별 함수 (금속 반사 보정)
// ==========================================
uint16_t detectCubeColorCode(int r, int g, int b) {
  float rf = r / 255.0f;
  float gf = g / 255.0f;
  float bf = b / 255.0f;

  float maxVal = max(rf, max(gf, bf));
  float minVal = min(rf, min(gf, bf));
  float delta = maxVal - minVal;

  float v = maxVal * 100.0f;
  float s = (maxVal == 0) ? 0 : (delta / maxVal) * 100.0f;

  // 1) 검정색: 명도(V) 35% 미만, 채도(S) 낮음
  if (v < 35.0f && s < 45.0f) {
    return 2; // 검정색 (BLACK)
  }

  // 2) 흰색: 명도(V) 70% 초과, 채도(S) 낮음
  if (v > 70.0f && s < 25.0f) {
    return 1; // 흰색 (WHITE)
  }

  // 2-1) 저채도 보호: AWB 끈 후 R이 미세하게 높은 센서 특성 보정
  //   채도가 20% 미만이고 R-G 차이가 25 이하면 무채색(흰/검)로 처리
  //   (Hue가 빨강으로 계산되더라도 채도가 낮으면 색상이 아님)
  if (s < 20.0f) {
    int rgDiff = r - g;  // R-G 차이
    if (v >= 50.0f) {
      return 1; // 명도 높음 → 흰색 (WHITE)
    } else {
      return 2; // 명도 낮음 → 검정색 (BLACK)
    }
  }

  // 색상 각도(Hue) 계산
  float h = 0;
  if (delta > 0) {
    if (maxVal == rf) h = 60.0f * fmod(((gf - bf) / delta), 6.0f);
    else if (maxVal == gf) h = 60.0f * (((bf - rf) / delta) + 2.0f);
    else h = 60.0f * (((rf - gf) / delta) + 4.0f);
  }
  if (h < 0) h += 360.0f;

  // 3) 4대 유채색 구분 (노랑/초록 경계 엄격 분리)
  if (h >= 335.0f || h < 28.0f) {
    return 3; // 빨간색 (RED)
  } else if (h >= 28.0f && h < 68.0f) {
    // 노란색은 명도(V)가 충분히 높아야 함 (어두운 노란빛은 검정/초록 반사광)
    if (v >= 45.0f) return 4; // 노란색 (YELLOW)
    else return 2;            // 어두우면 검정색 처리
  } else if (h >= 68.0f && h < 165.0f) {
    return 5; // 초록색 (GREEN)
  } else if (h >= 165.0f && h < 260.0f) {
    return 6; // 파란색 (BLUE)
  }

  return 0;
}

// ==========================================
// 7. 변 비율 기반 크기 판별 함수
// ==========================================
uint16_t classifySizeByRatio(int widthPx, int heightPx, float &outRatio) {
  int shortSide = min(widthPx, heightPx);
  int longSide  = max(widthPx, heightPx);

  if (shortSide <= 5) return 0; // 노이즈 방어

  outRatio = (float)longSide / (float)shortSide;

  if (outRatio < RATIO_SMALL_MAX) {
    return 1; // 소 (35×35×35, 비율 약 1.0)
  } else if (outRatio <= RATIO_MEDIUM_MAX) {
    return 2; // 중 (35×35×55, 비율 약 1.57)
  } else {
    return 3; // 대 (35×35×85, 비율 약 2.43)
  }
}

// ==========================================
// 8. 크기별 안정화 대기 시간 산출
// ==========================================
unsigned long getStabilizeDelay(uint16_t sizeCode) {
  switch (sizeCode) {
    case 1: return STABILIZE_SMALL;   // 소: 300ms
    case 2: return STABILIZE_MEDIUM;  // 중: 500ms
    case 3: return STABILIZE_LARGE;   // 대: 700ms
    default: return STABILIZE_MEDIUM;
  }
}

const char* getColorName(uint16_t code) {
  switch (code) {
    case 1: return "흰색";
    case 2: return "검정색";
    case 3: return "빨간색";
    case 4: return "노란색";
    case 5: return "초록색";
    case 6: return "파란색";
    default: return "미감지";
  }
}

const char* getSizeName(uint16_t code) {
  switch (code) {
    case 1: return "소(35mm)";
    case 2: return "중(55mm)";
    case 3: return "대(85mm)";
    default: return "미판별";
  }
}

// ==========================================
// 9. 초기 빈 스테인리스판 배경 학습 함수
// ==========================================
void captureBackground() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  for (int i = 0; i < fb->width * fb->height; i++) {
    uint16_t pixel = (fb->buf[i * 2] <<8) | fb->buf[i * 2 +1];
    // 배경은 HSV 명도(V)만 저장 (색상 차이가 적은 은색판 대응)
    bgR[i] = ((pixel >> 11) & 0x1F) * 255 / 31;  // R (Hue 계산용)
    bgG[i] = ((pixel >> 5) & 0x3F) * 255 / 63;  // G (Hue 계산용)
    // 명도(V) = 3채널 중 최댓값 → 밝기
    int r5 = ((pixel >>11) & 0x1F) * 255 / 31;
    int g6 = ((pixel >>5) & 0x3F) * 255 / 63;
    int b5 = (pixel & 0x1F) * 255 / 31;
    bgB[i] = max(r5, max(g6, b5));  // V값 저장 (명도)
  }
  esp_camera_fb_return(fb);
  isBgCaptured = true;
}

// ==========================================
// 10. 프레임 분석 (배경 차분 + 비율 크기 + 중심부 색상)
// ==========================================
void processFrame(camera_fb_t *fb) {
  if (!isBgCaptured) return;

  int minX = fb->width, maxX = 0;
  int minY = fb->height, maxY = 0;
  int diffPixels = 0;

  // 1단계: 배경과 비교하여 물체 외곽 좌표(Bounding Box) 추출
  //   — HSV 명도(V) 차이 기반: 색상 차이가 적어도 밝기 차이로 감지
  //   — 외곽 15px 여백(margin)으로 스테인리스 가장자리 반사 노이즈 제외
  for (int y = 15; y < fb->height - 15; y++) {
    for (int x = 20; x < fb->width - 20; x++) {
      int pIdx = y * fb->width + x;
      uint16_t pixel = (fb->buf[pIdx * 2] <<8) | fb->buf[pIdx * 2 +1];

      int r = ((pixel >>11) & 0x1F) * 255 / 31;
      int g = ((pixel >>5) & 0x3F) * 255 / 63;
      int b = (pixel & 0x1F) * 255 / 31;

      // 현재 픽셀의 명도(V) = 3채널 중 최댓값
      int curV = max(r, max(g, b));
      // 배경 명도(V)와의 차이
      int diff = abs(curV - bgB[pIdx]);

      if (diff > BG_DIFF_THRESH) {
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
        diffPixels++;
      }
    }
  }

  // 물체가 없음 (제거됨 또는 미배치)
  //   — diffPixels가 기준 미만이면 물체가 치워졌다고 판단
  //   — 단, 연속해서 감지되는 동안은 일시적 변동으로 무시
  if (diffPixels < MIN_OBJ_PIXELS) {
    if (!isObjectRemoved) {
      isObjectRemoved = true;
      isWaitingStabilize = false;
      if (ENABLE_DEBUG_MODE) {
        Serial.printf("[DBG] diffPixels=%d < %d → 물체 제거됨\n",
            diffPixels, MIN_OBJ_PIXELS);
      }
    }
    return;
  }

  // 디버그: 물체 감지됨
  if (ENABLE_DEBUG_MODE) {
    Serial.printf("[DBG] diffPixels=%d, box=(%d,%d)-(%d,%d) %dx%d\n",
        diffPixels, minX, minY, maxX, maxY, maxX - minX, maxY - minY);
  }

  // 2단계: 물체가 새로 들어온 상태 — 안정화 대기 시작
  if (isObjectRemoved) {
    if (!isWaitingStabilize) {
      // 1차 바운딩 박스에서 잠정 크기 산출 → 크기별 안정화 대기시간 설정
      int boxW = maxX - minX;
      int boxH = maxY - minY;
      float dummyRatio = 0.0f;
      pendingSizeCode = classifySizeByRatio(boxW, boxH, dummyRatio);

      objectEnterTime = millis();
      isWaitingStabilize = true;
    } else {
      // 크기별 차등 안정화 대기 시간 적용
      unsigned long stabilizeDelay = getStabilizeDelay(pendingSizeCode);
      if (millis() - objectEnterTime >= stabilizeDelay) {
        int boxW = maxX - minX;
        int boxH = maxY - minY;

        // 3단계: 외곽 30% 버리고 물체 정중앙(Core 40%) 영역만 색상 추출
        //   — 금속 반사광이 섞이는 가장자리를 버리고 순수 색상만 평균
        int coreMinX = minX + (boxW * 0.30f);
        int coreMaxX = maxX - (boxW * 0.30f);
        int coreMinY = minY + (boxH * 0.30f);
        int coreMaxY = maxY - (boxH * 0.30f);

        long sumR = 0, sumG = 0, sumB = 0;
        int coreCount = 0;

        for (int y = coreMinY; y <= coreMaxY; y++) {
          for (int x = coreMinX; x <= coreMaxX; x++) {
            int pIdx = y * fb->width + x;
            uint16_t pixel = (fb->buf[pIdx * 2] << 8) | fb->buf[pIdx * 2 +1];

            int r = ((pixel >> 11) & 0x1F) * 255 / 31;
            int g = ((pixel >> 5) & 0x3F) * 255 / 63;
            int b = (pixel & 0x1F) * 255 / 31;

            // RGB565 → 8bit 변환 시 R은 5bit(31단계), G은 6bit(63단계)이므로
            // G이 본래보다 밝게 보이는 센서 특성만 보정 (R/G 비율은 건드리지 않음)
            r = (int)(r * 31 / 31);  // R 보정 없음
            g = (int)(g * 63 / 63);  // G 보정 없음 (이전 0.88 제거)

            sumR += r;
            sumG += g;
            sumB += b;
            coreCount++;
          }
        }

        if (coreCount > 0) {
          int avgR = sumR / coreCount;
          int avgG = sumG / coreCount;
          int avgB = sumB / coreCount;

          float ratio = 0.0f;
          uint16_t detectedColor = detectCubeColorCode(avgR, avgG, avgB);
          uint16_t detectedSize  = classifySizeByRatio(boxW, boxH, ratio);

          if (detectedColor != 0 && detectedSize != 0) {
            curR = avgR;
            curG = avgG;
            curB = avgB;
            curColorCode = detectedColor;
            curSizeCode  = detectedSize;
            curRatio     = ratio;

            isObjectRemoved = false;
            isWaitingStabilize = false;
            hasNewDetection = true;
            detectCount++;

            // 디버그: 최종 감지 결과
            if (ENABLE_DEBUG_MODE) {
              Serial.printf("[DBG] DETECTED color=%d(%s) size=%d(%s) ratio=%.2f RGB=(%d,%d,%d)\n",
                  curColorCode, getColorName(curColorCode),
                  curSizeCode, getSizeName(curSizeCode),
                  curRatio, curR, curG, curB);
            }

            if (!ENABLE_DEBUG_MODE) {
              mb.Hreg(REG_COLOR_CODE, curColorCode);
              mb.Hreg(REG_R_VAL, (uint16_t)curR);
              mb.Hreg(REG_G_VAL, (uint16_t)curG);
              mb.Hreg(REG_B_VAL, (uint16_t)curB);
              mb.Hreg(REG_DETECT_CNT, detectCount);
              mb.Hreg(REG_SIZE_CODE, curSizeCode);
              mb.Hreg(REG_RATIO_X100, (uint16_t)(curRatio * 100));
            }
          }
        }
      }
    }
  }

  // 웹 스트리밍 디버깅 시 감지된 바운딩 박스 그리기
  if (ENABLE_DEBUG_MODE && maxX > minX && maxY > minY) {
    uint16_t *buf16 = (uint16_t *)fb->buf;
    for (int x = minX; x <= maxX; x++) {
      buf16[minY * fb->width + x] = 0xE007;
      buf16[maxY * fb->width + x] = 0xE007;
    }
    for (int y = minY; y <= maxY; y++) {
      buf16[y * fb->width + minX] = 0xE007;
      buf16[y * fb->width + maxX] = 0xE007;
    }
  }
}

// ==========================================
// 11. 웹 스트리밍 핸들러
// ==========================================
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[64];

  isStreaming = true;
  res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");

  while (res == ESP_OK) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    processFrame(fb);
    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
    esp_camera_fb_return(fb);
    fb = NULL;
    if (!jpeg_converted) { res = ESP_FAIL; break; }
    size_t hlen = snprintf(part_buf, sizeof(part_buf), "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", _jpg_buf_len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n--frame\r\n", 12);
    if (_jpg_buf) { free(_jpg_buf); _jpg_buf = NULL; }
    delay(40);
  }
  isStreaming = false;
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

// ==========================================
// 12. 초기 설정 (Setup)
// ==========================================
void setup() {
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QQVGA; // 160×120 (ESP32 메모리 효율)
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;
  if (esp_camera_init(&config) != ESP_OK) {
    while (true) { delay(1000); }
  }

  // 카메라 센서 설정
  //   — AWB(화이트밸런스) 끄기: 따뜻한 색조(R 과다) 보정 — 자동이 R을 높여 저채도 색이 모두 빨강으로 왜곡
  //   — AEC(자동노출)는 자동 유지: 물체 놓일 때 화면 밝기 급변 보정
  sensor_t *s = esp_camera_sensor_get();

  // 1단계: 자동 모드에서 안정화 (2초)
  s->set_whitebal(s,1);
  s->set_awb_gain(s,1);
  s->set_exposure_ctrl(s,1);
  delay(2000);

  // 2단계: AWB 끄기 (R 과다 보정 차단)
  s->set_whitebal(s,0);
  s->set_awb_gain(s,0);
  // 노출은 자동 유지 (화면 어두워짐 방지)

  // 3단계: AWB 고정 후 센서 안정화 대기 (20프레임 플러시)
  for (int i = 0; i < 20; i++) {
    camera_fb_t *flush = esp_camera_fb_get();
    if (flush) esp_camera_fb_return(flush);
    delay(50);
  }

  // 4단계: 빈 스테인리스판 기준 배경 캡처 (AWB 고정된 상태)
  captureBackground();

  // 검증: 캡처 후 첫 프레임과 배경 비교
  if (ENABLE_DEBUG_MODE) {
    camera_fb_t *verify = esp_camera_fb_get();
    if (verify) {
      int verifyDiff = 0;
      for (int i = 0; i < verify->width * verify->height; i++) {
        uint16_t pixel = (verify->buf[i * 2] << 8) | verify->buf[i * 2 +1];
        int r = ((pixel >> 11) & 0x1F) * 255 / 31;
        int g = ((pixel >> 5) & 0x3F) * 255 / 63;
        int b = (pixel & 0x1F) * 255 / 31;
        int diff = abs(r - bgR[i]) + abs(g - bgG[i]) + abs(b - bgB[i]);
        if (diff > BG_DIFF_THRESH) verifyDiff++;
      }
      esp_camera_fb_return(verify);
      Serial.printf("[DBG] 배경 검증: diffPixels=%d (0에 가까워야 정상)\n", verifyDiff);
      if (verifyDiff > 500) {
        // 배경 학습 실패 — 화이트밸런스 고정이 안 된 상태
        // 강제 재캡처 시도 (1회 추가)
        Serial.printf("[DBG] 배경 불안정 (diff=%d) — 재캡처 시도\n", verifyDiff);
        delay(1000);
        for (int i = 0; i < 10; i++) {
          camera_fb_t *flush = esp_camera_fb_get();
          if (flush) esp_camera_fb_return(flush);
          delay(50);
        }
        captureBackground();
        camera_fb_t *verify2 = esp_camera_fb_get();
        if (verify2) {
          int verifyDiff2 = 0;
          for (int i = 0; i < verify2->width * verify2->height; i++) {
            uint16_t pixel = (verify2->buf[i * 2] <<8) | verify2->buf[i * 2 +1];
            int r = ((pixel >> 11) & 0x1F) * 255 / 31;
            int g = ((pixel >> 5) & 0x3F) * 255 / 63;
            int b = (pixel & 0x1F) * 255 / 31;
            int diff = abs(r - bgR[i]) + abs(g - bgG[i]) + abs(b - bgB[i]);
            if (diff > BG_DIFF_THRESH) verifyDiff2++;
          }
          esp_camera_fb_return(verify2);
          Serial.printf("[DBG] 재캡처 후 검증: diffPixels=%d\n", verifyDiff2);
        }
      }
    }
  }

  if (ENABLE_DEBUG_MODE) {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }
    Serial.print("\n[Info] 빈 스테인리스 배경 학습 완료. 웹 주소: http://");
    Serial.println(WiFi.localIP());
    startCameraServer();
  } else {
    WiFi.mode(WIFI_OFF);
    Serial.begin(MODBUS_BAUD, SERIAL_8N1);
    mb.begin(&Serial, RS485_DE_RE_PIN);
    mb.slave(SLAVE_ID);
    mb.addHreg(REG_COLOR_CODE, 0);
    mb.addHreg(REG_R_VAL, 0);
    mb.addHreg(REG_G_VAL, 0);
    mb.addHreg(REG_B_VAL, 0);
    mb.addHreg(REG_DETECT_CNT, 0);
    mb.addHreg(REG_SIZE_CODE, 0);
    mb.addHreg(REG_RATIO_X100, 0);
  }
}

// ==========================================
// 13. 메인 루프
// ==========================================
void loop() {
  static unsigned long lastScan = 0;

  if (!ENABLE_DEBUG_MODE) {
    mb.task();
  }

  // 감지 주기: 60ms (기존 80ms → 반응성 향상)
  if (!isStreaming && (millis() - lastScan >= 60)) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      processFrame(fb);
      esp_camera_fb_return(fb);
    }
    lastScan = millis();
  }

  if (ENABLE_DEBUG_MODE && hasNewDetection) {
    Serial.printf("[#%d] 색상 %d:%s (R:%d G:%d B:%d) | 크기 %d:%s (변 비율: %.2f)\n",
      detectCount,
      curColorCode, getColorName(curColorCode),
      curR, curG, curB,
      curSizeCode, getSizeName(curSizeCode),
      curRatio);
    hasNewDetection = false;
  }
}
