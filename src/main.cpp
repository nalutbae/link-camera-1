#include <Arduino.h>
#include <Modbus.h>       // 명시적 추가
#include <ModbusRTU.h>    // 카메라 헤더보다 먼저 선언
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"


// ==========================================
// 1. 운용 모드 스위치 (매우 중요!)
// ==========================================
// true  : PC USB(ESP32-CAM-MB) 연결 시 시리얼 모니터 디버깅 및 Wi-Fi 스트리밍 활성
// false : 현장 RS485(PLC) 연결 시 순수 Modbus RTU 슬레이브로 동작 (MB 보드 분리 필수)
#define ENABLE_DEBUG_MODE true

// ==========================================
// 2. Wi-Fi 및 Modbus RTU 설정
// ==========================================
const char* ssid = "1실습실_2.4G";
const char* password = "tank5000";

#define SLAVE_ID        1       // Modbus 슬레이브 국번 (1번)
#define MODBUS_BAUD     9600    // PLC 통신 속도와 일치시킬 것
#define RS485_DE_RE_PIN 13      // MAX485 DE/RE 핀 연결 번호

// Modbus 레지스터 주소 정의
#define REG_COLOR_CODE  0       // 40001: 색상 숫자 코드
#define REG_R_VAL       1       // 40002: R 값
#define REG_G_VAL       2       // 40003: G 값
#define REG_B_VAL       3       // 40004: B 값

// ==========================================
// 3. AI-Thinker ESP32-CAM 핀 정의
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

int curR = 0, curG = 0, curB = 0;
uint16_t curColorCode = 0;
uint16_t lastLoggedCode = 0; // 직전에 로그로 출력한 색상 코드 기억용

// Modbus 레지스터 주소 정의에 감지 횟수(카운트) 추가
#define REG_DETECT_CNT  4       // 40005: 새 물체 감지 누적 횟수 (연속 동일 색상 구분용)

// 상태 추적 변수 추가
bool isObjectRemoved = true;    // 물체가 치워져 배경(0)을 거쳤는지 여부
bool hasNewDetection = false;   // 새 물체 감지 이벤트 발생 플래그
uint16_t detectCount = 0;       // 새 물체 감지 횟수


// ==========================================
// 4. 6가지 색상 판별 함수 (숫자 코드 반환)
// ==========================================
// 반환값: 0(미감지), 1(흰색), 2(검정), 3(빨강), 4(노랑), 5(초록), 6(파랑)
uint16_t detectCubeColorCode(int r, int g, int b) {
  float rf = r / 255.0f;
  float gf = g / 255.0f;
  float bf = b / 255.0f;

  float maxVal = max(rf, max(gf, bf));
  float minVal = min(rf, min(gf, bf));
  float delta = maxVal - minVal;

  float v = maxVal * 100.0f;
  float s = (maxVal == 0) ? 0 : (delta / maxVal) * 100.0f;

  if (v < 25.0f) return 2;                   // 검정색 (BLACK)
  if (v > 75.0f && s < 22.0f) return 1;      // 흰색 (WHITE)
  if (s < 25.0f) return 0;                   // 미감지/배경 (UNKNOWN)

  float h = 0;
  if (delta > 0) {
    if (maxVal == rf) h = 60.0f * fmod(((gf - bf) / delta), 6.0f);
    else if (maxVal == gf) h = 60.0f * (((bf - rf) / delta) + 2.0f);
    else h = 60.0f * (((rf - gf) / delta) + 4.0f);
  }
  if (h < 0) h += 360.0f;

  if (h >= 335.0f || h < 32.0f) return 3;    // 빨간색 (RED)
  if (h >= 32.0f && h < 82.0f) return 4;     // 노란색 (YELLOW)
  if (h >= 82.0f && h < 165.0f) return 5;    // 초록색 (GREEN)
  if (h >= 165.0f && h < 260.0f) return 6;   // 파란색 (BLUE)

  return 0;                                  // 미분류
}

// 색상 숫자 코드를 한글 이름으로 변환하는 헬퍼 함수
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

// ==========================================
// 5. 프레임 분석 및 Modbus 레지스터 갱신
// ==========================================
void processFrame(camera_fb_t *fb) {
  long sumR = 0, sumG = 0, sumB = 0;
  int pixelCount = 0;

  for (int y = 42; y < 79; y++) {
    for (int x = 62; x < 99; x++) {
      int idx = (y * fb->width + x) * 2;
      uint16_t pixel = (fb->buf[idx] << 8) | fb->buf[idx + 1];

      int r = ((pixel >> 11) & 0x1F) * 255 / 31;
      int g = ((pixel >> 5) & 0x3F) * 255 / 63;
      int b = (pixel & 0x1F) * 255 / 31;

      // 녹색 치우침 보정
      r = min(255, (int)(r * 1.15f));
      g = (int)(g * 0.85f);

      sumR += r;
      sumG += g;
      sumB += b;
      pixelCount++;
    }
  }

  curR = sumR / pixelCount;
  curG = sumG / pixelCount;
  curB = sumB / pixelCount;
  uint16_t detected = detectCubeColorCode(curR, curG, curB);

  if (detected == 0) {
    // 물체가 치워져 배경 상태가 되면 다음 물체를 받을 준비 완료
    isObjectRemoved = true;
  } else {
    // 배경(0)을 거친 후 새로 들어왔거나, 놓인 상태에서 색상이 바뀐 경우
    if (isObjectRemoved || (detected != curColorCode)) {
      curColorCode = detected;
      isObjectRemoved = false;   // 물체가 올라와 있는 상태로 잠금
      hasNewDetection = true;    // 로그 출력 트리거 활성화
      detectCount++;             // 감지 카운트 증가 (PLC가 동일 색상 연속 투입 인지용)

      if (!ENABLE_DEBUG_MODE) {
        mb.Hreg(REG_COLOR_CODE, curColorCode);
        mb.Hreg(REG_R_VAL, (uint16_t)curR);
        mb.Hreg(REG_G_VAL, (uint16_t)curG);
        mb.Hreg(REG_B_VAL, (uint16_t)curB);
        mb.Hreg(REG_DETECT_CNT, detectCount);
      }
    }
  }

  // 디버깅 모드일 때만 웹 스트리밍용 가이드 박스 그리기
  if (ENABLE_DEBUG_MODE) {
    uint16_t *buf16 = (uint16_t *)fb->buf;
    for (int x = 60; x <= 100; x++) {
      buf16[40 * fb->width + x] = 0xE007;
      buf16[80 * fb->width + x] = 0xE007;
    }
    for (int y = 40; y <= 80; y++) {
      buf16[y * fb->width + 60] = 0xE007;
      buf16[y * fb->width + 100] = 0xE007;
    }
  }
}

// ==========================================
// 6. 웹 스트리밍 서버 (디버깅 모드 전용)
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
    delay(30);
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
// 7. 초기 설정 (Setup)
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
  config.frame_size = FRAMESIZE_QQVGA;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  if (esp_camera_init(&config) != ESP_OK) {
    while (true) { delay(1000); }
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  delay(1500);
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 0);

  if (ENABLE_DEBUG_MODE) {
    // [모드 1] PC 디버깅 및 웹 화면 확인 모드
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }
    Serial.print("\n[Info] 브라우저 접속 주소: http://");
    Serial.println(WiFi.localIP());
    startCameraServer();
  } else {
    // [모드 2] 현장 Modbus RTU 슬레이브 운용 모드 (Wi-Fi OFF로 전력 및 안정성 극대화)
    WiFi.mode(WIFI_OFF);
    Serial.begin(MODBUS_BAUD, SERIAL_8N1);
    mb.begin(&Serial, RS485_DE_RE_PIN); // GPIO 13을 DE/RE 제어 핀으로 자동 사용
    mb.slave(SLAVE_ID);

    // 홀딩 레지스터 0~3번 생성 및 초기화
    mb.addHreg(REG_COLOR_CODE, 0);
    mb.addHreg(REG_R_VAL, 0);
    mb.addHreg(REG_G_VAL, 0);
    mb.addHreg(REG_B_VAL, 0);
    mb.addHreg(REG_DETECT_CNT, 0);

  }
}

// ==========================================
// 8. 메인 루프
// ==========================================
void loop() {
  static unsigned long lastScan = 0;

  if (!ENABLE_DEBUG_MODE) {
    // Modbus RTU 폴링 처리 (PLC 요청에 즉각 응답하기 위해 루프 최상단에서 계속 호출)
    mb.task();
  }

  // 0.1초(100ms) 주기로 카메라 프레임 캡처 및 색상 업데이트
  if (!isStreaming && (millis() - lastScan >= 100)) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      processFrame(fb);
      esp_camera_fb_return(fb);
    }

    // if (ENABLE_DEBUG_MODE) {
    //   // Serial.printf("Code:%d | R:%3d G:%3d B:%3d\n", curColorCode, curR, curG, curB);
    //   Serial.printf("%d: %s (R:%3d G:%3d B:%3d)\n", curColorCode, getColorName(curColorCode), curR, curG, curB);
    // }

    if (ENABLE_DEBUG_MODE && hasNewDetection) {
      Serial.printf("[#%d] %d: %s (R:%3d G:%3d B:%3d)\n", detectCount, curColorCode, getColorName(curColorCode), curR, curG, curB);
      hasNewDetection = false;
    }

    lastScan = millis();
  }
}
