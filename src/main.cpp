#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"

// ==========================================
// 1. Wi-Fi 설정 (본인 환경에 맞게 수정 필수)
// ==========================================
const char* ssid = "1실습실_2.4G";
const char* password = "tank5000";

// ==========================================
// 2. AI-Thinker ESP32-CAM 핀 정의
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

httpd_handle_t stream_httpd = NULL;
volatile bool isStreaming = false; // 웹 브라우저 접속 여부 플래그

// 최신 측정 결과 공유 변수
int curR = 0, curG = 0, curB = 0;
String curColor = "초기화 중...";

// ==========================================
// 3. 6가지 색상 판별 함수 (RGB -> HSV + 각도 보정)
// ==========================================
String detectCubeColor(int r, int g, int b) {
  float rf = r / 255.0f;
  float gf = g / 255.0f;
  float bf = b / 255.0f;

  float maxVal = max(rf, max(gf, bf));
  float minVal = min(rf, min(gf, bf));
  float delta = maxVal - minVal;

  // 명도(V: 0~100%) 및 채도(S: 0~100%) 계산
  float v = maxVal * 100.0f;
  float s = (maxVal == 0) ? 0 : (delta / maxVal) * 100.0f;

  // 무채색(검정색, 흰색) 우선 판별
  if (v < 25.0f) {
    return "BLACK (검정색)";
  }
  if (v > 75.0f && s < 22.0f) {
    return "WHITE (흰색)";
  }
  // 채도가 낮으면 배경 또는 회색조로 처리
  if (s < 25.0f) {
    return "UNKNOWN (배경/회색조)";
  }

  // 색상 각도(Hue: 0~360도) 계산
  float h = 0;
  if (delta > 0) {
    if (maxVal == rf) {
      h = 60.0f * fmod(((gf - bf) / delta), 6.0f);
    } else if (maxVal == gf) {
      h = 60.0f * (((bf - rf) / delta) + 2.0f);
    } else {
      h = 60.0f * (((rf - gf) / delta) + 4.0f);
    }
  }
  if (h < 0) h += 360.0f;

  // Hue 각도 구간별 4대 유채색 판별 (보정된 경계값 적용)
  if (h >= 335.0f || h < 32.0f) {
    return "RED (빨간색)";
  } else if (h >= 32.0f && h < 82.0f) {
    return "YELLOW (노란색)";
  } else if (h >= 82.0f && h < 165.0f) {
    return "GREEN (초록색)";
  } else if (h >= 165.0f && h < 260.0f) {
    return "BLUE (파란색)";
  }

  return "UNKNOWN (미분류)";
}

// ==========================================
// 4. 단일 프레임 분석 및 가이드라인 그리기 함수
// ==========================================
void processFrame(camera_fb_t *fb) {
  long sumR = 0, sumG = 0, sumB = 0;
  int pixelCount = 0;

  // 중앙 가이드라인 안쪽 픽셀 평균 계산 (X: 62~98, Y: 42~78)
  for (int y = 42; y < 79; y++) {
    for (int x = 62; x < 99; x++) {
      int idx = (y * fb->width + x) * 2;
      uint16_t pixel = (fb->buf[idx] << 8) | fb->buf[idx + 1];

      int r = ((pixel >> 11) & 0x1F) * 255 / 31;
      int g = ((pixel >> 5) & 0x3F) * 255 / 63;
      int b = (pixel & 0x1F) * 255 / 31;

      // 녹색 치우침(Green Tint) 보정 (R +15%, G -15%)
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
  curColor = detectCubeColor(curR, curG, curB);

  // 웹 화면용 초록색 가이드 박스 그리기 (RGB565 엔디안 반전: 0xE007)
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

// ==========================================
// 5. 웹 브라우저 실시간 스트리밍 핸들러
// ==========================================
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[64]; // 버퍼 배열 크기 명시

  isStreaming = true;
  res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");

  while (res == ESP_OK) {
    fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
      break;
    }

    processFrame(fb); // 스트리밍 중 프레임 분석 동시 수행

    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
    esp_camera_fb_return(fb);
    fb = NULL;

    if (!jpeg_converted) {
      res = ESP_FAIL;
      break;
    }

    size_t hlen = snprintf(part_buf, sizeof(part_buf), "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", _jpg_buf_len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n--frame\r\n", 12);

    if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    delay(30); // CPU 점유율 완화
  }

  isStreaming = false;
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

void setup() {
  Serial.begin(115200);
  
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
  
  // 오버플로우(EV-VSYNC-OVF) 방지를 위해 XCLK를 10MHz로 설정
  config.xclk_freq_hz = 10000000; 
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QQVGA; // 160x120
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("[Error] 카메라 초기화 실패");
    return;
  }

  // 부팅 직후 1.5초간 현재 조명에 맞춰 화이트밸런스 보정 후 고정
  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  delay(1500);
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 0);

  WiFi.begin(ssid, password);
  Serial.print("Wi-Fi 연결 중");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[Info] Wi-Fi 연결 완료!");
  Serial.print("[Info] 브라우저 접속 주소: http://");
  Serial.println(WiFi.localIP());

  startCameraServer();
}

void loop() {
  static unsigned long lastPrint = 0;

  // 웹 스트리밍 미사용 시에도 백그라운드에서 색상 감지 수행
  if (!isStreaming) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      processFrame(fb);
      esp_camera_fb_return(fb);
    }
  }

  // 0.3초 간격으로 시리얼 모니터에 결과 출력
  if (millis() - lastPrint >= 300) {
    Serial.printf("R:%3d G:%3d B:%3d --> %s\n", curR, curG, curB, curColor.c_str());
    lastPrint = millis();
  }
}
