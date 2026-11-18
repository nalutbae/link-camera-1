#include <Arduino.h>
#include <Modbus.h>
#include <ModbusRTU.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <stdarg.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "img_converters.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ==========================================
// 1. 운용 모드 스위치 (둘 다 켜도 됨)
// ==========================================
//  ENABLE_WEB    true  = Wi-Fi 웹 화면: 영상/마스크, 감지 기록(시스템 로그), Modbus 레지스터 값, 통신·카메라 상태
//                        → USB 개발보드(ESP32-CAM-MB) 없이 485 운용 중에도 브라우저로 상태 확인
//  ENABLE_MODBUS true  = UART0(U0T=GPIO1 / U0R=GPIO3)로 Modbus RTU 485 통신 (현장 운용: 485 모듈 연결)
//                false = 같은 UART0로 시리얼 로그 출력 (개발: ESP32-CAM-MB에 꽂아 시리얼 모니터로 볼 때)
//                ※ UART0는 USB 업로드·시리얼 모니터와 같은 핀이다.
//                  - true로 올리면 시리얼 모니터에는 부팅 안내 한 줄 외에 아무것도 나오지 않는다 (정상)
//                  - 485 운용 중에는 ESP32-CAM-MB를 빼고, 업로드할 때는 485 모듈의 TXD/RXD 선을 뺀다
#define ENABLE_WEB    true
#define ENABLE_MODBUS true
#define SERIAL_LOG    (!ENABLE_MODBUS)   // UART0는 하나뿐이라 Modbus와 시리얼 로그를 동시에 쓸 수 없다

// ==========================================
// 2. Wi-Fi 및 Modbus RTU 설정
// ==========================================
const char *ssid = "1실습실_2.4G";
const char *password = "tank5000";
const char *HOSTNAME = "link-cam1";     // 시리얼 없이 접속: http://link-cam1.local (또는 공유기 DHCP 목록에서 IP 확인)

#define SLAVE_ID        6
#define MODBUS_BAUD     9600
// 485 모듈 연결 핀 (UART0, ESP32-CAM 기본 시리얼 핀). 보드 인쇄: U0T = GPIO1, U0R = GPIO3
#define MB_SERIAL       Serial
#define MB_TX_PIN       1       // U0T: ESP32 송신 → 485 모듈
#define MB_RX_PIN       3       // U0R: 485 모듈 → ESP32 수신
// 485 송수신 방향 제어 핀
//   -1 : 자동 방향 모듈 (TTL485-V2.0 등 VCC/TXD/RXD/GND 4핀) — 연결할 핀 없음
//   13 : MAX485처럼 DE/RE 핀이 있는 모듈 — DE와 RE를 묶어 GPIO13에 연결
#define RS485_DE_RE_PIN -1
const unsigned long MB_LINK_TIMEOUT_MS = 5000;   // 이 시간 동안 요청이 없으면 '요청 없음'으로 표시

// Modbus 홀딩 레지스터 주소 정의 (0-based, Modbus 40001번지부터)
//   PLC에서 가장 중요한 색상·크기·물체 유무를 앞쪽(40001~40003)에 둔다
#define REG_COLOR_CODE  0       // 40001: 색상 코드 (1:흰, 2:검, 3:빨, 4:노, 5:초, 6:파)
#define REG_SIZE_CODE   1       // 40002: 크기 코드 (1:소, 2:중, 3:대)
#define REG_PRESENT     2       // 40003: 확정된 물체가 판 위에 있음 (0/1)
#define REG_R_VAL       3       // 40004: R 값 (판 기준 화이트밸런스 보정 후)
#define REG_G_VAL       4       // 40005: G 값
#define REG_B_VAL       5       // 40006: B 값
#define REG_DETECT_CNT  6       // 40007: 새 물체 감지 누적 횟수
#define REG_RATIO_X100  7       // 40008: 긴변/짧은변 비율 x 100 (예: 157 = 1.57배)
#define REG_COMMAND     8       // 40009: 명령. PLC가 1을 쓰면 배경 재학습 → 진행 중 2 → 완료 0
#define REG_COUNT       9

// LOG   : 매 프레임 상세 로그 — 시리얼 전용
// EVENT : 감지/제거/통신 등 주요 이벤트 — 시리얼 + 웹 '감지 기록'
#define LOG(...) do { if (SERIAL_LOG) Serial.printf(__VA_ARGS__); } while (0)
#define EVENT(...) logEvent(__VA_ARGS__)

// ==========================================
// 3. 영상 / 관심영역(ROI)
// ==========================================
#define IMG_W 160               // QQVGA
#define IMG_H 120
#define IMG_N (IMG_W * IMG_H)

// 분석 영역 (경계 포함). 판 가장자리/나사 등 반사 노이즈를 피하기 위한 여백
const int ROI_X0 = 20, ROI_X1 = 139;
const int ROI_Y0 = 15, ROI_Y1 = 104;
static_assert(ROI_X0 >= 1 && ROI_Y0 >= 1 && ROI_X1 <= IMG_W - 2 && ROI_Y1 <= IMG_H - 2,
              "ROI는 영상 가장자리에서 최소 1px 떨어져야 함 (이웃 픽셀 접근)");

// ==========================================
// 4. 물체 감지(배경 차분) 파라미터
// ==========================================
// 픽셀이 '물체'로 판정되는 조건 (둘 중 하나)
//   (a) 배경 대비 채도(최대-최소) 증가 ≥ CHROMA_THRESH  → 유채색 큐브. 은색판은 무채색이라 반사/그림자에 강함
//   (b) |ΔR|+|ΔG|+|ΔB| ≥ DIFF_THRESH 이면서 그림자가 아님 → 흰색/검정 큐브
const int  DIFF_THRESH       = 60;
const int  CHROMA_THRESH     = 30;
const bool USE_SHADOW_FILTER = true;
const int  SHADOW_MIN_PCT    = 50;   // 배경 밝기의 50~100% 이면서 무채색 변화 → 그림자로 보고 제외

const int MIN_BLOB_AREA = 150;       // 물체로 인정할 최소 덩어리 면적(px). 소형 큐브 ≈ 20x20 = 400px
// 가장 큰 덩어리가 MIN_BLOB_AREA/2 보다 작으면 '빈 판'으로 본다

// ==========================================
// 5. 크기 판별 (회전 무관, 2차 모멘트 기반 긴변/짧은변)
// ==========================================
//   이론값: 소(35/35=1.00), 중(55/35=1.57), 대(85/35=2.43)
//   판 중심에서 먼 물체는 옆면(높이 35mm)이 같이 보여 비율이 ±0.2 정도 흔들림
const float RATIO_SMALL_MAX  = 1.30f; // < 1.30 → 소 (35mm)
const float RATIO_MEDIUM_MAX = 1.95f; // 1.30~1.95 → 중 (55mm), > 1.95 → 대 (85mm)

// ==========================================
// 6. 색상 판별 (판 기준 화이트밸런스 + 픽셀 투표)
// ==========================================
const int   PX_CHROMA_MIN     = 25;    // 유채색 픽셀: 최대-최소 ≥ 25
const float PX_SAT_MIN        = 0.28f; //              그리고 채도(S) ≥ 0.28
const float CHROMA_SHARE_MIN  = 0.35f; // 내부 픽셀 중 유채색이 35% 이상이면 유채색 물체
const float WHITE_BLACK_SPLIT = 0.65f; // 무채색 물체: (평균 밝기 / 판 밝기) ≥ 0.65 → 흰색, 미만 → 검정
// 색상각(Hue) 경계
const float HUE_RED_FROM   = 335.0f;   // 335~360, 0~28 → 빨강
const float HUE_YELLOW_FROM = 28.0f;   // 28~68 → 노랑
const float HUE_GREEN_FROM  = 68.0f;   // 68~165 → 초록
const float HUE_BLUE_FROM   = 165.0f;  // 165~260 → 파랑
const float HUE_BLUE_TO     = 260.0f;  // 260~335 → 기타(보라/분홍) = 판정 불가

// ==========================================
// 7. 안정화 / 상태 전이
// ==========================================
const int           STABLE_FRAMES   = 5;     // 위치·면적이 변하지 않는 연속 프레임 수
const unsigned long STABLE_MIN_MS   = 300;   // 그리고 처음 감지 후 최소 경과 시간
const float         STABLE_MOVE_PX  = 1.5f;  // 프레임 간 중심 이동 허용치(px)
const float         STABLE_AREA_PCT = 0.12f; // 프레임 간 면적 변화 허용치
const int           EMPTY_FRAMES    = 4;     // 안정화 중: 연속 이만큼 판이 비어 있으면 취소

// 물체 제거 판정 — '확정된 물체가 있던 자리(풋프린트)'만 본다
//   예전 방식(ROI 전체에 덩어리가 하나도 없어야 제거)은 물체를 치운 뒤에도
//   작업자 팔·판 반사·조명 얼룩 등 다른 덩어리가 하나만 남아 있으면 제거를 놓쳤다.
const float REMOVE_OCC_MAX = 0.35f;  // 풋프린트 중 전경 픽셀 비율이 35% 미만이면 '비었음'
const int   REMOVE_FRAMES  = 4;      // 연속 이만큼 비어 있으면 '물체 제거'

// 교체 판정 — 빈 프레임 없이 같은 자리에 다른 물체가 놓인 경우 (빠른 교체)
const float         CHANGE_OVERLAP_MIN = 0.5f;   // 현재 덩어리가 풋프린트와 이만큼 겹쳐야 '같은 자리'
const float         CHANGE_AREA_PCT    = 0.25f;  // |면적 차| / 큰 쪽 면적 ≥ 25% → 크기가 다름 (소:중:대 ≈ 1:1.57:2.43)
const int           CHANGE_FRAMES      = 10;     // 이만큼 연속으로 (안정된 상태로) 달라 보여야 교체
const unsigned long CHANGE_MIN_MS      = 600;

// 조명 변화를 따라가도록 배경을 천천히 갱신 (모든 상태에서)
//   배경과 차이가 작은 픽셀만 갱신하므로 물체·그림자·손은 배경에 흡수되지 않는다.
//   → 물체가 오래 놓여 있는 동안 주변 판 밝기가 변해도, 치운 뒤 얼룩(유령 덩어리)이 남지 않음
const int BG_AVG_FRAMES   = 8;    // 배경 학습 시 평균 낼 프레임 수
const int BG_UPDATE_EVERY = 10;   // 이 프레임마다 갱신
const int BG_TRACK_MAX    = 24;   // |ΔR|+|ΔG|+|ΔB| 가 이 이하인 픽셀만 갱신 (DIFF_THRESH·CHROMA_THRESH보다 충분히 작게)

// ==========================================
// 8. AI-Thinker ESP32-CAM 핀 정의
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

// ==========================================
// 9. 전역 상태
// ==========================================
// 빈 스테인리스판 배경 (프레임 평균, PSRAM)
uint8_t *bgR = nullptr;
uint8_t *bgG = nullptr;
uint8_t *bgB = nullptr;
bool isBgReady = false;

// 판(배경) 평균색 → 화이트밸런스 게인 (은색판을 무채색 기준으로 사용)
float plateR = 0, plateG = 0, plateB = 0, plateGray = 128;
float gainR = 1, gainG = 1, gainB = 1;
int gainR8 = 256, gainG8 = 256, gainB8 = 256;   // 고정소수점(x256) 게인 — 픽셀 루프용

// 작업 버퍼 (PSRAM)
uint8_t  *fgRaw = nullptr;   // 0/1 원시 전경
uint8_t  *fgMap = nullptr;   // 0:배경 1:전경 2:다른 덩어리 3:선택 덩어리 가장자리 10+n:선택 덩어리 내부(n=픽셀 분류)
uint16_t *bfsQueue = nullptr;

struct BlobInfo {
  int   area = 0;
  int   seed = -1;
  int   minX = 0, minY = 0, maxX = 0, maxY = 0;
  bool  touchesBorder = false;
  bool  valid = false;           // 면적 충분 + ROI 경계 미접촉
  float cx = 0, cy = 0;          // 중심
  float len = 0, wid = 0;        // 긴변/짧은변 (px)
  float ratio = 0;               // len / wid
  float angleDeg = 0;            // 긴변 방향
  int64_t sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
};

struct ColorInfo {
  uint16_t code = 0;             // 판정 색상 코드 (0 = 판정 불가)
  int   r = 0, g = 0, b = 0;     // 보정 후 평균 RGB (판정 클래스 픽셀 기준)
  int   samples = 0;             // 분류에 사용한 픽셀 수
  int   votes[8] = {0};          // 1:무채-밝음 2:무채-어두움 3:빨 4:노 5:초 6:파 7:기타
  float chromaShare = 0;         // 유채색 픽셀 비율
  float vRatio = 0;              // 무채색 픽셀 평균 밝기 / 판 밝기
};

enum DetState { ST_EMPTY = 0, ST_SETTLING = 1, ST_CONFIRMED = 2 };
const char *STATE_NAMES[] = {"EMPTY", "SETTLING", "CONFIRMED"};
DetState detState = ST_EMPTY;

int emptyStreak = 0;
int stableCount = 0;
unsigned long settleStart = 0;
BlobInfo prevBlob;
bool settleWarned = false;

// 확정된 물체의 위치(풋프린트) — 제거/교체 판정 기준
uint8_t *footMask = nullptr;   // 1 = 확정 당시 물체 픽셀
int      footArea = 0;
BlobInfo confBlob;             // 확정 당시 덩어리

struct FootCheck {
  float occ = 0;               // 풋프린트 중 현재 전경인 픽셀 비율 (1 = 그대로 있음, 0 = 비었음)
  float bestOverlap = 0;       // 현재 최대 덩어리와 풋프린트의 겹침 / 둘 중 작은 면적
};
FootCheck lastFoot;
int removeStreak = 0;          // 풋프린트가 비어 보인 연속 프레임
int changeCount = 0;           // 같은 자리에 다른 물체로 보인 연속 프레임
unsigned long changeStart = 0;
BlobInfo changePrev;

// 안정화 구간 누적값
int   accColorVotes[8];
long  accRgb[8][3];
int   accRgbN[8];
float accRatioSum = 0;
int   accFrames = 0;

// 확정된 최신 측정 결과
int curR = 0, curG = 0, curB = 0;
uint16_t curColorCode = 0;
uint16_t curSizeCode = 0;
float curRatio = 0.0f;
uint16_t detectCount = 0;

uint32_t frameNo = 0;
float fps = 0;
volatile bool relearnRequested = false;

// 카메라 상태
bool cameraOk = false;
uint16_t camPid = 0;
volatile uint32_t camFails = 0;          // esp_camera_fb_get() 실패 누적
volatile unsigned long lastFrameMs = 0;  // 마지막으로 프레임을 받은 시각

// Modbus 레지스터 사본 — 웹 화면 표시용 (Modbus를 끈 상태에서도 '제공될 값'을 보여준다)
volatile uint16_t regMirror[REG_COUNT] = {0};

// Modbus 통신 통계 (이 장치 ID로 온, CRC가 정상인 요청만 집계됨)
volatile uint32_t mbReqCount = 0;        // 수신한 요청
volatile uint32_t mbOkCount = 0;         // 정상 응답
volatile unsigned long mbLastReqMs = 0;
volatile uint8_t  mbLastFc = 0;
volatile uint16_t mbLastAddr = 0, mbLastArg = 0;  // 시작 주소(0-based), 개수(FC06이면 쓴 값)
volatile bool     mbLastOk = false;

// 웹 '감지 기록' 링버퍼
#define EVT_MAX 40
#define EVT_LEN 192
struct EventLine {
  uint32_t seq;
  uint32_t ms;
  char text[EVT_LEN];
};
EventLine evtBuf[EVT_MAX];
volatile uint32_t evtSeq = 0;            // 지금까지 기록된 이벤트 수 (= 마지막 seq)
SemaphoreHandle_t evtMutex = nullptr;

bool webStarted = false;

// 디버그 화면: 왼쪽 카메라+측정 / 오른쪽 전경 마스크 (320x120 RGB565)
#define DBG_W (IMG_W * 2)
#define DBG_H IMG_H
uint8_t *dbgBuf = nullptr;
uint8_t *rawBuf = nullptr;      // 오버레이 없는 원본 프레임 (BMP 다운로드용)
SemaphoreHandle_t dbgMutex = nullptr;
volatile uint32_t dbgFrameId = 0;

struct DebugStatus {
  int state = 0;
  int fgRawCount = 0, fgCount = 0, components = 0;
  int stableCount = 0;
  BlobInfo blob;
  ColorInfo color;
  FootCheck foot;
  int footArea = 0, removeStreak = 0, changeCount = 0;
  float fps = 0;
  uint32_t frameNo = 0;
} dbgStatus;

httpd_handle_t web_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

// ==========================================
// 10. 유틸리티
// ==========================================
static inline int max3i(int a, int b, int c) { return a > b ? (a > c ? a : c) : (b > c ? b : c); }
static inline int min3i(int a, int b, int c) { return a < b ? (a < c ? a : c) : (b < c ? b : c); }
static inline float max3f(float a, float b, float c) { return a > b ? (a > c ? a : c) : (b > c ? b : c); }
static inline float min3f(float a, float b, float c) { return a < b ? (a < c ? a : c) : (b < c ? b : c); }

// RGB565(빅엔디언 바이트 순서) → 8bit RGB
static inline void readPx(const uint8_t *buf, int i, int &r, int &g, int &b) {
  uint16_t p = (buf[i * 2] << 8) | buf[i * 2 + 1];
  int r5 = (p >> 11) & 0x1F, g6 = (p >> 5) & 0x3F, b5 = p & 0x1F;
  r = (r5 << 3) | (r5 >> 2);
  g = (g6 << 2) | (g6 >> 4);
  b = (b5 << 3) | (b5 >> 2);
}

static inline void putPx(uint8_t *buf, int i, uint16_t c) {
  buf[i * 2] = c >> 8;
  buf[i * 2 + 1] = c & 0xFF;
}

static void *allocBig(size_t n) {
  void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = malloc(n);
  return p;
}

// 주요 이벤트: 시리얼(가능할 때) + 웹 '감지 기록'
static void logEvent(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void logEvent(const char *fmt, ...) {
  char line[EVT_LEN];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  if (SERIAL_LOG) Serial.println(line);
  if (!ENABLE_WEB || !evtMutex) return;
  if (xSemaphoreTake(evtMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  EventLine &e = evtBuf[evtSeq % EVT_MAX];
  e.seq = evtSeq + 1;
  e.ms = millis();
  memcpy(e.text, line, sizeof(line));
  evtSeq = e.seq;
  xSemaphoreGive(evtMutex);
}

// Modbus 홀딩 레지스터 쓰기 (웹 표시용 사본도 함께 갱신)
static void setReg(uint16_t addr, uint16_t val) {
  if (addr >= REG_COUNT) return;
  regMirror[addr] = val;
  if (ENABLE_MODBUS) mb.Hreg(addr, val);
}

static inline void mbTask() {
  if (ENABLE_MODBUS) mb.task();
}

// Modbus 요청 콜백 (mb.task() 안에서 호출됨) — 통신 상태 표시용
static Modbus::ResultCode onMbRequest(Modbus::FunctionCode fc, const Modbus::RequestData data) {
  mbReqCount = mbReqCount + 1;
  mbLastReqMs = millis();
  mbLastFc = fc;
  mbLastAddr = data.reg.address;
  mbLastArg = data.regCount;
  mbLastOk = false;            // 정상 처리되면 onMbSuccess에서 true
  return Modbus::EX_SUCCESS;
}

static Modbus::ResultCode onMbSuccess(Modbus::FunctionCode fc, const Modbus::RequestData data) {
  mbOkCount = mbOkCount + 1;
  mbLastOk = true;
  return Modbus::EX_SUCCESS;
}

// Modbus 모드에서는 대기 중에도 PLC 요청에 응답
static void serviceDelay(unsigned long ms) {
  unsigned long t0 = millis();
  do {
    mbTask();
    delay(2);
  } while (millis() - t0 < ms);
}

static void flushFrames(int n) {
  for (int i = 0; i < n; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    mbTask();
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

uint16_t classifySizeByRatio(float ratio) {
  if (ratio <= 0) return 0;
  if (ratio < RATIO_SMALL_MAX) return 1;   // 소 (35×35×35, 비율 약 1.0)
  if (ratio <= RATIO_MEDIUM_MAX) return 2; // 중 (35×35×55, 비율 약 1.57)
  return 3;                                // 대 (35×35×85, 비율 약 2.43)
}

// 보정된 RGB 픽셀의 색상 클래스: 0 = 무채색, 3~6 = 빨/노/초/파, 7 = 기타(보라/분홍)
static uint8_t hueClassOf(float r, float g, float b) {
  float mx = max3f(r, g, b), mn = min3f(r, g, b), d = mx - mn;
  if (d < PX_CHROMA_MIN || d < mx * PX_SAT_MIN) return 0;
  float h;
  if (mx == r)      h = 60.0f * fmodf((g - b) / d, 6.0f);
  else if (mx == g) h = 60.0f * ((b - r) / d + 2.0f);
  else              h = 60.0f * ((r - g) / d + 4.0f);
  if (h < 0) h += 360.0f;
  if (h >= HUE_RED_FROM || h < HUE_YELLOW_FROM) return 3;
  if (h < HUE_GREEN_FROM) return 4;
  if (h < HUE_BLUE_FROM)  return 5;
  if (h < HUE_BLUE_TO)    return 6;
  return 7;
}

// ==========================================
// 11. 배경 학습 (빈 판에서 노출/화이트밸런스 고정 후 여러 프레임 평균)
// ==========================================
void computePlateStats() {
  double sr = 0, sg = 0, sb = 0;
  int n = 0;
  for (int y = ROI_Y0; y <= ROI_Y1; y++) {
    for (int x = ROI_X0; x <= ROI_X1; x++) {
      int i = y * IMG_W + x;
      sr += bgR[i]; sg += bgG[i]; sb += bgB[i]; n++;
    }
  }
  plateR = sr / n; plateG = sg / n; plateB = sb / n;
  plateGray = (plateR + plateG + plateB) / 3.0f;
  gainR = plateR > 1 ? plateGray / plateR : 1;
  gainG = plateG > 1 ? plateGray / plateG : 1;
  gainB = plateB > 1 ? plateGray / plateB : 1;
  gainR8 = lroundf(gainR * 256);
  gainG8 = lroundf(gainG * 256);
  gainB8 = lroundf(gainB * 256);
}

// 배경 갱신 한 걸음: 차이의 1/8, 최소 1 단계
//   (단순 /8 은 채널당 차이가 8 미만이면 0이 되어, 작은 차이만 허용하는 갱신에서는 영영 따라가지 못한다)
static inline int bgStep(int d) {
  if (d > 0) return max(1, d / 8);
  if (d < 0) return min(-1, d / 8);
  return 0;
}

// 판 기준 화이트밸런스 적용 후의 채도(최대-최소)
//   센서 색조(예: AWB 고정 시 녹색 기운) 때문에 판 자체 채도가 40 가까이 나오면
//   주황빛으로 찍히는 빨간 큐브와 구분이 약해지므로, 보정 공간에서 비교한다.
static inline int wbChroma(int r, int g, int b) {
  int rc = (r * gainR8) >> 8, gc = (g * gainG8) >> 8, bc = (b * gainB8) >> 8;
  return max3i(rc, gc, bc) - min3i(rc, gc, bc);
}

void resetDetectionState() {
  detState = ST_EMPTY;
  emptyStreak = 0;
  stableCount = 0;
  accFrames = 0;
  footArea = 0;
  removeStreak = 0;
  changeCount = 0;
  lastFoot = FootCheck();
  setReg(REG_PRESENT, 0);
}

void learnBackground(bool readapt) {
  sensor_t *s = esp_camera_sensor_get();

  if (readapt) {
    // 자동 노출/게인/화이트밸런스로 현재 조명에 다시 맞춘다
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) flushFrames(1);
  }

  // 고정: 물체를 놓아도 화면 전체 밝기/색조가 변하지 않게 한다 (배경 차분의 전제 조건)
  //   — 자동 노출이 켜져 있으면 흰/검 물체를 놓는 순간 판 전체가 어두워지거나 밝아져서
  //     수천 픽셀이 '변화'로 잡힌다.
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_gain_ctrl(s, 0);
  flushFrames(15);

  uint16_t *acc = (uint16_t *)allocBig(IMG_N * 3 * sizeof(uint16_t));
  if (!acc) {
    EVENT("[BG] 메모리 부족 — 배경 학습 실패");
    return;
  }
  memset(acc, 0, IMG_N * 3 * sizeof(uint16_t));
  int got = 0;
  for (int k = 0; k < BG_AVG_FRAMES; k++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) continue;
    if (fb->width == IMG_W && fb->height == IMG_H && fb->format == PIXFORMAT_RGB565) {
      for (int i = 0; i < IMG_N; i++) {
        int r, g, b;
        readPx(fb->buf, i, r, g, b);
        acc[i * 3] += r; acc[i * 3 + 1] += g; acc[i * 3 + 2] += b;
      }
      got++;
    }
    esp_camera_fb_return(fb);
    mbTask();
  }
  if (got > 0) {
    for (int i = 0; i < IMG_N; i++) {
      bgR[i] = (acc[i * 3] + got / 2) / got;
      bgG[i] = (acc[i * 3 + 1] + got / 2) / got;
      bgB[i] = (acc[i * 3 + 2] + got / 2) / got;
    }
    computePlateStats();
    isBgReady = true;
  }
  free(acc);
  resetDetectionState();

  if (got > 0) {
    EVENT("[BG] 배경 학습 완료 (%d프레임 평균) 판 RGB=(%.0f,%.0f,%.0f) 게인 R%.2f G%.2f B%.2f",
          got, plateR, plateG, plateB, gainR, gainG, gainB);
  } else {
    EVENT("[BG] 배경 학습 실패 — 카메라 프레임을 받지 못함");
  }
}

// ==========================================
// 12. 연결 요소(덩어리) 탐색 — 8방향 BFS
// ==========================================
// fgMap에서 값이 from인 픽셀을 seed부터 채워 to로 바꾸고 통계를 낸다.
// 반환 후 bfsQueue[0..area-1]에 해당 덩어리 픽셀 목록이 남는다.
static int floodComponent(int seed, uint8_t from, uint8_t to, BlobInfo *st) {
  static const int DX[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static const int DY[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  int head = 0, tail = 0;
  bfsQueue[tail++] = seed;
  fgMap[seed] = to;
  if (st) {
    *st = BlobInfo();
    st->seed = seed;
    st->minX = IMG_W; st->minY = IMG_H;
  }
  while (head < tail) {
    int i = bfsQueue[head++];
    int x = i % IMG_W, y = i / IMG_W;
    if (st) {
      st->area++;
      st->sx += x; st->sy += y;
      st->sxx += x * x; st->syy += y * y; st->sxy += x * y;
      if (x < st->minX) st->minX = x;
      if (x > st->maxX) st->maxX = x;
      if (y < st->minY) st->minY = y;
      if (y > st->maxY) st->maxY = y;
      if (x <= ROI_X0 + 1 || x >= ROI_X1 - 1 || y <= ROI_Y0 + 1 || y >= ROI_Y1 - 1) st->touchesBorder = true;
    }
    // ROI 밖은 fgMap이 항상 0이므로 별도 경계 검사가 필요 없다 (static_assert 참고)
    for (int k = 0; k < 8; k++) {
      int n = (y + DY[k]) * IMG_W + (x + DX[k]);
      if (fgMap[n] == from) {
        fgMap[n] = to;
        bfsQueue[tail++] = n;
      }
    }
  }
  return tail;
}

// 2차 모멘트(관성) → 회전과 무관한 긴변/짧은변
//   균일한 직사각형 L×W에서 긴축 방향 분산 = L²/12 → L = √(12·λ1)
static void measureBlob(BlobInfo &b) {
  if (b.area <= 0) return;
  double n = b.area;
  double cx = b.sx / n, cy = b.sy / n;
  double mu20 = b.sxx / n - cx * cx + 1.0 / 12.0;  // +1/12: 픽셀 크기 보정
  double mu02 = b.syy / n - cy * cy + 1.0 / 12.0;
  double mu11 = b.sxy / n - cx * cy;
  double common = sqrt(((mu20 - mu02) / 2) * ((mu20 - mu02) / 2) + mu11 * mu11);
  double l1 = (mu20 + mu02) / 2 + common;
  double l2 = (mu20 + mu02) / 2 - common;
  if (l2 < 0.01) l2 = 0.01;
  b.cx = cx; b.cy = cy;
  b.len = sqrt(12.0 * l1);
  b.wid = sqrt(12.0 * l2);
  b.ratio = b.len / b.wid;
  b.angleDeg = 0.5 * atan2(2 * mu11, mu20 - mu02) * 180.0 / M_PI;
}

// 선택 덩어리(fgMap==3)의 내부 픽셀을 분류해 색상 판정
static void classifyBlobColor(const uint8_t *buf, int count, ColorInfo &ci) {
  ci = ColorInfo();
  int interior = 0;
  for (int k = 0; k < count; k++) {
    int i = bfsQueue[k];
    if (fgMap[i - 1] >= 3 && fgMap[i + 1] >= 3 && fgMap[i - IMG_W] >= 3 && fgMap[i + IMG_W] >= 3) interior++;
  }
  // 가장자리는 판 반사/경계 혼합색이 섞이므로 내부만 사용 (너무 작으면 전체 사용)
  bool useAll = interior < 20;

  float sum[8][3] = {{0}};
  float achroVSum = 0;
  int achroN = 0;
  for (int k = 0; k < count; k++) {
    int i = bfsQueue[k];
    bool isInterior = fgMap[i - 1] >= 3 && fgMap[i + 1] >= 3 && fgMap[i - IMG_W] >= 3 && fgMap[i + IMG_W] >= 3;
    if (!useAll && !isInterior) continue;
    int r8, g8, b8;
    readPx(buf, i, r8, g8, b8);
    float r = min(255.0f, r8 * gainR);
    float g = min(255.0f, g8 * gainG);
    float b = min(255.0f, b8 * gainB);
    uint8_t cls = hueClassOf(r, g, b);
    if (cls == 0) {
      float v = max3f(r, g, b);
      cls = (v >= plateGray * WHITE_BLACK_SPLIT) ? 1 : 2;
      achroVSum += v;
      achroN++;
    }
    ci.votes[cls]++;
    sum[cls][0] += r; sum[cls][1] += g; sum[cls][2] += b;
    ci.samples++;
    fgMap[i] = 10 + cls;
  }
  if (ci.samples == 0) return;

  int chromaN = ci.votes[3] + ci.votes[4] + ci.votes[5] + ci.votes[6] + ci.votes[7];
  ci.chromaShare = (float)chromaN / ci.samples;
  ci.vRatio = achroN > 0 ? (achroVSum / achroN) / plateGray : 0;

  float sr, sg, sb;
  int sn;
  if (ci.chromaShare >= CHROMA_SHARE_MIN) {
    int best = 3;
    for (int c = 4; c <= 7; c++) if (ci.votes[c] > ci.votes[best]) best = c;
    ci.code = (best == 7) ? 0 : best;
    sr = sum[best][0]; sg = sum[best][1]; sb = sum[best][2]; sn = ci.votes[best];
  } else {
    ci.code = (ci.vRatio >= WHITE_BLACK_SPLIT) ? 1 : 2;
    sr = sum[1][0] + sum[2][0]; sg = sum[1][1] + sum[2][1]; sb = sum[1][2] + sum[2][2];
    sn = ci.votes[1] + ci.votes[2];
  }
  if (sn > 0) { ci.r = sr / sn; ci.g = sg / sn; ci.b = sb / sn; }
}

// ==========================================
// 13. 디버그 화면 그리기
// ==========================================
static void drawLineLeft(int x0, int y0, int x1, int y1, uint16_t c) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (int guard = 0; guard < 1000; guard++) {
    if (x0 >= 0 && x0 < IMG_W && y0 >= 0 && y0 < IMG_H) putPx(dbgBuf, y0 * DBG_W + x0, c);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

static uint16_t maskColor(int i, int x, int y) {
  if (x < ROI_X0 || x > ROI_X1 || y < ROI_Y0 || y > ROI_Y1) return 0x0841;
  uint8_t v = fgMap[i];
  if (v >= 10) {
    switch (v - 10) {
      case 1: return 0xFFFF;  // 무채색-밝음
      case 2: return 0x4A49;  // 무채색-어두움
      case 3: return 0xF800;  // 빨강
      case 4: return 0xFFE0;  // 노랑
      case 5: return 0x07E0;  // 초록
      case 6: return 0x041F;  // 파랑
      default: return 0xF81F; // 기타
    }
  }
  if (v == 3) return 0x07FF;      // 선택 덩어리 가장자리 (색 판정 제외)
  if (v == 2) return 0x8410;      // 다른 덩어리
  if (detState == ST_CONFIRMED && footMask[i]) return 0x7800;  // 확정 위치 중 비어 보이는 부분
  if (fgRaw[i]) return 0x2945;    // 필터로 제거된 노이즈
  return 0x0000;
}

static void renderDebug(const camera_fb_t *fb, const BlobInfo &best, const ColorInfo &ci,
                        int fgRawCount, int fgCount, int components) {
  if (!dbgBuf || !dbgMutex) return;
  if (xSemaphoreTake(dbgMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

  memcpy(rawBuf, fb->buf, IMG_N * 2);
  for (int y = 0; y < IMG_H; y++) {
    memcpy(dbgBuf + (y * DBG_W) * 2, fb->buf + (y * IMG_W) * 2, IMG_W * 2);
    for (int x = 0; x < IMG_W; x++) {
      putPx(dbgBuf, y * DBG_W + IMG_W + x, maskColor(y * IMG_W + x, x, y));
    }
  }

  // ROI 테두리
  drawLineLeft(ROI_X0, ROI_Y0, ROI_X1, ROI_Y0, 0x8410);
  drawLineLeft(ROI_X1, ROI_Y0, ROI_X1, ROI_Y1, 0x8410);
  drawLineLeft(ROI_X1, ROI_Y1, ROI_X0, ROI_Y1, 0x8410);
  drawLineLeft(ROI_X0, ROI_Y1, ROI_X0, ROI_Y0, 0x8410);

  // 측정된 회전 사각형 (노랑=안정화 중, 초록=확정, 빨강=무효/경계 접촉)
  if (best.area >= MIN_BLOB_AREA / 2) {
    uint16_t c = !best.valid ? 0xF800 : (detState == ST_CONFIRMED ? 0x07E0 : 0xFFE0);
    float a = best.angleDeg * M_PI / 180.0f;
    float ux = cosf(a), uy = sinf(a), vx = -uy, vy = ux;
    float hl = best.len / 2, hw = best.wid / 2;
    int px[4], py[4];
    const int SL[4] = {1, 1, -1, -1}, SW[4] = {1, -1, -1, 1};
    for (int k = 0; k < 4; k++) {
      px[k] = lroundf(best.cx + SL[k] * hl * ux + SW[k] * hw * vx);
      py[k] = lroundf(best.cy + SL[k] * hl * uy + SW[k] * hw * vy);
    }
    for (int k = 0; k < 4; k++) drawLineLeft(px[k], py[k], px[(k + 1) % 4], py[(k + 1) % 4], c);
    int cx = lroundf(best.cx), cy = lroundf(best.cy);
    drawLineLeft(cx - 2, cy, cx + 2, cy, c);
    drawLineLeft(cx, cy - 2, cx, cy + 2, c);
  }

  dbgStatus.state = detState;
  dbgStatus.fgRawCount = fgRawCount;
  dbgStatus.fgCount = fgCount;
  dbgStatus.components = components;
  dbgStatus.stableCount = stableCount;
  dbgStatus.blob = best;
  dbgStatus.color = ci;
  dbgStatus.foot = lastFoot;
  dbgStatus.footArea = detState == ST_CONFIRMED ? footArea : 0;
  dbgStatus.removeStreak = removeStreak;
  dbgStatus.changeCount = changeCount;
  dbgStatus.fps = fps;
  dbgStatus.frameNo = frameNo;
  dbgFrameId = dbgFrameId + 1;
  xSemaphoreGive(dbgMutex);
}

// ==========================================
// 14. 결과 확정 / 상태 머신
// ==========================================
static void resetAccumulators() {
  memset(accColorVotes, 0, sizeof(accColorVotes));
  memset(accRgb, 0, sizeof(accRgb));
  memset(accRgbN, 0, sizeof(accRgbN));
  accRatioSum = 0;
  accFrames = 0;
}

static void accumulate(const BlobInfo &b, const ColorInfo &ci) {
  accColorVotes[ci.code]++;
  accRgb[ci.code][0] += ci.r; accRgb[ci.code][1] += ci.g; accRgb[ci.code][2] += ci.b;
  accRgbN[ci.code]++;
  accRatioSum += b.ratio;
  accFrames++;
}

static bool isSameBlob(const BlobInfo &a, const BlobInfo &b) {
  float areaTol = max(8.0f, a.area * STABLE_AREA_PCT);
  return fabsf(a.cx - b.cx) <= STABLE_MOVE_PX && fabsf(a.cy - b.cy) <= STABLE_MOVE_PX &&
         fabsf((float)(a.area - b.area)) <= areaTol;
}

// 확정 순간의 선택 덩어리(fgMap >= 3)를 풋프린트로 저장
static void saveFootprint(const BlobInfo &b) {
  memset(footMask, 0, IMG_N);
  footArea = 0;
  for (int y = ROI_Y0; y <= ROI_Y1; y++) {
    for (int x = ROI_X0; x <= ROI_X1; x++) {
      int i = y * IMG_W + x;
      if (fgMap[i] >= 3) { footMask[i] = 1; footArea++; }
    }
  }
  confBlob = b;
  removeStreak = 0;
  changeCount = 0;
}

// 풋프린트가 지금도 전경(배경과 다름)인지 — ROI의 다른 곳에 무엇이 있든 영향받지 않는다
static FootCheck checkFootprint(const BlobInfo &best) {
  FootCheck fc;
  if (footArea <= 0) return fc;
  int fg = 0, inBest = 0;
  for (int y = ROI_Y0; y <= ROI_Y1; y++) {
    for (int x = ROI_X0; x <= ROI_X1; x++) {
      int i = y * IMG_W + x;
      if (!footMask[i] || !fgMap[i]) continue;
      fg++;
      if (fgMap[i] >= 3) inBest++;      // 현재 최대 덩어리에 속함
    }
  }
  fc.occ = (float)fg / footArea;
  int denom = min(footArea, best.area);
  fc.bestOverlap = denom > 0 ? (float)inBest / denom : 0;
  return fc;
}

static bool isAchromaticCode(uint16_t c) { return c == 1 || c == 2; }

// 같은 자리의 현재 물체가 확정된 물체와 크기 또는 색이 다른가
static bool isDifferentObject(const BlobInfo &b, const ColorInfo &ci, const FootCheck &fc) {
  if (!b.valid || fc.bestOverlap < CHANGE_OVERLAP_MIN) return false;
  int bigger = max(b.area, confBlob.area);
  bool areaChanged = bigger > 0 && abs(b.area - confBlob.area) >= bigger * CHANGE_AREA_PCT;
  // 흰↔검은 그림자에 따라 뒤집힐 수 있어 교체 근거로 쓰지 않는다
  bool colorChanged = ci.code != 0 && ci.code != curColorCode &&
                      !(isAchromaticCode(ci.code) && isAchromaticCode(curColorCode));
  return areaChanged || colorChanged;
}

static void beginSettling(const BlobInfo &best, const ColorInfo &ci) {
  detState = ST_SETTLING;
  settleStart = millis();
  settleWarned = false;
  stableCount = 1;
  prevBlob = best;
  resetAccumulators();
  accumulate(best, ci);
}

static void markRemoved() {
  detState = ST_EMPTY;
  removeStreak = 0;
  changeCount = 0;
  setReg(REG_PRESENT, 0);
}

static bool tryConfirm() {
  int best = 0;
  for (int c = 1; c <= 6; c++) if (accColorVotes[c] > accColorVotes[best]) best = c;
  if (best == 0 || accFrames == 0) return false;

  float ratio = accRatioSum / accFrames;
  uint16_t size = classifySizeByRatio(ratio);
  if (size == 0) return false;

  curColorCode = best;
  curSizeCode = size;
  curRatio = ratio;
  curR = accRgb[best][0] / accRgbN[best];
  curG = accRgb[best][1] / accRgbN[best];
  curB = accRgb[best][2] / accRgbN[best];
  detectCount++;

  setReg(REG_COLOR_CODE, curColorCode);
  setReg(REG_R_VAL, (uint16_t)curR);
  setReg(REG_G_VAL, (uint16_t)curG);
  setReg(REG_B_VAL, (uint16_t)curB);
  setReg(REG_DETECT_CNT, detectCount);
  setReg(REG_SIZE_CODE, curSizeCode);
  setReg(REG_RATIO_X100, (uint16_t)lroundf(curRatio * 100));
  setReg(REG_PRESENT, 1);
  return true;
}

static void updateState(const BlobInfo &best, const ColorInfo &ci, const FootCheck &fc) {
  bool emptyFrame = best.area < MIN_BLOB_AREA / 2;
  emptyStreak = emptyFrame ? min(emptyStreak + 1, 1000000) : 0;

  switch (detState) {
    case ST_EMPTY:
      if (best.valid) {
        beginSettling(best, ci);
        LOG("[DBG] 물체 진입: 면적=%d 중심=(%.0f,%.0f)\n", best.area, best.cx, best.cy);
      }
      break;

    case ST_SETTLING:
      if (emptyStreak >= EMPTY_FRAMES) {
        detState = ST_EMPTY;
        LOG("[DBG] 안정화 전에 물체가 사라짐 → 취소\n");
        break;
      }
      if (!best.valid) {           // 손이 같이 잡히거나 ROI 경계에 걸침 → 처음부터 다시
        stableCount = 0;
        break;
      }
      if (stableCount > 0 && isSameBlob(prevBlob, best)) {
        stableCount++;
        accumulate(best, ci);
      } else {
        stableCount = 1;
        resetAccumulators();
        accumulate(best, ci);
      }
      prevBlob = best;
      LOG("[DBG] 안정화 %d/%d 면적=%d 긴변=%.1f 짧은변=%.1f 비율=%.2f 각도=%.0f 색=%d(%s) 유채%.0f%% 밝기비%.2f\n",
          stableCount, STABLE_FRAMES, best.area, best.len, best.wid, best.ratio, best.angleDeg,
          ci.code, getColorName(ci.code), ci.chromaShare * 100, ci.vRatio);

      if (stableCount >= STABLE_FRAMES && millis() - settleStart >= STABLE_MIN_MS) {
        if (tryConfirm()) {
          detState = ST_CONFIRMED;
          saveFootprint(best);
          EVENT("[#%u] 색상 %u:%s (R:%d G:%d B:%d) | 크기 %u:%s (변 비율: %.2f)",
                detectCount, curColorCode, getColorName(curColorCode), curR, curG, curB,
                curSizeCode, getSizeName(curSizeCode), curRatio);
        } else {
          if (!settleWarned) {
            EVENT("색상 판정 불가 (보라/분홍 계열?) — 계속 관찰");
            settleWarned = true;
          }
          stableCount = 0;
        }
      }
      break;

    case ST_CONFIRMED: {
      // (1) 제거: 확정된 자리가 배경으로 돌아왔는가 (다른 곳의 덩어리는 무시)
      //     손이 물체를 덮고 있는 동안은 그 자리가 전경이라 제거로 보지 않는다
      bool vacant = emptyFrame || fc.occ < REMOVE_OCC_MAX;
      removeStreak = vacant ? removeStreak + 1 : 0;
      if (removeStreak >= REMOVE_FRAMES) {
        EVENT("물체 제거됨 (확정 위치 점유율 %.0f%%, 남은 최대 덩어리 면적=%d)", fc.occ * 100, best.area);
        markRemoved();
        break;
      }

      // (2) 교체: 판이 비는 순간 없이 같은 자리에 크기/색이 다른 물체가 안정적으로 놓임
      if (isDifferentObject(best, ci, fc)) {
        if (changeCount > 0 && isSameBlob(changePrev, best)) {
          changeCount++;
        } else {
          changeCount = 1;
          changeStart = millis();
        }
        changePrev = best;
        if (changeCount >= CHANGE_FRAMES && millis() - changeStart >= CHANGE_MIN_MS) {
          EVENT("다른 물체로 교체됨 (면적 %d→%d, 색 %s→%s) → 제거 처리 후 다시 판정",
                confBlob.area, best.area, getColorName(curColorCode), getColorName(ci.code));
          markRemoved();
          beginSettling(best, ci);
        }
      } else {
        changeCount = 0;
      }
      break;
    }
  }
}

// ==========================================
// 15. 프레임 분석
// ==========================================
void processFrame(camera_fb_t *fb) {
  if (!isBgReady || !fgRaw || !fgMap || !bfsQueue) return;
  if (fb->width != IMG_W || fb->height != IMG_H || fb->format != PIXFORMAT_RGB565) return;
  const uint8_t *buf = fb->buf;
  frameNo++;

  bool doBgUpdate = (frameNo % BG_UPDATE_EVERY) == 0;

  // 1단계: 배경 차분 → 원시 전경 마스크
  memset(fgRaw, 0, IMG_N);
  int fgRawCount = 0;
  for (int y = ROI_Y0; y <= ROI_Y1; y++) {
    for (int x = ROI_X0; x <= ROI_X1; x++) {
      int i = y * IMG_W + x;
      int r, g, b;
      readPx(buf, i, r, g, b);
      int br = bgR[i], bg = bgG[i], bb = bgB[i];

      int dSum = abs(r - br) + abs(g - bg) + abs(b - bb);
      int curV = max3i(r, g, b), curC = wbChroma(r, g, b);
      int bgV = max3i(br, bg, bb), bgC = wbChroma(br, bg, bb);

      bool chromaFg = (curC - bgC) >= CHROMA_THRESH;
      bool shadow = USE_SHADOW_FILTER && curV < bgV && curV * 100 >= bgV * SHADOW_MIN_PCT &&
                    abs(curC - bgC) < CHROMA_THRESH;
      bool isFg = chromaFg || (dSum >= DIFF_THRESH && !shadow);

      if (isFg) {
        fgRaw[i] = 1;
        fgRawCount++;
      } else if (doBgUpdate && dSum <= BG_TRACK_MAX) {
        bgR[i] = br + bgStep(r - br);
        bgG[i] = bg + bgStep(g - bg);
        bgB[i] = bb + bgStep(b - bb);
      }
    }
  }
  if (doBgUpdate) computePlateStats();

  // 2단계: 3x3 다수결 필터 — 흩어진 노이즈 픽셀 제거, 작은 구멍 메우기 (크기는 유지)
  memset(fgMap, 0, IMG_N);
  int fgCount = 0;
  for (int y = ROI_Y0; y <= ROI_Y1; y++) {
    for (int x = ROI_X0; x <= ROI_X1; x++) {
      int i = y * IMG_W + x;
      int s = fgRaw[i - IMG_W - 1] + fgRaw[i - IMG_W] + fgRaw[i - IMG_W + 1] +
              fgRaw[i - 1] + fgRaw[i] + fgRaw[i + 1] +
              fgRaw[i + IMG_W - 1] + fgRaw[i + IMG_W] + fgRaw[i + IMG_W + 1];
      if (s >= 5) { fgMap[i] = 1; fgCount++; }
    }
  }

  // 3단계: 연결 요소 중 가장 큰 덩어리 선택
  //   (기존 방식은 모든 변화 픽셀의 min/max로 박스를 만들어 노이즈 1픽셀에도 박스가 ROI 전체로 커졌음)
  BlobInfo best, tmp;
  int components = 0;
  for (int y = ROI_Y0; y <= ROI_Y1; y++) {
    for (int x = ROI_X0; x <= ROI_X1; x++) {
      int i = y * IMG_W + x;
      if (fgMap[i] != 1) continue;
      floodComponent(i, 1, 2, &tmp);
      components++;
      if (tmp.area > best.area) best = tmp;
    }
  }

  // 4단계: 크기(회전 무관) + 색상
  ColorInfo ci;
  if (best.area > 0) {
    measureBlob(best);
    best.valid = best.area >= MIN_BLOB_AREA && !best.touchesBorder;
    int count = floodComponent(best.seed, 2, 3, nullptr);
    if (best.valid) classifyBlobColor(buf, count, ci);
  }

  // 5단계: 확정된 물체 자리 점검 → 상태 머신 (안정화 → 확정 → 제거/교체)
  lastFoot = detState == ST_CONFIRMED ? checkFootprint(best) : FootCheck();
  updateState(best, ci, lastFoot);

  if (ENABLE_WEB) renderDebug(fb, best, ci, fgRawCount, fgCount, components);
}

// ==========================================
// 16. 웹 디버그 서버 (포트 80: 페이지/상태/캡처, 포트 81: 스트림)
// ==========================================
static const char INDEX_HTML[] = R"rawliteral(<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM 큐브 판별</title>
<style>
:root{--bg:#111316;--card:#1b1e23;--line:#2d3138;--fg:#e8eaed;--mute:#9aa0a6;--ok:#3ecf6e;--warn:#f5b83d;--bad:#ff6b6b;--acc:#7cb8ff}
*{box-sizing:border-box}
body{margin:0;font:14px/1.45 system-ui,"Segoe UI","Malgun Gothic",sans-serif;background:var(--bg);color:var(--fg)}
header{display:flex;flex-wrap:wrap;gap:6px 14px;align-items:center;padding:12px 16px;border-bottom:1px solid var(--line)}
h1{font-size:17px;margin:0;margin-right:auto}
h2{font-size:13px;margin:0 0 10px;color:var(--mute);font-weight:600;display:flex;gap:8px;align-items:center;flex-wrap:wrap}
main{display:grid;gap:12px;padding:12px 16px;grid-template-columns:repeat(auto-fit,minmax(min(100%,460px),1fr));max-width:1400px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px;min-width:0}
.wide{grid-column:1/-1}
img{width:100%;max-width:960px;aspect-ratio:8/3;image-rendering:pixelated;border-radius:6px;background:#000;display:block}
.pill{display:inline-block;padding:2px 10px;border-radius:999px;font-size:12px;font-weight:600;background:#30343b;color:var(--fg)}
.pill.ok{background:#16351f;color:var(--ok)}.pill.warn{background:#3b2f14;color:var(--warn)}.pill.bad{background:#3d1b1b;color:var(--bad)}
.result{display:flex;gap:14px;align-items:center;margin-bottom:12px}
.swatch{width:60px;height:60px;border-radius:10px;border:1px solid #555;flex:none;background:repeating-linear-gradient(45deg,#2a2d33 0 6px,#22252a 6px 12px)}
.big{font-size:22px;font-weight:700}
.sub{color:var(--mute);font-size:13px}
dl.kv{display:grid;grid-template-columns:max-content 1fr;gap:5px 14px;margin:0}
dl.kv dt{color:var(--mute)}dl.kv dd{margin:0;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}
pre{margin:0;background:#0b0c0e;border-radius:6px;padding:8px 10px;font:12.5px/1.55 ui-monospace,Consolas,monospace;white-space:pre-wrap;overflow-wrap:anywhere}
#log{height:320px;overflow:auto}
.tbl{overflow-x:auto}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums;white-space:nowrap}
th,td{text-align:left;padding:6px 10px;border-bottom:1px solid var(--line)}
th{color:var(--mute);font-weight:600;font-size:12px}
td.v{font:700 14px ui-monospace,Consolas,monospace;text-align:right}
td.h{color:var(--mute);font:12px ui-monospace,Consolas,monospace}
tr td{transition:background 1.5s}tr.flash td{transition:none;background:#2c4220}
@media(max-width:600px){.hx{display:none}main{padding:12px}}
.note{color:var(--mute);font-size:12px;margin:8px 0 0}
button{background:#2b3440;color:var(--fg);border:1px solid #3c4756;border-radius:6px;padding:6px 12px;cursor:pointer;font:inherit}
button:hover{background:#344050}
a{color:var(--acc)}
.row{display:flex;flex-wrap:wrap;gap:8px 14px;align-items:center;margin-top:10px}
.legend{color:var(--mute);font-size:12px;margin-top:8px}
summary{cursor:pointer;color:var(--mute);font-weight:600;font-size:13px}
details[open] summary{margin-bottom:10px}
</style></head><body>
<header><h1>ESP32-CAM 큐브 판별</h1><span id="mode" class="pill"></span><span id="conn" class="pill">연결 중…</span></header>
<main>
<section class="card wide">
 <h2>카메라 영상 <span class="sub">왼쪽: 카메라 + 측정 · 오른쪽: 전경 마스크</span></h2>
 <img id="s" alt="카메라 스트림">
 <div class="legend">사각형 — 노랑: 안정화 중, 초록: 확정, 빨강: 무효(ROI 경계 접촉). 마스크 — 픽셀별 색 분류, 하늘색: 가장자리(판정 제외), 회색: 다른 덩어리, 진한 빨강: 확정 위치 중 비어 보이는 부분, 어두운 점: 제거된 노이즈</div>
 <div class="row"><button onclick="relearn()">배경 재학습</button>
 <a href="/capture" target="_blank">캡처(JPG)</a><a href="/raw.bmp" target="_blank">원본(BMP)</a><a href="/status" target="_blank">상태(JSON)</a></div>
</section>
<section class="card">
 <h2>감지된 물체 <span id="state" class="pill"></span></h2>
 <div class="result"><div id="sw" class="swatch"></div><div><div id="res" class="big">아직 감지 없음</div><div id="resSub" class="sub"></div></div></div>
 <dl class="kv" id="nowKv"></dl>
</section>
<section class="card">
 <h2>감지 기록 <span class="sub">시스템 로그 · 최근 이벤트</span></h2>
 <pre id="log"></pre>
</section>
<section class="card wide">
 <h2>Modbus RTU 홀딩 레지스터 <span id="mbPill" class="pill"></span><span id="mbCfg" class="sub"></span></h2>
 <div class="tbl"><table>
 <thead><tr><th>주소</th><th>오프셋</th><th>항목</th><th style="text-align:right">값</th><th class="hx">HEX</th><th>접근</th><th>의미</th></tr></thead>
 <tbody id="regs"></tbody></table></div>
 <p class="note" id="mbNote">FC03(Read Holding Registers)로 40001부터 9개를 한 번에 읽으면 됩니다. 오프셋은 0부터 세는 주소입니다(PLC 설정에 따라 사용). 40009에 1을 쓰면(FC06) 배경을 다시 학습합니다 → 진행 중 2 → 완료 0.</p>
</section>
<section class="card">
 <h2>통신 상태</h2>
 <dl class="kv" id="mbKv"></dl>
 <p class="note" id="mbHint"></p>
</section>
<section class="card">
 <h2>카메라 · 시스템</h2>
 <dl class="kv" id="sysKv"></dl>
</section>
<details class="card wide" id="det"><summary>상세 측정값 (JSON)</summary><pre id="st"></pre></details>
</main>
<script>
const $=id=>document.getElementById(id);
const COLORS=['미감지','흰색','검정색','빨간색','노란색','초록색','파란색'];
const SW=['','#f2f2f2','#161616','#e53935','#fdd835','#43a047','#1e88e5'];
const SIZES=['미판별','소(35mm)','중(55mm)','대(85mm)'];
const STATES={EMPTY:['판 비어 있음',''],SETTLING:['안정화 중','warn'],CONFIRMED:['확정','ok']};
const PIDS={0x26:'OV2640',0x3660:'OV3660',0x5640:'OV5640',0x77:'OV7725'};
const FCN={1:'Read Coils',2:'Read Discrete Inputs',3:'Read Holding Registers',4:'Read Input Registers',5:'Write Single Coil',6:'Write Single Register',15:'Write Multiple Coils',16:'Write Multiple Registers',22:'Mask Write Register',23:'Read/Write Multiple Registers'};
// 펌웨어의 REG_* 순서(40001~)와 같아야 한다
const REG_PRESENT=2;
const REGS=[
 ['색상 코드',v=>(COLORS[v]??'?')+' — 1흰 2검 3빨 4노 5초 6파'],
 ['크기 코드',v=>(SIZES[v]??'?')+' — 1소 2중 3대'],
 ['물체 있음',v=>v?'1 = 판 위에 있음':'0 = 없음 (제거됨)'],
 ['R 값',v=>'판 기준 화이트밸런스 보정 후 평균'],
 ['G 값',v=>'〃'],
 ['B 값',v=>'〃'],
 ['감지 누적 횟수',v=>'새 물체가 확정될 때마다 +1 (값이 바뀌면 새 데이터)'],
 ['변 비율 ×100',v=>'긴변/짧은변 = '+(v/100).toFixed(2)],
 ['명령',v=>({0:'0 = 대기',1:'1 = 배경 재학습 요청',2:'2 = 재학습 진행 중'})[v]??String(v)],
];
const tb=$('regs');
REGS.forEach((r,i)=>{const tr=document.createElement('tr');
 tr.innerHTML=`<td>${40001+i}</td><td>${i}</td><td>${r[0]}</td><td class="v">-</td><td class="h hx">-</td><td>${i==8?'읽기/쓰기':'읽기'}</td><td class="m"></td>`;tb.appendChild(tr);});
let prevRegs=null,lastOk=0;
const pill=(el,t,c)=>{el.textContent=t;el.className='pill '+(c||'');};
const kv=(el,rows)=>{el.innerHTML=rows.map(([k,v])=>`<dt>${k}</dt><dd>${v}</dd>`).join('');};
function ago(ms){if(ms<0)return '없음';if(ms<1000)return ms+'ms 전';if(ms<60000)return (ms/1000).toFixed(1)+'초 전';return Math.floor(ms/60000)+'분 '+Math.floor(ms%60000/1000)+'초 전';}
function upt(ms){const s=Math.floor(ms/1000);return Math.floor(s/3600)+'시간 '+Math.floor(s%3600/60)+'분 '+s%60+'초';}
function regRange(fc,a,n){
 if(fc==6)return `${40001+a} ← ${n}`;
 if(fc==3||fc==16||fc==23)return n>1?`${40001+a}~${40000+a+n} (${n}개)`:`${40001+a}`;
 if(fc==4)return n>1?`${30001+a}~${30000+a+n} (${n}개) — 입력 레지스터는 없음`:`${30001+a} — 입력 레지스터는 없음`;
 return `주소 ${a}, ${n}`;}
function render(j){
 const c=j.confirmed,m=j.modbus,regs=m.regs,st=STATES[j.state]||[j.state,''];
 pill($('state'),st[0],st[1]);
 if(c.count>0){
  $('sw').style.background=SW[c.color]||'';
  $('res').textContent=`${c.colorName} · ${c.sizeName}`;
  $('resSub').textContent=`#${c.count} · 색상 ${c.color} · 크기 ${c.size} · RGB(${c.rgb.join(', ')}) · 변 비율 ${c.ratio.toFixed(2)}`;
 }
 const b=j.blob;
 const rows=[['판 위 물체',regs[REG_PRESENT]?'<span class="pill ok">있음</span>':(c.count>0?'<span class="pill">없음 — 위 결과는 마지막 감지</span>':'<span class="pill">없음</span>')],
  ['현재 프레임',b.area>0?`덩어리 ${b.area}px · ${b.valid?'유효':'무효'+(b.touchesBorder?'(경계 접촉)':'(작음)')} · 비율 ${b.ratio.toFixed(2)} → ${b.size}`:'변화 없음'],
  ['현재 프레임 색',j.color.samples>0?`${j.color.name} · RGB(${j.color.rgb.join(', ')})`:'-']];
 if(j.state=='SETTLING')rows.push(['안정화',`${j.stable}프레임`]);
 if(j.state=='CONFIRMED')rows.push(['확정 위치 점유율',`${Math.round(j.spot.occupancy*100)}% (35% 미만이 이어지면 제거로 판정)`]);
 kv($('nowKv'),rows);
 [...tb.rows].forEach((tr,i)=>{const v=regs[i];tr.cells[3].textContent=v;tr.cells[4].textContent='0x'+v.toString(16).toUpperCase().padStart(4,'0');tr.cells[6].textContent=REGS[i][1](v);
  if(prevRegs&&prevRegs[i]!==v){tr.classList.add('flash');setTimeout(()=>tr.classList.remove('flash'),80);}});
 prevRegs=regs.slice();
 pill($('mode'),m.enabled?'운용: Modbus 485 + 웹':'디버그: 시리얼 로그 + 웹',m.enabled?'ok':'warn');
 $('mbCfg').textContent=`슬레이브 ID ${m.slave} · ${m.baud}bps 8N1 · TX GPIO${m.txPin} · RX GPIO${m.rxPin} · `+(m.dePin<0?'자동 방향 모듈':`DE/RE GPIO${m.dePin}`);
 let hint='';
 if(!m.enabled){pill($('mbPill'),'통신 꺼짐','warn');
  kv($('mbKv'),[['Modbus','꺼짐 (ENABLE_MODBUS = false)'],['UART0 (GPIO1/3)','시리얼 로그 출력용 (115200bps)']]);
  hint='표의 값은 통신을 켰을 때 PLC가 읽게 될 값입니다.';
 }else{
  const live=m.lastAgo>=0&&m.lastAgo<m.linkTimeout;
  pill($('mbPill'),m.requests==0?'요청 없음':(live?'통신 중':'요청 끊김'),m.requests==0?'warn':(live?'ok':'bad'));
  const r=[['수신 요청',`${m.requests}회 (정상 ${m.ok} · 오류 응답 ${m.requests-m.ok})`],['마지막 요청',m.requests?ago(m.lastAgo):'없음']];
  if(m.requests){r.push(['함수',`FC${String(m.lastFc).padStart(2,'0')} ${FCN[m.lastFc]||''}`],['대상',regRange(m.lastFc,m.lastAddr,m.lastArg)],['결과',m.lastOk?'<span class="pill ok">정상 응답</span>':'<span class="pill bad">오류 응답</span> 주소 범위/함수 코드 확인']);}
  kv($('mbKv'),r);
  if(m.requests==0)hint='요청이 한 번도 없으면 ESP32-CAM-MB가 빠져 있는지 확인하고, TXD/RXD 연결을 서로 바꿔 보고, 485 배선(A/B), 보오율, 슬레이브 ID를 확인하세요. CRC 오류나 다른 ID로 보낸 요청은 집계되지 않습니다.';
  else if(m.requests>m.ok)hint='오류 응답은 없는 주소(40010 이후 등)나 지원하지 않는 함수 코드를 요청했을 때 생깁니다.';
 }
 $('mbHint').textContent=hint;
 const cam=j.camera,w=j.wifi;
 kv($('sysKv'),[
  ['카메라',cam.ok?`<span class="pill ok">정상</span> ${PIDS[cam.pid]||''} (PID 0x${cam.pid.toString(16).toUpperCase()})`:'<span class="pill bad">초기화 실패</span> 배선/전원 확인 후 재부팅'],
  ['처리 속도',`${j.fps.toFixed(1)} fps · 처리 ${j.frame}프레임`],
  ['프레임 수신',`마지막 ${ago(cam.lastFrameAgo)} · 실패 누적 ${cam.fails}회`],
  ['Wi-Fi',`${w.ssid} · ${w.ip} · ${w.rssi} dBm`],
  ['주소',`http://${w.host}.local`],
  ['가동 시간',upt(j.uptime)],
  ['여유 메모리',`내부 ${(j.heap/1024).toFixed(0)}KB · PSRAM ${(j.psram/1024).toFixed(0)}KB`],
  ['판(배경)',`RGB(${j.plate.rgb.join(', ')}) · 게인 ${j.plate.gain.join(' / ')}`]]);
 if($('det').open)$('st').textContent=JSON.stringify(j,null,1);
}
async function pollStatus(){
 try{const j=await(await fetch('/status',{cache:'no-store'})).json();lastOk=Date.now();render(j);}catch(e){}
 if(Date.now()-lastOk>3000)pill($('conn'),'장치 응답 없음','bad');else pill($('conn'),'연결됨','ok');
 setTimeout(pollStatus,700);
}
let since=0,prevUp=0;const logLines=[];
const hms=d=>[d.getHours(),d.getMinutes(),d.getSeconds()].map(x=>String(x).padStart(2,'0')).join(':');
function addLog(s){logLines.push(s);if(logLines.length>300)logLines.shift();}
async function pollLog(){
 try{
  const t=await(await fetch('/events?since='+since,{cache:'no-store'})).text();
  const rows=t.split('\n'),h=rows.shift().split('\t'),now=+h[1],last=+h[2];
  if(now<prevUp||last<since){addLog('--- 장치가 재시작됨 ---');since=0;}
  prevUp=now;
  for(const r of rows){if(!r)continue;const a=r.indexOf('\t'),b=r.indexOf('\t',a+1);
   const seq=+r.slice(0,a),ms=+r.slice(a+1,b),txt=r.slice(b+1);
   if(seq<=since)continue;
   if(since&&seq>since+1)addLog(`… (${seq-since-1}줄 지나감)`);
   addLog(hms(new Date(Date.now()-(now-ms)))+'  '+txt);since=seq;}
  const el=$('log'),bottom=el.scrollTop+el.clientHeight>=el.scrollHeight-20;
  el.textContent=logLines.join('\n');if(bottom)el.scrollTop=el.scrollHeight;
 }catch(e){}
 setTimeout(pollLog,1000);
}
function relearn(){if(!confirm('판 위의 물체를 모두 치웠나요? 배경을 다시 학습합니다.'))return;fetch('/relearn').then(r=>r.text()).then(t=>alert(t));}
$('s').src='http://'+location.hostname+':81/stream';
pollStatus();pollLog();
</script></body></html>)rawliteral";

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// 감지 기록: 첫 줄 "now\t<가동ms>\t<마지막 seq>", 이후 "seq\t<기록ms>\t<내용>" (since 이후만)
static esp_err_t events_handler(httpd_req_t *req) {
  uint32_t since = 0;
  char q[48], v[16];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
      httpd_query_key_value(q, "since", v, sizeof(v)) == ESP_OK) {
    since = strtoul(v, nullptr, 10);
  }
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  uint32_t last = evtSeq;
  char line[EVT_LEN + 32];
  int n = snprintf(line, sizeof(line), "now\t%lu\t%lu\n", (unsigned long)millis(), (unsigned long)last);
  if (httpd_resp_send_chunk(req, line, n) != ESP_OK) return ESP_FAIL;

  uint32_t first = last > EVT_MAX ? last - EVT_MAX + 1 : 1;
  if (since + 1 > first) first = since + 1;
  for (uint32_t s = first; s <= last; s++) {
    EventLine e;
    bool ok = false;
    if (xSemaphoreTake(evtMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      e = evtBuf[(s - 1) % EVT_MAX];
      ok = e.seq == s;          // 읽는 사이 덮어써졌으면 건너뜀
      xSemaphoreGive(evtMutex);
    }
    if (!ok) continue;
    n = snprintf(line, sizeof(line), "%lu\t%lu\t%s\n", (unsigned long)e.seq, (unsigned long)e.ms, e.text);
    if (httpd_resp_send_chunk(req, line, n) != ESP_OK) return ESP_FAIL;
  }
  return httpd_resp_send_chunk(req, NULL, 0);
}

static size_t jsonAppend(char *buf, size_t cap, size_t n, const char *fmt, ...) __attribute__((format(printf, 4, 5)));
static size_t jsonAppend(char *buf, size_t cap, size_t n, const char *fmt, ...) {
  if (n >= cap) return n;
  va_list ap;
  va_start(ap, fmt);
  int w = vsnprintf(buf + n, cap - n, fmt, ap);
  va_end(ap);
  return w < 0 ? cap : n + w;   // cap 이상이면 잘림 → 호출자가 500 처리
}

static esp_err_t status_handler(httpd_req_t *req) {
  static char json[3072];
  const size_t cap = sizeof(json);
  DebugStatus st;
  if (xSemaphoreTake(dbgMutex, pdMS_TO_TICKS(200)) != pdTRUE) return httpd_resp_send_500(req);
  st = dbgStatus;
  xSemaphoreGive(dbgMutex);
  const BlobInfo &b = st.blob;
  const ColorInfo &c = st.color;
  size_t n = jsonAppend(json, cap, 0,
    "{\"state\":\"%s\",\"fps\":%.1f,\"frame\":%lu,\"stable\":%d,"
    "\"fgRaw\":%d,\"fg\":%d,\"components\":%d,"
    "\"blob\":{\"area\":%d,\"valid\":%s,\"touchesBorder\":%s,\"cx\":%.1f,\"cy\":%.1f,"
    "\"len\":%.1f,\"wid\":%.1f,\"ratio\":%.2f,\"angle\":%.0f,\"size\":\"%s\"},"
    "\"color\":{\"code\":%d,\"name\":\"%s\",\"rgb\":[%d,%d,%d],\"samples\":%d,\"chromaShare\":%.2f,\"vRatio\":%.2f,"
    "\"votes\":{\"white\":%d,\"black\":%d,\"red\":%d,\"yellow\":%d,\"green\":%d,\"blue\":%d,\"other\":%d}},"
    "\"spot\":{\"area\":%d,\"occupancy\":%.2f,\"bestOverlap\":%.2f,\"removeStreak\":%d,\"changeCount\":%d},"
    "\"plate\":{\"rgb\":[%.0f,%.0f,%.0f],\"gray\":%.0f,\"gain\":[%.2f,%.2f,%.2f]},"
    "\"confirmed\":{\"count\":%u,\"color\":%u,\"colorName\":\"%s\",\"size\":%u,\"sizeName\":\"%s\",\"ratio\":%.2f,\"rgb\":[%d,%d,%d]}",
    STATE_NAMES[st.state], st.fps, (unsigned long)st.frameNo, st.stableCount,
    st.fgRawCount, st.fgCount, st.components,
    b.area, b.valid ? "true" : "false", b.touchesBorder ? "true" : "false", b.cx, b.cy,
    b.len, b.wid, b.ratio, b.angleDeg, getSizeName(classifySizeByRatio(b.ratio)),
    c.code, getColorName(c.code), c.r, c.g, c.b, c.samples, c.chromaShare, c.vRatio,
    c.votes[1], c.votes[2], c.votes[3], c.votes[4], c.votes[5], c.votes[6], c.votes[7],
    st.footArea, st.foot.occ, st.foot.bestOverlap, st.removeStreak, st.changeCount,
    plateR, plateG, plateB, plateGray, gainR, gainG, gainB,
    detectCount, curColorCode, getColorName(curColorCode), curSizeCode, getSizeName(curSizeCode),
    curRatio, curR, curG, curB);

  // 카메라 · Wi-Fi · 시스템
  unsigned long now = millis();
  unsigned long lastFrame = lastFrameMs;
  n = jsonAppend(json, cap, n,
    ",\"uptime\":%lu,\"heap\":%u,\"psram\":%u,"
    "\"camera\":{\"ok\":%s,\"pid\":%u,\"fails\":%lu,\"lastFrameAgo\":%ld},"
    "\"wifi\":{\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"host\":\"%s\"}",
    now, (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram(),
    cameraOk ? "true" : "false", (unsigned)camPid, (unsigned long)camFails,
    lastFrame ? (long)(now - lastFrame) : -1L,
    ssid, WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(), HOSTNAME);

  // Modbus: 설정, 통신 통계, 레지스터 값 (40001~)
  uint32_t req_n = mbReqCount;
  unsigned long lastReq = mbLastReqMs;
  n = jsonAppend(json, cap, n,
    ",\"modbus\":{\"enabled\":%s,\"slave\":%d,\"baud\":%d,\"txPin\":%d,\"rxPin\":%d,\"dePin\":%d,\"linkTimeout\":%lu,"
    "\"requests\":%lu,\"ok\":%lu,\"lastAgo\":%ld,\"lastFc\":%u,\"lastAddr\":%u,\"lastArg\":%u,\"lastOk\":%s,\"regs\":[",
    ENABLE_MODBUS ? "true" : "false", SLAVE_ID, MODBUS_BAUD, MB_TX_PIN, MB_RX_PIN, RS485_DE_RE_PIN, MB_LINK_TIMEOUT_MS,
    (unsigned long)req_n, (unsigned long)mbOkCount, req_n ? (long)(now - lastReq) : -1L,
    (unsigned)mbLastFc, (unsigned)mbLastAddr, (unsigned)mbLastArg, mbLastOk ? "true" : "false");
  for (int i = 0; i < REG_COUNT; i++) n = jsonAppend(json, cap, n, "%s%u", i ? "," : "", (unsigned)regMirror[i]);
  n = jsonAppend(json, cap, n, "]}}");
  if (n >= cap) return httpd_resp_send_500(req);

  httpd_resp_set_type(req, "application/json; charset=utf-8");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t capture_handler(httpd_req_t *req) {
  if (!dbgBuf) return httpd_resp_send_500(req);
  uint8_t *jpg = NULL;
  size_t len = 0;
  bool ok = false;
  if (xSemaphoreTake(dbgMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    ok = fmt2jpg(dbgBuf, DBG_W * DBG_H * 2, DBG_W, DBG_H, PIXFORMAT_RGB565, 90, &jpg, &len);
    xSemaphoreGive(dbgMutex);
  }
  if (!ok) return httpd_resp_send_500(req);
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t res = httpd_resp_send(req, (const char *)jpg, len);
  free(jpg);
  return res;
}

static esp_err_t raw_bmp_handler(httpd_req_t *req) {
  if (!dbgBuf) return httpd_resp_send_500(req);
  uint8_t *bmp = NULL;
  size_t len = 0;
  bool ok = false;
  if (xSemaphoreTake(dbgMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    ok = fmt2bmp(rawBuf, IMG_N * 2, IMG_W, IMG_H, PIXFORMAT_RGB565, &bmp, &len);
    xSemaphoreGive(dbgMutex);
  }
  if (!ok) return httpd_resp_send_500(req);
  httpd_resp_set_type(req, "image/bmp");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t res = httpd_resp_send(req, (const char *)bmp, len);
  free(bmp);
  return res;
}

static esp_err_t relearn_handler(httpd_req_t *req) {
  relearnRequested = true;
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  return httpd_resp_send(req, "배경 재학습을 시작합니다 (약 3초). 판이 비어 있어야 합니다.", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t stream_handler(httpd_req_t *req) {
  static const char *BOUNDARY = "\r\n--frame\r\n";
  char part[80];
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  uint32_t lastId = 0;
  unsigned long lastSend = 0;
  esp_err_t res = ESP_OK;
  while (res == ESP_OK) {
    // 분석은 loop()에서만 수행하고, 여기서는 최신 디버그 화면만 인코딩해서 보낸다
    if (!dbgBuf) return httpd_resp_send_500(req);
    if (dbgFrameId == lastId && millis() - lastSend < 2000) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    uint8_t *jpg = NULL;
    size_t len = 0;
    bool ok = false;
    if (xSemaphoreTake(dbgMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      lastId = dbgFrameId;
      ok = fmt2jpg(dbgBuf, DBG_W * DBG_H * 2, DBG_W, DBG_H, PIXFORMAT_RGB565, 80, &jpg, &len);
      xSemaphoreGive(dbgMutex);
    }
    if (!ok) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
    size_t hlen = snprintf(part, sizeof(part), "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)len);
    res = httpd_resp_send_chunk(req, BOUNDARY, strlen(BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)jpg, len);
    free(jpg);
    lastSend = millis();
    vTaskDelay(pdMS_TO_TICKS(30));
  }
  return res;
}

void startWebServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.stack_size = 8192;
  httpd_uri_t uris[] = {
    {.uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL},
    {.uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = NULL},
    {.uri = "/events", .method = HTTP_GET, .handler = events_handler, .user_ctx = NULL},
    {.uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL},
    {.uri = "/raw.bmp", .method = HTTP_GET, .handler = raw_bmp_handler, .user_ctx = NULL},
    {.uri = "/relearn", .method = HTTP_GET, .handler = relearn_handler, .user_ctx = NULL},
  };
  if (httpd_start(&web_httpd, &config) == ESP_OK) {
    for (auto &u : uris) httpd_register_uri_handler(web_httpd, &u);
  }

  // 스트림은 별도 서버(포트 81)로 분리: 브라우저가 스트림을 보고 있어도 /status, /capture가 응답한다
  config.server_port = 81;
  config.ctrl_port += 1;
  httpd_uri_t stream_uri = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL};
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// Wi-Fi 연결/끊김 처리(웹 서버·mDNS 시작)와 Modbus 통신 상태 변화를 이벤트로 남긴다
static void networkTask() {
  static unsigned long lastCheck = 0;
  static bool wifiUp = false, mdnsStarted = false, mbLinkUp = false;
  unsigned long now = millis();
  if (now - lastCheck < 500) return;
  lastCheck = now;

  if (ENABLE_WEB) {
    bool up = WiFi.status() == WL_CONNECTED;
    if (up && !wifiUp) {
      if (!webStarted) { startWebServers(); webStarted = true; }
      if (!mdnsStarted && MDNS.begin(HOSTNAME)) {
        MDNS.addService("http", "tcp", 80);
        mdnsStarted = true;
      }
      EVENT("[WiFi] 연결됨: http://%s (http://%s.local) RSSI %d dBm",
            WiFi.localIP().toString().c_str(), HOSTNAME, (int)WiFi.RSSI());
    } else if (!up && wifiUp) {
      EVENT("[WiFi] 연결 끊김 — 자동 재연결 시도 중");
    }
    wifiUp = up;
  }

  if (ENABLE_MODBUS) {
    bool link = mbReqCount > 0 && now - mbLastReqMs < MB_LINK_TIMEOUT_MS;
    if (link && !mbLinkUp) {
      EVENT("[485] PLC 요청 수신 시작 (FC%02u, 주소 %u, %s)", (unsigned)mbLastFc, 40001u + mbLastAddr,
            mbLastOk ? "정상 응답" : "오류 응답");
    } else if (!link && mbLinkUp) {
      EVENT("[485] %lu초 넘게 PLC 요청 없음", MB_LINK_TIMEOUT_MS / 1000);
    }
    mbLinkUp = link;
  }
}

// ==========================================
// 17. 초기 설정 (Setup)
// ==========================================
void setup() {
  if (SERIAL_LOG) Serial.begin(115200);
  evtMutex = xSemaphoreCreateMutex();

  camera_config_t config = {};   // 미설정 필드(fb_location 등)가 쓰레기값이 되지 않도록 0 초기화
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
  config.frame_size = FRAMESIZE_QQVGA; // 160×120
  config.jpeg_quality = 12;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;
  // 카메라가 실패해도 멈추지 않는다 — 웹/Modbus는 계속 동작해서 실패 상태를 확인할 수 있게 한다
  esp_err_t camErr = esp_camera_init(&config);
  cameraOk = camErr == ESP_OK;
  if (cameraOk) camPid = esp_camera_sensor_get()->id.PID;

  bgR = (uint8_t *)allocBig(IMG_N);
  bgG = (uint8_t *)allocBig(IMG_N);
  bgB = (uint8_t *)allocBig(IMG_N);
  fgRaw = (uint8_t *)allocBig(IMG_N);
  fgMap = (uint8_t *)allocBig(IMG_N);
  bfsQueue = (uint16_t *)allocBig(IMG_N * sizeof(uint16_t));
  footMask = (uint8_t *)allocBig(IMG_N);
  if (footMask) memset(footMask, 0, IMG_N);
  if (ENABLE_WEB) {
    dbgBuf = (uint8_t *)allocBig(DBG_W * DBG_H * 2);
    rawBuf = (uint8_t *)allocBig(IMG_N * 2);
    dbgMutex = xSemaphoreCreateMutex();
    if (dbgBuf) memset(dbgBuf, 0, DBG_W * DBG_H * 2);
    if (rawBuf) memset(rawBuf, 0, IMG_N * 2);
    if (!dbgBuf || !rawBuf) { dbgBuf = nullptr; EVENT("[ERR] 디버그 버퍼 할당 실패"); }
  }
  if (!bgR || !bgG || !bgB || !fgRaw || !fgMap || !bfsQueue || !footMask) {
    LOG("[ERR] 작업 버퍼 할당 실패\n");
    while (true) { delay(1000); }
  }

  if (cameraOk) {
    EVENT("[CAM] 카메라 정상 (센서 PID=0x%02X)", (unsigned)camPid);
  } else {
    EVENT("[ERR] 카메라 초기화 실패 (0x%x) — 카메라 배선/전원 확인 후 재부팅", (unsigned)camErr);
  }

  if (ENABLE_MODBUS) {
    // UART0를 Modbus에 넘기기 전에 115200bps로 안내 한 줄 — 시리얼 모니터가 조용한 이유를 알 수 있게
    MB_SERIAL.begin(115200);
    MB_SERIAL.printf("\n[Info] Modbus 485 모드 (ENABLE_MODBUS=true): 이후 시리얼 로그 없음. "
                     "상태는 웹 http://%s.local 에서 확인\n", HOSTNAME);
    MB_SERIAL.flush();
    MB_SERIAL.end();
    // 이후 UART0에는 Modbus 응답만 나가야 한다. 시스템·카메라 드라이버 로그가 섞이면 응답 프레임이 깨진다
    esp_log_level_set("*", ESP_LOG_NONE);
    MB_SERIAL.begin(MODBUS_BAUD, SERIAL_8N1, MB_RX_PIN, MB_TX_PIN);
    MB_SERIAL.setDebugOutput(false);
    mb.begin(&MB_SERIAL, RS485_DE_RE_PIN);
    mb.slave(SLAVE_ID);
    for (int i = 0; i < REG_COUNT; i++) mb.addHreg(i, 0);
    mb.onRequest(onMbRequest);           // 통신 상태(요청 수/마지막 요청) 집계
    mb.onRequestSuccess(onMbSuccess);
    EVENT("[485] Modbus RTU 시작: 슬레이브 ID %d, %dbps 8N1, 40001~%d (UART0 TX=GPIO%d RX=GPIO%d)",
          SLAVE_ID, MODBUS_BAUD, 40000 + REG_COUNT, MB_TX_PIN, MB_RX_PIN);
  }

  // Wi-Fi는 기다리지 않는다 (Modbus 운용 중 Wi-Fi가 없어도 멈추지 않게). 연결되면 networkTask()가 웹 서버를 시작
  if (ENABLE_WEB) {
    WiFi.setHostname(HOSTNAME);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
  } else {
    WiFi.mode(WIFI_OFF);
  }

  // 빈 판에서 자동 노출/화이트밸런스로 적응(2초) → 고정 → 배경 평균 학습
  //   ※ 부팅 시 판 위에 물체가 없어야 함. 있었다면 치운 뒤 웹의 '배경 재학습' 또는 Modbus 40009에 1 쓰기
  if (cameraOk) learnBackground(true);
}

// ==========================================
// 18. 메인 루프
// ==========================================
void loop() {
  static unsigned long fpsT0 = 0;
  static int fpsFrames = 0;

  if (ENABLE_MODBUS) {
    mb.task();
    // 웹 표시용 사본을 실제 레지스터 값으로 맞춘다 (PLC가 쓴 값 포함)
    for (int i = 0; i < REG_COUNT; i++) regMirror[i] = mb.Hreg(i);
    if (regMirror[REG_COMMAND] == 1) {
      setReg(REG_COMMAND, 2);
      relearnRequested = true;
      EVENT("[485] PLC 명령: 배경 재학습 (40009 = 1)");
    }
  }
  networkTask();

  if (relearnRequested) {
    relearnRequested = false;
    if (cameraOk) {
      EVENT("[BG] 배경 재학습 시작...");
      learnBackground(true);
    } else {
      EVENT("[BG] 카메라 초기화 실패 상태라 재학습할 수 없음");
    }
    setReg(REG_COMMAND, 0);
  }

  if (cameraOk) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      lastFrameMs = millis();
      processFrame(fb);
      esp_camera_fb_return(fb);
      fpsFrames++;
    } else {
      camFails = camFails + 1;
    }
  }
  if (millis() - fpsT0 >= 1000) {
    fps = fpsFrames * 1000.0f / (millis() - fpsT0);
    fpsFrames = 0;
    fpsT0 = millis();
  }

  // 프레임 수신 실패는 10초에 한 번만 기록
  static uint32_t failsLogged = 0;
  static unsigned long failLogMs = 0;
  if (camFails != failsLogged && millis() - failLogMs >= 10000) {
    EVENT("[CAM] 프레임 수신 실패 (누적 %lu회)", (unsigned long)camFails);
    failsLogged = camFails;
    failLogMs = millis();
  }

  serviceDelay(1);
}
