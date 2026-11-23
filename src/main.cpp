#include <Arduino.h>
#include <Modbus.h>
#include <ModbusRTU.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <stdarg.h>
#include <new>
#include "esp_camera.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"
#include "lwip/etharp.h"
#include "lwip/tcpip.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "img_converters.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define FW_VERSION "2026.11.23"         // 웹 화면·/status에 표시 (장치가 여러 대일 때 어떤 펌웨어인지 구분)

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
// const char *ssid = "1실습실_2.4G"; 
const char *ssid = "1_rap_extender_2.4G";   // 192.168.10.254
const char *password = "tank5000";
const char *HOSTNAME = "link-cam1";     // 시리얼 없이 접속: http://link-cam1.local (또는 공유기 DHCP 목록에서 IP 확인)

// Wi-Fi 연결 안정화 (거리가 멀고 사람·장비가 가로막는 현장용)
//   고정 IP: 처음에 DHCP로 접속한 뒤 후보 주소를 앞에서부터 ARP로 물어보고, 응답이 없는(비어 있는) 첫 주소를 쓴다.
//            게이트웨이·서브넷·DNS는 DHCP로 받은 값을 그대로 쓴다. 후보가 모두 사용 중이면 DHCP 주소를 유지한다.
#define WIFI_STATIC_IP    true          // false = 항상 DHCP

const IPAddress STATIC_IP_CANDIDATES[] = {
  IPAddress(192, 168, 10, 230), IPAddress(192, 168, 10, 231),
  IPAddress(192, 168, 10, 232), IPAddress(192, 168, 10, 233),
};
#define WIFI_COUNTRY      "KR"          // 국가 코드 (채널 1~13 사용 가능)
#define WIFI_11B_ONLY     false         // true = 802.11b 전용: 속도는 느리지만 먼 거리 수신 감도가 좋아짐 (공유기가 11b를 허용해야 함)
#define WIFI_RECONNECT_MS 30000UL       // 이 시간 넘게 끊겨 있으면 연결을 처음부터 다시 시도
#define STREAM_MAX_FPS    5             // 웹 영상 스트림 최대 초당 프레임 (약한 신호에서 상태 화면이 멈추지 않게 대역 절약)

#define SLAVE_ID        6
#define MODBUS_BAUD     38400
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
#define REG_RATIO_X100  7       // 40008: 길이비 x 100 = 추정 길이 ÷ 35mm (예: 157 = 약 55mm). 소≈100, 중≈157, 대≈243
#define REG_COMMAND     8       // 40009: 명령/상태. PLC가 명령 코드(1~4)를 쓰면 실행 → 100+N(실행 중) → 0(완료) 또는 200+N(실패)
                                //        명령 목록과 상태 값은 '14-2. PLC 명령' 참고
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

// 검정 물체 보강
//   3D 프린팅 면이 빛을 반사해 검정 윗면이 판 밝기의 50~80%(회색)로 찍히면 위의 그림자 필터에 지워져
//   물체가 일부(진한 옆면·모서리)만 잡힌다. 검정으로 판정된 덩어리에는 그림자 필터 없이 다시 본 전경 중
//   '같은 열(좌우 BLACK_GROW_COLS 안)에서 덩어리 바닥보다 위'인 픽셀만 붙인다.
//   카메라 쪽 옆면은 빛을 못 받아 늘 진하게 잡히므로, 그보다 아래(카메라 쪽)에 생기는 진짜 그림자는 붙지 않는다.
const int BLACK_GROW_COLS = 8;

// ==========================================
// 5. 크기 판별 — 원근 보정 + 옆면 제거 + 최소 외접 사각형
// ==========================================
//  학습 샘플 30개(흰·빨·검 × 소·중·대, 여러 위치·방향)로 맞춘 값. 다시 맞추기: tools/size_check.py
//  카메라가 비스듬히 내려다보기 때문에
//   (1) 세로가 가로보다 짧게, 판 안쪽(화면 위)일수록 작게 찍힌다 → '원근 보정 좌표'로 바꿔서 잰다
//   (2) 카메라 쪽 옆면(높이 35mm)이 물체 아래쪽에 붙어 보인다 (흰색은 옆면이 판과 비슷한 회색이라 안 잡힘)
//       → 여러 방향의 폭에서 옆면이 늘린 만큼(아래 방향 성분)을 빼고, 가장 작은 외접 사각형 = 윗면으로 본다
//  예전 방식(긴변/짧은변 비율)은 옆면 때문에 가로로 놓인 중형을 소형으로, 비스듬한 대형을 중형으로 봤다.
//  샘플 측정 길이: 소 34~44mm, 중 50~56mm, 대 62~87mm → 경계는 그 사이
const float SIZE_PERSP_P       = 0.00203f; // 원근: 행 y의 확대율 g = 1 / (1 + P·(y - 60))
const float SIZE_Y_SCALE       = 1.1432f;  // 세로 축척 (기울어져서 세로가 짧게 찍힘)
const float SIZE_PX_PER_35MM   = 31.48f;   // 보정 좌표에서 35mm(단면 폭)의 길이
const float SIDE_FACE_PX       = 10.0f;    // 옆면이 같이 잡히는 물체(흰색 외)의 옆면 두께 (보정 좌표)
const float SIZE_SMALL_MAX_MM  = 47.0f;    // 추정 길이 < 47mm → 소(35mm)
const float SIZE_MEDIUM_MAX_MM = 58.5f;    // 47~58.5mm → 중(55mm), 그 이상 → 대(85mm)
const float SIZE_RECHECK_MM    = 3.0f;     // 확정 후 길이가 경계를 이만큼 넘어 다르게 보이면 다시 판정
const int   SIZE_CALIPER_STEP  = 3;        // 외접 사각형을 찾는 각도 간격(도)

// ==========================================
// 6. 색상 판별 (판 기준 화이트밸런스 + 픽셀 투표)
// ==========================================
const int   PX_CHROMA_MIN     = 25;    // 유채색 픽셀: 최대-최소 ≥ 25
const float PX_SAT_MIN        = 0.28f; //              그리고 채도(S) ≥ 0.28
const float CHROMA_SHARE_MIN  = 0.35f; // 내부 픽셀 중 유채색이 35% 이상이면 유채색 물체
const float WHITE_BLACK_SPLIT = 1.2f;  // 무채색 물체: (평균 밝기 / 판 밝기) ≥ 1.2 → 흰색, 미만 → 검정
                                       //   샘플: 흰 2.3~2.6, 검 0.44~0.66 (반사로 회색이 된 검정 윗면 때문에 0.65는 너무 높았음)
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
uint8_t  *fgRaw = nullptr;   // 원시 전경 비트: 1 = 전경(그림자 필터 적용), 2 = 그림자 필터 없이 본 전경 (검정 보강용)
uint8_t  *fgMap = nullptr;   // 0:배경 1:전경 2:다른 덩어리 3:선택 덩어리 가장자리 4:검정 보강 10+n:선택 덩어리 내부(n=픽셀 분류)
uint16_t *bfsQueue = nullptr;
uint8_t  *measMask = nullptr;   // 크기 측정용 마스크 (선택 덩어리 + 검정 보강 → 열림 → 가장 큰 덩어리)
uint8_t  *measTmp = nullptr;

struct BlobInfo {
  int   area = 0;
  int   seed = -1;
  int   minX = 0, minY = 0, maxX = 0, maxY = 0;
  bool  touchesBorder = false;
  bool  valid = false;           // 면적 충분 + ROI 경계 미접촉
  float cx = 0, cy = 0;          // 중심
  float len = 0, wid = 0;        // 2차 모멘트 긴변/짧은변 (px, 화면 그대로 — 참고용)
  float ratio = 0;               // len / wid (참고용, 크기 판별에는 쓰지 않음)
  float angleDeg = 0;            // 긴변 방향
  float lenMm = 0, widMm = 0;    // 원근·옆면 보정 후 윗면 크기 추정 (mm) — 크기 판별 기준
  float boxX[4] = {0}, boxY[4] = {0};   // 추정 윗면 사각형 꼭짓점 (화면 좌표, 디버그 표시용)
  int   grown = 0;               // 검정 보강으로 붙인 픽셀 수
  bool  sideCut = false;         // 옆면 보정 적용 여부 (흰색 외)
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
#define ACC_LEN_MAX 32
float accLen[ACC_LEN_MAX];       // 프레임별 추정 길이(mm) → 확정 시 중앙값
int   accLenN = 0;
int   accFrames = 0;

// 확정된 최신 측정 결과
int curR = 0, curG = 0, curB = 0;
uint16_t curColorCode = 0;
uint16_t curSizeCode = 0;
float curLenMm = 0.0f;           // 확정에 쓴 추정 길이 (안정화 프레임들의 중앙값)
uint16_t detectCount = 0;

uint32_t frameNo = 0;
float fps = 0;
// 메인 루프(카메라 → 감지·판정 → 명령) 한 바퀴의 최대 간격. Wi-Fi가 끊기거나 웹을 많이 써도 변하지 않아야 정상
//   (웹 '카메라 · 시스템'에 표시, LOOP_STALL_WARN_MS를 넘으면 감지 기록에 남김. 배경 학습의 약 3초는 제외)
const unsigned long LOOP_STALL_WARN_MS = 1000;
volatile unsigned long loopGapMax10s = 0, loopGapMaxBoot = 0;
bool loopGapSkip = false;          // 배경 학습(일부러 약 3초 머묾) 직후 한 번은 재지 않음
bool lastBestValid = false;      // 마지막 프레임에 물체로 보이는 덩어리가 있었나 (PLC 명령 1·2의 '판 비었나' 확인용)
uint8_t bgVersion = 0;           // 배경 학습에 성공할 때마다 +1 (학습 데이터의 배경 스냅샷 구분용)

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

// Wi-Fi 상태 (웹 표시용)
enum IpMode : uint8_t { IPM_DHCP, IPM_PROBING, IPM_STATIC, IPM_FALLBACK };
const char *IPM_NAMES[] = {"dhcp", "probing", "static", "fallback"};
volatile uint8_t ipMode = IPM_DHCP;
volatile uint8_t wifiDiscReason = 0;         // 마지막 끊김 사유 코드 (Wi-Fi 이벤트 콜백에서 기록)
volatile int8_t  wifiDiscRssi = 0;           // 끊길 때의 신호 세기
volatile unsigned long wifiDiscMs = 0;
volatile uint32_t wifiDrops = 0;             // 연결된 상태에서 끊긴 횟수
volatile unsigned long wifiUpSinceMs = 0;    // 현재 연결이 시작된 시각 (0 = 끊김)
volatile int wifiRssiAvg = 0, wifiRssiMin = 0, wifiRssiMinBoot = 0;  // 최근 1분 평균·최저, 부팅 후 최저
static const char *wifiReasonText(uint8_t r);

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

// Modbus는 전용 태스크(modbusTask)가 1ms마다 처리한다.
//   예전에는 메인 루프가 영상을 한 장 처리할 때마다(0.15~0.3초) mb.task()를 한 번 불러서 응답이 40~300ms로 들쭉날쭉했고,
//   PLC 응답 대기 시간을 넘긴 응답은 PLC 프레임 모니터에 '모름'으로 찍혀 버려졌다. 이제 요청 후 수 ms 안에 응답한다.
//   레지스터는 메인 루프(판정 결과 쓰기)와 이 태스크(PLC 읽기/쓰기)가 함께 쓰므로 mbMutex로 보호한다.
//   재귀 뮤텍스: 확정 결과 여러 개를 쓰는 동안 바깥에서 한 번 더 잡아, PLC가 반쯤 바뀐 값을 읽지 않게 한다 (tryConfirm)
SemaphoreHandle_t mbMutex = nullptr;
static inline void mbLock() { if (mbMutex) xSemaphoreTakeRecursive(mbMutex, portMAX_DELAY); }
static inline void mbUnlock() { if (mbMutex) xSemaphoreGiveRecursive(mbMutex); }

static void modbusTask(void *) {
  for (;;) {
    mbLock();
    mb.task();
    mbUnlock();
    vTaskDelay(1);   // 1ms (FreeRTOS tick 1kHz)
  }
}

// Modbus 레지스터 쓰기 — 홀딩(FC03, 40001~)과 입력(FC04, 30001~)에 같은 값. 웹 표시용 사본도 함께 갱신
static void setReg(uint16_t addr, uint16_t val) {
  if (addr >= REG_COUNT) return;
  regMirror[addr] = val;
  if (!ENABLE_MODBUS) return;
  mbLock();
  mb.Hreg(addr, val);
  mb.Ireg(addr, val);
  mbUnlock();
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

static void flushFrames(int n) {
  for (int i = 0; i < n; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
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

// 추정 길이(mm) → 크기 코드
uint16_t classifySizeByLen(float mm) {
  if (mm <= 0) return 0;
  if (mm < SIZE_SMALL_MAX_MM) return 1;    // 소 (35×35×35)
  if (mm < SIZE_MEDIUM_MAX_MM) return 2;   // 중 (35×35×55)
  return 3;                                // 대 (35×35×85)
}

// 확정된 크기와 '확실히' 다른 길이인가 (경계 근처에서 흔들리는 값으로는 다시 판정하지 않는다)
static bool sizeClearlyDiffers(float mm, uint16_t code) {
  if (mm <= 0 || code == 0) return false;
  const float m = SIZE_RECHECK_MM;
  switch (code) {
    case 1: return mm >= SIZE_SMALL_MAX_MM + m;
    case 2: return mm < SIZE_SMALL_MAX_MM - m || mm >= SIZE_MEDIUM_MAX_MM + m;
    default: return mm < SIZE_MEDIUM_MAX_MM - m;
  }
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

// 반환: 배경을 새로 학습했으면 true
bool learnBackground(bool readapt) {
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
    return false;
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
  }
  if (got > 0) {
    for (int i = 0; i < IMG_N; i++) {
      bgR[i] = (acc[i * 3] + got / 2) / got;
      bgG[i] = (acc[i * 3 + 1] + got / 2) / got;
      bgB[i] = (acc[i * 3 + 2] + got / 2) / got;
    }
    computePlateStats();
    isBgReady = true;
    bgVersion++;
  }
  free(acc);
  resetDetectionState();

  if (got > 0) {
    EVENT("[BG] 배경 학습 완료 (%d프레임 평균) 판 RGB=(%.0f,%.0f,%.0f) 게인 R%.2f G%.2f B%.2f",
          got, plateR, plateG, plateB, gainR, gainG, gainB);
  } else {
    EVENT("[BG] 배경 학습 실패 — 카메라 프레임을 받지 못함");
  }
  loopGapSkip = true;   // 이 약 3초는 메인 루프 지연으로 세지 않는다
  return got > 0;
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
// 12-1. 크기 측정 (원근 보정 + 옆면 제거 + 최소 외접 사각형)
// ==========================================
//  tools/size_check.py 의 measure()와 같은 계산 (학습 샘플로 검증·보정할 때 그 스크립트를 쓴다)
const float IMG_CXF = IMG_W / 2.0f, IMG_CYF = IMG_H / 2.0f;   // 원근 보정 기준점 (영상 중심)
#define CALIPER_N ((89 / SIZE_CALIPER_STEP) + 1)              // 0°, 3°, … 87°
static float rowG[IMG_H], rowY[IMG_H], rowHx[IMG_H], rowHy[IMG_H];   // 행별 확대율 · 보정 y · 픽셀 반폭
static float calCos[CALIPER_N], calSin[CALIPER_N];

static void initSizeTables() {
  for (int y = 0; y < IMG_H; y++) {
    float dv = y - IMG_CYF;
    float g = 1.0f / (1.0f + SIZE_PERSP_P * dv);
    rowG[y] = g;
    rowY[y] = SIZE_Y_SCALE * dv * g;
    rowHx[y] = 0.5f * g;
    rowHy[y] = 0.5f * SIZE_Y_SCALE * g * g;
  }
  for (int k = 0; k < CALIPER_N; k++) {
    float a = k * SIZE_CALIPER_STEP * (float)M_PI / 180.0f;
    calCos[k] = cosf(a);
    calSin[k] = sinf(a);
  }
}

// 보정 좌표 → 화면 좌표 (디버그 화면에 추정 윗면을 그릴 때)
static void rectToImg(float X, float Y, float &x, float &y) {
  float dv = Y / (SIZE_Y_SCALE - SIZE_PERSP_P * Y);
  float g = 1.0f / (1.0f + SIZE_PERSP_P * dv);
  x = IMG_CXF + X / g;
  y = IMG_CYF + dv;
}

struct MBox { int x0, y0, x1, y1; };   // measMask가 들어 있는 범위 (ROI 안)

static inline void boxAdd(MBox &bx, int x, int y) {
  if (x < bx.x0) bx.x0 = x;
  if (x > bx.x1) bx.x1 = x;
  if (y < bx.y0) bx.y0 = y;
  if (y > bx.y1) bx.y1 = y;
}

// 그림자 필터 없이 본 전경(fgRaw 비트 2)의 3x3 다수결
static inline bool weakAt(int i) {
  int s = 0;
  for (int dy = -IMG_W; dy <= IMG_W; dy += IMG_W)
    for (int dx = -1; dx <= 1; dx++) s += (fgRaw[i + dy + dx] >> 1) & 1;
  return s >= 5;
}

// 검정 보강 (BLACK_GROW_COLS 설명 참고): 덩어리에서 시작해 '열별 바닥보다 위'인 약한 전경으로 키운다
static int growBlack(MBox &bx) {
  static int16_t colBot[IMG_W], winBot[IMG_W];
  static const int DX[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static const int DY[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  for (int x = 0; x < IMG_W; x++) colBot[x] = -1;
  int tail = 0;
  for (int y = bx.y0; y <= bx.y1; y++) {
    for (int x = bx.x0; x <= bx.x1; x++) {
      int i = y * IMG_W + x;
      if (measMask[i]) { colBot[x] = y; bfsQueue[tail++] = i; }
    }
  }
  for (int x = 0; x < IMG_W; x++) {
    int m = -1;
    for (int k = max(0, x - BLACK_GROW_COLS); k <= min(IMG_W - 1, x + BLACK_GROW_COLS); k++) m = max(m, (int)colBot[k]);
    winBot[x] = m;
  }
  int head = 0, added = 0;
  while (head < tail) {
    int i = bfsQueue[head++];
    int x = i % IMG_W, y = i / IMG_W;
    for (int k = 0; k < 8; k++) {
      int nx = x + DX[k], ny = y + DY[k];
      if (nx < ROI_X0 || nx > ROI_X1 || ny < ROI_Y0 || ny > ROI_Y1) continue;
      int n = ny * IMG_W + nx;
      if (measMask[n] || ny > winBot[nx] || !weakAt(n)) continue;
      measMask[n] = 1;
      bfsQueue[tail++] = n;
      added++;
      if (fgMap[n] < 3) fgMap[n] = 4;   // 화면 마스크에 보강 픽셀로 표시
      boxAdd(bx, nx, ny);
    }
  }
  return added;
}

// 3x3 열림(침식 후 팽창): 배경 가장자리 잡음으로 생긴 1~2px 가는 선·돌기 제거
static void openMask(const MBox &bx) {
  memset(measTmp, 0, IMG_N);
  for (int y = bx.y0; y <= bx.y1; y++) {
    for (int x = bx.x0; x <= bx.x1; x++) {
      int i = y * IMG_W + x;
      measTmp[i] = measMask[i - IMG_W - 1] & measMask[i - IMG_W] & measMask[i - IMG_W + 1] &
                   measMask[i - 1] & measMask[i] & measMask[i + 1] &
                   measMask[i + IMG_W - 1] & measMask[i + IMG_W] & measMask[i + IMG_W + 1];
    }
  }
  for (int y = bx.y0; y <= bx.y1; y++) {
    for (int x = bx.x0; x <= bx.x1; x++) {
      int i = y * IMG_W + x;
      if (!measMask[i]) continue;
      measMask[i] = measTmp[i - IMG_W - 1] | measTmp[i - IMG_W] | measTmp[i - IMG_W + 1] |
                    measTmp[i - 1] | measTmp[i] | measTmp[i + 1] |
                    measTmp[i + IMG_W - 1] | measTmp[i + IMG_W] | measTmp[i + IMG_W + 1];
    }
  }
}

static int floodMask(int seed, uint8_t from, uint8_t to) {
  static const int D[8] = {-IMG_W - 1, -IMG_W, -IMG_W + 1, -1, 1, IMG_W - 1, IMG_W, IMG_W + 1};
  int head = 0, tail = 0;
  bfsQueue[tail++] = seed;
  measMask[seed] = to;
  while (head < tail) {
    int i = bfsQueue[head++];
    for (int k = 0; k < 8; k++) {   // ROI 밖은 measMask가 항상 0
      int n = i + D[k];
      if (measMask[n] == from) { measMask[n] = to; bfsQueue[tail++] = n; }
    }
  }
  return tail;
}

// 열림으로 떨어져 나간 조각을 버리고 가장 큰 덩어리만 남긴다. 반환: 남은 면적
static int keepLargest(MBox &bx) {
  int bestSeed = -1, bestArea = 0;
  for (int y = bx.y0; y <= bx.y1; y++) {
    for (int x = bx.x0; x <= bx.x1; x++) {
      int i = y * IMG_W + x;
      if (measMask[i] != 1) continue;
      int a = floodMask(i, 1, 2);
      if (a > bestArea) { bestArea = a; bestSeed = i; }
    }
  }
  if (bestSeed < 0) return 0;
  floodMask(bestSeed, 2, 3);
  MBox nb = {IMG_W, IMG_H, -1, -1};
  for (int y = bx.y0; y <= bx.y1; y++) {
    for (int x = bx.x0; x <= bx.x1; x++) {
      int i = y * IMG_W + x;
      measMask[i] = measMask[i] == 3;
      if (measMask[i]) boxAdd(nb, x, y);
    }
  }
  bx = nb;
  return bestArea;
}

// 보정 좌표에서 각도를 바꿔 가며 외접 사각형을 대고, 옆면 몫(side × 아래 방향 성분)을 뺀 넓이가 가장 작은 것 = 윗면
//   옆면은 윗면을 아래로 side만큼 늘린 모양이라, 어느 방향의 폭에서든 'side × 그 방향의 세로 성분'만큼만 커진다.
static void caliperMeasure(BlobInfo &b, const MBox &bx, float side) {
  int ne = 0;   // 가장자리 픽셀만 (극값은 항상 가장자리에서 나온다)
  for (int y = bx.y0; y <= bx.y1; y++) {
    for (int x = bx.x0; x <= bx.x1; x++) {
      int i = y * IMG_W + x;
      if (measMask[i] && (!measMask[i - 1] || !measMask[i + 1] || !measMask[i - IMG_W] || !measMask[i + IMG_W]))
        bfsQueue[ne++] = i;
    }
  }
  if (ne < 3) return;
  float bestA = 1e30f, bU0 = 0, bV0 = 0, bLu = 0, bLv = 0;
  int bestK = 0;
  for (int k = 0; k < CALIPER_N; k++) {
    float c = calCos[k], s = calSin[k];
    float u0 = 1e9f, u1 = -1e9f, v0 = 1e9f, v1 = -1e9f;
    for (int e = 0; e < ne; e++) {
      int i = bfsQueue[e];
      int x = i % IMG_W, y = i / IMG_W;
      float X = (x - IMG_CXF) * rowG[y], Y = rowY[y];
      float pu = X * c + Y * s, pv = -X * s + Y * c;
      float ru = rowHx[y] * c + rowHy[y] * s, rv = rowHx[y] * s + rowHy[y] * c;
      if (pu - ru < u0) u0 = pu - ru;
      if (pu + ru > u1) u1 = pu + ru;
      if (pv - rv < v0) v0 = pv - rv;
      if (pv + rv > v1) v1 = pv + rv;
    }
    float lu = max(1.0f, u1 - u0 - side * s), lv = max(1.0f, v1 - v0 - side * c);
    if (lu * lv < bestA) { bestA = lu * lv; bestK = k; bU0 = u0; bV0 = v0; bLu = lu; bLv = lv; }
  }
  b.lenMm = max(bLu, bLv) * 35.0f / SIZE_PX_PER_35MM;
  b.widMm = min(bLu, bLv) * 35.0f / SIZE_PX_PER_35MM;
  // 추정 윗면 꼭짓점: u ∈ [u0, u0+lu], v ∈ [v0, v0+lv] (옆면은 u·v의 큰 쪽 끝에 붙어 있었음)
  float c = calCos[bestK], s = calSin[bestK];
  const float us[4] = {bU0, bU0 + bLu, bU0 + bLu, bU0}, vs[4] = {bV0, bV0, bV0 + bLv, bV0 + bLv};
  for (int k = 0; k < 4; k++) rectToImg(us[k] * c - vs[k] * s, us[k] * s + vs[k] * c, b.boxX[k], b.boxY[k]);
}

// 선택 덩어리(fgMap >= 3) → [검정이면 보강] → 열림 → 가장 큰 덩어리 → 윗면 크기(mm)
static void measureSize(BlobInfo &b, const ColorInfo &ci) {
  b.lenMm = b.widMm = 0;
  b.grown = 0;
  b.sideCut = false;
  if (!measMask || !measTmp || b.area <= 0) return;
  memset(measMask, 0, IMG_N);
  MBox bx = {b.minX, b.minY, b.maxX, b.maxY};
  for (int y = bx.y0; y <= bx.y1; y++)
    for (int x = bx.x0; x <= bx.x1; x++)
      if (fgMap[y * IMG_W + x] >= 3) measMask[y * IMG_W + x] = 1;
  if (ci.code == 2) b.grown = growBlack(bx);
  openMask(bx);
  if (keepLargest(bx) == 0) return;
  b.sideCut = ci.code != 1;   // 흰색은 옆면이 잡히지 않는다 (판과 비슷한 회색이라 그림자 필터에 빠짐)
  caliperMeasure(b, bx, b.sideCut ? SIDE_FACE_PX : 0.0f);
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
  if (v == 4) return 0xA81F;      // 검정 보강 (반사로 회색이 된 검정 윗면을 되살린 픽셀)
  if (v == 2) return 0x8410;      // 다른 덩어리
  if (detState == ST_CONFIRMED && footMask[i]) return 0x7800;  // 확정 위치 중 비어 보이는 부분
  if (fgRaw[i] & 1) return 0x2945;   // 필터로 제거된 노이즈
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

  // 사각형 (노랑=안정화 중, 초록=확정, 빨강=무효/경계 접촉)
  //   유효한 덩어리: 원근·옆면을 보정해 추정한 '윗면' / 무효: 덩어리의 모멘트 사각형
  if (best.area >= MIN_BLOB_AREA / 2) {
    uint16_t c = !best.valid ? 0xF800 : (detState == ST_CONFIRMED ? 0x07E0 : 0xFFE0);
    int px[4], py[4];
    if (best.lenMm > 0) {
      for (int k = 0; k < 4; k++) { px[k] = lroundf(best.boxX[k]); py[k] = lroundf(best.boxY[k]); }
    } else {
      float a = best.angleDeg * M_PI / 180.0f;
      float ux = cosf(a), uy = sinf(a), vx = -uy, vy = ux;
      float hl = best.len / 2, hw = best.wid / 2;
      const int SL[4] = {1, 1, -1, -1}, SW[4] = {1, -1, -1, 1};
      for (int k = 0; k < 4; k++) {
        px[k] = lroundf(best.cx + SL[k] * hl * ux + SW[k] * hw * vx);
        py[k] = lroundf(best.cy + SL[k] * hl * uy + SW[k] * hw * vy);
      }
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
  accLenN = 0;
  accFrames = 0;
}

static void accumulate(const BlobInfo &b, const ColorInfo &ci) {
  accColorVotes[ci.code]++;
  accRgb[ci.code][0] += ci.r; accRgb[ci.code][1] += ci.g; accRgb[ci.code][2] += ci.b;
  accRgbN[ci.code]++;
  if (b.lenMm > 0 && accLenN < ACC_LEN_MAX) accLen[accLenN++] = b.lenMm;
  accFrames++;
}

// 안정화 프레임들의 추정 길이 중앙값 (한두 프레임이 손·흔들림으로 튀어도 영향 없음)
static float medianLen() {
  if (accLenN <= 0) return 0;
  float t[ACC_LEN_MAX];
  memcpy(t, accLen, accLenN * sizeof(float));
  for (int i = 1; i < accLenN; i++) {
    float v = t[i];
    int j = i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  return accLenN % 2 ? t[accLenN / 2] : (t[accLenN / 2 - 1] + t[accLenN / 2]) / 2;
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
  // 추정 길이가 확정된 크기와 확실히 다름 — 놓는 도중(손에 가려지거나 기울어진 채) 확정되었다가
  // 물체가 제자리를 잡은 경우도 여기서 바로잡는다 (면적 변화가 25%에 못 미쳐도)
  bool sizeChanged = sizeClearlyDiffers(b.lenMm, curSizeCode);
  return areaChanged || colorChanged || sizeChanged;
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

  float lenMm = medianLen();
  uint16_t size = classifySizeByLen(lenMm);
  if (size == 0) return false;

  curColorCode = best;
  curSizeCode = size;
  curLenMm = lenMm;
  curR = accRgb[best][0] / accRgbN[best];
  curG = accRgb[best][1] / accRgbN[best];
  curB = accRgb[best][2] / accRgbN[best];
  detectCount++;

  // 한 번에 기록: 그동안 PLC 요청은 기다렸다가 처리된다 → 한 번의 읽기로 항상 같은 판정 결과의 값들을 받는다
  mbLock();
  setReg(REG_COLOR_CODE, curColorCode);
  setReg(REG_R_VAL, (uint16_t)curR);
  setReg(REG_G_VAL, (uint16_t)curG);
  setReg(REG_B_VAL, (uint16_t)curB);
  setReg(REG_DETECT_CNT, detectCount);
  setReg(REG_SIZE_CODE, curSizeCode);
  setReg(REG_RATIO_X100, (uint16_t)lroundf(curLenMm / 35.0f * 100));
  setReg(REG_PRESENT, 1);
  mbUnlock();
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
      LOG("[DBG] 안정화 %d/%d 면적=%d 추정 %.1fx%.1fmm%s 보강%d 색=%d(%s) 유채%.0f%% 밝기비%.2f\n",
          stableCount, STABLE_FRAMES, best.area, best.lenMm, best.widMm, best.sideCut ? "(옆면 보정)" : "",
          best.grown, ci.code, getColorName(ci.code), ci.chromaShare * 100, ci.vRatio);

      if (stableCount >= STABLE_FRAMES && millis() - settleStart >= STABLE_MIN_MS) {
        if (tryConfirm()) {
          detState = ST_CONFIRMED;
          saveFootprint(best);
          EVENT("[#%u] 색상 %u:%s (R:%d G:%d B:%d) | 크기 %u:%s (추정 길이 %.0fmm)",
                detectCount, curColorCode, getColorName(curColorCode), curR, curG, curB,
                curSizeCode, getSizeName(curSizeCode), curLenMm);
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
          EVENT("다른 물체로 교체됨 (면적 %d→%d, 색 %s→%s, 길이 %.0f→%.0fmm) → 다시 판정",
                confBlob.area, best.area, getColorName(curColorCode), getColorName(ci.code), curLenMm, best.lenMm);
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
// 14-2. PLC 명령 (40009)
// ==========================================
//  PLC가 40009(오프셋 8)에 FC06으로 명령 코드를 '한 번' 쓰면 장치가 실행하고, 같은 레지스터에 진행 상태를 돌려준다.
//  웹 화면의 명령 버튼(/cmd?c=N)도 이 레지스터에 같은 값을 써서 똑같이 동작한다.
//
//  명령 (PLC가 쓰는 값)
//    1  초기화           배경 재학습 + 감지 결과(40001~40008)·누적 감지 횟수를 0으로.   판이 비어 있어야 함, 약 3초
//    2  배경 재학습      배경만 다시 학습. 마지막 결과·누적 횟수는 유지(40003만 0).     판이 비어 있어야 함, 약 3초
//    3  감지 결과 초기화  감지 결과·누적 감지 횟수를 0으로 (배경 유지). 판 위 물체는 #1로 다시 판정.  즉시
//    4  다시 판정        확정을 취소하고 판 위 물체를 처음부터 다시 판정 (누적 횟수는 이어서 +1).   즉시
//
//  상태 (장치가 쓰는 값)
//    0        대기 — 이전 명령 완료. 새 명령을 쓸 수 있음
//    1~4      PLC가 쓴 명령을 장치가 아직 읽지 않음 (보통 0.2초 안에 100+N으로 바뀜)
//    100+N    N번 명령 실행 중 (101~104)
//    200+N    N번 명령 실패 (201~204). PLC가 0이나 새 명령을 쓸 때까지 유지. 이유는 웹 '감지 기록'
//    299      지원하지 않는 값을 씀
//
//  1·2번은 판 위에 물체가 보이면(유효한 덩어리) 최대 CMD_EMPTY_WAIT_MS 동안 치워지기를 기다리고, 그래도 있으면 실패(201/202).
//  물체가 있는 채로 배경을 학습하면 그 물체가 배경이 되어, 치운 뒤 그 자리가 '물체'로 보이고 같은 자리의 물체는 안 보인다.
//  (조명이 크게 바뀌어 화면 전체가 달라진 경우는 ROI 경계에 닿는 덩어리라 물체로 보지 않으므로 그대로 실행된다)
#define CMD_INIT      1
#define CMD_RELEARN   2
#define CMD_RESET     3
#define CMD_REJUDGE   4
#define CMD_MAX       4
#define CMD_RUNNING   100
#define CMD_FAILED    200
#define CMD_UNKNOWN   299
const unsigned long CMD_EMPTY_WAIT_MS = 2000;   // 1·2번: 판이 비기를 기다리는 최대 시간 (로봇이 막 치운 직후 대비)

static const char *cmdName(uint16_t c) {
  switch (c) {
    case CMD_INIT: return "초기화";
    case CMD_RELEARN: return "배경 재학습";
    case CMD_RESET: return "감지 결과 초기화";
    case CMD_REJUDGE: return "다시 판정";
    default: return "알 수 없음";
  }
}

// 웹 표시용 마지막 명령 정보 (status_handler가 cmdMux 안에서 복사)
struct CmdState {
  uint32_t seq = 0;              // 끝난 명령 수 — 웹이 결과가 나왔는지 알아채는 용도
  uint16_t code = 0;             // 마지막(또는 실행 중) 명령 값
  uint8_t  phase = 0;            // 0 없음, 1 실행 중, 2 완료, 3 실패
  bool     fromWeb = false;
  bool     byObject = false;     // 실패 이유가 '판 위 물체' (웹에서 그래도 실행할지 묻기 위함)
  unsigned long startMs = 0, endMs = 0;
  char     msg[112] = "";
} cmdSt;
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

uint16_t cmdDevVal = 0;          // 장치가 마지막으로 40009에 쓴 값 — 지금 값이 이것과 다르면 PLC(또는 웹)가 새로 쓴 것
volatile int  webCmdCode = 0;    // 웹 버튼이 쓴 명령 (출처 표시용)
volatile bool webCmdForce = false;   // 웹에서 '그래도 실행' — 판 비었는지 확인을 건너뜀
int  pendCmd = 0;                // 1·2번: 판이 비기를 기다리는 중인 명령
bool pendForce = false;
uint32_t pendFrame = 0;
unsigned long pendDeadline = 0;

static uint16_t readCmdReg() { return ENABLE_MODBUS ? mb.Hreg(REG_COMMAND) : regMirror[REG_COMMAND]; }

// 감지 결과와 누적 감지 횟수를 0으로 (40001~40008) — 한 번에 기록
static void resetResults() {
  detectCount = 0;
  curColorCode = curSizeCode = 0;
  curLenMm = 0;
  curR = curG = curB = 0;
  mbLock();
  for (int i = 0; i < REG_COMMAND; i++) setReg(i, 0);
  mbUnlock();
}

static void cmdBegin(uint16_t code, bool fromWeb) {
  portENTER_CRITICAL(&cmdMux);
  cmdSt.code = code;
  cmdSt.phase = 1;
  cmdSt.fromWeb = fromWeb;
  cmdSt.byObject = false;
  cmdSt.startMs = millis();
  cmdSt.endMs = 0;
  cmdSt.msg[0] = 0;
  portEXIT_CRITICAL(&cmdMux);
}

// 명령 종료: 결과를 40009에 쓴다 (단, 실행하는 동안 PLC가 다른 값을 썼으면 덮어쓰지 않고 그 값을 다음에 처리)
static void cmdEnd(uint16_t regVal, bool ok, bool byObject, const char *fmt, ...) __attribute__((format(printf, 4, 5)));
static void cmdEnd(uint16_t regVal, bool ok, bool byObject, const char *fmt, ...) {
  char m[sizeof(cmdSt.msg)];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(m, sizeof(m), fmt, ap);
  va_end(ap);
  mbLock();
  if (readCmdReg() == cmdDevVal) {
    setReg(REG_COMMAND, regVal);
    cmdDevVal = regVal;
  }
  mbUnlock();
  unsigned long now = millis();
  portENTER_CRITICAL(&cmdMux);
  cmdSt.phase = ok ? 2 : 3;
  cmdSt.byObject = byObject;
  cmdSt.endMs = now;
  memcpy(cmdSt.msg, m, sizeof(m));
  cmdSt.seq++;
  portEXIT_CRITICAL(&cmdMux);
  EVENT("[명령] %u %s %s (%.1f초): %s", cmdSt.code, cmdName(cmdSt.code), ok ? "완료" : "실패",
        (now - cmdSt.startMs) / 1000.0f, m);
}

// 웹 버튼: PLC와 똑같이 40009에 명령 값을 쓴다
static void webCommand(int code, bool force) {
  webCmdForce = force;
  webCmdCode = code;
  mbLock();
  setReg(REG_COMMAND, code);
  mbUnlock();
}

// 메인 루프 시작마다: 40009에 새 값이 쓰였는지 보고 접수
static void pollCommand() {
  mbLock();
  uint16_t v = readCmdReg();
  if (v == cmdDevVal) { mbUnlock(); return; }   // 새로 쓴 값 없음
  uint16_t ack = v == 0 ? 0 : (v <= CMD_MAX ? CMD_RUNNING + v : CMD_UNKNOWN);
  setReg(REG_COMMAND, ack);                     // 받자마자 '실행 중'으로 (PLC가 접수를 바로 확인)
  cmdDevVal = ack;
  mbUnlock();

  bool web = webCmdCode != 0 && webCmdCode == (int)v;
  bool force = web && webCmdForce;
  webCmdCode = 0;
  webCmdForce = false;
  const char *src = web ? "웹" : "PLC";

  if (pendCmd) {   // 판이 비기를 기다리던 명령은 새로 쓴 값으로 대체된다
    EVENT("[명령] %d %s 취소 — %s가 40009에 %u을(를) 씀", pendCmd, cmdName(pendCmd), src, v);
    pendCmd = 0;
    portENTER_CRITICAL(&cmdMux);
    cmdSt.phase = 3;
    cmdSt.endMs = millis();
    snprintf(cmdSt.msg, sizeof(cmdSt.msg), "새로 쓴 값 %u으로 취소", v);
    cmdSt.seq++;
    portEXIT_CRITICAL(&cmdMux);
  }
  if (v == 0) return;                           // 0 쓰기 = 실패 표시 지우기 (또는 대기 중 명령 취소)

  cmdBegin(v, web);
  if (v > CMD_MAX) {
    cmdEnd(CMD_UNKNOWN, false, false, "%s가 지원하지 않는 값 %u을 씀 (명령은 1~%d)", src, v, CMD_MAX);
    return;
  }
  EVENT("[명령] %s → %u %s", src, v, cmdName(v));
  switch (v) {
    case CMD_INIT:
    case CMD_RELEARN:
      if (!cameraOk) {
        cmdEnd(CMD_FAILED + v, false, false, "카메라 초기화 실패 상태라 배경을 학습할 수 없음");
        return;
      }
      pendCmd = v;              // 판이 비었는지는 명령을 받은 뒤 찍은 프레임으로 확인 (servicePendingCommand)
      pendForce = force || !isBgReady;   // 배경이 아직 없으면 확인할 방법이 없으므로 바로 학습
      pendFrame = frameNo;
      pendDeadline = millis() + CMD_EMPTY_WAIT_MS;
      break;
    case CMD_RESET:
      resetResults();
      resetDetectionState();
      cmdEnd(0, true, false, "감지 결과(40001~40008)와 누적 감지 횟수를 0으로. 판 위 물체가 있으면 #1로 다시 판정");
      break;
    case CMD_REJUDGE:
      resetDetectionState();
      cmdEnd(0, true, false, "확정 취소 — 판 위 물체를 다시 판정 (새 결과는 누적 %u회 다음 번호)", detectCount);
      break;
  }
}

// 메인 루프에서 프레임 처리 뒤마다: 1·2번 명령을 판이 비었을 때 실행
static void servicePendingCommand() {
  if (!pendCmd) return;
  bool late = (long)(millis() - pendDeadline) >= 0;
  if (!pendForce) {
    if (frameNo < pendFrame + 2) {              // 명령을 받은 뒤 찍힌 프레임이 아직 없음
      if (late) {
        int c = pendCmd;
        pendCmd = 0;
        cmdEnd(CMD_FAILED + c, false, false, "카메라 프레임을 받지 못해 판 상태를 확인할 수 없음");
      }
      return;
    }
    if (lastBestValid) {
      if (!late) return;                        // 물체가 치워지기를 기다림
      int c = pendCmd;
      pendCmd = 0;
      cmdEnd(CMD_FAILED + c, false, true, "판 위에 물체가 있음 (%.0f초 기다림). 판을 비우고 다시 쓰세요",
             CMD_EMPTY_WAIT_MS / 1000.0f);
      return;
    }
  }
  int c = pendCmd;
  pendCmd = 0;
  if (!learnBackground(true)) {
    cmdEnd(CMD_FAILED + c, false, false, "배경 학습 실패 — 카메라 프레임을 받지 못함");
    return;
  }
  if (c == CMD_INIT) {
    resetResults();
    cmdEnd(0, true, false, "배경 재학습 + 감지 결과(40001~40008)·누적 감지 횟수 0");
  } else {
    cmdEnd(0, true, false, "배경 재학습 (마지막 결과·누적 %u회 유지)", detectCount);
  }
}

// ==========================================
// 14-1. 학습 데이터 수집 (웹의 크기/색상 학습 버튼)
// ==========================================
//   버튼을 누르면 '확정'된 물체의 원본 영상·전경 마스크·측정값을 정답(라벨)과 함께 PSRAM에 저장한다.
//   판정 기준은 바로 바뀌지 않는다 — 모은 데이터로 보정값을 계산해 펌웨어에 반영하는 용도.
//   같은 물체(감지 번호가 같음)에 버튼을 또 누르면 새로 찍지 않고 그 샘플의 라벨만 추가/수정한다.
//   PSRAM은 전원이 꺼지면 지워진다 → 끝나면 /samples.zip 으로 내려받을 것.
#define SMP_MAX    100
#define ROI_W      (ROI_X1 - ROI_X0 + 1)   // 120
#define ROI_H      (ROI_Y1 - ROI_Y0 + 1)   // 90
#define ROI_N      (ROI_W * ROI_H)
#define BGSNAP_MAX 8
const unsigned long BGSNAP_REFRESH_MS = 10UL * 60 * 1000;   // 배경은 조금씩 추적되므로 10분마다 새 스냅샷
static_assert(ROI_N % 4 == 0, "mask packing");

struct Sample {
  uint16_t id = 0, det = 0;        // 샘플 번호, 장치 감지 번호(detectCount)
  uint32_t ms = 0;                 // 저장 시각 (가동 ms)
  int8_t   bg = -1;                // 배경 스냅샷 번호 (-1 = 없음)
  uint8_t  sizeLabel = 0, colorLabel = 0;   // 정답 (0 = 미입력)
  uint8_t  predSize = 0, predColor = 0;     // 장치 판정 = PLC로 보낸 확정 결과
  float    confLenMm = 0;          // 확정에 쓴 추정 길이 (mm)
  int16_t  confRgb[3] = {0};
  uint8_t  frameSize = 0;          // 저장한 프레임 한 장의 추정 길이로 본 크기
  BlobInfo blob;                   // 저장한 프레임의 덩어리 측정값 (좌표는 전체 화면 기준)
  ColorInfo color;
  float    plate[3] = {0}, gain[3] = {0};
  uint8_t  raw[ROI_N * 2];         // ROI 원본 RGB565 (카메라 바이트 순서: 상위 바이트 먼저)
  uint8_t  mask[ROI_N / 4];        // 2비트/픽셀: 0 배경, 1 다른 덩어리, 2 선택 덩어리, 3 검정 보강 픽셀
};
struct BgSnap { uint8_t ver; uint32_t ms; uint8_t *rgb; };   // rgb: ROI_N×3 (R,G,B 순)

Sample *smp[SMP_MAX];
int smpCount = 0;
uint16_t smpNextId = 1;
BgSnap bgSnap[BGSNAP_MAX];
int bgSnapCount = 0;
SemaphoreHandle_t smpMutex = nullptr;

// 웹 요청 → 메인 루프 (확정 상태의 프레임에서 저장)
struct LearnReq {
  volatile bool pending = false, done = false;
  char kind = 's';                 // 's' 크기, 'c' 색상
  uint8_t value = 0;
  unsigned long deadline = 0;
  char result[300];
} learnReq;

static const char SIZE_KEYS[] = "-SML";      // 파일 이름용 (소/중/대)
static const char COLOR_KEYS[] = "-WKRYGB";  // 흰/검/빨/노/초/파

static int findSample(uint16_t id) {
  for (int k = 0; k < smpCount; k++) if (smp[k]->id == id) return k;
  return -1;
}

// 현재 배경의 스냅샷 번호 (없거나 오래됐으면 새로 저장). 실패하면 가장 최근 것, 그것도 없으면 -1
static int bgSnapForNow(unsigned long now) {
  int last = bgSnapCount - 1;
  if (last >= 0 && bgSnap[last].ver == bgVersion && now - bgSnap[last].ms < BGSNAP_REFRESH_MS) return last;
  if (bgSnapCount >= BGSNAP_MAX) return last;
  uint8_t *p = (uint8_t *)heap_caps_malloc(ROI_N * 3, MALLOC_CAP_SPIRAM);
  if (!p) return last;
  int k = 0;
  for (int y = ROI_Y0; y <= ROI_Y1; y++) {
    for (int x = ROI_X0; x <= ROI_X1; x++) {
      int i = y * IMG_W + x;
      p[k++] = bgR[i]; p[k++] = bgG[i]; p[k++] = bgB[i];
    }
  }
  bgSnap[bgSnapCount] = {bgVersion, now, p};
  return bgSnapCount++;
}

static void learnFinish(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void learnFinish(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(learnReq.result, sizeof(learnReq.result), fmt, ap);
  va_end(ap);
  learnReq.pending = false;
  learnReq.done = true;
}

// processFrame() 끝에서 호출: 학습 버튼 요청이 있으면 확정 상태일 때 저장
static void serviceLearnRequest(const uint8_t *buf, const BlobInfo &best, const ColorInfo &ci) {
  if (!learnReq.pending) return;
  unsigned long now = millis();
  if (detState != ST_CONFIRMED || !best.valid) {
    static const char *ST_KO[] = {"판 비어 있음", "안정화 중", "확정"};
    if ((long)(now - learnReq.deadline) > 0) {
      learnFinish("{\"ok\":false,\"error\":\"물체가 확정되지 않았습니다 (현재: %s%s). 물체를 놓고 초록 사각형이 된 뒤 누르세요\"}",
                  ST_KO[detState], best.touchesBorder ? ", 분석 영역 경계에 걸침" : "");
    }
    return;
  }

  xSemaphoreTake(smpMutex, portMAX_DELAY);
  Sample *s = smpCount > 0 ? smp[smpCount - 1] : nullptr;
  bool merged = s && s->det == detectCount;   // 같은 물체 → 라벨만 추가/수정
  if (!merged) {
    if (smpCount >= SMP_MAX) {
      xSemaphoreGive(smpMutex);
      learnFinish("{\"ok\":false,\"error\":\"저장 공간이 가득 참 (%d개). ZIP으로 받은 뒤 전체 삭제하세요\"}", SMP_MAX);
      return;
    }
    if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) < sizeof(Sample) + 300000) {
      xSemaphoreGive(smpMutex);
      learnFinish("{\"ok\":false,\"error\":\"PSRAM 부족. ZIP으로 받은 뒤 전체 삭제하세요\"}");
      return;
    }
    void *mem = heap_caps_malloc(sizeof(Sample), MALLOC_CAP_SPIRAM);
    if (!mem) {
      xSemaphoreGive(smpMutex);
      learnFinish("{\"ok\":false,\"error\":\"메모리 할당 실패\"}");
      return;
    }
    s = new (mem) Sample();
    s->id = smpNextId++;
    s->det = detectCount;
    s->ms = now;
    s->predSize = curSizeCode;
    s->predColor = curColorCode;
    s->confLenMm = curLenMm;
    s->confRgb[0] = curR; s->confRgb[1] = curG; s->confRgb[2] = curB;
    s->frameSize = classifySizeByLen(best.lenMm);
    s->blob = best;
    s->color = ci;
    s->plate[0] = plateR; s->plate[1] = plateG; s->plate[2] = plateB;
    s->gain[0] = gainR; s->gain[1] = gainG; s->gain[2] = gainB;
    for (int y = 0; y < ROI_H; y++) {
      memcpy(s->raw + y * ROI_W * 2, buf + ((y + ROI_Y0) * IMG_W + ROI_X0) * 2, ROI_W * 2);
      for (int x = 0; x < ROI_W; x++) {
        uint8_t v = fgMap[(y + ROI_Y0) * IMG_W + x + ROI_X0];
        uint8_t lv = v == 0 ? 0 : v < 3 ? 1 : v == 4 ? 3 : 2;
        int k = y * ROI_W + x;
        s->mask[k >> 2] |= lv << ((k & 3) * 2);
      }
    }
    s->bg = bgSnapForNow(now);
    smp[smpCount++] = s;
  }
  bool isSize = learnReq.kind == 's';
  uint8_t label = learnReq.value;
  uint8_t prevLabel = isSize ? s->sizeLabel : s->colorLabel;
  if (isSize) s->sizeLabel = label; else s->colorLabel = label;
  uint8_t pred = isSize ? s->predSize : s->predColor;
  uint16_t id = s->id;
  float lenMm = s->confLenMm;
  int count = smpCount;
  xSemaphoreGive(smpMutex);

  const char *labelName = isSize ? getSizeName(label) : getColorName(label);
  const char *predName = isSize ? getSizeName(pred) : getColorName(pred);
  EVENT("[학습] 샘플 #%u %s 정답 %s / 판별 %s %s (길이 %.0fmm)%s", id, isSize ? "크기" : "색상", labelName, predName,
        label == pred ? "O" : "X", lenMm, merged ? (prevLabel ? " — 라벨 수정" : " — 같은 물체에 추가") : "");
  learnFinish("{\"ok\":true,\"id\":%u,\"merged\":%s,\"corrected\":%s,\"kind\":\"%s\",\"label\":%u,\"pred\":%u,"
              "\"labelName\":\"%s\",\"predName\":\"%s\",\"lenMm\":%.1f,\"count\":%d}",
              id, merged ? "true" : "false", merged && prevLabel ? "true" : "false", isSize ? "size" : "color",
              label, pred, labelName, predName, lenMm, count);
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
      bool weakFg = chromaFg || dSum >= DIFF_THRESH;   // 그림자 필터 없이 (검정 보강용)
      fgRaw[i] = (isFg ? 1 : 0) | (weakFg ? 2 : 0);

      if (isFg) {
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
      int s = (fgRaw[i - IMG_W - 1] & 1) + (fgRaw[i - IMG_W] & 1) + (fgRaw[i - IMG_W + 1] & 1) +
              (fgRaw[i - 1] & 1) + (fgRaw[i] & 1) + (fgRaw[i + 1] & 1) +
              (fgRaw[i + IMG_W - 1] & 1) + (fgRaw[i + IMG_W] & 1) + (fgRaw[i + IMG_W + 1] & 1);
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

  // 4단계: 색상 → 크기 (색에 따라 검정 보강·옆면 보정 여부가 달라짐)
  ColorInfo ci;
  if (best.area > 0) {
    measureBlob(best);
    best.valid = best.area >= MIN_BLOB_AREA && !best.touchesBorder;
    int count = floodComponent(best.seed, 2, 3, nullptr);
    if (best.valid) {
      classifyBlobColor(buf, count, ci);
      measureSize(best, ci);
    }
  }

  lastBestValid = best.valid;

  // 5단계: 확정된 물체 자리 점검 → 상태 머신 (안정화 → 확정 → 제거/교체)
  lastFoot = detState == ST_CONFIRMED ? checkFootprint(best) : FootCheck();
  updateState(best, ci, lastFoot);
  if (ENABLE_WEB) serviceLearnRequest(buf, best, ci);

  // 디버그 화면은 웹 서버가 시작된 뒤(= Wi-Fi가 한 번이라도 연결된 뒤)에만 그린다. 볼 사람이 없으면 CPU만 쓴다
  if (ENABLE_WEB && webStarted) renderDebug(fb, best, ci, fgRawCount, fgCount, components);
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
.bar{display:inline-block;height:8px;min-width:2px;border-radius:4px;background:var(--warn);vertical-align:middle}
td.b{width:45%}tr.rec td{background:#16351f}
.lk{color:var(--mute);font-weight:600;min-width:34px}
.lrow button{min-width:74px;padding:9px 12px;font-weight:600}
.dot{display:inline-block;width:11px;height:11px;border-radius:50%;margin-right:6px;border:1px solid #666;vertical-align:-1px}
td.x{color:var(--bad);font-weight:700}td.o{color:var(--ok)}
table.wrap td{white-space:normal;vertical-align:top}table.wrap td:first-child{width:56px}
table.cmdt td:nth-child(2),table.cmdt td:nth-child(5),table.cmdt td:last-child{white-space:nowrap}
ul.note{padding-left:18px}ul.note li{margin:3px 0}
.thumbs{display:flex;gap:8px;flex-wrap:wrap;margin-top:8px}
.thumbs img{width:240px;max-width:48%;aspect-ratio:4/3}
.cm td,.cm th{text-align:center;padding:4px 8px}
#lnTog{padding:3px 12px;font-size:12px;font-weight:600;border-radius:999px}
#lnTog.on{background:#1d3a5c;border-color:var(--acc);color:var(--acc)}
button:disabled{opacity:.5;cursor:wait}
summary{cursor:pointer;color:var(--mute);font-weight:600;font-size:13px}
details[open] summary{margin-bottom:10px}
</style></head><body>
<header><h1>ESP32-CAM 큐브 판별</h1><span id="mode" class="pill"></span><span id="conn" class="pill">연결 중…</span><button id="lnTog" onclick="lnToggle()" title="학습 데이터 수집 영역 보이기/숨기기 (페이지를 열 때마다 꺼진 상태로 시작)">학습 모드</button></header>
<main>
<section class="card wide">
 <h2>카메라 영상 <span class="sub">왼쪽: 카메라 + 측정 · 오른쪽: 전경 마스크</span></h2>
 <img id="s" alt="카메라 스트림">
 <div class="legend">사각형 = 원근·옆면을 보정해 추정한 물체 윗면 — 노랑: 안정화 중, 초록: 확정, 빨강: 무효(ROI 경계 접촉). 마스크 — 픽셀별 색 분류, 하늘색: 가장자리(판정 제외), 보라: 검정 보강(반사로 회색이 된 검정 윗면), 회색: 다른 덩어리, 진한 빨강: 확정 위치 중 비어 보이는 부분, 어두운 점: 제거된 노이즈</div>
 <div class="row"><button onclick="relearn()">배경 재학습</button>
 <a href="/capture" target="_blank">캡처(JPG)</a><a href="/raw.bmp" target="_blank">원본(BMP)</a><a href="/status" target="_blank">상태(JSON)</a></div>
</section>
<section class="card wide" id="lnCard" hidden>
 <h2>학습 데이터 수집 <span id="lnPill" class="pill">0개</span><span id="lnMem" class="sub"></span></h2>
 <p class="note" style="margin:0 0 6px">물체를 놓고 <b>초록 사각형(확정)</b>이 되면 정답 버튼을 누르세요. 원본 영상·마스크·측정값이 정답과 함께 장치 메모리에 저장되고, 장치 판정과 비교한 정확도가 아래에 표시됩니다.
 같은 물체에 크기·색상 버튼을 모두 누르면 한 샘플에 함께 기록되며, 잘못 누른 버튼은 다시 누르면 고쳐집니다.
 판정 기준은 바로 바뀌지 않습니다(모은 데이터로 보정값을 계산해 펌웨어에 반영). <b>전원이 꺼지면 지워지므로</b> 끝나면 ZIP으로 받아 두세요.</p>
 <div class="row lrow"><span class="lk">크기</span><button onclick="learn('s',1)">소 35</button><button onclick="learn('s',2)">중 55</button><button onclick="learn('s',3)">대 85</button></div>
 <div class="row lrow" id="lnColors"><span class="lk">색상</span></div>
 <p class="note" id="lnMsg"></p>
 <div class="thumbs" id="lnThumbs"></div>
 <div id="lnAcc"></div>
 <div class="row"><a href="/samples.zip">ZIP으로 모두 받기</a><button onclick="lnDel('last=1','마지막 샘플을 삭제할까요?')">마지막 샘플 삭제</button><button onclick="lnDel('all=1','저장된 샘플을 모두 삭제할까요? (ZIP으로 받아 두었는지 확인)')">전체 삭제</button></div>
 <details style="margin-top:10px"><summary>샘플 목록</summary><div class="tbl"><table>
 <thead><tr><th>#</th><th>크기 정답→판별</th><th>색상 정답→판별</th><th>길이(확정)</th><th>이 프레임 길이×폭</th><th>위치(x,y)</th><th>보강</th><th>영상</th><th></th></tr></thead>
 <tbody id="lnList"></tbody></table></div></details>
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
 <p class="note" id="mbNote">FC03(Read Holding Registers, 40001~) 또는 FC04(Read Input Registers, 30001~)로 오프셋 0부터 9개를 한 번에 읽으면 됩니다. 두 함수는 같은 값을 돌려줍니다. 오프셋은 0부터 세는 주소입니다(PLC 프레임에 실제로 들어가는 값, 예: 오프셋 3 = 40004/30004 = R 값). 40009는 PLC가 명령을 쓰는 자리입니다(FC06) — 아래 <b>PLC 명령</b> 참고.</p>
</section>
<section class="card wide" id="cmdCard">
 <h2>PLC 명령 (40009) <span id="cmdPill" class="pill">대기</span></h2>
 <p class="note" style="margin:0 0 8px">PLC가 <b>40009(오프셋 8)</b>에 FC06으로 명령 값을 <b>한 번</b> 쓰면 장치가 실행하고, 같은 레지스터에 진행 상태를 돌려줍니다 (100+명령 = 실행 중 → 0 = 완료, 200+명령 = 실패). 아래 <b>실행</b> 버튼도 PLC와 똑같이 40009에 값을 쓰므로 PLC 없이 시험할 수 있습니다.</p>
 <div class="tbl"><table class="wrap cmdt">
 <thead><tr><th>값</th><th>명령</th><th>하는 일</th><th>조건 · 걸리는 시간</th><th>실행 중 / 실패</th><th></th></tr></thead>
 <tbody>
 <tr><td class="v">1</td><td><b>초기화</b></td><td>배경을 다시 학습하고, 감지 결과(40001~40008)와 <b>누적 감지 횟수(40007)</b>를 모두 0으로 만듭니다. 작업(교대·로트)을 새로 시작할 때.</td><td>판이 비어 있어야 함 (물체가 보이면 최대 2초 기다림) · 약 3초</td><td>101 / 201</td><td><button onclick="cmd(1)">실행</button></td></tr>
 <tr><td class="v">2</td><td><b>배경 재학습</b></td><td>배경만 다시 학습합니다. 마지막 결과와 누적 감지 횟수는 그대로 두고, 40003(물체 있음)만 0이 됩니다. 조명이 바뀌었거나 빈 판인데 물체가 있다고 나올 때.</td><td>판이 비어 있어야 함 (최대 2초 기다림) · 약 3초</td><td>102 / 202</td><td><button onclick="cmd(2)">실행</button></td></tr>
 <tr><td class="v">3</td><td><b>감지 결과 초기화</b></td><td>감지 결과(40001~40008)와 누적 감지 횟수를 0으로 만듭니다. 배경은 그대로. 판 위에 물체가 있으면 그 물체가 #1로 다시 판정됩니다.</td><td>즉시</td><td>103 / 203</td><td><button onclick="cmd(3)">실행</button></td></tr>
 <tr><td class="v">4</td><td><b>다시 판정</b></td><td>지금 확정된 결과를 취소하고 판 위 물체를 처음부터 다시 판정합니다. 누적 감지 횟수는 이어서 +1 (새 결과로 받음).</td><td>즉시 (새 결과는 약 1초 뒤)</td><td>104 / 204</td><td><button onclick="cmd(4)">실행</button></td></tr>
 </tbody></table></div>
 <dl class="kv" id="cmdKv"></dl>
 <p class="note" id="cmdMsg"></p>
 <details style="margin-top:10px"><summary>40009 값의 의미와 PLC 프로그램 작성 방법</summary>
 <div class="tbl"><table class="wrap"><thead><tr><th>40009 값</th><th>의미</th><th>PLC가 할 일</th></tr></thead><tbody>
 <tr><td class="v">0</td><td>대기 — 이전 명령이 끝남</td><td>새 명령을 쓸 수 있음</td></tr>
 <tr><td class="v">1~4</td><td>명령이 쓰였지만 장치가 아직 읽지 않음 (보통 0.2초 안에 바뀜)</td><td>기다림</td></tr>
 <tr><td class="v">101~104</td><td>그 명령을 실행하는 중. 1·2번은 판이 비기를 최대 2초 기다린 뒤 약 3초 동안 배경을 학습 (이 동안에도 통신은 정상 응답)</td><td>기다림. 이 동안의 판정 결과로 새 물체를 판단하지 않음</td></tr>
 <tr><td class="v">201~204</td><td>그 명령이 실패해서 <b>아무것도 바뀌지 않음</b>. 이유는 웹 '감지 기록'의 [명령] 줄 (판 위 물체, 카메라 오류 등)</td><td>원인을 없앤 뒤 명령을 다시 쓰거나, 0을 써서 표시를 지움</td></tr>
 <tr><td class="v">299</td><td>지원하지 않는 값을 씀</td><td>PLC 프로그램의 명령 값 확인 후 0을 씀</td></tr>
 </tbody></table></div>
 <pre style="margin-top:8px">[명령 보내기]  40009가 0(또는 2xx)일 때 FC06으로 40009(오프셋 8)에 명령 값을 한 번 쓰기
[끝 기다리기]  40009를 계속 읽어서
                 0       → 완료 (1·3이면 이후 감지 횟수는 1부터 다시 셈)
                 101~104 → 실행 중, 계속 기다림
                 201~204 → 실패: 판을 비우는 등 원인을 없애고 다시 명령
               시간 초과는 10초 정도로 (1·2번: 최대 2초 대기 + 약 3초 학습)
프레임 예 (슬레이브 6):  명령 1 쓰기  06 06 00 08 00 01 C8 7F
                         0 쓰기      06 06 00 08 00 00 09 BF
                         40009 읽기  06 03 00 08 00 01 04 7F</pre>
 <ul class="note">
 <li>명령은 <b>한 번만</b> 쓰세요. 매 스캔마다 계속 쓰면 끝날 때마다 다시 실행됩니다.</li>
 <li>40001~40009를 FC16으로 한꺼번에 쓰지 마세요. 40009에 들어간 값이 명령으로 실행되고, 결과 레지스터도 덮어써집니다.</li>
 <li>실행 중에 다른 명령을 쓰면, 판이 비기를 기다리던 1·2번은 취소되고 새 명령이 실행됩니다. 배경 학습(약 3초) 도중이면 학습이 끝난 뒤 새 명령이 실행됩니다.</li>
 <li>초기화해도 학습 데이터(샘플), Wi-Fi·통신 설정, 통신 통계는 지워지지 않습니다.</li>
 <li>1·2번은 판 위에 물체가 보이면 실행하지 않습니다(물체가 배경이 되는 것을 막기 위함). 웹 버튼으로 실행했는데 물체가 없는데도 거부되면 '그래도 실행'을 고를 수 있습니다.</li>
 </ul>
 </details>
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
<section class="card wide">
 <h2>Wi-Fi <span id="wfPill" class="pill"></span></h2>
 <dl class="kv" id="wfKv"></dl>
 <div class="row"><button id="scanBtn" onclick="wifiScan()">주변 채널 검사</button><span class="sub" id="scanMsg">이 장치 위치에서 보이는 공유기의 채널과 신호를 검사합니다 (2~4초, 그동안 영상이 잠깐 멈출 수 있음)</span></div>
 <div id="scanRes"></div>
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
 ['길이비 ×100',v=>'추정 길이 ÷ 35mm = '+(v/100).toFixed(2)+` (약 ${Math.round(v*0.35)}mm)`],
 ['명령 / 상태',v=>cmdText(v)],
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
 if(fc==4)return (n>1?`${30001+a}~${30000+a+n} (${n}개)`:`${30001+a}`)+` = ${40001+a}${n>1?'~'+(40000+a+n):''}과 같은 값`;
 return `주소 ${a}, ${n}`;}
function render(j){
 const c=j.confirmed,m=j.modbus,regs=m.regs,st=STATES[j.state]||[j.state,''];
 pill($('state'),st[0],st[1]);
 if(c.count>0){
  $('sw').style.background=SW[c.color]||'';
  $('res').textContent=`${c.colorName} · ${c.sizeName}`;
  $('resSub').textContent=`#${c.count} · 색상 ${c.color} · 크기 ${c.size} · RGB(${c.rgb.join(', ')}) · 추정 길이 ${c.lenMm.toFixed(0)}mm`;
 }
 const b=j.blob;
 const meas=b.lenMm>0?` · 추정 ${b.lenMm.toFixed(0)}×${b.widMm.toFixed(0)}mm${b.grown?` (검정 보강 ${b.grown}px)`:''} → ${b.size}`:'';
 const rows=[['판 위 물체',regs[REG_PRESENT]?'<span class="pill ok">있음</span>':(c.count>0?'<span class="pill">없음 — 위 결과는 마지막 감지</span>':'<span class="pill">없음</span>')],
  ['현재 프레임',b.area>0?`덩어리 ${b.area}px · ${b.valid?'유효':'무효'+(b.touchesBorder?'(경계 접촉)':'(작음)')}${meas}`:'변화 없음'],
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
  else if(m.requests>m.ok)hint='오류 응답은 없는 주소(오프셋 9 이후 = 40010/30010 이후)나 지원하지 않는 함수 코드(FC03·FC04·FC06·FC16 외)를 요청했을 때 생깁니다.';
 }
 $('mbHint').textContent=hint;
 renderCmd(j.cmd);
 const cam=j.camera,w=j.wifi;
 kv($('sysKv'),[
  ['카메라',cam.ok?`<span class="pill ok">정상</span> ${PIDS[cam.pid]||''} (PID 0x${cam.pid.toString(16).toUpperCase()})`:'<span class="pill bad">초기화 실패</span> 배선/전원 확인 후 재부팅'],
  ['처리 속도',`${j.fps.toFixed(1)} fps · 처리 ${j.frame}프레임`],
  ['메인 루프 지연',j.loopGapMs==null?'-':`최근 10초 최대 ${j.loopGapMs}ms · 부팅 후 최대 ${j.loopGapMaxMs}ms <span class="sub">(배경 학습 제외. Wi-Fi가 끊겨도 같아야 정상, 1초를 넘으면 감지 기록에 남음)</span>`],
  ['프레임 수신',`마지막 ${ago(cam.lastFrameAgo)} · 실패 누적 ${cam.fails}회`],
  ['펌웨어',j.fw||'(이전 버전)'],
  ['가동 시간',upt(j.uptime)],
  ['여유 메모리',`내부 ${(j.heap/1024).toFixed(0)}KB · PSRAM ${(j.psram/1024).toFixed(0)}KB`],
  ['판(배경)',`RGB(${j.plate.rgb.join(', ')}) · 게인 ${j.plate.gain.join(' / ')}`]]);
 renderWifi(w);
 lnSamples=j.samples||0;lnTogLabel();
 if($('det').open)$('st').textContent=JSON.stringify(j,null,1);
}
// RSSI 등급: -67 이상 양호, -75 이상 보통, 그 아래 약함
const rq=r=>r>=-67?['양호','ok']:r>=-75?['보통','warn']:['약함','bad'];
const IPM={dhcp:'DHCP',probing:'고정 IP 후보 확인 중',static:'고정 IP',fallback:'DHCP (고정 IP 후보 사용 불가)'};
function renderWifi(w){
 const q=rq(w.rssi);
 pill($('wfPill'),`신호 ${q[0]} ${w.rssi} dBm`,q[1]);
 kv($('wfKv'),[
  ['공유기',`${w.ssid} · 채널 ${w.ch} · ${w.bssid}`],
  ['IP',`${w.ip} (${IPM[w.ipMode]||w.ipMode})`],
  ['접속 주소',`<a href="http://${w.ip}">http://${w.ip}</a> · http://${w.host}.local`],
  ['신호(RSSI)',`현재 ${w.rssi} · 최근 1분 평균 ${w.rssiAvg} / 최저 ${w.rssiMin} · 부팅 후 최저 ${w.rssiMinBoot} dBm`],
  ['연결 유지',`${w.upFor>=0?upt(w.upFor):'-'} · 부팅 후 끊김 ${w.drops}회`],
  ['마지막 끊김',w.lastReasonAgo>=0?`${ago(w.lastReasonAgo)} · 사유 ${w.lastReason}: ${w.lastReasonText}`:'없음'],
  ['설정',`절전 ${w.sleep?'켬':'끔'} · 송신 ${w.txPower} dBm · 20MHz · ${w.b11Only?'802.11b 전용':'b/g/n'} · 영상 최대 ${w.streamFps}fps`]]);
}
// 주변 채널 검사: 채널마다 겹치는 정도(앞뒤 4채널)와 신호 세기로 혼잡도 계산. 자기 공유기(같은 SSID)는 제외
async function wifiScan(){
 const btn=$('scanBtn');btn.disabled=true;$('scanMsg').textContent='검사 중…';
 try{
  const j=await(await fetch('/wifiscan',{cache:'no-store'})).json();
  if(j.error){$('scanMsg').textContent='검사 실패: '+j.error;return;}
  const score=Array(14).fill(0),cnt=Array(14).fill(0),best=Array(14).fill(-999);let others=0;
  for(const a of j.aps){
   if(a.s===j.ownSsid||a.ch<1||a.ch>13)continue;
   others++;cnt[a.ch]++;best[a.ch]=Math.max(best[a.ch],a.r);
   const occ=[a.ch];if(a.sc==1)occ.push(a.ch+4);if(a.sc==2)occ.push(a.ch-4);   // 40MHz 폭이면 보조 채널도 차지
   for(let c=1;c<=13;c++)for(const p of occ)score[c]+=Math.max(0,1-Math.abs(p-c)/5)*Math.pow(10,a.r/10)*1e6;
  }
  const max=Math.max(...score.slice(1),1e-9),rec=[1,6,11].reduce((m,c)=>score[c]<score[m]?c:m,1);
  let h='<div class="tbl"><table><thead><tr><th>채널</th><th>AP 수</th><th>가장 강한 신호</th><th class="b">혼잡도 (다른 공유기 간섭)</th></tr></thead><tbody>';
  for(let c=1;c<=13;c++){
   const tag=(c==j.ownCh?' · 현재':'')+(c==rec?' · 추천':'');
   h+=`<tr${c==rec?' class="rec"':''}><td>${c}${[1,6,11].includes(c)?' ★':''}${tag}</td><td>${cnt[c]}</td><td>${cnt[c]?best[c]+' dBm':'-'}</td><td class="b"><span class="bar" style="width:${Math.round(score[c]/max*100)}%"></span></td></tr>`;
  }
  $('scanRes').innerHTML=h+'</tbody></table></div><p class="note">★ 1·6·11은 서로 겹치지 않는 채널입니다. 채널은 공유기 관리 화면에서 바꾸며(채널 폭 20MHz 권장), ESP32는 바뀐 채널을 자동으로 따라갑니다.</p>';
  $('scanMsg').textContent=`다른 공유기 ${others}개 · 현재 채널 ${j.ownCh} → 추천 채널 ${rec} (1·6·11 중 가장 한산)`;
 }catch(e){$('scanMsg').textContent='검사 실패 (장치 응답 없음)';}
 finally{btn.disabled=false;}
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
// ---------- 학습 데이터 수집 ----------
const SZS=['-','소','중','대'],CLS=['-','흰','검','빨','노','초','파'];
COLORS.slice(1).forEach((n,i)=>{const b=document.createElement('button');b.innerHTML=`<span class="dot" style="background:${SW[i+1]}"></span>${n.replace('색','')}`;b.onclick=()=>learn('c',i+1);$('lnColors').appendChild(b);});
// 학습 모드 토글: 페이지를 열 때마다 꺼진 상태(숨김, 현장 운용 화면)로 시작한다. 켠 상태는 기억하지 않는다
let lnOn=false,lnSamples=0;
try{localStorage.removeItem('learnUI');}catch(e){}   // 예전 펌웨어가 브라우저에 남긴 '켜짐' 기억 지우기
function lnTogLabel(){const t=$('lnTog');t.classList.toggle('on',lnOn);
 t.textContent=lnOn?'학습 모드 끄기':(lnSamples>0?`학습 모드 (샘플 ${lnSamples}개)`:'학습 모드');}
function lnApply(){$('lnCard').hidden=!lnOn;lnTogLabel();if(lnOn)lnLoad();}
function lnToggle(){lnOn=!lnOn;lnApply();}
let lnBusy=false;
function lnButtons(dis){document.querySelectorAll('.lrow button').forEach(b=>b.disabled=dis);}
async function learn(k,v){
 if(lnBusy)return;lnBusy=true;lnButtons(true);
 $('lnMsg').textContent=`저장 중… (${k=='s'?SIZES[v]:COLORS[v]}) — 물체가 확정될 때까지 최대 5초 기다립니다`;
 try{
  const j=await(await fetch(`/learn?${k}=${v}`,{cache:'no-store'})).json();
  if(!j.ok){$('lnMsg').innerHTML=`<span class="pill bad">저장 안 됨</span> ${j.error}`;}
  else{
   const m=j.label==j.pred;
   $('lnMsg').innerHTML=`<span class="pill ${m?'ok':'bad'}">${m?'판별 일치':'판별 틀림'}</span> 샘플 #${j.id} ${j.kind=='size'?'크기':'색상'} 정답 <b>${j.labelName}</b> / 장치 판별 <b>${j.predName}</b> · 추정 길이 ${j.lenMm.toFixed(0)}mm`+
    (j.corrected?' · 라벨을 고쳤습니다':j.merged?' · 같은 물체 샘플에 추가':'')+` · 저장 ${j.count}개`;
   $('lnThumbs').innerHTML=`<img src="/sample.bmp?id=${j.id}&k=raw" alt="원본"><img src="/sample.bmp?id=${j.id}&k=mask" alt="마스크">`;
  }
 }catch(e){$('lnMsg').textContent='저장 실패 (장치 응답 없음)';}
 lnBusy=false;lnButtons(false);lnLoad();
}
async function lnDel(q,msg){
 if(!confirm(msg))return;
 try{const j=await(await fetch('/sample_del?'+q,{cache:'no-store'})).json();$('lnMsg').textContent=`${j.removed}개 삭제 · 남은 샘플 ${j.count}개`;$('lnThumbs').innerHTML='';}catch(e){}
 lnLoad();
}
function cmTable(rows,names,lab,pred){   // 혼동표: 행=정답, 열=판별
 let h=`<table class="cm"><thead><tr><th>정답＼판별</th>${names.map(n=>`<th>${n}</th>`).join('')}</tr></thead><tbody>`;
 for(let a=1;a<names.length;a++){const r=rows.filter(s=>s[lab]==a);if(!r.length)continue;
  h+=`<tr><th>${names[a]}</th>`+names.map((_,p)=>{const n=r.filter(s=>s[pred]==p).length;return `<td class="${n?(p==a?'o':'x'):''}">${n||''}</td>`;}).join('')+'</tr>';}
 return h+'</tbody></table>';
}
async function lnLoad(){
 try{
  const j=await(await fetch('/samples',{cache:'no-store'})).json(),S=j.samples;
  pill($('lnPill'),`${j.count}/${j.max}개`,j.count?'ok':'');
  $('lnMem').textContent=`PSRAM 여유 ${(j.psramFree/1024).toFixed(0)}KB`;
  const sz=S.filter(s=>s.size),co=S.filter(s=>s.color);
  const acc=(a,f)=>a.length?`${a.filter(f).length}/${a.length} (${Math.round(a.filter(f).length/a.length*100)}%)`:'-';
  let h=`<dl class="kv" style="margin-top:8px"><dt>크기 정확도</dt><dd>${acc(sz,s=>s.size==s.predSize)}</dd><dt>색상 정확도</dt><dd>${acc(co,s=>s.color==s.predColor)}</dd></dl>`;
  h+='<div class="row" style="align-items:flex-start">';
  if(sz.length)h+=`<div class="tbl">${cmTable(sz,['미판별','소','중','대'],'size','predSize')}</div>`;
  if(co.length)h+=`<div class="tbl">${cmTable(co,['미판별','흰','검','빨','노','초','파'],'color','predColor')}</div>`;
  $('lnAcc').innerHTML=h+'</div>';
  $('lnList').innerHTML=S.slice().reverse().map(s=>{
   const c1=s.size?(s.size==s.predSize?'o':'x'):'',c2=s.color?(s.color==s.predColor?'o':'x'):'';
   return `<tr><td>${s.id}</td><td class="${c1}">${SZS[s.size]}→${SZS[s.predSize]}</td><td class="${c2}">${CLS[s.color]}→${CLS[s.predColor]}</td><td>${s.confLenMm.toFixed(0)}mm</td><td>${s.lenMm.toFixed(0)}×${s.widMm.toFixed(0)}mm</td><td>${s.cx.toFixed(0)}, ${s.cy.toFixed(0)}</td><td>${s.grown?s.grown+'px':''}</td>`+
    `<td><a href="/sample.bmp?id=${s.id}&k=raw" target="_blank">원본</a> <a href="/sample.bmp?id=${s.id}&k=mask" target="_blank">마스크</a></td><td><button onclick="lnDel('id=${s.id}','샘플 #${s.id}을 삭제할까요?')">삭제</button></td></tr>`;}).join('');
 }catch(e){}
}
// PLC 명령 (40009) — 펌웨어 '14-2. PLC 명령'과 같은 값
const CMDS={1:'초기화',2:'배경 재학습',3:'감지 결과 초기화',4:'다시 판정'};
function cmdText(v){
 if(v==0)return '0 = 대기 (명령을 쓸 수 있음) / 명령값은 아래 패널 참조';
 if(v>=1&&v<=4)return `${v} = ${CMDS[v]} 명령이 쓰임 (곧 실행)`;
 if(v>=101&&v<=104)return `${v} = ${CMDS[v-100]} 실행 중`;
 if(v>=201&&v<=204)return `${v} = ${CMDS[v-200]} 실패 (0을 쓰면 지워짐)`;
 if(v==299)return '299 = 지원하지 않는 값을 씀';
 return String(v);}
const CMD_ASK={1:'판 위를 비웠나요?\n배경을 다시 학습하고, 감지 결과와 누적 감지 횟수를 0으로 만듭니다 (약 3초).',
 2:'판 위를 비웠나요?\n배경을 다시 학습합니다 (약 3초). 감지 결과와 누적 감지 횟수는 그대로입니다.',
 3:'감지 결과(40001~40008)와 누적 감지 횟수를 0으로 만듭니다.\n판 위에 물체가 있으면 #1로 다시 판정됩니다.',
 4:'판 위 물체를 처음부터 다시 판정합니다.\n새 결과는 누적 감지 횟수 +1로 나옵니다.'};
let cmdWait=null;
async function cmd(c,force){
 if(!force&&!confirm(CMD_ASK[c]))return;
 try{
  const j=await(await fetch(`/cmd?c=${c}${force?'&force=1':''}`,{cache:'no-store'})).json();
  if(!j.ok){$('cmdMsg').textContent=j.error;return;}
  cmdWait={c,seq:j.seq};
  $('cmdMsg').textContent=`${c} ${CMDS[c]} 요청함${force?' (판 확인 없이)':''} — 진행 상태는 위에 표시됩니다`;
 }catch(e){$('cmdMsg').textContent='장치 응답 없음';}
}
function renderCmd(k){
 if(!k)return;
 const r=k.reg,PH=['','실행 중','완료','실패'];
 pill($('cmdPill'),r==0?'대기':(r<=4?'명령 쓰임':(r<200?'실행 중':'실패')),r==0?'':(r<200?'warn':'bad'));
 const rows=[['40009 현재 값',cmdText(r)]];
 if(k.code)rows.push(['마지막 명령',`${k.code} ${k.name} — <b>${PH[k.phase]}</b> · ${k.web?'웹':'PLC'}에서 · ${k.phase==1?ago(k.ago)+' 시작':ago(k.ago)}${k.dur>=0?` · ${(k.dur/1000).toFixed(1)}초 걸림`:''}`]);
 if(k.msg)rows.push(['내용',k.msg]);
 kv($('cmdKv'),rows);
 if(cmdWait&&k.seq!==cmdWait.seq&&k.code==cmdWait.c&&k.phase>=2){
  const w=cmdWait;cmdWait=null;$('cmdMsg').textContent='';
  if(k.phase==3&&k.byObject&&confirm(`판 위에 물체가 보여서 '${CMDS[w.c]}'을(를) 실행하지 않았습니다.\n\n물체가 정말 없는데 조명 얼룩·반사 때문에 그렇게 보이는 것이라면 '확인'을 눌러 판 확인 없이 실행합니다.\n(물체가 있는 채로 배경을 학습하면 그 물체가 배경이 됩니다)`))cmd(w.c,true);
 }
}
function relearn(){cmd(2);}
$('s').src='http://'+location.hostname+':81/stream';
pollStatus();pollLog();lnApply();setInterval(()=>{if(lnOn)lnLoad();},10000);
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
  static char json[5120];
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
    "\"len\":%.1f,\"wid\":%.1f,\"ratio\":%.2f,\"angle\":%.0f,"
    "\"lenMm\":%.1f,\"widMm\":%.1f,\"sideCut\":%s,\"grown\":%d,\"size\":\"%s\"},"
    "\"color\":{\"code\":%d,\"name\":\"%s\",\"rgb\":[%d,%d,%d],\"samples\":%d,\"chromaShare\":%.2f,\"vRatio\":%.2f,"
    "\"votes\":{\"white\":%d,\"black\":%d,\"red\":%d,\"yellow\":%d,\"green\":%d,\"blue\":%d,\"other\":%d}},"
    "\"spot\":{\"area\":%d,\"occupancy\":%.2f,\"bestOverlap\":%.2f,\"removeStreak\":%d,\"changeCount\":%d},"
    "\"plate\":{\"rgb\":[%.0f,%.0f,%.0f],\"gray\":%.0f,\"gain\":[%.2f,%.2f,%.2f]},"
    "\"confirmed\":{\"count\":%u,\"color\":%u,\"colorName\":\"%s\",\"size\":%u,\"sizeName\":\"%s\",\"lenMm\":%.1f,\"ratio\":%.2f,\"rgb\":[%d,%d,%d]}",
    STATE_NAMES[st.state], st.fps, (unsigned long)st.frameNo, st.stableCount,
    st.fgRawCount, st.fgCount, st.components,
    b.area, b.valid ? "true" : "false", b.touchesBorder ? "true" : "false", b.cx, b.cy,
    b.len, b.wid, b.ratio, b.angleDeg,
    b.lenMm, b.widMm, b.sideCut ? "true" : "false", b.grown, getSizeName(classifySizeByLen(b.lenMm)),
    c.code, getColorName(c.code), c.r, c.g, c.b, c.samples, c.chromaShare, c.vRatio,
    c.votes[1], c.votes[2], c.votes[3], c.votes[4], c.votes[5], c.votes[6], c.votes[7],
    st.footArea, st.foot.occ, st.foot.bestOverlap, st.removeStreak, st.changeCount,
    plateR, plateG, plateB, plateGray, gainR, gainG, gainB,
    detectCount, curColorCode, getColorName(curColorCode), curSizeCode, getSizeName(curSizeCode),
    curLenMm, curLenMm / 35.0f, curR, curG, curB);

  // 카메라 · Wi-Fi · 시스템
  unsigned long now = millis();
  unsigned long lastFrame = lastFrameMs;
  n = jsonAppend(json, cap, n,
    ",\"fw\":\"" FW_VERSION "\",\"samples\":%d,\"uptime\":%lu,\"loopGapMs\":%lu,\"loopGapMaxMs\":%lu,\"heap\":%u,\"psram\":%u,"
    "\"camera\":{\"ok\":%s,\"pid\":%u,\"fails\":%lu,\"lastFrameAgo\":%ld},"
    "\"wifi\":{\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"host\":\"%s\"",
    smpCount, now, (unsigned long)loopGapMax10s, (unsigned long)loopGapMaxBoot,
    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram(),
    cameraOk ? "true" : "false", (unsigned)camPid, (unsigned long)camFails,
    lastFrame ? (long)(now - lastFrame) : -1L,
    ssid, WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(), HOSTNAME);
  unsigned long upSince = wifiUpSinceMs, discMs = wifiDiscMs;
  uint8_t reason = wifiDiscReason;
  n = jsonAppend(json, cap, n,
    ",\"ch\":%d,\"bssid\":\"%s\",\"ipMode\":\"%s\",\"rssiAvg\":%d,\"rssiMin\":%d,\"rssiMinBoot\":%d,"
    "\"upFor\":%ld,\"drops\":%lu,\"lastReason\":%u,\"lastReasonText\":\"%s\",\"lastReasonAgo\":%ld,"
    "\"txPower\":%.1f,\"sleep\":%s,\"b11Only\":%s,\"streamFps\":%d}",
    (int)WiFi.channel(), WiFi.BSSIDstr().c_str(), IPM_NAMES[ipMode], wifiRssiAvg, wifiRssiMin, wifiRssiMinBoot,
    upSince ? (long)(now - upSince) : -1L, (unsigned long)wifiDrops, (unsigned)reason, wifiReasonText(reason),
    discMs ? (long)(now - discMs) : -1L,
    (int)WiFi.getTxPower() / 4.0f, WiFi.getSleep() ? "true" : "false", WIFI_11B_ONLY ? "true" : "false", STREAM_MAX_FPS);

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
  n = jsonAppend(json, cap, n, "]}");

  // PLC 명령 (40009): 현재 값과 마지막 명령 결과
  CmdState cs;
  portENTER_CRITICAL(&cmdMux);
  cs = cmdSt;
  portEXIT_CRITICAL(&cmdMux);
  long dur = cs.endMs ? (long)(cs.endMs - cs.startMs) : (cs.phase == 1 ? (long)(now - cs.startMs) : -1L);
  n = jsonAppend(json, cap, n,
    ",\"cmd\":{\"reg\":%u,\"seq\":%lu,\"code\":%u,\"name\":\"%s\",\"phase\":%u,\"web\":%s,\"byObject\":%s,"
    "\"ago\":%ld,\"dur\":%ld,\"waitEmptyMs\":%lu,\"msg\":\"%s\"}}",
    (unsigned)regMirror[REG_COMMAND], (unsigned long)cs.seq, (unsigned)cs.code, cmdName(cs.code), (unsigned)cs.phase,
    cs.fromWeb ? "true" : "false", cs.byObject ? "true" : "false",
    cs.code ? (long)(now - (cs.endMs ? cs.endMs : cs.startMs)) : -1L, dur, CMD_EMPTY_WAIT_MS, cs.msg);
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

// 예전 주소 호환: 명령 2(배경 재학습)를 판 확인 없이 실행 — 사람이 화면을 보고 누르는 경우
static esp_err_t relearn_handler(httpd_req_t *req) {
  webCommand(CMD_RELEARN, true);
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
    // 새 프레임이 없거나, 최대 초당 프레임(STREAM_MAX_FPS)을 넘으면 기다린다 (2초마다는 같은 프레임이라도 보냄)
    if ((dbgFrameId == lastId && millis() - lastSend < 2000) || millis() - lastSend < 1000UL / STREAM_MAX_FPS) {
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
  }
  return res;
}

// JSON 문자열 이스케이프 (SSID에 따옴표 등이 있을 수 있음)
static void jsonEscape(char *out, size_t cap, const char *s) {
  size_t n = 0;
  for (; *s && n + 7 < cap; s++) {
    unsigned char ch = (unsigned char)*s;
    if (ch == '"' || ch == '\\') { out[n++] = '\\'; out[n++] = ch; }
    else if (ch < 0x20) n += snprintf(out + n, cap - n, "\\u%04x", ch);
    else out[n++] = ch;
  }
  out[n] = 0;
}

// 주변 Wi-Fi 검사: 장치 위치에서 보이는 공유기의 채널·신호 세기 (약 2~4초 걸림, 그동안 영상이 잠깐 멈출 수 있음)
static esp_err_t wifiscan_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  int n = WiFi.scanNetworks(false, true, false, 150);   // 동기 · 숨은 SSID 포함 · 능동 검색 · 채널당 150ms
  if (n < 0) {
    WiFi.scanDelete();
    return httpd_resp_sendstr(req, "{\"error\":\"검색 실패 (Wi-Fi 재연결 중일 수 있음)\"}");
  }
  char esc[80], line[200];
  jsonEscape(esc, sizeof(esc), ssid);
  int len = snprintf(line, sizeof(line), "{\"ownSsid\":\"%s\",\"ownCh\":%d,\"ownBssid\":\"%s\",\"aps\":[",
                     esc, (int)WiFi.channel(), WiFi.BSSIDstr().c_str());
  esp_err_t res = httpd_resp_send_chunk(req, line, len);
  for (int i = 0; i < n && res == ESP_OK; i++) {
    wifi_ap_record_t *ap = (wifi_ap_record_t *)WiFi.getScanInfoByIndex(i);
    if (!ap) continue;
    jsonEscape(esc, sizeof(esc), (const char *)ap->ssid);
    len = snprintf(line, sizeof(line), "%s{\"s\":\"%s\",\"b\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"ch\":%u,\"sc\":%u,\"r\":%d}",
                   i ? "," : "", esc, ap->bssid[0], ap->bssid[1], ap->bssid[2], ap->bssid[3], ap->bssid[4], ap->bssid[5],
                   (unsigned)ap->primary, (unsigned)ap->second, (int)ap->rssi);
    res = httpd_resp_send_chunk(req, line, len);
  }
  WiFi.scanDelete();
  if (res == ESP_OK) res = httpd_resp_send_chunk(req, "]}", 2);
  if (res == ESP_OK) res = httpd_resp_send_chunk(req, NULL, 0);
  return res;
}

// ---------- 학습 데이터: 웹 처리 ----------

static inline void wr16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void wr32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

static int queryInt(httpd_req_t *req, const char *key, int def) {
  char q[64], v[16];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) return def;
  if (httpd_query_key_value(q, key, v, sizeof(v)) != ESP_OK) return def;
  return atoi(v);
}

// BMP 헤더 (아래→위 행 순서, 행 길이는 4바이트 배수). 반환: 픽셀 데이터 시작 위치
static size_t bmpHeader(uint8_t *p, int w, int h, int bpp, int paletteN) {
  int row = ((w * bpp / 8) + 3) & ~3;
  uint32_t off = 14 + 40 + paletteN * 4;
  memset(p, 0, off);
  p[0] = 'B'; p[1] = 'M';
  wr32(p + 2, off + row * h); wr32(p + 10, off);
  wr32(p + 14, 40); wr32(p + 18, w); wr32(p + 22, h); wr16(p + 26, 1); wr16(p + 28, bpp);
  wr32(p + 34, row * h); wr32(p + 38, 2835); wr32(p + 42, 2835); wr32(p + 46, paletteN);
  return off;
}
#define BMP24_LEN (54 + ROI_W * 3 * ROI_H)
#define BMP8_LEN  (54 + 256 * 4 + ROI_W * ROI_H)
static_assert((ROI_W * 3) % 4 == 0 && ROI_W % 4 == 0, "BMP rows must not need padding");

// 원본 RGB565 → 24비트 BMP
static size_t bmpFromRaw(uint8_t *out, const uint8_t *raw) {
  size_t o = bmpHeader(out, ROI_W, ROI_H, 24, 0);
  for (int y = ROI_H - 1; y >= 0; y--) {
    for (int x = 0; x < ROI_W; x++) {
      int r, g, b;
      readPx(raw, y * ROI_W + x, r, g, b);
      out[o++] = b; out[o++] = g; out[o++] = r;
    }
  }
  return o;
}
// 배경 스냅샷(R,G,B) → 24비트 BMP
static size_t bmpFromRgb(uint8_t *out, const uint8_t *rgb) {
  size_t o = bmpHeader(out, ROI_W, ROI_H, 24, 0);
  for (int y = ROI_H - 1; y >= 0; y--) {
    const uint8_t *p = rgb + y * ROI_W * 3;
    for (int x = 0; x < ROI_W; x++, p += 3) { out[o++] = p[2]; out[o++] = p[1]; out[o++] = p[0]; }
  }
  return o;
}
// 마스크 → 8비트 팔레트 BMP (픽셀 값 = 마스크 단계 0~3)
static size_t bmpFromMask(uint8_t *out, const uint8_t *mask) {
  size_t o = bmpHeader(out, ROI_W, ROI_H, 8, 256);
  static const uint8_t PAL[4][3] = {{0, 0, 0}, {90, 90, 90}, {255, 255, 255}, {170, 0, 255}};   // R,G,B
  for (int k = 0; k < 4; k++) { out[54 + k * 4] = PAL[k][2]; out[55 + k * 4] = PAL[k][1]; out[56 + k * 4] = PAL[k][0]; }
  for (int y = ROI_H - 1; y >= 0; y--) {
    for (int x = 0; x < ROI_W; x++) {
      int k = y * ROI_W + x;
      out[o++] = (mask[k >> 2] >> ((k & 3) * 2)) & 3;
    }
  }
  return o;
}

static void sampleFileName(char *out, size_t cap, const Sample *s, const char *kind) {
  snprintf(out, cap, "s%03u_%c_%c_%s.bmp", s->id, SIZE_KEYS[s->sizeLabel], COLOR_KEYS[s->colorLabel], kind);
}

// 샘플 한 개의 JSON (full = ZIP용 전체 항목)
static size_t sampleJson(char *o, size_t cap, const Sample *s, bool full) {
  const BlobInfo &b = s->blob;
  size_t n = snprintf(o, cap,
    "{\"id\":%u,\"det\":%u,\"ms\":%lu,\"size\":%u,\"color\":%u,\"predSize\":%u,\"predColor\":%u,\"confLenMm\":%.1f,"
    "\"frameSize\":%u,\"lenMm\":%.1f,\"widMm\":%.1f,\"grown\":%d,"
    "\"area\":%d,\"cx\":%.2f,\"cy\":%.2f,\"len\":%.2f,\"wid\":%.2f,\"ratio\":%.3f,\"angle\":%.1f",
    s->id, s->det, (unsigned long)s->ms, s->sizeLabel, s->colorLabel, s->predSize, s->predColor, s->confLenMm,
    s->frameSize, b.lenMm, b.widMm, b.grown, b.area, b.cx, b.cy, b.len, b.wid, b.ratio, b.angleDeg);
  if (full && n < cap) {
    const ColorInfo &c = s->color;
    char fr[40], fm[40];
    sampleFileName(fr, sizeof(fr), s, "raw");
    sampleFileName(fm, sizeof(fm), s, "mask");
    n += snprintf(o + n, cap - n,
      ",\"bbox\":[%d,%d,%d,%d],\"touchesBorder\":%s,\"frameColor\":%u,\"rgb\":[%d,%d,%d],\"confRgb\":[%d,%d,%d],"
      "\"colorSamples\":%d,\"chromaShare\":%.3f,\"vRatio\":%.3f,\"votes\":[%d,%d,%d,%d,%d,%d,%d],"
      "\"plate\":[%.1f,%.1f,%.1f],\"gain\":[%.3f,%.3f,%.3f],\"bg\":%d,\"raw\":\"%s\",\"mask\":\"%s\"",
      b.minX, b.minY, b.maxX, b.maxY, b.touchesBorder ? "true" : "false", c.code, c.r, c.g, c.b,
      s->confRgb[0], s->confRgb[1], s->confRgb[2], c.samples, c.chromaShare, c.vRatio,
      c.votes[1], c.votes[2], c.votes[3], c.votes[4], c.votes[5], c.votes[6], c.votes[7],
      s->plate[0], s->plate[1], s->plate[2], s->gain[0], s->gain[1], s->gain[2], s->bg, fr, fm);
  }
  if (n < cap) n += snprintf(o + n, cap - n, "}");
  return n;
}

// 전체 메타데이터 + 샘플 목록 JSON (PSRAM 버퍼를 할당해 반환, 호출자가 free)
static char *samplesJson(bool full, size_t *outLen) {
  size_t cap = 2048 + (size_t)SMP_MAX * (full ? 820 : 390);
  char *o = (char *)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
  if (!o) return nullptr;
  xSemaphoreTake(smpMutex, portMAX_DELAY);
  size_t n = snprintf(o, cap,
    "{\"fw\":\"%s\",\"host\":\"%s\",\"count\":%d,\"max\":%d,\"psramFree\":%u,"
    "\"img\":[%d,%d],\"roi\":[%d,%d,%d,%d],"
    "\"sizeModel\":{\"perspP\":%.5f,\"yScale\":%.4f,\"pxPer35mm\":%.2f,\"sidePx\":%.1f,\"smallMaxMm\":%.1f,\"mediumMaxMm\":%.1f,"
    "\"blackGrowCols\":%d,\"whiteBlackSplit\":%.2f},"
    "\"sizeNames\":[\"미입력\",\"소(35mm)\",\"중(55mm)\",\"대(85mm)\"],"
    "\"colorNames\":[\"미입력\",\"흰색\",\"검정색\",\"빨간색\",\"노란색\",\"초록색\",\"파란색\"],"
    "\"maskLevels\":\"0 배경, 1 다른 덩어리, 2 선택 덩어리, 3 검정 보강 픽셀\",\"backgrounds\":[",
    FW_VERSION, HOSTNAME, smpCount, SMP_MAX, (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
    IMG_W, IMG_H, ROI_X0, ROI_Y0, ROI_X1, ROI_Y1,
    SIZE_PERSP_P, SIZE_Y_SCALE, SIZE_PX_PER_35MM, SIDE_FACE_PX, SIZE_SMALL_MAX_MM, SIZE_MEDIUM_MAX_MM,
    BLACK_GROW_COLS, WHITE_BLACK_SPLIT);
  for (int k = 0; k < bgSnapCount && n < cap; k++)
    n += snprintf(o + n, cap - n, "%s{\"bg\":%d,\"ver\":%u,\"ms\":%lu,\"file\":\"bg%d.bmp\"}", k ? "," : "", k,
                  bgSnap[k].ver, (unsigned long)bgSnap[k].ms, k);
  if (n < cap) n += snprintf(o + n, cap - n, "],\"samples\":[");
  for (int k = 0; k < smpCount && n < cap; k++) {
    if (k && n < cap) o[n++] = ',';
    n += sampleJson(o + n, cap - n, smp[k], full);
  }
  xSemaphoreGive(smpMutex);
  if (n < cap) n += snprintf(o + n, cap - n, "]}");
  if (n >= cap) { free(o); return nullptr; }
  *outLen = n;
  return o;
}

// /cmd?c=1..4[&force=1]: PLC가 40009에 쓰는 것과 같은 명령. 바로 응답하고, 결과는 /status의 cmd로 확인
//   force=1: 명령 1·2에서 '판이 비었나' 확인을 건너뜀 (물체가 없는데 조명 얼룩 등으로 거부될 때 사람이 판단해서)
static esp_err_t cmd_handler(httpd_req_t *req) {
  int c = queryInt(req, "c", 0);
  bool force = queryInt(req, "force", 0) == 1;
  char out[160];
  if (c < 1 || c > CMD_MAX) {
    snprintf(out, sizeof(out), "{\"ok\":false,\"error\":\"명령은 1~%d 입니다\"}", CMD_MAX);
  } else {
    uint32_t seq;
    portENTER_CRITICAL(&cmdMux);
    seq = cmdSt.seq;
    portEXIT_CRITICAL(&cmdMux);
    webCommand(c, force);
    snprintf(out, sizeof(out), "{\"ok\":true,\"code\":%d,\"name\":\"%s\",\"force\":%s,\"seq\":%lu}", c, cmdName(c),
             force ? "true" : "false", (unsigned long)seq);
  }
  httpd_resp_set_type(req, "application/json; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
}

// /learn?s=1..3 (크기) 또는 /learn?c=1..6 (색상): 메인 루프가 확정 상태 프레임에서 저장할 때까지 기다린다
static esp_err_t learn_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  int s = queryInt(req, "s", 0), c = queryInt(req, "c", 0);
  char kind = s ? 's' : 'c';
  int val = s ? s : c;
  if ((s && (s < 1 || s > 3)) || (!s && (c < 1 || c > 6))) return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"잘못된 요청\"}");
  if (!cameraOk) return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"카메라 초기화 실패 상태\"}");
  if (learnReq.pending) return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"이전 요청 처리 중\"}");
  learnReq.kind = kind;
  learnReq.value = val;
  learnReq.deadline = millis() + 5000;     // 5초 안에 확정 상태가 되면 저장
  learnReq.done = false;
  learnReq.pending = true;
  unsigned long t0 = millis();
  while (!learnReq.done && millis() - t0 < 7000) vTaskDelay(pdMS_TO_TICKS(30));
  if (!learnReq.done) {
    learnReq.pending = false;
    return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"장치 응답 시간 초과 (배경 학습 중일 수 있음)\"}");
  }
  return httpd_resp_sendstr(req, learnReq.result);
}

// /samples : 메타데이터 + 샘플 목록 (웹 화면용)
static esp_err_t samples_handler(httpd_req_t *req) {
  size_t len = 0;
  char *j = samplesJson(false, &len);
  if (!j) return httpd_resp_send_500(req);
  httpd_resp_set_type(req, "application/json; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t r = httpd_resp_send(req, j, len);
  free(j);
  return r;
}

// /sample.bmp?id=N&k=raw|mask  또는  /sample.bmp?bg=K
static esp_err_t sample_bmp_handler(httpd_req_t *req) {
  char q[64], k[8] = "raw";
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) httpd_query_key_value(q, "k", k, sizeof(k));
  int id = queryInt(req, "id", -1), bg = queryInt(req, "bg", -1);
  bool isMask = strcmp(k, "mask") == 0;
  uint8_t *out = (uint8_t *)heap_caps_malloc(BMP24_LEN > BMP8_LEN ? BMP24_LEN : BMP8_LEN, MALLOC_CAP_SPIRAM);
  if (!out) return httpd_resp_send_500(req);
  size_t len = 0;
  xSemaphoreTake(smpMutex, portMAX_DELAY);
  if (bg >= 0 && bg < bgSnapCount) {
    len = bmpFromRgb(out, bgSnap[bg].rgb);
  } else if (id >= 0) {
    int i = findSample(id);
    if (i >= 0) len = isMask ? bmpFromMask(out, smp[i]->mask) : bmpFromRaw(out, smp[i]->raw);
  }
  xSemaphoreGive(smpMutex);
  esp_err_t r;
  if (!len) {
    httpd_resp_set_status(req, "404 Not Found");
    r = httpd_resp_sendstr(req, "샘플 없음");
  } else {
    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    r = httpd_resp_send(req, (const char *)out, len);
  }
  free(out);
  return r;
}

// /sample_del?id=N | ?last=1 | ?all=1
static esp_err_t sample_del_handler(httpd_req_t *req) {
  int id = queryInt(req, "id", -1), last = queryInt(req, "last", 0), all = queryInt(req, "all", 0);
  int removed = 0;
  xSemaphoreTake(smpMutex, portMAX_DELAY);
  if (all) {
    for (int k = 0; k < smpCount; k++) free(smp[k]);
    removed = smpCount;
    smpCount = 0;
    for (int k = 0; k < bgSnapCount; k++) free(bgSnap[k].rgb);
    bgSnapCount = 0;
  } else {
    int i = last ? smpCount - 1 : findSample(id);
    if (i >= 0) {
      free(smp[i]);
      for (int k = i; k < smpCount - 1; k++) smp[k] = smp[k + 1];
      smpCount--;
      removed = 1;
    }
  }
  int left = smpCount;
  xSemaphoreGive(smpMutex);
  if (removed) EVENT("[학습] 샘플 %d개 삭제 (남은 샘플 %d개)", removed, left);
  char r[64];
  snprintf(r, sizeof(r), "{\"ok\":true,\"removed\":%d,\"count\":%d}", removed, left);
  httpd_resp_set_type(req, "application/json; charset=utf-8");
  return httpd_resp_sendstr(req, r);
}

// ZIP(무압축) 스트리밍: 파일마다 CRC를 먼저 계산할 수 있도록 메모리에서 만든 뒤 보낸다
static uint32_t crc32buf(const uint8_t *d, size_t n) {
  static uint32_t table[256];
  static bool ready = false;
  if (!ready) {
    for (uint32_t i = 0; i < 256; i++) {
      uint32_t c = i;
      for (int k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      table[i] = c;
    }
    ready = true;
  }
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++) c = table[(c ^ d[i]) & 0xFF] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

struct ZipOut {
  struct Ent { char name[40]; uint32_t crc, size, off; };
  httpd_req_t *req;
  Ent *ents;
  int n = 0, cap;
  uint32_t off = 0;
  esp_err_t err = ESP_OK;
  void send(const void *p, size_t len) {
    if (err == ESP_OK && len) err = httpd_resp_send_chunk(req, (const char *)p, len);
    off += len;
  }
  void add(const char *name, const uint8_t *data, uint32_t len) {
    if (err != ESP_OK || n >= cap) return;
    Ent &e = ents[n++];
    strlcpy(e.name, name, sizeof(e.name));
    e.crc = crc32buf(data, len);
    e.size = len;
    e.off = off;
    uint16_t nl = strlen(e.name);
    uint8_t h[30] = {0};
    wr32(h, 0x04034b50); wr16(h + 4, 10); wr16(h + 12, 0x0021);   // 버전 1.0, 무압축, 날짜 1980-01-01
    wr32(h + 14, e.crc); wr32(h + 18, len); wr32(h + 22, len); wr16(h + 26, nl);
    send(h, 30);
    send(e.name, nl);
    for (uint32_t k = 0; k < len; k += 8192) send(data + k, len - k < 8192 ? len - k : 8192);
  }
  void finish() {
    uint32_t cdOff = off;
    for (int k = 0; k < n; k++) {
      uint8_t c[46] = {0};
      uint16_t nl = strlen(ents[k].name);
      wr32(c, 0x02014b50); wr16(c + 4, 20); wr16(c + 6, 10); wr16(c + 14, 0x0021);
      wr32(c + 16, ents[k].crc); wr32(c + 20, ents[k].size); wr32(c + 24, ents[k].size); wr16(c + 28, nl);
      wr32(c + 42, ents[k].off);
      send(c, 46);
      send(ents[k].name, nl);
    }
    uint8_t z[22] = {0};
    wr32(z, 0x06054b50); wr16(z + 8, n); wr16(z + 10, n); wr32(z + 12, off - cdOff); wr32(z + 16, cdOff);
    send(z, 22);
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, NULL, 0);
  }
};

// /samples.zip : samples.json + 샘플별 원본/마스크 BMP + 배경 BMP
static esp_err_t samples_zip_handler(httpd_req_t *req) {
  size_t jlen = 0;
  char *json = samplesJson(true, &jlen);
  uint8_t *bmp = (uint8_t *)heap_caps_malloc(BMP24_LEN > BMP8_LEN ? BMP24_LEN : BMP8_LEN, MALLOC_CAP_SPIRAM);
  ZipOut z;
  z.req = req;
  z.cap = 2 * SMP_MAX + BGSNAP_MAX + 1;
  z.ents = (ZipOut::Ent *)heap_caps_malloc(sizeof(ZipOut::Ent) * z.cap, MALLOC_CAP_SPIRAM);
  if (!json || !bmp || !z.ents) {
    free(json); free(bmp); free(z.ents);
    return httpd_resp_send_500(req);
  }
  char disp[80];
  snprintf(disp, sizeof(disp), "attachment; filename=\"%s_samples.zip\"", HOSTNAME);
  httpd_resp_set_type(req, "application/zip");
  httpd_resp_set_hdr(req, "Content-Disposition", disp);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  z.add("samples.json", (const uint8_t *)json, jlen);
  free(json);

  // 목록을 먼저 복사해 두고, 파일마다 잠깐씩만 잠근다 (그사이 삭제된 샘플은 건너뜀)
  uint16_t ids[SMP_MAX];
  xSemaphoreTake(smpMutex, portMAX_DELAY);
  int cnt = smpCount, bgs = bgSnapCount;
  for (int k = 0; k < cnt; k++) ids[k] = smp[k]->id;
  xSemaphoreGive(smpMutex);

  char name[40];
  for (int k = 0; k < bgs && z.err == ESP_OK; k++) {
    xSemaphoreTake(smpMutex, portMAX_DELAY);
    size_t len = k < bgSnapCount ? bmpFromRgb(bmp, bgSnap[k].rgb) : 0;
    xSemaphoreGive(smpMutex);
    snprintf(name, sizeof(name), "bg%d.bmp", k);
    if (len) z.add(name, bmp, len);
  }
  for (int k = 0; k < cnt && z.err == ESP_OK; k++) {
    for (int part = 0; part < 2; part++) {
      xSemaphoreTake(smpMutex, portMAX_DELAY);
      int i = findSample(ids[k]);
      size_t len = 0;
      if (i >= 0) {
        len = part == 0 ? bmpFromRaw(bmp, smp[i]->raw) : bmpFromMask(bmp, smp[i]->mask);
        sampleFileName(name, sizeof(name), smp[i], part == 0 ? "raw" : "mask");
      }
      xSemaphoreGive(smpMutex);
      if (len) z.add(name, bmp, len);
    }
  }
  z.finish();
  free(bmp);
  free(z.ents);
  return z.err;
}

void startWebServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.stack_size = 8192;
  // 웹 서버(포트 80·81)는 코어 0 (Wi-Fi·네트워크와 같은 코어)에 고정한다.
  //   기본값(코어 지정 없음, 우선순위 5)이면 영상 스트림 JPEG 인코딩·ZIP 만들기가 코어 1에서 돌면서
  //   우선순위 1인 메인 루프(감지·판정)를 밀어낸다. 코어 1은 메인 루프와 Modbus 태스크만 쓴다.
  //   코어 0의 Wi-Fi(23)·lwIP(18)·카메라 태스크는 우선순위가 더 높아 웹 처리에 밀리지 않는다.
  config.core_id = 0;
  httpd_uri_t uris[] = {
    {.uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL},
    {.uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = NULL},
    {.uri = "/events", .method = HTTP_GET, .handler = events_handler, .user_ctx = NULL},
    {.uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL},
    {.uri = "/raw.bmp", .method = HTTP_GET, .handler = raw_bmp_handler, .user_ctx = NULL},
    {.uri = "/relearn", .method = HTTP_GET, .handler = relearn_handler, .user_ctx = NULL},
    {.uri = "/cmd", .method = HTTP_GET, .handler = cmd_handler, .user_ctx = NULL},
    {.uri = "/wifiscan", .method = HTTP_GET, .handler = wifiscan_handler, .user_ctx = NULL},
    {.uri = "/learn", .method = HTTP_GET, .handler = learn_handler, .user_ctx = NULL},
    {.uri = "/samples", .method = HTTP_GET, .handler = samples_handler, .user_ctx = NULL},
    {.uri = "/sample.bmp", .method = HTTP_GET, .handler = sample_bmp_handler, .user_ctx = NULL},
    {.uri = "/sample_del", .method = HTTP_GET, .handler = sample_del_handler, .user_ctx = NULL},
    {.uri = "/samples.zip", .method = HTTP_GET, .handler = samples_zip_handler, .user_ctx = NULL},
  };
  config.max_uri_handlers = 16;
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

// ---------- Wi-Fi 보조 함수 ----------

// 끊김 사유 코드 → 설명 (ESP-IDF wifi_err_reason_t)
static const char *wifiReasonText(uint8_t r) {
  switch (r) {
    case 0:   return "없음";
    case 2:   return "인증 만료";
    case 3:   return "공유기가 연결을 끊음";
    case 4:   return "무응답으로 공유기가 연결을 끊음";
    case 8:   return "장치가 연결을 끊음 (재접속 시도)";
    case 15:  return "4-way 핸드셰이크 시간 초과 (신호 약함 또는 비밀번호)";
    case 200: return "비컨 수신 끊김 (신호 약함·간섭)";
    case 201: return "공유기를 찾지 못함 (신호 약함·꺼짐·SSID 확인)";
    case 202: return "인증 실패 (비밀번호 확인)";
    case 203: return "연결 실패";
    case 204: return "핸드셰이크 시간 초과 (신호 약함 또는 비밀번호)";
    case 205: return "연결 실패";
    default:  return WiFi.STA.disconnectReasonName((wifi_err_reason_t)r);
  }
}

// Wi-Fi 이벤트 태스크에서 호출됨 — 값만 기록하고, 로그는 networkTask()가 남긴다
static void onWiFiDisconnected(arduino_event_id_t, arduino_event_info_t info) {
  wifiDiscReason = info.wifi_sta_disconnected.reason;
  wifiDiscRssi = info.wifi_sta_disconnected.rssi;
  wifiDiscMs = millis();
}

// ARP로 주소 사용 여부 확인 (lwIP 코어 잠금 후 호출 — 이 프레임워크는 CONFIG_LWIP_TCPIP_CORE_LOCKING=1)
static struct netif *staLwipNetif() {
  esp_netif_t *en = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  return en ? (struct netif *)esp_netif_get_netif_impl(en) : nullptr;
}
static void arpAsk(const IPAddress &ip) {
  struct netif *nif = staLwipNetif();
  if (!nif) return;
  ip4_addr_t a;
  a.addr = (uint32_t)ip;
  LOCK_TCPIP_CORE();
  etharp_request(nif, &a);
  UNLOCK_TCPIP_CORE();
}
static bool arpAnswered(const IPAddress &ip) {
  struct netif *nif = staLwipNetif();
  if (!nif) return false;
  ip4_addr_t a;
  a.addr = (uint32_t)ip;
  struct eth_addr *mac = nullptr;
  const ip4_addr_t *ipRet = nullptr;
  LOCK_TCPIP_CORE();
  ssize_t idx = etharp_find_addr(nif, &a, &mac, &ipRet);
  UNLOCK_TCPIP_CORE();
  return idx >= 0;
}

static void startWebAndMdns() {
  static bool mdnsStarted = false;
  if (!webStarted) { startWebServers(); webStarted = true; }
  if (!mdnsStarted && MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
  }
}

static void announceConnected(unsigned long downMs) {
  startWebAndMdns();
  char down[40] = "";
  if (downMs) snprintf(down, sizeof(down), ", %lu초 만에 복구", downMs / 1000);
  EVENT("[WiFi] 연결됨: http://%s (http://%s.local) 채널 %d, RSSI %d dBm, %s%s",
        WiFi.localIP().toString().c_str(), HOSTNAME, (int)WiFi.channel(), (int)WiFi.RSSI(),
        ipMode == IPM_STATIC ? "고정 IP" : "DHCP", down);
}

// 고정 IP 후보 확인: 0.5초마다 한 단계씩 진행 (카메라 처리·485 응답을 막지 않게)
//   후보마다 ARP 요청을 최대 3번 보내고, 응답이 없으면 비어 있는 주소로 보고 사용한다
static void ipProbeStep(unsigned long now) {
  static uint8_t idx = 0, tries = 0;
  const uint8_t n = sizeof(STATIC_IP_CANDIDATES) / sizeof(STATIC_IP_CANDIDATES[0]);
  if (ipMode != IPM_PROBING) return;
  const IPAddress &cand = STATIC_IP_CANDIDATES[idx];
  IPAddress me = WiFi.localIP(), mask = WiFi.subnetMask();

  if (tries == 0) {
    if (((uint32_t)cand & (uint32_t)mask) != ((uint32_t)me & (uint32_t)mask)) {
      ipMode = IPM_FALLBACK;
      EVENT("[WiFi] 고정 IP 후보(%s)가 현재 네트워크(%s)와 달라 DHCP 주소를 사용",
            cand.toString().c_str(), me.toString().c_str());
      announceConnected(0);
      return;
    }
  }
  bool taken = cand != me && tries > 0 && arpAnswered(cand);
  if (taken) {
    EVENT("[WiFi] %s 사용 중 (다른 장치가 응답)%s", cand.toString().c_str(), idx + 1 < n ? " → 다음 후보" : "");
    idx++;
    tries = 0;
    if (idx >= n) {
      ipMode = IPM_FALLBACK;
      EVENT("[WiFi] 고정 IP 후보가 모두 사용 중 — DHCP 주소 %s 유지", me.toString().c_str());
      announceConnected(0);
    }
    return;
  }
  if (cand == me || tries >= 3) {    // 지금 내 주소이거나, 3번 물어도 응답 없음 → 비어 있음
    IPAddress gw = WiFi.gatewayIP(), dns1 = WiFi.dnsIP(0), dns2 = WiFi.dnsIP(1);
    if (WiFi.config(cand, gw, mask, dns1, dns2)) {
      ipMode = IPM_STATIC;
      EVENT("[WiFi] 고정 IP %s 사용 (게이트웨이 %s, DHCP로 받았던 주소 %s)",
            cand.toString().c_str(), gw.toString().c_str(), me.toString().c_str());
    } else {
      ipMode = IPM_FALLBACK;
      EVENT("[WiFi] 고정 IP 설정 실패 — DHCP 주소 %s 유지", me.toString().c_str());
    }
    announceConnected(0);
    return;
  }
  arpAsk(cand);
  tries++;
}

// RSSI 기록: 2초마다, 최근 1분(30개) 평균·최저와 부팅 후 최저
static void sampleRssi() {
  static int8_t hist[30];
  static uint8_t cnt = 0, pos = 0;
  int r = WiFi.RSSI();
  if (r >= 0 || r < -127) return;
  hist[pos] = r;
  pos = (pos + 1) % 30;
  if (cnt < 30) cnt++;
  int sum = 0, mn = 0;
  for (int i = 0; i < cnt; i++) {
    sum += hist[i];
    if (i == 0 || hist[i] < mn) mn = hist[i];
  }
  wifiRssiAvg = sum / cnt;
  wifiRssiMin = mn;
  if (wifiRssiMinBoot == 0 || r < wifiRssiMinBoot) wifiRssiMinBoot = r;
}

// Wi-Fi 연결/끊김 처리(고정 IP 결정, 웹 서버·mDNS 시작, 재연결 감시)와 Modbus 통신 상태 변화를 이벤트로 남긴다
static void networkTask() {
  static unsigned long lastCheck = 0, lastRssi = 0, retryMs = 0, downSinceMs = 0;
  static uint32_t retries = 0;
  static bool wifiUp = false, mbLinkUp = false;
  unsigned long now = millis();
  if (now - lastCheck < 500) return;
  lastCheck = now;

  if (ENABLE_WEB) {
    bool up = WiFi.status() == WL_CONNECTED;
    if (up && !wifiUp) {
      wifiUpSinceMs = now;
      unsigned long downMs = downSinceMs ? now - downSinceMs : 0;
      downSinceMs = 0;
      retries = 0;
      if (WIFI_STATIC_IP && ipMode == IPM_DHCP) {
        ipMode = IPM_PROBING;     // 첫 연결: 고정 IP 후보 확인 후 웹 서버 시작
        EVENT("[WiFi] 공유기 연결됨 (DHCP %s, 채널 %d, RSSI %d dBm) — 고정 IP 후보 확인 중",
              WiFi.localIP().toString().c_str(), (int)WiFi.channel(), (int)WiFi.RSSI());
      } else {
        announceConnected(downMs);
      }
    } else if (!up && wifiUp) {
      wifiUpSinceMs = 0;
      downSinceMs = retryMs = now;
      wifiDrops = wifiDrops + 1;
      uint8_t rs = wifiDiscReason;
      EVENT("[WiFi] 연결 끊김 (사유 %u: %s, 직전 RSSI %d dBm) — 자동 재연결 시도 중",
            (unsigned)rs, wifiReasonText(rs), (int)wifiDiscRssi);
    }
    wifiUp = up;

    if (up) {
      ipProbeStep(now);
      if (now - lastRssi >= 2000) { lastRssi = now; sampleRssi(); }
    } else if (now - retryMs >= WIFI_RECONNECT_MS) {
      // 자동 재연결이 멈춘 경우를 대비해 연결을 처음부터 다시 시작 (로그는 처음 3번, 이후 10번마다)
      retryMs = now;
      retries++;
      uint8_t rs = wifiDiscReason;
      if (retries <= 3 || retries % 10 == 0) {
        EVENT("[WiFi] %lu초 넘게 연결 안 됨 — 연결을 처음부터 다시 시도 (%lu회째, 마지막 사유 %u: %s)",
              WIFI_RECONNECT_MS / 1000, (unsigned long)retries, (unsigned)rs, wifiReasonText(rs));
      }
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }
  }

  if (ENABLE_MODBUS) {
    bool link = mbReqCount > 0 && now - mbLastReqMs < MB_LINK_TIMEOUT_MS;
    if (link && !mbLinkUp) {
      EVENT("[485] PLC 요청 수신 시작 (FC%02u, 주소 %u, %s)", (unsigned)mbLastFc,
            (mbLastFc == 4 ? 30001u : 40001u) + mbLastAddr, mbLastOk ? "정상 응답" : "오류 응답");
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
  smpMutex = xSemaphoreCreateMutex();

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
  measMask = (uint8_t *)allocBig(IMG_N);
  measTmp = (uint8_t *)allocBig(IMG_N);
  if (measMask) memset(measMask, 0, IMG_N);
  initSizeTables();
  if (ENABLE_WEB) {
    dbgBuf = (uint8_t *)allocBig(DBG_W * DBG_H * 2);
    rawBuf = (uint8_t *)allocBig(IMG_N * 2);
    dbgMutex = xSemaphoreCreateMutex();
    if (dbgBuf) memset(dbgBuf, 0, DBG_W * DBG_H * 2);
    if (rawBuf) memset(rawBuf, 0, IMG_N * 2);
    if (!dbgBuf || !rawBuf) { dbgBuf = nullptr; EVENT("[ERR] 디버그 버퍼 할당 실패"); }
  }
  if (!bgR || !bgG || !bgB || !fgRaw || !fgMap || !bfsQueue || !footMask || !measMask || !measTmp) {
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
    mb.addHreg(0, 0, REG_COUNT);         // FC03 홀딩 레지스터 40001~ (40009는 PLC가 쓰는 명령)
    mb.addIreg(0, 0, REG_COUNT);         // FC04 입력 레지스터 30001~ — 같은 값 (PLC가 FC04로 읽는 경우)
    mb.onRequest(onMbRequest);           // 통신 상태(요청 수/마지막 요청) 집계
    mb.onRequestSuccess(onMbSuccess);
    mbMutex = xSemaphoreCreateRecursiveMutex();
    // 코어 1(메인 루프와 같은 코어)에서 웹 서버(우선순위 5)·메인 루프(1)보다 높은 우선순위로 → 영상 처리·JPEG 인코딩 중에도 응답
    xTaskCreatePinnedToCore(modbusTask, "modbus", 4096, nullptr, 6, nullptr, 1);
    EVENT("[485] Modbus RTU 시작: 슬레이브 ID %d, %dbps 8N1, FC03 40001~%d / FC04 30001~%d (UART0 TX=GPIO%d RX=GPIO%d)",
          SLAVE_ID, MODBUS_BAUD, 40000 + REG_COUNT, 30000 + REG_COUNT, MB_TX_PIN, MB_RX_PIN);
  }

  // Wi-Fi는 기다리지 않는다 (Modbus 운용 중 Wi-Fi가 없어도 멈추지 않게). 연결되면 networkTask()가 웹 서버를 시작
  if (ENABLE_WEB) {
    WiFi.onEvent(onWiFiDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.setHostname(HOSTNAME);
    WiFi.setSleep(false);                          // 절전 끄기: 약한 신호에서 패킷을 놓쳐 끊기는 일을 줄인다 (전류는 조금 늘어남)
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);     // 모든 채널을 검색해서
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL); // 같은 이름의 공유기가 여럿이면 신호가 가장 센 곳에 연결
    WiFi.mode(WIFI_STA);
    esp_wifi_set_country_code(WIFI_COUNTRY, true); // 채널 1~13
    if (WIFI_11B_ONLY) esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);  // 20MHz 폭: 붐비는 2.4GHz에서 간섭을 덜 받는다
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);           // 송신 출력 최대
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

  // 메인 루프 간격 측정 — 한 바퀴가 LOOP_STALL_WARN_MS를 넘으면 무엇인가 감지·판정을 막고 있다는 뜻
  static unsigned long prevLoopMs = 0, gapWinStart = 0, gapWin = 0, stallLogMs = 0;
  static uint32_t stallCount = 0;
  unsigned long loopNow = millis();
  if (prevLoopMs && !loopGapSkip) {
    unsigned long gap = loopNow - prevLoopMs;
    if (gap > gapWin) gapWin = gap;
    if (gap > loopGapMaxBoot) loopGapMaxBoot = gap;
    if (gap >= LOOP_STALL_WARN_MS) {
      stallCount++;
      if (stallLogMs == 0 || loopNow - stallLogMs >= 10000) {   // 감지 기록이 넘치지 않게 10초에 한 번만
        EVENT("[SYS] 메인 루프가 %lu ms 멈춤 (감지·판정이 그만큼 늦어짐, 누적 %lu회)", gap, (unsigned long)stallCount);
        stallLogMs = loopNow;
      }
    }
  }
  loopGapSkip = false;
  prevLoopMs = loopNow;
  if (loopNow - gapWinStart >= 10000) { loopGapMax10s = gapWin; gapWin = 0; gapWinStart = loopNow; }

  if (ENABLE_MODBUS) {
    // 웹 표시용 사본을 실제 레지스터 값으로 맞춘다 (PLC가 쓴 값 포함). 요청 응답은 modbusTask가 따로 처리
    mbLock();
    for (int i = 0; i < REG_COUNT; i++) regMirror[i] = mb.Hreg(i);
    mbUnlock();
  }
  pollCommand();   // 40009 명령 접수 (PLC 또는 웹)
  networkTask();

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
  servicePendingCommand();   // 명령 1·2: 판이 비었으면 배경 재학습 (약 3초 동안 여기서 머묾, 그동안도 Modbus는 응답)
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

  delay(1);
}
