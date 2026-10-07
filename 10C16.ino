#include <Servo.h>
#include <math.h>

// ---------------- 핀 설정 ----------------
#define PIN_SERVO 10
#define PIN_TRIG  12
#define PIN_ECHO  13

// ---------------- 초음파센서 설정 ----------------
#define SND_VEL 346.0          // 음속 (m/s)
#define INTERVAL 25            // 거리 측정 주기 (ms)
#define PULSE_DURATION 10      // TRIG 펄스 길이 (us)
#define DETECT_DISTANCE 150    // 이 거리(mm) 안에 들어오면 차량으로 판단
#define OUT_COUNT 20           // 연속 20번(0.5초) 멀리 측정되면 차량이 지나간 것으로 판단

#define TIMEOUT ((unsigned long)(INTERVAL * 1000UL / 2))  // 12.5ms (약 2m까지 측정)
#define SCALE (0.001 * 0.5 * SND_VEL)                       // duration(us) -> 거리(mm)

// ---------------- 차단기 설정 ----------------
#define ANGLE_DOWN 180         // 차단기 내림
#define ANGLE_UP   90          // 차단기 올림
unsigned long MOVING_TIME = 1500;  // 한 번 움직이는 데 걸리는 시간 (ms)

// ---------------- 사용할 함수 선택 ----------------
#define CURVE_SIGMOID    0
#define CURVE_SMOOTHSTEP 1

#define CURVE_TYPE CURVE_SIGMOID

#define SIGMOID_K 6.0          // sigmoid 기울기 (클수록 가운데에서 빨라짐)

Servo myServo;

unsigned long last_sampling_time = 0;
unsigned long moveStartTime;

int startAngle   = ANGLE_DOWN;
int stopAngle    = ANGLE_DOWN;
int currentAngle = ANGLE_DOWN;

bool isInside = false;   // 차량이 감지 범위 안에 있는지
bool isMoving = false;   // 차단기가 움직이는 중인지
int  farCount = 0;       // 연속으로 멀리 측정된 횟수


// function prototypes
float USS_measure(int TRIG, int ECHO);
double sigmoid(double x);
double smoothstep(double x);
double curve(double x);
void moveServo(int angle);
void updateServo();


void setup() {
  myServo.attach(PIN_SERVO);
  myServo.write(ANGLE_DOWN);   // 처음엔 차단기 내림

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Serial.begin(57600);
}


void loop() {
  updateServo();   // 서보는 loop가 돌 때마다 갱신

  // 거리 측정은 25ms마다
  if (millis() - last_sampling_time < INTERVAL)
    return;
  last_sampling_time += INTERVAL;

  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  Serial.print("Distance:");
  Serial.print(dist_raw);
  Serial.print(",Boundary:");
  Serial.print(DETECT_DISTANCE);
  Serial.print(",Angle:");
  Serial.println(currentAngle);

  // dist_raw < 0 : 2m 안에 아무것도 없음 (멀리 있는 것으로 처리)
  bool near = (dist_raw > 0 && dist_raw <= DETECT_DISTANCE);

  if (near) {
    farCount = 0;
    if (!isInside) {
      // 차량 감지 -> 차단기 올림
      isInside = true;
      moveServo(ANGLE_UP);
    }
  }
  else if (isInside) {
    // 센서 값이 한 번 튀는 것에 반응하지 않도록 연속으로 멀 때만 내림
    farCount++;
    if (farCount >= OUT_COUNT) {
      // 차량 통과 -> 차단기 내림
      isInside = false;
      farCount = 0;
      moveServo(ANGLE_DOWN);
    }
  }
}


// 초음파센서로 거리(mm) 측정
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, TIMEOUT);
  if (duration == 0)
    return -1.0;

  return duration * SCALE;
}


// ======================================================
// 서보 제어 함수 (입력 x: 진행 비율 0~1, 출력: 위치 비율 0~1)
// ======================================================

// (1) sigmoid 함수
// x(0~1)를 그대로 넣으면 sigmoid의 거의 직선인 부분만 써서 등속 이동과 비슷해진다.
// 그래서 x를 -k/2 ~ +k/2 범위로 넓혀서 S자 모양 전체를 사용하고,
// 시작값이 0, 끝값이 1이 되도록 정규화한다.
double sigmoid(double x) {
  double s0 = 1.0 / (1.0 + exp(SIGMOID_K / 2.0));    // x = 0 일 때
  double s1 = 1.0 / (1.0 + exp(-SIGMOID_K / 2.0));   // x = 1 일 때
  double s  = 1.0 / (1.0 + exp(-SIGMOID_K * (x - 0.5)));
  return (s - s0) / (s1 - s0);
}

// (2) smoothstep 함수: y = 3x^2 - 2x^3
// 곱셈만으로 계산하는 간단한 3차 다항식이다.
// x=0, x=1에서 기울기(속도)가 0이라 출발과 정지가 부드럽다.
double smoothstep(double x) {
  return x * x * (3.0 - 2.0 * x);
}

double curve(double x) {
#if CURVE_TYPE == CURVE_SIGMOID
  return sigmoid(x);
#else
  return smoothstep(x);
#endif
}


// 새 목표 각도로 이동 시작
void moveServo(int angle) {
  startAngle = currentAngle;   // 지금 위치에서 출발 (중간에 방향이 바뀌어도 튀지 않음)
  stopAngle  = angle;
  moveStartTime = millis();
  isMoving = true;
}


// 실습3 구조: 지난 시간으로 진행 비율을 구해서 각도 계산
void updateServo() {
  if (!isMoving)
    return;

  unsigned long progress = millis() - moveStartTime;

  if (progress <= MOVING_TIME) {
    double x = (double)progress / MOVING_TIME;   // 진행 비율 0~1
    double y = curve(x);                         // 위치 비율 0~1
    currentAngle = startAngle + (stopAngle - startAngle) * y;
    myServo.write(currentAngle);
  }
  else {
    currentAngle = stopAngle;    // 이동 끝
    myServo.write(stopAngle);
    isMoving = false;
  }
}
