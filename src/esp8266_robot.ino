#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <string.h>

// TODO: 환경에 맞게 수정
const char* WIFI_SSID = "iptime";
const char* WIFI_PASS = "happy3938!";
const int UDP_PORT = 4210;

WiFiUDP udp;

// L298N 기준 핀 정의
// 좌측 모터: IN1/IN2 + ENA(PWM)
// 우측 모터: IN3/IN4 + ENB(PWM)
// TODO: 실제 배선에 맞게 수정
const int IN1 = D1;
const int IN2 = D2;
const int ENA = D3;
const int IN3 = D5;
const int IN4 = D6;
const int ENB = D7;

unsigned long lastCmdMs = 0;
const unsigned long CMD_TIMEOUT_MS = 500;
// 모터 튜닝값: 차체마다 다르므로 필요 시 숫자만 조절
const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = false;
const float LEFT_GAIN = 1.00f;
const float RIGHT_GAIN = 1.00f;
const int MIN_EFFECTIVE_PWM = 95;  // 이 값보다 낮으면 모터가 못 도는 경우가 많음
const int MAX_EFFECTIVE_PWM = 255;

int applyMotorTuning(int cmd, bool invert, float gain) {
  // 좌우 편차 보정
  int tuned = (int)(cmd * gain);
  tuned = constrain(tuned, -255, 255);

  if (invert) {
    tuned = -tuned;
  }

  // 사소한 PWM(죽은구간)은 최소 구동 PWM으로 끌어올림
  if (tuned > 0 && tuned < MIN_EFFECTIVE_PWM) {
    tuned = MIN_EFFECTIVE_PWM;
  } else if (tuned < 0 && tuned > -MIN_EFFECTIVE_PWM) {
    tuned = -MIN_EFFECTIVE_PWM;
  }

  tuned = constrain(tuned, -MAX_EFFECTIVE_PWM, MAX_EFFECTIVE_PWM);
  return tuned;
}

void setMotor(int in1, int in2, int pwmPin, int value) {
  value = constrain(value, -255, 255);

  if (value > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, value);
  } else if (value < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -value);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void stopAll() {
  setMotor(IN1, IN2, ENA, 0);
  setMotor(IN3, IN4, ENB, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  analogWriteRange(255);
  stopAll();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
  Serial.print("UDP listening on port ");
  Serial.println(UDP_PORT);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char buf[64] = {0};
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';

      // 노트북 연결 확인: "PING" -> 동일 발신지로 "PONG" (모터 명령 아님)
      if (strncmp(buf, "PING", 4) == 0) {
        udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write((const uint8_t*)"PONG", 4);
        udp.endPacket();
        Serial.println("[UDP] PING received -> PONG sent");
      } else {
        int left = 0;
        int right = 0;
        // 입력 포맷: "120,-80"
        if (sscanf(buf, "%d,%d", &left, &right) == 2) {
          int leftOut = applyMotorTuning(left, INVERT_LEFT_MOTOR, LEFT_GAIN);
          int rightOut = applyMotorTuning(right, INVERT_RIGHT_MOTOR, RIGHT_GAIN);
          setMotor(IN1, IN2, ENA, leftOut);
          setMotor(IN3, IN4, ENB, rightOut);
          lastCmdMs = millis();

          // 실제 출력값 확인용 로그
          static unsigned long lastLogMs = 0;
          if (millis() - lastLogMs > 250) {
            Serial.printf("[MOTOR] in L=%d R=%d -> out L=%d R=%d\n", left, right, leftOut, rightOut);
            lastLogMs = millis();
          }
        }
      }
    }
  }

  // 통신 끊김 시 안전 정지
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopAll();
  }
}
