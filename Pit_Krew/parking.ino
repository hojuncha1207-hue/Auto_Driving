#include "Arduino.h"

// --- 핀 정의 ---
const int Resist_pin = A5;   
const int Right_handle = 7;   
const int Left_handle = 8;   
const int Motor_L_1 = 3;
const int Motor_L_2 = 4;
const int Motor_R_1 = 5;
const int Motor_R_2 = 6;

// --- 가변저항 값 (핸들 각도) ---
const int center = 480;       
const int left_max = 580;     
const int right_max = 300;   

// --- 제어 파라미터 ---
const int deadzone = 20;     
const float Kp = 0.7;         
const int steer_min = 90;     
const int steer_max = 110;   
const int drive_base_speed = 80;
const int parking_speed =50;

// --- 초음파 센서 (두목님, 선 확인 필수!) ---
// 아까 말씀하신 대로 수정했습니다. (반대면 숫자를 바꿔주세요)
const int trig_01 = A1; 
const int echo_01 = A0;

// --- 상태 변수 ---
long prevDistance = 0; 
bool parkingDone = false; 

// --- 함수 선언 ---
void set_handle(int target);
void wait_steer_ready(int target);
void move_forward(int val);
void move_backward(int val);
void stop_motors();
long measureDistance();

void setup() {
  Serial.begin(9600);
  
  pinMode(trig_01, OUTPUT);
  pinMode(echo_01, INPUT);
  
  pinMode(Resist_pin, INPUT);
  pinMode(Right_handle, OUTPUT);
  pinMode(Left_handle, OUTPUT);
  pinMode(Motor_L_1, OUTPUT);
  pinMode(Motor_L_2, OUTPUT);
  pinMode(Motor_R_1, OUTPUT);
  pinMode(Motor_R_2, OUTPUT);

  // 초기 거리 측정
  prevDistance = measureDistance(); 
  
  wait_steer_ready(center);
  
  Serial.println("두목님, 감시를 시작합니다.");
  delay(1000); 
}

void loop() {
  if (parkingDone) {
    stop_motors();
    return;
  }

  // 1. 평소 주행
  move_forward(drive_base_speed);
  set_handle(center); 

  // 2. 거리 측정
  long currentDistance = measureDistance();

  Serial.print("cm | 현재: "); Serial.println(currentDistance);

  if (currentDistance <= 40) {
  
  }

  prevDistance = currentDistance;
  delay(100); 
}

// --- 보조 함수들 ---


void set_handle(int target) {
  int current_value = analogRead(Resist_pin);
  int error = target - current_value;

  if (abs(error) <= deadzone) {
    analogWrite(Right_handle, 0);
    analogWrite(Left_handle, 0);
  } else {
    int steer_pwm = constrain(abs(error) * Kp, steer_min, steer_max);
    if (error < 0) {
      analogWrite(Right_handle, steer_pwm);
      digitalWrite(Left_handle, LOW);
    } else {
      digitalWrite(Right_handle, LOW);
      analogWrite(Left_handle, steer_pwm);
    }
  }
}

void move_forward(int val) {
  analogWrite(Motor_L_1, val);
  digitalWrite(Motor_L_2, LOW);
  analogWrite(Motor_R_1, val);
  digitalWrite(Motor_R_2, LOW);
}

void move_backward(int val) {
  digitalWrite(Motor_L_1, LOW);
  analogWrite(Motor_L_2, val);
  digitalWrite(Motor_R_1, LOW);
  analogWrite(Motor_R_2, val);
}

void stop_motors() {
  digitalWrite(Motor_L_1, LOW);
  digitalWrite(Motor_L_2, LOW);
  digitalWrite(Motor_R_1, LOW);
  digitalWrite(Motor_R_2, LOW);
}
