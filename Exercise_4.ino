#include "Arduino.h"

// --- 핀 정의 ---
const int Resist_pin = A5;  
const int Right_handle = 7;
const int Left_handle = 8;  
const int Motor_L_1 = 3;
const int Motor_L_2 = 4;
const int Motor_R_1 = 5;                    
const int Motor_R_2 = 6;

// --- 설정 값 ---
const int center = 480;      
const int left_max = 580;    
const int right_max = 300;   
const int deadzone = 20;    
const float Kp = 0.7;        
const int steer_min = 90;    
const int steer_max = 110;  
const int drive_base_speed = 60;

// --- 상태 변수 ---
int target_val = center;    
float current_drive_speed = 0;

// 속도 계산 함수 (기존 유지)
float calculate_drive_speed(float angle) {
  float s;
  if(abs(angle) >= 25) {
    s = 40; // 커브 시 감속 (사용자 환경에 맞게 조정)
  } else {
    s = (float)drive_base_speed - (abs(angle) * abs(angle) / 100.0);
  }
  return s;
}

// 핸들 제어 함수 (기존 유지)
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

// 주행 함수
void move_forward(float val) {
  if (val <= 0) {
    digitalWrite(Motor_L_1, LOW); digitalWrite(Motor_L_2, LOW);
    digitalWrite(Motor_R_1, LOW); digitalWrite(Motor_R_2, LOW);
  } else {
    analogWrite(Motor_L_1, (int)val);
    digitalWrite(Motor_L_2, LOW);
    analogWrite(Motor_R_1, (int)val);
    digitalWrite(Motor_R_2, LOW);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  pinMode(Right_handle, OUTPUT); pinMode(Left_handle, OUTPUT);
  pinMode(Motor_L_1, OUTPUT); pinMode(Motor_L_2, OUTPUT);
  pinMode(Motor_R_1, OUTPUT); pinMode(Motor_R_2, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    float received_angle = Serial.parseFloat(); 
    int speed_mode = Serial.parseInt();         

    // --- 조향 타겟 결정 (문법 수정 완료) ---
    if (received_angle > 7.0) {
      target_val = right_max; // 오른쪽 하드 (회피)
    }
    else if (received_angle < -7.0) {
      target_val = left_max; // 일반 좌회전
    }
    else {
      target_val = center; // 직진
    }

    // --- 속도 결정 ---
    if (speed_mode == 1) {
      current_drive_speed = 0; // 정지
    } else {
      current_drive_speed = calculate_drive_speed(received_angle); // 주행
    }

    while(Serial.available() > 0) { Serial.read(); }
  }

  set_handle(target_val);
  move_forward(current_drive_speed);
  delay(5);
}