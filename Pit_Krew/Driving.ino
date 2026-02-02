#include "Arduino.h"


// --- 핀 정의 ---
const int Resist_pin = A5;  
const int Right_handle = 7;  // [물리적 확인] 값을 줄이는 방향 (좌회전 수행)
const int Left_handle = 8;   // [물리적 확인] 값을 키우는 방향 (우회전 수행)
const int Motor_L_1 = 3;
const int Motor_L_2 = 4;
const int Motor_R_1 = 5;
const int Motor_R_2 = 6;


// --- 두목님이 새로 잡으신 가변저항 값 적용 ---
const int center = 480;      // S (직진)
const int left_max = 580;    // L (좌회전)
const int right_max = 300;   // R (우회전)


// --- 제어 파라미터 ---
const int deadzone = 20;    
const float Kp = 0.7;        
const int steer_min = 90;    
const int steer_max = 110;  
const int drive_base_speed = 120;


// --- 상태 변수 ---
int target_val = center;    
float current_drive_speed = 0;


// 함수 선언 (setup에서 사용하기 위해 미리 알림)
void set_handle(float target);




// 조향각에 따른 가변 속도 계산
float calculate_drive_speed(float angle) {
  float s;
  if(abs(angle) >= 25) {
    s = 80; // 급커브 감속
  } else {
    s = abs(float(drive_base_speed) - (abs(angle) * abs(angle) / 100.0));
 
  return s;
}
}


// 핸들 제어 함수
void set_handle(float target) {
  int current_value = analogRead(Resist_pin);
  int error = target - current_value;


  if (abs(error) <= deadzone) {
    analogWrite(Right_handle, 0);
    analogWrite(Left_handle, 0);
  }
  else {
    int steer_pwm = constrain(abs(error) * Kp, steer_min, steer_max);


    if (error < 0) {
      // 현재값이 목표보다 큼 -> 값을 줄여야 함 (좌회전)
      analogWrite(Right_handle, steer_pwm);
      digitalWrite(Left_handle, LOW);
    }
    else {
      // 현재값이 목표보다 작음 -> 값을 키워야 함 (우회전)
      digitalWrite(Right_handle, LOW);
      analogWrite(Left_handle, steer_pwm);
    }
  }
}


void move_forward(float val) {
  analogWrite(Motor_L_1, (int)val);
  digitalWrite(Motor_L_2, LOW);
  analogWrite(Motor_R_1, (int)val);
  digitalWrite(Motor_R_2, LOW);
}


void setup() {
  Serial.begin(9600);
  pinMode(Right_handle, OUTPUT);
  pinMode(Left_handle, OUTPUT);
  pinMode(Motor_L_1, OUTPUT);
  pinMode(Motor_L_2, OUTPUT);
  pinMode(Motor_R_1, OUTPUT);
  pinMode(Motor_R_2, OUTPUT);
}
void strait(){
  
}
void loop() {
  // 1. 파이썬 명령 수신
  if (Serial.available() > 0) {
    float received_angle = Serial.parseFloat();


    current_drive_speed = calculate_drive_speed(received_angle);
   
    // --- [수정 핵심] ---
    // 직접 모터를 움직이지 않고, '목표 변수(target_val)'만 바꿔줍니다.
    // 이렇게 해야 루프 맨 아래에서 안정적으로 모터를 제어할 수 있습니다.
   /*const int center = 480;      // S (직진)
  const int left_max = 580;    // L (좌회전)
  const int right_max = 300;   // R (우회전)
*/
    if (received_angle > 7) {
      target_val = right_max
    }
    else if (received_angle < -7) {
      // -7도보다 작으면 좌회전 고정
      target_val = left_max;
    }
    else {
      // 그 사이값(-7 ~ 7)이면 중앙 정렬
      target_val = center+(pow(center-received_angle)/2)
    }

    // 통신 버퍼 비우기 (밀림 방지)
    while(Serial.available() > 0) { Serial.read(); }
  }


  // 2. 조향 및 주행 유지 (if문 밖에서 계속 실행되어야 함)
  // 위에서 결정된 target_val을 따라 모터가 힘을 계속 유지합니다.
  set_handle(target_val);
  move_forward(current_drive_speed);


  delay(5);
}
