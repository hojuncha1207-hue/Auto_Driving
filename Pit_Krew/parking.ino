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
const int parking_speed = 50;

// --- 초음파 센서 ---
const int trig_01 = A1;
const int echo_01 = A0;

// --- 상태 변수 ---
bool parkingDone = false;

// --- 함수 선언 ---
void set_handle(int target);
void parking_start();
void wait_steer_ready(int target);
void move_forward(int val);
void move_backward(int val);
void stop_motors();
long measureDistance();
bool Check_Val(int value) {
    // static 변수는 함수가 종료되어도 값이 초기화되지 않고 메모리에 남습니다.
    static int count = 0;

    if (value < 60) {
        count++; // 조건 만족 시 카운트 증가
    }
    else {
        count = 0; // 조건 불만족 시 즉시 리셋
    }

    // 5번 이상 연속으로 조건을 만족했는지 확인
    if (count >= 5) {
        return true;
    }
    else {
        return false;
    }
}
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

    wait_steer_ready(center);

    Serial.println("두목님, 감시를 시작합니다.");
    delay(1000);
}
void loop() {
    // 주차가 완료되었으면 모터 끄고 루프 종료 (아무것도 안 함)
    if (parkingDone) {
        stop_motors();
        return;
    }

    // 1. 평소 주행 (감시 중)
    move_forward(drive_base_speed);

    set_handle(center);

    // 2. 거리 측정
    long currentDistance = measureDistance();

    Serial.print("cm | 현재: "); Serial.println(currentDistance);

    if (Check_Val(currentDistance)) {
        // 5번 연속 감지 성공! -> 주차 공간 발견으로 판단
        Serial.println(">>> 주차 공간 발견! 정지합니다. <<<");

        stop_motors();
        delay(1000); 

        parking_start();
        parkingDone = true;
    }

}
// --- 보조 함수들 ---
long measureDistance() {
    long duration, distance;

    // 1. 일단 트리거 핀을 깨끗하게 비워줍니다 (Low)
    digitalWrite(trig_01, LOW);
    delayMicroseconds(2);

    // 2. 초음파 발사! (10마이크로초 동안 High 유지)
    digitalWrite(trig_01, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_01, LOW);

    duration = pulseIn(echo_01, HIGH, 30000);

    if (duration == 0) {
        return 999;
    }

    distance = duration * 0.034 / 2;

    return distance;
}

void set_handle(int target) {
    int current_value = analogRead(Resist_pin);
    int error = target - current_value;

    if (abs(error) <= deadzone) {
        analogWrite(Right_handle, 0);
        analogWrite(Left_handle, 0);
    }
    else {
        int steer_pwm = constrain(abs(error) * Kp, steer_min, steer_max);
        if (error < 0) {
            analogWrite(Right_handle, steer_pwm);
            digitalWrite(Left_handle, LOW);
        }
        else {
            digitalWrite(Right_handle, LOW);
            analogWrite(Left_handle, steer_pwm);
        }
    }
}
void parking_start() {
    set_handle(left_max);
    move_forward(parking_speed);
	delay(3000);
    stop_motors();
    set_handle(right_max);
    delay(500);
    move_backward(parking_speed);
    delay(1000);
	set_handle(center);
    move_backward(parking_speed);
    delay(3000);
//---------------------------------------------
    stop_motors();
    delay(3000);
    move_forward(parking_speed);
    delay(1500);
    set_handle(right_max);
    move_forward(parking_speed);
    delay(6000);
	set_handle(center);
    delay(50000);
	
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
