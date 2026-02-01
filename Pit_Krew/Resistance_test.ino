// --- 핀 정의 ---
const int Resist_pin = A5;   // 가변저항 연결 핀
const int Right_handle = 7;  // 모터 드라이버 입력 1
const int Left_handle = 8;   // 모터 드라이버 입력 2

// --- 가변저항 목표값 (핸들 각도) ---
const int center = 480;      // 중앙
const int left_max = 580;    // 왼쪽 끝
const int right_max = 300;   // 오른쪽 끝

// --- 제어 파라미터 (필요시 조정) ---
const int deadzone = 15;     // 오차 허용 범위 (이 범위 안이면 멈춤)
const float Kp = 1.2;        // 비례 제어 상수 (반응 속도)
const int steer_min = 80;    // 모터 최소 PWM (힘이 너무 약하면 안 움직임)
const int steer_max = 200;   // 모터 최대 PWM (너무 빠르면 확 꺾임)

// --- 변수 ---
int target_angle = center;   // 초기 목표는 중앙

void setup() {
  Serial.begin(9600); // 시리얼 통신 시작

  pinMode(Resist_pin, INPUT);
  pinMode(Right_handle, OUTPUT);
  pinMode(Left_handle, OUTPUT);
  
  // 시작하자마자 현재 위치 확인
  target_angle = center; 
  Serial.println(">>> 핸들 제어 테스트 시작 <<<");
  Serial.println("명령어: L(왼쪽), R(오른쪽), S(중앙)");
}

void loop() {
  // 1. 시리얼 입력 받기 (명령어 처리)
  if (Serial.available() > 0) {
    char cmd = Serial.read(); // 문자 읽기
    
    // 대소문자 구분 없이 처리
    if (cmd == 'L' || cmd == 'l') {
      target_angle = left_max;
      Serial.println("명령: 왼쪽으로 이동 (Left)");
    } 
    else if (cmd == 'R' || cmd == 'r') {
      target_angle = right_max;
      Serial.println("명령: 오른쪽으로 이동 (Right)");
    } 
    else if (cmd == 'S' || cmd == 's') {
      target_angle = center;
      Serial.println("명령: 중앙 정렬 (Straight)");
    }
  }

  // 2. 현재 가변저항 값 읽기
  int current_val = analogRead(Resist_pin);

  // 3. 모터 제어 (목표값으로 이동)
  set_handle(target_angle, current_val);

  // 4. 상태 모니터링 출력
  Serial.print("목표값: ");
  Serial.print(target_angle);
  Serial.print(" \t| 현재값: ");
  Serial.print(current_val);
  Serial.print(" \t| 오차: ");
  Serial.println(target_angle - current_val);

  delay(100); // 너무 빠른 출력 방지
}

// --- 핸들 모터 제어 함수 ---
void set_handle(int target, int current) {
  int error = target - current; // 목표 - 현재

  // 오차가 허용 범위(deadzone) 이내면 정지
  if (abs(error) <= deadzone) {
    digitalWrite(Right_handle, LOW);
    digitalWrite(Left_handle, LOW);
  } 
  else {
    // PWM 속도 계산 (오차가 클수록 빠르게)
    int steer_pwm = constrain(abs(error) * Kp, steer_min, steer_max);

    // 방향 결정
    // 값이 커지는 방향이 왼쪽(580)이라고 가정 (배선에 따라 다를 수 있음)
    if (error > 0) { 
      // 목표가 더 크다 -> 값을 키워야 한다 -> 왼쪽 모터 구동
      // (만약 반대로 움직이면 이 아래 두 줄을 서로 바꾸세요)
      digitalWrite(Right_handle, LOW);
      analogWrite(Left_handle, steer_pwm);
    } 
    else {
      // 목표가 더 작다 -> 값을 줄여야 한다 -> 오른쪽 모터 구동
      analogWrite(Right_handle, steer_pwm);
      digitalWrite(Left_handle, LOW);
    }
  }
}
