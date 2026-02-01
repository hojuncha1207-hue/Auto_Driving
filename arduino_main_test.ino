void loop() {
  if (Serial.available() > 0) {
    float received_angle = Serial.parseFloat(); 
    int speed_mode = Serial.parseInt();         

    // --- 조향 타겟 결정 (문법 오류 수정) ---
    if (received_angle >= 99.0) {
      target_val = 200; // 오른쪽 급회전 (회피)
    }
    else if (received_angle > 7.0 && received_angle < 99.0) {
      target_val = right_max; // 일반 우회전
    }
    else if (received_angle < -7.0 && received_angle > -99.0) {
      target_val = left_max; // 일반 좌회전
    }
    else if (received_angle <= -99.0) {
      target_val = 680; // 왼쪽 급회전 (회피)
    }
    else {
      target_val = center; // 직진
    }

    // --- 속도 제어 로직 ---
    if (speed_mode == 1) {
      current_drive_speed = 0;  // 정지 (신호등)
    } 
    else if (speed_mode == 2) {
      current_drive_speed = 50; // 회피 주행 (더 조심스럽게 서행)
    } 
    else {
      // 일반 주행: 각도에 따른 가변 속도
      current_drive_speed = calculate_drive_speed(received_angle);
    }

    // 버퍼 비우기
    while(Serial.available() > 0) { Serial.read(); }
  }

  set_handle(target_val);
  move_forward(current_drive_speed);
  delay(5);
}