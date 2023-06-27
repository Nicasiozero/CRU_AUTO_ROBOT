void debugger_position(String topic) {
  imu_run();
  if(topic == "omni"){
    update_position_omni(map_yaw(ypr_readable[0]));
  }
  else{
    update_position_motor(map_yaw(ypr_readable[0]));
  }
  Serial.println("x : " + String(x) + " y : " + String(y) + " yaw :" + String(map_yaw(ypr_readable[0])));
  send_data_ros(x, y, map_yaw(ypr_readable[0]), 0);
}

void debugger_imu() {
  imu_run();
  Serial.println("yaw_degree : " + String(map_yaw(ypr_readable[0])) + " yaw_orginal : " + String(ypr_readable[0]));
  send_data_ros(0, 0, map_yaw(ypr_readable[0]), ypr_readable[0]);
}

void debugger_encoder(){
  Serial.println("pulseLeft : " + String(pulseLeft) + " pulseRight : " + String(pulseRight));
  send_data_ros(pulseLeft, pulseRight, 0, 0);
}

void debugger_sensor(){
  Serial.println(String(analogRead(A8)) + " : " +String(analogRead(A9)));
  delay(10);
}
