
void pid_controller(int set_pointM1, int set_pointM2) {                                                                     // control the motor towards the setpoint value
  static int errorM1 = 0, errorM2 = 0, sum_error_M1 = 0, sum_error_M2 = 0, rpm_M1 = 0, rpm_M2 = 0, mA_M1 = 0, mA_M2 = 0;
  static int previous_errorM1 = 0, previous_errorM2 = 0;

  bus.PollCAN();

  rpm_M1 = abs((bus.Get(0).Velocity() / (2 * PI)) * 60);
  rpm_M2 = abs((bus.Get(1).Velocity() / (2 * PI)) * 60);

  errorM1 = abs(set_pointM1) - rpm_M1;
  errorM2 = abs(set_pointM2) - rpm_M2;


  mA_M1 = (kp * errorM1) + (ki * sum_error_M1) + (kd * (errorM1 - previous_errorM1));
  mA_M2 = (kp * errorM2) + (ki * sum_error_M2) + (kd * (errorM2 - previous_errorM2));
  sum_error_M1 += errorM1;
  sum_error_M2 += errorM2;

  //  Serial.println("ML : " + String(rpm_M1) + ", MR : " + String(rpm_M2) + " : error : " + String(errorM1) + " : " + String(errorM2));

  if (sum_error_M1 >= set_pointM1) {
    sum_error_M1 = set_pointM1;
  }
  if (sum_error_M1 <= -abs(set_pointM1)) {
    sum_error_M1 = -abs(set_pointM1);
  }
  if (sum_error_M2 >= set_pointM2) {
    sum_error_M2 = set_pointM2;
  }
  if (sum_error_M2 <= -abs(set_pointM2)) {
    sum_error_M2 = -abs(set_pointM2);
  }

  if (mA_M1 >= 1000) {
    mA_M1 = 1000;
  }
  if (mA_M1 <= -1000) {
    mA_M1 = -1000;
  }
  if (mA_M2 >= 1000) {
    mA_M2 = 1000;
  }
  if (mA_M2 <= -1000) {
    mA_M2 = -1000;
  }

  if (set_pointM1 >= 0 && set_pointM2 >= 0) {
    long now = millis();
    if (now - last_command >= 10) {
      last_command = now;
      bus.CommandTorques(-1 * mA_M1, mA_M2, 0, 0, C610Subbus::kOneToFourBlinks);
    }
  } else if (set_pointM1 < 0 && set_pointM2 < 0) {
    long now = millis();
    if (now - last_command >= 10) {
      last_command = now;

      bus.CommandTorques(mA_M1, -1 *  mA_M2, 0, 0, C610Subbus::kOneToFourBlinks);
    }
  } else if (set_pointM1 >= 0 && set_pointM2 < 0) {
    long now = millis();
    if (now - last_command >= 10) {
      last_command = now;

      bus.CommandTorques(-mA_M1, -mA_M2, 0, 0, C610Subbus::kOneToFourBlinks);
    }
  } else if (set_pointM1 < 0 && set_pointM2 >= 0) {
    long now = millis();
    if (now - last_command >= 10) {
      last_command = now;

      bus.CommandTorques(mA_M1, mA_M2, 0, 0, C610Subbus::kOneToFourBlinks);
    }

  }

  previous_errorM1 = errorM1;
  previous_errorM2 = errorM2;
}


void heading_pid(int setPoint, int base_speed, int max_speed, float _kp, float _ki, float _kd) {
  static int error = 0, sum_error = 0;
  static int motor_speed;
  static int speedL = 0;
  static int speedR = 0;
  static int previous_error = 0;

  error = setPoint - map_yaw(ypr_readable[0]);

  if (error < -180) {
    error += 360;
  };
  if (error > 180) {
    error -= 360;
  };

  motor_speed = (_kp * error) + (_ki * sum_error) + (_kd * (error - previous_error));
  speedL = base_speed - motor_speed;
  speedR = base_speed + motor_speed;

  if (sum_error >= setPoint) {
    sum_error = abs(setPoint);
  }
  if (sum_error <= -abs(setPoint)) {
    sum_error = -abs(setPoint);
  }

  if (base_speed >= 0) {
    if (speedL >= max_speed) {
      speedL = max_speed;
    }
    if (speedR >= max_speed) {
      speedR = max_speed;
    }
  }
  else {
    if (speedL < max_speed) {
      speedL = max_speed;
    }
    if (speedR < max_speed) {
      speedR = max_speed;
    }
  }

  previous_error = error;
  sum_error += error;

  pid_controller(speedR, speedL);
}


int *distance_control(float setPoint, float now_distance, int base_speed, float _kp, float _ki, float _kd) {
  static float error = 0, sum_error = 0;
  static int motor_speed;
  static int speed[2];
  static float previous_error = 0;

  if (base_speed >= 0) {
    error = setPoint - now_distance;
    motor_speed = (_kp * error) + (_ki * sum_error) + (_kd * (error - previous_error));
  }
  else {
    error = now_distance - setPoint;
    motor_speed = (_kp * error) + (_ki * sum_error) + (_kd * (error - previous_error));
  }


  speed[0] = (base_speed + 50) - (base_speed - motor_speed);
  speed[1] = (base_speed + 50) - (base_speed - motor_speed);



  if (speed[0] >= base_speed) {
    speed[0] = base_speed;
  }
  if (speed[1] >= base_speed) {
    speed[1] = base_speed;
  }

  if (speed[0] < -base_speed) {
    speed[0] = -base_speed;
  }
  if (speed[1] < -base_speed) {
    speed[1] = -base_speed;
  }

  previous_error = error;
  sum_error += error;

  if (sum_error >= setPoint) {
    sum_error = abs(setPoint);
  }
  if (sum_error <= -abs(setPoint)) {
    sum_error = -abs(setPoint);
  }

  return speed;
}
