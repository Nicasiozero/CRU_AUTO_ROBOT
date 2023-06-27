
void encoder_L() {
  if (digitalRead(ENCODER_LA) != digitalRead(ENCODER_LB)) {
    pulseLeft++;
  } else {
    pulseLeft--;
  }
}

void encoder_R() {
  if (digitalRead(ENCODER_RA) != digitalRead(ENCODER_RB)) {
    pulseRight++;
  } else {
    pulseRight--;
  }
}

void point_to_point(float _x, float _y, int speed) {
  float distance_L, distance_R, avg_distance, x_current, y_current, x_target, y_target, distance, yaw_degree, avg_pulse;
  int *_speed;
  x_current = x;
  y_current = y;

  x_target = _x - x_current;
  y_target = _y - y_current;

  if (localization_mode == false) {
    avg_pulse = (pulseLeft + pulseRight) / 2;
    avg_distance = avg_pulse / 762.0;

  } else {
    distance_L = -1 * ((bus.Get(0).Position()) * (180 / PI)) / 360 * (2 * PI * 0.03);
    distance_R = -1 * ((bus.Get(1).Position() * -1) * (180 / PI)) / 360 * (2 * PI * 0.03);
    avg_distance = (distance_L + distance_R) / 2.0;
  }

  if (speed >= 0) {
    distance = sqrt(pow(x_target, 2) + pow(y_target, 2)) + avg_distance;
    yaw_degree = cal_traget_degree(x_target, y_target);
  }
  else {
    distance = avg_distance - sqrt(pow(x_target, 2) + pow(y_target, 2));
    yaw_degree = map_opposite_yaw(cal_traget_degree(x_target, y_target));
  }

  while (true) {
    imu_run();
    
    if (localization_mode == false) {
      update_position_omni(map_yaw(ypr_readable[0]));
      avg_pulse = (pulseLeft + pulseRight) / 2;
      avg_distance = avg_pulse / 762.0;
    } else {
      update_position_motor(map_yaw(ypr_readable[0]));
      distance_L = -1 * ((bus.Get(0).Position()) * (180 / PI)) / 360 * (2 * PI * 0.03);
      distance_R = -1 * ((bus.Get(1).Position() * -1) * (180 / PI)) / 360 * (2 * PI * 0.03);
      avg_distance = (distance_L + distance_R) / 2.0;
    }
    
    if (speed >= 0) {
      _speed = distance_control(distance, avg_distance, speed, kpD, kiD, kdD);
      if (avg_distance >= distance) {
        bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); delay(500);
        break;
      } else {
        heading_pid(yaw_degree, _speed[0], _speed[1], kpH, kiH, kdH);
      }
    }
    else {
      _speed = distance_control(abs(distance), abs(avg_distance), speed, kpD, kiD, kdD);
      if (avg_distance <= distance) {
        bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); delay(500);
        break;
      } else {
        heading_pid(yaw_degree, -_speed[0], -_speed[1], kpH, kiH, kdH);
      }
    }
  }
  send_data_ros(x, y, map_yaw(ypr_readable[0]), 10);
}


void BridgeCircular(int refL , int refR) {
  while (true) {
    imu_run();
    //update_position_omni(map_yaw(ypr_readable[0]));
    if ((map_yaw(ypr_readable[0]) >= 0 && map_yaw(ypr_readable[0]) <= 2) || (map_yaw(ypr_readable[0]) >= 358 && map_yaw(ypr_readable[0]) <= 360) ) {
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); delay(300);
      break;

    }
    if (analogRead(A8) < refL && analogRead(A9) > refR) {
      pid_controller(5, 40);
    }
    else if (analogRead(A8) > refL && analogRead(A9) < refR) {
      pid_controller(40, 5);
    }
    else if (analogRead(A8) > refL && analogRead(A9) > refR) {
      pid_controller(40, 40);
    }
    else{
      pid_controller(40, 40);
    }
   // send_data_ros(x, y, map_yaw(ypr_readable[0]), 10);
  }
}


void BridgeCircular2(int refL , int refR) {
  while (true) {
    imu_run();
    update_position_omni(map_yaw(ypr_readable[0]));
    if ((map_yaw(ypr_readable[0]) >= 178 && map_yaw(ypr_readable[0]) <= 182)) {
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); delay(300);
      break;

    }
    if (analogRead(A8) < refL && analogRead(A9) > refR) {
      pid_controller(5, 40);
    }
    else if (analogRead(A8) > refL && analogRead(A9) < refR) {
      pid_controller(40, 5);
    }
    else if (analogRead(A8) > refL && analogRead(A9) > refR) {
      pid_controller(40, 40);
    }
    else{
      pid_controller(40, 40);
    }
   // send_data_ros(x, y, map_yaw(ypr_readable[0]), 10);
  }
}


void stop_() {
  bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
  delay(50000000);

}
