
void update_position_motor(float yaw) {
  float distance_L = -1 * ((bus.Get(0).Position()) * (180 / PI)) / 360 * (2 * PI * 0.03);
  float distance_R = -1 * ((bus.Get(1).Position() * -1) * (180 / PI)) / 360 * (2 * PI * 0.03);
  float avg_distance = (distance_L + distance_R) / 2.0;
  float yaw_radian = abs(yaw) * (PI / 180);

  x += ((avg_distance - previous_d) * cos(yaw_radian));
  y += ((avg_distance - previous_d) * sin(yaw_radian));
  previous_d = avg_distance;
}

void update_position_omni(float yaw) {
  float avg_pulse = (pulseLeft + pulseRight) / 2;
  float distance = avg_pulse / 762.0;
  float yaw_radian = abs(yaw) * (PI / 180);

  x += ((distance - previous_d) * cos(yaw_radian));
  y += ((distance - previous_d) * sin(yaw_radian));
  previous_d = distance;
}
