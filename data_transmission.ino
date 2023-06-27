uint8_t package[16];
uint8_t rx_data[20];
int i = 0;

unsigned long _time, ptime;

void send_data_ros(float data1, float data2, float data3, float data4) {
  int _package[20];
  unsigned long setTime;
  union format_data {
    float asFloat;
    byte asByte[4];
  };

  format_data _data[4];

  _data[0].asFloat = data1;
  _data[1].asFloat = data2;
  _data[2].asFloat = data3;
  _data[3].asFloat = data4;

  _package[0] = '#';
  _package[1] = 'f';

  _package[2] = _data[0].asByte[0];
  _package[3] = _data[0].asByte[1];
  _package[4] = _data[0].asByte[2];
  _package[5] = _data[0].asByte[3];

  _package[6] = _data[1].asByte[0];
  _package[7] = _data[1].asByte[1];
  _package[8] = _data[1].asByte[2];
  _package[9] = _data[1].asByte[3];

  _package[10] = _data[2].asByte[0];
  _package[11] = _data[2].asByte[1];
  _package[12] = _data[2].asByte[2];
  _package[13] = _data[2].asByte[3];

  _package[14] = _data[3].asByte[0];
  _package[15] = _data[3].asByte[1];
  _package[16] = _data[3].asByte[2];
  _package[17] = _data[3].asByte[3];

  _package[18] = '\r';
  _package[19] = '\n';

  setTime = millis();
  while (millis() - setTime <= 10) {
    for (int i = 0; i < 20; i++) {
      Serial1.write(_package[i]);
    }
  }

}
void recieve_ros() {
  while (Serial1.available())
  {
    if (i <= 20)
    {
      rx_data[i] = Serial1.read() ;
      i++ ;
    }
    else
    {
      i = 0 ;
      break ;
    }
  }

  for (int j = 0 ; j < 20 ; j++)
  {
    while (rx_data[j] == 35) {
      int count = j;
      for (int i = 0; i < 16; i++) {
        package[i] = rx_data[count];
        count++;
      }
      if (package[0] == 35 && package[1] == 102 && package[14] == 13 && package[15] == 10) {
        data[0].asByte[0] = package[2];
        data[0].asByte[1] = package[3];
        data[0].asByte[2] = package[4];
        data[0].asByte[3] = package[5];

        data[1].asByte[0] = package[6];
        data[1].asByte[1] = package[7];
        data[1].asByte[2] = package[8];
        data[1].asByte[3] = package[9];

        data[2].asByte[0] = package[10];
        data[2].asByte[1] = package[11];
        data[2].asByte[2] = package[12];
        data[2].asByte[3] = package[13];

        break;

      }
      else {
        //Serial.println("error");
        break;
      }
    }
  }

}

void start_() {
  while (digitalRead(9) == 1) {
    digitalWrite(12, 1);
    bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); // sensor check manual
  }
  digitalWrite(12, 0);
}

void start_2() {
  while (analogRead(A7) < 300) {
    digitalWrite(12, 1);
    bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); // sensor check manual
  }
  digitalWrite(12, 0);
}

void wait() {
  count = 0;
  while (count <= 25) {
    recieve_ros();
    send_data_ros(x, y, map_yaw(ypr_readable[0]), 0);
    count += data[1].asFloat;
    digitalWrite(11, 1);
    bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); // sensor check manual
  }
  digitalWrite(11, 0);

}

void wait2() {
  count = 0;
  int state = 0;
  while (true) {
    recieve_ros();
    send_data_ros(x, y, map_yaw(ypr_readable[0]), 0);
    digitalWrite(11, 1);
    bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); // sensor check manual

    while (data[1].asFloat == 0) {
      Serial.println("run1");
      recieve_ros();
      while (data[1].asFloat == 1) {
        Serial.println("run2");
        recieve_ros();
        state = 3;
        break;
      }
      if (state == 3) {
        break;
      }
    }
    if (state == 3) {
      break;
    }
  }

  while (count < 15) {
    recieve_ros();
    send_data_ros(x, y, map_yaw(ypr_readable[0]), 0);
    bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); // sensor check manual
    count += data[1].asFloat;

    Serial.println("break");
  }
  data[1].asFloat = 0;
  
  digitalWrite(11, 0);

}



void wait_light() {
  count = 0;
  int setTime = millis();
  while (count <= 25) {
    recieve_ros();
    send_data_ros(x, y, map_yaw(ypr_readable[0]), 1);
    count += data[0].asFloat;
    digitalWrite(11, 1);
    bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); // sensor check manual

    if (data[1].asFloat == 1) {
      Serial.println("sawaddeekabbb");
      break;
    }

  }
  digitalWrite(11, 0);

}


void test_wait_light() {
  digitalWrite(11, 1);
  count = 0;
  while (true) {
    recieve_ros();
    send_data_ros(x, y, map_yaw(ypr_readable[0]), 1);
    if ( data[0].asFloat == 1) {
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);delay(500);
      delay(4500);
      data[0].asFloat = 0;
      send_data_ros(x, y, map_yaw(ypr_readable[0]), 2);
      break;
    }
    else if(data[0].asFloat == 2){
      data[0].asFloat = 0;
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);delay(300);
      send_data_ros(x, y, map_yaw(ypr_readable[0]), 2);
      break;
    }
    bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); // sensor check manual
  }
  
  digitalWrite(11, 0);

}
