void imu_init() {
  // initialize i2c
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(0x68);
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(89);
  mpu.setYGyroOffset(-3);
  mpu.setZGyroOffset(-22);
  mpu.setXAccelOffset(-4548);
  mpu.setYAccelOffset(-1006);
  mpu.setZAccelOffset(1366);


  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void dmpDataReady() {
  mpuInterrupt = true;
}

void imu_run() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    ypr_readable[0] = (ypr[0] * 180 / M_PI);
    ypr_readable[1] = (ypr[1] * 180 / M_PI);
    ypr_readable[2] = (ypr[2] * 180 / M_PI);
  }
}

int map_yaw(float value) {
  float yaw_degree = 0;

  if (value >= 0 && value <= 90) {
    yaw_degree = map(value * -1, 0, -90, 90, 0);
  } else if (value > 90 && value <= 180) {
    yaw_degree = map(value, 90, 180, 360, 270);
  } else if (value >= -90 && value <= 0) {
    yaw_degree = map(value * -1, 0, 90, 90, 180);
  } else if (value >= -90 && value <= 0) {
    yaw_degree = map(value * -1, 0, 90, 90, 180);
  } else if (value <= -90 && value >= -180) {
    yaw_degree = map(value * -1, 90, 180, 180, 270);
  }

  return yaw_degree;  
}

int cal_traget_degree(float x, float y) {
  float yaw_degree = 0;

  if (x >= 0 && y >= 0) {
    yaw_degree = abs(atan(y / x) * (180 / PI));
  } else if (x < 0 && y >= 0) {
    yaw_degree = 180 - abs(atan(y / x) * (180 / PI));
  } else if (x < 0 && y < 0) {
    yaw_degree = 180 + abs(atan(y / x) * (180 / PI));
  } else if (x >= 0 && y < 0) {
    yaw_degree = 360 - abs(atan(y / x) * (180 / PI));
  }

  return yaw_degree;
}

int map_opposite_yaw(int now_degree) {
  int oppsite_degree = 0;
  if (now_degree >= 0 && now_degree <= 180) {
    oppsite_degree = map(now_degree, 0, 180, 180, 360);
  }
  else if (now_degree > 180 && now_degree < 360) {
    oppsite_degree = map(now_degree, 180, 360, 0, 180);
  }

  return oppsite_degree;
}
