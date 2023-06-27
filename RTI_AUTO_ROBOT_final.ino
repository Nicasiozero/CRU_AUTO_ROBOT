#include <Arduino.h>
#include <C610Bus.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>


MPU6050 mpu;
C610Bus<CAN0> bus;

#define ENCODER_LA 7                   //pinA left Encoder
#define ENCODER_LB 8                   //pinB left Encoder
#define ENCODER_RA 6                   //pinA right Encoder
#define ENCODER_RB 5                   //pinB right Encoder
#define INTERRUPT_PIN 17               //Interrupt pin imu

bool dmpReady = false;

uint8_t mpuIntStatus;
uint8_t devStatus;
uint8_t fifoBuffer[64];
uint16_t packetSize;
uint16_t fifoCount;      // count of all bytes currently in FIFO

int menu;
int pulseLeft;
int pulseRight;

long last_command = 0;

double kp = 250, ki = 1, kd = 0;        //motor control
double kpH = 3.5, kiH = 0.01, kdH = 0;  //heading control
double kpD = 400, kiD = 0, kdD = 0;     //distanc control

// pid kph 3.5 rpm 150-170
// pid kph <= 3.25 rpm  >= 200

float ypr[3];
float ypr_readable[3];

float x = 0;
float y = 0;
float previous_d = 0;

Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 gy;       // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y,U z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container

volatile bool mpuInterrupt = false;

int speed_motor = 130;

bool localization_mode = false;  // false = omni_localize, true = m2006 motor


union convert_data {
  float asFloat;
  byte asByte[4];
};

convert_data data[3];

float count = 0, count2 = 0;
float pointx_error, pointy_error;


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(ENCODER_LA, INPUT);
  pinMode(ENCODER_LB, INPUT);
  pinMode(ENCODER_RA, INPUT);
  pinMode(ENCODER_RB, INPUT);

  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);

  imu_init();

  attachInterrupt(ENCODER_LA, encoder_L, RISING);
  attachInterrupt(ENCODER_RA, encoder_R, RISING);
}

void loop() {
  //debugger_imu();
  // check_point3(30, 90, 800, 780);
  game();
  //debugger_sensor();
  //debugger_position("omni");
  //wait();
  //test_wait_light();
  // test_wait_light() ;/
  //Serial.println(analogRead(A7));
  // wait2();
}

void game() {
  start_();
  point_to_point(0.0, 1.48, 140);
  point_to_point(0.32, 1.48, 70);
  start_2();

  point_to_point(0.08, 1.48, -70);
  lock(90);

  wait2();
  point_to_point(0.03, 2.04, 100);
  point_to_point(0.58, 2.05, 70);
  check_point2(30, 0, 780, 770);
  point_to_point(x - 0.02, y, -50);
  L90();

  test_wait_light();
  point_to_point(-0.01, 0.75, 100);
  BridgeCircular(800, 780);
  lock(358);
  run_delay(3500, 356);
  check_point3(30, 356, 810, 780);
  point_to_point(x - 0.02, y, -50);
  check_point3(20, 356, 810, 780);
  point_to_point(x - 0.02, y, -50);
  R90();

  point_to_point(-0.01, 1.00, 90);
  wait2();
  point_to_point(-0.01, 1.38, 90);
  point_to_point(0.56, 1.38, 90);

  check_point2(30, 0, 800, 780);
  point_to_point(x - 0.02, y, -50);
  L90();

  test_wait_light();
  point_to_point(-0.01, 0.75, 100);

  BridgeCircular2(780, 770);
  lock(357);
  run_delay(3500, 357);
  //
  //lock(180);
  //
  check_point2(30, 357, 800, 780);
  point_to_point(x - 0.02, y, -50);
  L90();
  //
  //point_to_point(0, -0.50, -100);





  bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
  delay(50000000);
}

void check_point(int refL, int refR) {
  while (true) {
    if ((analogRead(A8) < refL && analogRead(A9) < refR ) || (analogRead(A8) > refL && analogRead(A9) > refR )) {
      break;
    }
    else if (analogRead(A8) > refL && analogRead(A9) < refR) {
      pid_controller(60, 40);
    }
    else if (analogRead(A8) < refL && analogRead(A9) > refR) {
      pid_controller(40, 60);
    }
  }
}


void check_point2(int speedM, int degree, int refL , int refR) {
  while (true) {
    imu_run();
    update_position_omni(map_yaw(ypr_readable[0]));
    if (analogRead(A8) < refL && analogRead(A9) < refR ) {
      break;
    }
    else {
      heading_pid(degree, speedM, speedM, kpH, kiH, kdH);
    }
  }
}

void check_point3(int speedM, int degree, int refL , int refR) {
  while (true) {
    imu_run();
    update_position_omni(map_yaw(ypr_readable[0]));
    if (analogRead(A8) < refL && analogRead(A9) < refR ) {
      break;
    }
    else if (analogRead(A8) > refL && analogRead(A9) < refR ) {
      pid_controller(0, speedM);
    }
    else if (analogRead(A8) < refL && analogRead(A9) > refR ) {
      pid_controller(speedM, 0);
    }
    else {
      heading_pid(degree, speedM, speedM, kpH, kiH, kdH);
    }
  }
}


void L90() {
  while (true) {
    imu_run();
    update_position_omni(map_yaw(ypr_readable[0]));
    if (map_yaw(ypr_readable[0]) == 90) {
      x = 0;
      y = 0;
      pulseLeft = 0;
      pulseRight = 0;
      previous_d = 0;
      break;
    }
    else {
      heading_pid(90, 30, 30, kpH, kiH, kdH);
    }
  }
}


void R90() {
  while (true) {
    imu_run();
    update_position_omni(map_yaw(ypr_readable[0]));
    if (map_yaw(ypr_readable[0]) == 270) {
      x = 0;
      y = 0;
      pulseLeft = 0;
      pulseRight = 0;
      previous_d = 0;
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); delay(300);
      imu_init();
      delay(500);
      break;
    }
    else {
      heading_pid(270, 20, 20, kpH, kiH, kdH);
    }
  }
}

void lock(int degree) {
  while (true) {
    imu_run();
    update_position_omni(map_yaw(ypr_readable[0]));
    if (map_yaw(ypr_readable[0]) == degree) {
      break;
    }
    else {
      heading_pid(degree, 30, 30, kpH, kiH, kdH);
    }
  }
}

void run_delay(int key, int degree) {
  int _setTime = millis();
  while (true) {
    imu_run();
    if (millis() - _setTime >= key) {
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks); delay(300);
      break;
    }
    else {
      heading_pid(degree, 100, 100, kpH, kiH, kdH);
    }
  }
}
