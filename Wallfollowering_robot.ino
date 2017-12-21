#include <PID_v1.h>

#define PIN_DISTANCE_LEFT A0
#define PIN_DISTANCE_CENTER A6
#define PIN_DISTANCE_RIGHT A7

#define RED_LED A3
#define GREEN_LED A2
#define PIN_BUTTON A1

#define PIN_ENCODER_LEFT_A 11
#define PIN_ENCODER_LEFT_B 2
#define PIN_ENCODER_RIGHT_A 12
#define PIN_ENCODER_RIGHT_B 3

#define PIN_MOTOR_LEFT_1 9
#define PIN_MOTOR_LEFT_2 10
#define PIN_MOTOR_RIGHT_1 5
#define PIN_MOTOR_RIGHT_2 6


// Invert encoder directions if needed
const boolean INVERT_ENCODER_LEFT = true;
const boolean INVERT_ENCODER_RIGHT = true;

// Invert motor directions if needed
const boolean INVERT_MOTOR_LEFT = true;
const boolean INVERT_MOTOR_RIGHT = false;


// Loop count, used for print statements
int count = 0;
float encoder_val = 0.96;
float p_correction = 0.0;

float k_p_a = 0.4;
float k_i_a = 0.05;
float k_d_a = 0.0;

float k_p_l = 0.004;
float k_i_l = 0.0005;
float k_d_l = 0.0;

float error = 0.0;
double velocity_linear = 0;
double velocity_linear_power;

double velocity_angular;
double velocity_angular_power;

double va_setpoint = 0.0;
double vl_setpoint = 180.0;

//Wall Correcting Constants
float k_p_rl = 0.06; // Sweet spot is around 0.07 to 0.10. I set this lower so it doesn't jerk with gaps in the wall.
float k_i_rl = 0.0;
float k_d_rl = 0.0;

double lw_setpoint = 20.0;
double rw_setpoint = 20.0; // used to be 22.0

double lw_distance = 0.0;
double rw_distance = 0.0;
double center_distance = 0.0;

int turn_ready_count = 0;
//int turn_left_ready_count1 = 0;
//int turn_left_ready_count2 = 0;
//int turn_right_ready_count = 0;

boolean nextTurnLeft = false;

PID velocity_linear_pid(&velocity_linear, &velocity_linear_power, &vl_setpoint, k_p_l, k_i_l, k_d_l, DIRECT);

PID velocity_angular_pid(&velocity_angular, &velocity_angular_power, &va_setpoint, k_p_a, k_i_a, k_d_a, DIRECT);

PID lw_correction_pid(&lw_distance, &va_setpoint, &lw_setpoint, k_p_rl, k_i_rl, k_d_rl, DIRECT);

PID rw_correction_pid(&rw_distance, &va_setpoint, &rw_setpoint, k_p_rl, k_i_rl, k_d_rl, REVERSE);

int useRW = 0;

int switchOnRw = 0;
int switchOffRw = 0;

double switchThresholdFar = 25.0; // 25.0 works best so far
double switchThresholdNear = 20.0; // 20.0 works best so far

boolean debugMode = false;

int switch_ready_threshold = 125;

void setup() {
  Serial.begin(9600);
  pinSetup();

  velocity_linear_pid.SetOutputLimits(-1.0, 1.0);
  velocity_linear_pid.SetSampleTime(10);
  velocity_linear_pid.SetMode(AUTOMATIC);

  velocity_angular_pid.SetOutputLimits(-1.0, 1.0);
  velocity_angular_pid.SetSampleTime(10);
  velocity_angular_pid.SetMode(AUTOMATIC);

  rw_correction_pid.SetOutputLimits(-1.0, 1.0);
  rw_correction_pid.SetSampleTime(10);
  rw_correction_pid.SetMode(AUTOMATIC);

  lw_correction_pid.SetOutputLimits(-1.0, 1.0);
  lw_correction_pid.SetSampleTime(10);
  lw_correction_pid.SetMode(AUTOMATIC);
}

void loop() {
  calcAndRun();
  checkTurn();
 decideWallToFollow();

  // Print debug info every 1000 loops
  printDistance();

  checkEncodersZeroVelocity();
  updateDistanceSensors();
}

void calcAndRun() {
  // How fast are we going? in mm/sec and rad/sec
  velocity_linear = getLinearVelocity();
  velocity_angular = getAngularVelocity();

  velocity_linear_pid.Compute();
  velocity_linear_power = 0.2;
  velocity_angular_pid.Compute();
  if (useRW == 0) {
  lw_correction_pid.Compute();
  }
  else {
  rw_correction_pid.Compute();
  }
  readDistance();
  if (debugMode == false) {
  applyPowerLeft(velocity_linear_power - velocity_angular_power);
  applyPowerRight(velocity_linear_power + velocity_angular_power);
  }
}

void readDistance() {
  // ================= Calculate Wall LR Sensor Terms.
  lw_distance = readDistanceLeft();
  rw_distance = readDistanceRight();
  center_distance = readDistanceCenter();
  float left_error = lw_distance - lw_setpoint;
  float right_error = rw_distance - rw_setpoint;
  float lw_correction = k_p_rl * lw_setpoint;
  float rw_correction = k_p_rl * rw_setpoint;
  // =================
}

void checkTurn() {
  if ((center_distance < 12.0 && lw_distance < 16.0) || (lw_distance < 10.0 && rw_distance < 10.0)) {
  turn_ready_count++;
  if (turn_ready_count > 150) {
    if (debugMode == false) {
      if (useRW == 0) {
        turnLeft();
        switchOffRw = 0;
        switchOnRw = 0;
      }
      else {
        turnRight();
        switchOffRw = 0;
        switchOnRw = 0;
      }
    }
    turn_ready_count = 0;
  }
  }
  else {
  turn_ready_count = 0;
  }
}

void decideWallToFollow () { //switches to rw (left wall following) when there is a hole in right wall, and vise versa.
  if (useRW == 0) {
  digitalWrite(RED_LED, 1);
  digitalWrite(GREEN_LED, 0);
  if (lw_distance > switchThresholdFar && rw_distance < switchThresholdNear) {
    switchOnRw++;
    if (switchOnRw > switch_ready_threshold) {
      useRW = 1;
      switchOnRw = 0;
      switchOffRw = 0;
    }
  }
  else {
    switchOnRw = 0;
  }
  }
  else {
  digitalWrite(RED_LED, 0);
  digitalWrite(GREEN_LED, 1);
  if (rw_distance > switchThresholdFar && lw_distance < switchThresholdNear) {
    switchOffRw++;
    if (switchOffRw > switch_ready_threshold) {
      useRW = 0;
      switchOffRw = 0;
      switchOnRw = 0;
    }
  }
  else {
    switchOffRw = 0;
  }
  }
}

void printDistance() {
  if (count % 1000 == 0) {
  //Serial.print(velocity_angular);
  //Serial.print(" ");
  //Serial.println(velocity_linear / 100.0);

  Serial.print("Distance Left: ");
  Serial.println(readDistanceLeft());

  Serial.print("Distance Right: ");
  Serial.println(readDistanceRight());
  Serial.print("Distance Center: ");
  Serial.println(center_distance);

  }
  count++;
}

