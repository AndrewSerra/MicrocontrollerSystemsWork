#include <Wire.h>
#include <MPU6050.h>

#define BRIGHT_GREEN_COLOR   0x0000FF
#define BRIGHT_RED_COLOR     0xFF0000
#define DIM_GREEN_COLOR      0x00001F
#define DIM_WHITE_COLOR      0x1F1F1F
#define DIM_RED_COLOR        0x1F0000
#define DIM_BLUE_COLOR       0x001F00
#define DIM_GREEN_BLUE_COLOR 0x001F1F
#define DIM_RED_BLUE_COLOR   0x1F1F00
#define LED_DATA_PIN 12
#define LED_CLOCK_PIN 11
#define STATE_TIME_THRESHOLD 500
#define TURN_DEGREES_90 90

enum directionState_t {
  STOPPED,
  FORWARD,
  BACKWARD,
  LEFT_TURN,
  RIGHT_TURN,
  CALIBRATE_MOTOR_STICTION
};
directionState_t directionState = CALIBRATE_MOTOR_STICTION;
directionState_t prevDirectionState = FORWARD;

// motor driver pins
const int AB_mtr_INA_PIN = 10;
const int AB_mtr_INB_PIN = 9;
const int CD_mtr_INC_PIN = 8;
const int CD_mtr_IND_PIN = 7;

const int PWM_AB_PIN = 6;
const int PWM_CD_PIN = 5;

int MotorABpwmOffset = 0;
int MotorCDpwmOffset = 0;

int turnCount = 0;
int stateTimer = 0;
boolean isNewState = true;
boolean squareCompleted = false;
int robot_commanded_heading = 0;
MPU6050 mpu; // declare a variable called mpu of datatype MPU6050

unsigned long timeStampStartOfLoopMs = 0;

float timeStepS = 0.01;

float pitch, roll, yaw = 0.0f; // pitch, roll and yaw values

Vector normalizedGyroDPS; //stores the three gyroscope readings XYZ in degrees per second (DPS)

volatile bool newDataFlag = false; // boolean flag to indicate that timer1 overflow has occurred


void configureTimer0RegisterForPWMtoDriveMotor() {

  TCCR0A = 0b10100011;
  TCCR0B = 0b00000011;
  OCR0A = 120;
  OCR0B = 120;
}
//==================================================
// RGB led color is Green = DIM_GREEN_COLOR
void go_forward(int rate, int steering) {
  display_color_on_RGB_led(DIM_GREEN_COLOR);
  digitalWrite(AB_mtr_INA_PIN, LOW);  
  digitalWrite(AB_mtr_INB_PIN, HIGH);  
  digitalWrite(CD_mtr_INC_PIN, LOW);  
  digitalWrite(CD_mtr_IND_PIN, HIGH);  
  
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate - steering, 0, 255));  
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate + steering, 0, 255));
}
//==================================================
// RGB led color is White = DIM_WHITE_COLOR
void go_backward(int rate, int steering) {
  display_color_on_RGB_led(DIM_WHITE_COLOR);
  digitalWrite(AB_mtr_INA_PIN, HIGH);  
  digitalWrite(AB_mtr_INB_PIN, LOW);  
  digitalWrite(CD_mtr_INC_PIN, HIGH);  
  digitalWrite(CD_mtr_IND_PIN, LOW);  
  
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate - steering, 0, 255));  
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate + steering, 0, 255));
}
//==================================================
// RGB led color is Green plus Blue = DIM_GREEN_BLUE_COLOR
void turn_clockwise(int rate) {
  OCR0A = MotorCDpwmOffset + rate;
  OCR0B = MotorCDpwmOffset + rate;

  digitalWrite(AB_mtr_INA_PIN, LOW);  
  digitalWrite(AB_mtr_INB_PIN, HIGH);  
  digitalWrite(CD_mtr_INC_PIN, LOW);  
  digitalWrite(CD_mtr_IND_PIN, LOW); 
//  display_color_on_RGB_led(DIM_GREEN_BLUE_COLOR);
}
//==================================================
// RGB led color is Red plus Blue = DIM_RED_BLUE_COLOR
void turn_counterclockwise(int rate) {
  OCR0A = MotorABpwmOffset + rate;
  OCR0B = MotorABpwmOffset + rate;

  digitalWrite(AB_mtr_INA_PIN, LOW);  
  digitalWrite(AB_mtr_INB_PIN, LOW);  
  digitalWrite(CD_mtr_INC_PIN, LOW);  
  digitalWrite(CD_mtr_IND_PIN, HIGH); 
//  display_color_on_RGB_led(DIM_RED_BLUE_COLOR);
}
//==================================================
// RGB led color is Red = DIM_RED_COLOR
void stop_action(int rate) {
  OCR0A = rate;
  OCR0B = rate;

  PORTD &= 0x7F;  // In d 0
  PORTB &= 0xF8;  // In a 0, in b 0, in c 0

  display_color_on_RGB_led(DIM_RED_COLOR);
}
//==================================================
void display_color_on_RGB_led(unsigned long color) {

  unsigned long bitmask = 0UL; // UL unsigned long literal (forces compiler to use long data type)
  unsigned long masked_color_result = 0UL;

  digitalWrite(LED_CLOCK_PIN, LOW); //start with clock low.

  for (int i = 23; i >= 0; i--) { // clock out one data bit at a time, starting with the MSB first
    bitmask = (1UL << i); // build bit mask. Note must use "1UL" unsigned long literal, not "1"

    masked_color_result = color & bitmask; // reveals just one bit of color at time

    boolean data_bit = !(masked_color_result == 0); // this is the bit of data to be clocked out.

    digitalWrite(LED_DATA_PIN, data_bit);
    digitalWrite(LED_CLOCK_PIN, HIGH);
    digitalWrite(LED_CLOCK_PIN, LOW);
  }

  digitalWrite(LED_CLOCK_PIN, HIGH);
  delay(1); // after writing data to LED driver, must hold clock line
  // high for 1 ms to latch color data in led shift register.
}//display_color_on_RGB_led()
//==================================================
int find_motorstiction_using_gyro(int forward_pin, int backward_pin, int pwm_pin) {

  bool calibrating_motor_stiction = true;
  int pwm_value = 0;

  digitalWrite(forward_pin, LOW);
  digitalWrite(backward_pin, HIGH);

  while (calibrating_motor_stiction) {
    if (abs(mpu.readNormalizeGyro().ZAxis) > 2) calibrating_motor_stiction = false;

    if (pwm_value > 250) { //failed calibration
      Serial.print(F("Calibration failed. Check if battery is connected/switched on."));
      analogWrite(pwm_pin, 0); // send pwm value to motor

      while (1) {
        display_color_on_RGB_led(DIM_RED_COLOR);
        delay(100);
        display_color_on_RGB_led(DIM_BLUE_COLOR);
        delay(100);
      }
      calibrating_motor_stiction = false;
    }

    analogWrite(pwm_pin, pwm_value++);

    delay(20);
  }

  return pwm_value;

}//find_motorstiction_using_gyro()
//==================================================
void robot_goes_forward_at_given_yaw_at_speed(float yaw_heading, int rate) {
  go_forward(rate, 3.0 * (yaw_heading - yaw));
}//robot_goes_forward_at_given_yaw_at_speed()
//==================================================
void robot_goes_backward_at_given_yaw_at_speed(float yaw_heading, int rate) {
  go_backward(rate, 3.0 * (yaw_heading - yaw));
}//robot_goes_backward_at_given_yaw_at_speed()
//==================================================
void robot_turns_to_heading(float yaw_heading) {  
  if (yaw < (yaw_heading - 2)) {    
    turn_counterclockwise(1 + 0.15 * ((yaw_heading + 2) - yaw));  
  }  
  else if (yaw > (yaw_heading + 2)) { 
    turn_clockwise(1 + 0.15 * (yaw - (yaw_heading + 2)));  
  }  
  else {    
    stop_action(0); 
  }
}//robot_turns_to_heading()
//==================================================

void setup() {
  Serial.begin(9600);
  Serial.println(F("Testing motor A and B using Timer0 in fast PWM mode 3."));
  Serial.println(F("Requires external 9V battery pack."));

  DDRD |= 0xE0; // 0b1110 0000
  DDRB |= 0x1F; // 0b0001 1111

  PORTB &= 0b11100111;

  TCCR1A = 0b00000010;
  TCCR1B = 0b00011100;

  ICR1 = 625;

  cli();
  TIMSK1 = 0x01;
  sei();

  // Initialize MPU6050 to have full scale range of 2000 degrees per second

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {

    Serial.println("Could not find a valid MPU6050 sensor, check wiring.");

    delay(1000);

  }

  mpu.calibrateGyro(); // Calibrate gyroscope- must be done with sensor not moving.

  mpu.setThreshold(1);// sets level below which changes in gyro readings are ignored.

  // helps to reduce noise. 1 = one standard deviation. Range is 0 to 3.


  configureTimer0RegisterForPWMtoDriveMotor();
}

ISR(TIMER1_OVF_vect) {
  newDataFlag = true;
}

void loop() {
  unsigned int timestampStartOfMillis = millis(); // mark time start
  isNewState = prevDirectionState != directionState;
  prevDirectionState = directionState;
  squareCompleted = turnCount == 4;     // Check if the four turns are completed

  // Calculate the yaw value
  normalizedGyroDPS = mpu.readNormalizeGyro();
  yaw = yaw + normalizedGyroDPS.ZAxis * timeStepS;

  // Display the yaw value
//  Serial.print(F("Yaw: "));
//  Serial.println(yaw);

  switch (directionState) {
    case STOPPED:
      // Entry housekeeping
      if (isNewState) {
        Serial.println("New state is STOPPED.");
        stop_action(120);
        stateTimer = 0;
        turnCount = 0;
      }

      // State business
      stateTimer++;

      // Exit housekeeping
      if (stateTimer > STATE_TIME_THRESHOLD) {
        directionState = FORWARD;
      }
      break;
    case FORWARD:
      // Entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw;        
        Serial.print("New state is FORWARD, \tcommanded heading is ");         
        Serial.println(robot_commanded_heading);        
        display_color_on_RGB_led(DIM_GREEN_COLOR);
        stateTimer = 0;
      }

      // State business
      robot_goes_forward_at_given_yaw_at_speed(robot_commanded_heading, 70);
      stateTimer++;

      // Exit housekeeping
      if (stateTimer > STATE_TIME_THRESHOLD) {
        directionState = LEFT_TURN;
      }
      break;
    case BACKWARD:
      // Entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw;        
        Serial.print("New state is BACKWARD, \tcommanded heading is ");         
        Serial.println(robot_commanded_heading);        
        display_color_on_RGB_led(DIM_GREEN_COLOR);
        stateTimer = 0;
      }

      // State business
      robot_goes_backward_at_given_yaw_at_speed(robot_commanded_heading, 70);
      stateTimer++;

      // Exit housekeeping
      if (stateTimer > STATE_TIME_THRESHOLD) {
        directionState = LEFT_TURN;
      }
      break;
    case LEFT_TURN:
      // Entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw + 90;        
        Serial.print(F("new state is LEFT_TURN,\tcommanded heading is "));        
        Serial.println(robot_commanded_heading);        
        display_color_on_RGB_led(BRIGHT_RED_COLOR);
      }

      // State business
      stateTimer++;
      robot_turns_to_heading(robot_commanded_heading);

      // Exit housekeeping
      if(yaw > robot_commanded_heading) {
        directionState = FORWARD;
      }
      break;
    case RIGHT_TURN:
      // Entry housekeeping
      if(isNewState) {
        robot_commanded_heading = yaw - 90;        
        Serial.print(F("new state is RIGHT_TURN,\tcommanded heading is "));        
        Serial.println(robot_commanded_heading);        
        display_color_on_RGB_led(BRIGHT_GREEN_COLOR);
      }

      // State business
      stateTimer++;
      robot_turns_to_heading(robot_commanded_heading);

      // Exit housekeeping
      if(yaw < robot_commanded_heading) {
        directionState = FORWARD;
      }
      break;
    case CALIBRATE_MOTOR_STICTION:
      // Entry housekeeping
      if (isNewState) {
        Serial.println("New state is CALIBRATE_MOTOR_STICTION.");

        // Find motor stiction using gyro
        MotorABpwmOffset = find_motorstiction_using_gyro(AB_mtr_INA_PIN, AB_mtr_INB_PIN, PWM_AB_PIN);
        Serial.print("Motor Phase offset is ");
        Serial.println(MotorABpwmOffset);
        display_color_on_RGB_led(DIM_GREEN_COLOR);
        delay(500);

        MotorCDpwmOffset = find_motorstiction_using_gyro(CD_mtr_INC_PIN, CD_mtr_IND_PIN, PWM_CD_PIN);
        Serial.print("Motor Phase offset is ");
        Serial.println(MotorCDpwmOffset);

        display_color_on_RGB_led(DIM_BLUE_COLOR);
        delay(500);
      }

      // State business

      // Exit housekeeping
      directionState = STOPPED;
      break;
    default:
      directionState = STOPPED;
      break;
  }
  delay(10);
}
