/* Lab7_I2C.ino */

#include <Wire.h>

#include <MPU6050.h>

#define LED_CLOCK_PIN 11

#define LED_DATA_PIN 12

#define DIMBLUE_COLOR 0x00001F

#define DIMRED_COLOR 0x1F0000

#define DIMGREEN_COLOR 0x001F00


MPU6050 mpu; // declare a variable called mpu of datatype MPU6050

unsigned long timeStampStartOfLoopMs = 0;

float timeStepS = 0.01;

float pitch, roll, yaw = 0.0f; // pitch, roll and yaw values

Vector normalizedGyroDPS; //stores the three gyroscope readings XYZ in degrees per second (DPS)

volatile bool newDataFlag = false; // boolean flag to indicate that timer1 overflow has occurred

unsigned long startMicroseconds, elapsedMicroseconds; // use for profiling time for certain tasks

//==============================================================================
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

void setup() {
  DDRD |= 0xE0;
  DDRB |= 0x1F;
  PORTB &= 0b11100111;

  TCCR0A = 0b10100011;
  TCCR0B = 0b00000011;
  OCR0A = 120;
  OCR0B = 120;
  
  TCCR1A = 0b00000010;
  TCCR1B = 0b00011100;

  ICR1 = 625;

  cli();
  TIMSK1 = 0x01;
  sei();

  Serial.begin(115200);

  // Initialize MPU6050 to have full scale range of 2000 degrees per second

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {

    Serial.println("Could not find a valid MPU6050 sensor, check wiring.");

    delay(1000);

  }

  mpu.calibrateGyro(); // Calibrate gyroscope- must be done with sensor not moving.

  mpu.setThreshold(1);// sets level below which changes in gyro readings are ignored.

  // helps to reduce noise. 1 = one standard deviation. Range is 0 to 3.

} // setup

ISR(TIMER1_OVF_vect) {
  newDataFlag = true;
}
//==============================================================================

void loop() {

  while (!newDataFlag) {}; // stay stuck here until new data arrives, then run loop

  // this will occur every 10 millisecond

  //elapsedMicroseconds=micros()-startMicroseconds; // check time for Timer 1 OVF

  startMicroseconds = micros(); // mark time at start of main loop code

  normalizedGyroDPS = mpu.readNormalizeGyro();

  // elapsedMicroseconds=micros()-startMicroseconds; // check time for I2C comms

  // Calculate Pitch, Roll and Yaw

  pitch = pitch + normalizedGyroDPS.YAxis * timeStepS;

  roll = roll + normalizedGyroDPS.XAxis * timeStepS;

  yaw = yaw + normalizedGyroDPS.ZAxis * timeStepS;

  // elapsedMicroseconds=micros()-startMicroseconds; // check time without prints

  Serial.print(pitch);

  Serial.print(F(" "));

  Serial.print(roll);

  Serial.print(F(" "));

  Serial.println(yaw);

//  elapsedMicroseconds = micros() - startMicroseconds; // check total time main loop

//  Serial.print(F("elapsed time in microseconds = "));
//
//  Serial.println(elapsedMicroseconds);

  display_color_on_RGB_led(DIMBLUE_COLOR); // default is dim blue color

  OCR0A = 0; // set motor speed off

  if (yaw > 5) {

    display_color_on_RGB_led(DIMRED_COLOR);

    OCR0A = 80; // set motor speed slow

    digitalWrite(10, LOW); //motor direction pins

    digitalWrite(9, HIGH); //motor direction pins

  }

  if (yaw < -5) {

    display_color_on_RGB_led(DIMGREEN_COLOR);

    OCR0A = 80; // set motor speed slow

    digitalWrite(10, HIGH); //motor direction pins

    digitalWrite(9, LOW); //motor direction pins

  }

  newDataFlag = false;

} //loop
