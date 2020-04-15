#define DIM_GREEN_COLOR      0x00001F
#define DIM_WHITE_COLOR      0x1F1F1F
#define DIM_RED_COLOR        0x1F0000
#define DIM_GREEN_BLUE_COLOR 0x001F1F
#define DIM_RED_BLUE_COLOR   0x1F1F00
#define LED_DATA_PIN 12
#define LED_CLOCK_PIN 11
#define STATE_TIME_THRESHOLD 1500

enum directionState_t{ STOPPED, FORWARD, BACKWARD, LEFT_TURN, RIGHT_TURN };
directionState_t directionState = STOPPED;
directionState_t prevDirectionState = FORWARD;

int turnCount = 0;
int stateTimer = 0;
boolean isNewState = true;
boolean squareCompleted = false;

void configureTimer0RegisterForPWMtoDriveMotor() {
  Serial.print("TCCR0A = ");
  Serial.println(TCCR0A);
  Serial.print("TCCR0B = ");
  Serial.println(TCCR0B);
  
  TCCR0A = 0b10100011;
  TCCR0B = 0b00000011;
  OCR0A = 120;
  OCR0B = 120;
}
//==================================================
// RGB led color is Green = DIM_GREEN_COLOR
void go_forward(int rate) {
  OCR0A = rate;
  OCR0B = rate;

  PORTD |= 0x80;  // In d 1
  PORTB |= 0x02;  // In b 1
  PORTB &= 0xFA;  // In a 0, in c 0

  display_color_on_RGB_led(DIM_GREEN_COLOR);
}
//==================================================
// RGB led color is White = DIM_WHITE_COLOR
void go_backward(int rate) {
  OCR0A = rate;
  OCR0B = rate;

  PORTD &= 0x7F;  // In d 0
  PORTB &= 0xFD;  // In b 0
  PORTB |= 0x05;  // In a 1, in c 1

  display_color_on_RGB_led(DIM_WHITE_COLOR);
}
//==================================================
// RGB led color is Green plus Blue = DIM_GREEN_BLUE_COLOR
void turn_clockwise(int rate) {
  OCR0A = rate;
  OCR0B = rate;
  
  PORTD &= 0x7F;  // In d 0
  PORTB &= 0xF9;  // In a 0, in c 0
  PORTB |= 0x02;  // In b 1

  display_color_on_RGB_led(DIM_GREEN_BLUE_COLOR);
}
//==================================================
// RGB led color is Red plus Blue = DIM_RED_BLUE_COLOR
void turn_counterclockwise(int rate) {
  OCR0A = rate;
  OCR0B = rate;
  
  PORTD |= 0x80;  // In d 1
  PORTB &= 0xF8;  // In a 0, in b 0, in c 0

  display_color_on_RGB_led(DIM_RED_BLUE_COLOR);
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

  unsigned long bitmask=0UL; // UL unsigned long literal (forces compiler to use long data type)
  unsigned long masked_color_result=0UL;

  digitalWrite(LED_CLOCK_PIN,LOW); //start with clock low.

  for(int i=23; i>=0; i--) { // clock out one data bit at a time, starting with the MSB first
    bitmask= (1UL<<i); // build bit mask. Note must use "1UL" unsigned long literal, not "1"

    masked_color_result = color & bitmask; // reveals just one bit of color at time

    boolean data_bit=!(masked_color_result==0); // this is the bit of data to be clocked out.

    digitalWrite(LED_DATA_PIN,data_bit);
    digitalWrite(LED_CLOCK_PIN,HIGH);
    digitalWrite(LED_CLOCK_PIN,LOW);   
  }

  digitalWrite(LED_CLOCK_PIN,HIGH);
  delay(1); // after writing data to LED driver, must hold clock line
  // high for 1 ms to latch color data in led shift register.
}//display_color_on_RGB_led()
//==================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("Testing motor A and B using Timer0 in fast PWM mode 3."));
  Serial.println(F("Requires external 9V battery pack."));

  DDRD |= 0xE0;
  DDRB |= 0x1F;

  configureTimer0RegisterForPWMtoDriveMotor();
}

void loop() {
  unsigned int timestampStartOfMillis = millis(); // mark time start
  isNewState = prevDirectionState != directionState;
  prevDirectionState = directionState;
  sqaureCompleted = turnCount == 4;

  switch(directionState) {
    case STOPPED:
      // Entry housekeeping
      if(isNewState) {
        Serial.println("New state is STOPPED.");
        stop_action(120);
        stateTimer = 0;
      }
      
      // State business
      stateTimer++;
      
      // Exit housekeeping
      if(stateTimer > STATE_TIME_THRESHOLD && !squareCompleted) {
        directionState = FORWARD;
      }
      break;
    case FORWARD:
      // Entry housekeeping
      if(isNewState) {
        Serial.println("New state is FORWARD.");
        go_forward(120);
        stateTimer = 0;
      }

      // State business
      stateTimer++;
      
      // Exit housekeeping
      if(stateTimer > STATE_TIME_THRESHOLD && !squareCompleted) {
        directionState = LEFT_TURN;
      }
      else if (squareCompleted) {
        directionState = STOPPED;
      }
      break;
    case BACKWARD:
      // Entry housekeeping
      if(isNewState) {
        Serial.println("New state is BACKWARD.");
        go_backward(120);
        stateTimer = 0;
      }

      // State business
      stateTimer++;
      
      // Exit housekeeping
      if(stateTimer > STATE_TIME_THRESHOLD) {
        directionState = LEFT_TURN;
      }
      break;
    case LEFT_TURN:
      // Entry housekeeping
      if(isNewState) {
        Serial.println("New state is TURN_LEFT.");
        turn_counterclockwise(100);
        stateTimer = 0;
      }

      // State business
      stateTimer++;

      // Exit housekeeping
      if(stateTimer > (STATE_TIME_THRESHOLD - 1300)) {
        turnCount++;
        directionState = FORWARD;
      }
      break;
    case RIGHT_TURN:
      // Entry housekeeping
      if(isNewState) {
        Serial.println("New state is TURN_RIGHT.");
        turn_clockwise(120);
        stateTimer = 0;
      }

      // State business
      stateTimer++;

      // Exit housekeeping
      if(stateTimer > STATE_TIME_THRESHOLD) {
        directionState = FORWARD;
      }
      break;
    default:
      directionState = STOPPED;
      break;
  }
  delay(10); 
}
