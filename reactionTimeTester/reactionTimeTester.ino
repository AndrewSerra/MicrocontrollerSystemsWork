// Andrew Serra
// 02-16-2020
// Description: A reaction time tester that checks if
//              the user can hit the switch button under
//              0.5 seconds, if so it will light up the
//              green led and then turn back to red.

// Pin Values
#define RLED_PIN 7 // PORT D
#define YLED_PIN 6 // PORT D
#define GLED_PIN 5 // PORT D
#define BTN_PIN  0 // PORT B

// Time Constants
#define RANDOM_MAX 5000
#define RANDOM_MIN 3000
#define GREEN_LED_DELAY 3000

// Timer Values
#define TCNT_START_VALUE 34286

enum { RED, YELLOW, GREEN };

boolean isSwPressed, prevSwPressed, isSwJustPressed;
boolean isNewState;
volatile int state = RED;
int prevState = !state;
int randomDelayValue;

void setup() {
  Serial.begin(9600);

  DDRD |= (1 << RLED_PIN) | (1 << YLED_PIN)| (1 << GLED_PIN); // Output LEDs
  DDRB &= (0 << BTN_PIN); // Input Switch Button 

  PORTD &= (0 << RLED_PIN) | (0 << YLED_PIN)| (0 << GLED_PIN);  // Drive LED outputs LOW
  PORTB |= (1 << BTN_PIN);  // Set input pullup

  // Normal Mode - Timer 1
  TCCR1A = 0; 
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0);  // Set on compare match
  TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);  // Prescale 256

  cli();
  TIMSK1 = 0x01; 
  
  randomSeed(analogRead(0));
}

// Interrupt when timer overflows
ISR(TIMER1_OVF_vect) {
  state = RED;
}

void loop() {
  prevSwPressed = isSwPressed;
  isSwPressed = (~PINB) & (1 << BTN_PIN);
  isSwJustPressed = !prevSwPressed && isSwPressed;
  
  isNewState = (state != prevState);
  prevState = state;

  switch(state) {
    case RED:
      // Entry housekeeping
      if(isNewState) {
        Serial.println(F("STATE: RED"));
        randomDelayValue = random(RANDOM_MIN, RANDOM_MAX);
      }

      // State business
      PORTD &= (0 << YLED_PIN) | (0 << GLED_PIN);
      PORTD |= (1 << RLED_PIN);

      delay(randomDelayValue);
      cli();

      // Exit housekeeping
      state = YELLOW;
      break;
    case YELLOW:
      // Entry housekeeping
      if(isNewState) {
        Serial.println(F("STATE: YELLOW"));
        TCNT1 = TCNT_START_VALUE;
        sei();
      }

      // State business
      PORTD &= (0 << RLED_PIN) | (0 << GLED_PIN);
      PORTD |= (1 << YLED_PIN);

      // Exit housekeeping
      if(isSwJustPressed) {
        state = GREEN;
      }
      break;
    case GREEN:
      // Entry housekeeping
      if(isNewState) {
        Serial.println(F("STATE: GREEN"));
      }
      
      // State business
      PORTD &= (0 << RLED_PIN) | (0 << YLED_PIN);
      PORTD |= (1 << GLED_PIN);

      // Wait while green is on
      delay(GREEN_LED_DELAY);
      cli();

      // Exit housekeeping
      state = RED;
      break;
    default:
      state = RED;
      break;
  }
  delay(1);
}
