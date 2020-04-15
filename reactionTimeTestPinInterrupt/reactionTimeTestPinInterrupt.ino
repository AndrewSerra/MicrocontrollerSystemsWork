// Andrew Serra
// 02-24-2020
// Description: A reaction time tester that checks if
//              the user can hit the switch button under
//              0.5 seconds, if so it will light up the
//              green led and then turn back to red.

// Pins
// RED LED PIN      -> PORT D6
// YELLOW LED PIN   -> PORT D5
// GREEN LED PIN    -> PORT D4
// BUTTON PIN       -> PORT B3
// START BUTTON PIN -> PORT D7 -> PCINT23

// Time Constants
#define RANDOM_MAX_VAL 5000
#define RANDOM_MIN_VAL 3000
#define GREEN_LED_DELAY 3000

// Timer Values
#define TCNT_START_VALUE 34286

enum { RED, YELLOW, GREEN };

boolean isSwPressed, prevSwPressed, isSwJustPressed;
boolean isNewState;
volatile boolean start = false;
volatile int state = RED;
int prevState = !state;
int randomDelayValue;

void setup() {
  Serial.begin(9600);

  // Setup data direction registers
  DDRD |= 0x70; // LEDs output
  DDRD &= 0x7B; // Butttons
//  DDRB &= 0xF7; // Reaction button pin 2 input

  PORTD &= 0x8F; // Drive LEDs LOW
  PORTD |= 0x84; // Input pullups
//  PORTB |= 0x08; // Input pullup pin 11

  // Normal Mode - Timer 1
  TCCR1A = 0; 
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0);  // Set on compare match
  TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);  // Prescale 256

  // Pin Signal change detection
  PCICR = 0x04;

  // External Interrupt setup
  EICRA = 0x02;

  // Set the interrupt mask registers
  cli();
  TIMSK1 = 0x01;
  EIMSK = 0x01;
  PCMSK2 |= 0x80;
  sei();
  
  randomSeed(analogRead(0));
}
// External interrupt
ISR(INT0_vect) {
  state = start ? GREEN : state;
}
// Interrupt when external interrupt
ISR(PCINT2_vect) {
  if(~PIND & 0x80) {
    start = true;
  }
}
// Interrupt when timer overflows
ISR(TIMER1_OVF_vect) {
  state = start ? RED : state;
}

void loop() {
//  prevSwPressed = isSwPressed;
//  isSwPressed = (~PINB) & (1 << BTN_PIN);
//  isSwJustPressed = !prevSwPressed && isSwPressed;
  
  isNewState = (state != prevState);
  prevState = state;

  switch(state) {
    case RED:
      // Entry housekeeping
      if(isNewState) {
        start = false;
        Serial.println(F("STATE: RED"));
        randomDelayValue = random(RANDOM_MIN_VAL, RANDOM_MAX_VAL);
      }

      // State business
      PORTD |= 0x40; // Turn on RED LED
      PORTD &= 0xCF; // Turn off YELLOW and GREEN LED

      delay(randomDelayValue);
    
      // Exit housekeeping
      if(start) {
        state = YELLOW;
      }
      break;
    case YELLOW:
      // Entry housekeeping
      if(isNewState) {
        Serial.println(F("STATE: YELLOW"));
        TCNT1 = TCNT_START_VALUE;
      }

      // State business
      PORTD |= 0x20; // Turn on YELLOW LED
      PORTD &= 0xAF; // Turn off RED and GREEN LED
      
      // Exit housekeeping
//      if(isSwJustPressed) {
//        state = GREEN;
//      }
      break;
    case GREEN:
      // Entry housekeeping
      if(isNewState) {
        Serial.println(F("STATE: GREEN"));
      }
      
      // State business
      PORTD |= 0x10; // Turn on GREEN LED
      PORTD &= 0x9F; // Turn off RED and YELLOW LED

      // Wait while green is on
      delay(GREEN_LED_DELAY);

      // Exit housekeeping
      state = RED;
      break;
    default:
      state = RED;
      break;
  }
  delay(1);
}
