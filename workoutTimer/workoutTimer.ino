// Andrew Serra
// Date: 02-14-2020
// Description: Timer stays at RED LED until button
//              is pressed and then the GREEN LED
//              turns on for 60 seconds

// Count values
#define COUNT_THRESHOLD 15
#define COUNT_START_VALUE 0

// Pin positions in registers
#define GREEN_LED_PIN 2
#define RED_LED_PIN 1
#define SW_PIN 2

// Normal mode TCNT Start val
#define TCNT_START_VALUE 3036

volatile int count = COUNT_START_VALUE;
boolean isSwPressed, prevSwPressed, isSwJustPressed;

void setup() {
  DDRD &= (0 << SW_PIN);
  DDRB |= (1 << RED_LED_PIN) | (1 << GREEN_LED_PIN);
  
  PORTD |= (1 << SW_PIN);
  PORTB &= (0 << GREEN_LED_PIN);
  PORTB |= (1 << RED_LED_PIN);

  TCCR1A &= (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0);
  TCCR1A &= (0 << WGM11) | (0 << WGM10);
  TCCR1B &= (0 << WGM13) | (0 << WGM12);
  TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);

  TCNT1 = TCNT_START_VALUE;

  cli();
  TIMSK1 |= 0x01;
}

ISR(TIMER1_OVF_vect) {
  
  // Increment the count
  count++;

  // Clear interrupts
  if(count == COUNT_THRESHOLD) {
    cli();
  }
}

void loop() {

  prevSwPressed = isSwPressed;
  isSwPressed = ~PIND & (1 << SW_PIN);
  isSwJustPressed = !prevSwPressed & isSwPressed;

  // Set interrupt and LED Signals
  if(isSwJustPressed && (~PINB & (1 << GREEN_LED_PIN))) {
    PORTB &= (0 << RED_LED_PIN);
    PORTB |= (1 << GREEN_LED_PIN);
    sei();
  }
  
  // Reset values and LED signals
  if(count == COUNT_THRESHOLD) {
    count = COUNT_START_VALUE;
    TCNT1 = TCNT_START_VALUE;
    PORTB &= (0 << GREEN_LED_PIN);
    PORTB |= (1 << RED_LED_PIN);
  }
}
