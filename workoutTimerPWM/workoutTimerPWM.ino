// Andrew Serra
// Date: 02-15-2020
// Description: Timer stays at RED LED until button
//              is pressed and then the GREEN LED
//              turns on for 60 seconds with PWM
//              signal for dimmed LEDs

// Count values
#define COUNT_THRESHOLD 15
#define COUNT_START_VALUE 0

// Pin positions in registers
#define GREEN_LED_PIN 5 // PD
#define RED_LED_PIN 1   // PB
#define SW_PIN 2        // PD

// Normal mode TCNT1 Start val
#define TCNT_START_VALUE 3036
// Fast PWM Mode led brightness value
#define OCR0B_75_PERC_VAL 150

volatile int count = COUNT_START_VALUE;
boolean isSwPressed, prevSwPressed, isSwJustPressed;

void setup() {
  DDRD &= (0 << SW_PIN);
  DDRD |= (1 << GREEN_LED_PIN);
  DDRB |= (1 << RED_LED_PIN);

  PORTD |= (1 << SW_PIN);
  PORTB |= (1 << RED_LED_PIN);

  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A |= (0 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0);
  TCCR0A |= (1 << WGM01) | (1 << WGM00);
  TCCR0B |= (1 << WGM02);
  TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00); // 1024 prescale

  // For Fast PWM Mode in Timer0
  OCR0A = 200;
  OCR0B = 0;
  
  TCCR1A &= (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0);
  TCCR1A &= (0 << WGM11) | (0 << WGM10);
  TCCR1B &= (0 << WGM13) | (0 << WGM12);
  TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);

  TCNT1 = TCNT_START_VALUE;

  cli();
  
  TIMSK1 = 0x01;

  Serial.begin(9600);
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
  if(isSwJustPressed && (PINB & (1 << RED_LED_PIN))) {
    PORTB &= (0 << RED_LED_PIN);
    OCR0B = OCR0B_75_PERC_VAL;
    sei();
  }
  
  // Reset values and LED signals
  if(count == COUNT_THRESHOLD) {
    count = COUNT_START_VALUE;
    TCNT1 = TCNT_START_VALUE;
    OCR0B = 0;
    PORTB |= (1 << RED_LED_PIN);
  }
}
