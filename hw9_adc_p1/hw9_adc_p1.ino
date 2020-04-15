// Andrew Serra
// PWM signal from ADC value to control
// LED Brightness.
#define MAX_COUNT_PWM 65536
#define MIN_COUNT_PWM 0

void setup() {
  // Set pin 6 for OCR0A output LED pin 6 
  DDRD |= 0x40;
  
  DDRC &= 0xFB; // Pin 2 input
  PORTC &= 0xFB; // No input pullup 

  // Mode 15 - Fast PWM, Prescale 8 due to ADC
  TCCR1A = 0b10000011;
  TCCR1B = 0b00011010;

  OCR1A = 0 // Start with brightness at 0

  ADMUX = 0b01100010; // Use AVcc, left align, ADC2 for mux
  ADCSRA = 0b11000011;
  ADCSRB = 0b00000000; // Free running mode
}

void loop() {
  while(ADCSRA & 0x10) {
    OCR1A = map(ADC, 0, 1023, MIN_COUNT_PWM, MAX_COUNT_PWM);
  }
}
