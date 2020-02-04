// Date: 02-03-2020
// Andrew Serra
// Description: A 1 second LED blinking program
//              created using timer registers
//              and interrupts.
void setup() {
  // PIN 2 for the LED
  // Assign pin
  DDRD |= 0x04;

  // Make PIN 2 a OUTPUT
  // Output it with a LOW signal
  PORTD &= 0xFB;

  cli();
  //
  // Initialize Timer 1 to CTC Mode
  //
  TCNT1 = 0;
  TCCR1A = 0x00;
  TCCR1B = 0x00;

  // Clears OC1X on compare match
  TCCR1A |= 0x80; // Set COM1A1
  TCCR1A &= 0xBF; // Clear COM1A0
  
  // Counts up, updates OCR1A immediately and sets TOV1 flag at MAX
  TCCR1A &= 0xFC; // Clear WGM11 and WGM10
  TCCR1B |= 0x08; // Set WGM12
  
  // Setup prescaler clkI/O 256
  TCCR1B |= 0x04; // Set CS12
  TCCR1B &= 0xFC; // Clear CS11 and CS10

  // Set compare register
  OCR1A = 31999;

  // Enable interrupts
  TIMSK1 |= 0x02;
  
  // Turn on global interrupts
  sei();

  Serial.begin(9600);
}
ISR(TIMER1_COMPA_vect) {
  if(PORTD & 0x04) {
    PORTD &= 0xFB;
  }
  else {
    PORTD |= 0x04;
  }
}
void loop() {
}
