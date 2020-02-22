// Andrew Serra
// 02-21-2020
// Description: Change the brightness of the LED 
//              if switch buttons are pressed

// Pin 2 - Switch button to increase
// Pin 3 - Switch button to decrease
// Pin 6 - OCR0A output for LED brightness

volatile int countPercentage = 10;
int prevCountPercentage;

boolean isNewPercentage = false;

// Calculate the count value from the percentage
int calcCountFromPerc(int perc) {
  return int(256 * perc / 100);
}

void setup() {
  // Clear pin 2 and 3 for input switch button
  DDRD &= 0xF3;
  // Set pin 6 for OCR0A output LED pin 6 
  DDRD |= 0x40;

  // Set input pullups for switch buttons
  PORTD |= 0x0C;
  
  // Mode 7 - Prescale 256
  TCCR0A = 0b10100011;
  TCCR0B = 0b00000100;

  // Start at 10% brightness
  OCR0A = calcCountFromPerc(10);
  
  EICRA |= 0x0F; // Rising edge detection

  cli();
  EIMSK = 0x03;  // Set the external interrupt
  sei();
}

void loop() {
  isNewPercentage = prevCountPercentage != countPercentage;
  prevCountPercentage = countPercentage;

  // Increment only if the percentage just changed
  if(isNewPercentage){
    OCR0A = calcCountFromPerc(countPercentage);
  }
}

ISR(INT0_vect) {
  if(countPercentage < 90) {
    countPercentage += 10;
  }
}

ISR(INT1_vect) {
  if(countPercentage > 10) {
    countPercentage -= 10;
  }
}
