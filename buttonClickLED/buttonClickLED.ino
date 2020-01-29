// Andrew Serra
// Date: 01-27-2020
// Homework 2
// Control a circuit with four LEDs to digital pins 4-7
// Switches connected to digital pins 0-3
// Description: Reads inputs from switches and 
//              turns on the corresponding LED
//              if switch is pressed.

void setup() {
  // Create outputs on pins 4 to 7
  // and inputs on pins 0 to 3
  DDRD = 0xF0;
  // Output low for pins 4 to 7
  // and create pullup for pins 0 to 3
  PORTD = 0x0F;
}

void loop() {
  // Invert PIND because switch inputs are ACTIVE LOW
  // Then shift right four positions to get the inverted
  // inputs to where the outputs are.
  // Set the output values in PORTD to drive outputs with 
  // respective values
  PORTD |= ~(PIND << 4);

  // If the value of PIND is greater than 0x0F (any output can be high)
  // clear the values of outputs
  if(PIND > 0x0F) {
    PORTD &= 0x0F;
  }
  delay(1);
}
