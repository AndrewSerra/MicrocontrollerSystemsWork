void setup() {
  ADMUX = 0b01100011;
  PORTC &= 0xF7;
  ADCSRB = 0x00;
  ADCSRA = 0b11000000; 
}

void loop() {
   while(ADCSRA & 0x10) {
    Serial.print("ADC value: ");
    Serial.println((ADCH << 8 | ADCL, DEC);
   }
}
