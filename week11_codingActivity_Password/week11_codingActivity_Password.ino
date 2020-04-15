
volatile bool buttonPressed = false;
String Password;
boolean pwdMessagePrinted = false;

void setup() {
  Serial.begin(9600);
  
  EICRA = 0x02; // INT0, Falling Edge Triggering

  cli();
  EIMSK = 0x01; // Enable interrupts for INT0
  sei();

  unclock();
}

void loop() {

  if(!pwdMessagePrinted) {
    Serial.println("Password accepted");
    pwdMessagePrinted = true;
  }

  if (buttonClicked) {
    Serial.println("Please enter a 6 character password:");
    while (Serial.available() == 0) {}  //wait for serial terminal to be ready
    Password = Serial.readString();

    Serial.print("your password is: ");  //just to verify it was read correctly
    Serial.println(Password);

    buttonClicked = false;

    // Location 0 - Store a boolean whether a password is saved
    EEPROM_write(0x000, 0x001);
    // Location 1 to 6 - Save characters of the passwords
    for (int i = 0; i < 5; i++) {
      EEPROM_write(i + 1, Password[i]);
    }
  }
}

ISR(INT0_vect) {
  buttonPressed = true;
}

boolean unlock() {

  bool passwordCorrect = false;

  if (EEPROM_read(0x001)) {
    while (!passwordCorrect) {
      Serial.println("Please enter a 6 character password:");
      while (Serial.available() == 0) {}  //wait for serial terminal to be ready
      passwordEntry = Serial.readString();

      Serial.print("your password is: ");  //just to verify it was read correctly
      Serial.println(passwordEntry);

      for (int i = 0; i < 6; i++) {
        if (EEPROM_read(i + 1) == passwordEntry[i]) {
          passwordCorrect = true;
        }
        else {
          passwordCorrect = false;
          break;
        }
      }
    }
    return true;
  }
}

byte EEPROM_read(unsigned int address) {

  // Wait for EEPE to clear
  while (EECR & 0b00000010) {}

  // Setup address and data registers
  EEAR = address;
  // Start EEPROM read, Set EERE
  EECR |= 0b00000001;

  return EEDR;
}

void EEPROM_write(unsigned int address, byte data) {

  // Wait for EEPE to clear
  while (EECR & 0b00000010) {}

  // Setup address and data registers
  EEAR = address;
  EEDR = data;

  cli();
  // Write high to EEMPe
  EECR |= 0b00000100;
  // Start EEPROM write, Set EEPE
  EECR |= 0b00000010;
  sei();
}
