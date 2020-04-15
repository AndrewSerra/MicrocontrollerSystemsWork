// Andrew Serra
// Description: The program accepts a encryption key, message
//              to encrypt, and a peripheral address for I2C comms
//              then sends the encrypted characters to the peripheral

#define ENCYRPTION_KEY_LEN 16
#define MAX_MESSAGE_LEN 128

char encryption_key[ENCYRPTION_KEY_LEN];
char encrypted_message[MAX_MESSAGE_LEN];
bool input_valid = false;
bool encryption_complete = false;
bool message_shown = false;
bool process_complete = false;
unsigned int peripheral_addr;

void initI2C(unsigned long bit_rate) {
  // Set TWBR for desired bit rate
  TWBR = ((16000000 / bit_rate) - 16) / 2;
  // Enable TWI
  TWCR |= 0b00000100;
  // Initialized PC4 and PC5 as inputs
  DDRC &= 0b11001111;
  // Enable pullups PC4 and PC5
  PORTC |= 0b00110000;
}

void i2cWaitForComplete(void) {
  while(!(TWCR & 0x080)) {}
}

void i2cStart(void) {
  // Clear the interrupt, Initiate start sequence, enable TWI
  TWCR = 0b10100100;
  // Wait to know sstart complete
  i2cWaitForComplete(); 
}

void i2cStop(void) {
  // Clear the interrupt, initiate stop sequence, enable TWI
  TWCR = 0b10010100;
}

void i2cSend(byte data) {
  // Set Data to register
  TWDR = data;
  // Clear the interrupt and enable
  TWCR = 0b10000100;
  // Wait to know data is sent
  i2cWaitForComplete();
}

byte i2cReadAck(void) {
  // Clear the interrupt, allow ACK, enable TWI
  TWCR = 0b11000100;
  // Wait to know data is received
  i2cWaitForComplete();
  // Return received data
  return (TWDR);
}

void setup() {
  Serial.begin(9600);
  delay(200);
  initI2C(100000)  // Set to 100kHz
}

void loop() {
  if(!message_shown) { 
    Serial.print(F("Enter peripheral device address: ")); 
    message_shown = true;
  }

  while(Serial.available()) {
    unsigned int peripheral_addr = Serial.parseInt();
    
    if(peripheral_addr > 127) {
      input_valid = false;
      message_shown = false;
      Serial.println(peripheral_addr);
      Serial.println("Invalid address. Maximum location is 1024.");
    }
    else {
      input_valid = true;
    }
  }

  if(!encryption_complete) {
    input_valid = false;
    message_shown = false;
    
    if(!message_shown) { 
      Serial.print(F("Enter an encryption key: ")); 
      message_shown = true;
    }
    // Request the encryption key
    while(Serial.available() && !input_valid) {
      char input[ENCYRPTION_KEY_LEN];
      Serial.readStringUntil('\n').toCharArray(input, ENCYRPTION_KEY_LEN+1);
     
      if(strlen(input) != ENCYRPTION_KEY_LEN) {
        Serial.println(F("Encryprtion key is not 16 characters long. Terminating..."));
      }
      else {
        input_valid = true;
        Serial.println(input);
        memcpy(encryption_key, input, sizeof(input[0])*ENCYRPTION_KEY_LEN);
        message_shown = false;
      }
    }

    if(!message_shown && input_valid) { 
      Serial.print(F("Enter a message to encrypt: "));
      message_shown = true;
    }
    
    // Request the message
    if(Serial.available() && input_valid && !encryption_complete) {
      char message[MAX_MESSAGE_LEN];
      Serial.readStringUntil('\n').toCharArray(message, MAX_MESSAGE_LEN);
      Serial.println(message);

      // i2c Ops init
      i2cStart();
      i2cSend(peripheral_addr); // Send peripheral address
      i2cSend(0);               // Write Operation
      if(i2cReadAck()) {
        i2cSend(WHOAMI);
      }
      // Iterate over message characters, send to peripheral device
      for(int i=0; i < MAX_MESSAGE_LEN; i+=ENCYRPTION_KEY_LEN) {
        // Iterate over the encryption key
        for(int j=0; j < ENCYRPTION_KEY_LEN; j++) {
          encrypted_message[i+j] = (byte)message[i+j] ^ (byte)encryption_key[j];
          encrypted_message[i+j] = (encrypted_message[i+j] == 0x00) ? 0xFF : encrypted_message[i+j];

          if(i2cReadAck()) {
            i2cSend(encrypted_message[i+j]);
          }
        }
      }
      i2cStop();
      encryption_complete = true;
    }
  }
}
