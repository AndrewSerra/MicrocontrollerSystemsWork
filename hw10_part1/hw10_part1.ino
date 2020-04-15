// Andrew Serra
// Description: The program accepts a encryption key and message
//              to encrypt then stores it in the first 128 storage
//              locations of the EEPROM

#define ENCYRPTION_KEY_LEN 16
#define MAX_MESSAGE_LEN 128

char encryption_key[ENCYRPTION_KEY_LEN];
char encrypted_message[MAX_MESSAGE_LEN];
bool input_valid = false;
bool encryption_complete = false;
bool message_shown = false;

void EEPROM_write(unsigned int  address, byte data) {
  while(EECR & 0b00000010) {}

  EEAR = address;
  EEDR = data;
  cli();
  EECR |= 0b00000100;
  EECR |= 0b00000010;
  sei();
}

void setup() {
  Serial.begin(9600);
  delay(200);
  Serial.println("...Message Encryptor...");
}

void loop() {
  if(!encryption_complete) {
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
  
      // Iterate over message characters
      for(int i=0; i < MAX_MESSAGE_LEN; i+=ENCYRPTION_KEY_LEN) {
        // Iterate over the encryption key
        for(int j=0; j < ENCYRPTION_KEY_LEN; j++) {
          encrypted_message[i+j] = (byte)message[i+j] ^ (byte)encryption_key[j];

          encrypted_message[i+j] = (encrypted_message[i+j] == 0x00) ? 0xFF : encrypted_message[i+j];
          EEPROM_write(i+j, encrypted_message[i+j]);
        }
      }
      Serial.println("EEPROM Write completed successfully. Terminating...");
      encryption_complete = true;
    }
  }
}
