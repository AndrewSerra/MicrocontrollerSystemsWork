#include <IRremote.h>

#define IR_RECV_PIN A3

IRrecv irrecv(IR_RECV_PIN);
decode_results IR_command_received;

bool sequenceCorrect = true;
bool messageShown = false;
String input;
int inputCount = 0;
char sequence[3] = { 'c', '4', 'd' };

void setup() {
  Serial.begin(9600);
  
  TCCR1A = 0b10000000; // Comm A clear, ctc mode4
  TCCR1B = 0b00010100; // prescale 256

  OCR1A = 62500;
  TCNT1 = 0;
  
  cli();
  TIMSK1 = 0x02;
}

void loop() {
  while (sequenceCorrect && inputCount < 3) {
    input = "";
    if (irrecv.decode(&IR_command_received)) {

      if (IR_command_received.value != sequence[inputCount]) {
        sequenceCorrect = false;
      }
    }
  }

  if (!messageShown && !sequenceCorrect) {
    messageShown = true;
    Serial.println("Password is incorrect.");
  }
  else if (!messageShown && sequenceCorrect) {
    messageShown = true;
    Serial.println("Password is correct.");
    sei();
  }
}

ISR(TIMER1_COMPA_vect) {
  Serial.println("Five more weeks until summer!");
}
