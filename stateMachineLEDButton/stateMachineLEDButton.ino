// Andrew Serra
// 02-06-2020
// Microcontrollers Practical 1
// Description: State machine that turns on an LED
//              when a switch is pressed and turns 
//              off an LED when switch is released
#define LED_PIN 10
#define SWITCH_PIN 6

enum { systemREADY, yellowLED_ON, yellowLED_OFF };

boolean isSwPressed, prevIsSwPressed, isSwJustReleased, isSwJustPressed;
boolean isNewState;
int state = systemREADY, prevState = !state;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
  prevIsSwPressed = isSwPressed;
  isSwPressed = !digitalRead(SWITCH_PIN);
  isSwJustPressed = (isSwPressed && !prevIsSwPressed);
  isSwJustReleased = (!isSwPressed && prevIsSwPressed);
  
  isNewState = (state != prevState);
  prevState = state;

  switch(state) {
    case systemREADY:
      // Entry housekeeping
      if(isNewState) {
       Serial.println("system: ready");
      }

      // State business
       digitalWrite(LED_PIN, LOW); // Turn LED off

       // exit condition and housekeeping
       if(isSwPressed) {
        state = yellowLED_ON;
       }
      break;
    case yellowLED_ON:
      // Entry housekeeping
      if(isNewState) {
       Serial.println("led_state: yellowLED_ON");
      }

      // State business
       digitalWrite(LED_PIN, HIGH); // Turn LED off

       // exit condition and housekeeping
       if(isSwJustReleased) {
        state = yellowLED_OFF;
       }
      break;
    case yellowLED_OFF:
      // Entry housekeeping
      if(isNewState) {
       Serial.println("led_state: yellowLED_OFF");
      }

      // State business
       digitalWrite(LED_PIN, LOW); // Turn LED off

       // exit condition and housekeeping
       if(isSwJustPressed) {
        state = yellowLED_ON;
       }
       else {
        state = yellowLED_OFF;
       }
      break;
    default:
      state = systemREADY;
  }
  
}
