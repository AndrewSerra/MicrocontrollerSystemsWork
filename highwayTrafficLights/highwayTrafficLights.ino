#define MSEC_SAMPLE 1
 
enum { STOP, HW_WAIT, HW_PASS, HW_SLOW, FR_PASS, FR_SLOW };

int state = STOP, prevState = !state;
int stateTimer = 0;
boolean isNewState; 

void setup() {
  // Set the outputs (lights)
  DDRD |= 0xFC;
  // Set detector c
  DDRB |= 0x01;

  // Drive all lights to LOW
  PORTD &= 0x03;
  // No input pullup for detector c
  PORTB &= 0xFE;
}

void loop() {

  // Update state information
  isNewState = (state != prevState);
  prevState = state;

  if(state == STOP) {
    // Entry housekeeping
    if(isNewState) {
      stateTimer = 0;
      PORTD = 0x90; // Red lights on
    }

    // State business
    stateTimer++;

    // State transition
    // Check if detector is on
    if(stateTimer >= 5000 && (PINB & 0x01)) {
      state = FR_PASS;
    }
    else if(stateTimer >= 5000 && !(PINB & 0x01)){
      state = HW_PASS;
    }
  }
  else if(state == HW_WAIT) {
    // Entry housekeeping
    if(isNewState) {
      stateTimer = 0;
      PORTD = 0x50; // Red lights on
    }

    // State business
    stateTimer++;

    // State transition
    // Check if 40s is complete
    if(stateTimer >= 40000) {
      state = HW_PASS;
    }
  }
  else if(state == HW_PASS) {
    // Entry housekeeping
    if(isNewState) {
      stateTimer = 0;
      PORTD = 0x30; // Red light Farm, Green light Highway
    }

    // State business
    stateTimer++;

    // State transition
    // Check if 40s is not complete and detectorC not high
    if(!(stateTimer <= 40000 && !(PINB & 0x01))) {
      state = HW_SLOW;
    }
  }
  else if(state == HW_SLOW) {
    // Entry housekeeping
    if(isNewState) {
      stateTimer = 0;
      PORTD = 0x40; // highway yellow light on
    }

    // State business
    stateTimer++;

    // State transition
    // Check if 5s is complete
    if(stateTimer >= 5000) {
      state = STOP;
    }
  }
  else if(state == HW_PASS) {
    // Entry housekeeping
    if(isNewState) {
      stateTimer = 0;
      PORTD = 0x30; // Red light Farm, Green light Highway
    }

    // State business
    stateTimer++;

    // State transition
    // Check if 40s is not complete and detectorC not high
    if(stateTimer >= 40000 && (PINB & 0x01)) {
      state = HW_SLOW;
    }
  }
  else if(state == HW_SLOW) {
    // Entry housekeeping
    if(isNewState) {
      stateTimer = 0;
      PORTD = 0x50; // highway yellow light on
    }

    // State business
    stateTimer++;

    // State transition
    // Check if 5s is complete
    if(stateTimer >= 5000) {
      state = STOP;
    }
  }
  else if(state == HW_PASS) {
    // Entry housekeeping
    if(isNewState) {
      stateTimer = 0;
      PORTD = 0x30; // Red light Farm, Green light Highway
    }

    // State business
    stateTimer++;

    // State transition
    // Check if 40s is not complete and detectorC not high
    if(!(stateTimer >= 40000 && !(PINB & 0x01))) {
      state = HW_SLOW;
    }
  }
  else if(state == FR_PASS) {
    // Entry housekeeping
    if(isNewState) {
      stateTimer = 0;
      PORTD = 0x84; // farm road green light on, highway red light
    }

    // State business
    stateTimer++;

    // State transition
    // Check if 40s is complete
    if(stateTimer > 40000) {
      state = FR_SLOW;
    }
//    else if(!(PINB & 0x01)) {
//      state = FR_SLOW;
//    }
  }
  else if(state == FR_SLOW) {
    // Entry housekeeping
    if(isNewState) {
      stateTimer = 0;
      PORTD = 0x88; // farm road yellow light on, highway red light
    }

    // State business
    stateTimer++;

    // State transition
    // Check if 5s is complete
    if(stateTimer >= 5000) {
      state = STOP;
    }
  }
  else {
    state = STOP;
  }
  
  delay(MSEC_SAMPLE);
}
