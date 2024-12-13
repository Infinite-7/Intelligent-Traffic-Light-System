// Pin definitions for LEDs
#define RED_LED_PIN 13   // Red LED
#define YELLOW_LED_PIN 12 // Yellow LED
#define GREEN_LED_PIN 11  // Green LED

// Timer constants (light durations in milliseconds)
#define BASE_GREEN_TIME 5000    // Green light: 5 seconds
#define YELLOW_TIME 3000    // Yellow light: 3 seconds
#define RED_TIME 5000      // Red light: 5 seconds

// Pedestrian button
#define PED_BUTTON_PIN A0   // Pedestrian crossing button pin
#define LDR_PIN A2        // LDR connected to analog pin A2
#define POT_PIN A1        // Potentiometer connected to analog pin A1


// Timer variables
volatile uint8_t lightState = 0;       // 0 = Red, 1 = Green, 2 = Yellow
volatile bool pedestrianRequest = false; // Flag for pedestrian crossing request
volatile bool sequenceRunning = false; // Flag for sequence running
volatile bool waitForSlaves = false;   // Flag to wait for Slave Boards to complete
uint16_t greenTime = BASE_GREEN_TIME;  // Green light duration (adjustable)

void setup() {
  // Configure LED pins as outputs using DDRx
  DDRB |= (1 << DDB5); // Set pin 13 as output (Red LED)
  DDRB |= (1 << DDB4); // Set pin 12 as output (Yellow LED)
  DDRB |= (1 << DDB3); // Set pin 11 as output (Green LED)

  // Configure pedestrian button pin as input
  DDRC &= ~(1 << DDC0); // Set A0 as input
  PORTC |= (1 << PORTC0); // Enable internal pull-up resistor on A0

  // Initialize Timer1 for 1-second intervals
  TCCR1A = 0;                          // Normal port operation, CTC mode
  TCCR1B = (1 << WGM12) | (1 << CS12); // CTC mode, prescaler = 256
  OCR1A = 62500;                       // Compare value for 1-second interval (16MHz clock)
  TIMSK1 = (1 << OCIE1A);              // Enable Timer1 compare interrupt

  // Initialize Serial Communication
  Serial.begin(9600);

  // Start with the Red light
  lightState = 0; // Red state
  PORTB |= (1 << PORTB5); // Turn on Red LED
}

void loop() {
  if (!sequenceRunning && !waitForSlaves) {
    // Start the traffic light sequence
    sequenceRunning = true;
    checkTrafficDensity(); // Adjust green time based on traffic density
  }

  // Wait for "DONE" signal from Slave 3
  if (waitForSlaves) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim(); // Remove extra spaces and newlines
      //Serial.println("Received: " + command); // Debug
      if (command == "DONE") {
        waitForSlaves = false; // All slaves are done
        sequenceRunning = true; // Restart sequence
      }
    }
  }

  // Debounce the pedestrian button
  static uint8_t lastButtonState = 0;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50; // 50ms debounce time

  uint8_t currentButtonState = PINC & (1 << PINC0); // Read button state from A0
  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentButtonState == 0) { // Button pressed (active LOW)
      pedestrianRequest = true;   // Set pedestrian request flag
    }
  }
  lastButtonState = currentButtonState;

}

// Timer1 interrupt service routine
ISR(TIMER1_COMPA_vect) {
  static uint16_t elapsedTime = 0; // Track elapsed time in milliseconds
  elapsedTime += 1000;             // Increment elapsed time by 1 second

  // Handle the current light state
  if (sequenceRunning) {
    switch (lightState) {
      case 0: // Red light
        if (elapsedTime >= RED_TIME) {
          elapsedTime = 0;
          lightState = 1; // Move to Green state
          PORTB &= ~(1 << PORTB5); // Turn off Red LED
          PORTB |= (1 << PORTB3);  // Turn on Green LED
        }
        break;

      case 1: // Green light
        if (elapsedTime >= greenTime) {
          elapsedTime = 0;
          lightState = 2; // Move to Yellow state
          PORTB &= ~(1 << PORTB3); // Turn off Green LED
          PORTB |= (1 << PORTB4);  // Turn on Yellow LED
        }
        break;

      case 2: // Yellow light
        if (elapsedTime >= YELLOW_TIME) {
          elapsedTime = 0;
          lightState = 0; // Return to Red state
          PORTB &= ~(1 << PORTB4); // Turn off Yellow LED
          PORTB |= (1 << PORTB5);  // Turn on Red LED

          if (pedestrianRequest == true) {
            // Handle pedestrian crossing
            pedestrianRequest = false; // Reset the request flag
            handlePedestrianCrossing();
          }

          // Sequence complete: Notify Slave 1 and wait for all slaves
          sequenceRunning = false;
          greenTime = BASE_GREEN_TIME; // Reset green time
          waitForSlaves = true;
          Serial.println("NEXT");
          delay(100); // Short delay to stabilize communication
        }
        break;
    }
  }
}

void handlePedestrianCrossing() {
  // Blink Red LED for pedestrian crossing (optional)
  for (int i = 0; i < 3; i++) {
    PORTB |= (1 << PORTB5); // Turn on Red LED
    _delay_ms(500);
    PORTB &= ~(1 << PORTB5); // Turn off Red LED
    _delay_ms(500);
  }
  PORTB |= (1 << PORTB5); // Turn on Red LED
}

// Function to read analog value from a specific pin
uint16_t readAnalog(uint8_t pin) {
  ADMUX = (ADMUX & 0xF0) | (pin & 0x0F); // Select ADC channel
  ADCSRA |= (1 << ADSC);                 // Start conversion
  while (ADCSRA & (1 << ADSC));          // Wait for conversion to complete
  return ADC;                            // Return ADC value
}

// Function to check traffic density and adjust green time
void checkTrafficDensity() {
  uint16_t ldrValue = readAnalog(LDR_PIN); // Read LDR value
  uint16_t potValue = readAnalog(POT_PIN); // Read Potentiometer value

  Serial.print("LDR Value: ");
  Serial.println(ldrValue);
  Serial.print("Potentiometer Threshold: ");
  Serial.println(potValue);

  // If LDR value exceeds the potentiometer threshold, increase green time
  if (ldrValue > potValue) {
    greenTime = BASE_GREEN_TIME + 5000; // Extend green time by 5 seconds
    Serial.println("Traffic detected! Extending Green Time.");
  } else {
    Serial.println("No traffic detected. Using default Green Time.");
  }
}

