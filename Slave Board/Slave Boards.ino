// Pin definitions for LEDs
#define RED_LED_PIN 13   // Red LED
#define YELLOW_LED_PIN 12 // Yellow LED
#define GREEN_LED_PIN 11  // Green LED

// Timer constants (light durations in milliseconds)
#define GREEN_TIME 5000    // Green light: 10 seconds
#define YELLOW_TIME 3000    // Yellow light: 3 seconds
#define RED_TIME 5000      // Red light: 10 seconds

// Timer variables
volatile uint8_t lightState = 0;       // 0 = Red, 1 = Green, 2 = Yellow
volatile bool sequenceRunning = false; // Flag for sequence running

void setup() {
  // Configure LED pins as outputs using DDRx
  DDRB |= (1 << DDB5); // Set pin 13 as output (Red LED)
  DDRB |= (1 << DDB4); // Set pin 12 as output (Yellow LED)
  DDRB |= (1 << DDB3); // Set pin 11 as output (Green LED)

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
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove extra spaces and newlines
    Serial.println("Received: " + command); // Debug
    if (command == "NEXT") {
      sequenceRunning = true; // Start the traffic light sequence
    }
  }
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
        if (elapsedTime >= GREEN_TIME) {
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
          sequenceRunning = false; // Sequence complete

          // Notify the next board (or master if Slave 3)
          if (Serial) {
            Serial.println(lightState == 0 ? "DONE" : "NEXT");
          }
        }
        break;
    }
  }
}
