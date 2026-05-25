/*
 * Arduino Z-Axis Controller for 2D LiDAR to 3D Point Cloud Reconstruction
 *
 * Purpose
 * -------
 * This firmware controls a stepper-motor-driven lead screw that moves the
 * 2D LiDAR sensor along the Z axis. The host computer sends the desired scan
 * height in millimetres through the serial port. While the scanner moves, the
 * Arduino streams the current Z position to the host. The C++ acquisition
 * program combines this Z position with each 2D LiDAR scan point to generate
 * 3D coordinates: x, y, z.
 *
 * Serial protocol
 * ---------------
 * Input from host:
 *   <scan_height_mm>\n
 * Output to host:
 *   0
 *   1
 *   2
 *   ...
 *   off
 *
 * Hardware pins
 * -------------
 * stepPin   : step pulse input of the stepper driver
 * dirPin    : direction input of the stepper driver
 * buttonPin : push button used to start the scan after the height is received
 */

const int stepPin = 5;
const int dirPin = 2;
const int buttonPin = 7;

const int stepsPerRevolution = 200;      // Full motor steps per revolution.
const float leadScrewPitchMm = 1.25;     // Linear displacement in mm per screw revolution.

const int displayedZIncrementMm = 1;     // Z increment sent to the host for each displayed step.
const int stepsPerDisplayedMm = 160;     // Number of motor steps corresponding to 1 mm in the reconstruction display.

int targetDistanceMm = -1;
unsigned long totalSteps = 0;
unsigned long stepCounter = 0;
int currentZPositionMm = 0;

bool previousButtonState = HIGH;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600);
}

void performStep() {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(1030);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(1030);
}

void loop() {
  bool currentButtonState = digitalRead(buttonPin);

  // Detect a button press on the falling edge.
  if (previousButtonState == HIGH && currentButtonState == LOW) {
    delay(30);  // Debounce delay.

    if (digitalRead(buttonPin) == LOW) {
      // Read the target height sent by the host application.
      if (Serial.available()) {
        int receivedValue = Serial.parseInt();
        if (receivedValue > 0) {
          targetDistanceMm = receivedValue;
          totalSteps = (unsigned long)((float)targetDistanceMm * (float)stepsPerRevolution / leadScrewPitchMm);
        } else {
          Serial.println("Error: invalid distance");
          return;
        }
      } else {
        Serial.println("Error: no distance received");
        return;
      }

      // Start the scan cycle.
      stepCounter = 0;
      currentZPositionMm = 0;
      Serial.println(currentZPositionMm);  // Initial Z position.
      delay(412);

      // Upward motion.
      digitalWrite(dirPin, LOW);
      for (unsigned long i = 0; i < totalSteps; i++) {
        performStep();
        stepCounter++;
        if (stepCounter % stepsPerDisplayedMm == 0) {
          currentZPositionMm += displayedZIncrementMm;
          Serial.println(currentZPositionMm);
        }
      }

      delay(824);

      // Downward return motion.
      digitalWrite(dirPin, HIGH);
      stepCounter = 0;
      for (unsigned long i = 0; i < totalSteps; i++) {
        performStep();
        stepCounter++;
        if (stepCounter % stepsPerDisplayedMm == 0) {
          currentZPositionMm -= displayedZIncrementMm;
          Serial.println(currentZPositionMm);
        }
      }

      delay(412);
      Serial.println("off");  // End-of-scan signal for the host application.
    }
  }

  previousButtonState = currentButtonState;
}
