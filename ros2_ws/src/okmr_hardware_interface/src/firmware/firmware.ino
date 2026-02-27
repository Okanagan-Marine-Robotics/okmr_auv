#include <ESP32Servo.h>
const int minThrottle = 1100; // Minimum throttle in microseconds (1ms)
const int maxThrottle = 1900; // Maximum throttle in microseconds (2ms)
float throttle[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

#define NUM_SERVOS 1
#define NUM_ACTUATORS 2

int pins[] = {12, 13, 14, 25, 26, 27, 32, 2}; 
int servo_pins[] = {4};
int servo_inits[] = {1500};
int actuator_pins[] = {5, 21};
//note: add servo and actuator commands
//100<1500\n for servo command on servo_pins[0] (pwm same as motors)
//200<1\n for actuator command on actuaotr_pins[0] (digital high or low)
//add command interpretation to recvWithEndMarker

int killSwitchPin = 15;
int ledPin1 = 19;
int ledPin2 = 22;
int ledPin3 = 23;
int leakSensorPin = 4;
bool killSwitchEnabled = false;
unsigned long lastKillswitchPrintTime = 0;
const unsigned long KILLSWITCH_PRINT_INTERVAL = 100; // Print killswitch status every 100ms
int lastKillswitchState = -1; // Track previous state for change detection

Servo motors[8];
Servo servos[NUM_SERVOS];

void killSwitch() {
  int switchState = digitalRead(killSwitchPin);
  killSwitchEnabled = (switchState == HIGH);

  if (killSwitchEnabled) {
    // Set all throttle values to neutral when killswitch is enabled
    for (int i = 0; i < 8; i++) {
      throttle[i] = 1500;
    }
    digitalWrite(ledPin1, LOW);  // Turn off LED when killswitch is enabled
  } else {
    digitalWrite(ledPin1, HIGH); // Turn on LED when killswitch is not enabled
  }

  // Print immediately if state changed, otherwise check time throttling
  bool stateChanged = (switchState != lastKillswitchState);
  bool timeToSend = (millis() - lastKillswitchPrintTime >= KILLSWITCH_PRINT_INTERVAL);

  if (stateChanged || timeToSend) {
    Serial.print("killswitch<");
    Serial.print(switchState);
    Serial.println();
    lastKillswitchPrintTime = millis();
    lastKillswitchState = switchState;
  }
}

void escArm() {
  delay(500);
  for (int i = 0; i < 8; i++) {
    motors[i].writeMicroseconds(1500);
  }
  delay(2000);
  for (int i = 0; i < 8; i++) {
    motors[i].writeMicroseconds(1500);
  }
}

void setup() {
  // Initialize PWM for the ESC
  for (int i = 0; i < 8; i++) {
    motors[i] = Servo();
    motors[i].attach(pins[i]);
  }

  Serial.begin(115200);
  Serial.println("Arming ESC...");
  pinMode(killSwitchPin, INPUT_PULLUP);
  //pinMode(leakSensorPin, INPUT);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  escArm();
  Serial.println("ESCs Armed.");
  initServos();

  // Initialize actuator pins
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    pinMode(actuator_pins[i], OUTPUT);
    digitalWrite(actuator_pins[i], LOW);
  }

  while(Serial.available() > 0){
    Serial.read();
  }

}

void initServos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servo_pins[i]);
    servos[i].writeMicroseconds(servo_inits[i]);
  }
  Serial.println("Servos initialized.");
}
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;

    // Process multiple characters per call to prevent buffer buildup
    while (Serial.available() > 0 && ndx < numChars - 1) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
            break; // Process one complete message per call
        }
    }

    // Clear buffer if overflow detected
    if (ndx >= numChars - 1) {
        ndx = 0;
        // Clear remaining data in serial buffer to prevent corruption
        while (Serial.available() > 0) {
            Serial.read();
        }
    }
}

unsigned long lastThrottleTime = 0;
const unsigned long THROTTLE_TIMEOUT = 2000;  // 2 second timeout
int currMotor = 0;

void parseNewData(){
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(receivedChars,"<");      // get the first part - the string
    int command = atoi(strtokIndx);

    strtokIndx = strtok(NULL, "<");
    float value = atof(strtokIndx);     // convert this part to a float

    if (command >= 100 && command < 200) {
      // Servo command: 100 + servo_index
      int servoIndex = command - 100;
      if (servoIndex >= 0 && servoIndex < NUM_SERVOS) {
        int servoValue = constrain(value, minThrottle, maxThrottle);
        servos[servoIndex].writeMicroseconds(servoValue);
      }
    }
    else if (command >= 200 && command < 300) {
      // Actuator command: 200 + actuator_index
      int actuatorIndex = command - 200;
      if (actuatorIndex >= 0 && actuatorIndex < NUM_ACTUATORS) {
        digitalWrite(actuator_pins[actuatorIndex], value > 0 ? HIGH : LOW);
      }
    }
    else {
      // Motor command: 0-7
      currMotor = command;
      if (currMotor >= 0 && currMotor < 8) {
        // Clamp throttle values to safe range
        throttle[currMotor] = constrain(value, minThrottle, maxThrottle);
        lastThrottleTime = millis();  // Update last received time
      }
    }
    newData = false;
}

void loop() {
  // Check for serial input
  /*
  int leakReading = analogRead(leakSensorPin);
  Serial.print("leak<");
  Serial.print(leakReading);
  Serial.println();
  */
  recvWithEndMarker();
  if(newData)parseNewData();

  // Check for throttle timeout
  if (millis() - lastThrottleTime > THROTTLE_TIMEOUT) {
    //Serial.println("RESET");
    for (int i = 0; i < 8; i++) {
      throttle[i] = 1500;
    }
  }

  killSwitch();
  
  for (int i = 0; i < 8; i++) {
    motors[i].writeMicroseconds((int)throttle[i]);
  }
}
