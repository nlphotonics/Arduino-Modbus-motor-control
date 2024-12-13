#include <SoftwareSerial.h>

// Define pins for MsAX485
#define MAX485_DE 4
#define MAX485_RE 4
#define RX_PIN 3
#define TX_PIN 2

// Define the number of force sensor used
int activeSensor = 0; // 0 = none, 1 = sensor 1, 2 = sensor 2

// Define pins for force sensors
const int forceSensor1Pin = A2; // Sensor 1
const int forceSensor2Pin = A4; // Sensor 2

// Force sensor thresholds
const int activationThreshold = 50; // Minimum value to detect a press
const int midThreshold = 600;       // Threshold for "Mid" range
const int highThreshold = 800;      // Threshold for "High" range

const int lowspeed = 1000;      //speed value for motor
const int medspeed = 2000;
const int highspeed = 3000;


// Create a SoftwareSerial instance
SoftwareSerial RS485Serial(RX_PIN, TX_PIN);

// Motor state variables
bool motorInitialized = false; // Tracks if the motor has been initialized
uint16_t lastSpeed = 0;        // Tracks the last set speed

// Function prototypes
void sendCommand(byte command[], size_t length);
void setMotorSpeed(uint16_t rpm);
uint16_t calculateCRC16(byte* data, size_t length);

String getForceRange(int forceValue);
void handleForceInput();

void setup() {
  // Set up DE/RE pin for output
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, LOW); // Start in receive mode

  // Start serial communication
  RS485Serial.begin(9600); // Ensure this matches your motor driver's baud rate
  Serial.begin(9600);      // For debugging
}

void loop() {
  // Handle force input and set motor speed accordingly
  handleForceInput();
  delay(100); // Small delay to stabilize readings
}

// Handle force sensor input
void handleForceInput() {
  // Read force sensor values
  int forceValue1 = analogRead(forceSensor1Pin);
  int forceValue2 = analogRead(forceSensor2Pin);

  // Determine active sensor if no sensor is currently active
  if (activeSensor == 0) {
    if (forceValue1 > activationThreshold) {
      activeSensor = 1;
      Serial.println("Sensor 1 Activated");
    } else if (forceValue2 > activationThreshold) {
      activeSensor = 2;
      Serial.println(" | Sensor 2 Activated");
    }
  }

  // Process input based on the active sensor
  uint16_t motorSpeed = 0;
  if (activeSensor == 1) {
    // Serial.print("Sensor 1 Active | Value: ");
    // Serial.print(forceValue1);
    // Serial.print(" | Range: ");
    // Serial.println(getForceRange(forceValue1));

    if (forceValue1 > highThreshold) {
      motorSpeed = highspeed; // High speed
      Serial.println("Motor Speed at High");
    } else if (forceValue1 > midThreshold) {
      motorSpeed = medspeed; // Medium speed
      Serial.println("Motor Speed at Med");
    } else if (forceValue1 > activationThreshold) {
      motorSpeed = lowspeed; // Low speed
      Serial.println("Motor Speed at Low");
    } else {
      Serial.println("Sensor 1 Released");
      activeSensor = 0; // Release active sensor
    }
  } else if (activeSensor == 2) {
    // Serial.print("Sensor 2 Active | Value: ");
    // Serial.print(forceValue2);
    // Serial.print(" | Range: ");
    // Serial.println(getForceRange(forceValue2));

    if (forceValue2 > highThreshold) {
      motorSpeed = highspeed; // High speed
      Serial.println("Motor Speed at High");
    } else if (forceValue2 > midThreshold) {
      motorSpeed = medspeed; // Medium speed
      Serial.println("Motor Speed at Med");
    } else if (forceValue2 > activationThreshold) {
      motorSpeed = lowspeed; // Low speed
      Serial.println("Motor Speed at Low");
    } else {
      Serial.println("Sensor 2 Released");
      activeSensor = 0; // Release active sensor
    }
  }

  // Perform natural stop if no sensor is active
  if (activeSensor == 0) {
    if (motorInitialized) {
      Serial.println("Stopping motor...");
      byte naturalStop[] = {0x01, 0x06, 0x80, 0x00, 0x08, 0x02, 0x26, 0x0B};
      sendCommand(naturalStop, sizeof(naturalStop));
      motorInitialized = false;
      lastSpeed = 0;
    }
    return; // No further processing needed if motor is stopped
  }

  // Check if motor needs to be started
  if (motorSpeed > 0) {
    if (!motorInitialized) {
      // Send the Two Pole Pair Start command only once when starting the motor
      byte twoPolePairStart[] = {0x01, 0x06, 0x80, 0x00, 0x09, 0x02, 0x27, 0x9B};
      Serial.println("Sending 2 Pole Pair Start command...");
      sendCommand(twoPolePairStart, sizeof(twoPolePairStart));
      delay(1000); // Allow time for the motor to initialize
      motorInitialized = true;
    }

    // Set motor speed if it's different from the last speed
    if (motorSpeed != lastSpeed) {
      // Serial.print("Setting motor speed to ");
      // Serial.print(motorSpeed);
      // Serial.println(" RPM");
      setMotorSpeed(motorSpeed);
      lastSpeed = motorSpeed;
      delay(100);
    }
  } 
}

// Function to get force range
String getForceRange(int forceValue) {
  if (forceValue > highThreshold) {
    return "High";
  } else if (forceValue > midThreshold) {
    return "Mid";
  } else {
    return "Low";
  }
}

// SET MOTOR SPEED FUNCTION
void setMotorSpeed(uint16_t rpm) {
  byte speedCommand[8];
  
  // Basic command structure
  speedCommand[0] = 0x01;  // Slave address
  speedCommand[1] = 0x06;  // Function code (write)
  speedCommand[2] = 0x80;  // Register high byte
  speedCommand[3] = 0x05;  // Register low byte
  
  // Convert RPM to hex values
  speedCommand[4] = rpm & 0xFF;           // Low byte of RPM
  speedCommand[5] = (rpm >> 8) & 0xFF;    // High byte of RPM
  
  // Calculate CRC16
  uint16_t crc = calculateCRC16(speedCommand, 6);
  speedCommand[6] = crc & 0xFF;          // CRC low byte
  speedCommand[7] = (crc >> 8) & 0xFF;   // CRC high byte
  
  sendCommand(speedCommand, sizeof(speedCommand));
}

void sendCommand(byte command[], size_t length) {
  // Enable send mode
  digitalWrite(MAX485_DE, HIGH);
  delay(1);  // Small delay to ensure the line is ready

  // Send the command over RS485
  for (size_t i = 0; i < length; i++) {
    RS485Serial.write(command[i]);
  }
  RS485Serial.flush(); // Wait until all data is sent

  // Switch back to receive mode
  digitalWrite(MAX485_DE, LOW);
}

// CRC16 calculation function
uint16_t calculateCRC16(byte* data, size_t length) {
  uint16_t crc = 0xFFFF;
  
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (byte j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  return crc;
}
// END OF SET MOTOR SPEED
