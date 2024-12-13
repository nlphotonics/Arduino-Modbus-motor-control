/* 
This sketch uses Arduino to control a brushless DC motor via a MAX485 interface. 
The communication protocol is Modbus. The physical implementation is RS485.

Reference:
RS485 and Modbus Protocol: https://www.modbustools.com/modbus.html

HEX to decimal: https://www.rapidtables.com/convert/number/hex-to-decimal.html
CRC calculator (HEX-to-HEX, CRC-16/MODBUS): https://www.texttool.com/crc-online

Motor: https://www.omc-stepperonline.com/24v-172w-35rpm-geared-brushless-dc-motor-100-1-high-precision-gearbox-57blr70-24-02-hg100
Motor driver: https://www.omc-stepperonline.com/digital-brushless-dc-motor-driver-12v-48vdc-max-15-0a-400w-bld-510b
Arduino: https://docs.arduino.cc/hardware/uno-r4-minima/
MAX485 interface: https://www.reichelt.com/ch/en/shop/product/developer_boards_-_ttl_to_rs485_max485-282703
*/

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



// Function prototypes
void sendCommand(byte command[], size_t length);
void setMotorSpeed(uint16_t rpm);
uint16_t calculateCRC16(byte* data, size_t length);

String getForceRange(int forceValue);
void handleForceInput();

/* read the true speed value (in RPM), NOTE: 100:1 geared motor
   return the actual motor speed in RPM (0~3500 in principle)
   return 65535 when error happens     */
unsigned int readMotorSpeed(); 
unsigned int readMotorDirection(); // return FR=0 for CW, return FR=1 for CCW, return 65535 for error
void enableCW(); // EN=1, FR=0, enable rotation of the motor in the CW direction 
void enableCCW(); // EN=1, FR=1, enable rotation of the motor in the CCW direction
void naturalStop(); // EN=0 stop the motor naturally, will first call readMotorDirection()

void setup() {
  // Set up DE/RE pin for output
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, LOW); // Start in receive mode

  // Start serial communication
  RS485Serial.begin(9600); // Ensure this matches your motor driver's baud rate
  Serial.begin(9600);      // For debugging

  // Initialization
  while(1){
    delay(2000);
    Serial.println("Initializing the motor...");
    digitalWrite(MAX485_DE, HIGH); // Disable receive mode = Enable send mode
    while (RS485Serial.available() > 0) {
      RS485Serial.read(); // clear all previously received data
    }
    Serial.println("Sending write speed 0 command...");
    setMotorSpeed(0);
    // Check if data is available in the RS485 buffer
    int availableBytes = RS485Serial.available();
    if (availableBytes > 0) {
      byte response[availableBytes]; // Create a buffer to store the response
      int index = 0;
      while (RS485Serial.available() > 0) {
        response[index] = RS485Serial.read();
        index++;
      }
      // 01 06 80 05 00 00 B0 0B 
      if (response[0]==0x01&&response[1]==0x06&&response[2]==0x80&&response[3]==0x05&&response[4]==0x00&&response[5]==0x00&&response[6]==0xB0&&response[7]==0x0B){
        Serial.println("The motor speed is set to 0!");
        Serial.println("Finish initialization!");
        break;
      } else {
        Serial.println("Wrong response from the motor!");
      }
    }else {
      Serial.println("No response from the motor!");
    }
  }
  
}

void loop() {
  // Handle force input and set motor speed accordingly
  handleForceInput();
  delay(20); // Small delay to stabilize readings
}

// Handle force sensor input
void handleForceInput() {
  // Read force sensor values
  int forceValue1 = analogRead(forceSensor1Pin);
  int forceValue2 = analogRead(forceSensor2Pin);
  if (activeSensor==0){
    if (forceValue1 > activationThreshold) {
      naturalStop();
      setMotorSpeed(0);
      while(readMotorSpeed()!=0){
        delay(50);
      }
      enableCW();
      activeSensor = 1;
      Serial.println("Sensor 1 Activated");
    } else if (forceValue2 > activationThreshold) {
      naturalStop();
      setMotorSpeed(0);
      while(readMotorSpeed()!=0){
        delay(50);
      }
      enableCCW();
      activeSensor = 2;
      Serial.println("Sensor 2 Activated");
    } else {
      // activeSensor = 0;
      naturalStop();
      setMotorSpeed(0);
    }
    
    
  } else if (activeSensor==1){
    if (forceValue1 > highThreshold) {
      setMotorSpeed(highspeed); // High speed
      Serial.println("Motor Speed at High");
    } else if (forceValue1 > midThreshold) {
      setMotorSpeed(medspeed); // Medium speed
      Serial.println("Motor Speed at Med");
    } else if (forceValue1 > activationThreshold) {
      setMotorSpeed(lowspeed); // Low speed
      Serial.println("Motor Speed at Low");
    } else {
      Serial.println("Sensor 1 Released");
      activeSensor = 0; // Release active sensor
    }
    
  } else { //activeSensor==2
    if (forceValue2 > highThreshold) {
      setMotorSpeed(highspeed); // High speed
      Serial.println("Motor Speed at High");
    } else if (forceValue2 > midThreshold) {
      setMotorSpeed(medspeed); // Medium speed
      Serial.println("Motor Speed at Med");
    } else if (forceValue2 > activationThreshold) {
      setMotorSpeed(lowspeed); // Low speed
      Serial.println("Motor Speed at Low");
    } else {
      Serial.println("Sensor 2 Released");
      activeSensor = 0; // Release active sensor
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

  Serial.print("setMotorSpeed: ");
  Serial.println(rpm);
  sendCommand(speedCommand, sizeof(speedCommand));
}


// return the actual motor speed in RPM 
// return 65535 when error happens
unsigned int readMotorSpeed() {
  // Command to read speed from register 0x8018, read 0x0001=1 number of register
  byte readSpeedCommand[] = {0x01, 0x03, 0x80, 0x18, 0x00, 0x01, 0x2D,0xCD}; 
  digitalWrite(MAX485_DE, HIGH); // Disable receive mode = Enable send mode
  while (RS485Serial.available() > 0) {
    RS485Serial.read(); // clear all previously received data
  }
  \(readSpeedCommand, sizeof(readSpeedCommand)); // Send the read command
  int availableBytes = RS485Serial.available(); // Check if data is available in the RS485 buffer
  if (availableBytes > 0) {
    byte response[availableBytes]; // Create a buffer to store the response
    int index = 0;
    while (RS485Serial.available() > 0) {
      response[index] = RS485Serial.read();
      index++;
    }
    // Display the raw response (in hexadecimal format)
    // Serial.print("Raw response: ");
    // for (int i = 0; i < availableBytes; i++) {
    //   Serial.print(response[i], HEX);  // Print each byte in HEX format
    //   Serial.print(" ");
    // }
    // Serial.println(); // Newline after the raw response display

    // Extract the RPM data in decimal
    for (int i = 0; i < availableBytes; i++) {
      if (response[i]==0x01 && response[i+1]==0x03 && response[i+2]==0x02) // the bytes before the RPM data
      {
        // Probably add the CRC check here ......
        unsigned int combinedValue = (response[i+4] << 8) | response[i+3]; // 2 bytes for speed data
        unsigned int motorRPM = combinedValue * 5; // in decimal, 5 is 20/(number of poles), see driver manual
        return motorRPM;
      }
    }
    // no valid RPM data: 
    return 65535; // max value for unsigned int
        
  } else {
    // no response: 
    return 65535; // max value for unsigned int
  }
}

// read motor rotation direction (FR value)
// For CW, FR=0, for CCW, FR=1
// return 65535 when error happens
unsigned int readMotorDirection(){
  // Command to read FR value from register 0x8000, read 0x0001=1 number of register
  byte readDirectionCommand[] = {0x01, 0x03, 0x80, 0x00, 0x00, 0x01, 0xAD,0xCA}; 
  digitalWrite(MAX485_DE, HIGH); // Disable receive mode = Enable send mode
  while (RS485Serial.available() > 0) {
    RS485Serial.read(); // clear all previously received data
  }
  sendCommand(readDirectionCommand, sizeof(readDirectionCommand));
  // Check if data is available in the RS485 buffer
  int availableBytes = RS485Serial.available();
  if (availableBytes > 0) {
    byte response[availableBytes]; // Create a buffer to store the response
    int index = 0;
    while (RS485Serial.available() > 0) {
      response[index] = RS485Serial.read();
      index++;
    }    
    // Display the raw response (in hexadecimal format)
    // Serial.print("Raw response: ");
    // for (int i = 0; i < availableBytes; i++) {
    //   Serial.print(response[i], HEX);  // Print each byte in HEX format
    //   Serial.print(" ");
    // }
    // Serial.println(); // Newline after the raw response display

    // Extract the FR data
    for (int i = 0; i < availableBytes; i++) {
      if (response[i]==0x01 && response[i+1]==0x03 && response[i+2]==0x02) // the bytes before the RPM data
      {
        // Probably add the CRC check here ......
        unsigned int FR = (response[i+3] >> 1) & 0x01; // get the second lowest bit value
        return FR;
      }
    }
    // no valid FR data: 
    return 65535; // max value for unsigned int
        
  } else {
    // no response: 
    return 65535; // max value for unsigned int
  }
}

// set EN=1 and FR=0
void enableCW() {
  byte enableCWCommand[] = {0x01, 0x06, 0x80, 0x00, 0x09, 0x02, 0x27, 0x9B};
  // 01: motor address; 06: Write Single Register in Modbus protocol; 
  // 8000: Register address; 09: 1001 in binary, EN=1, NW=1 (see driver manual); 02: number of pole pairs;
  // 27: CRC Check Lo; 9B: CRC Check Hi; the full CRC code is thus 0x9B27
  sendCommand(enableCWCommand, sizeof(enableCWCommand));
}
// set EN=1 and FR=1
void enableCCW(){
  byte enableCCWCommand[] = {0x01, 0x06, 0x80, 0x00, 0x0B, 0x02, 0x26, 0xFB};
  // 0x0B: 1011 in binary, NW=1, FR=1, EN=1
  sendCommand(enableCCWCommand, sizeof(enableCCWCommand));
}

// stop the motor by setting EN=0
void naturalStop(){
  unsigned int FR = readMotorDirection();
  if (FR==0){
    byte naturalStopCWCommand[] = {0x01, 0x06, 0x80, 0x00, 0x08, 0x02, 0x26, 0x0B};
    // 0x08: 1000 in binary, NW=1, EN=0
    sendCommand(naturalStopCWCommand, sizeof(naturalStopCWCommand));
  } else if (FR==1){
    byte naturalStopCCWCommand[] = {0x01, 0x06, 0x80, 0x00, 0x0A, 0x02, 0x27, 0x6B};
    // 0x0A: 1010 in binary, NW=1, FR=1, EN=0
    sendCommand(naturalStopCCWCommand, sizeof(naturalStopCCWCommand));
  } else {
    Serial.println("From naturalStop: Invalid FR value!!!");
  }
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
  delay(150); // wait for the motor to response
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
