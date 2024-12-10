/* 
Reference:
RS485 and Modbus Protocol: https://www.modbustools.com/modbus.html
Motor: https://www.omc-stepperonline.com/24v-172w-35rpm-geared-brushless-dc-motor-100-1-high-precision-gearbox-57blr70-24-02-hg100
Motor driver: https://www.omc-stepperonline.com/digital-brushless-dc-motor-driver-12v-48vdc-max-15-0a-400w-bld-510b
Arduino: https://docs.arduino.cc/hardware/uno-r4-minima/
MAX485 interface: https://www.reichelt.com/ch/en/shop/product/developer_boards_-_ttl_to_rs485_max485-282703
*/

#include <SoftwareSerial.h>

// Define pins for MAX485
// DE RE are connected to the same pin for half-duplex communication (either send or receive at a given time)
#define MAX485_DE 4
#define MAX485_RE 4
#define RX_PIN 2
#define TX_PIN 3

// Create a SoftwareSerial instance
SoftwareSerial RS485Serial(RX_PIN, TX_PIN);

// Function prototypes
void sendCommand(byte command[], size_t length);
void readMotorSpeed();


void setup() {
  // Set up DE/RE pin for output
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, LOW); // Start in receive mode

  // Start serial communication
  RS485Serial.begin(9600); // Ensure this matches your motor driver's baud rate
  Serial.begin(9600);      // For debugging
}

void loop() {
  // Send the 2 Pole Pair Start Command in setup
  byte twoPolePairStart[] = {0x01, 0x06, 0x80, 0x00, 0x09, 0x02, 0x27, 0x9B};
  Serial.println("Sending 2 Pole Pair Start command...");
  sendCommand(twoPolePairStart, sizeof(twoPolePairStart));
  delay(1000); 
  // Wait for 1 second
  
  // Write Speed 1000 RPM
  byte writeSpeed1000[] = {0x01, 0x06, 0x80, 0x05, 0xE8, 0x03, 0xBE, 0x0A};
  Serial.println("Sending Write Speed 1000 RPM command...");
  sendCommand(writeSpeed1000, sizeof(writeSpeed1000));
  delay(3000); // Wait for 3 seconds
  
  // Read motor speed
  readMotorSpeed();
  
  // Write Speed 1200 RPM
  byte writeSpeed1200[] = {0x01, 0x06, 0x80, 0x05, 0xB0, 0x04, 0xC4, 0x08};
  Serial.println("Sending Write Speed 1200 RPM command...");
  sendCommand(writeSpeed1200, sizeof(writeSpeed1200));
  delay(3000); // Wait for 3 seconds
  
  // Read motor speed
  readMotorSpeed();

  // Natural Stop
  byte naturalStop[] = {0x01, 0x06, 0x80, 0x00, 0x08, 0x02, 0x26, 0x0B};
  Serial.println("Sending Natural Stop command...");
  sendCommand(naturalStop, sizeof(naturalStop));
  delay(4000); // Wait for 4 seconds

  // Read motor speed
  readMotorSpeed();
}

void sendCommand(byte command[], size_t length) {
  // Enable send mode
  digitalWrite(MAX485_DE, HIGH);

  // Send the command over RS485
  for (size_t i = 0; i < length; i++) {
    RS485Serial.write(command[i]);
  }
  RS485Serial.flush(); // Wait until all data is sent

  // Switch back to receive mode
  digitalWrite(MAX485_DE, LOW);
}

void readMotorSpeed() {
  // Command to read speed from register 0x8018, read 0x0001 number of register
  byte readSpeedCommand[] = {0x01, 0x03, 0x80, 0x18, 0x00, 0x01, 0x2D,0xCD}; 
  
  // Disable receive mode = Enable send mode
  digitalWrite(MAX485_DE, HIGH);
  // clear all received data
  while (RS485Serial.available() > 0) {
    RS485Serial.read();
  }
  
  // Send the read command to get motor speed
  sendCommand(readSpeedCommand, sizeof(readSpeedCommand));

  // Wait for the response 
  delay(100); // Give the motor driver time to respond

  // Check if data is available in the RS485 buffer
  int availableBytes = RS485Serial.available();
  if (availableBytes > 0) {
    byte response[availableBytes]; // Create a buffer to store the response
    int index = 0;
    
    // Read the available bytes into the response array
    while (RS485Serial.available() > 0) {
      response[index] = RS485Serial.read();
      index++;
    }
    
    // Display the raw response (in hexadecimal format)
    Serial.print("Raw response: ");
    for (int i = 0; i < availableBytes; i++) {
      Serial.print(response[i], HEX);  // Print each byte in HEX format
      Serial.print(" ");
    }
    Serial.println(); // Newline after the raw response display

    // Extract the RPM data in decimal
    for (int i = 0; i < availableBytes; i++) {
      if (response[i]==0x01 && response[i+1]==0x03 && response[i+2]==0x02) // the bytes before the RPM data
      {
        // Probably add the CRC check here
        // .....
        //
        unsigned int combinedValue = (response[i+4] << 8) | response[i+3]; // 2 bytes for speed data
        unsigned int motorRPM = combinedValue * 5; // in decimal, 5 is 20/(number of poles)
        Serial.print("The actual RPM of motor is: ");
        Serial.println(motorRPM);
        break; // exit the FOR loop
      } else {
        Serial.println("X");
      }
    }
        
  } else {
    Serial.println("Error: No response or incomplete data received.");
  }
}

