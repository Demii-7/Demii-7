/*
// No Parity Example
void setup() {
  Serial.begin(9600, SERIAL_8N1); // Set baudrate to 9600
  delay(2000); // Wait for baud rate to stabilize before trasnmitting data
}

void loop() {
  // put your main code here, to run repeatedly:
  byte dataToTransmit = 0x55; // Transmit data = 85 = binary 01010101 = ASCII U
  Serial.write(dataToTransmit);
  delay(2500); // detay for 2.5 seconds before attempting to transmit again
}
*/
/*
// Even Parity
void setup() {
  Serial.begin(9600, SERIAL_8E1); // Set baudrate to 9600
  delay(2000); // Wait for baud rate to stabilize before trasnmitting data
}

void loop() {
  // put your main code here, to run repeatedly:
  byte dataToTransmit = 0x3A; // Transmit data = 58 = binary 00111010 = ASCII :
  Serial.write(dataToTransmit);
  delay(2500); // detay for 2.5 seconds before attempting to transmit again
}
/*


// Odd Parity
void setup() {
  Serial.begin(115200, SERIAL_8O1); // Set baudrate to 9600
  delay(2000); // Wait for baud rate to stabilize before trasnmitting data
}

void loop() {
  // put your main code here, to run repeatedly:
  byte dataToTransmit = 0x78; // Transmit data = 120 = binary 01111000 = ASCII x
  Serial.write(dataToTransmit);
  delay(2500); // detay for 2.5 seconds before attempting to transmit again
}
*/

// Define protocol constants
#define ST1 0x81
#define ST2 0xA1
#define TRM 0x00
#define SEP 0x3B // ';'
#define MT_LED_CONTROL 0xC1
#define MT_SENSOR_REQUEST 0xB1
#define MT_SENSOR_RESPONSE 0xB2 

// Message size constraints
const int MAX_MESSAGE_SIZE = 24;
const int MAX_VALUE_SIZE = 10;

// LED pin definitions
const int LED_R_PIN = 9;
const int LED_G_PIN = 10;
const int LED_B_PIN = 11;

// Buffer for incoming data
char rxBuffer[MAX_MESSAGE_SIZE + 1]; // +1 for NULL terminator
int rxIndex = 0;
bool receiving_message = false; // Flag to indicate if we are currently inside a message

void setup() {
  // Initialize UART communication at a common baud rate
  Serial.begin(9600); // 9600 baud rate 
  
  // Set LED pins as output
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  
  while (Serial.available() > 0) {}

  Serial.println("UART Protocol Initialized. Waiting for data...");
}

void loop() {
  // Check for available serial data without blocking the main code
  while (Serial.available() > 0) {
    int incomingByte = Serial.read();  // Read the next byte

    // Print the raw byte value (in hexadecimal) and ASCII if printable
    Serial.print("Received byte (hex):");
    //Serial.println((int)incomingByte, HEX);
    
    // Message Detection Logic
    if (!receiving_message) {
      Serial.write(incomingByte);
      // Look for the start sequence: ST1 then ST2
      if (incomingByte == 0x81) {
        Serial.println("Found ST1 byte.");
        // First Start Byte found, wait for the second
      }else{
        if (incomingByte == 0xA1) {
          Serial.println("Found ST2 byte.");
          receiving_message = true;
          rxIndex = 0;  // Start fresh accumulation
          Serial.println("Start sequence found: ST1, ST2");
        } else {
          Serial.println("Error: Second byte is not ST2.");
        }
      }
    } else {
      // Accumulate data bytes after ST1, ST2 are detected
      if (rxIndex < MAX_MESSAGE_SIZE) {
        rxBuffer[rxIndex++] = incomingByte;
        Serial.print("Accumulating byte: 0x");
        Serial.println((int)incomingByte, HEX);
      }

      // Check for the End Byte (Terminator)
      if (incomingByte == TRM) {
        receiving_message = false;
        rxBuffer[rxIndex] = '\0';  // Null-terminate the accumulated data
        Serial.println("End byte (TRM) received. Processing message...");
        processMessage(rxBuffer);  // Call the message processing function
        rxIndex = 0;  // Reset index for next message
      }

      // Safety check: if buffer overflows, discard and reset
      if (rxIndex >= MAX_MESSAGE_SIZE) {
        Serial.println("Error: Message too long. Discarding.");
        receiving_message = false;
        rxIndex = 0;
      }
    }
  }
}


/* Parses the received message buffer and acts based on the Message Type.
 * The buffer starts after ST1 and ST2, and ends with TRM.
 * Structure in rxBuffer: [MT] [V1] [SEP] [V2] [SEP] ... [VN] [SEP] [TRM]
 */
void processMessage(char* buffer) {
  // Check if buffer contains at least the Message Type and Terminator
  if (rxIndex < 2) { 
    Serial.println("Error: Malformed message received (too short).");
    return;
  }

  // The first byte is the Message Type
  byte messageType = buffer[0]; 
  
  Serial.print("Message Type Extracted: 0x");
  Serial.println(messageType, HEX);

  // Check if message type matches the expected values
  if (messageType == MT_LED_CONTROL) {
    Serial.println("LED Control message detected.");
    handleLEDControl(buffer + 1); // Pass the data portion of the buffer
  } else if (messageType == MT_SENSOR_REQUEST) {
    Serial.println("Sensor Request message detected.");
    handleSensorRequest();
  } else if (messageType == MT_SENSOR_RESPONSE) {
    Serial.println("Sensor Response message detected.");
    handleSensorRequest();
  }else {
    Serial.print("Warning: Unknown Message Type 0x");
    Serial.println(messageType, HEX);
  }
}

/**
 * Handles the LED control message (MT_LED_CONTROL).
 * Expected data: [Value 1 (R)] [SEP] [Value 2 (G)] [SEP] [Value 3 (B)] [SEP] [TRM]
 */
void handleLEDControl(char* data) {
  Serial.println("Handling LED Control message...");
  
  // Find the separator and extract values as strings
  char *str_part, *str_remainder;
  
  // The first call finds the first value
  str_part = strtok_r(data, ";", &str_remainder); 
  
  int values[3] = {-1, -1, -1};  // Initialize to -1 for error checking
  int count = 0;
  
  Serial.println("Extracting RGB Values:");

  while (str_part != NULL && count < 3) {
    // Print the extracted part before converting
    Serial.print("Extracted Value (as string): ");
    Serial.println(str_part);
    
    // Convert the char array (string) value to an integer for PWM
    values[count] = atoi(str_part); 
    
    // Safety clamp PWM value (0-255)
    if (values[count] < 0) values[count] = 0;
    if (values[count] > 255) values[count] = 255;
    
    // Print the value after conversion
    Serial.print("Converted Numeric Value: ");
    Serial.println(values[count]);
    
    count++;
    
    // The next call continues from the remainder
    str_part = strtok_r(NULL, ";", &str_remainder); 
  }

  if (count == 3) {
    Serial.print("LED Brightness R: "); Serial.print(values[0]);
    Serial.print(" G: "); Serial.print(values[1]);
    Serial.print(" B: "); Serial.println(values[2]);
    
    // Update the PWM values for the LEDs 
    analogWrite(LED_R_PIN, values[0]);
    analogWrite(LED_G_PIN, values[1]);
    analogWrite(LED_B_PIN, values[2]);
  } else {
    Serial.println("Error: LED message expected 3 values, got " + String(count));
  }
}

/**
 * Handles the sensor request message (MT_SENSOR_REQUEST).
 * Sends a sensor reading back.
 */
void handleSensorRequest() {
  Serial.println("Handling Sensor Request message. Sending fake sensor data...");
  
  // 1. Read the sensor (using a placeholder value)
  float sensorReading = 25.43; 
  
  // 2. Prepare the response message
  // Response: [ST1] [ST2] [MT_SENSOR_RESPONSE] [Sensor Value] [SEP] [TRM]
  
  // Convert float to char array as required by the protocol
  char valueBuffer[MAX_VALUE_SIZE];
  dtostrf(sensorReading, 1, 2, valueBuffer); // Convert float to string (min 1 width, 2 decimal places)
  
  // Construct and send the response frame
  Serial.write(ST1);
  Serial.write(ST2);
  Serial.write(MT_SENSOR_RESPONSE); // Use the response type
  
  // Send the sensor value char array
  Serial.print(valueBuffer); 
  
  // Send the separator and terminator
  Serial.write(SEP);
  Serial.write(TRM);
  
  Serial.println("Sensor data sent.");
}