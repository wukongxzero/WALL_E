/*
 * HC-05 AT Command Pass-through for Arduino Mega
 * Uses Hardware Serial1 (Pins 18 & 19)
 */

void setup() {
  // Start serial communication with the computer
  Serial.begin(9600);
  
  // Start serial communication with the HC-05
  // Note: 38400 is the default baud rate for HC-05 AT Mode
  Serial1.begin(38400); 

  Serial.println("HC-05 AT Mode Configurator Ready.");
  Serial.println("Make sure your Serial Monitor is set to 'Both NL & CR' and '9600 baud'.");
  Serial.println("Type AT to test the connection.");
  Serial.println("--------------------------------------------------");
}

void loop() {
  // Read from HC-05 and print to computer
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  
  // Read from computer and send to HC-05
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
}
