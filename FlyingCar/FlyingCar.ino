#include <SoftwareSerial.h>

// Bluetooth Pin Setup: RX = Pin 10, TX = Pin 11
// Connect Bluetooth TXD to Arduino Pin 10
// Connect Bluetooth RXD to Arduino Pin 11
SoftwareSerial Bluetooth(10, 11); 

// Left side motors connected to L298N IN1 & IN2
const int IN1 = 2;  
const int IN2 = 3; 

// Right side motors connected to L298N IN3 & IN4
const int IN3 = 4; 
const int IN4 = 5;

char command; // Variable to store data coming from the Bluetooth app

void forward();
void backward();
void turnLeft();
void turnRight();
void stopCar();

void setup() {
  // Set all motor control pins as OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize Bluetooth serial communication at 9600 baud rate
  Bluetooth.begin(9600);
  
  // Initialize USB serial communication for debugging on Serial Monitor
  Serial.begin(9600); 
  Serial.println(F("=========================================="));
  Serial.println(F(" Bluetooth Car Firmware Ready (Pins 2,3,4,5)"));
  Serial.println(F(" Bluetooth RX=10, TX=11 @ 9600 baud"));
  Serial.println(F("=========================================="));
}

void loop() {
  // Check if data is available from the Bluetooth module
  if (Bluetooth.available() > 0) {
    command = Bluetooth.read(); // Read the incoming character
    Serial.print(F("Command received: "));
    Serial.println(command);    // Print the character to Serial Monitor
    
    // Control the car based on the received command
    if (command == 'F' || command == 'f') { forward(); }       // F = Move Forward
    else if (command == 'B' || command == 'b') { backward(); } // B = Move Backward
    else if (command == 'L' || command == 'l') { turnLeft(); } // L = Turn Left
    else if (command == 'R' || command == 'r') { turnRight(); }// R = Turn Right
    else if (command == 'S' || command == 's') { stopCar(); }  // S = Stop the Car
  }
}

// Function to move the car forward
void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println(F("Action: FORWARD"));
}

// Function to move the car backward
void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println(F("Action: BACKWARD"));
}

// Function to spin/turn the car left
void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println(F("Action: TURN LEFT"));
}

// Function to spin/turn the car right
void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println(F("Action: TURN RIGHT"));
}

// Function to stop all motors
void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println(F("Action: STOP"));
}
