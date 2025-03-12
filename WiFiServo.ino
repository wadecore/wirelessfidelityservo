#include <ESP8266WiFi.h>

// Wi-Fi credentials
const char* ssid = "Your_WiFi_SSID";
const char* password = "Your_WiFi_Password";

// Motor driver pins
const int motorPin1 = D1; // IN1 on L298N
const int motorPin2 = D2; // IN2 on L298N
const int enablePin = D3; // ENA on L298N

// Rotary hall effect sensor pin
const int hallSensorPin = A0; // Connect sensor output to A0

// RGB LED pins
const int redPin = D5;
const int greenPin = D6;
const int bluePin = D7;

// Buzzer pin
const int buzzerPin = D8;

// Wi-Fi server
WiFiServer server(80);

// PID constants
float Kp = 2.0; // Proportional gain
float Ki = 0.1; // Integral gain
float Kd = 0.5; // Derivative gain

// PID variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// Motor control variables
int targetPosition = 0; // Target position (0-360 degrees)
int currentPosition = 0; // Current position (0-360 degrees)
int motorSpeed = 128; // Default speed (0-255)

// Sensor calibration
const int sensorMin = 0;   // Minimum sensor reading (e.g., 0 degrees)
const int sensorMax = 1023; // Maximum sensor reading (e.g., 360 degrees)

// Buzzer notes
const int noteC = 262; // Frequency for note C4
const int noteE = 330; // Frequency for note E4
const int noteG = 392; // Frequency for note G4

// Servo ID
String servoID = "Servo1"; // Default servo ID

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize motor driver pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Initialize RGB LED pins
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Initialize buzzer pin
  pinMode(buzzerPin, OUTPUT);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Wi-Fi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server.begin();
}

void loop() {
  // Check for client connections
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client connected");
    String request = client.readStringUntil('\r');
    Serial.println(request);
    client.flush();

    // Handle the request
    if (request.indexOf("GET /setPosition") >= 0) {
      // Extract parameters from the request
      int idStart = request.indexOf("id=");
      int posStart = request.indexOf("pos=");
      int speedStart = request.indexOf("speed=");

      if (idStart != -1 && posStart != -1 && speedStart != -1) {
        String requestedID = request.substring(idStart + 3, posStart - 1);
        if (requestedID == servoID) { // Check if the ID matches
          targetPosition = request.substring(posStart + 4, speedStart - 1).toInt();
          motorSpeed = request.substring(speedStart + 6).toInt();

          // Report receiving the command
          Serial.print("Received command to move to position: ");
          Serial.println(targetPosition);

          // Play a 3-note jingle
          playJingle();
        }
      }
    } else if (request.indexOf("GET /setID") >= 0) {
      // Extract new servo ID from the request
      int idStart = request.indexOf("id=");
      if (idStart != -1) {
        servoID = request.substring(idStart + 3);
        Serial.print("Servo ID set to: ");
        Serial.println(servoID);
      }
    }

    // Send the current position and servo ID as a response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("");
    client.print("Servo ID: ");
    client.println(servoID);
    client.print("Current Position: ");
    client.println(currentPosition);
    client.print("Motor Speed: ");
    client.println(motorSpeed);
    client.stop();
    Serial.println("Client disconnected");
  }

  // Read current position from the rotary hall effect sensor
  int sensorValue = analogRead(hallSensorPin);
  currentPosition = map(sensorValue, sensorMin, sensorMax, 0, 360); // Map to 0-360 degrees

  // Calculate PID output
  error = targetPosition - currentPosition;
  integral += error;
  derivative = error - lastError;
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  // Constrain the output to motor speed limits
  int constrainedOutput = constrain(output, -motorSpeed, motorSpeed);

  // Control the motor
  if (constrainedOutput > 0) {
    // Rotate clockwise
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, abs(constrainedOutput));
    setRGB(255, 0, 0); // Red

    // Report moving to the target position
    Serial.print("Moving to position: ");
    Serial.println(targetPosition);
  } else if (constrainedOutput < 0) {
    // Rotate counter-clockwise
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(enablePin, abs(constrainedOutput));
    setRGB(0, 0, 255); // Blue

    // Report moving to the target position
    Serial.print("Moving to position: ");
    Serial.println(targetPosition);
  } else {
    // Stop the motor
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, 0);
    setRGB(0, 255, 0); // Green

    // Report arrival at the target position
    Serial.print("Arrived at position: ");
    Serial.print(currentPosition);
    Serial.print(" (Hall effect sensor value: ");
    Serial.print(sensorValue);
    Serial.println(")");
  }

  // Small delay to avoid excessive updates
  delay(10);
}

// Function to set RGB LED color
void setRGB(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

// Function to play a 3-note jingle
void playJingle() {
  tone(buzzerPin, noteC, 200); // Play note C
  delay(250);
  tone(buzzerPin, noteE, 200); // Play note E
  delay(250);
  tone(buzzerPin, noteG, 200); // Play note G
  delay(250);
  noTone(buzzerPin); // Stop the buzzer
}