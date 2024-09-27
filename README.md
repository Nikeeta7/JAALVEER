# JAALVEER
// MAKING AN AUTONOMUS WATER SURFACE CLEANING ROBOT
// HERE ID THE CODE FOR CONTROLLING ALL THE FEAURES OF OUR ROBOT AUTOMATICALLY
// WE ARE CURRENTLY WORKING ON THIS ROBOT AND WILL BE UPDATING THIS CODE FREQUENTLY.
#include <Servo.h>         // Library to control servo motors
#include <Wire.h>          // Library for I2C communication (not used here)
#include <SoftwareSerial.h> // Library for serial communication (not needed without GPS)
// Define pin assignments for each component
#define ESC_PIN 9          // Pin connected to ESC signal input (for brushless motor control)
#define MOTOR_PIN 3        // Pin connected to motor driver (used for speed control with PWM)
#define SERVO_PIN 10       // Pin connected to standard servo motor (for sweeping action)
#define TRIG_PIN 2         // Pin connected to the trigger of ultrasonic sensor
#define ECHO_PIN 4         // Pin connected to the echo of ultrasonic sensor
#define CAMERA_PIN 7       // Pin connected to camera module control (if applicable)
#define STRAINER_PIN 8     // Pin connected to servo for varied length strainer
// Create Servo objects for different servos
Servo esc;              // Servo object for controlling the ESC
Servo myServo;         // Servo object for standard servo motor
Servo strainerServo;  // Servo object for the strainer servo
void setup() {
  // Attach the ESC to the defined pin
  esc.attach(ESC_PIN);
  delay(2000); // Wait for 2 seconds to allow ESC initialization
  // Set motor control pin as output
  pinMode(MOTOR_PIN, OUTPUT);
  // Attach servos to their respective pins
  myServo.attach(SERVO_PIN);
  strainerServo.attach(STRAINER_PIN);
  // Set ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT); // Trigger pin as output
  pinMode(ECHO_PIN, INPUT);  // Echo pin as input
  // Set camera pin (if used) as output
  pinMode(CAMERA_PIN, OUTPUT);
  // Start serial communication for debugging
  Serial.begin(9600);
  // Initialize motor speed to 0 (off)
  analogWrite(MOTOR_PIN, 0);
}
void loop() {
  // Control the ESC for motor speed
  esc.write(90);  // Set throttle to 50% (mid-point)
  delay(2000);    // Run for 2 seconds
  esc.write(0);   // Set throttle to 0% (stop motor)
  delay(2000);    // Pause for 2 seconds
  esc.write(180); // Set throttle to 100% (full speed)
  delay(2000);    // Run for 2 seconds
  // Measure distance using the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);         // Clear the trigger pin
  delayMicroseconds(2);                // Wait for 2 microseconds
  digitalWrite(TRIG_PIN, HIGH);        // Set trigger pin high
  delayMicroseconds(10);               // Wait for 10 microseconds
  digitalWrite(TRIG_PIN, LOW);         // Set trigger pin low
  long duration = pulseIn(ECHO_PIN, HIGH);  // Measure the duration of the echo pulse
  long distance = (duration / 2) / 29.1;   // Calculate distance in cm
  // Print distance to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  // Control motor speed based on distance
  if (distance < 20) {
    analogWrite(MOTOR_PIN, 255); // Full speed if object is within 20 cm
  } else {
    analogWrite(MOTOR_PIN, 0);   // Stop motor if object is beyond 20 cm
  }
  // Sweep the servo motor from 0 to 180 degrees and back
  for (int pos = 0; pos <= 180; pos++) {
    myServo.write(pos);         // Move servo to position
    delay(15);                  // Wait for the servo to reach the position
  }
  for (int pos = 180; pos >= 0; pos--) {
    myServo.write(pos);         // Move servo back to position
    delay(15);                  // Wait for the servo to reach the position
  }
  // Control the strainer servo based on distance
  if (distance < 10) {
    strainerServo.write(0);    // Retract strainer if distance is less than 10 cm
  } else if (distance < 20) {
    strainerServo.write(90);   // Partially extend strainer if distance is between 10 and 20 cm
  } else {
    strainerServo.write(180);  // Fully extend strainer if distance is greater than 20 cm
  }
  // Control camera module (if applicable)
  digitalWrite(CAMERA_PIN, HIGH); // Turn camera on (if required)
  delay(1000);                    // Wait for 1 second
  digitalWrite(CAMERA_PIN, LOW);  // Turn camera off (if required)
  // Delay before next loop iteration
  delay(1000); // 1-second delay
} 
  // HERE IS THE CODE FOR GPS WORKING (NEO - 6M)
  #include <TinyGPS++.h>
#include <SoftwareSerial.h>
// GPS module communication pins
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;  // Default baud rate for NEO-6M
// Create instances of TinyGPS++ and SoftwareSerial
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
void setup()
{
  Serial.begin(115200);  // Initialize serial monitor
  ss.begin(GPSBaud);     // Initialize software serial for GPS

  Serial.println(F("NEO-6M GPS Module Test"));
}
void loop()
{
  // Check if data is available on GPS module
  while (ss.available() > 0)
  {
    char c = ss.read();  // Read a character from the GPS module
    // Print raw GPS data (NMEA sentences) for debugging purposes
    Serial.print(c);
    // Feed the GPS data into the TinyGPS++ library
    if (gps.encode(c))
    {
      // If a valid location is obtained, display the data
      if (gps.location.isUpdated())
      {
        Serial.println(F("\n--- GPS Data ---"));
        Serial.print(F("Latitude: "));
        Serial.println(gps.location.lat(), 6);
        Serial.print(F("Longitude: "));
        Serial.println(gps.location.lng(), 6);
        Serial.print(F("Satellites: "));
        Serial.println(gps.satellites.value());
        Serial.print(F("HDOP: "));
        Serial.println(gps.hdop.value());
        Serial.println();
      }}}}




