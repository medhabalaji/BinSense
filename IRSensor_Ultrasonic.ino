// IR sensor pins
#define IR_25_PIN 32
#define IR_50_PIN 33
#define IR_75_PIN 25
#define IR_100_PIN 26

// Ultrasonic sensor pins
#define TRIG_PIN 27
#define ECHO_PIN 14

// Servo pin
#define SERVO_PIN 13  // Change if needed

// Hand detection threshold (in cm)
#define HAND_TRIGGER_DISTANCE 20

#include <ESP32Servo.h>
Servo lidServo;

void setup() {
  Serial.begin(115200);

  // IR sensors
  pinMode(IR_25_PIN, INPUT);
  pinMode(IR_50_PIN, INPUT);
  pinMode(IR_75_PIN, INPUT);
  pinMode(IR_100_PIN, INPUT);

  // Ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo setup
  lidServo.setPeriodHertz(50); // Standard 50Hz
  lidServo.attach(SERVO_PIN, 500, 2400); // Pulse width range
  lidServo.write(0); // Lid closed
}

int getBinLevelFromIR() {
  if (digitalRead(IR_100_PIN) == LOW) return 100;
  else if (digitalRead(IR_75_PIN) == LOW) return 75;
  else if (digitalRead(IR_50_PIN) == LOW) return 50;
  else if (digitalRead(IR_25_PIN) == LOW) return 25;
  else return 0;
}

long getDistanceFromUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // cm
  return distance;
}

void openLid() {
  Serial.println("Opening lid...");
  lidServo.write(90); // Open position
  delay(3000);        // Wait 3 seconds
  Serial.println("Closing lid...");
  lidServo.write(0);  // Closed position
}

void loop() {
  // Bin fill level
  int binLevel = getBinLevelFromIR();
  Serial.print("Bin fill level: ");
  Serial.print(binLevel);
  Serial.println("%");

  // Hand detection
  long handDistance = getDistanceFromUltrasonic();
  Serial.print("Hand distance: ");
  Serial.print(handDistance);
  Serial.println(" cm");

  if (handDistance > 0 && handDistance < HAND_TRIGGER_DISTANCE) {
    openLid();
  }

  Serial.println("----------------------");
  delay(1000);
}