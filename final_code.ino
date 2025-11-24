#include <ESP32Servo.h>

// IR sensor pins
#define IR_25_PIN 32
#define IR_50_PIN 33
#define IR_75_PIN 25
#define IR_100_PIN 26

// Ultrasonic pins
#define TRIG_PIN 23
#define ECHO_PIN 22

// Servo pin
#define SERVO_PIN 13

// Threshold
#define HAND_TRIGGER_DISTANCE 20

Servo lidServo;
bool lidOpen = false;
unsigned long lidOpenTime = 0;

void setup() {
  Serial.begin(115200);

  // IR sensors
  pinMode(IR_25_PIN, INPUT);
  pinMode(IR_50_PIN, INPUT);
  pinMode(IR_75_PIN, INPUT);
  pinMode(IR_100_PIN, INPUT);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo
  lidServo.attach(SERVO_PIN);
  lidServo.write(0); // start closed
}

void loop() {
  // --- Bin fill ---
  bool ir25 = digitalRead(IR_25_PIN) == LOW;
  bool ir50 = digitalRead(IR_50_PIN) == LOW;
  bool ir75 = digitalRead(IR_75_PIN) == LOW;
  bool ir100 = digitalRead(IR_100_PIN) == LOW;

  int binLevel = 0;
  if (ir100) binLevel = 100;
  else if (ir75) binLevel = 75;
  else if (ir50) binLevel = 50;
  else if (ir25) binLevel = 25;

  // --- Ultrasonic ---
  long handDistance = getDistance();

  if (handDistance > 0 && handDistance <= HAND_TRIGGER_DISTANCE) {
    if (!lidOpen) {
      Serial.println("Hand detected, trash can opening");
      lidServo.write(90); // open angle
      lidOpen = true;
      lidOpenTime = millis();
    }
    // If already open, stay open
  } else {
    if (lidOpen && (millis() - lidOpenTime >= 5000)) {
      Serial.println("Hand not detected, dustbin closing");
      lidServo.write(0); // closed angle
      lidOpen = false;
    }
  }

  // --- Serial output ---
  Serial.println("---------------");
  Serial.print("Bin fill level: ");
  Serial.print(binLevel);
  Serial.println("%");

  if (handDistance > 0) {
    Serial.print("Hand distance: ");
    Serial.print(handDistance);
    Serial.println(" cm");
  } else {
    Serial.println("Hand distance: no echo");
  }

  Serial.print("Trash can state: ");
  Serial.println(lidOpen ? "OPEN" : "CLOSED");

  delay(2000); // slower updates
}

long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms timeout
  if (duration == 0) return 0; // no echo
  return duration * 0.034 / 2; // cm
}