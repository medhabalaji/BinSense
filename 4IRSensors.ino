// IR sensor pin assignments
#define IR_25_PIN 32
#define IR_50_PIN 33
#define IR_75_PIN 25
#define IR_100_PIN 26

void setup() {
  Serial.begin(115200);

  // Set IR sensor pins as input
  pinMode(IR_25_PIN, INPUT);
  pinMode(IR_50_PIN, INPUT);
  pinMode(IR_75_PIN, INPUT);
  pinMode(IR_100_PIN, INPUT);
}

void loop() {
  // Read sensor states
  bool ir25 = digitalRead(IR_25_PIN) == LOW;
  bool ir50 = digitalRead(IR_50_PIN) == LOW;
  bool ir75 = digitalRead(IR_75_PIN) == LOW;
  bool ir100 = digitalRead(IR_100_PIN) == LOW;

  // Print raw sensor status
  Serial.println("Sensor Status:");
  Serial.print("25%: "); Serial.println(ir25 ? "TRASH DETECTED" : "Clear");
  Serial.print("50%: "); Serial.println(ir50 ? "TRASH DETECTED" : "Clear");
  Serial.print("75%: "); Serial.println(ir75 ? "TRASH DETECTED" : "Clear");
  Serial.print("100%: "); Serial.println(ir100 ? "TRASH DETECTED" : "Clear");

  // Determine bin level
  int binLevel = 0;
  if (ir100) binLevel = 100;
  else if (ir75) binLevel = 75;
  else if (ir50) binLevel = 50;
  else if (ir25) binLevel = 25;

  Serial.print("Bin is approximately ");
  Serial.print(binLevel);
  Serial.println("% full");

  Serial.println("----------------------");
  delay(1000);
}