const int pingPin1 = 5; // ขาสำหรับ Ultrasonic Sensor ที่ 1
const int pingPin2 = 6; // ขาสำหรับ Ultrasonic Sensor ที่ 2
int inPin1 = 18; // ขาสำหรับ Ultrasonic Sensor ที่ 1
int inPin2 = 19; // ขาสำหรับ Ultrasonic Sensor ที่ 2

void setup() {
  Serial.begin(9600);
}

void loop() {
  long duration1, duration2, cm1, cm2;

  pinMode(pingPin1, OUTPUT);
  pinMode(pingPin2, OUTPUT);

  digitalWrite(pingPin1, LOW);
  digitalWrite(pingPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin1, HIGH);
  digitalWrite(pingPin2, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin1, LOW);
  digitalWrite(pingPin2, LOW);

  pinMode(inPin1, INPUT);
  pinMode(inPin2, INPUT);
  
  duration1 = pulseIn(inPin1, HIGH);
  duration2 = pulseIn(inPin2, HIGH);

  cm1 = microsecondsToCentimeters(duration1);
  cm2 = microsecondsToCentimeters(duration2);

  if (cm1 > 7) {
    Serial.println("Sensor 1: ระยะห่างเกินกำหนด!");
  } else {
    Serial.print("Sensor 1: ");
    Serial.print(cm1);
    Serial.println("cm");
  }

  if (cm2 > 7) {
    Serial.println("Sensor 2: ระยะห่างเกินกำหนด!");
  } else {
    Serial.print("Sensor 2: ");
    Serial.print(cm2);
    Serial.println("cm");
  }

  delay(1000);
}

long microsecondsToCentimeters(long microseconds) {
  // คำนวณระยะทางเช่นเดียวกับเดิม
  return microseconds / 29 / 2;
}
