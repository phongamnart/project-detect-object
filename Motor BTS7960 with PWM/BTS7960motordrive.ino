int SENSOR_PIN = A0;

int RPWM_Output = 22;
int LPWM_Output = 23;

const int freq = 5000; // ความถี่ในการสร้างสัญญาณ PWM
const int ledChannel = 0; // ช่อง 0-15
const int resolution = 8; // ความละเอียด 0-16 bit

void setup()
{
  Serial.begin(115200);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(RPWM_Output, ledChannel);

}

void loop()
{
  int sensorValue = map(analogRead(33), 0, 4095, 0, 255);
  Serial.println(sensorValue);


  digitalWrite(LPWM_Output, 0);
  ledcWrite(ledChannel, sensorValue);


}