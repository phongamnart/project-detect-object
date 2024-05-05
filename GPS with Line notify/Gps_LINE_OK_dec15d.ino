#include <TinyGPS++.h>
#include <HardwareSerial.h> // เพิ่มไลบรารี HardwareSerial
#include <WiFi.h> // เปลี่ยนไลบรารีให้เป็น WiFi.h
#include <HTTPClient.h> // เพิ่มไลบรารี HTTPClient.h
#include <ArtronShop_LineNotify.h> // นำเข้าไลบารี่ ArtronShop_LineNotify

static const int RXPin = 22, TXPin = 21; // ตั้งค่าขา RX และ TX ของ GPS ใหม่
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // ใช้ HardwareSerial สำหรับ GPS ใน ESP32

// Config connect WiFi
#define WIFI_SSID "Terk"
#define WIFI_PASSWORD "12345678"

// Line config
#define LINE_TOKEN "fbIzZs8EUDTvsoBZvdsdRMGIqnT3gULg7eVxzvG7MOb"

String message = "https%3A%2F%2Fwww.google.com%2Fmaps%2Fplace%2F";
String la = "7.012919" ;
String lon = "100.473209" ;

void setup() {
  Serial.begin(115200); // เปลี่ยนความเร็ว Serial เป็น 115200 bps
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin); // เริ่มต้นการใช้งาน HardwareSerial สำหรับ GPS
  
  Serial.println("start");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("connecting");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  gps_sen();
  delay(1000); // หน่วงเวลา 1 วินาที
}

void gps_sen(){
  while (gpsSerial.available() > 0){
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()){
      la = String(gps.location.lat(),6); 
      lon =  String(gps.location.lng(),6) ;
      Serial.print("Latitude= "); 
      Serial.print(la);
      Serial.print(" Longitude= "); 
      Serial.println(lon);
      message = "https%3A%2F%2Fwww.google.com%2Fmaps%2Fplace%2F";
      message += la + "%2C%2B" + lon + "%2F%40" + la + "%40"+ lon + "%2C18z  " ;
      Line_Notify(message);
    }
  } 
}

void Line_Notify(String message) {
  HTTPClient http;
  
  http.begin("https://notify-api.line.me/api/notify");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.addHeader("Authorization", "Bearer " + String(LINE_TOKEN));
  
  int httpResponseCode = http.POST("message=" + message);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(httpResponseCode);
    Serial.println(response);
  } else {
    Serial.println("Error on sending POST: " + String(httpResponseCode));
  }
  
  http.end();
}
