#define BLYNK_TEMPLATE_ID "TMPLXMuFJ0Eq"
#define BLYNK_DEVICE_NAME "DO AN 1B"
#define BLYNK_AUTH_TOKEN "cNDC8DDZ5Zi9J7POa7brcGX9gehb0QsG"
#define BLYNK_FIRMWARE_VERSION        "0.1.0"
#define BLYNK_PRINT Serial
#define led2 12
#define led3 13
#define soil A0
boolean bt1_state=HIGH;
boolean bt2_state=HIGH;
#define APP_DEBUG
#define USE_NODE_MCU_BOARD
#include "BlynkEdgent.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <BlynkSimpleEsp8266_SSL.h>  // Đảm bảo bạn đã cài đặt thư viện này

#include <ESP8266WiFi.h> //
char auth[] = "cNDC8DDZ5Zi9J7POa7brcGX9gehb0QsG";
char ssid[] = "iPhone 7 Plus";
char pass[] = "061002ak";
const int sensor_pin = A0;
String giatri;
String t,h;
unsigned long times=millis();
int soilLevel, soilR;
WidgetLED led(V0);
BlynkTimer timer;

void readSoil()
{

    Blynk.virtualWrite(V6,soilLevel);
  
}

void blinkLedWidget(){
  if (led.getValue()) {
    led.off();
  } else {
    led.on();
  }
}

  BLYNK_WRITE(V3){
  int p = param.asInt();
  digitalWrite(led2, p); 
    if(bt1_state==1 ){
      digitalWrite(led2,1);
      Blynk.virtualWrite(V3,digitalRead(led2));
      bt1_state=0;
      delay(200);
    }
  }
BLYNK_WRITE(V4){
  int p = param.asInt();
  digitalWrite(led3, p); 

    if(bt2_state==1 ){
   
      digitalWrite(led3,1);
      Blynk.virtualWrite(V4,digitalRead(led3));
      bt2_state=0;
      delay(200);
    }}
void setup(){
  Serial.begin(9600);
  delay(100);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(1000L, blinkLedWidget);
}
void loop() {
  Blynk.run();
  timer.run();
      if (Serial.available() >= 2) {
    int lowByte = Serial.read();  // Đọc byte thấp đầu tiên
    int highByte = Serial.read(); // Đọc byte cao thứ hai
    int receivedValue = (highByte << 8  ) | lowByte; // Ghép hai byte thành giá trị 16 bit
    int soilLevel =map(receivedValue, 1790, 3970, 100, 0);
    Serial.print("Received Value: ");
    Serial.println(receivedValue);
    Serial.print("soil moisture percent: ");
    Serial.print(soilLevel);
    Serial.println("%");
    Blynk.virtualWrite(V6,soilLevel);
    Serial.print(soilLevel);
      }

   delay(400); 
    times = millis();
     
  }