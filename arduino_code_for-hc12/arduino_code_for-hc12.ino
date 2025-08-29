#include <SoftwareSerial.h>

// HC-12 connections
#define HC12_TX 10   // Arduino pin → HC-12 RX
#define HC12_RX 11   // Arduino pin ← HC-12 TX

SoftwareSerial HC12(HC12_TX, HC12_RX);

void setup() {
  Serial.begin(9600);   // Serial to computer
  HC12.begin(9600);     // Serial to HC-12
}

void loop() {
  // Forward everything from HC-12 → Serial
  if (HC12.available()) {
    while (HC12.available()) {
      char c = HC12.read();
      Serial.write(c);
    }
  }

  // Forward everything from Serial → HC-12
  if (Serial.available()) {
    while (Serial.available()) {
      char c = Serial.read();
      HC12.write(c);
    }
  }
}
