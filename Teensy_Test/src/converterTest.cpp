#if NATIVE
#include <iostream>

#include <Arduino.h>


#define SDA_PIN 25
#define SCL_PIN 24
#define TX_PIN  1


void setup() {

    Serial.begin(115200);


    pinMode(SDA_PIN, OUTPUT);
    pinMode(SCL_PIN, OUTPUT);
    pinMode(TX_PIN, OUTPUT);

    digitalWrite(SDA_PIN, HIGH);
    digitalWrite(SCL_PIN, HIGH);
    digitalWrite(TX_PIN, HIGH);

}

void loop() {
    digitalWrite(SDA_PIN, HIGH);
    digitalWrite(SCL_PIN, HIGH);
    digitalWrite(TX_PIN, HIGH);
    Serial.println("HIGH");
    delay(1000);

    digitalWrite(SDA_PIN, LOW);
    digitalWrite(SCL_PIN, LOW);
    digitalWrite(TX_PIN, LOW);
    Serial.println("LOW");
    delay(1000);
}

#endif

