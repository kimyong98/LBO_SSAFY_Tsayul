/*
Don't Use Interrupt as attachInterrupt. It will be occurs a lot of interrupts, bit by bit 
*/


#include <HardwareSerial.h>

#define RX_PIN 16
#define TX_PIN 17

HardwareSerial mySerial(2);

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() {
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n');
    double val = inputString.toDouble();
    char temp[20];
    sprintf(temp, "%08.5f", val);
    temp[]
    Serial.print("temp: ");
    Serial.println(temp);

    double debug = atof(temp);


    // for(int i = 0; i < 8; i++) {
    //   // Must be "write()", Not "print()"
    //   mySerial.write(data[i]);
    //   Serial.print(data[i]);
    //   Serial.print(", ");
    // }

    Serial.println(debug, 8);
  }
}