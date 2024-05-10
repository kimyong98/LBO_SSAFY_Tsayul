/*
Don't Use Interrupt as attachInterrupt. It will be occurs a lot of interrupts, bit by bit 
*/


#include <HardwareSerial.h>

#define RX_PIN 16
#define TX_PIN 17

HardwareSerial mySerial(2);



double distance;



void TaskMakeDistance(void *param) {
  uint8_t data[8];
  bool is_now_can_print = false;
  for(;;) {
    // Update distance(Latest data)
    while(mySerial.available() >= 8) {
      // ex) 8byte 8byte "8byte" 3byte
      for(int i = 0; i < 8; i++) {
        data[i] = mySerial.read();
      }

      is_now_can_print = true;
    }

    if(is_now_can_print) {
      memcpy(&distance, data, sizeof(double));
      Serial.print("Distance: ");
      Serial.println(distance, 12);
      is_now_can_print = false;
    }


    vTaskDelay(10);
  }
  
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  xTaskCreate(TaskMakeDistance, "TaskMakeDistance", 2048, NULL, 1, NULL);
}

void loop() {  
  vTaskDelay(10000);
}
