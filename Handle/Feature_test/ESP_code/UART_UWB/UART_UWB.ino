/*
Don't Use Interrupt as attachInterrupt. It will be occurs a lot of interrupts, bit by bit 
*/


#include <HardwareSerial.h>
#define UART_QUEUE_SIZE 1024

#define RX_PIN 16
#define TX_PIN 17

HardwareSerial mySerial(2);

SemaphoreHandle_t uart_Semaphore;


uint8_t uart_rx_queue[UART_QUEUE_SIZE];
uint16_t uart_front = 0;
uint16_t uart_rear = 0;



void TaskCollectRX(void *param) {

}

void TaskPrintRxQueue(void *param) {
  Serial.println("Taks!!!");
  
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  uart_Semaphore = xSemaphoreCreateBinary();


  // xTaskCreate(TaskPrintRxQueue, "TaskPrintRxQueue", 2048, NULL, 1, NULL);

  

}

void loop() {
  uint8_t data[8];
  // Tx
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n');
    mySerial.println(inputString);
  }

  // Rx
  if (mySerial.available() >= 8) {
    for(int i = 0; i < 8; i++) {
      data[i] = mySerial.read();
      Serial.print(data[i]);
      Serial.print(", ");
    }
    Serial.println();
    double val;
    memcpy(&val, data, sizeof(double));

    Serial.println(val, 12);
  }

  
  vTaskDelay(10);
}

// Rx
  // if (mySerial.available()) {
  //   String receivedString = mySerial.readStringUntil('\n');
  //   Serial.println(receivedString);
  // }