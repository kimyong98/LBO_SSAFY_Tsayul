#include "BluetoothSerial.h"
#include "esp_bt_device.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define QUEUE_MAX_SIZE 256
const TickType_t loopDelay = 1000000;

BluetoothSerial SerialBT;

SemaphoreHandle_t xSemaphore = NULL;

char Que[QUEUE_MAX_SIZE];
volatile uint8_t rear = 0U;
volatile uint8_t front = 0U;
bool is_created = 0;

// function for address, Mac address must be equal value at addr of RPI code
void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
  
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);
    
    if (i < 5){
      Serial.print(":");
    }
  }
}




void Task_Print_Message_Queue(void *pvParameters) {
  for(;;){
    if (rear != front) {
      for(int i = front; i != rear; i++) {
        Serial.print(Que[i]);
      }
      Serial.println();
      rear = 0U;
      front = 0U;
    }
    vTaskDelay(100);
  }
  // vTaskDelete(NULL);
}



// void IRAM_ATTR rxInterrupt()
// esp_spp_cb_event_t event, esp_spp_cb_param_t *param
void rxInterrupt(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_DATA_IND_EVT) { 
    while(SerialBT.available()) {
      Que[rear] = SerialBT.read(); // 데이터를 읽음
      rear++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n---Start---");
  SerialBT.begin("ESP32test"); //Bluetooth device name
  
  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial.println("Device Name: ESP32test");
  Serial.print("BT MAC: ");
  printDeviceAddress();
  Serial.println();
  
 
  // attachInterrupt(digitalPinToInterrupt(1), rxInterrupt, RISING);
  SerialBT.register_callback(rxInterrupt);

  xTaskCreate(Task_Print_Message_Queue, "Task_Print_Message_Queue", 1024, NULL, 1, NULL);
}

void loop() {
  vTaskDelay(1000);
}