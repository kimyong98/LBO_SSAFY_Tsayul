// #include "TwoWayRangingResponder.h"
#include "blooth_lib.h"
#include "driver/gpio.h"
#include "MPU9250.h"

#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>

#include "ESP32Servo.h"

#include <HardwareSerial.h>










// semaphore 
SemaphoreHandle_t imu_semaphore_b;
SemaphoreHandle_t uwb_semaphore_b;

// ---------- beep (start) ----------
#define BUZZER_PIN 25
// ---------- beep (end) -----------


// ---------- Uart_UWB (start) ----------
#define RX_PIN 16
#define TX_PIN 17
HardwareSerial mySerial(2);
double distance;
// ---------- Uart_UWB (end) ----------




// ---------- servo value (start) ---------------
#define SERVO_PIN 27
Servo myservo;
volatile int angle = 90;
// ---------- servo value (end) -----------------




// ---------- button value (start) --------------
#define CHATTER_TIME 2000

const int buttonPin0 = 32;
const int buttonPin1 = 33;
const int buttonPin2 = 25;
const int buttonPin3 = 26;

volatile int buttonState = 0;

int buttonLastTime = -213456;

int buttonLastTime0 = -213456;
int buttonLastTime1 = -213456;
int buttonLastTime2 = -213456;
int buttonLastTime3 = -213456;
// ---------- button value (end) -----------------



// --------- MPU value (Start) ---------
IMU_Return_t imu_data;

MPU9250 mpu;
// IMU sequence 
// MPU_init() -> 

void getIMU(); // Task: Update IMU data (imu_data)
void update_quaternion(); // update quanternion
void Mpu_init(); // Init
void update_acc(); // update acc
// --------- MPU value (End) ---------



// -------- MPU function(start) -----------
void Mpu_init() {
  Wire.begin();
  delay(100);

  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(1000);
      }
  }

  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.verbose(true);
  mpu.calibrateAccelGyro();
  delay(1000);
  mpu.calibrateMag();
  mpu.verbose(false);
}



void getIMU() {
  if (mpu.update()) 
    update_quaternion();

  vTaskDelay(15);

  //delay(10);
  if (mpu.update()) 
    update_quaternion();

  update_acc();
}

void update_quaternion() {
  imu_data.gyro_x = mpu.getQuaternionX();
  imu_data.gyro_y = mpu.getQuaternionY();
  imu_data.gyro_z = mpu.getQuaternionZ();
  imu_data.gyro_w = mpu.getQuaternionW();
}

void update_acc() {
  imu_data.acc_x = mpu.getAccX(); // * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY
  imu_data.acc_y = mpu.getAccY();
  imu_data.acc_z = mpu.getAccZ();
}
// -------- MPU function(End) -----------




// -------- button function(start) -------
void IRAM_ATTR buttonISR0();
void IRAM_ATTR buttonISR1();
void IRAM_ATTR buttonISR2();
void IRAM_ATTR buttonISR3();

void Button_Init() {
  pinMode(buttonPin0, INPUT_PULLUP);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(buttonPin0), buttonISR0, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin1), buttonISR1, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin2), buttonISR2, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin3), buttonISR3, FALLING);
}


void IRAM_ATTR buttonISR0() { // cancle
  int nowTime = (int)xTaskGetTickCount();
  if(nowTime - CHATTER_TIME > buttonLastTime0) {
    Serial.println("Pushed: -1");
    buttonState = -1;
    buttonLastTime0 = nowTime;
    buttonLastTime = nowTime;
  }
}

void IRAM_ATTR buttonISR1() {
  int nowTime = (int)xTaskGetTickCount();
  if(nowTime - CHATTER_TIME > buttonLastTime1) {
    Serial.println("Pushed: 1");
    buttonState = 1;
    buttonLastTime1 = nowTime;
    buttonLastTime = nowTime;
  }
}

void IRAM_ATTR buttonISR2() {
  int nowTime = (int)xTaskGetTickCount();
  if(nowTime - CHATTER_TIME > buttonLastTime2) {
    Serial.println("Pushed: 2");
    buttonState = 2;
    buttonLastTime2 = nowTime;
    buttonLastTime = nowTime;
  }
}

void IRAM_ATTR buttonISR3() {
  int nowTime = (int)xTaskGetTickCount();
  if(nowTime - CHATTER_TIME > buttonLastTime3) {
    Serial.println("Pushed: 3");
    buttonState = 3;
    buttonLastTime3 = nowTime;
    buttonLastTime = nowTime;
  }
}
// ---------- button function(end) --------



// --------- beep function (start) ----------
void beep_Init() {
  gpio_pad_select_gpio(34);
  gpio_set_direction(gpio_num_t(BUZZER_PIN), gpio_mode_t(2));
}

void beep_func1(){
  Serial.println("beep_func1");
  gpio_set_level(gpio_num_t(BUZZER_PIN), 1);
  vTaskDelay(100);
  gpio_set_level(gpio_num_t(BUZZER_PIN), 0);
}

void beep_func2(){
  Serial.println("beep_func2");
  for (int i = 0; i < 3; i++) {
    gpio_set_level(gpio_num_t(BUZZER_PIN), 1);
    vTaskDelay(100); 
    gpio_set_level(gpio_num_t(BUZZER_PIN), 0);
    vTaskDelay(300);
  }
}

void beep_func3(){
  Serial.println("beep_func3");
  gpio_set_level(gpio_num_t(BUZZER_PIN), 1);
  vTaskDelay(1000);
  gpio_set_level(gpio_num_t(BUZZER_PIN), 0);
}
// --------- beep function (end) ----------




// --------- Servo function (start) ----------
void Servo_Init() {
  myservo.setPeriodHertz(50);// Standard 50hz servo
  myservo.attach(SERVO_PIN, 500, 2400);
  myservo.write(90);
}
// --------- Servo function (end) ----------



//---------------------- Task (start) -------------------
void TaskGetIMU(void *param) {
  for(;;) {
    // if(xSemaphoreTake(imu_semaphore_b, portMAX_DELAY) == pdTRUE) {
    getIMU(); // update IMU data

    // xSemaphoreGive(uwb_semaphore_b);
    
    // Serial.println(imu_data.acc_x);
    vTaskDelay(500);
    // }
  }
}

void TaskBeep(void *param) {
  for(;;) {
    // We use extern value "rx_beep" in File: "blooth_lib.h" 
    if(rx_beep == 0) {
      ;
    }
    else if(rx_beep == 1) {
      beep_func1();
    }
    else if(rx_beep == 2) {
      beep_func2();
    }
    else if(rx_beep == 3) {
      beep_func3();
    }
    else {
      // Serial.printf("?? why rx_beep is over 4 !!!!");
      ;
    }

    rx_beep = 0;

    vTaskDelay(1000);
  }
}




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



void TaskServoMotorSet(void *param) {
  for(;;) {
    myservo.write(angle);

    vTaskDelay(500);
  }
}


void TaskResetButtonState(void *param) {
  for(;;) {
    if( (int)xTaskGetTickCount() - buttonLastTime > 2000 ) {
      buttonState = 0;
      Serial.println("Reset");
    }

    vTaskDelay(1200);
  }
}
// --------------------- Task (end) --------------


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Start");
  // put your setup code here, to run once:

  // Uart_UWB
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  xTaskCreate(TaskMakeDistance, "TaskMakeDistance", 2048, NULL, 1, NULL);

  // Button
  Button_Init();
  xTaskCreate(TaskResetButtonState, "TaskResetButtonState", 1024, NULL, 20, NULL);

  // IMU = MPU
  Mpu_init();
  delay(500);

  // UWB = Distance

  // bluetooth
  blooth_lib_setup();
  delay(500);

  // Servo
  Servo_Init();
  xTaskCreate(TaskServoMotorSet, "TaskServoMotorSet", 1024, NULL, 12, NULL);


  // Buzzer
  beep_Init();

  xTaskCreate(TaskBeep, "TaskBeep", 1024, NULL, 10, NULL);

  xTaskCreate(TaskGetIMU, "TaskGetIMU", 2048, NULL, 1, NULL);


  // xSemaphoreGive(imu_semaphore_b);
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(10000);
}


