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

void TaskResetButtonState(void *param) {
  for(;;) {
    if( (int)xTaskGetTickCount() - buttonLastTime > 2000 ) {
      buttonState = 0;
      Serial.println("Reset");
    }

    vTaskDelay(1200);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Button Start~~!");


  pinMode(buttonPin0, INPUT_PULLUP);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(buttonPin0), buttonISR0, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin1), buttonISR1, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin2), buttonISR2, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin3), buttonISR3, FALLING);

  xTaskCreate(TaskResetButtonState, "TaskResetButtonState", 1024, NULL, 20, NULL);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("button: ");
  Serial.println(buttonState);

  Serial.print("button0: ");
  Serial.print(buttonLastTime0);
  Serial.print(", ");
  Serial.print("button1: ");
  Serial.print(buttonLastTime1);
  Serial.print(", ");
  Serial.print("button2: ");
  Serial.print(buttonLastTime2);
  Serial.print(", ");
  Serial.print("button3: ");
  Serial.println(buttonLastTime3);
  Serial.println();

  vTaskDelay(300);
}