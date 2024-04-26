# 손잡이 개발 문서

작성자: 김영래

## 목록
### 1. 모듈 사용 예시 코드 및 핀 배치도
    - 1.1. IMU (MPU6050 / MPU 9250)
    - 1.2. UWB(DWM 1000)
    - 1.3. 서보모터 (MG996R)
    - 1.4. 블루투스 통신
    - 1.5. 버튼
    - 1.6. 부저
    - 1.7. 멀티 쓰레딩

### 2. UWB 질문














## 1. 모듈 사용 예시 코드 및 핀 배치도
### 1.1. IMU (MPU6050 / MPU 9250)
---
Interface: I2C

PIN: 

비고: 실제 사용은 MPU 9250이나 현재 없는 관계로 MPU 6050으로 대체하겠음

### 1.2. UWB(DWM 1000)
---
Interface: SPI

PIN: 

비고: python Library로 진행중이나 막힘이 있음
GPIO를 BCM으로(BOARD말고) 핀 세팅한건 SPI1.0 인데 코드는 SPI0.0을 기준으로 하고 있음.

이 부분의 오류점이 있어서 좀 더 시도해 보아야함.


### 1.3. 서보모터(MG996R)
---
Interface: PWM

PIN: VCC - 5V, GND - GND, PWM - 12(BCM 18)

<img src="./IMG_README/서보모터_사진.PNG" width=800xp>

[그림 1 - 서보모터 핀 배치]

비고: 


참고자료: https://blog.naver.com/chandong83/221850060160

코드: 
```python
import RPi.GPIO as GPIO
from time import sleep

servoPin          = 12
SERVO_MAX_DUTY    = 12
SERVO_MIN_DUTY    = 3

GPIO.setmode(GPIO.BOARD) # 핀 번호를 BCM으로 하고 싶다면 GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPin, GPIO.OUT)

servo = GPIO.PWM(servoPin, 50)
servo.start(0)


def setServoPos(degree):
  if degree > 180:
    degree = 180

  duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
  print("Degree: {} to {}(Duty)".format(degree, duty))

  servo.ChangeDutyCycle(duty)


if __name__ == "__main__":
  setServoPos(0)
  sleep(1)
  setServoPos(45)
  sleep(1)

  setServoPos(90)
  sleep(1)


  setServoPos(135)
  sleep(1)

  setServoPos(180)
  sleep(1)


  servo.stop()

  GPIO.cleanup()
```


### 1.4. 블루투스 통신
---
Interface: 없음

PIN: 없음

비고: Bluetooth 4.2 (BLE)


### 1.5. 버튼
Interface: GPIO

PIN: 


비고: 인터럽트 사용


### 1.6. 부저
---
Interface: PWM

PIN: GND - GND, PWM - 12(BCM 18)

<img src="./IMG_README/부저_사진.PNG" width=800px>

[그림 2 - 부저 사진]

비고: 

참고자료: https://rasino.tistory.com/entry/%E3%80%90%EB%9D%BC%EC%A6%88%EB%B2%A0%EB%A6%ACPi20%E3%80%91-%ED%94%BC%EC%97%90%EC%A1%B0-%EB%B6%80%EC%A0%80-%EC%82%AC%EC%9A%A9%ED%95%98%EA%B8%B0-with-%ED%8C%8C%EC%9D%B4%EC%8D%AC-%EC%BD%94%EB%93%9C

코드: 
```python
import RPi.GPIO as GPIO
import time


buzzer = 12
GPIO.setmode(GPIO.BOARD)
GPIO.setup(buzzer, GPIO.OUT)
GPIO.setwarnings(False)
pwm = GPIO.PWM(buzzer, 262)


if __name__ == "__main__":
    pwm.start(50.0)
    time.sleep(0.5)
    pwm.stop()

    GPIO.cleanup()
```


### 1.7. 멀티 쓰레딩
---
Interface: 없음

PIN: 없음

비고: 

예제코드1: task1 과 task2로 1, 2초마다 print하는 코드




<br><br>


예저코드2: task1을 수행하고 task2를 호출(join) 후 exit









## UWB 질문

현재 https://github.com/pedestrian0423/DW1000_Raspi_Python_library 에서 파이썬 라이브러리를 가져와서 라즈베리파이 4B 환경에서 테스트 중이다. 월요일부터 목요일까지 진행했으나 성공하지 못해서 모듈 개발자에게 질문을 할 내용을 작성하도록 하겠다.

작성된 내용은 ISSUE로 질문하고,
stack overflow에도 게시할 예정임.


__인삿말__
---
안녕하세요. 현재 "삼성 청년 SW 아카데미"라는 교육 프로그램에서 임베디드 과정을 수강 중인 학생입니다.

프로젝트에서 UWB를 통한 두 노드간 거리 측정을 위해 DWM1000을 사용하려 합니다.
(두 노드는 최종적으로는 ESP32와 Jetson Nano이나, 현재 테스트를 위해 라즈베리파이 4B로 진행 중입니다.)

본 라이브러리를 사용하려고 하는데 이슈가 생겨 질문 드립니다.



__현재 본인이 사용하는 환경__

```
Node 1

Raspberry Pi 4B
Python 3.9.2
Rasberry Pi OS (Legacy, 64-bit) release; 2024-03-12
DWM1000


Node 2

Jetson Nano model-B
Python 3.8.10
Ubuntu 20.04
DWM1000
```

Ubuntu 20.04 release; https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html

현재 두 노드 간 거리 측정을 위해 제공해주신 Example/DW1000RangingAnchor.py와 Example/DW1000RangingTAG.py를 사용하려 합니다.

__최종 목적__
---
DWM1000을 통한 두 노드간 거리 측정



__코드에서 궁금한 점__
---
제공해주신 라이브러리에서 궁금한 점이 2개 있습니다.

<img src='./IMG_README/quest/100.PNG'>

DW1000.py의 코드를 보면 SPI0,0을 쓰는 것으로 보입니다.

```python
def begin(self, irq, rst=None, bus=None, device=None):
  self._bus = 0
  self._device = 0
```

<img src='./IMG_README/quest/101.PNG'>
<img src='./IMG_README/quest/102.PNG'>

하지만 DW1000RangingAnchor.py의 코드를 보면 사용하는 SPI는 SPI1,0으로 보입니다. 

Q1. 제공해주신 코드에서 pin 연결은 "SPI0,0"인지 "SPI1,0"인지 궁금합니다.
```python
# SPI0,0
# DWM1000           RPi
# MOSI              19 (BCM 10)
# MISO              21 (BCM 9)
# CLK               23 (BCM 11)
# CSN               24 (BCM 8)


# SPI1,0
# DWM1000           RPi
# MOSI              38 (BCM 20)
# MISO              35 (BCM 19)
# CLK               40 (BCM 21)
# CSN               36 (BCM 16)
```

Q2. MISO pin을 irq pin으로 지정한게 맞습니까?
```python3
PIN_IRQ = 19
```
인데 BCM기준으로 19번은 SPI1의 MISO로 보입니다.

만약 MISO pin을 irq pin으로 지정한게 맞다면, DWM1000의 IRQ pin은 연결이 없습니까?

Q3. DW<1000의 RSTN pin은 사용하지 않습니까?
<img src='./IMG_README/quest/103.PNG'>
만약 사용한다면, DW1000RangingAnchor.py에서 

```python
# Before
DW1000.begin(PIN_IRQ)

# After
PIN_RST = 23 # 16 (BCM 23)

DW1000.begin(PIN_IRQ, PIN_RST) 
```
으로 변경하면 됩니까?

__현재까지의 진행__
---
__전처리__
---

<img src='./IMG_README/quest/01.PNG'>
[사진 1]

git clone을 완료하고 Example에 있는 예제 파일들을 "DW1000.py" 파일이 있는 디렉토리에 카피해옴.


<img src="./IMG_README/quest/02.PNG">
[사진 2]

```shell
$ python3 DW1000RangingAnchor.py
```

DW1000RangingAnchor.py를 실행함

[사진 2]와 같이 "DW1000.py"에서 함수 print에 ()가 없었음

<img src="./IMG_README/quest/03.PNG">
[사진 3]


<img src="./IMG_README/quest/04.PNG">
[사진 4]

[사진 3]의 Line:48과 [사진 4]의 Line:1263에서 ()가 없는 부분을 추가했습니다.

```python
# DW1000.py

# Line:48
# Before
print self.spi
# After
print (self.spi)

# Line:1263
# Before
print "DW1000.py 1263:\t ", data
# After
print ("DW1000.py 1263:\t ", data)
```


그리고 

모듈 monotonic 대신 time을 사용했습니다.

그래서 DW1000RangingAnchor.py와 DW1000RangingTAG.py 에서 변경이 있습니다.

```python
# Before
import monotonic
...
return int(round(monotonic.monotonic() * C.MILLISECONDS))

# After
import time
...
return int(round(time.monotonic() * C.MILLISECONDS))
```

혹시 몰라 monotonic을 설치했습니다. 
```shell
$ pip3 install monotonic
```
을 했습니다.


__DW1000RangingAnchor.py 편집__
---

<img src="./IMG_README/quest/05.PNG">
[사진 5]

```
$ python3 DW1000RangingAnchor.py
```
를 하면 [사진 5]와 같은 오류가 나온다.

DW1000RangingAnchor.py 를 살펴보았다.

<img src="./IMG_README/quest/06.PNG">
[사진 6]



<img src="./IMG_README/quest/07.PNG">
[사진 7]


<img src="./IMG_README/quest/08.PNG">
[사진 8]

인스턴스가 아닌 클래스 자체로 사용하고 있었다.

transmitPollAck(),
transmitRangeAcknowledge(),
transmitRangeFailed(),
receiver(),
computeRangeAsymetric(),
loop(),
try,
except 에서 DW1000.모듈() 방식으로 사용한다. 

<img src="./IMG_README/quest/09.PNG">

그래서 먼저 인스턴스로 선언하였다.

```python
dw1000 = DW1000.DW1000()
```


그리고

<img src='./IMG_README/quest/10.PNG'>
[사진 10]


transmitPollAck(),
transmitRangeAcknowledge(),
transmitRangeFailed(),
receiver(),
try,
except 도 바꾸었다.

```python
# Before
DW1000.modulename()

# After
dw1000.modulename()


# global 변수로 인스턴스 dw1000을 사용하는 각 module안에 추가했다.
global dw1000

```

__DW1000RangingTAG.py 수정__
---
DW1000RangingTAG.py를 DW1000RangingAnchor.py처럼 수정한다.
- dw1000 = DW1000.DW1000()
- DW1000.module() -> dw1000.module()
- global dw1000  (dw1000을 사용하는 모듈에 한하여)


__DW1000.py 수정__
---
이제 다시 DW1000RangingTAG.py와 DW1000RangingAnchor.py를 실행해보자

<img src="./IMG_README/quest/11.PNG">
[사진 11]
<img src="./IMG_README/quest/12.PNG">
[사진 12]

[사진 11]과 [사진 12]와 같은 오류가 나온다.

DW1000.py를 더 수정하겠다.

DW1000.py의 Line:84로 이동한다.

self._irq가 integer가 아닌 None이라 문제인듯 하다.

<img src='./IMG_README/quest/13.PNG'>
[사진 13]

[사진 13]과 같이 수정했다.

```shell
$ python3 DW1000RangingAnchor.py
```

를 해보자

<img src='./IMG_README/quest/14.PNG'>
[사진 14]

None이 맞다.

irq 핀을 직접 정수로 넣자. (irq핀이 MISO핀이 아니라는 가정하에 BCM 18로 지정했습니다.)

<img src='./IMG_README/quest/15.PNG'>

실행했습니다.

<img src='./IMG_README/quest/16.PNG'>

맨 아래 TypeError부터 해결했습니다.

<img src='./IMG_README/quest/17.PNG'>

```python
# Before
idx = bit / 8

# After
idx = bit // 8
```
몫과 나머지를 구하는 듯 하여
float가 반환되는 '/'연산자 대신 int가 반환되는 '//' 을 사용했습니다.

DW1000RangingAnchor.py와 DW1000RanginTAG.py를 실행한 결과 입니다.

<img src='./IMG_README/quest/18.PNG'>

<img src='./IMG_README/quest/19.PNG'>


__질문__
---
위 과정을 통해 [그림 18]과 [그림 19]까지 진행했습니다.

Q3. 현재 출력값이 올바른 출력입니까?

Device ID나 Unique ID와 같이 대부분이 0으로 되어있는데 

실제로는 어떤 값이 나와야 하고, 
아무런 출력이 사진 이외에 추가적인 출력이 없는게 맞는지 궁금합니다.


Q4. 거리 측정을 위해 사용하는 파일이 DW1000RangingAnchor.py와 DW1000RanginTAG.py 맞습니까?


Q5. 해당 라이브러리를 사용한 라즈베리파이(Or Jetson Nano Ubuntu20.04; Python3.8.10)와 ESP32의 거리 측정이 가능한가요? 

ESP32의 DWM1000 라이브러리는 https://github.com/F-Army/arduino-dw1000-ng 에서 가져왔습니다.