# ROS2

---

## ROS2 폴더 구조

```bash
├── src
│   ├── pkg1
│   │   └── ...
│   ├── pkg2
│   │   └── ...
│   └── pkg3
│       ├── launch
│       │   └── test_launch.py
│       ├── resource
│       ├── test
│       ├── pkg3
│       │   │── map.py
│       │   └── path.py
│       ├── etc
│       └── setup.py
```

---

## ROS2 기본 node 구조

```
import rclpy
from rclpy.node import Node

class driving(Node):

  def __init__(self):
    super().__init__('driving')


def main(args = None):
  rclpy.init(args=args)
  drive_test = driving()
  rclpy.spin(drive_test)
  drive_test.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
```

- rclpy : ROS Client Library for the Python language

---

## Topic 발행/구독, Timer 예시

```
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Twist
from sensor_msgs.msg import LaserScan, PointCloud
from math import pi,cos,sin

class driving(Node):

  def __init__(self):
    super().__init__('driving')
    self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
    self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    self.timer = self.create_timer(0.1, self.timer_callback)

    self.cmd_msg = Twist()

    self.is_collision = False
    self.handle_direction = False

  def check_collision(self, msg):
    for angle,r in enumerate(msg.ranges):

      if 0 <= angle < 10 or angle > 350:
        if 0.0 < r < 1.5:
          return True
    return False

  def lidar_callback(self, msg):
    self.is_collision = self.check_collision(msg)

    tempL = 0
    tempR = 0
    for angle,r in enumerate(msg.ranges):
      if 88 <= angle <= 92:
        tempL += r
      elif 268 <= angle <= 272:
        tempR += r

    if tempL <= tempR:
      self.handle_direction = True
    else:
      self.handle_direction = False

  def timer_callback(self):

    if self.is_collision:
      self.cmd_msg.linear.x = 0.0

      if self.handle_direction:
        self.cmd_msg.angular.z = 0.2
      else:
        self.cmd_msg.angular.z = -0.2
    else:
      self.cmd_msg.linear.x = 1.0
      self.cmd_msg.angular.z = 0.0

    self.cmd_publisher.publish(self.cmd_msg)


def main(args = None):
  rclpy.init(args=args)
  drive_test = driving()
  rclpy.spin(drive_test)
  drive_test.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
```

- create_publisher(msg 타입, topic 이름, queue 갯수) : 원하는 topic에 원하는 형식의 데이터 발행 용도
- create_subscription(msg 타입, topic 이름, 콜백 함수, queue 갯수) : 원하는 topic 구독 대기 / 해당 topic에 데이터 발행됐을 경우 콜백 함수 실행(콜백 함수 정의 필수)
- create_timer(타이머 주기(second), 콜백 함수) : 일정 주기마다 콜백 함수 실행시키는 타이머 생성

---

## setup 파일

```
from setuptools import setup

package_name = 'test_209'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SSAFY',
    maintainer_email='ksshc@snu.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'name1 = pkg3.map:main',
            'name2 = pkg3.path:main',
        ],
    },
)
```

- console_scripts 배열 안에 패키지, node 이름으로 node 추가

---

## launch 파일

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'pkg3',
            node_executable = 'name1',
            node_name = 'name1',
            output = 'screen',
        ),
        Node(
            package = 'pkg3',
            node_executable = 'name2',
            node_name = 'name2',
            output = 'screen',
        ),
    ])
```

- 런치 파일 실행시 실행하고 싶은 Node 추가
- node_executable, node_name 은 setup.py에서 설정한 이름
- output 옵션은 런치 파일 실행시에도 터미널에서 print 값 찍히도록 하는 옵션

---

## 터틀봇 자율주행 로직

### 1. Map

사전 세팅 단계

- 2D Lidar 기반 SLAM 활용
- 실내를 2차원 grid로 변환하여 초기 50(미탐색) 값을 lidar 레이저가 지나간 공간은 --, 반사된 공간은 ++ 하면서 업데이트
- 최소 0(빈 공간), 최대 100(장애물)

### 2. Odometry

항상

- IMU 센서로 가속도 값 적분하면서 위치 업데이트
- 누적 오차 때문에 추가적인 보정 필요(ex - lidar, UWB 활용)

### 3. Path

서비스 단계 - 기능 ON

- dijkstar or A_star 사용하여 global path 생성
- 현재 위치 추정 후 global path에서 local path 생성

### 4. Follow

서비스 단계 - Path 생성 이후

- local path 기반 경로 추종
- 현재 위치와 다음 포인트 간의 거리, 각도 계산
- 필요한 각속도 값에 따라 선속도/각속도 값 보정 제어
