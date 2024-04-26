# 터틀봇3 시뮬레이터 세팅
## 환경 세팅
- Ubuntu 20.04 LTS Desktop 이상 (Windows WSL 환경)
- ROS2 Foxy
- 해당 문서는 권장 사향의 OS 및 ROS2의 설치 완료를 가정하고 기술되었다.

## 터틀봇 필수 패키지 설치
- 시뮬레이터 설치 전, 터틀봇의 필수 패키지를 설치한다.

### ROS2 패키지 Dependency 설치
#### 0. ROS setup 등록 확인
```(bash)
# bashrc에 등록되었다면...
source ~/.bashrc

# 또는
source /opt/ros/foxy/setup.bash

```
#### 1. Gazebo11 설치
```(bash)
sudo apt-get install ros-foxy-gazebo-*
```
#### 2. Cartographer 설치
```(bash)
sudo apt install ros-foxy-cartographer
sudo apt install ros-foxy-cartographer-ros
```

#### 3. Navigation 2 설치
```(bash)
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup
```

### 터틀봇3 패키지 설치
```(bash)
sudo apt install ros-foxy-dynamixel-sdk
sudo apt install ros-foxy-turtlebot3-msgs
sudo apt install ros-foxy-turtlebot3
```

### 환경 변수 설정
```(bash)
# 현재 팀에서 공통적으로 사용하고 있는 ROS 도메인 아이디
echo 'export ROS_DOMAIN_ID=51' >> ~/.bashrc

echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc

source ~/.bashrc
```

## 터틀봇 워크 스페이스 빌드
### 워크 스페이스 구성 및 빌드
```(bash)
mkdir ~/turtlebot3_ws
cd ~/turtlebot3_ws
mkdir src
cd src

git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd ~/turtlebot3_ws
colcon build --symlink-install
```

### Gazebo 세팅 및 사용
### bash 설정
```(bash)
source ~/turtlebot3_ws/install/setup.bash

# 직접 실행 또는 아래처럼 .bashrc 등록 및 사용을 할 수 있다.
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc

source ~/.bashrc
```
### Gazebo 실행
```(bash)
ros2 launch utrtlebot3_gazebo turtlebot3_world.launch.py
```

### (Optional) SLAM 구동
```(bash)
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
## 참고
- 다른 기능 사용 또는 세팅에 차이가 있다면 ROBOTIS 공식 문헌을 참고한다.
- [메뉴얼](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
