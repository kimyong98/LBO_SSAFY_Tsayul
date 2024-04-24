# Ubuntu에 ROS2 Humble 설치
## 1. locale 설정
- locale이 UFT-8을 지원할 필요가 있다.
- locale 명령어를 통해 UTF-8 지원을 확인하고, 지원하지 않는다면 변경해준다.

```(bash)
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

## 2. Source 설정
- Ubuntu Universe repository를 활성화한다.
```(bash)
sudo apt install software-properties-common
sudo add-apt-repository universe
```
- apt의 ROS 2 GPG key 추가한다.
```(bash)
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
- source 리스트에 repository를 추가한다.
```(bash)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
- apt repository 캐시를 업데이트한다.
```(bash)
sudo apt update
sudo apt upgrade
```

## 3. ROS 설치
- 옵션1: ROS 데스크탑 설치
```(bash)
sudo apt install ros-humble-desktop
```
- 옵션2: ROS-Base 설치
```(bash)
sudo apt install ros-humble-ros-base
```
- 개발툴 설치
```(bash)
sudo apt install ros-dev-tools
```

## 4. 설정 스크립트 수행
- 설정 스크립트
```(bash)
source /opt/ros/humble/setup.bash
```

## 5. 예제 테스트
- talker와 listener를 각각의 다른 터미널에서 실행해서 결과를 확인한다
- talker 실행
```(bash)
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```
- listener 실행
```(bash)
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```