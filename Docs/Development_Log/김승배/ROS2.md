# ROS2

---

## 터틀봇 자율주행 로직

### 1. Map

- 2D Lidar 기반 SLAM 활용
- 실내를 2차원 grid로 변환하여 초기 50(미탐색) 값을 lidar 레이저가 지나간 공간은 --, 반사된 공간은 ++ 하면서 업데이트
- 최소 0(빈 공간), 최대 100(장애물)

### 2. Odometry

- IMU 센서로 가속도 값 적분하면서 위치 업데이트
- 누적 오차 때문에 추가적인 보정 필요(ex - lidar, UWB 활용)

### 3. Path

- dijkstar or A_star 사용하여 global path 생성
- 현재 위치 추정 후 global path에서 local path 생성

### 4. Follow

- local path 기반 경로 추종
- 현재 위치와 다음 포인트 간의 거리, 각도 계산
- 필요한 각속도 값에 따라 선속도/각속도 값 보정 제어
