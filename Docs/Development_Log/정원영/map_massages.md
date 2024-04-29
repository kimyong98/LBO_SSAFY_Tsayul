# ROS MAP 메시지 형식
- ROS에서의 SLAM으로 생성된 맵을 통해 목적지 설정 및 자율 주행을 위해 map을 발행해서 사용할 필요가 있다.
- 이전에 생성한 A* (A Star) 알고리즘 코드가 ROS에서 제공하는 nav_msgs/OccupancyGrid 메시지 형태를 사용하므로 이를 분석했다.

## Topic: nav_msgs/OccupancyGrid

|타입|이름|상세정보|
|:---:|:---:|:---:|
|std_msgs/Header|header|맵 Frame Id 및 정보|
|nav_msgs/MapMetaData|info|맵 관련 데이터|
|int8[]|data|맵 occupancy(점유) 데이터|

## Topic: nav_msgs/MapMetaData

|타입|이름|상세정보|
|:---:|:---:|:---:|
|time|map_load_time|맵 리딩 시간|
|float32|resolution|맵 해상도|
|uint32|width|지도 너비|
|uint32|height|지도 높이|
|geometry_msgs/Pose|origin|지도 원점|
