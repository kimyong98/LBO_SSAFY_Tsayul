# VSLAM
## SLAM 이란?
Simultaneous Localization And Mapping의 약어.
- "내가 어디에 있지?"(Localization) + "내 주변에 뭐가 있지?"(Mapping) 를 동시에 풀고자 하는 방법

## SLAM 종류
### Bayes 필터 기반
- ex) EKF SLAM, Fast SLAM
1. 다음 2개의 입력값을 받는다.
    - 현재 위치에 대한 예측값
    = 랜드마크의 위치에 대한 측정값
2. 각각의 사전확률이 Likelihood와 결합
3. 로봇의 현재 위치와 랜드마크의 위치에 대한 사후 확률 업데이트

### Graph 구조를 이용한 최적화에 기반
- 궤적을 Graph로 표현하고 제약 조건을 만족하는 형태로 최적화하는 방법
    - Node: 위치
    - Edge: 두 노드 사이의 제약 조건 (회전과 이동)

### Feature based Method (or Indirect Method)
- 영상에서 특징점을 기반으로 카메라 궤적을 추적하고 매핑을 수행하는 방법
- ex) ORB-SLAM

### Direct Method
- 영상 간의 밝기(Intensity) 값 차이를 직접 이용해 카메라의 궤적을 추적하고 매핑을 수행하는 직접 기반 방법
- ex) LSD SLAM

### 딥러닝 기술
- 각 프레임마다 CNN을 기반으로 특징점을 추출하고 연속된 특징점을 RNN으로 매핑을 수행하는 방법
- ex) DeepVO
- 정확도는 낮지만, 비지도 혹은 반지도 학습을 적용하는 방안으로 연구가 되기도 함.


## SLAM Pipeline
- Frontend와 Backend로 구성됨.
![SLAM Pipeline](./image/SLAM_pipeline.png)