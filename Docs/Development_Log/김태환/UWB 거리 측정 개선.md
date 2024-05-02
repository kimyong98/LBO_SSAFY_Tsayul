### 거리 측정 개선

---

- 손잡이 UWB RANGE 신호 반응 없는 현상 수정
    - 손잡이 부분 코드 이상점 체크
    
- Rpi - Rpi 간 거리 측정 방식 확인
    - 옥텟 단위 저장한 값으로 distance값 재 연산 후 계산값 / 측정값 비교
    

### RANGE 이상 부분 체크

---

- 안내 손잡이(ESP32) = Anchor
    - Arduino 코드로 동작 (라이브러리 : cpp)
- 안내 로봇(Jetson Nano) = TAG
    - Python 코드로 동작 (라이브러리 : Python)

- 문제 상황

<aside>
💡 Anchor 모듈에서 RANGE (Data Header Index = 2) 신호를 수신 받지 못함

- POLL (Index = 0)인 신호는 잘 받음
- POLL 신호 받고 POLL_ACK 신호를 TAG로 전송하는 것 까지는 확인
- TAG에서는 POLL_ACK 신호를 받고 RANGE 신호를 전송함
</aside>

- 문제점 발생 예상 지점
    1. msgId ≠ expectedMsgId 문에서 protocolFailed 발생
    2. POLL_ACK 신호 보내면서 sentAck & receivedAck = false로 변경 → resetInactive → expectedMsgId = POLL로 변경
        1. POLL_ACK를 보내고 RANGE 신호가 들어올 때, expectedMsgId가 POLL로 변경되어 있어서 이상 발생
    
- 해결 방법
    - ESP32 RTOS의 Task 할당을 위한 UWB의 딜레이가 1s로 잡혀있었음 → 1ms로 전환하여 통신오류 해결
    

### 거리 측정 개선

---

- 측정 방식 : DS-TWR (Double Sided Two-Way Ranging)
- 현재 측정 상황
    - 실제 거리는 1.5 ~ 2.0m로 예상
    - 측정 거리는 2.0 ~ 5.0m 사이로 변동이 큼
    
- 해결 방안1 - Configuration 값 수정
    - 기존 값
    
    | SFD_MODE | Standard |
    | --- | --- |
    | Channel | 5 |
    | Data_Rate | 6.8Mbps |
    | Pulse_Frequency(PRF) | 16MHz |
    | Preamble_Length | 64 |
    | Preamble_code | 4 |
    - 변경 값
    
    | SFD_MODE | Standard |
    | --- | --- |
    | Channel | 5 |
    | Data_Rate | 110Kbps |
    | Pulse_Frequency(PRF) | 16MHz |
    | Preamble_Length | 256 |
    | Preamble_code | 4 |
    - Pulse_Frequnecy 64MHz로 변경 시도 → 통신 자체가 안됨 (원인 불명)
    - 값 변경 후 오차 폭 2.0 ~ 5.5m ⇒ 2.0 ~ 3.5m 소폭 향상
    
- 해결 방안2 - 측정 환경 개선
    - 안내 손잡이와 로봇 사이 장애물을 최대한 치운 상태에서 측정
        
        → 측정 오차 0.1m 수준으로 감소