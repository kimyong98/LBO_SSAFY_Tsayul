# eSpeak - simple tts

## 1. eSpeak 설치
```
sudo apt-get install espeak
```

## 2. TTS 실행
```(bash)
# 한국어 실행
espeak -v ko "안녕하세요"

# 영어 실행
espeak "Hello"
```

## 3. 옵션
```(bash)
espeak -s 160 -p 95 -a 200 -v ko+f3 "안녕하세요"
```

- s 옵션: 분당 160 단어의 속도. 기본 160, 범위 80~260
- p 옵션: pitch. 기본 50, 범위 0~99
- a 옵션: 볼륨. 기본 100, 범위 0~200
- v 옵션: 목소리 옵션. ko+f3 => 한국어(ko)의 3번째 목소리(f3)

- f 옵션: 파일명을 붙여서 텍스트 파일을 읽음

## 4. Python 예제 코드

```(python)
import os

def speak(option, msg) :
    os.system("espeak {} '{}'".format(option,msg))
    
option = '-s 160 -p 95 -a 200 -v ko+f3'
msg = '안녕하세요'

print('espeak', option, msg)
speak(option,msg)
```
