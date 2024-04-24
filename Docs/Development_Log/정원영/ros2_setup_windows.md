# ROS2 설치 (윈도우)
## 1. choco 설치
- https://chocolatey.org/install
- 위 사이트에서 설치 커맨드 복사 및 터미널에 붙여넣기
- 설치 후 choco 명령어로 설치 여부 확인
## 2. python 설치
- `choco install -y python --version 3.7.5`
- 설치 후 python 명령어를 이용해 설치 확인 
## 3. Visual C++ 빌드 툴 설치
- `choco install -y vcredist2013 vcredist140`
- visual studio installer에서 **C++를 사용한 데스크톱 개발** 설치
## 4. OPENSSL 설치
- https://slproweb.com/products/Win32OpenSSL.html
- 위 페이지에서 윈도우 용 Openssl 설치
- 이후 프롬포트에 해당 명령어 입력
  - `setx -m OPENSSL_CONF C:\(Openssl 설치 경로)\bin\openssl.cfg`
- PC 환경변수 설정에서 Path 변수에 해당 경로 추가
  - `C:\(Openssl 설치 경로)\bin`
## 5. OpenCV 설치
- 주어진 파일에서 opencv 설치 파일 실행 및 프롬포트에 아래 명령어 입력
- `setx -m OpenCV_DIR C:\opencv`
## 6. CMake 설치
- `choco install -y cmake.install -version==3.19.3`
- 프롬포트에 아래 명령어 입력
  - `C:\Program Files\CMake\bin`
