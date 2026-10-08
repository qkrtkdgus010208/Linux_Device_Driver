# Jetson 개발 환경 확인 — kkt

## 확인한 내용
- 작업 브랜치: dev/kkt
- Jetson Linux: R36.5.2
- Ubuntu: 22.04.5 LTS
- Linux 커널: 5.15.199-tegra
- GCC: 11.4.0
- GNU Make: 4.3
- libgpiod 도구 및 개발 파일: 1.6.3
- GPIO: gpiochip0(164 lines), gpiochip1(32 lines)
- PWM 관련 장치: pwmchip0~4, 각각 채널 1개
- pwmchip4는 tachometer 경로에 연결됨
- 현재 커널의 build 경로 및 Makefile 존재 확인

## 아직 확인하지 않은 내용
- JetPack 패키지 버전
- 실제 사용할 GPIO 핀과 PWM 채널
- 팬 사양과 구동 회로
- EC11, LED Bar, 저항 어레이의 사양과 배선
- C 프로그램 및 커널 모듈의 실제 빌드
- 실제 하드웨어 동작
