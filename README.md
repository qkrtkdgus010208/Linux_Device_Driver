# NVIDIA Jetson Orin Nano Embedded Linux Device Driver Project

## 1. 프로젝트 개요

본 프로젝트는 **NVIDIA Jetson Orin Nano** (JetPack 5/6, Linux Kernel 5.10 / 5.15) 환경에서 동작하는 하드웨어 제어 디바이스 드라이버 시스템입니다.
EC11 로터리 엔코더(S1, S2, KEY), L298N(HW-095) DC 모터 드라이버, WCNLB8-SR12 8-Bar LED 어레이를 각각 독립된 캐릭터 디바이스 드라이버(커널 모듈)로 구동하고, 유저 공간 제어 데몬(`app/motor_daemon`)을 통해 실시간 연동 제어를 수행합니다.

### 주요 시스템 구성 및 기능
1. **EC11 로터리 엔코더 (`drivers/encoder/ec11_driver.ko`)**:
   - 핀 구성: `S1`(회전 A상), `S2`(회전 B상), `KEY`(푸시 스위치 버튼)
   - GPIO 하드웨어 하강 에지 인터럽트(`IRQF_TRIGGER_FALLING`) 기반 회전 및 클릭 감지
   - 커널 내부 시간차(`ktime`) 기반 채터링 방지(Debounce) 필터링
   - `poll()` / `epoll()` 대기 큐(`wait_queue`) 및 원형 이벤트 큐(`struct ec11_event`, 12바이트) 제공
2. **L298N DC 모터 드라이버 (`drivers/motor/l298n_driver.ko`)**:
   - Linux 커널 하드웨어 PWM API(`pwm_request`, `pwm_config`, `pwm_enable`) 기반 ENA 속도 제어
   - Jetson Orin Nano 하드웨어 PWM 컨트롤러(`32e0000.pwm` / `pwmchip3`, `pwm_id=3`, Pin 32) 연동
   - GPIO 기반 방향 제어(정회전 `FORWARD`, 역회전 `BACKWARD`, 정지 `STOP`, 급제동 `BRAKE`)
   - 0~100% 듀티비 및 0~8단계 속도 프로파일 매핑
3. **WCNLB8-SR12 8-Bar LED 어레이 (`drivers/led_bar/led_bar_driver.ko`)**:
   - 8개 전용 GPIO 라인 매핑
   - 0~8단계 실시간 레벨 미터 표시 및 8비트 비트마스크(RAW) 직접 점등 지원
4. **유저 공간 통합 제어 데몬 (`app/motor_daemon`)**:
   - `poll()` 비동기 I/O 이벤트 드리븐 구조 (CPU 점유율 최소화)
   - 노브 회전 -> 0~8단계 속도 연산 -> 모터 PWM 듀티비 조정 -> LED Bar 실시간 갱신
   - 노브 클릭 -> 모터 정회전/역회전 토글 (방향 전환 시 LED Bar 50ms 순간 점멸 피드백)
   - 터미널 실시간 ASCII 대시보드 UI 및 시그널(`SIGINT`/`SIGTERM`) 안전 종료(Graceful Shutdown)

---

## 2. 하드웨어 핀 맵 및 배선 가이드

### 2.1 EC11 로터리 엔코더 모듈 (실크 인쇄: 5V, GND, KEY, S2, S1)

| 엔코더 모듈 핀 | Jetson 40-Pin 헤더 | Tegra GPIO 포트 | Line Offset | 연결 설명 및 주의사항 |
|---|---|---|---|---|
| **5V (VCC)** | **Pin 1 (3.3V Power)** | 3.3V Rail | - | **⚠️ 필수 주의**: 5V(Pin 2/4) 금지! **반드시 Pin 1(3.3V)**에 연결 |
| **GND** | **Pin 6 (GND)** | GND | - | 공통 접지 (풀업 저항 동작 기준점) |
| **S1 (CLK)** | **Pin 11** | PR.04 | 112 | 회전 펄스 채널 A (Falling Edge IRQ) |
| **S2 (DT)** | **Pin 13** | PY.00 | 122 | 회전 펄스 채널 B (방향 판별 GPIO 입력) |
| **KEY (SW)** | **Pin 15** | PN.01 | 85 | 푸시 스위치 버튼 (Falling Edge IRQ) |

> **⚠️ 전원 및 풀업 저항 결선 주의사항**:
> 1. **3.3V 연결 필수**: Jetson Orin Nano의 GPIO는 **3.3V 전용 로직 레벨**입니다. 모듈에 `5V`라고 인쇄되어 있더라도 Jetson **Pin 2/4(5V)에 꽂으면 SoC 핀이 영구 손상**될 수 있습니다. 반드시 **Pin 1 (3.3V)**에 꽂아야 3.3V 풀업 저항이 안전하게 작동합니다.
> 2. **플로팅(Floating) 방지**: VCC(3.3V) 또는 GND가 빠지면 신호선이 공중에 떠서 초당 수십만 번의 가짜 인터럽트가 폭주합니다. 5가닥 모두 단단히 연결되어 있는지 확인하세요.
> 3. **PCB 모듈형 vs 단품 부품 구별**: 회전축 아래에 네모난 기판(PCB)과 5개 핀 헤더가 달린 모듈형 제품은 내부에 10kΩ 풀업 저항이 실장되어 있습니다. 만약 기판이 없는 단품 부품(다리 5개짜리 은색 알몸 부품)을 사용한다면 외부에 10kΩ 풀업 저항을 직접 달아주어야 합니다.

---

### 2.2 L298N DC 모터 드라이버 (HW-095)

| L298N 모터 드라이버 핀 | Jetson 40-Pin 헤더 / 전원 | Tegra 포트 | 기능 및 연결 설명 |
|---|---|---|---|
| **ENA** | **Pin 32** | PG.06 (Offset 41) | 하드웨어 PWM 속도 제어 (`32e0000.pwm` / `pwmchip3`, PWM7) |
| **IN1** | **Pin 29** | PQ.05 (Offset 105) | 모터 방향 제어 GPIO 출력 1 |
| **IN2** | **Pin 31** | PQ.06 (Offset 106) | 모터 방향 제어 GPIO 출력 2 |
| **+12V (VCC 단자)** | **외부 배터리/어댑터 (+)** | - | **⚠️ 외부 모터 전원(7.4V~12V) 필수!** (Jetson 전원 불가) |
| **GND (단자대)** | **외부 전원 (-) & Jetson Pin 9** | GND | **⚠️ 공통 접지 필수!** Jetson GND와 외부 전원 (-)를 함께 묶어 연결 |
| **OUT1 / OUT2** | **DC 모터 단자** | - | 구동할 DC 모터 양단 연결 |

> **⚠️ 모터 구동 필수 하드웨어 체크리스트**:
> 1. **외부 전원 필수**: L298N은 내부 트랜지스터 전압 강하(1.5V~2V)가 크므로 Jetson의 5V/3.3V 핀으로는 모터를 돌릴 수 없습니다. 나사 단자대 `+12V`에 외부 배터리(7.4V 이상) 또는 12V DC 어댑터를 연결하세요.
> 2. **공통 접지(Common GND)**: L298N의 나사 단자 `GND`에 **외부 전원 (-)극과 Jetson Pin 9 (GND)를 함께 연결**해야 Jetson의 제어 신호가 인식됩니다.
> 3. **ENA 점퍼 캡**: Pin 32로 PWM 속도를 제어하려면 ENA 핀의 **검은색 점퍼 캡을 분리**하고 Jetson Pin 32를 연결해야 합니다. (점퍼 캡을 꽂아두면 PWM과 무관하게 100% 최고 속도로 고정됩니다.)
> 4. **5V_EN 점퍼**: L298N 보드 뒷부분의 `5V_EN` 점퍼 캡이 장착되어 있어야 L298N 내부 로직 칩이 켜집니다.

---

### 2.3 WCNLB8-SR12 8-Bar LED 어레이

| LED Bar 핀 | Jetson 40-Pin 헤더 | Tegra GPIO 포트 | Line Offset | 기능 |
|---|---|---|---|---|
| **Bar 1** | **Pin 16** | PY.04 | 126 | GPIO 출력 (1단계) |
| **Bar 2** | **Pin 18** | PY.03 | 125 | GPIO 출력 (2단계) |
| **Bar 3** | **Pin 22** | PY.01 | 123 | GPIO 출력 (3단계) |
| **Bar 4** | **Pin 36** | PR.05 | 113 | GPIO 출력 (4단계) |
| **Bar 5** | **Pin 37** | PY.02 | 124 | GPIO 출력 (5단계) |
| **Bar 6** | **Pin 38** | PI.01 | 52 | GPIO 출력 (6단계) |
| **Bar 7** | **Pin 40** | PI.00 | 51 | GPIO 출력 (7단계) |
| **Bar 8** | **Pin 24** | PZ.06 | 136 | GPIO 출력 (8단계) |
| **GND** | **Pin 14** | GND | - | 공통 캐소드 접지 (330Ω 저항 연결 권장) |

---

## 3. Jetson-IO 40핀 확장 헤더 설정 (최초 1회 필수!)

Jetson Orin Nano는 기본 상태에서 핀들이 SPI, I2S, 클록(CLK) 등으로 예약되어 있어 일반 GPIO나 PWM 신호가 나가지 않습니다. 반드시 `jetson-io.py` 도구를 통해 핀 기능을 활성화해야 합니다.

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

### 설정 순서:
1. 메뉴에서 **`Configure Jetson 40pin Header`** 선택 $\rightarrow$ Enter
2. **`Configure header pins manually`** 선택 $\rightarrow$ Enter
3. 목록에서 스페이스바(Space)를 눌러 아래 핀 기능들을 정확하게 선택합니다:

| 설정 항목 (핀 번호) | 선택할 기능 | 용도 및 설명 |
|---|---|---|
| **`extperiph3_clk (29)`** | **`[*] gpio`** | L298N IN1 방향 제어 GPIO |
| **`extperiph4_clk (31)`** | **`[*] gpio`** | L298N IN2 방향 제어 GPIO |
| **`pwm7 (32)`** | **`[*] pwm7`** | L298N ENA 하드웨어 PWM 출력 |
| **`uarta-cts/rts (11,36)`** | **`[*] gpio`** | EC11 S1 (Pin 11) & LED Bar 4 (Pin 36) |
| **`spi3 (13,16,18,22,37)`** | **`[*] gpio`** | EC11 S2 (Pin 13) & LED Bar 1, 2, 3, 5 |
| **`pwm1 (15)`** | **`[*] gpio`** | EC11 KEY 스위치 (Pin 15) |
| **`i2s2 (12,35,38,40)`** | **`[*] gpio`** | LED Bar 6, 7 (Pin 38, 40) |
| **`spi1 (19,21,23,24,26)`** | **`[*] gpio`** | LED Bar 8 (Pin 24) |

4. **`Back`** 선택 $\rightarrow$ **`Save and reboot to apply changes`** 선택하여 저장 후 재부팅합니다.

---

## 4. 디렉터리 구조

```
/home/aidl/Linux_Device_Driver/
├── drivers/
│   ├── encoder/
│   │   ├── Makefile            (EC11 커널 모듈 빌드 설정)
│   │   └── ec11_driver.c       (EC11 S1/S2/KEY 인터럽트 드라이버)
│   ├── motor/
│   │   ├── Makefile            (L298N 커널 모듈 빌드 설정)
│   │   └── l298n_driver.c      (L298N 하드웨어 PWM 및 GPIO 드라이버)
│   ├── led_bar/
│   │   ├── Makefile            (8-Bar LED 커널 모듈 빌드 설정)
│   │   └── led_bar_driver.c    (8-Bar LED 출력 드라이버)
│   └── Makefile                (드라이버 하위 모듈 통합 빌드)
├── app/
│   ├── Makefile                (유저 데몬 빌드 설정)
│   └── main.c                  (이벤트 감지 및 통합 제어 데몬)
├── include/
│   └── common_ioctl.h          (공통 IOCTL 명령 및 구조체 헤더)
├── Makefile                    (전체 프로젝트 통합 Makefile)
├── pin.png                     (Jetson-IO 핀 설정 참고 이미지)
└── README.md                   (프로젝트 문서)
```

---

## 5. 빌드 방법

프로젝트 루트 디렉터리에서 `make`를 실행합니다:

```bash
cd /home/aidl/Linux_Device_Driver
make
```

빌드가 완료되면 다음 파일들이 생성됩니다:
- `drivers/encoder/ec11_driver.ko`
- `drivers/motor/l298n_driver.ko`
- `drivers/led_bar/led_bar_driver.ko`
- `app/motor_daemon`

클린 빌드가 필요한 경우:
```bash
make clean
```

---

## 6. 모듈 적재 및 디바이스 노드 생성

### 자동 적재 (`make load`)
```bash
sudo make load
```

### 수동 적재 (`insmod`)
필요에 따라 모듈 파라미터를 지정하여 적재할 수 있습니다:

```bash
# 1. EC11 엔코더 드라이버 적재 (채터링 방지 권장값 debounce_ms=15)
sudo insmod drivers/encoder/ec11_driver.ko debounce_ms=15 key_debounce_ms=80

# 2. L298N 모터 드라이버 적재 (Pin 32 = pwmchip3 -> pwm_id=3)
sudo insmod drivers/motor/l298n_driver.ko pwm_id=3

# 3. 8-Bar LED 드라이버 적재
sudo insmod drivers/led_bar/led_bar_driver.ko

# 4. 디바이스 노드 일반 사용자 접근 권한 부여
sudo chmod 666 /dev/ec11 /dev/l298n_motor /dev/led_bar
```

### 모듈 적재 상태 확인 (`make status`)
```bash
make status
```
또는
```bash
lsmod | grep -E "ec11_driver|l298n_driver|led_bar_driver"
ls -l /dev/ec11 /dev/l298n_motor /dev/led_bar
```

정상 생성 시 다음과 같은 캐릭터 디바이스 노드가 표시됩니다:
- `/dev/ec11`
- `/dev/l298n_motor`
- `/dev/led_bar`

---

## 7. 개별 디바이스 드라이버 단위 테스트

### 7.1 WCNLB8-SR12 LED Bar 드라이버 테스트
```bash
# 레벨 미터 테스트 (0 ~ 8 단계 점등)
echo 3 > /dev/led_bar   # Bar 1 ~ 3 점등
echo 8 > /dev/led_bar   # Bar 1 ~ 8 전체 점등
echo 0 > /dev/led_bar   # 전체 소등

# 현재 점등 상태 확인
cat /dev/led_bar
```

### 7.2 L298N 모터 드라이버 테스트
```bash
# 정회전 설정
echo "forward" > /dev/l298n_motor

# 속도 설정 (0 ~ 100%)
echo 50 > /dev/l298n_motor

# 역회전 설정
echo "backward" > /dev/l298n_motor

# 모터 정지
echo "stop" > /dev/l298n_motor

# 현재 모터 상태 확인
cat /dev/l298n_motor
```

### 7.3 EC11 엔코더 이벤트 확인
```bash
# 엔코더 노브를 돌리거나 누를 때 12바이트 이벤트가 1줄씩 찍히는지 확인
hexdump -C /dev/ec11
```
* **정상**: 손을 대지 않았을 때는 아무것도 출력되지 않고, 노브를 1칸 돌리거나 버튼을 누를 때만 1줄씩 패킷이 출력됩니다.
* **이상**: 손을 안 댔는데도 글자가 폭포수처럼 쏟아진다면 3.3V/GND 결선 불량(플로팅) 상태입니다.

---

## 8. 통합 제어 애플리케이션 실행

통합 제어 데몬을 실행하면 실시간 터미널 대시보드가 표시되며 하드웨어가 완벽히 연동됩니다.

```bash
./app/motor_daemon
```

### 조작 방법
- **엔코더 시계 방향 회전 (CW)**:
  - 속도 단계 1단계 증가 (최대 8단계, PWM 듀티비 100%)
  - LED Bar 점등 개수 1개씩 추가
  - 모터 가속
- **엔코더 반시계 방향 회전 (CCW)**:
  - 속도 단계 1단계 감소 (최소 0단계, 모터 정지)
  - LED Bar 점등 개수 1개씩 감소
  - 모터 감속 및 0단계 시 정지
- **엔코더 누름 스위치(KEY) 클릭**:
  - 모터 회전 방향 토글 (`FORWARD` <-> `BACKWARD`)
  - 방향 전환 시 LED Bar 전체가 50ms 동안 순간 점멸하여 피드백 제공
- **안전 종료 (`Ctrl + C`)**:
  - 모터 즉시 정지 및 PWM 듀티비 0% 리셋
  - LED Bar 전체 소등
  - 파일 디스크립터 안전 종료 후 클린 셧다운

---

## 9. 문제 해결 및 자주 묻는 질문 (FAQ / Troubleshooting)

### Q1. 데몬 실행 시 `Speed Step : [ 8 / 8 ]` 값이 계속 제멋대로 바뀌거나, `hexdump -C /dev/ec11`에 데이터가 멈추지 않고 쏟아져 나옵니다.
* **원인**: **신호선 플로팅(Floating) 현상**입니다. EC11 모듈의 VCC(3.3V) 및 GND 선이 연결되지 않았거나 헐거워서 풀업 저항이 작동하지 않아 노이즈 인터럽트가 폭주한 것입니다.
* **해결책**:
  1. EC11의 `5V` 핀을 Jetson **Pin 1 (3.3V)**에, `GND` 핀을 **Pin 6 (GND)**에 단단히 꽂아주세요. (⚠️ Pin 2 5V에 꽂으면 안 됩니다!)
  2. 빵판이나 듀퐁 점퍼선의 접촉 상태를 확인하세요.
  3. 접점 채터링이 심한 경우 `sudo insmod drivers/encoder/ec11_driver.ko debounce_ms=15`와 같이 디바운스 필터 시간을 늘려 로드하세요.

### Q2. `cat /dev/l298n_motor`에는 정상 출력되는데 모터가 전혀 돌지 않습니다.
* **원인**:
  1. Jetson-IO에서 Pin 29, 31, 32 핀 기능이 `gpio` 및 `pwm7`로 활성화되지 않은 경우
  2. L298N에 외부 모터 전원(12V)이 연결되지 않았거나, Jetson GND와 외부 전원 GND가 공통 접지되지 않은 경우
  3. L298N 보드의 ENA 핀에 점퍼 캡이 빠진 상태에서 Pin 32 연결이 불량한 경우
* **해결책**:
  1. **L298N 10초 직결 테스트**: L298N의 ENA에 점퍼 캡을 꽂고, IN1을 5V에, IN2를 GND에 직접 연결해 봅니다. 모터가 돌지 않으면 외부 12V 전원 또는 모터 단자 결선 문제입니다.
  2. [3. Jetson-IO 설정](#3-jetson-io-40핀-확장-헤더-설정-최초-1회-필수)을 참고하여 Pin 29/31을 `gpio`로, Pin 32를 `pwm7`로 설정하고 재부팅했는지 확인하세요.
  3. 드라이버가 Pin 32용 PWM ID `3`으로 정상 로드되었는지 확인하세요 (`sudo insmod drivers/motor/l298n_driver.ko pwm_id=3`).

---

## 10. 모듈 언로드 및 정리

테스트 완료 후 커널 모듈을 언로드합니다:

```bash
sudo make unload
```

또는 수동 제거:
```bash
sudo rmmod led_bar_driver
sudo rmmod l298n_driver
sudo rmmod ec11_driver
```