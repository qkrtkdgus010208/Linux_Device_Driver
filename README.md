# NVIDIA Jetson Orin Nano Embedded Linux Device Driver Project

## 1. 프로젝트 개요 및 시스템 아키텍처

본 프로젝트는 **NVIDIA Jetson Orin Nano** (JetPack 5/6, Linux Kernel 5.10 / 5.15) 환경에서 동작하는 임베디드 리눅스 캐릭터 디바이스 드라이버 시스템입니다.
EC11 로터리 엔코더, L298N(HW-095) DC 모터 드라이버, WCNLB8-SR12 8-Bar LED 어레이를 각각 독립된 커널 모듈로 제어하고, 유저 공간 통합 제어 데몬(`app/motor_daemon`)을 통해 실시간 연동을 수행합니다.

### 1.1 전체 시스템 아키텍처

```
[ 하드웨어 계층 ]
  EC11 Rotary Encoder (S1, S2, KEY)
      │ (Hardware Edge IRQ)
      ▼
[ 커널 드라이버 계층 ]
  drivers/encoder/ec11_driver.ko   ──> /dev/ec11 (Quadrant FSM & WaitQueue)
      │
      ▼
[ 유저 공간 제어 데몬 ]
  app/motor_daemon (poll() 비동기 이벤트 루프)
  - 노브 회전 감지 -> 0~8단계 속도 프로파일 매핑
  - 푸시 스위치 클릭 -> 모터 정/역회전 토글 (350ms 쿨다운)
      │
      ├───────────────────────────────────────────┐
      ▼                                           ▼
[ 커널 드라이버 계층 ]                     [ 커널 드라이버 계층 ]
  drivers/motor/l298n_driver.ko              drivers/led_bar/led_bar_driver.ko
  ──> /dev/l298n_motor (PWM + GPIO)          ──> /dev/led_bar (8-Line GPIO)
      │                                           │
      ▼                                           ▼
[ 하드웨어 액추에이터 / 인디케이터 ]       [ 하드웨어 액추에이터 / 인디케이터 ]
  L298N DC 모터 (ENA PWM, IN1/IN2)           WCNLB8-SR12 8-Bar LED 레벨 미터
```

### 1.2 주요 모듈별 특성

1. **EC11 로터리 엔코더 드라이버 (`drivers/encoder/ec11_driver.ko`)**:
   - **쿼드러처 상태 머신 (Quadrature FSM)**: 16-상태 전이 룩업 테이블(`quad_table`)을 적용하여 접점 바운스를 자연 배제.
   - **디텐트 탈출 잠금 해제 (Exit-Detent Unlatch)**: 클릭 홈(Detent)을 벗어날 때 래치를 해제하고 임계값 2에서 즉각 반응하여 역방향 회전 시 딜레이(2칸 넘김 현상) 및 중복 펄스를 원천 차단.
   - **비동기 I/O 지원**: 커널 원형 큐(`struct ec11_event`, 64 슬롯) 및 `wait_queue` 기반 `poll()` / `epoll()` 대기 큐 제공.
   - **푸시 스위치 (KEY)**: 하강 에지 인터럽트와 150ms 디바운스 필터링으로 노브 조작 진동에 의한 오작동 방지.

2. **L298N DC 모터 드라이버 (`drivers/motor/l298n_driver.ko`)**:
   - **하드웨어 PWM API**: Linux 커널 `pwm_request`, `pwm_config`, `pwm_enable` API를 사용하여 Tegra PWM 컨트롤러(Pin 32, `pwmchip3`) 직접 제어 (기본 주기 1kHz).
   - **H-Bridge 방향 제어**: Pin 29(IN1), Pin 31(IN2) GPIO 출력을 통한 정회전(`FORWARD`), 역회전(`BACKWARD`), 정지(`STOP`), 급제동(`BRAKE`).
   - **속도 프로파일**: 저속 시동 토크를 보장하는 0~8단계 듀티비 변환 테이블 및 0~100% 임의 듀티비 제어.

3. **WCNLB8-SR12 8-Bar LED 드라이버 (`drivers/led_bar/led_bar_driver.ko`)**:
   - **독립 GPIO 제어**: 8개의 개별 GPIO 핀을 매핑하여 모터 회전 속도(0~8단계)를 레벨 미터 형태로 시각화.
   - **비트마스크(RAW) 제어**: 8비트 마스크를 통한 개별 LED 제어 및 방향 전환 시 순간 점멸(30ms) 피드백 지원.

4. **유저 공간 통합 제어 데몬 (`app/motor_daemon`)**:
   - `poll()` 비동기 다중화 기반 이벤트 루프로 CPU 오버헤드 최소화 (Zero Polling).
   - 실시간 터미널 ANSI 대시보드 렌더링.
   - `SIGINT`/`SIGTERM` 수신 시 모터 정지 및 LED 소등 후 안전 종료(Graceful Shutdown).

---

## 2. 하드웨어 핀 맵 및 결선 테이블

Jetson Orin Nano 40핀 확장 헤더 기준 물리 핀 번호 및 Tegra GPIO / PWM 신호 매핑입니다.

### 2.1 EC11 로터리 엔코더 모듈

| 모듈 핀명 | Jetson 물리 핀 | Tegra GPIO 포트 | Line Offset | 연결 설명 및 주의사항 |
|:---:|:---:|:---:|:---:|:---|
| **VCC (5V 표기)** | **Pin 1 (3.3V)** | 3.3V Power | - | **주의**: 5V(Pin 2/4) 연결 절대 금지! **반드시 Pin 1(3.3V)** 연결 |
| **GND** | **Pin 6 (GND)** | GND | - | 공통 접지 (풀업 저항 기준점) |
| **S1 (CLK)** | **Pin 11** | PR.04 | 112 | 회전 펄스 채널 A (양방향 에지 IRQ) |
| **S2 (DT)** | **Pin 13** | PY.00 | 122 | 회전 펄스 채널 B (양방향 에지 IRQ) |
| **KEY (SW)** | **Pin 15** | PN.01 | 85 | 푸시 스위치 버튼 (하강 에지 IRQ) |

> **전원 주의사항**: Jetson Orin Nano의 모든 40핀 헤더 GPIO는 **3.3V 로직 레벨**입니다. 모듈 기판에 `5V`라고 인쇄되어 있더라도 Jetson **Pin 2/4(5V)에 연결하면 SoC 핀이 영구 손상**될 수 있습니다. 반드시 **Pin 1 (3.3V)**에 연결하십시오.

---

### 2.2 L298N DC 모터 드라이버 (HW-095)

| 모듈 단자 / 핀 | Jetson 물리 핀 / 외부 전원 | Tegra 포트 | 신호 기능 및 설명 |
|:---:|:---:|:---:|:---|
| **ENA** | **Pin 32** | PG.06 (Offset 41) | 하드웨어 PWM 속도 제어 (`32e0000.pwm` / `pwmchip3`) |
| **IN1** | **Pin 29** | PQ.05 (Offset 105) | 모터 방향 제어 GPIO 출력 1 |
| **IN2** | **Pin 31** | PQ.06 (Offset 106) | 모터 방향 제어 GPIO 출력 2 |
| **+12V (단자대)** | **외부 전원 (+) 단자** | - | **필수**: 7.4V~12V 외부 전원 (Jetson 5V 전원 사용 불가) |
| **GND (단자대)** | **외부 전원 (-) & Jetson Pin 9** | GND | **필수**: 외부 전원 (-)극과 Jetson GND를 묶는 **공통 접지(Common GND)** |
| **OUT1 / OUT2** | **DC 모터 단자** | - | 제어 대상 DC 모터 양단 연결 |

> **모터 구동 핵심 점검 사항**:
> 1. **외부 전원 분리**: L298N 내부 전압 강하(약 1.5~2V)로 인해 Jetson 내부 전원으로는 모터 구동이 불가능합니다. 나사 단자대 `+12V`에 외부 배터리(7.4V 이상) 또는 12V SMPS/어댑터를 연결하세요.
> 2. **공통 접지 (Common GND)**: L298N의 나사 단자 `GND`에 **외부 전원 (-)극과 Jetson Pin 9 (GND)를 반드시 함께 연결**해야 Jetson의 3.3V 제어 신호가 L298N 내부 로직에 올바르게 인식됩니다.
> 3. **ENA 점퍼 캡 분리**: Jetson Pin 32로 PWM 속도를 제어하려면 ENA 핀의 **점퍼 캡을 반드시 제거**하고 Jetson Pin 32를 연결해야 합니다.

---

### 2.3 WCNLB8-SR12 8-Bar LED 어레이

| LED Bar 핀 | Jetson 물리 핀 | Tegra GPIO 포트 | Line Offset | 표시 기능 |
|:---:|:---:|:---:|:---:|:---|
| **Bar 1** | **Pin 16** | PY.04 | 126 | 레벨 1단계 (최저속) |
| **Bar 2** | **Pin 18** | PY.03 | 125 | 레벨 2단계 |
| **Bar 3** | **Pin 22** | PY.01 | 123 | 레벨 3단계 |
| **Bar 4** | **Pin 36** | PR.05 | 113 | 레벨 4단계 |
| **Bar 5** | **Pin 37** | PY.02 | 124 | 레벨 5단계 |
| **Bar 6** | **Pin 38** | PI.01 | 52 | 레벨 6단계 |
| **Bar 7** | **Pin 40** | PI.00 | 51 | 레벨 7단계 |
| **Bar 8** | **Pin 24** | PZ.06 | 136 | 레벨 8단계 (최고속) |
| **GND** | **Pin 14** | GND | - | 공통 캐소드 접지 (330Ω 저항 권장) |

---

## 3. Jetson-IO 40핀 확장 헤더 설정 (최초 1회 필수)

Jetson Orin Nano의 40핀 헤더 기본 핀맵은 SPI, I2S 등으로 할당되어 있습니다. 일반 GPIO 및 하드웨어 PWM을 사용하기 위해 `jetson-io.py` 도구를 통해 핀 mux 기능을 활성화해야 합니다.

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

### 설정 순서:
1. **`Configure Jetson 40pin Header`** 선택 -> Enter
2. **`Configure header pins manually`** 선택 -> Enter
3. 스페이스바(Space)를 눌러 다음 항목들을 활성화합니다:

| 설정 항목 (핀 번호) | 설정할 기능 | 연결 대상 |
|:---|:---:|:---|
| `extperiph3_clk (29)` | `[*] gpio` | L298N IN1 |
| `extperiph4_clk (31)` | `[*] gpio` | L298N IN2 |
| `pwm7 (32)` | `[*] pwm7` | L298N ENA (하드웨어 PWM) |
| `uarta-cts/rts (11,36)` | `[*] gpio` | EC11 S1 (11), LED Bar 4 (36) |
| `spi3 (13,16,18,22,37)` | `[*] gpio` | EC11 S2 (13), LED Bar 1, 2, 3, 5 |
| `pwm1 (15)` | `[*] gpio` | EC11 KEY (15) |
| `i2s2 (12,35,38,40)` | `[*] gpio` | LED Bar 6 (38), LED Bar 7 (40) |
| `spi1 (19,21,23,24,26)` | `[*] gpio` | LED Bar 8 (24) |

4. **`Back`** -> **`Save and reboot to apply changes`**를 선택하여 저장 후 재부팅합니다.

---

## 4. 디렉터리 구조

```
/home/aidl/Linux_Device_Driver/
├── Makefile                    # 전체 통합 빌드/모듈 적재/제거 Makefile
├── README.md                   # 프로젝트 문서 및 사용 가이드
├── pin.png                     # Jetson 40핀 헤더 핀아웃 다이어그램
├── include/
│   └── common_ioctl.h          # 공통 IOCTL 매크로 및 데이터 구조체
├── drivers/
│   ├── Makefile                # 하위 드라이버 통합 빌드 Makefile
│   ├── encoder/
│   │   ├── Makefile            # EC11 모듈 빌드 설정
│   │   └── ec11_driver.c       # EC11 엔코더 캐릭터 디바이스 드라이버
│   ├── motor/
│   │   ├── Makefile            # L298N 모듈 빌드 설정
│   │   └── l298n_driver.c      # L298N DC 모터 PWM/GPIO 드라이버
│   └── led_bar/
│       ├── Makefile            # LED Bar 모듈 빌드 설정
│       └── led_bar_driver.c    # WCNLB8-SR12 8-Bar LED 드라이버
└── app/
    ├── Makefile                # 유저 데몬 빌드 Makefile
    └── main.c                  # 이벤트 다중화 및 통합 제어 데몬
```

---

## 5. 빌드 및 모듈 적재/실행/제거 가이드

### 5.1 전체 프로젝트 빌드

프로젝트 루트 디렉터리에서 `make`를 실행합니다:

```bash
cd /home/aidl/Linux_Device_Driver
make
```

빌드가 완료되면 다음 아티팩트가 생성됩니다:
- `drivers/encoder/ec11_driver.ko`
- `drivers/motor/l298n_driver.ko`
- `drivers/led_bar/led_bar_driver.ko`
- `app/motor_daemon`

빌드 산출물 클린 정리:
```bash
make clean
```

---

### 5.2 커널 모듈 적재 및 권한 설정

#### 방법 1: 자동 적재 (권장)
최상위 Makefile 타깃을 실행하면 모듈 적재와 `/dev` 노드 권한(0666)이 일괄 적용됩니다:
```bash
sudo make load
```

#### 방법 2: 수동 적재 (`insmod`)
```bash
# 1. EC11 엔코더 드라이버 적재
sudo insmod drivers/encoder/ec11_driver.ko

# 2. L298N 모터 드라이버 적재 (Pin 32 = pwmchip3 -> pwm_id=3)
sudo insmod drivers/motor/l298n_driver.ko pwm_id=3

# 3. 8-Bar LED 드라이버 적재
sudo insmod drivers/led_bar/led_bar_driver.ko

# 4. 디바이스 노드 접근 권한 부여
sudo chmod 666 /dev/ec11 /dev/l298n_motor /dev/led_bar
```

#### 모듈 적재 상태 확인:
```bash
make status
```
또는
```bash
lsmod | grep -E "ec11_driver|l298n_driver|led_bar_driver"
ls -l /dev/ec11 /dev/l298n_motor /dev/led_bar
```

정상 생성된 디바이스 노드:
- `/dev/ec11` (주번호 동적 할당, 캐릭터 디바이스)
- `/dev/l298n_motor`
- `/dev/led_bar`

---

### 5.3 개별 디바이스 단위 테스트

통합 데몬 구동 전 개별 디바이스 노드가 정상 작동하는지 확인합니다.

#### 1) WCNLB8-SR12 LED Bar 테스트
```bash
echo 3 > /dev/led_bar    # Bar 1~3 점등
echo 8 > /dev/led_bar    # Bar 1~8 전체 점등
echo 0 > /dev/led_bar    # 전체 소등
cat /dev/led_bar         # 현재 레벨 및 비트마스크 확인
```

#### 2) L298N DC 모터 드라이버 테스트
```bash
echo "forward" > /dev/l298n_motor    # 정회전 설정
echo "step 4" > /dev/l298n_motor     # 4단계(55% PWM) 구동
echo "backward" > /dev/l298n_motor   # 역회전 전환
echo "stop" > /dev/l298n_motor       # 정지
cat /dev/l298n_motor                 # 현재 모터 상태 확인
```

#### 3) EC11 엔코더 이벤트 확인
```bash
hexdump -C /dev/ec11
```
* **정상 동작**: 정지 시에는 아무 패킷도 출력되지 않으며, 노브를 1클릭 회전하거나 버튼을 누를 때마다 12바이트 이벤트(`struct ec11_event`)가 1줄씩 정확하게 출력됩니다.

---

### 5.4 통합 제어 데몬 실행

```bash
./app/motor_daemon
```

#### 런타임 터미널 대시보드:
```
==================================================================
          JETSON ORIN NANO EMBEDDED MOTOR CONTROLLER              
==================================================================
 System State : [RUNNING]
 Direction    : [FORWARD  (Clockwise)]
 Speed Step   : [ 3 / 8 ] (PWM Duty:  45%)
 LED Bar      : [ # # # . . . . . ]
 Encoder Pos  : +3 steps
------------------------------------------------------------------
 [Operation Instructions]
  * Rotate Knob (S1/S2) : Speed Step (0 -> 8)
  * Click Knob  (KEY)   : Toggle Direction (FWD <-> REV)
  * Press Ctrl + C      : Safe Stop and Exit
==================================================================
```

#### 사용자 조작 방법:
- **노브 시계 방향 회전 (CW)**: 속도 단계 증가 (최대 8단계, PWM 듀티비 100%), LED Bar 1칸 추가 점등, 모터 가속.
- **노브 반시계 방향 회전 (CCW)**: 속도 단계 감소 (최소 0단계, 모터 정지), LED Bar 1칸 감소 점등, 모터 감속.
- **노브 푸시 스위치 클릭 (KEY)**: 모터 회전 방향 토글 (`FORWARD` <-> `BACKWARD`), 방향 전환 피드백으로 LED Bar 30ms 순간 점멸.
- **종료 (`Ctrl + C`)**: 모터 즉시 정지(PWM 0%), LED Bar 소등, 장치 파일 디스크립터 안전 종료.

---

### 5.5 커널 모듈 언로드 및 정리

작업 완료 후 모듈을 안전하게 언로드합니다:

```bash
sudo make unload
```

또는 수동 제거:
```bash
sudo rmmod led_bar_driver
sudo rmmod l298n_driver
sudo rmmod ec11_driver
```

---

## 6. 하드웨어 및 안전 주의사항 (Troubleshooting)

1. **전원 분리 및 모터 서지 전류 보호**:
   - 모터 구동 시 발생하는 역기전력(Back-EMF) 및 피크 서지 전류가 Jetson 3.3V/5V 라인으로 역류하지 않도록 반드시 외부 독립 전원(7.4V~12V)을 사용하십시오.
   - 외부 전원 (-)극과 Jetson Pin 9(GND)는 반드시 **공통 접지(Common GND)**로 묶여 있어야 정상적인 로직 전위 기준점이 형성됩니다.

2. **3.3V 로직 레벨 절대 준수**:
   - Jetson Orin Nano의 40핀 헤더는 5V 입력을 견디지 못합니다(Non-5V tolerant). EC11 VCC 단자에 5V를 인가하면 SoC 내부 GPIO 버퍼가 소손될 수 있으므로 반드시 **Pin 1 (3.3V)**에 연결하십시오.

3. **엔코더 플로팅(Floating) 노이즈 차단**:
   - VCC 또는 GND 듀퐁선 결선이 헐거워 접촉 불량이 생기면 신호선이 공중에 떠서 초당 수만 회의 가짜 인터럽트가 발생할 수 있습니다. 빵판 및 점퍼선 체결 상태를 단단히 고정하십시오.