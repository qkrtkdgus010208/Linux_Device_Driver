# Jetson Orin Nano 모터 제어 프로젝트

EC11 회전 이벤트를 유저 공간 앱이 읽어 L298N(HW-095)의 PWM 속도와 방향을
제어하고, WCNLB8-SR12의 8개 바에 출력 단계를 표시한다.
각 하드웨어는 독립된 캐릭터 디바이스 모듈이다.

## 1. 지원 환경과 현재 검증 범위

- 대상: NVIDIA Jetson Orin Nano 개발 키트의 40핀 헤더, JetPack 5/6의
  Linux 5.10/5.15, ARM64.
- 실제 빌드 확인: `5.15.199-tegra`, GCC 11.4.0, ARM64.
  `make -j4 W=1`로 모듈 3개와 앱을 빌드했다. 소스 경고는 없으며,
  배포판 컴파일러 패치 버전 문자열 차이에 따른 Kbuild 경고는 있다.
- `make test`의 앱 통합 테스트 10개 통과. FIFO 이벤트와 테스트 전용
  IOCTL 대체 라이브러리로 실제 앱 실행 파일을 검사한다.
- 5.10 API 호환성을 고려해 작성했으나 5.10 커널의 실제 빌드/적재는 미검증이다.
  연결된 모터/LED/엔코더의 출력, pinmux, 전기적 동작도 미검증이다.
  이 작업에서 `insmod`나 핀 설정 변경은 실행하지 않았다.
- PWM, GPIO 컨트롤러, GPIO IRQ, 캐릭터 디바이스와 devtmpfs/udev가 활성화된
  해당 보드의 커널 설정 및 정확히 일치하는 헤더/Module.symvers가 필요하다.
  커널 6.x는 이 프로젝트의 지원 범위에 포함하지 않는다.

## 2. 파일 구성

```text
Linux_Device_Driver/
├── Makefile
├── README.md
├── include/
│   ├── common_ioctl.h       # 커널/유저 공간 공용 ABI
│   └── driver_common.h      # 캐릭터 디바이스 및 동적 GPIO 번호 공통 처리
├── drivers/
│   ├── encoder/
│   │   ├── ec11.c
│   │   └── Makefile
│   ├── motor/
│   │   ├── l298n_motor.c
│   │   └── Makefile
│   └── led_bar/
│       ├── led_bar.c
│       └── Makefile
├── app/
│   ├── main.c
│   └── Makefile
├── scripts/board-info.sh    # 읽기 전용 GPIO/PWM 보드 정보
├── deploy/motor-daemon.service
└── tests/
    ├── Makefile
    ├── mock_ioctl.c
    └── test_app.py
```

## 3. 물리 핀과 실제 컨트롤러 매핑

요청한 **물리 핀 번호**를 그대로 사용한다. 요청의 일부 GPIO 이름은 Orin Nano
공식 매핑과 다르므로 아래의 실제 매핑을 적용했다. GPIO offset은
`tegra234-gpio` 컨트롤러 내부 번호이며 물리 핀이나 전역 Linux GPIO 번호가 아니다.
모듈은 `gpiochip_find()`로 컨트롤러를 찾아 현재 `base + offset`을 계산한다.
기본 offset과 PWM 경로는 [NVIDIA Jetson.GPIO 매핑](https://github.com/NVIDIA/jetson-gpio/blob/master/lib/python/Jetson/GPIO/gpio_pin_data.py)의
Orin NX/Nano 정의를 기준으로 한다.

| 신호 | 물리 핀 | SoC GPIO | chip offset | 기능/설정 |
| --- | ---: | --- | ---: | --- |
| EC11 CLK/A | 11 | PR.04 | 112 | GPIO 입력, Falling IRQ |
| EC11 DT/B | 13 | PY.00 | 122 | GPIO 입력 |
| EC11 SW | 15 | PN.01 | 85 | GPIO 입력, Falling IRQ |
| L298N ENA | 32 | PG.06 | 41 | PWM7, `32e0000.pwm`, 채널 0 |
| L298N IN1 | 29 | PQ.05 | 105 | GPIO 출력 |
| L298N IN2 | 31 | PQ.06 | 106 | GPIO 출력 |
| LED Bar 1 | 16 | PY.04 | 126 | GPIO 출력 |
| LED Bar 2 | 18 | PY.03 | 125 | GPIO 출력 |
| LED Bar 3 | 22 | PY.01 | 123 | GPIO 출력 |
| LED Bar 4 | 36 | PR.05 | 113 | GPIO 출력 |
| LED Bar 5 | 37 | PY.02 | 124 | GPIO 출력 |
| LED Bar 6 | 38 | PI.01 | 52 | GPIO 출력 |
| LED Bar 7 | 40 | PI.00 | 51 | GPIO 출력 |
| LED Bar 8 | 24 | PZ.06 | 136 | GPIO 출력 |

Pin 11은 UART1_RTS 계열 핀이고 GPIO09는 Pin 7이다. Pin 13은 SPI1_SCK,
GPIO11은 Pin 31이다. GPIO08은 이 헤더의 Pin 31 신호명이 아니다.
Pin 32의 PWM7 **하드웨어 채널 0**과 `pwm_request()`의 **전역 PWM ID**도
서로 구분해야 한다. 모터 모듈은 PWM ID를 자동 추정하지 않는다.

캐리어 보드가 다르면 회로도와 `gpioinfo`에서 매핑을 재확인하고 모듈 파라미터를
조정한다. 헤더 핀의 전압/전류 조건은
[Orin Nano Carrier Board Specification, Table 3-3](https://developer.nvidia.com/downloads/assets/embedded/secure/jetson/orin_nano/docs/jetson_orin_nano_devkit_carrier_board_specification_sp.pdf)을
참조한다.

## 4. 배선과 전기적 조건

이 헤더의 신호는 3.3 V이다. 모터 전원은 별도 전원을 사용하고 Jetson,
모터 드라이버, LED 구동 회로의 GND를 공통 연결한다. Jetson GPIO로 모터를
급전하지 않는다.

**8개 GPIO는 각 바의 논리 출력을 직접 제어하지만, LED 전류를 헤더에서 직접
공급하면 안 된다.** 사용 핀들은 TXB0108을 경유하고 공식 사양의 출력 구동 조건은
약 ±20 µA이다. 바 LED 부하는 별도 전원과 높은 입력 임피던스의 CMOS 버퍼 또는
버퍼를 거친 트랜지스터/MOSFET 구동 회로로 공급한다. LED마다 직렬 전류 제한
저항을 사용하고, `R = (구동전압 - LED 순방향전압 - 스위치 전압강하) / LED 전류`로
산정한다. WCNLB8-SR12의 실물 핀 배열과 극성은 부품 자료/멀티미터로 확인한다.
이 프로젝트의 Bar 1~8 번호는 논리 순서이며 LED 패키지 핀 번호를 뜻하지 않는다.
근거: [NVIDIA 캐리어 보드 전기 사양](https://developer.nvidia.com/downloads/assets/embedded/secure/jetson/orin_nano/docs/jetson_orin_nano_devkit_carrier_board_specification_sp.pdf).

L298N의 입력 HIGH 기준은 2.3 V 이상이지만 입력 전류가 최대 100 µA이므로,
Jetson 헤더의 약한 출력과 직결했을 때 동작을 보장할 수 없다. ENA/IN1/IN2에도
3.3 V CMOS 입력을 받는 버퍼를 배치해 L298N 입력 부하를 분리한다.
근거: [ST L298 데이터시트](https://www.st.com/resource/en/datasheet/l298.pdf).

- ENA 점퍼를 제거하고 Pin 32 PWM 신호를 ENA 구동 회로에 연결한다.
  모터는 OUT1/OUT2를 사용한다. HW-095의 5 V regulator 점퍼는 ENA 점퍼와
  별개이며, 실제 모듈 회로와 전원 조건에 맞춰 설정한다.
- 부팅/모듈 미적재/전원 차단 중 ENA가 뜨지 않도록 **버퍼 출력 이후의 ENA**에
  비활성 LOW를 보장하는 풀다운/출력 enable 회로를 둔다. 전원이 다른 회로의
  역급전도 방지한다.
- EC11 A/B 접점의 공통 단자는 GND, SW의 반대 단자는 GND에 연결한다.
  입력의 기본 상태는 HIGH이다. 이 모듈은 내부 pull-up을 설정하지 않는다.
  3.3 V pull-up 또는 입력 버퍼를 사용하고 실제 파형을 확인한다.
  일반적인 5 V 엔코더 모듈의 출력/풀업을 헤더에 그대로 연결하지 않는다.
- TXB0108 헤더 측의 강한 풀업/풀다운은 신호 레벨을 저하시킬 수 있다.
  직접 추가할 경우 TI의 저항 조건을 지키고, 접점에 사용하는 강한 풀업이나
  RC 필터는 입력 버퍼의 접점 측에 둔다.
  근거: [TI TXB0108 데이터시트](https://www.ti.com/lit/ds/symlink/txb0108.pdf).
- 모터 전원은 모터의 정격/기동 전류와 모듈의 방열, 역기전력 보호 회로에 맞춘다.
  최초 테스트는 무부하 상태와 낮은 듀티에서 수행한다.

## 5. Jetson pinmux 준비

GPIO API나 PWM API는 헤더의 pinmux 설정을 대신하지 않는다. 사용 중인 BSP와
캐리어 보드에 맞게 아래 기능과 입출력 방향을 먼저 설정한다.

1. Pin 32만 PWM7 출력으로 설정한다. Pin 15의 PWM1은 켜지 않는다.
2. Pin 11/13/15는 GPIO 입력, Pin 29/31 및 8개 LED 핀은 GPIO 출력으로 설정한다.
3. 사용 핀을 점유하는 SPI/UART/I2S 기능/기존 디바이스를 해당 보드 DT에서 해제한다.
   공유 SPI 버스 등을 사용 중이라면 그 장치를 먼저 분리하거나 핀 설계를 변경한다.
4. 개발 키트에서는 `sudo /opt/nvidia/jetson-io/jetson-io.py`로 지원하는 헤더
   기능 변경을 저장하고 재부팅한다. UI가 필요한 GPIO 방향/패드 설정까지 제공하지
   않는 경우에는 해당 JetPack의 pinmux spreadsheet와 MB1 BCT/DT 변경을 적용한다.
   Pin 38처럼 기본 주변장치 입력인 핀도 LED GPIO 출력 설정이 필요하다.
5. 재부팅 후 아래 명령과 pinmux/파형을 확인한다.

참조: [NVIDIA Jetson-IO/Expansion Headers](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/HR/ConfiguringTheJetsonExpansionHeaders.html),
[Orin NX/Nano pinmux와 보드 설정](https://docs.nvidia.com/jetson/archives/r35.6.0/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonOrinNxNanoSeries.html).

```bash
cd /home/aidl/Linux_Device_Driver
make board-info
gpiodetect
gpioinfo
sudo cat /sys/kernel/debug/gpio
sudo cat /sys/kernel/debug/pwm
```

`gpiodetect/gpioinfo`는 GPIO character-device 접근이 제공되는 보드에서 사용한다.
컨테이너에 `/dev/gpiochip*`가 노출되지 않았거나 debugfs가 마운트되지 않았다면
보드의 호스트에서 확인한다. 모듈이 GPIO를 점유하면 같은 라인을 `gpioset`으로
동시에 제어하지 않는다.

`make board-info`에서 `32e0000.pwm` 경로를 가진 `pwmchipN`을 찾는다.
5.10/5.15에서 `N`은 그 chip의 전역 PWM base이며, 이 컨트롤러의 채널 0은
`pwm_id=N`으로 요청한다. 다른 chip은 fan 등에 사용될 수 있으므로 `pwmchip0`을
임의로 선택하지 않는다. 근거: [Linux 5.10 PWM API와 sysfs](https://www.kernel.org/doc/html/v5.10/driver-api/pwm.html).

현재 읽기 전용 조회 결과는 `pwmchip1 -> .../32e0000.pwm/pwm/pwmchip1`이었다.
이 환경에서는 `pwm_id=1`이며 BSP/부팅 순서에 따라 달라질 수 있다.
드라이버는 요청한 PWM의 provider 이름과 채널 0을 검사한다.
오실로스코프로 Pin 32 및 ENA의 PWM 파형을 확인해야 실제 출력 연결을 검증할 수 있다.

## 6. 빌드

보드의 개발 도구 설치 예시:

```bash
sudo apt-get update
sudo apt-get install build-essential python3 gpiod
```

`/lib/modules/$(uname -r)/build`가 없다면 해당 L4T 릴리스에 맞는 NVIDIA 커널
헤더 패키지/빌드 트리를 설치한다. 일반 Ubuntu 헤더로 Tegra 모듈을 빌드하지 않는다.
다른 릴리스의 헤더, 커널 설정 또는 Module.symvers를 섞지 않는다.

```bash
cd /home/aidl/Linux_Device_Driver
make -j4
make test
modinfo drivers/encoder/ec11.ko
modinfo drivers/motor/l298n_motor.ko
modinfo drivers/led_bar/led_bar.ko
```

산출물:

```text
drivers/encoder/ec11.ko
drivers/motor/l298n_motor.ko
drivers/led_bar/led_bar.ko
app/motor_daemon
```

서브 디렉터리 빌드와 커스텀 커널/크로스 빌드:

```bash
make -C drivers/encoder
make -C drivers/motor
make -C drivers/led_bar
make app
make KDIR=/path/to/prepared/kernel/build
make KDIR=/path/to/arm64/kernel/build ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
make clean
```

크로스 빌드 후 `make test`는 ARM64 실행 파일을 실행할 수 있는 대상 보드에서
수행한다. 커널 빌드 트리는 해당 커널의 설정/준비 및 Module.symvers가 있어야 한다.

## 7. 모듈 적재와 앱 실행

pinmux와 배선을 준비한 뒤 `make board-info`가 출력한 PWM ID를 사용한다.
아래 `1`은 현재 조회된 보드의 예시다.

```bash
cd /home/aidl/Linux_Device_Driver
sudo insmod drivers/encoder/ec11.ko
sudo insmod drivers/motor/l298n_motor.ko pwm_id=1
sudo insmod drivers/led_bar/led_bar.ko
ls -l /dev/ec11 /dev/l298n_motor /dev/led_bar
sudo dmesg -T | tail -n 40
sudo ./app/motor_daemon
```

각 모듈이 `alloc_chrdev_region`, `cdev_add`, `class_create`, `device_create`로
노드를 등록한다. 노드 자동 생성에는 보드의 devtmpfs/udev가 필요하다.
세 디바이스는 각각 단일 open만 허용한다. 중복 실행은 `EBUSY`로 거절한다.
기본 실행 예시는 root 권한을 사용한다.

- 기본 시작: 선택 단계 0, 모터 OFF, 정회전, LED 0개.
- 회전: 선택 단계를 0~8로 제한한다. OFF 중에도 선택 단계는 조절할 수 있다.
- 짧은 클릭: 선택 단계를 유지하고 모터 ON/OFF를 전환한다.
- 800 ms 이상 긴 클릭: 선택 단계를 유지하고 방향을 전환한다.
  버튼 이벤트는 디바운스를 거쳐 **버튼을 놓을 때** 발생한다.
- 방향이 바뀌고 모터가 구동 중이면 드라이버가 PWM 0/disable 및 방향 GPIO LOW를
  먼저 적용하고 기본 100 ms 정지 지연 후 반대 방향을 출력한다.
- LED는 실제 요청 출력 단계를 표시한다. OFF면 선택 단계와 관계없이 0개다.
- Ctrl+C/SIGTERM, 앱 오류 및 마지막 fd close 시 모터 정지와 LED 소등을 요청한다.

| 단계 | PWM 듀티 | LED 개수 (ON일 때) |
| ---: | ---: | ---: |
| 0 | 0% | 0 |
| 1 | 12.5% | 1 |
| 2 | 25% | 2 |
| 3 | 37.5% | 3 |
| 4 | 50% | 4 |
| 5 | 62.5% | 5 |
| 6 | 75% | 6 |
| 7 | 87.5% | 7 |
| 8 | 100% | 8 |

표의 속도는 듀티 기반 개방 루프 명령이다. RPM 피드백 제어는 구현하지 않는다.
저속에서 모터가 기동하지 않거나 부하에 따라 실제 RPM이 달라질 수 있다.

```bash
sudo ./app/motor_daemon --reverse-encoder
sudo ./app/motor_daemon --button-mode toggle
sudo ./app/motor_daemon --button-mode direction
sudo ./app/motor_daemon --long-press-ms 1000
./app/motor_daemon --help
```

`toggle`은 길이와 관계없이 클릭으로 ON/OFF만 전환한다.
`direction`은 클릭으로 방향만 전환하며 활성 상태로 시작하지만 초기 단계가 0이므로
초기 출력은 0%이다. 이 모드는 회전으로 단계를 올리면 바로 모터가 움직인다.
정/역 물리 방향이 기대와 반대이면 모터 배선/IN1·IN2 배선을 확인한다.

## 8. 구현과 ABI

`include/common_ioctl.h`를 커널 모듈과 앱이 같이 사용한다.
구조체는 고정 폭 정수와 정렬된 64비트 타임스탬프를 사용하며 reserved는 0이어야 한다.
각 드라이버에는 CONFIG_COMPAT용 IOCTL 포인터 변환도 포함했다.

| 디바이스 | 인터페이스 | 데이터 |
| --- | --- | --- |
| `/dev/ec11` | `read`, `poll` | 24바이트 `struct ec11_event` 한 개 |
| `/dev/ec11` | `EC11_IOC_GET_STATS` | 누락 수와 생성 sequence |
| `/dev/l298n_motor` | `MOTOR_IOC_SET_STATE` | duty_permille 0~1000, direction 0/1, enabled 0/1 |
| `/dev/l298n_motor` | `MOTOR_IOC_GET_STATE` | 마지막 요청 상태 또는 정지 상태 |
| `/dev/l298n_motor` | `MOTOR_IOC_KEEPALIVE` | 인자 없는 heartbeat |
| `/dev/led_bar` | `LED_BAR_IOC_SET_LEVEL` | 포인터로 전달하는 `__u32`, 0~8 |
| `/dev/led_bar` | `LED_BAR_IOC_GET_LEVEL` | 현재 레벨 `__u32` |

EC11 회전 IRQ는 A의 Falling Edge에서 B를 읽어 +1/-1 이벤트를 만든다.
기본 1500 µs 이내의 추가 Falling Edge는 버린다. SW Falling IRQ는 delayed work를
시작하고, 기본 15 ms 후 눌림을 확인하며 두 번 연속 HIGH 샘플에서 놓임을 확인한다.
이벤트의 버튼 value는 눌린 기간의 ms 값이며 놓임 확인 지연도 포함한다.
IRQ에서는 수면 가능한 GPIO/PWM API를 호출하지 않는다.

이 방식은 A Falling Edge 기준 1x 디코딩이다. EC11 변형에 따라 한 detent당
이벤트 개수가 다를 수 있다. 디바운스 시간보다 긴 채터링이나 매우 빠른 회전,
느리게 흔들리는 접점의 방향 판정까지 완전히 복원하지는 않는다.
실물 파형을 보고 `debounce_us`를 조절하거나 입력 측 하드웨어 필터를 추가한다.

FIFO는 이벤트 128개이며 full이면 새 이벤트를 버리고 dropped를 증가시킨다.
앱은 non-blocking `read`와 50 ms `poll`, sequence 검사, 100 ms 간격 stats 검사로
이벤트 누락을 감지하면 정지하고 오류 종료한다.

모터는 요구한 `pwm_request`, `pwm_config`, `pwm_enable`, `pwm_disable`, `pwm_free`
API를 사용한다. 초기화 시 `pwm_apply_state`로 정상 극성과 disabled 상태를 설정한다.
듀티 계산은 64비트 산술을 사용하고 mutex로 제어/워치독을 직렬화한다.
기본 PWM 주기는 1,000,000 ns, 즉 1 kHz이다.

모터 모듈은 마지막 SET/KEEPALIVE 이후 기본 1000 ms 타임아웃을 감시한다.
검사 주기는 타임아웃의 절반이므로 정상 스케줄링에서 약 1~1.5초 후 정지하며
시스템 부하에 따른 지연이 추가될 수 있다. 워치독이 발동하면 이후 SET/KEEPALIVE는
`ETIMEDOUT`을 반환한다. 자동 재가동하지 않으며 앱을 종료하고 다시 열어야 한다.
GET으로는 마지막 요청 상태를 읽을 수 있으나 실제 RPM/파형 측정 결과는 아니다.

주요 모듈 파라미터:

| 모듈 | 파라미터 | 기본값 |
| --- | --- | --- |
| ec11 | gpiochip_label, clk_offset, dt_offset, sw_offset | tegra234-gpio, 112, 122, 85 |
| ec11 | debounce_us, button_debounce_ms | 1500, 15 |
| l298n_motor | in1_offset, in2_offset | 105, 106 |
| l298n_motor | pwm_id | -1: 명시하지 않으면 적재 실패 |
| l298n_motor | pwm_controller | 32e0000.pwm: provider 이름 검사 |
| l298n_motor | period_ns, reverse_deadtime_ms, watchdog_ms | 1000000, 100, 1000 |
| led_bar | bar_offsets | 126,125,123,113,124,52,51,136 |
| led_bar | active_low | false |

모터와 LED에도 `gpiochip_label`을 지정할 수 있다. 모든 파라미터는 적재 시에만
설정한다. LED 연결 회로가 LOW 활성이라면 `active_low=1`을 사용한다.

```bash
sudo insmod drivers/encoder/ec11.ko debounce_us=3000 button_debounce_ms=20
sudo insmod drivers/motor/l298n_motor.ko pwm_id=1 period_ns=1000000 reverse_deadtime_ms=300 watchdog_ms=1500
sudo insmod drivers/led_bar/led_bar.ko active_low=1
```

다른 파라미터로 적재하려면 기존 앱을 종료하고 기존 모듈을 먼저 제거한다.
100 ms 정지 지연이 모터의 완전한 기계적 정지를 보장하는 것은 아니므로 실물 모터의
관성/부하를 고려해 늘린다. 이 모듈에는 suspend/resume 자동 복구가 없으므로
시스템 suspend 전에 앱을 종료하고 모터 전원을 비활성화한다.

## 9. 테스트 절차

### 하드웨어 없이 앱 테스트

```bash
make test
```

범위: 시작/종료, 0/8 경계 clamp, 모든 속도의 125-permille 간격,
짧은/긴 클릭, 방향 전환, 방향 전용/토글 전용 모드, 회전 반전,
idle heartbeat, sequence 누락, 잘린/알 수 없는 이벤트,
모터/LED/heartbeat 오류, FIFO overflow, 잘못된 앱 인자.
테스트 라이브러리는 tests 프로세스에서만 LD_PRELOAD로 사용되며 실제 GPIO/PWM을
접근하지 않는다. 커널 IRQ 동시성이나 실제 PWM 출력에 대한 테스트는 아니다.

### 엔코더 단독 확인

앱을 실행하지 않은 상태에서 ec11만 적재하고 아래를 실행한다.
회전하면 type=1, value=+1/-1이고 클릭 후 놓으면 type=2, value=hold_ms가 출력된다.

```bash
sudo python3 - <<'PY'
import os, select, struct
fd = os.open('/dev/ec11', os.O_RDONLY | os.O_NONBLOCK)
try:
    while True:
        select.select([fd], [], [])
        raw = os.read(fd, 24)
        kind, value, timestamp, sequence, reserved = struct.unpack('<IiQII', raw)
        print(kind, value, timestamp, sequence, reserved, flush=True)
finally:
    os.close(fd)
PY
```

### LED 단독 확인

모터 전원을 비활성화하고 LED 모듈을 적재한 뒤 아래를 실행한다.
레벨 0~8이 순서대로 표시되고 종료 시 모두 꺼져야 한다.
`0x40044c01`은 ARM64 Linux에서 `_IOW('L', 1, __u32)`에 해당한다.

```bash
sudo python3 - <<'PY'
import fcntl, os, struct, time
fd = os.open('/dev/led_bar', os.O_RDWR)
try:
    for level in range(9):
        fcntl.ioctl(fd, 0x40044c01, struct.pack('<I', level))
        time.sleep(0.5)
finally:
    fcntl.ioctl(fd, 0x40044c01, struct.pack('<I', 0))
    os.close(fd)
PY
```

### 모터와 전체 연결 확인

1. 최초에는 모터 부하 전원을 비활성화하고 데몬을 실행한다. 시작 출력이 0%인지
   확인한다. 단계 1을 선택하고 짧게 클릭해 ON으로 전환한다.
2. 오실로스코프로 Pin 32/버퍼 출력/ENA에서 주기 1 ms, 듀티 12.5%를 확인한다.
   단계 4는 50%, 단계 8은 100% HIGH 출력이다.
3. 방향 전환 시 ENA 비활성 구간이 먼저 존재하고 IN1/IN2가 전환되는지 확인한다.
   IN1=HIGH, IN2=LOW가 논리 정회전이다.
4. 배선 검증 후 모터 전원을 공급한다. 낮은 단계에서 기동을 확인하고 단계별로
   변경하며 LED 수가 출력 단계와 일치하는지 확인한다.
5. 짧은 클릭 OFF, Ctrl+C, SIGTERM 시 모터가 정지하고 LED가 꺼지는지 확인한다.
6. 별도 터미널에서 PID를 확인해 `sudo kill -STOP <PID>`로 heartbeat를 중단한다.
   기본 약 1~1.5초 후 모터 워치독 정지를 확인한다. 이 동안 앱이 멈춰 있으므로
   LED는 마지막 레벨을 유지할 수 있다. `sudo kill -CONT <PID>` 후 앱은
   ETIMEDOUT으로 종료하고 LED를 소등한다. 재시험은 앱을 다시 실행한다.
7. `sudo kill -KILL <PID>`의 경우 fd가 닫혀 모터와 LED의 release 처리가 실행되는지
   확인한다. 모듈이 열려 있는 동안 `rmmod`는 사용 중인 모듈로 거절되어야 한다.

IRQ 확인은 `sudo cat /proc/interrupts`에서 `ec11-clk`, `ec11-sw`를 찾아 회전/클릭
전후 증가 여부를 비교한다. `sudo dmesg -w`로 오류 및 워치독 로그를 확인한다.

## 10. 서비스 실행과 종료

선택적으로 systemd에서 포그라운드 데몬을 관리할 수 있다.

```bash
sudo install -m 0755 app/motor_daemon /usr/local/sbin/motor_daemon
sudo install -m 0644 deploy/motor-daemon.service /etc/systemd/system/motor-daemon.service
sudo systemctl daemon-reload
sudo systemctl start motor-daemon.service
sudo journalctl -u motor-daemon.service -f
sudo systemctl stop motor-daemon.service
```

서비스는 모듈을 자동 적재하지 않으며 세 노드가 있을 때만 시작한다.
자동 재시작은 하지 않는다. 부팅 자동 적재/enable은 보드 설정과 실물 검증을 마친
후 별도로 구성한다.

```bash
sudo rmmod led_bar
sudo rmmod l298n_motor
sudo rmmod ec11
```

## 11. 문제 해결

| 현상 | 확인할 항목 |
| --- | --- |
| Invalid module format | uname -r, KDIR, vermagic, Module.symvers/설정 일치 |
| Key was rejected | 대상 커널의 모듈 서명 정책에 맞춰 모듈 서명 |
| ENODEV/GPIO 초기화 실패 | 컨트롤러 label, offset, 해당 GPIO 드라이버 등록 |
| EBUSY | 기존 데몬, SPI/UART/I2S 드라이버, sysfs PWM export 점유 |
| pwm_id 적재 실패 | board-info 경로, provider 이름, PWM7 활성 여부 |
| PWM API 성공이나 Pin 32에 파형 없음 | pinmux/MB1 BCT, 캐리어 배선, 입력 임피던스, 실제 오실로스코프 파형 |
| ENA가 계속 HIGH | ENA 점퍼 제거, 버퍼/풀다운 및 극성 확인 |
| 모터가 움직이지 않음 | 모터/논리 전원, 공통 GND, OUT1/OUT2, 기동 듀티, L298N 입력 레벨 |
| 회전 방향/이벤트 수 불일치 | --reverse-encoder, A/B 배선, EC11 detent 유형, debounce_us |
| 버튼 이벤트 없음 | GPIO input/pull-up, SW GND, pinmux, 버튼을 놓았는지 확인 |
| LED가 약하거나 잘못 점등 | 전류 공급 버퍼, 저항, 패키지 극성/핀 배열, active_low |
| ETIMEDOUT | 앱/시스템 지연, 워치독 로그; 원인을 해결하고 앱 재실행 |

PWM이 sysfs에 export되어 있는 경우 해당 채널 사용자를 중지하고, 실제 점유 경로의
`unexport`에 채널 번호 0을 써 해제한 뒤 모듈을 적재한다. 이 작업은 다른 PWM
사용자가 없는지 확인한 후 시행한다.
