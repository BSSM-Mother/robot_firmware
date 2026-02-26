# STM32F103 로봇 모터 제어 시스템

UART를 통해 라즈베리파이로부터 명령을 받아 L298N 모터 드라이버를 제어하는 STM32 펌웨어.

## 하드웨어 정보

### MCU
- **보드**: NUCLEO-F103RB, Blue Pill (STM32F103C8T6)
- **칩**: STM32F103RB / STM32F103C8T6 (ARM Cortex-M3)
- **Flash**: 128 KB (RB) / 64 KB 또는 128 KB (C8T6 클론)
- **RAM**: 20 KB
- **클럭**: HSI 8MHz (PLL 미사용)

### 통신 인터페이스

#### NUCLEO-F103RB: UART2 (ST-LINK를 통해 라즈베리파이와 연결)
| 핀 | GPIO | 기능 | 참고 |
|----|------|------|------|
| TX | PA2 | 송신 | ST-LINK로 출력 |
| RX | PA3 | 수신 | ST-LINK로 입력 |
| **보레이트** | | **115200 baud** | |
| **Format** | | 8N1 (8bit, No parity, 1 stop) | |

#### Blue Pill: UART1
| 핀 | GPIO | 기능 | 참고 |
|----|------|------|------|
| TX | PA9 | 송신 | USB-TTL 연결 권장 |
| RX | PA10 | 수신 | USB-TTL 연결 권장 |
| **보레이트** | | **115200 baud** | |
| **Format** | | 8N1 (8bit, No parity, 1 stop) | |

### 모터 제어 핀 (L298N)

#### 방향 제어 (GPIO Output)
| 핀 | GPIO | 기능 | 설명 |
|----|------|------|------|
| IN1 | PB0 | 왼쪽 모터 (+) | High=정방향, Low=역방향 |
| IN2 | PB1 | 왼쪽 모터 (-) | High=역방향, Low=정방향 |
| IN3 | PB2 | 오른쪽 모터 (+) | High=정방향, Low=역방향 |
| IN4 | PB3 | 오른쪽 모터 (-) | High=역방향, Low=정방향 |

#### 속도 제어 (PWM - Timer)
| 핀 | Timer | Channel | 기능 | 범위 |
|----|-------|---------|------|------|
| ENA | PA8 | TIM1_CH1 | 왼쪽 모터 속도 | 0-255 |
| ENB | PA7 | TIM3_CH2 | 오른쪽 모터 속도 | 0-255 |

**PWM 특성:**
- 주파수: 1MHz (8MHz / 8)
- 분해능: 8bit (0-255)
- 주기: 256μs

## 통신 프로토콜

### 명령 형식
```
속도,왼쪽방향,오른쪽방향\r\n
```

### 파라미터
- **속도**: 0-255 (0=정지, 255=최고속)
- **방향**: 
  - `1` = 전진 (Forward)
  - `0` = 정지 (Stop)
  - `2` = 후진 (Reverse)

### 예제 명령

| 명령 | 설명 |
|------|------|
| `100,1,1\r\n` | 속도100 직진 (양쪽 모두 전진) |
| `80,1,2\r\n` | 우회전 (왼쪽전진, 오른쪽후진) |
| `80,2,1\r\n` | 좌회전 (왼쪽후진, 오른쪽전진) |
| `100,2,2\r\n` | 속도100 후진 (양쪽 모두 후진) |
| `0,0,0\r\n` | 정지 |

### 응답
성공적으로 파싱된 명령에 대해:
```
OK:L=FWD,R=FWD\r\n
```

**응답 형식:**
- `OK:L=[상태],R=[상태]\r\n`
- L/R 상태: `FWD` (전진), `REV` (후진), `STOP` (정지)

## 라즈베리파이 연결

### UART 핀 매핑
```
라즈베리파이          NUCLEO-F103RB
─────────────────────────────────
TX (GPIO 14)  ──→  PA3 (USART2_RX)
RX (GPIO 15)  ←──  PA2 (USART2_TX)
GND           ────  GND

라즈베리파이          Blue Pill
─────────────────────────────────
TX (GPIO 14)  ──→  PA10 (USART1_RX)
RX (GPIO 15)  ←──  PA9 (USART1_TX)
GND           ────  GND
```

### Python 예제
```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# 직진
ser.write(b'100,1,1\r\n')
time.sleep(2)

# 정지
ser.write(b'0,0,0\r\n')

ser.close()
```

### Bash 예제
```bash
# 직진 (속도100)
echo "100,1,1" > /dev/ttyUSB0

# 회전 (우회전)
echo "80,1,2" > /dev/ttyUSB0

# 정지
echo "0,0,0" > /dev/ttyUSB0
```

## 빌드 및 업로드

### 필수 도구
- STM32CubeIDE 또는 VS Code + CMake
- arm-none-eabi-gcc (GNU Tools for STM32)
- ST-Link V2/V3

### 빌드
```bash
cmake --preset Debug
cmake --build --preset Debug
```

Blue Pill 64KB:
```bash
cmake --preset BluePill-64-Debug
cmake --build --preset BluePill-64-Debug
```

Blue Pill 128KB:
```bash
cmake --preset BluePill-128-Debug
cmake --build --preset BluePill-128-Debug
```

### 업로드 (VS Code)
1. **F5**: 디버그 시작 (코드 업로드)
2. **Shift+F5**: 디버거 종료 (프로그램 실행)

### 업로드 (명령줄)
```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program build/Release/mother.elf verify reset exit"
```

## 시스템 초기화 순서

1. **시스템 클럭**: HSI 8MHz 초기화
2. **UART**: 115200 baud로 초기화 (NUCLEO=USART2 PA2/PA3, Blue Pill=USART1 PA9/PA10)
3. **GPIO**: PB0-3 출력, PA7, PA8 PWM 출력 설정
4. **Timer**: TIM1, TIM3 PWM 타이머 설정 (1MHz, 256 주기)
5. **시작 메시지**: `"Motor Control System Started\r\n"` 전송
6. **명령 대기**: UART에서 명령 수신 대기

## 파일 구조

```
.
├── CMakeLists.txt              # CMake 빌드 설정
├── CMakePresets.json           # CMake 프리셋
├── stm32f103xb_flash.ld        # 링커 스크립트
├── README.md                   # 이 파일
├── Inc/
│   └── motor.h                 # 모터 제어 함수 헤더
├── Src/
│   ├── main.c                  # 메인 프로그램 및 초기화 코드
│   ├── startup_stm32f103xx.S   # STM32 스타트업 코드
│   ├── syscall.c               # 시스템 콜 구현
│   └── sysmem.c                # 메모리 관리
└── build/
    └── Release/
        └── mother.elf          # 컴파일된 바이너리
```

## 주요 함수

| 함수 | 설명 |
|------|------|
| `system_clock_init()` | 시스템 클럭 초기화 (HSI 8MHz) |
| `uart_init()` | UART 초기화 (보드별 USART1/USART2) |
| `gpio_init()` | GPIO 핀 초기화 |
| `timer_init()` | PWM 타이머 초기화 |
| `uart_rx_handler()` | UART 수신 처리 |
| `parse_motor_command()` | 명령 문자열 파싱 |
| `set_motor_speed()` | 모터 속도 제어 |
| `on_motor_speed_changed()` | 상태 변경 콜백 |