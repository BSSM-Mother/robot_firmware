#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

// 모터 제어 함수
void system_clock_init(void);
void uart_init(void);
void gpio_init(void);
void timer_init(void);
void uart_putchar(char c);
void uart_puts(const char *str);
void uart_rx_handler(void);
void parse_motor_command(const char *cmd);
void set_motor_speed(int left_speed, int right_speed);
void delay_ms(uint32_t ms);
void on_motor_speed_changed(int left_speed, int right_speed);

// 모터 상태 구조체
typedef struct {
    int left_speed;   // -255 ~ 255
    int right_speed;  // -255 ~ 255
} motor_state_t;

extern motor_state_t motor_state;

#endif // __MOTOR_H__
