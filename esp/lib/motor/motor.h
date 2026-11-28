#ifndef motor.h

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>

#include "driver/pulse_cnt.h"

#include "pinos.h"

typedef enum { // agrupa os motores 
    LEFT,
    RIGHT
} motor_side_t;

#define LOW  0
#define HIGH 1

// Configuração do timer PWM
#define LEDC_MODE       LEDC_LOW_SPEED_MODE // são variaveis prontas do ESP IDF que voce escolhe com base no seu objetivo, essa é a mais adequada p/ motor
#define LEDC_TIMER      LEDC_TIMER_0 // canal q cronometraa o tempo, tem 4(0,1,2,3), escolheu o primeiro vazio
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT   // 0–1023, 10 bits, valor de 0 - 100% q o motor funciona
#define LEDC_FREQUENCY  5000                 // Hz, dita a frequencia q o duty muda


/* Macro functions */
#define MOTOR_INPUT_1(MOTOR) MOTOR == (LEFT) ? INPUT_LEFT_1 : INPUT_RIGHT_1 // peg um pino de output pro motor e abre a chave da direita ou da esquerda (da ponte h)
#define MOTOR_INPUT_2(MOTOR) MOTOR == (LEFT) ? INPUT_LEFT_2 : INPUT_RIGHT_2 // abre o pino oposto pra fazer o fluxo, esse fluxo define se vai pra frente ou pra tras
#define MOTOR_CHANNEL(MOTOR) MOTOR == (LEFT) ? LEDC_CHANNEL_LEFT : LEDC_CHANNEL_RIGHT // retorna o canal do motor



/* Functions */
void init_gpio();

void init_pwm();

esp_err_t update_motor();

esp_err_t _set_forward(motor_side_t motor);

esp_err_t _set_backward(motor_side_t motor);

#endif