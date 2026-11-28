#ifndef encoder.h

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/pulse_cnt.h"

#include "pinos.h"

/* Enum */
typedef enum {
    ENC_LEFT = 0,
    ENC_RIGHT = 1
} encoder_side_t;

//PCNT limites
#define PCNT_HIGH_LIMIT 10000
#define PCNT_LOW_LIMIT  -10000




/* Macro functions */ //redireciona para os pinos com=nforme o motor q esta sendo tratado
#define ENCODER_INPUT_A(NUM) NUM == (ENC_RIGHT) ? CHA_ENCODER_1R : CHA_ENCODER_1L
#define ENCODER_INPUT_B(NUM) NUM == (ENC_RIGHT) ? CHA_ENCODER_2R : CHA_ENCODER_2L

extern pcnt_unit_handle_t selected_encoder_L; // A unidade (o contador em si) // extern pq se nn ele define outra vez qnd eu chamar o arquivo ddnv?
extern pcnt_unit_handle_t selected_encoder_R; // A unidade (o contador em si) // extern pq se nn ele define outra vez qnd eu chamar o arquivo ddnv?

int get_encoder_vel(pcnt_unit_handle_t handler); 
int get_encoder_position(pcnt_unit_handle_t handler);

pcnt_unit_handle_t init_encoder(encoder_side_t side);

#endif