#include "motor.h"

void init_gpio()
{
    
    gpio_set_direction(STAND_BY, GPIO_MODE_OUTPUT);
    gpio_set_level(STAND_BY, HIGH);

    // Motor 1:
    gpio_set_direction(INPUT_LEFT_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(INPUT_LEFT_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEDC_OUTPUT_LEFT, GPIO_MODE_OUTPUT);
    
    // Motor 2:
    gpio_set_direction(INPUT_RIGHT_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(INPUT_RIGHT_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LEDC_OUTPUT_RIGHT, GPIO_MODE_OUTPUT);
    
}

void init_pwm()
{
    // Configurar o timer PWM. Alunos devem verificar e completar os parâmetros.
    ledc_timer_config_t ledc_timer = { // estabelece as configurações
        .speed_mode       = LEDC_MODE, // Definir o modo de operação(LOw ou High, LOW é mais adequada pra motor)
        .timer_num        = LEDC_TIMER, // Selecionar o timer (0, pois nenhum ta sendo usado)
        .duty_resolution  = LEDC_DUTY_RES, // Definir a resolução do duty (10 bits é o padrao pra motores?)
        .freq_hz          = LEDC_FREQUENCY, // Definir a frequência ( 1kHz tmb é o padrão pra um motor suave)
        .clk_cfg          = LEDC_AUTO_CLK // Configuração do clock
    };

    ledc_timer_config(&ledc_timer); // efetiva a mudança

    ledc_channel_config_t ledc_left_channel = { // configura o motor usando o mesmo relogio p/ td ser sinconizado
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_LEFT,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_LEFT,
        .duty = 0,  // Set duty to 0%
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_left_channel);  // efetiva a mudança

    ledc_channel_config_t ledc_right_channel = { // configura o motor usando o mesmo relogio p/ td ser sinconizado
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_RIGHT,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE, // desativa o aviso de q acabou o ciclo do Duty
        .gpio_num = LEDC_OUTPUT_RIGHT,
        .duty = 0,  // Set duty to 0%
        .hpoint = 0
    };
    ledc_channel_config(&ledc_right_channel); // efetiva a mudança
}

esp_err_t update_motor(motor_side_t motor, int u)
{
    if(u > 1023){
        u = 1023;
    } else if(u < -1023){
        u = -1023;
    }

    if(u > 0){ // pra frente
        _set_forward(motor);
    } else {
        _set_backward(motor);
    }
    u = abs(u);
    ledc_set_duty(LEDC_MODE, MOTOR_CHANNEL(motor), u); // define  o sentido q o motor deve tomar e a intensidade(u)
    ledc_update_duty(LEDC_MODE, MOTOR_CHANNEL(motor)); // da "upload" nas definiçoes feitas

    return ESP_OK;
}

esp_err_t _set_forward(motor_side_t motor)
{
    gpio_set_level(MOTOR_INPUT_1(motor), LOW);
    gpio_set_level(MOTOR_INPUT_2(motor), HIGH);

    return ESP_OK;
}

esp_err_t _set_backward(motor_side_t motor)
{
    gpio_set_level(MOTOR_INPUT_1(motor), HIGH);
    gpio_set_level(MOTOR_INPUT_2(motor), LOW);


    return ESP_OK;
}