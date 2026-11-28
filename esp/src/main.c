    #include <stdio.h>
    #include "motor.h"
    #include "encoder.h"

    void app_main(void)
    {
        init_gpio();
        init_pwm();

        pcnt_unit_handle_t encoder_left = init_encoder(ENC_LEFT);
        pcnt_unit_handle_t encoder_right = init_encoder(ENC_RIGHT);
        
        //motor da esquerda
        update_motor(LEFT, 400);
        // motor da direita
        update_motor(RIGHT, 400);
        
        while(1){
            vTaskDelay(pdMS_TO_TICKS(100)); 
            int left_tick = get_encoder_vel(encoder_left);
            int right_tick = get_encoder_vel(encoder_right);

            printf("Ticks-> L: %d | R: %d\n", left_tick, right_tick);
            //vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }