#include "encoder.h"

pcnt_unit_handle_t selected_encoder_L;
pcnt_unit_handle_t selected_encoder_R;

static const char *TAG = "ENCODER";

pcnt_unit_handle_t init_encoder(encoder_side_t side){ 
    pcnt_unit_handle_t side_handler = NULL;

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_channel_handle_t pcnt_chan_b = NULL;

    //Definição da configuração da unidade com os limites máximos e mínimos de contagem dos encoders
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &side_handler));

    //Definição do filtro em nanosegundos
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(side_handler, &filter_config));//Configuração filtro

    //Definição da configuração do canal a do encoder
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_INPUT_A(side), //Pino do canal a do encoder esquerdo
        .level_gpio_num = ENCODER_INPUT_B(side), //Pino do canal b do encoder esquerdo
    };

    //Configuração do canal
    ESP_ERROR_CHECK(pcnt_new_channel(side_handler, &chan_a_config, &pcnt_chan_a));

    
    //Definição da configuração do canal b do encoder
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_INPUT_B(side),
        .level_gpio_num = ENCODER_INPUT_A(side),
    };
    ESP_ERROR_CHECK(pcnt_new_channel(side_handler, &chan_b_config, &pcnt_chan_b));
    
    //Configuração do canal

    //configura as açoes do encoder
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    

    //Inicia o hardware PCNT
    ESP_ERROR_CHECK(pcnt_unit_enable(side_handler)); 
    ESP_ERROR_CHECK(pcnt_unit_clear_count(side_handler)); 
    ESP_ERROR_CHECK(pcnt_unit_start(side_handler));

    return side_handler;
}

int get_encoder_vel(pcnt_unit_handle_t handler){ 
    int count = 0;

    ESP_ERROR_CHECK(pcnt_unit_get_count(handler, &count)); 
    pcnt_unit_clear_count(handler); 
    return count;
}

int get_encoder_position(pcnt_unit_handle_t handler){ 
    int count = 0; 
    ESP_ERROR_CHECK(pcnt_unit_get_count(handler, &count)); 
    return count;
}