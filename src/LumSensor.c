#include <stdio.h>
#include "pico/stdlib.h"
#include "init.h"
#include "bh1750.h"

int main() {
    initializeSystem(); // Inicia o sistema
    sleep_ms(2000);

    printf("Iniciando leitura de luminosidade...\n");
    i2c_inst_t *i2c = bh1750_init(i2c0, 0, 1); // Usa a instância 'i2c0' retornada pela função de inicialização

    while (1) {

        // Variáveis para armazenar o valor lido
        char lux_str[32];
        float lux_value;

        bh1750_read_lux(i2c, &lux_value); // Leitura do sensor BH1750 e armazenamento do valor lido
        sprintf(lux_str, "Luminosidade: %.2f", lux_value);

        updateTextLine(lux_str, 0, 10, 1, 128); 

        // Envia tudo para a tela
        ssd1306_show(&display);
        sleep_ms(1000);
    }
    return 0;
}