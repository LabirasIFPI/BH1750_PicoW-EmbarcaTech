// init.c
#include "init.h"

void initDisplayI2C() {
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void initializeSystem(void) {
    stdio_init_all();
    sleep_ms(1000);

    // configura I2C em 400 kHz
    printf("Configurando I2C do OLED...\n");
    initDisplayI2C();

    printf("Iniciando SSD1306...\n");
    // inicializa display; se der erro, imprime e trava
    if ( initializeDisplay() != 0 ) {
        printf("Erro ao inicializar o SSD1306\n");
    }
  
    // tela limpa
    printf("Tela limpa.\n");
    clearScreen();
}


