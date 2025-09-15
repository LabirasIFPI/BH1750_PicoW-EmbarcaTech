#include "display.h"

ssd1306_t display; // Declara uma instância do display

// Inicializa o display
int initializeDisplay() {
    if (!ssd1306_init(&display, SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_ADDRESS, i2c1)) { 
        printf("Falha ao inicializar o display SSD1306\n"); 
        return 1;
    }

    return 0;
}

void showText(const char *texto, uint32_t x, uint32_t y, uint32_t scale){
    ssd1306_draw_string(&display, x, y, scale, texto); // Desenha o texto
    ssd1306_show(&display); // Atualiza a tela
}

void clearScreen(){
    ssd1306_clear(&display); // Limpa a tela
    ssd1306_show(&display); // Atualiza a tela 
}

void updateTextLine(const char* text, uint32_t x, uint32_t y, uint32_t scale, uint32_t clear_width) {
    // A altura da fonte padrão é 8 pixels
    uint32_t char_height = 8;

    // Limpa a área retangular onde o texto ficará
    ssd1306_clear_square(&display, x, y, clear_width, char_height * scale);

    // Desenha a nova string
    ssd1306_draw_string(&display, x, y, scale, text);
}