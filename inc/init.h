// init.h
#ifndef INIT_H
#define INIT_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "display.h"

// inicializa os pinos e o barramento I2C
void initDisplayI2C();

// inicializa todo o sistema: stdio, I2C, display e sensor
void initializeSystem(void);

#endif // INIT_H
