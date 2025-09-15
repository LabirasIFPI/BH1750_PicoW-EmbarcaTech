# Sensor de Luminosidade - BH1750

[Datasheet de referência](https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf?srsltid=AfmBOor0MN1NJrlTPM749pTecZipNDhvWp_-7gtG8foZ7jLmrmAGu18P)

Esta documentação detalha o funcionamento e a aplicação de um módulo de software para ler dados de luminosidade do sensor BH1750 com o Raspberry Pi Pico via comunicação I2C.

## 1. Como Utilizar o Módulo do Sensor

Esta seção é um guia direto para integrar o driver do sensor BH1750 em um novo projeto.

### **1.1. Conexão de Hardware**

| Componente | Pino no Pico | Observação |
| --- | --- | --- |
| **Sensor VCC** | 3V3 (OUT) | Tensão de alimentação para o sensor. |
| **Sensor GND** | GND | Essencial. O terra (GND) deve ser comum entre o Pico e o sensor. |
| **Sensor SDA** | Qualquer pino I2C | Pino de dados. O padrão no módulo é o **GPIO0**. |
| **Sensor SCL** | Qualquer pino I2C | Pino de clock. O padrão no módulo é o **GPIO1**. |

> **Ponto Importante:** O sensor BH1750 utiliza o protocolo I2C. Certifique-se de usar pinos do Raspberry Pi Pico que sejam compatíveis com a função I2C e que o barramento (`i2c0` ou `i2c1`) seja inicializado corretamente no software.

### **1.2. Integração dos Arquivos**

- **bh1750.h** para o diretório de cabeçalhos (ex: `inc/`).
- **bh1750.c** para o diretório de código-fonte (ex: `src/`).

### **1.3. Configuração do Build (CMakeLists.txt)**

Para que o SDK do Pico compile seu projeto, adicione `bh1750.c` às fontes e a biblioteca `hardware_i2c` às dependências.

1. **Adicionar o arquivo fonte:**

    ```makefile
    add_executable(meu_projeto
        main.c
        src/bh1750.c
        # ... outros arquivos .c
    )
    ```

2. **Adicionar a biblioteca de hardware:**

    ```makefile
    target_link_libraries(meu_projeto
        pico_stdlib
        hardware_i2c
    )
    ```

### **1.4. Exemplo de Uso no Código**

Com os arquivos integrados e o build configurado, usar o sensor é simples:

```c
#include "pico/stdlib.h"
#include "bh1750.h" // Inclui a interface do nosso módulo
#include <stdio.h>

int main() {
    // Inicializações do sistema (ex: stdio_init_all())
    stdio_init_all();
    sleep_ms(2000);

    // 1. Inicializa o hardware I2C para o sensor
    // A função bh1750_init retorna a instância i2c para uso posterior
    i2c_inst_t *i2c = bh1750_init(i2c0, 0, 1);

    while(1) {
        // 2. Lê o valor de luminosidade em lux
        float lux_value;
        bh1750_read_lux(i2c, &lux_value);

        // Imprime o valor lido no terminal serial
        printf("Luminosidade: %.2f lux\\n", lux_value);

        sleep_ms(1000);
    }

    return 0;
}
```

### 2. Análise Técnica do Módulo do Sensor

### **2.1. Princípios de Comunicação com o BH1750 (I2C)**

O sensor BH1750 é controlado via protocolo **I2C (Inter-Integrated Circuit)**. Ele opera como um dispositivo "escravo" em um barramento de dois fios, respondendo às solicitações do "mestre" (o Raspberry Pi Pico).

- **Endereçamento:** O sensor possui um endereço I2C fixo (`0x23`) que o Pico usa para direcionar comandos.
- **Comandos:** O Pico envia comandos específicos para ligar o sensor, definir o modo de medição (ex: leitura contínua de alta resolução) e solicitar os dados de luminosidade.
- **Leitura de Dados:** Após a solicitação, o sensor disponibiliza 2 bytes (16 bits) de dados, que representam o nível de luz medido. O módulo de software é responsável por ler esses bytes e convertê-los para a unidade padrão, lux.

### **2.2. Arquivo de Cabeçalho `bh1750.h`**

```c
#ifndef BH1750_I2C_H
#define BH1750_I2C_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// bh1750 I2C address, isto utilizando o I2C 0 da BitDogLab
#define I2C0_SDA_PIN 0
#define I2C0_SCL_PIN 1

// Function prototypes
i2c_inst_t* bh1750_init(i2c_inst_t *i2c_instance, uint sda_pin, uint scl_pin);
void bh1750_set_mode(i2c_inst_t *i2c, uint8_t mode);
void bh1750_read_lux(i2c_inst_t *i2c, float *lux);

#endif // BH1750_I2C_H
```

Este arquivo é a interface pública do módulo. Ele define:

- **`I2C0_SDA_PIN` e `I2C0_SCL_PIN`**: Os pinos GPIO padrão para a comunicação I2C com o sensor.
- **Protótipos das Funções**:
  - `i2c_inst_t* bh1750_init(...)`: Prepara o barramento I2C e configura o sensor para a operação.
  - `void bh1750_read_lux(...)`: Executa a leitura dos dados do sensor e os converte para lux.

### **2.3. Implementação `bh1750.c`**

```c
#include "bh1750.h"

// --- Constantes do BH1750 ---
const uint8_t BH1750_ADDR = 0x23;
const uint8_t BH1750_POWER_ON = 0x01;
const uint8_t BH1750_CONT_HIGH_RES_MODE = 0x10;

// Configuração do I2C 0 da placa e do sensor
i2c_inst_t* bh1750_init(i2c_inst_t *i2c_instance, uint sda_pin, uint scl_pin) {
    printf("Configurando BH1750 no I2C...\n");

    // 1. Inicializa o barramento I2C com a instância e velocidade desejada
    i2c_init(i2c_instance, 100 * 1000);
    
    // 2. Define os pinos GPIO para as funções I2C
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    
    // 3. Ativa as resistências de pull-up internas
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    // 4. Configura o sensor BH1750
    bh1750_set_mode(i2c_instance, BH1750_POWER_ON);
    sleep_ms(10); // Pequeno delay para estabilização
    bh1750_set_mode(i2c_instance, BH1750_CONT_HIGH_RES_MODE);

    // BH1750_CONT_HIGH_RES_MODE -> É um modo de leitura com uma resolução de 0.1 lx, dessa forma o 
    // valor lido pode variar entre 0 e 65535, basicamente 
    // O BH1750 continua a medir a luz continuamente com precisão de 1 lux e 
    // mantém os resultados atualizados para quando solicitar
    
    // 5. Aguarda o tempo da primeira conversão
    sleep_ms(180);

    printf("BH1750 configurado e pronto para leitura.\n");

    // Retorna o ponteiro para a instância I2C para uso futuro na main
    return i2c_instance;
}

// A função para configuração flexível, aceitando a instância I2C como parâmetro
void bh1750_set_mode(i2c_inst_t *i2c, uint8_t mode) {
    i2c_write_blocking(i2c, BH1750_ADDR, &mode, 1, false);

    // BH1750_ADDR -> Endereço do sensor BH1750 no barramento I2C
}

// Leitura do sensor BH1750 
void bh1750_read_lux(i2c_inst_t *i2c, float *lux) {
    uint8_t buffer[2];

    // Lê 2 bytes (MSB e LSB) do sensor através do barramento I2C.
    i2c_read_blocking(i2c, BH1750_ADDR, buffer, 2, false);
    
    // Combina os dois bytes (MSB << 8 | LSB) em um único valor inteiro de 16 bits.
    uint16_t raw = (buffer[0] << 8) | buffer[1];
    
    // Converte o valor bruto para lux, conforme a fórmula do datasheet para o modo de alta resolução.
    // Fórmula: Lux = (Valor Medido) / 1.2
    *lux = raw / 1.2f;
}
```

- **`bh1750_init()`**: Esta função configura o periférico I2C do Pico.
    1. **Inicializa o I2C**: Chama `i2c_init()` para ativar o barramento I2C especificado com uma frequência de clock.
    2. **Configura os Pinos**: Associa os pinos GPIO às funções I2C (SDA/SCL) e ativa os resistores de pull-up internos.
    3. **Configura o Sensor**: Envia comandos via I2C para o endereço `0x23` para ligar o sensor e colocá-lo no modo de leitura contínua de alta resolução (`0x10`).
    4. **Aguarda a Primeira Conversão**: Inclui um delay para garantir que o sensor tenha tempo de realizar a primeira medição antes que o código principal tente ler um valor.
- **`bh1750_read_lux()`**: Esta função realiza a leitura dos dados.
    1. **Leitura I2C**: Chama `i2c_read_blocking()` para solicitar e receber 2 bytes de dados do endereço do sensor.
    2. **Conversão para Lux**: Combina os dois bytes recebidos em um único valor inteiro de 16 bits. Em seguida, divide esse valor bruto por 1.2, conforme a fórmula do datasheet do BH1750 para o modo de alta resolução, obtendo o valor final em lux.

> Documentação por [@Nicolas Rafael](https://github.com/NicolasRaf)