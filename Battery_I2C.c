#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "lib/CW2017.h"

// Definições dos pinos I2C
#define I2C_PORT_BATTERY i2c0
#define I2C0_SDA_PIN 0  // GPIO0 para I2C0 SDA
#define I2C0_SCL_PIN 1  // GPIO1 para I2C0 SCL


// Velocidade do I2C (100 kHz)
#define I2C_BAUDRATE 100000

void scan_i2c_bus(i2c_inst_t *i2c, const char *bus_name) {
    printf("\nEscaneando barramento %s...\n", bus_name);
    bool found = false;
    
    // Escaneia endereços de 0x08 a 0x77 (endereços válidos de 7 bits)
    for (uint8_t addr = 8; addr <= 0x77; addr++) {
        uint8_t buffer;
        // Tenta ler um byte do endereço atual
        int ret = i2c_read_blocking(i2c, addr, &buffer, 1, false);
        
        // Se a leitura for bem-sucedida (ou seja, dispositivo responde), exibe o endereço
        if (ret >= 0) {
            printf("Dispositivo encontrado no endereço: 0x%02X\n", addr);
            found = true;
        }
    }
    
    if (!found) {
        printf("Nenhum dispositivo I2C encontrado no %s.\n", bus_name);
    }
}


// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

int main()
{
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
//Fim do modo BOOTSEL

    



    // Inicializa a UART para saída
    stdio_init_all();
    
    // Aguarda um momento para garantir que a UART esteja pronta
    sleep_ms(2000);
    printf("Iniciando scanner I2C para Raspberry Pi Pico\n");

    // Inicializa I2C0
    i2c_init(i2c0, I2C_BAUDRATE);
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_PIN);
    gpio_pull_up(I2C0_SCL_PIN);

  

    while (true) {
        // Escaneia ambos os barramentos I2C
        scan_i2c_bus(i2c0, "I2C0");
        
        
        printf("\nEscaneamento concluído. Aguardando 5 segundos antes do próximo escaneamento...\n");
        sleep_ms(5000);
    }

    return 0;
}