#include "CW2017.h"
#include "hardware/i2c.h"

//Inicia o CI no modo normal se iniciar novamente restarta
//É necessário sempre iniciar o CI antes de ler ou escrever
void cw2017_init(i2c_inst_t *i2c){
    uint8_t buf[2];
    buf[0] = REG_CONFIG;  
    buf[1] = 0x30; // limpa a memória
    i2c_write_blocking(i2c, ADDR_W, buf, 2, false);
    sleep_ms(100);
    buf[1] = 0x00; // faz o reset
    i2c_write_blocking(i2c, ADDR_W, buf, 2, false);
}

float cw2017_read_vcell(i2c_inst_t *i2c) {
    uint8_t reg_addr = REG_VCELL_H;
    uint8_t vcell_data[2];
    uint16_t vcell_raw;

    i2c_write_blocking(i2c, ADDR_R, &reg_addr, 1, true);
    i2c_read_blocking(i2c, ADDR_R, vcell_data, 2, false);

    // Combina os 2 bytes em um valor de 14 bits
    vcell_raw = (vcell_data[0] << 8) | vcell_data[1];
    vcell_raw &= 0x3FFF;

    return (float)vcell_raw * 312.5 / 1000000;
}

// Função para ler a temperatura
float cw2017_read_temp(i2c_inst_t *i2c) {
    uint8_t reg_addr = REG_TEMP;
    uint8_t temp_data[1];
    uint8_t temp_raw;

    i2c_write_blocking(i2c, ADDR_R, &reg_addr, 1, true);
    i2c_read_blocking(i2c, ADDR_R, temp_data, 1, false);

    temp_raw = temp_data[0];

    return -40.0 + (float)temp_raw / 2.0;
}


