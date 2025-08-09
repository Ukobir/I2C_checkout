#ifndef BMP280_H
#define BMP280_H

#include "hardware/i2c.h"

// Defina os endereços e registros conforme o código original
#define ADDR_R _u(0xC7)
#define ADDR_W _u(0xC6)

#define REG_VERSION _u(0x00)
#define REG_VCELL_H _u(0x02)
#define REG_VCELL_L _u(0x03)
#define REG_SOC_H _u(0x04)
#define REG_SOC_L _u(0x05)
#define REG_TEMP _u(0x06)
#define REG_CONFIG _u(0x08)
#define REG_INT_CONF _u(0x0A)
#define REG_SOC_ALERT _u(0x0B)
#define REG_TEMP_MAX _u(0x0C)
#define REG_TEMP_MIN _u(0x0D)
#define REG_VOLT_ID_H _u(0x0E)
#define REG_VOLT_ID_L _u(0x0F)
#define REG_HOST_H _u(0xA0)
#define REG_HOST_L _u(0xA1)

void cw2017_init(i2c_inst_t *i2c);
float cw2017_read_vcell(i2c_inst_t *i2c);
// Função para ler a temperatura
float cw2017_read_temp(i2c_inst_t *i2c);


#endif
