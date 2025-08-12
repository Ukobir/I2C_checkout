#include "CW2017.h"
#include "hardware/i2c.h"

// Inicia o CI no modo normal se iniciar novamente restarta
// É necessário sempre iniciar o CI antes de ler ou escrever
void cw2017_init(i2c_inst_t *i2c)
{
    uint8_t buf[2];
    buf[0] = REG_CONFIG;
    buf[1] = 0x30; // limpa a memória
    i2c_write_blocking(i2c, CW2017_ADDR, buf, 2, false);
    sleep_ms(100);
    buf[1] = 0x00; // faz o reset
    i2c_write_blocking(i2c, CW2017_ADDR, buf, 2, false);
}

float cw2017_read_vcell(i2c_inst_t *i2c)
{
    uint8_t reg_addr = REG_VCELL_H;
    uint8_t vcell_data[2];
    uint16_t vcell_raw;

    i2c_write_blocking(i2c, CW2017_ADDR, &reg_addr, 1, true);
    i2c_read_blocking(i2c, CW2017_ADDR, vcell_data, 2, false);

    // Combina os 2 bytes em um valor de 14 bits
    vcell_raw = (vcell_data[0] << 8) | vcell_data[1];
    vcell_raw &= 0x3FFF;

    return (float)vcell_raw * 312.5 / 1000000;
}

// Função para ler a temperatura
float cw2017_read_temp(i2c_inst_t *i2c)
{
    uint8_t reg_addr = REG_TEMP;
    uint8_t temp_data[1];
    uint8_t temp_raw;

    i2c_write_blocking(i2c, CW2017_ADDR, &reg_addr, 1, true);
    i2c_read_blocking(i2c, CW2017_ADDR, temp_data, 1, false);

    temp_raw = temp_data[0];

    return -40.0 + (float)temp_raw / 2.0;
}

// Função para ler o Estado de Carga (SoC)
float cw2017_read_soc(i2c_inst_t *i2c)
{
    uint8_t reg_addr = REG_SOC_H;
    uint8_t soc_data[2];
    uint16_t soc_raw;

    // Escrita do endereço do registro a ser lido
    i2c_write_blocking(i2c, CW2017_ADDR, &reg_addr, 1, true);
    // Leitura de 2 bytes (REG_SOC_H e REG_SOC_L)
    i2c_read_blocking(i2c, CW2017_ADDR, soc_data, 2, false);

    // Combina os 2 bytes em um valor de 16 bits
    soc_raw = (soc_data[0] << 8) | soc_data[1];

    // Converte o valor bruto para porcentagem
    // A parte inteira é o byte superior, e a parte fracionária é o byte inferior / 256
    float soc_percent = (float)soc_data[0] + (float)soc_data[1] / 256.0;

    return soc_percent;
}

// Tamanho da memória da AT24CS02 em bytes (2 kbit = 2048 bits = 256 bytes)
const uint16_t EEPROM_SIZE_BYTES = 256;

// Função para imprimir os dados em formato de hexdump
void print_hexdump(i2c_inst_t *i2c)
{

    uint8_t data_read[EEPROM_SIZE_BYTES];
    // Para ler a partir do início, escrevemos o endereço 0x00 para o dispositivo.
    uint8_t start_mem_addr = 0x00;
    int write_result = i2c_write_blocking(i2c0, AT24CS02_ADDR, &start_mem_addr, 1, true);
    if (write_result == PICO_ERROR_GENERIC)
    {
        printf("Erro: Não foi possível comunicar com a EEPROM. Verifique as conexões e o endereço I2C.\n");
    }

    printf("Ponteiro de memória da EEPROM definido para 0x00. Lendo dados...\n");
    int read_result = i2c_read_blocking(i2c0, AT24CS02_ADDR, data_read, EEPROM_SIZE_BYTES, false);

    if (read_result != EEPROM_SIZE_BYTES)
    {
        printf("Erro: Falha ao ler os dados da EEPROM. Foram lidos %d de %d bytes.\n", read_result, EEPROM_SIZE_BYTES);
    }
    printf("---- Leitura da EEPROM (256 bytes) ----\n");
    printf("Addr:  00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
    printf("------------------------------------------------------\n");
    for (int i = 0; i < EEPROM_SIZE_BYTES; i += 16)
    {
        printf("%04X: ", i);
        for (int j = 0; j < 16; j++)
        {
            if (i + j < EEPROM_SIZE_BYTES)
            {
                printf("%02X ", data_read[i + j]);
            }
            else
            {
                printf("   ");
            }
        }
        printf("\n");
    }
    printf("-------------------- Fim ---------------------\n");
}
