#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_cortex.h"

typedef enum {
    I2C_RESULT_OK,
    I2C_RESULT_BUS_ERROR,
    I2C_RESULT_ACK_FAILURE,
    I2C_RESULT_ARBITRATION_LOST,
    I2C_RESULT_OVERRUN,
    I2C_RESULT_TIMEOUT
  } i2c_result_t;

i2c_result_t i2c_read(I2C_TypeDef* i2c_dev, uint8_t slave_addr, uint8_t reg_addr, uint8_t* read_buf, uint8_t read_size);
i2c_result_t i2c_write(I2C_TypeDef* i2c_dev, uint8_t slave_addr, uint8_t reg_addr, uint8_t* write_buf, uint8_t write_size);
void i2c_print_error(i2c_result_t result);