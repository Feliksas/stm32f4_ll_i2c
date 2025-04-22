#include <stdint.h>
#include <stdio.h>
#include "i2c.h"

#define I2C_TIMEOUT 25

static inline void i2c_reset(I2C_TypeDef* i2c_dev) {
    LL_I2C_GenerateStopCondition(i2c_dev);
    LL_I2C_Disable(i2c_dev);
    LL_I2C_Enable(i2c_dev);
  }
  
static inline i2c_result_t i2c_start(I2C_TypeDef* i2c_dev) {  
    LL_I2C_GenerateStartCondition(i2c_dev);
    (void)SysTick->CTRL; // reset timer

    uint8_t timeout = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_SB(i2c_dev)) {
        if (LL_SYSTICK_IsActiveCounterFlag()) {
            if (timeout-- == 0) {
                i2c_reset(i2c_dev);
                return I2C_RESULT_TIMEOUT;
            }
        }
    }
    return I2C_RESULT_OK;
}

static inline i2c_result_t i2c_check_tx_result(I2C_TypeDef* i2c_dev, uint32_t success_flag) {
    volatile uint32_t reg = 0;
    uint8_t timeout = I2C_TIMEOUT;

    (void)SysTick->CTRL; // reset timer
    while (1) {
        reg = i2c_dev->SR1;
        if (reg & I2C_SR1_BERR) {
            return I2C_RESULT_BUS_ERROR;
        }
        if (reg & I2C_SR1_AF) {
            LL_I2C_GenerateStopCondition(i2c_dev);
            return I2C_RESULT_ACK_FAILURE;
        }
        if (reg & I2C_SR1_ARLO) {
            LL_I2C_ClearFlag_ARLO(i2c_dev);
            return I2C_RESULT_ARBITRATION_LOST;
        }
        if (reg & success_flag) { // I2C_SR1_ADDR for address phase, I2C_SR1_TXE for data phase
            break;
        }
        if (LL_SYSTICK_IsActiveCounterFlag()) {
            if (timeout-- == 0) {
                i2c_reset(i2c_dev);
                return I2C_RESULT_TIMEOUT;
            }
        }
    }
    return I2C_RESULT_OK;
}

static inline i2c_result_t i2c_send_addr(I2C_TypeDef* i2c_dev, uint8_t addr) {
  LL_I2C_TransmitData8(i2c_dev, addr);
  return i2c_check_tx_result(i2c_dev, I2C_SR1_ADDR);
}

static inline i2c_result_t i2c_send_byte(I2C_TypeDef* i2c_dev, uint8_t* data) {
    LL_I2C_TransmitData8(i2c_dev, *data);
    return i2c_check_tx_result(i2c_dev, I2C_SR1_TXE);
}

static inline i2c_result_t i2c_recv_byte(I2C_TypeDef* i2c_dev, uint8_t* data) {
    LL_I2C_AcknowledgeNextData(i2c_dev, LL_I2C_ACK);
    (void)SysTick->CTRL; // reset timer

    uint8_t timeout = I2C_TIMEOUT;
    while (!LL_I2C_IsActiveFlag_RXNE(i2c_dev)) {
        if (LL_SYSTICK_IsActiveCounterFlag()) {
            if (timeout-- == 0) {
                i2c_reset(i2c_dev);
                return I2C_RESULT_TIMEOUT;
            }
        }
    }
    *data = LL_I2C_ReceiveData8(i2c_dev);
    return I2C_RESULT_OK;
}

i2c_result_t i2c_read(I2C_TypeDef* i2c_dev, uint8_t slave_addr, uint8_t reg_addr, uint8_t* read_buf, uint8_t read_size) {
    i2c_result_t ret = I2C_RESULT_OK;  

    if ((ret = i2c_start(i2c_dev)) != I2C_RESULT_OK) {
        return ret;
    }

    if ((ret = i2c_send_addr(i2c_dev, (slave_addr << 1))) != I2C_RESULT_OK) {
      return ret;
    } 

    (void)i2c_dev->SR2; // clear addr condition

    if ((ret = i2c_send_byte(i2c_dev, &reg_addr)) != I2C_RESULT_OK) {
      return ret;
    } 
    LL_I2C_GenerateStopCondition(i2c_dev);

    if ((ret = i2c_start(i2c_dev)) != I2C_RESULT_OK) {
      return ret;
    }
    
    if ((ret = i2c_send_addr(i2c_dev, ((slave_addr << 1) | 0x1))) != I2C_RESULT_OK) {
      return ret;
    } 

    (void)i2c_dev->SR2; // clear addr condition
    
    while(read_size) {
        if ((ret = i2c_recv_byte(i2c_dev, &read_buf[--read_size])) != I2C_RESULT_OK) {
        return ret;
       } 
    }
    LL_I2C_AcknowledgeNextData(i2c_dev, LL_I2C_NACK); // End of transfer
    LL_I2C_GenerateStopCondition(i2c_dev);
    
    return ret;
}

i2c_result_t i2c_write(I2C_TypeDef* i2c_dev, uint8_t slave_addr, uint8_t reg_addr, uint8_t* write_buf, uint8_t write_size) {
    uint8_t ret = I2C_RESULT_OK;
    if ((ret = i2c_start(i2c_dev)) != I2C_RESULT_OK) {
        return ret;
    }

    if ((ret = i2c_send_addr(i2c_dev, (slave_addr << 1))) != I2C_RESULT_OK) {
        return ret;
    } 
    
    (void)i2c_dev->SR2; // clear addr condition
    
    if ((ret = i2c_send_byte(i2c_dev, &reg_addr)) != I2C_RESULT_OK) {
        return ret;
    }

    while(write_size) {
        if ((ret = i2c_send_byte(i2c_dev, &write_buf[--write_size])) != I2C_RESULT_OK) {
            return ret;
        } 
    }
    LL_I2C_GenerateStopCondition(i2c_dev);
    return ret;
}

void i2c_print_error(i2c_result_t result) {
    switch (result){
        case I2C_RESULT_OK:
            return;
        case I2C_RESULT_BUS_ERROR:
            printf("ERR: I2C bus error\n");
            break;
        case I2C_RESULT_ACK_FAILURE:
            printf("ERR: I2C bus unexpected NACK received\n");
            break;
        case I2C_RESULT_ARBITRATION_LOST:
            printf("ERR: I2C bus arbitration lost\n");
            break;
        case I2C_RESULT_OVERRUN:
            printf("ERR: I2C bus overrun/underrun error\n");
            break;
        case I2C_RESULT_TIMEOUT:
            printf("ERR: I2C operation timed out\n");
            break;
        default:
            return;
      }
}