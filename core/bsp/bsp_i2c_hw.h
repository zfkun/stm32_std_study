#ifndef __BSP_I2C_HW_H__
#define __BSP_I2C_HW_H__

void HwI2C_Init(void);
void HwI2C_Start(void);
void HwI2C_Stop(void);
void HwI2C_SendAddress(uint8_t addr, uint8_t dir, uint32_t event);
void HwI2C_SendData(uint8_t data, uint32_t event);
uint8_t HwI2C_ReceiveData(void);
void HwI2C_Ack(FunctionalState state);

#endif