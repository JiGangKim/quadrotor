/*
 * myperipheral.c - peripheral
 *
 *  Created on: 2016. 7. 18.
 *      Author: jgkim2020
 *
 * I do not take any responsibility for any consequences resulting from the
 * use of this software. USE AT YOUR OWN RISK!
 *
 */

void InitLED(void);
void InitParam(void);
int InitRotorPWM(uint32_t SysClkFreq, uint32_t PWMFreq);
void UART0IntHandler(void);
void XBEEIntHandler(void);
void InitUART0(uint32_t SysClkFreq, uint32_t baudrate); // PC
void InitXBEEUART(uint32_t SysClkFreq, uint32_t baudrate); // XBEE
void Timer1000HzIntHandler(void);
void InitTIMER1000Hz(uint32_t SysClkFreq);
void InitI2C0(uint32_t SysClkFreq);
void ADC0IntHandler(void);
void InitADC0(void);
void I2C_checkErr(uint32_t i2c_base, uint32_t uart_base, uint32_t line);
void I2CReceive(uint32_t i2c_base, uint32_t uart_base, uint32_t slave_addr, uint8_t reg, uint8_t num_of_read, uint8_t* data);
void I2CSend(uint32_t i2c_base, uint32_t uart_base, uint8_t slave_addr, uint8_t num_of_write, ...);
void ReadMPU6050(uint32_t i2c_base, uint8_t mode);
void ReadHMC5883L(uint32_t i2c_base);
float LifttoPWM(float lift);
void sensor_calibration(uint32_t SysClkFreq);
