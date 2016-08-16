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

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include <math.h>
#include <string.h>

#include "driverlib/adc.h"
#include "driverlib/can.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"

#include "mymatrix.h"
#include "myuartstdio.h"
#include "myvariable.h"
#include "myquaternion.h"

void InitLED(void)
{
	SysCtlPeripheralEnable(LED_PORT);
	GPIOPinTypeGPIOOutput(LED_PORT, LED_R_PIN);
	GPIOPinTypeGPIOOutput(LED_PORT, LED_G_PIN);
	GPIOPinTypeGPIOOutput(LED_PORT, LED_B_PIN);
}

void InitParam(void)
{
	// Parameters for sensor readings
	raw_byte_a = raw_byte;
	raw_byte_w = raw_byte + 6;
	raw_byte_b = raw_byte + 12;
	raw_a = raw_data;
	raw_w = raw_data + 3;
	raw_b = raw_data + 6;
	raw_offset_a = raw_data_offset;
	raw_offset_w = raw_data_offset + 3;
	raw_offset_b = raw_data_offset + 6;
	a = data;
	w = data + 4;
	b = data + 8;
	euler = data + 12;
	quaternion = data + 15;
	quaternion_acc = data + 20;
	quaternion_mag = data + 24;
	offset_w[0] = -0.036192;
	offset_w[1] = -0.035898;
	offset_w[2] = 0.088110;
	quaternion_calib[0] = 0.999614;
	quaternion_calib[1] = 0.003334;
	quaternion_calib[2] = -0.027553;
	quaternion_calib[3] = 0.0;

	// Parameters for Kalman filter
	IdentityMatrix(H);
	IdentityMatrix(P); // estimate covariance
	IdentityMatrix(Q);
	ScalarXMatrix(0.004, Q, Q); // Q, covariance of the process noise 0.00025
	IdentityMatrix(R);
	ScalarXMatrix(3.0, R, R); // R, covariance of the observation noise 10.0
	x[2] = 1; x[3] = 0; x[4] = 0; x[5] = 0;

	// Parameters for PID
	euler_set[0] = 0;	euler_set[1] = 0;	euler_set[2] = 0;
}

int InitRotorPWM(uint32_t SysClkFreq, uint32_t PWMFreq)
{
	uint32_t PWMClkFreq, Load;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeGPIOOutput(ROTOR_POWER_PORT, ROTOR_POWER_PIN);
	GPIOPinWrite(ROTOR_POWER_PORT, ROTOR_POWER_PIN, 0);
	PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_2); // PWM clock frequency is 40MHz
	GPIOPinConfigure(GPIO_PE4_M0PWM4);
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinConfigure(GPIO_PB4_M0PWM2);
	GPIOPinConfigure(GPIO_PC4_M0PWM6);
	GPIOPinTypePWM(ROTOR1_PORT, ROTOR1_PIN);
	GPIOPinTypePWM(ROTOR2_PORT, ROTOR2_PIN);
	GPIOPinTypePWM(ROTOR3_PORT, ROTOR3_PIN);
	GPIOPinTypePWM(ROTOR4_PORT, ROTOR4_PIN);
	PWMClkFreq = SysClkFreq / 2; // PWM clock frequency is 40MHz
	Load = (PWMClkFreq / PWMFreq); // 40000000/1000 - 1 = 39999
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, Load);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, Load);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, Load);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, Load);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, Load/100.0*20);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, Load/100.0*20);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, Load/100.0*20);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, Load/100.0*20);
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	PWMGenEnable(PWM0_BASE, PWM_GEN_2);
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);
	return Load;
}

void UART0IntHandler(void)
{
	char flag;
	uint32_t intStatus = UARTIntStatus(UART0_BASE, false);
	UARTIntClear(UART0_BASE, intStatus);
	if(intStatus == UART_INT_RT || intStatus == UART_INT_RX)
	{
		flag = UARTCharGet(UART0_BASE);
		UARTCharPut(UART0_BASE, flag);
	}
}

void XBEEIntHandler(void)
{
	uint32_t intStatus = UARTIntStatus(UART1_BASE, false);
	UARTIntClear(UART1_BASE, intStatus);
	if(intStatus == UART_INT_RT || intStatus == UART_INT_RX)
	{
		XBEEIntflag = 1;
		XBEEIntmsg[XBEEIntmsg_pointer++] = UARTCharGet(UART1_BASE);
		if(XBEEIntmsg_pointer == XBEEIntmsg_size - 1) myUARTprintf(UART1_BASE, "Invalid Input! (Buffer Overflow)%n");
		else UARTCharPut(UART1_BASE, XBEEIntmsg[XBEEIntmsg_pointer - 1]);
	}
}

void InitUART0(uint32_t SysClkFreq, uint32_t baudrate)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SysClkFreq, baudrate, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	IntEnable(INT_UART0);
	UARTIntRegister(UART0_BASE, UART0IntHandler);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	IntMasterEnable();
	myUARTprintf(UART0_BASE, "## UART0 Initialization Complete ##%n");
}

void InitXBEEUART(uint32_t SysClkFreq, uint32_t baudrate)
{
	SysCtlPeripheralEnable(XBEE_POWER_PORT);
	GPIOPinTypeGPIOOutput(XBEE_POWER_PORT, XBEE_POWER_PIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralReset(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(XBEE_PORT, XBEE_RX_PIN | XBEE_TX_PIN);
	UARTConfigSetExpClk(UART1_BASE, SysClkFreq, baudrate, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	IntEnable(INT_UART1);
	UARTIntRegister(UART1_BASE, XBEEIntHandler);
	UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
	IntMasterEnable();
}

void Timer1000HzIntHandler(void)
{
	uint32_t timerIntStatus = TimerIntStatus(TIMER0_BASE, false);
	TimerIntClear(TIMER0_BASE, timerIntStatus);
	if(timerIntStatus == TIMER_TIMB_TIMEOUT)
	{
		Tick1000Hz++;
		if(Tick1000Hz == 1000)
		{
			Tick1000Hz = 0;
			seconds++;
		}
	}
}

void InitTIMER1000Hz(uint32_t SysClkFreq)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC);
	TimerPrescaleSet(TIMER0_BASE, TIMER_B, 1);
	TimerLoadSet(TIMER0_BASE, TIMER_B, SysClkFreq/1000/2);
	IntEnable(INT_TIMER0B);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
	TimerIntRegister(TIMER0_BASE, TIMER_B, Timer1000HzIntHandler);
	IntMasterEnable();
	TimerEnable(TIMER0_BASE, TIMER_B);
}

void ADC0IntHandler(void)
{
	ADCIntClear(ADC0_BASE, 3);
	ADCSequenceDataGet(ADC0_BASE, 3, &raw_v);
}

void InitADC0(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	IntEnable(INT_ADC0SS3);
	ADCIntRegister(ADC0_BASE, 3, ADC0IntHandler);
	ADCIntEnable(ADC0_BASE, 3);
	IntMasterEnable();
}

void InitI2C0(uint32_t SysClkFreq)
{
	// Enable I2C module 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

	// Reset module
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

	// Enable GPIO peripheral that contains I2C0
	SysCtlPeripheralEnable(I2C_PORT);

	// Configure the pin muxing for I2C0 functions on PB2 and PB3
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	// Select the I2C function for these pins
	GPIOPinTypeI2CSCL(I2C_PORT, I2C_SCL_PIN);
	GPIOPinTypeI2C(I2C_PORT, I2C_SDA_PIN);

    // Enable and initialize the I2C0 master module. Use the system clock for
    // the I2C0 module. The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
	I2CMasterInitExpClk(I2C0_BASE, SysClkFreq, false);

	// Clear I2C FIFOS
	HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void I2C_checkErr(uint32_t i2c_base, uint32_t uart_base, uint32_t line)
{
	int32_t error;
	error = I2CMasterErr(i2c_base);
	if(error != I2C_MASTER_ERR_NONE)
	{
		myUARTprintf(uart_base, "[%s:%d] error != I2C_MASTER_ERR_NONE: %d%n", __FILE__, line, error);
	}
}

void I2CReceive(uint32_t i2c_base, uint32_t uart_base, uint32_t slave_addr, uint8_t reg, uint8_t num_of_read, uint8_t* data)
{
	int i;
	//wait for MCU to finish transaction
	while(I2CMasterBusy(i2c_base));
	//specify that we are writing (a register address) to the slave device
    I2CMasterSlaveAddrSet(i2c_base, slave_addr, false);
    //specify register to be read
    I2CMasterDataPut(i2c_base, reg);
    //send control byte and register address byte to slave device
    I2CMasterControl(i2c_base, I2C_MASTER_CMD_BURST_SEND_START);
    //wait for MCU to finish transaction
    while(I2CMasterBusy(i2c_base));
    //check for errors
    I2C_checkErr(i2c_base, uart_base, __LINE__);
	//specify that we are going to read from slave device
	I2CMasterSlaveAddrSet(i2c_base, slave_addr, true);
	//if there is only one argument, we only need to use the single receive I2C function
	if(num_of_read == 1)
	{
		//send control byte and read from the register we specified
		I2CMasterControl(i2c_base, I2C_MASTER_CMD_SINGLE_RECEIVE);
		//wait for MCU to finish transaction
		while(I2CMasterBusy(i2c_base));
		//check for errors
		I2C_checkErr(i2c_base, uart_base, __LINE__);
		//read data pulled from the specified register
		*data = I2CMasterDataGet(i2c_base);
	}
	//otherwise, we start transmission of multiple bytes on the I2C bus
	else
	{
		//send BURST START control byte and read from the register we specified
		I2CMasterControl(i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_START);
		//wait for MCU to finish transaction
		while(I2CMasterBusy(i2c_base));
		//check for errors
		I2C_checkErr(i2c_base, uart_base, __LINE__);
		//read data pulled from the specified register
		*data = I2CMasterDataGet(i2c_base);
		for(i = 1; i < (num_of_read - 1); i++)
		{
			//send BURST CONT control byte and read from the register we specified
			I2CMasterControl(i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
			//wait for MCU to finish transaction
			while(I2CMasterBusy(i2c_base));
			//check for errors
			I2C_checkErr(i2c_base, uart_base, __LINE__);
			//read data pulled from the specified register
			*(data + i) = I2CMasterDataGet(i2c_base);
		}
		//send BURST FINISH control byte and read from the register we specified
		I2CMasterControl(i2c_base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		//wait for MCU to finish transaction
		while(I2CMasterBusy(i2c_base));
		//check for errors
		I2C_checkErr(i2c_base, uart_base, __LINE__);
		//read data pulled from the specified register
		*(data + num_of_read - 1) = I2CMasterDataGet(i2c_base);
	}
}

void I2CSend(uint32_t i2c_base, uint32_t uart_base, uint8_t slave_addr, uint8_t num_of_write, ...)
{
	int i;
	//wait for MCU to finish transaction
	while(I2CMasterBusy(i2c_base));
	// Tell the master module what address it will place on the bus when communicating with the slave.
	I2CMasterSlaveAddrSet(i2c_base, slave_addr, false);
    //stores list of variable number of arguments
    va_list vargs;
    //specifies the va_list to "open" and the last fixed argument so vargs knows where to start looking
    va_start(vargs, num_of_write);
    //put data to be sent into FIFO
    I2CMasterDataPut(i2c_base, va_arg(vargs, uint32_t));
    //if there is only one argument, we only need to use the single send I2C function
    if(num_of_write == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(i2c_base, I2C_MASTER_CMD_SINGLE_SEND);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(i2c_base));
        //check for errors
        I2C_checkErr(i2c_base, uart_base, __LINE__);
        //"close" variable argument list
        va_end(vargs);
    }
    //otherwise, we start transmission of multiple bytes on the I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(i2c_base, I2C_MASTER_CMD_BURST_SEND_START);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(i2c_base));
        //check for errors
        I2C_checkErr(i2c_base, uart_base, __LINE__);
        //send num_of_args-2 pieces of data, using the BURST_SEND_CONT command of the I2C module
        for(i = 1; i < (num_of_write - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(i2c_base, va_arg(vargs, uint32_t));
            //send next data that was just placed into FIFO
            I2CMasterControl(i2c_base, I2C_MASTER_CMD_BURST_SEND_CONT);
            // Wait until MCU is done transferring.
            while(I2CMasterBusy(i2c_base));
            //check for errors
            I2C_checkErr(i2c_base, uart_base, __LINE__);
        }
        //put last piece of data into I2C FIFO
        I2CMasterDataPut(i2c_base, va_arg(vargs, uint32_t));
        //send next data that was just placed into FIFO
        I2CMasterControl(i2c_base, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(i2c_base));
        //check for errors
        I2C_checkErr(i2c_base, uart_base, __LINE__);
        //"close" variable args list
        va_end(vargs);
    }
}

void ReadMPU6050(uint32_t i2c_base, uint8_t mode)
{
	if(mode)
	{
		I2CReceive(I2C0_BASE, UART1_BASE, MPU6050_ADDRESS, ACC_XOUT_H, 6, raw_byte_a);
		// sensor frame -> body frame
		raw_a[0] = -(raw_byte_a[2] << 8) | raw_byte_a[3];
		raw_a[1] = -(raw_byte_a[0] << 8) | raw_byte_a[1];
		raw_a[2] = (raw_byte_a[4] << 8) | raw_byte_a[5];
		a[0] = raw_a[0]/16384.0*9.8;
		a[1] = raw_a[1]/16384.0*9.8;
		a[2] = raw_a[2]/16384.0*9.8;
		a[3] = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
	}
	else
	{
		I2CReceive(I2C0_BASE, UART1_BASE, MPU6050_ADDRESS, GYRO_XOUT_H, 6, raw_byte_w);
		// sensor frame -> body frame
		raw_w[0] = (raw_byte_w[2] << 8) | raw_byte_w[3];
		raw_w[1] = (raw_byte_w[0] << 8) | raw_byte_w[1];
		raw_w[2] = -(raw_byte_w[4] << 8) | raw_byte_w[5];
		w[0] = raw_w[0]/131.0*PI/180;
		w[1] = raw_w[1]/131.0*PI/180;
		w[2] = raw_w[2]/131.0*PI/180;
		w[3] = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
	}
}

void ReadHMC5883L(uint32_t i2c_base)
{
	I2CReceive(I2C0_BASE, UART1_BASE, HMC5883L_ADDRESS, MAG_XOUT_H, 6, raw_byte_b);
	// sensor frame -> body frame
	raw_b[0] = (raw_byte_b[4] << 8) | raw_byte_b[5];
	raw_b[1] = (raw_byte_b[0] << 8) | raw_byte_b[1];
	raw_b[2] = -(raw_byte_b[2] << 8) | raw_byte_b[3];
	b[0] = raw_b[0]/1.52;
	b[1] = raw_b[1]/1.52;
	b[2] = raw_b[2]/1.52;
	b[3] = sqrtf(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
}

float LifttoPWM(float lift) // lift in gf (gram force), returns PWM duty in percent
{
	if(0.115*lift + 9.3829 < 70) return 0.115*lift + 9.3829;
	else return 70.0;
}

void sensor_calibration(uint32_t SysClkFreq)
{
	int i;
	myUARTprintf(UART1_BASE, ">> Place the quad on a level surface...%n");
	myUARTprintf(UART1_BASE, ">> Calibrating in 3 seconds...%n");
	SysCtlDelay(SysClkFreq);
	myUARTprintf(UART1_BASE, ">> Starting calibration...%n");
	for(i = 0; i < 3; i++) offset_w[i] = 0.0;
	for(i = 0; i < 3; i++) offset_a[i] = 0.0;
	i = 500;
	while(i)
	{
		if(Tick1000Hz%OPERATING_PERIOD == 0)
		{
			ReadMPU6050(I2C0_BASE, 1); // ACC
			offset_a[0] = offset_a[0] + a[0];
			offset_a[1] = offset_a[1] + a[1];
			offset_a[2] = offset_a[2] + a[2];
			i--;
		}
	}
	for(i = 0; i < 3; i++) offset_a[i] *= 0.002;

	cos_calib = fabs(offset_a[2])/sqrtf(offset_a[0]*offset_a[0] + offset_a[1]*offset_a[1] + offset_a[2]*offset_a[2]);
	quaternion_calib[0] = sqrt(0.5*(1 + cos_calib));
	quaternion_calib[1] = sqrt(0.5*(1 - cos_calib))*offset_a[1]/sqrtf(offset_a[0]*offset_a[0] + offset_a[1]*offset_a[1]);
	quaternion_calib[2] = -sqrt(0.5*(1 - cos_calib))*offset_a[0]/sqrtf(offset_a[0]*offset_a[0] + offset_a[1]*offset_a[1]);
	quaternion_calib[3] = 0.0;

	myUARTprintf(UART1_BASE, ">> ACC recent value: %l %l %l%n", a[0], a[1], a[2]);
	myUARTprintf(UART1_BASE, ">> ACC offset: %l %l %l%n", offset_a[0], offset_a[1], offset_a[2]);
	Quaternion_qvq(quaternion_calib, offset_a);
	myUARTprintf(UART1_BASE, ">> ACC offset correction: %l %l %l%n", offset_a[0], offset_a[1], offset_a[2]);
	myUARTprintf(UART1_BASE, ">> quaternion_calib: %l %l %l %l%n", quaternion_calib[0], quaternion_calib[1], quaternion_calib[2], quaternion_calib[3]);

	i = 500;
	while(i)
	{
		if(Tick1000Hz%OPERATING_PERIOD == 0)
		{
			ReadMPU6050(I2C0_BASE, 0); // GYRO
			Quaternion_qvq(quaternion_calib, w);
			offset_w[0] = offset_w[0] + w[0];
			offset_w[1] = offset_w[1] + w[1];
			offset_w[2] = offset_w[2] + w[2];
			i--;
		}
	}
	for(i = 0; i < 3; i++) offset_w[i] *= 0.002;

	myUARTprintf(UART1_BASE, ">> GYRO recent value: %l %l %l%n", w[0], w[1], w[2]);
	myUARTprintf(UART1_BASE, ">> GYRO offset: %l %l %l%n", offset_w[0], offset_w[1], offset_w[2]);
	myUARTprintf(UART1_BASE, ">> Calibration complete%n");
}
