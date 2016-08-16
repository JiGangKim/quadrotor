/******************************************************************************
 *
 * 2016_myQUAD
 * Quadrotor Attitude Control
 *
 * main.c
 *
 * CC-BY-SA (c) 2016 JiGang Kim
 *
 * TARGET MCU IS TM4C123GH6PM. USE AT YOUR OWN RISK!
 *
 * IN NO EVENT SHALL THE COPYRIGHT OWNER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * REFERENCES:
 *
 ******************************************************************************/

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include <math.h>
#include <string.h>

#include "driverlib/adc.h"
#include "driverlib/can.h"
#include "driverlib/debug.h"
//#include "driverlib/fpu.h"
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

#include "utils/mycontroller.h"
#include "utils/mykalman.h"
#include "utils/mymatrix.h"
#include "utils/myparser.h"
#include "utils/myperipheral.h"
#include "utils/myquaternion.h"
#include "utils/myuartstdio.h"
#include "utils/myvariable.h"

int main(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	uint32_t ui32SysClkFreq = SysCtlClockGet();

	InitUART0(ui32SysClkFreq, 115200);
	InitXBEEUART(ui32SysClkFreq, 115200);
	GPIOPinWrite(XBEE_POWER_PORT, XBEE_POWER_PIN, XBEE_POWER_PIN);
	SysCtlDelay(ui32SysClkFreq/30);
	XBEEIntmsg_pointer = 0;
	XBEEIntflag = 0;
	myUARTprintf(UART1_BASE, ">> XBEEUART initialization complete%n");
	CUIHandler("Y", ">> Type 'Y' to proceed to initialization...%n", 0);
	myUARTprintf(UART1_BASE, ">> Executing peripheral initialization...%n");
	//FPULazyStackingEnable();
	//myUARTprintf(UART1_BASE, ">> FPU lazy stacking enabled%n");
	InitTIMER1000Hz(ui32SysClkFreq);
	myUARTprintf(UART1_BASE, ">> TIMER initialization complete%n");
	InitI2C0(ui32SysClkFreq);
	myUARTprintf(UART1_BASE, ">> I2C initialization complete%n");
	I2CSend(I2C0_BASE, UART1_BASE, MPU6050_ADDRESS, 2, MPU6050_PWR, 0x00); // MPU6050 Power ON
	I2CSend(I2C0_BASE, UART1_BASE, MPU6050_ADDRESS, 2, MPU6050_ACC_CONFIG, 0x00); // MPU6050 16384 LSB/g
	I2CSend(I2C0_BASE, UART1_BASE, MPU6050_ADDRESS, 2, MPU6050_GYRO_CONFIG, 0x00); // MPU6050 131 LSB/dps
	myUARTprintf(UART1_BASE, ">> MPU6050 initialization complete (Power On, 16384 LSB/g, 131 LSB/dps)%n");
	I2CSend(I2C0_BASE, UART1_BASE, HMC5883L_ADDRESS, 2, HMC5883L_CONFIG_REGA, 0x18); // HMC5883L 75Hz Mode
	I2CSend(I2C0_BASE, UART1_BASE, HMC5883L_ADDRESS, 2, HMC5883L_CONFIG_REGB, 0x60); // HMC5883L 1.52 mG/LSB
	I2CSend(I2C0_BASE, UART1_BASE, HMC5883L_ADDRESS, 2, HMC5883L_MODE_REG, 0x00); // HMC5883L Continuous Mode
	myUARTprintf(UART1_BASE, ">> HMC5883L initialization complete (Continuous Mode, 75Hz Mode, 1.52 mG/LSB)%n");
	InitADC0();
	myUARTprintf(UART1_BASE, ">> ADC initialization complete%n");
	PWMLoad = InitRotorPWM(ui32SysClkFreq, PWMFreq);
	myUARTprintf(UART1_BASE, ">> PWM initialization complete%n");
	InitParam();
	myUARTprintf(UART1_BASE, ">> Parameter initialization complete%n");
	if(CUIHandler("C", ">> Press 'C' to calibrate sensor. Press any key to skip...%n", 1)) sensor_calibration(ui32SysClkFreq);

	XBEEIntflag = 0;
	XBEEIntmsg_pointer = 0;
	while(1)
	{
		STATUS_CTRL_h = 0;
		STATUS_CTRL_v = 0;
		STATUS_CTRL_p = 0;
		STATUS_CTRL_X = 0;
		STATUS_CTRL_w = 0;
		STATUS_CTRL_s = 0;
		STATUS_CTRL_ad = -1;
		STATUS_CTRL_58 = -1;
		STATUS_CTRL_46 = -1;

		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWMLoad*0);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWMLoad*0);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMLoad*0);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMLoad*0);
		CUIHandler("ARM", ">>Type 'ARM' to arm rotors...%n", 0);
		GPIOPinWrite(ROTOR_POWER_PORT, ROTOR_POWER_PIN, ROTOR_POWER_PIN);
		SysCtlDelay(ui32SysClkFreq/3.0);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWMLoad*0.2);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWMLoad*0.2);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMLoad*0.2);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMLoad*0.2);
		SysCtlDelay(ui32SysClkFreq/3.0);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWMLoad*0.0);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWMLoad*0.0);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMLoad*0.0);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMLoad*0.0);
		myUARTprintf(UART1_BASE, ">> Rotors are armed!%n");
		CUIHandler("LOOP", ">> Type 'LOOP' to start loop...%n", 0);
		myUARTprintf(UART1_BASE, ">> Type 'h' for help. Executing loop...%n");


		while(STATUS_loop)
		{
			if(Tick1000Hz%OPERATING_PERIOD == 0)
			{
				TickLogStart = Tick1000Hz;
				ReadMPU6050(I2C0_BASE, 0); // GYRO
				Quaternion_qvq(quaternion_calib, w);
				w[0] = w[0] - offset_w[0];
				w[1] = w[1] - offset_w[1];
				w[2] = w[2] - offset_w[2];
				ReadMPU6050(I2C0_BASE, 1); // ACC
				Quaternion_qvq(quaternion_calib, a);
				if(Tickloop%4 == 0) ReadHMC5883L(I2C0_BASE); // MAG
				//ADCProcessorTrigger(ADC0_BASE, 3);
				//myUARTprintf(UART0_BASE, "acceleration %l %l %l %l %n", a[0], a[1], a[2], a[3]);
				//myUARTprintf(UART0_BASE, "rate of rotation %l %l %l %n", w[0], w[1], w[2]);
				//myUARTprintf(UART0_BASE, "magnetic field %l %l %l %n", b[0], b[1], b[2]);
				//myUARTprintf(UART0_BASE, "Voltage: %l%n", ((float)raw_v)*3.3/4096.0*6.1);
				KALMAN_ACCMAGGYRO();
				//myUARTprintf(UART0_BASE, "%f %f %f %d%n", eulerK[0]*180.0/PI, eulerK[1]*180.0/PI, eulerK[2]*180.0/PI, Tick1000Hz);
				PDPController();
				//myUARTprintf(UART0_BASE, "%f %f %f %f %d%n", w_req[0], w_req[1], w_req[2], angle_delta*180.0/PI, Tick1000Hz);
				//myUARTprintf(UART1_BASE, "Tickloop: %d Tick1000Hz: %d%n", Tickloop, Tick1000Hz);

				if(XBEEIntflag == 1 && Tickloop%4 == 0) // execute only when xbeeuart interrupt occurs with polling frequency of 50Hz
				{
					XBEEIntflag = 0;
					XBEEIntmsg_pointer = 0;
					myUARTprintf(UART1_BASE, "%n");
					switch(XBEEIntmsg[0])
					{
					case 'h' : // help
						STATUS_CTRL_h = 1;
						break;
					case 'v' : // toggle view data
						STATUS_CTRL_v++;
						STATUS_CTRL_v = STATUS_CTRL_v%2;
						break;
					case 'p' : // arm
						STATUS_CTRL_p++;
						STATUS_CTRL_p = STATUS_CTRL_p%2;
						myUARTprintf(UART1_BASE, ">> PI+P controller engaged!%n");
						break;
					case 'X' : // disarm
						PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PWMLoad*0);
						PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, PWMLoad*0);
						PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, PWMLoad*0);
						PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWMLoad*0);
						GPIOPinWrite(ROTOR_POWER_PORT, ROTOR_POWER_PIN, 0);
						thrust_base = 0.0;
						STATUS_loop = 0;
						myUARTprintf(UART1_BASE, ">> Quad is disarmed! (PWM duty set to 0%, ESC power off)%n");
						break;
					case 'w' : // throttle up
						if(thrust_base == 0.0) thrust_base = 500.0;
						else if(thrust_base < 1000.0) thrust_base += 100.0;
						else if(thrust_base == 1300.0) thrust_base = 1300.0;
						else thrust_base += 10.0;
						myUARTprintf(UART1_BASE, ">> thrust: %f%n", thrust_base);
						break;
					case 's' : // throttle down
						if(thrust_base <= 500.0) thrust_base = 0.0;
						else if(thrust_base <= 1000.0) thrust_base -= 100.0;
						else thrust_base -= 10.0;
						myUARTprintf(UART1_BASE, ">> thrust: %f%n", thrust_base);
						break;
					case 'd' : // turn right (increase psi)
						yaw_set_delta += 3.0/180*PI;
						if(yaw_set_delta > 6.0/180*PI) yaw_set_delta = 6.0/180*PI;
						STATUS_CTRL_ad = Tick1000Hz;
						myUARTprintf(UART1_BASE, ">> yaw set: %f%n", yaw_set_delta*180/PI);
						break;
					case 'a' : // turn left (decrease psi)
						yaw_set_delta -= 3.0/180*PI;
						if(yaw_set_delta < -6.0/180*PI) yaw_set_delta = -6.0/180*PI;
						STATUS_CTRL_ad = Tick1000Hz;
						myUARTprintf(UART1_BASE, ">> yaw set: %f%n", yaw_set_delta*180/PI);
						break;
					case '5' : // pitch up (increase theta)
						euler_set[1] += 3.0/180*PI;
						if(euler_set[1] > 6.0/180*PI) euler_set[1] = 6.0/180*PI;
						STATUS_CTRL_58 = Tick1000Hz;
						myUARTprintf(UART1_BASE, ">> pitch set: %f%n", euler_set[1]*180/PI);
						break;
					case '8' : // pitch down (decrease theta)
						euler_set[1] -= 3.0/180*PI;
						if(euler_set[1] < -6.0/180*PI) euler_set[1] = -6.0/180*PI;
						STATUS_CTRL_58 = Tick1000Hz;
						myUARTprintf(UART1_BASE, ">> pitch set: %f%n", euler_set[1]*180/PI);
						break;
					case '6' : // roll right (increase phi)
						euler_set[0] += 3.0/180*PI;
						if(euler_set[0] > 6.0/180*PI) euler_set[0] = 6.0/180*PI;
						STATUS_CTRL_46 = Tick1000Hz;
						myUARTprintf(UART1_BASE, ">> roll set: %f%n", euler_set[0]*180/PI);
						break;
					case '4' : // roll left (decrease phi)
						euler_set[0] -= 3.0/180*PI;
						if(euler_set[0] < -6.0/180*PI) euler_set[0] = -6.0/180*PI;
						STATUS_CTRL_46 = Tick1000Hz;
						myUARTprintf(UART1_BASE, ">> roll set: %f%n", euler_set[0]*180/PI);
						break;
					default :
						break;
					}
					XBEEIntmsg[0] = 'x';
				}

				if(Tickloop == 50)
				{
					ADCProcessorTrigger(ADC0_BASE, 3);
					//if(((float)raw_v)*3.3/4096.0*6.1 < 10.0) myUARTprintf(UART1_BASE, ">> WARNING! LOW BATTERY!%n");
				}

				if((STATUS_CTRL_v == 1) && (Tickloop == 100)) myUARTprintf(UART1_BASE, ">> STATUS (type 'v' to turn off)%n");
				if((STATUS_CTRL_v == 1) && (Tickloop == 101)) myUARTprintf(UART1_BASE, ">>  ______        ______%n");
				if((STATUS_CTRL_v == 1) && (Tickloop == 102)) myUARTprintf(UART1_BASE, ">> |  R4  |      |  R1  |%n");
				if((STATUS_CTRL_v == 1) && (Tickloop == 103)) myUARTprintf(UART1_BASE, ">>| %f  |    | %f  |%n", 0.001*thrust[3], 0.001*thrust[0]);
				if((STATUS_CTRL_v == 1) && (Tickloop == 104)) myUARTprintf(UART1_BASE, ">> |__CW__||    ||__CC__|%n");
				if((STATUS_CTRL_v == 1) && (Tickloop == 105)) myUARTprintf(UART1_BASE, ">>         ||  ||%n");
				if((STATUS_CTRL_v == 1) && (Tickloop == 106)) myUARTprintf(UART1_BASE, ">>           XX%n");
				if((STATUS_CTRL_v == 1) && (Tickloop == 107)) myUARTprintf(UART1_BASE, ">>  ______ ||  || ______%n");
				if((STATUS_CTRL_v == 1) && (Tickloop == 108)) myUARTprintf(UART1_BASE, ">> |  R3  ||    ||  R2  |%n");
				if((STATUS_CTRL_v == 1) && (Tickloop == 109)) myUARTprintf(UART1_BASE, ">>| %f  |    | %f  |%n", 0.001*thrust[2], 0.001*thrust[1]);
				if((STATUS_CTRL_v == 1) && (Tickloop == 110)) myUARTprintf(UART1_BASE, ">> |__CC__|      |__CW__|%n");
				if((STATUS_CTRL_v == 1) && (Tickloop == 111)) myUARTprintf(UART1_BASE, ">> BATT VTG: %f%n", ((float)raw_v)*3.3/4096.0*6.1*12.195/12.265);
				if((STATUS_CTRL_v == 1) && (Tickloop == 112)) myUARTprintf(UART1_BASE, "Roll: %f Pitch: %f ", eulerK[0]*180.0/PI, eulerK[1]*180.0/PI);
				if((STATUS_CTRL_v == 1) && (Tickloop == 113)) myUARTprintf(UART1_BASE, "Yaw: %f%n", eulerK[2]*180.0/PI);
				if((STATUS_CTRL_v == 1) && (Tickloop == 114)) myUARTprintf(UART1_BASE, "T_x: %f T_y: %f ", torque_req[0], torque_req[1]);
				if((STATUS_CTRL_v == 1) && (Tickloop == 115)) myUARTprintf(UART1_BASE, "T_z: %f delta theta: %f%n", torque_req[2], angle_delta*180.0/PI);

				if((STATUS_CTRL_h == 1) && (Tickloop == 150)) myUARTprintf(UART1_BASE, ">> Controls%n");
				if((STATUS_CTRL_h == 1) && (Tickloop == 151)) myUARTprintf(UART1_BASE, ">> v   : toggle view data%n");
				if((STATUS_CTRL_h == 1) && (Tickloop == 152)) myUARTprintf(UART1_BASE, ">> p   : engage PD+P controller%n");
				if((STATUS_CTRL_h == 1) && (Tickloop == 153)) myUARTprintf(UART1_BASE, ">> X   : disarm (CAUTION! ONLY ");
				if((STATUS_CTRL_h == 1) && (Tickloop == 154)) myUARTprintf(UART1_BASE, "ENGAGE WHEN THROTTLE IS ZERO!)%n");
				if((STATUS_CTRL_h == 1) && (Tickloop == 155)) myUARTprintf(UART1_BASE, ">> w/s : throttle up/down%n");
				if((STATUS_CTRL_h == 1) && (Tickloop == 156)) myUARTprintf(UART1_BASE, ">> d/a : turn right/left%n");
				if((STATUS_CTRL_h == 1) && (Tickloop == 157)) myUARTprintf(UART1_BASE, ">> 5/8 : pitch up/down%n");
				if((STATUS_CTRL_h == 1) && (Tickloop == 158))
				{
					myUARTprintf(UART1_BASE, ">> 6/4 : roll right/left%n");
					STATUS_CTRL_h = 0;
				}


				Tickloop++;
				Tickloop = Tickloop%200;
				TickLogFinish = Tick1000Hz;
				if(TickLogFinish == TickLogStart) SysCtlDelay(ui32SysClkFreq/3.0*0.002); // 2ms delay
				else if(TickLogFinish - TickLogStart >= 5) myUARTprintf(UART1_BASE, ">> WARNING! LOOP TIME LIMIT EXCEEDED BY %dms!%n", TickLogFinish - TickLogStart - 5);
			}
		}

	}
	return 0;
}
