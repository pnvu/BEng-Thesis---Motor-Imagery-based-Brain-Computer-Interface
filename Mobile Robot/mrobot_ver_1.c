/*
 * @name: mrobot_ver_1.c
 * @description: Software for mobile robot version 1.0
 * 				 Receive signal from Computer via Bluetooth
 * 				 Using PWM to control motor
 * @date: Dec - 21 - 2016 (This is the day of Sing, yeh!)
 * @author: VPi
 */

// Include libraries
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/qei.h"
#include "driverlib/uart.h"

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

// Define PWM variable
#define PWM_FREQUENCY 500 // PWM frequency
volatile uint32_t ui32Load; // cycle of PWM

/*
 * @name: UART_config
 * @description:
 * 		- Configure UART2 at 2 pin PD6 and PD7
 * 		- Interface with Bluetooth
 */
void UART_config(uint32_t baudrate){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	GPIOPinConfigure(GPIO_PD6_U2RX);
	GPIOPinConfigure(GPIO_PD7_U2TX);
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6|GPIO_PIN_7);
	UARTClockSourceSet(UART2_BASE, UART_CLOCK_SYSTEM);

	//UARTStdioConfig(0, baudrate, SysCtlClockGet());
	// Cai dat toc do baud va cau hinh du lieu
	UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), baudrate,
			(UART_CONFIG_WLEN_8| UART_CONFIG_STOP_ONE| UART_CONFIG_PAR_NONE));

	//UARTEnable(UART2_BASE);
}

/*
 * @name: motor1_config
 * @description: Configure PWM and directional pin for driver 1
 * @schematic:
 * 		PB4	--- PWMA
 * 		PB0 --- AIN1
 * 		PB1 --- AIN2
 * 		PB2 --- STBY
 */
void motor1_config(void){
	// PWM Configuration
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);// Bat PWM0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PB4_M0PWM2);//Bat PWM cho chan PB4
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);// Cau hinh kieu dem xuong

	// Directional Configuration
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
}

/*
 * @name: motor2_config
 * @description: Configure PWM and directional pin for driver 2
 * @schematic:
 * 		PE4 --- PWMB
 * 		PE0 --- AIN1
 * 		PE1 --- AIN2
 * 		PE2 --- STBY
 */
void motor2_config(void){
	// PWM Configuration
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);// Bat PWM1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PE4_M1PWM2);//Bat PWM cho chan PE4
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);// Cau hinh kieu dem xuong

	// Directional Configuration
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
}

int main(void) {
	uint32_t ui32PWMclock; // PWM clock
	char read = 0; // Receive character from Bluetooth
	
	// System clock configuration
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);//40Mhz

	// Configure UART Module with baudrate = 9600
	UART_config(9600);

	// Configure led from port F
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	// PWM clock configuration
	SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
	motor1_config(); // motor 1 config
	motor2_config(); // motor 2 config

	// Configurate pin in driver1
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1|GPIO_PIN_2, 0xFF);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
	// Configurate pin in driver2
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_2, 0xFF);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);

	ui32PWMclock = SysCtlClockGet() / 16;
	ui32Load = (ui32PWMclock / PWM_FREQUENCY) - 1;

	// Load period PWM for driver 1
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);// nap period
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 15 * ui32Load / 100);// Nap chu ki nhiem vu
	PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);// Bat dau chay
	// Load period PWM for driver 2
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);// nap period
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 15 * ui32Load / 100);// Nap chu ki nhiem vu
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);// Bat dau chay

	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0xFF);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0xFF);

	while(1){
		read = UARTCharGet(UART2_BASE);
		switch(read){
		case 'u':
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);

			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
			break;
		case 'r':
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0xFF);

			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_2);
			break;
		case 'd':

			break;
		case 'l':
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0xFF);
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);

			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x0E);
			break;
		case 'n':
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0xFF);
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0xFF);

			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
			break;
		}
	}
}
