/*-------------------------------------------------------------------------------------------
 ********************************************************************************************
 *-------------------------------------------------------------------------------------------
 *
 *				DATA LOGGER DE VARIABLES AMBIENTALES INTERNAS
 *							CIMEC CONICET - UTN FRP
 *								     2016
 *
 *						Polo, Franco		fjpolo@frp.utn.edu.ar
 *						Burgos, Sergio		sergioburgos@frp.utn.edu.ar
 *						Bre, Facundo		facubre@cimec.santafe-conicet.gov.ar
 *
 *	datalogger.c
 *
 *	Descripción:
 *
 *  Desarrollo del firmware de la placa base del data logger, constando de:
 *
 *  - Periféricos I2C:
 *  	a) HR y Tbs		HIH9131		0b0100111		0x27
 *  	b) Ev			TSL2563		0b0101001		0x29
 *  	c) Va			ADS			0b1001000		0x48
 *  	d) Tg			LM92		0b1001011		0x51
 *  	e) RTC			DS1703		0b1101000		0x68
 *
 *  - Periféricos OneWire@PD6
 *  	a) Ts01			MAX31850	ROM_Addr		0x3B184D8803DC4C8C
 *  	b) Ts02			MAX31850	ROM_Addr		0x3B0D4D8803DC4C3C
 *  	c) Ts03			MAX31850	ROM_Addr		0x3B4D4D8803DC4C49
 *  	d) Ts04			MAX31850	ROM_Addr		0x3B234D8803DC4C99
 *  	e) Ts05			MAX31850	ROM_Addr		0x3B374D8803DC4C1E
 *  	f) Ts06			MAX31850	ROM_Addr
 *
 *  - IHM
 *  	a) RESET		!RST
 *  	b) SW_SD		PC6
 *  	c) SW_ON		PC5
 *  	d) SW_1			PC7
 *  	e) WAKE			PF2
 *  	f) LEDON		PE0
 *  	g) LED1			PE1
 *  	h) LED2			PE2
 *
 *  - SD
 *  	a) SD_IN		PA6
 *  	b) SD_RX		PA4
 *  	c) SD_TX		PA5
 *  	d) SD_CLK		PA2
 *  	e) SD_FSS		PA3
 *
 *--------------------------------------------------------------------------------------------
 *********************************************************************************************
 *-------------------------------------------------------------------------------------------*/
/*********************************************************************************************
 * INCLUDES
 ********************************************************************************************/
// standard C
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
// inc
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
// driverlib
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/hibernate.h"
#include "driverlib/i2c.h"
// datalogger
#include "datalogger/datalogger.h"
#include "datalogger/delay.h"
// third_party
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"

/*****************************************************************************************************
 * Function prototypes
 ****************************************************************************************************/
//void DataLoggingON(void);
//void DataLoggingOFF(void);
//void Init(void);
void InitI2C3(void);
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...);
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);
unsigned char dec2bcd(unsigned char val);
unsigned char bcd2dec(unsigned char val);
void SetTimeDate(unsigned char sec, unsigned char min,
		unsigned char hour,unsigned char day,
		unsigned char date, unsigned char month,unsigned char year);
unsigned char GetClock(unsigned char reg);
char Int2Char(int var);
char* itoa(int value, char* result, int base);
void push(unsigned int v);
unsigned int pop(void);
unsigned int full(void);
void inttostr(unsigned int v, char *str);
void FloatToString(char *str, float f, char size);

/*****************************************************************************************************
 * Defines
 ****************************************************************************************************/
// SWITCHES
#define SW_PORT 	GPIO_PORTC_BASE
#define SW_ON 		GPIO_PIN_5
#define SW_SD 		GPIO_PIN_6
#define SW_1 		GPIO_PIN_7
// LEDS
#define LED_PORT 	GPIO_PORTE_BASE
#define LED_ON 		GPIO_PIN_0
#define LED_1 		GPIO_PIN_1
#define LED_2 		GPIO_PIN_2
// SD Card
#define SD_PORT		GPIO_PORTA_BASE
#define SD_IN		GPIO_PIN_6
#define SD_RX		GPIO_PIN_4
#define SD_TX		GPIO_PIN_5
#define SD_CLK		GPIO_PIN_2
#define SD_FSS		GPIO_PIN_3
// Timer0
#define TOGGLE_FREQUENCY 1
// Tg
#define SLAVE_ADDRESS_TG 0x4B
#define TG_REG_READ 0x00
#define TG_REG_LOWPOW 0x01
// Ev
#define SLAVE_ADDRESS_EV 0x29
// RTC
#define SLAVE_ADDR_RTC 0x68
#define SEC 0x00
#define MIN 0x01
#define HRS 0x02
#define DAY 0x03
#define DATE 0x04
#define MONTH 0x05
#define YEAR 0x06
#define CNTRL 0x07
// Va ADC slave address
#define SLAVE_ADDR_VA 0x48
#define VA_REG_READ 0x00
// I2C3
#define GPIO_PD0_I2C3SCL        0x00030003
#define GPIO_PD1_I2C3SDA        0x00030403
// str2string
#define MAXDIGIT 10
// SSI1


/*********************************************************************************************
 * Global variables
 * ******************************************************************************************/
//
int SWRead;
//
int SWRead, SD_Read;
// Estado del modulo hibernacion
unsigned long ulStatus;
unsigned long ulPeriod;
//Datos a mantener durante la hibernacion
//0: sensor_flag
//1: hibernate_flag
//2: Tg y RTC
unsigned long ulNVData[3] = { 1, 0, 0};
//
//static unsigned long g_ulDataRx, MSB,LSB, Sign;
//static unsigned long Tg_Raw, Tg_MSB,Tg_LSB, Tg_Sign;
unsigned char sec,min,hour,day,date,month,year;
static float Tg=0;
//static string Dias[7] = {'Lunes', 'Martes', 'Miercoles', 'Jueves', 'Viernes', 'Sabado', 'Domingo'};
// str2string
typedef struct
{
	unsigned int data[MAXDIGIT];
	unsigned int p;
	unsigned int mx;
}stStack;
stStack stack = {{0}, 0, MAXDIGIT};
//
// SD Card variables
//
FATFS FatFs;    /* Work area (file system object) for logical drive */
FIL fil;        /* File object */
//
FRESULT fr;     /* FatFs return code */
FILINFO fno;
UINT br;    /* File read count */
//
DIR *dir;
//static FILINFO fileInfo;
//
int i=0;
const char str_date[] = "Fecha";
const char str_hour[] = "Hora";
const char str_tg[] = "Temperatura de globo [ºC]";
const char str_hr[] = "Humedad relativa [%]";
const char str_tbs[] = "Temperatura de bulbo seco [ºC]";
const char str_ev[] = "Iluminancia [lux]";
const char str_va[] = "Velocidad de aire [m/s]";
const char newline[] = "\r\n";
const char comma[] = ";";
static char dir_date[9];
static char dir_time[6];
static char full_date[9];
static char full_time[9];
//char file_name[8];
//char str[8];
//char fname[] = "1400.csv";
const char null[]="0";
int filesize;
uint16_t HIHRaw[2];
static double t_bs;
static double hr;
static char charTg[7];
static char charHR[6];
static char charTbs[6];
static char charEv[10];
double Ev;
/*****************************************************************************************************
 * InitI2C3
 ****************************************************************************************************/
//initialize I2C module 0 & 3
//Slightly modified version of TI's example code
void InitI2C3(void)
{
	//
	// The I2C0 peripheral must be enabled before use.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
	//
	// For this example I2C3 is used with PortB[0:1].  The actual port and
	// pins used may be different on your part, consult the data sheet for
	// more information.  GPIO port D needs to be enabled so these pins can
	// be used.
	// TODO: change this to whichever GPIO port you are using.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//
	// Configure the pin muxing for I2C3 functions on port D0 and D1.
	// This step is not necessary if your part does not support pin muxing.
	// TODO: change this to select the port/pin you are using.
	//
	GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	GPIOPinConfigure(GPIO_PD1_I2C3SDA);
	//
	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	// TODO: change this to select the port/pin you are using.
	//
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
	//
	// Enable loopback mode.  Loopback mode is a built in feature that helps
	// for debug the I2Cx module.  It internally connects the I2C master and
	// slave terminals, which effectively lets you send data as a master and
	// receive data as a slave.  NOTE: For external I2C operation you will need
	// to use external pull-ups that are faster than the internal pull-ups.
	// Refer to the datasheet for more information.
	//
	//HWREG(I2C3_BASE + I2C_O_MCR) |= 0x01;
	//
	// Enable the I2C3 interrupt on the processor (NVIC).
	//
	//IntEnable(I2C0_IRQHandler);
	//
	// Configure and turn on the I2C3 slave interrupt.  The I2CSlaveIntEnableEx()
	// gives you the ability to only enable specific interrupts.  For this case
	// we are only interrupting when the slave device receives data.
	//
	//I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);
	//
	// Enable and initialize the I2C3 master module.  Use the system clock for
	// the I2C0 module.  The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.  For this example we will use a data rate of 100kbps.
	//
	I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), false);
	//
	// Enable the I2C3 slave module.
	//
	//I2CSlaveEnable(I2C3_BASE);

	//
	// Set the slave address to SLAVE_ADDRESS.  In loopback mode, it's an
	// arbitrary 7-bit number (set in a macro above) that is sent to the
	// I2CMasterSlaveAddrSet function.
	//
	//I2CSlaveInit(I2C3_BASE, SLAVE_ADDRESS);

	//
	// Tell the master module what address it will place on the bus when
	// communicating with the slave.  Set the address to SLAVE_ADDRESS
	// (as set in the slave module).  The receive parameter is set to false
	// which indicates the I2C Master is initiating a writes to the slave.  If
	// true, that would indicate that the I2C Master is initiating reads from
	// the slave.
	//
	//I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDRESS, false);
	//
	// Enable interrupts to the processor.
	//
	//IntMasterEnable();
}

/*****************************************************************************************************
 * I2CSend
 ****************************************************************************************************/
// Sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
	// Tell the master module what address will place on the bus when
	// communicating with the slave.
	I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, false);

	// Stores list of variable number of arguments
	va_list vargs;
	// Specifies the va_list to "open" and the last fixed argument
	//so vargs knows where to start looking
	va_start(vargs, num_of_args);

	// Put data to be sent into FIFO
	I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));
	// If there is only one argument, we only need to use the
	//single send I2C function
	if(num_of_args == 1)
	{
		//Initiate send of data from the MCU
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
		// Wait until MCU is done transferring.
		SysCtlDelay(500);
		while(I2CMasterBusy(I2C3_BASE));
		//"close" variable argument list
		va_end(vargs);
	}
	// Otherwise, we start transmission of multiple bytes on the
	//I2C bus
	else
	{
		// Initiate send of data from the MCU
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		// Wait until MCU is done transferring.
		SysCtlDelay(500);
		while(I2CMasterBusy(I2C3_BASE));
		// Send num_of_args-2 pieces of data, using the
		//BURST_SEND_CONT command of the I2C module
		unsigned char i;
		for(i = 1; i < (num_of_args - 1); i++)
		{
			// Put next piece of data into I2C FIFO
			I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));
			// Send next data that was just placed into FIFO
			I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
			// Wait until MCU is done transferring.
			SysCtlDelay(500);
			while(I2CMasterBusy(I2C3_BASE));
		}
		// Put last piece of data into I2C FIFO
		I2CMasterDataPut(I2C3_BASE, va_arg(vargs, uint32_t));
		// Send next data that was just placed into FIFO
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
		// Wait until MCU is done transferring.
		SysCtlDelay(500);
		while(I2CMasterBusy(I2C3_BASE));
		//"close" variable args list
		va_end(vargs);
	}
}

/*****************************************************************************************************
 * I2CReceive
 ****************************************************************************************************/
// Read specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
{
	// Specify that we are writing (a register address) to the
	//slave device
	I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, false);
	//specify register to be read
	I2CMasterDataPut(I2C3_BASE, reg);
	//send control byte and register address byte to slave device
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	//wait for MCU to finish transaction
	SysCtlDelay(500);
	while(I2CMasterBusy(I2C3_BASE));
	//specify that we are going to read from slave device
	I2CMasterSlaveAddrSet(I2C3_BASE, slave_addr, true);
	//send control byte and read from the register we
	//specified
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	//wait for MCU to finish transaction
	SysCtlDelay(500);
	while(I2CMasterBusy(I2C3_BASE));
	//return data pulled from the specified register
	return I2CMasterDataGet(I2C3_BASE);
}
/*****************************************************************************************************
 * dec2bcd
 ****************************************************************************************************/
//decimal to BCD conversion
unsigned char dec2bcd(unsigned char val)
{
	return (((val / 10) << 4) | (val % 10));
}
// convert BCD to binary
unsigned char bcd2dec(unsigned char val)
{
	return (((val & 0xF0) >> 4) * 10) + (val & 0x0F);
}

/*****************************************************************************************************
 * SetTimeDate
 ****************************************************************************************************/
//Set Time
void SetTimeDate(unsigned char sec, unsigned char min, unsigned char hour,unsigned char day, unsigned char date, unsigned char month,unsigned char year)
{
	I2CSend(SLAVE_ADDR_RTC,8,SEC,dec2bcd(sec),dec2bcd(min),dec2bcd(hour),dec2bcd(day),dec2bcd(date),dec2bcd(month),dec2bcd(year));
}

/*****************************************************************************************************
 * GetClock
 ****************************************************************************************************/
//Get Time and Date
unsigned char GetClock(unsigned char reg)
{
	unsigned char clockData = I2CReceive(SLAVE_ADDR_RTC,reg);
	return bcd2dec(clockData);
}

/*****************************************************************************************************
 * push
 ****************************************************************************************************/
//Push from stack?
void push(unsigned int v)
{
	if(stack.p < stack.mx)
	{
		stack.data[stack.p] = v;
		stack.p ++;
	}
}

/*****************************************************************************************************
 * pop
 ****************************************************************************************************/
//Pop from stack?
unsigned int pop(void)
{
	unsigned int ret = 0;
	if(stack.p > 0)
	{
		stack.p--;
		ret = stack.data[stack.p];
	}
	return ret;
}

/*****************************************************************************************************
 * full
 ****************************************************************************************************/
//Stack full?
unsigned int full(void)
{
	return !(stack.mx - stack.p);
}
unsigned int empty(void)
{
	return !stack.p;
}

/*****************************************************************************************************
 * inttostr
 ****************************************************************************************************/
//Convert int to string
void inttostr(unsigned int v, char *str)
{
	unsigned int part;
	while(v && !full())
	{
		part = v % 10;
		push(part + 48);
		v = v / 10;
	}
	while(!empty())
	{
		*str = pop();
		str++;
	}
	*str = '\0';
}

/*****************************************************************************************************
 * Int2Char
 ****************************************************************************************************/
// Date and time are used to create directories, one value at a time.
char Int2Char(int var){
	return var+'0';
}

/*****************************************************************************************************
 * itoa
 ****************************************************************************************************/
// Integer to char string
char* itoa(int value, char* result, int base) {
	// check that the base if valid
	if (base < 2 || base > 36) { *result = '\0'; return result; }

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while ( value );

	// Apply negative sign
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr--= *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}
/*****************************************************************************************************
 * my_strcat
 ****************************************************************************************************/
// Concatenate strings
char *
my_strcat(char *dest, const char *src)
{
	char *rdest = dest;

	while (*dest)
		dest++;
	while (*dest++ = *src++)
		;
	return rdest;
}

/*****************************************************************************************************
 * FloatToString
 ****************************************************************************************************/
// convert float to string one decimal digit at a time
// assumes float is < 65536 and ARRAYSIZE is big enough
// problem: it truncates numbers at size without rounding
// str is a char array to hold the result, float is the number to convert
// size is the number of decimal digits you want
void FloatToString(char *str, float f, char size)

{
	char pos;  // position in string
	char len;  // length of decimal part of result
	char* curr;  // temp holder for next digit
	int value;  // decimal digit(s) to convert
	pos = 0;  // initialize pos, just to be sure
	value = (int)f;  // truncate the floating point number
	itoa(value,str, 10);  // this is kinda dangerous depending on the length of str
	// now str array has the digits before the decimal
	if (f < 0 )  // handle negative numbers
	{
		f *= -1;
		value *= -1;
	}
	len = strlen(str);  // find out how big the integer part was
	pos = len;  // position the pointer to the end of the integer part
	str[pos++] = '.';  // add decimal point to string

	while(pos < (size + len + 1) )  // process remaining digits
	{
		f = f - (float)value;  // hack off the whole part of the number
		f *= 10;  // move next digit over
		value = (int)f;  // get next digit
		itoa(value, curr, 10); // convert digit to string
		str[pos++] = *curr; // add digit to result string and increment pointer
	}
}

/*********************************************************************************************
 * Main loop
 *
 * - Using GPIO to create a new file using date and time
 *
 ********************************************************************************************/

int main(void)
{
	//
	// Initialization
	Initialize();
	// GPIO INT enable
	IntEnable(INT_GPIOC);
	// Initialize I2C module 3
	InitI2C3();
	//
	//TODO Set time and date
	//SetTimeDate(0, 27, 21, 6, 14, 7,16);
	//
	// Get time and date
	//
	sec = GetClock(SEC);
	min = GetClock(MIN);
	hour = GetClock(HRS);
	//day = GetClock(DAY);
	date = GetClock(DATE);
	month = GetClock(MONTH);
	year = GetClock(YEAR);
	//
	// Date
	//
	full_date[0] = Int2Char(year/10);
	full_date[1] = Int2Char(year%10);
	full_date[2] = '-';
	full_date[3] = Int2Char(month/10);
	full_date[4] = Int2Char(month%10);
	full_date[5] = '-';
	full_date[6] = Int2Char(date/10);
	full_date[7] = Int2Char(date%10);
	full_date[8] = '\0';
	//
	//inttostr(((month*100)+date), &dir_date);
	for(i=0;i<=9;i++){
		dir_date[i] = full_date[i];
	}
	// Directory date
	/*for(i=0;i<=5;i++){
		dir_date[i] = full_date[i+3];
	}*/

	// Time
	full_time[0]=Int2Char(hour/10);
	full_time[1]=Int2Char(hour%10);
	full_time[2]=':';
	full_time[3]=Int2Char(min/10);
	full_time[4]=Int2Char(min%10);
	full_time[5]=':';
	full_time[6]=Int2Char(sec/10);
	full_time[7]=Int2Char(sec%10);
	full_time[8] = '\0';
	//
	dir_time[0] = full_time[0];
	dir_time[1] = full_time[1];
	dir_time[2] = '-';
	dir_time[3] = full_time[3];
	dir_time[4] = full_time[4];
	dir_time[5] = '\0';
	//inttostr(((hour*100)+min), &dir_time);
	// Directory time
	/*dir_time[0] = full_time[0];
	dir_time[1] = full_time[1];
	dir_time[2] = '-';
	dir_time[3] = full_time[3];
	dir_time[4] = full_time[4];*/
	//
	//
	//
	if( f_mount(0,&FatFs) == FR_OK ){
		f_mkdir(dir_date);
		f_chdir(dir_date);
		f_mkdir(dir_time);
		f_chdir(dir_time);
		if(f_open(&fil, "Data.csv", FA_WRITE | FA_OPEN_ALWAYS ) == FR_OK){
			// First row str_date
			fr = f_write(&fil, str_date, strlen(str_date), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_hour, strlen(str_hour), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_tbs, strlen(str_tbs), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_hr, strlen(str_hr), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_tg, strlen(str_tg), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_ev, strlen(str_ev), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, str_va, strlen(str_va), &br);
			fr = f_write(&fil, comma, strlen(comma), &br);
			fr = f_write(&fil, newline, strlen(newline), &br);
		}
		f_close(&fil);
	}

	//
	//
	//
	//f_mount(0, null);
	while(1)
	{
		//Wait for GPIO interrupt
	}
}
/*********************************************************************************************
 * GPIOPortC_IRQHandler
 * ******************************************************************************************/
void GPIOPortC_IRQHandler(void){
	//
	// Get time and date
	//
	//
	sec = GetClock(SEC);
	min = GetClock(MIN);
	hour = GetClock(HRS);
	date = GetClock(DATE);
	month = GetClock(MONTH);
	year = GetClock(YEAR);
	// Date
	full_date[0] = Int2Char(year/10);
	full_date[1] = Int2Char(year%10);
	full_date[2] = '-';
	full_date[3] = Int2Char(month/10);
	full_date[4] = Int2Char(month%10);
	full_date[5] = '-';
	full_date[6] = Int2Char(date/10);
	full_date[7] = Int2Char(date%10);
	// Time
	full_time[0]=Int2Char(hour/10);
	full_time[1]=Int2Char(hour%10);
	full_time[2]=':';
	full_time[3]=Int2Char(min/10);
	full_time[4]=Int2Char(min%10);
	full_time[5]=':';
	full_time[6]=Int2Char(sec/10);
	full_time[7]=Int2Char(sec%10);
	//
	// Get RH and Tbs
	//
	readHIH(&HIHRaw);
	hr = getHR(&HIHRaw);
	t_bs = getTBS(&HIHRaw);
	Tg = getTG();
	Ev = getEV();

	//
	// Convert to string
	// TODO make this shit work
	//
	FloatToString(charTg, Tg, 2);
	FloatToString(charHR, hr, 2);
	FloatToString(charTbs, t_bs, 2);
	FloatToString(charEv, Ev, 2);
	//
	if( f_mount(0,&FatFs) == FR_OK ){
		if (f_stat(dir_date, &fno) == FR_OK){
			f_chdir(dir_date);
			if (f_stat(dir_time, &fno) == FR_OK){
				f_chdir(dir_time);
				if(f_open(&fil, "Data.csv", FA_READ | FA_WRITE | FA_OPEN_ALWAYS) == FR_OK){
					//
					filesize=f_size(&fil);
					if(f_lseek(&fil, f_size(&fil)) == FR_OK){
						// Flash LED
						GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0, GPIO_PIN_0);
						delayMS(500);
						GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0, 0);
						// date
						fr = f_write(&fil, full_date, strlen(full_date), &br);
						fr = f_write(&fil, comma, strlen(comma), &br);
						// time
						fr = f_write(&fil, full_time, strlen(full_time), &br);
						fr = f_write(&fil, comma, strlen(comma), &br);
						// Tbs
						fr = f_write(&fil, charTbs, strlen(charTbs), &br);
						fr = f_write(&fil, comma, strlen(comma), &br);
						// HR
						fr = f_write(&fil, charHR, strlen(charHR), &br);
						fr = f_write(&fil, comma, strlen(comma), &br);
						// Tg
						fr = f_write(&fil, charTg, strlen(charTg), &br);
						fr = f_write(&fil, comma, strlen(comma), &br);
						// Ev
						fr = f_write(&fil, charEv, strlen(charEv), &br);
						fr = f_write(&fil, comma, strlen(comma), &br);
						// Va
						fr = f_write(&fil, null, strlen(null), &br);
						fr = f_write(&fil, comma, strlen(comma), &br);
						fr = f_write(&fil, newline, strlen(newline), &br);
						//end of f_seek
					}
					//end of f_open
					f_close(&fil);
				}
			}
		}
		//end of f_mount
		f_mount(0, null);
	}
	// Clear interrupt source
	GPIOIntClear(SW_PORT, SW_1);

}
