/*
 * BME680_write_read_test.c
 *
 * Created: 5/3/2023 1:32:30 PM
 * Author : wji
 */ 

#include <avr/interrupt.h>
#include <avr/io.h>
#define F_CPU 4000000
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

// Display buffer for DOG LCD using sprintf()
char dsp_buff1[17];
char dsp_buff2[17];
char dsp_buff3[17];


#define CLK_PER                                         4000000     // 4MHz default clock
#define TWI0_BAUD(F_SCL, T_RISE)                        ((((((float)CLK_PER / (float)F_SCL)) - 10 - ((float)CLK_PER * T_RISE))) / 2)

#define I2C_SCL_FREQ                                    100000
// #define LM75_ADDRESS									0b10010001
#define I2C_SLAVE_RESPONSE_ACKED                        (!(TWI_RXACK_bm & TWI0.MSTATUS))
#define I2C_DATA_RECEIVED                               (TWI_RIF_bm & TWI0.MSTATUS)

// LM75 Stuff
// uint8_t temp_reg_high;
// uint8_t temp_reg_low;
// uint16_t LM75_temp_reg = 0;
// int16_t LM75_temp = 0;
// static void I2C_0_init(void);
// static uint16_t TWI0_LM75_read(uint8_t address);

// BME680 related constants
#define BME680_SLAVE_ADDRESS 0x76

// Global variables
uint8_t status, id;
uint32_t temperature_raw;

// Function prototypes
static void TWI0_BME680_init(void);
static void BME680_TWI_write(uint8_t slave_address, uint8_t reg_address, uint8_t data);
static uint8_t BME680_TWI_read(uint8_t slave_address, uint8_t reg_address);

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
void user_delay_ms(uint32_t period);

void lcd_spi_transmit_CMD (unsigned char cmd);
void lcd_spi_transmit_DATA (unsigned char cmd);
void init_spi_lcd (void);
void init_lcd_dog (void);
void delay_40mS(void);
void delay_30uS(void);
void update_lcd_dog(void);



// BME680-related functions
void TWI0_BME680_init(void) {
	// Set up the pins for SDA (PA2) and SCL (PA3)
	//PORTA.DIRSET = PIN3_bm; // SCL
	//PORTA.DIRCLR = PIN2_bm; // SDA
	
	// Configure I2C pins
	PORTA.DIRSET = PIN2_bm; // Set PA2 as an output
	PORTA.DIRSET = PIN3_bm; // Set PA3 as an output

	// Initialize TWI0
	// Configure I2C baud rate
	// TWI0.MBAUD = (uint8_t)TWI0_BAUD(I2C_SCL_FREQ, 0); //setting baud rate
	//TWI0.MBAUD = (uint8_t)TWI0_BAUD(100000, 10); // 100 kHz, 10 ns rise time
	TWI0.MBAUD = 1; // 100 kHz, 10 ns rise time
	TWI0.MCTRLA = TWI_ENABLE_bm;
	TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc;
}

void BME680_TWI_write(uint8_t slave_address, uint8_t reg_address, uint8_t data) {
	TWI0.MADDR = (slave_address << 1) | 0; // Write mode
	while (!(TWI0.MSTATUS & TWI_WIF_bm));


		TWI0.MDATA = reg_address;
		while (!(TWI0.MSTATUS & TWI_WIF_bm));
		
		
		TWI0.MDATA = data;
		while (!(TWI0.MSTATUS & TWI_WIF_bm));
		


	TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
}

uint8_t BME680_TWI_read(uint8_t slave_address, uint8_t reg_address) {
	uint8_t data = 0;
	TWI0.MADDR = (slave_address << 1) | 0; // Write mode
	while (!(TWI0.MSTATUS & TWI_WIF_bm));

	
	TWI0.MDATA = reg_address;
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MCTRLB |= TWI_MCMD_STOP_gc;

		
	TWI0.MADDR = (slave_address << 1) | 1; // Read mode
	//while (!(TWI0.MSTATUS & TWI_WIF_bm));
	while (!(TWI0.MSTATUS & TWI_RIF_bm));
			
	data = TWI0.MDATA;
	
	TWI0.MCTRLB = 0x04 | TWI_MCMD_RECVTRANS_gc;
	TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
	return data;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		BME680_TWI_write(dev_id, reg_addr + i, reg_data[i]);
	}
	return 0; // Return 0 for success
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	for (uint16_t i = 0; i < len; i++) {
		reg_data[i] = BME680_TWI_read(dev_id, reg_addr + i);
	}
	return 0; // Return 0 for success
}

void user_delay_ms(uint32_t period) {
	for (uint32_t i = 0; i < period; i++) {
		_delay_ms(1);
	}
}

/*
void user_delay_ms(uint32_t period) {
	_delay_ms(period);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	int8_t rslt = 0;

	rslt = i2c_start(dev_id<<1 | I2C_WRITE);
	if (rslt != 0) {
		return rslt;
	}

	i2c_write(reg_addr);
	rslt = i2c_start(dev_id<<1 | I2C_READ);

	for (uint16_t i = 0; i < len; i++) {
		reg_data[i] = i2c_read(i < len - 1 ? I2C_ACK : I2C_NACK);
	}

	i2c_stop();
	return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	int8_t rslt = 0;

	rslt = i2c_start(dev_id<<1 | I2C_WRITE);
	if (rslt != 0) {
		return rslt;
	}

	i2c_write(reg_addr);

	for (uint16_t i = 0; i < len; i++) {
		i2c_write(reg_data[i]);
	}

	i2c_stop();
	return rslt;
}
*/

/*
void BME680_TWI_write(uint8_t slave_address, uint8_t reg_address, uint8_t data) {
	TWI0.MADDR = (slave_address << 1) | 0; // Write mode
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MDATA = reg_address;
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MDATA = data;
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
}

uint8_t BME680_TWI_read(uint8_t slave_address, uint8_t reg_address) {
	uint8_t data;
	TWI0.MADDR = (slave_address << 1) | 0; // Write mode
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MDATA = reg_address;
	while (!(TWI0.MSTATUS & TWI_WIF_bm));
	TWI0.MADDR = (slave_address << 1) | 1; // Read mode
	while (!(TWI0.MSTATUS & TWI_RIF_bm));
	data = TWI0.MDATA;
	TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
	return data;
}
*/

///////////////////////// Driver Functions //////////////////////////

void lcd_spi_transmit_CMD (unsigned char cmd) {
	//while (!(SPI0.INTFLAGS & SPI_IF_bm)) {}    //wait until Tx ready
	PORTC_OUT = 0x00;						   //RS = 0 for command
	PORTA.OUT &= ~PIN7_bm;				       //assert slave select, not needed when MSSEN = 1
	SPI0.DATA = cmd;						   //send command
	while (!(SPI0.INTFLAGS & SPI_IF_bm)) {}    //wait until Tx ready
	PORTA.OUT |= PIN7_bm; 					   //unassert slave select, not needed when MSSEN = 1
}

void lcd_spi_transmit_DATA (unsigned char cmd) {
	//while (!(SPI0.INTFLAGS & SPI_IF_bm)) {}    //wait until Tx ready
	PORTC_OUT = 0x01;						   //RS = 1 for data
	PORTA.OUT &= ~PIN7_bm;				       //assert slave select, not needed when MSSEN = 1
	SPI0.DATA = cmd;						   //send command
	while (!(SPI0.INTFLAGS & SPI_IF_bm)) {}    //wait until Tx ready
	PORTA.OUT |= PIN7_bm; 					   //unassert slave select, not needed when MSSEN = 1
}

void init_spi_lcd (void) {
	PORTA.DIR |= PIN4_bm; // MOSI channel	PORTA.DIR &= ~PIN5_bm; // MISO channel
	PORTA.DIR |= PIN6_bm; // SCK channel
	PORTA.DIR |= PIN7_bm; // SS channel
	
	PORTC_DIR = 0x01; //PC0 Output
	PORTF_OUT = 0x00; //RS = 0  for command
	
	SPI0.CTRLA |= 0x20; //manager mode enable
	SPI0.CTRLA |= 0x08; //Clock double enable
	SPI0.CTRLA |= 0x01; //SPI enable

	CLKCTRL.OSCHFCTRLA |= 0x10; //20MHz main clock
}

void init_lcd_dog (void) {
	
	init_spi_lcd();		//Initialize SPI0
	
	//start_dly_40ms:
	delay_40mS();    //startup delay.


	//func_set1:
	lcd_spi_transmit_CMD(0x39);   // send function set #1
	delay_30uS();	//delay for command to be processed


	//func_set2:
	lcd_spi_transmit_CMD(0x39);	//send function set #2
	delay_30uS();	//delay for command to be processed


	//bias_set:
	lcd_spi_transmit_CMD(0x1E);	//set bias value.
	delay_30uS();	//delay for command to be processed


	//power_ctrl:
	lcd_spi_transmit_CMD(0x55);	//~ 0x50 nominal for 5V
	//~ 0x55 for 3.3V (delicate adjustment).
	delay_30uS();	//delay for command to be processed


	//follower_ctrl:
	lcd_spi_transmit_CMD(0x6C);	//follower mode on...
	delay_40mS();	//delay for command to be processed


	//contrast_set:
	lcd_spi_transmit_CMD(0x7F);	//~ 77 for 5V, ~ 7F for 3.3V
	delay_30uS();	//delay for command to be processed


	//display_on:
	lcd_spi_transmit_CMD(0x0c);	//display on, cursor off, blink off
	delay_30uS();	//delay for command to be processed


	//clr_display:
	lcd_spi_transmit_CMD(0x01);	//clear display, cursor home
	delay_30uS();	//delay for command to be processed


	//entry_mode:
	lcd_spi_transmit_CMD(0x06);	//clear display, cursor home
	delay_30uS();	//delay for command to be processed
}

void delay_40mS(void) {
	for (int i = 0; i < 40; i++){
		_delay_ms(1);
	}
}

void delay_30uS(void) {
	for (int i = 0; i < 30; i++){
		_delay_us(1);
	}
}

// Updates the LCD display lines 1, 2, and 3, using the
// contents of dsp_buff_1, dsp_buff_2, and dsp_buff_3, respectively.
void update_lcd_dog(void) {

	init_spi_lcd();		//init SPI port for LCD.

	// send line 1 to the LCD module.
	lcd_spi_transmit_CMD(0x80);	//init DDRAM addr-ctr
	delay_30uS();
	for (int i = 0; i < 16; i++) {
		lcd_spi_transmit_DATA(dsp_buff1[i]);
		delay_30uS();
	}
	
	// send line 2 to the LCD module.
	lcd_spi_transmit_CMD(0x90);	//init DDRAM addr-ctr
	delay_30uS();
	for (int i = 0; i < 16; i++) {
		lcd_spi_transmit_DATA(dsp_buff2[i]);
		delay_30uS();
	}
	
	// send line 3 to the LCD module.
	lcd_spi_transmit_CMD(0xA0);	//init DDRAM addr-ctr
	delay_30uS();
	for (int i = 0; i < 16; i++) {
		lcd_spi_transmit_DATA(dsp_buff3[i]);
		delay_30uS();
	}
}

int main(void) {
	// Initialize the BME680 and the LCD
	TWI0_BME680_init();
	//init_lcd_dog();
	volatile uint32_t temp_msb;
	volatile uint32_t temp_lsb;
	volatile uint32_t temp_xlsb;
	// Software reset the BME680 and read the status and ID registers
	BME680_TWI_write(BME680_SLAVE_ADDRESS, 0xE0, 0xB6);
	status = BME680_TWI_read(BME680_SLAVE_ADDRESS, 0x73);
	id = BME680_TWI_read(BME680_SLAVE_ADDRESS, 0xD0);
	BME680_TWI_write(BME680_SLAVE_ADDRESS, 0x73, 0x10);
	status = BME680_TWI_read(BME680_SLAVE_ADDRESS, 0x73);

	while (1) {
		// Enable the BME680 to read only temperature and start a conversion using forced mode
		//BME680_TWI_write(BME680_SLAVE_ADDRESS, 0x74, 0x25);

		// Wait for the measurement to complete by polling the appropriate flag
		//while (!(BME680_TWI_read(BME680_SLAVE_ADDRESS, 0x1D) & 0x80)) {
			//_delay_ms(10);
		//}

		// Read each of the three registers that hold the temperature result
		temperature_raw = 0;
		BME680_TWI_write(BME680_SLAVE_ADDRESS, 0x74, 0x25);
		temp_msb = BME680_TWI_read(BME680_SLAVE_ADDRESS, 0x22);
		temp_lsb = BME680_TWI_read(BME680_SLAVE_ADDRESS, 0x23);
		temp_xlsb = BME680_TWI_read(BME680_SLAVE_ADDRESS, 0x24);

		// Store the 20-bit value, right justified, in the temperature_raw global variable
		temperature_raw = temp_msb << 12;
		temperature_raw |= temp_lsb << 4;
		temperature_raw |= temp_xlsb >> 4;
		//temperature_raw = ((temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4));

		// Display the temperature raw value on the LCD
		//sprintf(dsp_buff1, "Status: 0x%02X", status);
		//sprintf(dsp_buff2, "ID: 0x%02X", id);
		//sprintf(dsp_buff3, "Temp Raw: %lu", temperature_raw);
		
		//update_lcd_dog();

		//_delay_ms(1000);
	}
}