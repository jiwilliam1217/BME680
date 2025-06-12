/*
 * BME680_write_read_test.c
 *
 * Created: 4/26/2023 3:37:46 PM
 * Author : wji
 */ 

#define F_CPU 4000000
#define USART3_BAUD_RATE(BAUD_RATE) ((float)(4000000 * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

// Display buffer for DOG LCD using sprintf()
char dsp_buff1[17];
char dsp_buff2[17];
char dsp_buff3[17];

#define CLK_PER                                         4000000     // 4MHz default clock
#define TWI0_BAUD(F_SCL, T_RISE)                        ((((((float)CLK_PER / (float)F_SCL)) - 10 - ((float)CLK_PER * T_RISE))) / 2)

void TWI0_BME680_init (void);
uint8_t BME680_TWI_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t data_byte);
uint8_t BME680_TWI_read(uint8_t slave_addr, uint8_t read_reg);



static void TWI0_BME680_init (void)
{
	TWI0.MCTRLA = TWI_ENABLE_bm; //enables I2C
	TWI0.MBAUD = (uint8_t)TWI0_BAUD(I2C_SCL_FREQ, 0); //setting baud rate

	PORTA.DIRSET = PIN2_bm; // Set PA2 as an output
	PORTA.DIRSET = PIN3_bm; // Set PA3 as an output
	
	TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}



static uint8_t BME680_TWI_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t data_byte)
{
	uint16_t data;

	// Send slave address and R/W bit
	TWI0.MADDR = address;
	//while (!I2C_DATA_RECEIVED);    //wait for ack

	// Read temperature register high byte
	data = TWI0.MDATA;
	data = data << 8; //shift
	TWI0.MCTRLB = TWI_ACKACT_ACK_gc | TWI_MCMD_RECVTRANS_gc;// send ack
	//while (!I2C_DATA_RECEIVED);

	// Read temperature register low byte
	data |= TWI0.MDATA;
	TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;

	// Mask
	data = 0xFF80 & data;
		
		return data;
}



static uint8_t BME680_TWI_read(uint8_t slave_addr, uint8_t read_reg)
{
	uint16_t data;

	// Send slave address and R/W bit
	TWI0.MADDR = address;
	while (!I2C_DATA_RECEIVED);    //wait for ack

	// Read temperature register high byte
	data = TWI0.MDATA;
	data = data << 8; //shift
	TWI0.MCTRLB = TWI_ACKACT_ACK_gc | TWI_MCMD_RECVTRANS_gc;// send ack
	while (!I2C_DATA_RECEIVED);

	// Read temperature register low byte
	data |= TWI0.MDATA;
	TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;

	// Mask
	data = 0xFF80 & data;
	
	return data;
}



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
	PORTA.DIR |= PIN4_bm; // MOSI channel
	PORTA.DIR &= ~PIN5_bm; // MISO channel
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
	
	init_lcd_dog();
	
	while(1) {
		
		update_lcd_dog();
	}
}

