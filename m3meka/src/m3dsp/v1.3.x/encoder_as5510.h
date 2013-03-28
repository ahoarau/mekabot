#ifndef _ENCODER_AS5510_H
#define _ENCODER_AS5510_H

#ifdef USE_AS5510

//Definitions:

#define AS5510_I2C_ADDR			0xAC

#define AS5510_SENSITIVITY_50MT	0	// +- 50mT
#define AS5510_SENSITIVITY_25MT	1	// +- 25mT
#define AS5510_SENSITIVITY_12MT	2	// +- 12.5mT
#define AS5510_SENSITIVITY_18MT	3	// +- 18.75mT

#define AS5510_MODE_FAST			0x00
#define AS5510_MODE_SLOW			0x04
#define AS5510_POLARITY_NORMAL		0x00
#define AS5510_POLARITY_INVERSED	0x02
#define AS5510_POWERDOWN_OFF		0x00
#define AS5510_POWERDOWN_ON		0x01

//Prototypes:

short get_as5510_raw();
void step_as5510();
void setup_as5510(void);
void configure_as5510(unsigned char mode, unsigned char sensitivity);
unsigned char read_byte_i2c(unsigned char slave_adress, unsigned char offset);
void write_byte_i2c(unsigned char slave_adress, unsigned char offset, char data);
short get_magnetic_field(unsigned char sensitivity_mode);
void setup_I2C1(void);

//Variables:

extern unsigned char as5510_i2c_buffer;
extern short as5510_raw_value;
extern short as5510_value;

#endif
#endif
