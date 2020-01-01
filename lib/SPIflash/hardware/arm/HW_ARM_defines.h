// *** Hardwarespecific defines ***
#define cbi(reg, bitmask) *reg &= ~bitmask
#define sbi(reg, bitmask) *reg |= bitmask

#define regtype volatile uint32_t
#define regsize uint32_t

//#define F(arg)	(arg)

// *** Hardware SPI-specific defines
#define SPI_CLOCK_4MHZ		0x15
#define SPI_CLOCK_21MHZ		0x04

#define SPI_CLOCK_DEFAULT	0x04

#define SPI_CHIP_SEL EEPROM_CS
