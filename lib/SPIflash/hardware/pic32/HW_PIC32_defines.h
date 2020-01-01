// *** Hardwarespecific defines ***
#define cbi(reg, bitmask) (*(reg + 1)) = bitmask
#define sbi(reg, bitmask) (*(reg + 2)) = bitmask

#define PROGMEM
#define regtype volatile uint32_t
#define regsize uint16_t

// *** Hardware SPI-specific defines
#define SPI_CLOCK_20MHZ		0x01
#define SPI_CLOCK_13MHZ		0x02
#define SPI_CLOCK_10MHZ		0x03
#define SPI_CLOCK_8MHZ		0x04
#define SPI_CLOCK_4MHZ		0x09
#define SPI_CLOCK_2MHZ		0x13
#define SPI_CLOCK_1MHZ		0x27

#define SPI_CLOCK_DEFAULT	0x02
