void SPIflash::_SPIstart(uint8_t rate)
{
	digitalWrite(SS, HIGH);
	pinMode(SS, OUTPUT);
	digitalWrite(_pinCE, HIGH);
	pinMode(_pinCE, OUTPUT);
	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);
	pinMode(MISO, INPUT);

	SPCR |= _BV(MSTR);
	SPCR |= _BV(SPE);

	SPCR = (SPCR & ~SPI_CLOCK_MASK) | (rate & SPI_CLOCK_MASK);
	SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((rate >> 2) & SPI_2XCLOCK_MASK);
}

void SPIflash::_SPIwrite(byte data)
{
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));
}

byte SPIflash::_SPIread(void)
{
	SPDR = 0x00;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}
