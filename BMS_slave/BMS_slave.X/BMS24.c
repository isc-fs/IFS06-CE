/*
 * ISC FS RACING TEAM
 * BMS_SLAVE
 * Slave Battery Management System: up to 24-cell lithium ion voltage and temperature management
 * Author: rmg
 * 
 * MCU: ATMega16M1 (Also works with 32M1/64M1) 
 * Fuses: 8Mhz+ external crystal, CKDIV8 off, brownout 4.2V
 *
 * Created on March 9, 2024, 1:48 AM
 */

#define CAN_BAUD_RATE	250	// Code knows how to do 125, 250, 500, 1000kbps
#define USE_29BIT_IDS	1	// Or 0 for 11-bit IDs
#define DISABLE_TEMPS	0

#define LOW_LTC_CORRECTION		6	// Calibration to account for voltage drop through buffer resistors and LTC6802 variations
#define HIGH_LTC_CORRECTION		6	// Typical value is about 6 for both of these, but may be +/-3mV in some cases

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>

#define BASE_ID	300 // Starting ID used for BMS module messaging to/from EVMS
enum { BMS12_REQUEST_DATA, BMS12_REPLY1, BMS12_REPLY2, BMS12_REPLY3, BMS12_REPLY4 };

#define true 1
#define false 0

#define COMMS_TIMEOUT	32 // at 32Hz, i.e 1 second timeout

// SPI interface pins
#define	CSBI		(1<<PC5)
#define CSBI_PORT	PORTC
#define	SDO			(1<<PC4)
#define SDO_PORT	PINC
#define SDI			(1<<PC6)
#define SDI_PORT	PORTC
#define SCKI		(1<<PB3)
#define SCKI_PORT	PORTB

#define	CSBI2		(1<<PB6)
#define CSBI2_PORT	PORTB
#define	SDO2		(1<<PB5)
#define SDO2_PORT	PINB
#define SDI2		(1<<PB7)
#define SDI2_PORT	PORTB
#define SCKI2		(1<<PD0)
#define SCKI2_PORT	PORTD

// Status LED
#define GREEN_PORT	PORTC
#define GREEN		(1<<PC1)
#define RED_PORT	PORTD
#define RED			(1<<PD3)

// LTC6802 Command codes
#define WRCFG	0x01
#define RDCFG	0x02
#define	RDCV	0x04
#define RDTMP	0x08
#define STCVAD	0x10
#define STTMPAD	0x30

// Inputs from hex rotary pot for module ID selection
#define	MOD_ID_NUM8		(PIND & (1<<PD5))
#define MOD_ID_NUM4		(PIND & (1<<PC7))
#define MOD_ID_NUM2		(PINB & (1<<PB2))
#define MOD_ID_NUM1		(PIND & (1<<PD6))

// Function declarations
void SetupPorts();
int LineariseTemp(int adc);
void CanTX(long packetID, char bytes);
void WriteSPIByte(unsigned char byte);
unsigned char ReadSPIByte();
void WriteSPIByte2(unsigned char byte);
unsigned char ReadSPIByte2();

// Global variables
uint8_t txData[8]; // CAN transmit buffer
volatile uint8_t dataRequestedL = false; // Low group, LTC #2
volatile uint8_t dataRequestedH = false; // High group, LTC #1

uint16_t moduleID = 0;

short commsTimer = 0;

volatile int temp[4]; // In deg C
volatile int voltage[24]; // In millivolts

volatile uint16_t shuntVoltage; // In millivolts
uint32_t shuntBits;


SIGNAL(CAN_INT_vect) // Interrupt function when a new CAN message is received
{
	uint8_t rxData[8];
	int8_t length, savedCANPage;
	savedCANPage = CANPAGE; // Saves current MOB
	CANPAGE = CANHPMOB & 0xF0; // Selects MOB with highest priority interrupt
	if (CANSTMOB & (1<<RXOK))
	{
		length = CANCDMOB & 0x0F; // Number of bytes to receive is bottom four bits of this reg
		for (int8_t i=0; i<length; i++) rxData[i] = CANMSG; // This autoincrements when read
		
		// ID has top 8 bits in IDT1 and bottom 3 bits at TOP of IDT2
		long rxPacketID;
		if (USE_29BIT_IDS)
			rxPacketID = (((long)CANIDT1)<<21) + (((long)CANIDT2)<<13) + (CANIDT3<<5) + (CANIDT4>>3);
		else
			rxPacketID = (CANIDT1<<3) + (CANIDT2>>5);

		// Only one packet we care about - the data request
		if (rxPacketID == moduleID) // Request is for us
		{
			shuntVoltage = (rxData[0]<<8) + rxData[1]; // Big endian format (high byte first)
			dataRequestedL = true;
		}

		if (rxPacketID == moduleID+10) // Or the next ID
		{
			shuntVoltage = (rxData[0]<<8) + rxData[1];
			dataRequestedH = true;
		}

		CANCDMOB = (1<<CONMOB1) | (8<<DLC0) | ((1<<IDE)*USE_29BIT_IDS); // Enable reception, data length 8
		// Note: The DLC field of CANCDMOB register is updated by the received MOB, and if it differs from above, an error is set
	}
	CANSTMOB = 0x00; // Reset interrupt reason on selected channel
	CANPAGE = savedCANPage;
}

void main()
{
	_delay_ms(100); // Allow everything to stabilise on startup

	SetupPorts();
	
	GetModuleID();

	// Initialising variables
	for (int n=0; n<24; n++) voltage[n] = 0;
	for (int n=0; n<4; n++) temp[n] = 0;

	sei(); // Enable interrupts
	wdt_enable(WDTO_120MS); // Enable watchdog timer

	unsigned char cellBytes[2][25];
	unsigned char tempBytes[2][5];
	short voltages[24][8];
	short tempBuffer[4][8];
	
	char counter = 0;
	char slowCounter = 0;
	while (1)
	{
		wdt_reset();

		// Comms with LTC6802s..
		// Split up the 32-bit shuntBits variable into two 12-bit chunks for each LTC
		uint32_t shuntBitsL = shuntBits&0x0FFF; // Lower 12 bits
		uint32_t shuntBitsH = shuntBits>>12; // Upper 12 bits

		// Write config registers, LTC #1, which is the left side (more positive)
		CSBI_PORT &= ~CSBI; // Pull down to start command
		WriteSPIByte(WRCFG); // write configuration group
		WriteSPIByte(0b00000001);
		WriteSPIByte(((unsigned short)(shuntBitsH & 0x00FF))); // Bottom byte of shunt bits
		WriteSPIByte(((unsigned short)(shuntBitsH>>8))); // Top four bits of shunt bits, right shifted 1 byte
		WriteSPIByte(0b00000000);
		WriteSPIByte(0b00000000);
		WriteSPIByte(0b00000000);	
		CSBI_PORT |= CSBI; // Pull up to end command
		_delay_us(100);

		// Start voltage sampling
		CSBI_PORT &= ~CSBI;
		WriteSPIByte(STCVAD);
		CSBI_PORT |= CSBI;

		// Write config registers, LTC #2, which is the right side (more negative)
		CSBI2_PORT &= ~CSBI2; // Pull down to start command
		WriteSPIByte2(WRCFG); // write configuration group
		WriteSPIByte2(0b00000001);
		WriteSPIByte2(((unsigned short)(shuntBitsL & 0x00FF))); // Bottom byte of shunt bits
		WriteSPIByte2(((unsigned short)(shuntBitsL>>8))); // Top four bits of shunt bits, right shifted 1 byte
		WriteSPIByte2(0b00000000);
		WriteSPIByte2(0b00000000);
		WriteSPIByte2(0b00000000);	
		CSBI2_PORT |= CSBI2; // Pull up to end command
		_delay_us(100);

		// Start voltage sampling
		CSBI2_PORT &= ~CSBI2;
		WriteSPIByte2(STCVAD);
		CSBI2_PORT |= CSBI2;

		_delay_ms(20); // Cell sampling can take up to 16ms - anything we need to do in the meantime? Not really.
	
		// Start temperature sampling
		CSBI_PORT &= ~CSBI;
		WriteSPIByte(STTMPAD);
		CSBI_PORT |= CSBI;

		// Start temperature sampling, ltc #2
		CSBI2_PORT &= ~CSBI2;
		WriteSPIByte2(STTMPAD);
		CSBI2_PORT |= CSBI2;

		_delay_ms(5); // Temp sampling should only take ~3ms
	
		// Read cell voltage registers
		CSBI_PORT &= ~CSBI;
		WriteSPIByte(RDCV);
		SDI_PORT |= SDI;
		for (int n=0; n<25; n++) cellBytes[0][n] = ReadSPIByte();
		CSBI_PORT |= CSBI;
		_delay_us(100);

		// Read temperature data
		CSBI_PORT &= ~CSBI;
		WriteSPIByte(RDTMP);
		SDI_PORT |= SDI;
		for (int n=0; n<5; n++) tempBytes[0][n] = ReadSPIByte();
		CSBI_PORT |= CSBI;
		_delay_us(100);

		// Read cell voltage registers, ltc #2
		CSBI2_PORT &= ~CSBI2;
		WriteSPIByte2(RDCV);
		SDI2_PORT |= SDI2;
		for (int n=0; n<25; n++) cellBytes[1][n] = ReadSPIByte2();
		CSBI2_PORT |= CSBI2;
		_delay_us(100);

		// Read temperature data, ltc #2
		CSBI2_PORT &= ~CSBI2;
		WriteSPIByte2(RDTMP);
		SDI2_PORT |= SDI2;
		for (int n=0; n<5; n++) tempBytes[1][n] = ReadSPIByte2();
		CSBI2_PORT |= CSBI2;
		_delay_us(100);

		// Extract voltage data
		for (int n=0; n<12; n+=2)
		{
			voltages[n][counter] = (cellBytes[1][n*3/2] + 256 * (cellBytes[1][n*3/2+1] & 0x0F))*3/2;
			voltages[n+1][counter] = (((cellBytes[1][n*3/2+1] & 0xF0)>>4) + cellBytes[1][n*3/2+2]*16)*3/2;
		}
		for (int n=0; n<12; n+=2)
		{
			voltages[12+n][counter] = (cellBytes[0][n*3/2] + 256 * (cellBytes[0][n*3/2+1] & 0x0F))*3/2;
			voltages[12+n+1][counter] = (((cellBytes[0][n*3/2+1] & 0xF0)>>4) + cellBytes[0][n*3/2+2]*16)*3/2;
		}

		// Extract temperature data
		tempBuffer[0][counter] = tempBytes[1][0] + 256 * (tempBytes[1][1] & 0x0F); // gives mV
		tempBuffer[1][counter] = ((tempBytes[1][1] & 0xF0)>>4) + tempBytes[1][2]*16; // gives mV
		tempBuffer[2][counter] = tempBytes[0][0] + 256 * (tempBytes[0][1] & 0x0F); // gives mV
		tempBuffer[3][counter] = ((tempBytes[0][1] & 0xF0)>>4) + tempBytes[0][2]*16; // gives mV

		counter++;
		if (counter >= 8) // Slow loop, about 4Hz
		{
			counter = 0;

			slowCounter++;
			if (slowCounter >= 4) slowCounter = 0;

			char notAllZeroVolts = false;
			for (int n=0; n<24; n++) // Calculate average voltage over last 8 samples, and update shunts if required
			{
				int average = 0;
				for (int c=0; c<8; c++) average += voltages[n][c]/2;
				voltage[n] = average>>2;

				int correction = LOW_LTC_CORRECTION;
				if (n >= 12) correction = HIGH_LTC_CORRECTION;
				if (voltage[n] > 0)
				{
					voltage[n] += correction;
					if (n==0 || n==12) voltage[n] -= correction/2; // First cells have less drop due to single 3.3Kohm resistor in play
				}

				if (voltage[n] > 5000) // Probably means no cells are plugged in to power the LTC
					voltage[n] = 0;

				if (voltage[n] > 0) notAllZeroVolts = true;

				if (voltage[n] > shuntVoltage && shuntVoltage > 0)
					shuntBits |= (1<<n);
				else
					shuntBits &= ~(1<<n);
			}

			// Calculate temperature averages
			for (int n=0; n<4; n++)
			{
				int average = 0;
				for (int c=0; c<8; c++) average += tempBuffer[n][c];
				temp[n] = average>>3;
			}

			// Update Status LED(s)
			RED_PORT &= ~RED; // Most cases have red light off and green on
			GREEN_PORT |= GREEN;
			if (shuntBits != 0 & slowCounter&0x01) // Red/orange flash if shunting
				RED_PORT |= RED;
//			else if (!notAllZeroVolts) // Blink red if no cells detected
//			{
//				GREEN_PORT &= ~GREEN;
//				if (slowCounter&0x01) RED_PORT |= RED;
//			}
			else if (commsTimer == COMMS_TIMEOUT && slowCounter&0x01) // Blink green if no CAN comms
				GREEN_PORT &= ~GREEN;
		}

		if (commsTimer < COMMS_TIMEOUT)
			commsTimer++;
		else
			shuntVoltage = 0; // If comms times out, kill all shunt balancers just to be safe

		if (dataRequestedL)
		{
			dataRequestedL = false;
			commsTimer = 0;

			// Voltage packets
			for (int packet=0; packet<3; packet++)
			{
				for (int n=0; n<4; n++)
				{
					txData[n*2] = voltage[packet*4+n]>>8; // Top 8 bits
					txData[n*2+1] = voltage[packet*4+n]&0xFF; // Bottom 8 bits
				}
				CanTX(moduleID+packet+1, 8);
				_delay_ms(1); // Brief intermission between packets?
			}

			// Temperature packet
			txData[0] = LineariseTemp(temp[0]);
			txData[1] = LineariseTemp(temp[1]);
			txData[2] = txData[3] = txData[4] = txData[5] = txData[6] = txData[7] = 0; // Zeroes, reserved
			CanTX(moduleID + BMS12_REPLY4, 8);
			
		}
		else if (dataRequestedH)
		{
			dataRequestedH = false;
			commsTimer = 0;

			// Voltage packets
			for (int packet=0; packet<3; packet++)
			{
				for (int n=0; n<4; n++)
				{
					txData[n*2] = voltage[12+packet*4+n]>>8; // Top 8 bits
					txData[n*2+1] = voltage[12+packet*4+n]&0xFF; // Bottom 8 bits
				}
				CanTX(moduleID+10+packet+1, 8);
				_delay_ms(1); // Brief intermission between packets?
			}

			// Temperature packet
			txData[0] = LineariseTemp(temp[2]);
			txData[1] = LineariseTemp(temp[3]);
			txData[2] = txData[3] = txData[4] = txData[5] = txData[6] = txData[7] = 0; // Zeroes, reserved
			CanTX(moduleID + 10 + BMS12_REPLY4, 8);
			
		}
		else
			_delay_ms(4); // Talking to LTC takes 27ms, so this makes it 31ms, which inverts to about 32Hz. Accuracy not important.

		GetModuleID(); // Update in case it changed at runtime
	}
}

void CanTX(long packetID, char bytes)
{
	CANPAGE = 0x00; // Select MOB0 for transmission
	while (CANEN2 & (1<<ENMOB0)); // Wait for MOB0 to be free
	CANSTMOB = 0x00;

	if (USE_29BIT_IDS) // CAN 2.0b is 29-bit IDs, CANIDT4 has bits 0-4 in top 5 bits, CANID3 has 5-12
	{
		CANIDT1 = packetID>>21;
		CANIDT2 = packetID>>13;
		CANIDT3 = packetID>>5;
		CANIDT4 = (packetID & 0b00011111)<<3;
	}
	else // CAN 2.0a is 11-bit IDs, IDT1 has top 8 bits, IDT2 has bottom three bits BUT at top of byte!
	{
		CANIDT1 = (packetID>>3); // Packet ID
		CANIDT2 = (packetID & 0x07)<<5;
		CANIDT3 = 0x00;
		CANIDT4 = 0x00;
	}

	for (int8_t i=0; i<bytes; i++) CANMSG = txData[i];

	CANCDMOB = (1<<CONMOB0) | (bytes<<DLC0) | ((1<<IDE)*USE_29BIT_IDS); // Enable transmission, 8-bit data

	while (!(CANSTMOB & (1<<TXOK)) && !(CANSTMOB & (1<<AERR))); // Wait for transmission to finish (via setting of TXOK flag)

	CANCDMOB = 0x00; // Disable transmission
	CANSTMOB = 0x00; // Clear TXOK flag
}

// Function to convert ADC level to temperature. From datasheet for Epcos 100Kohm NTC B25/100 of 4540 K
// Temps: 	-40  -30  -20  -10    0   10   20   30   40   50   60   70   80    90    100   110   120   130   140   150  160
// Rt/R25: 46.4 23.3 12.2  6.61 3.71 2.15 1.28 0.78 0.49 0.32 0.21 0.14 0.095 0.066 0.047 0.033 0.024 0.018 0.014 0.010 0.08
// ADC scale seems to be 0-2048, but slight drop due to input impedance makes scale 0-2030 or so
int tempData[21] = { 1987,1946,1876,1763,1599,1386,1140,890,668,492,352,249,176,126,91,65,48,36,28,20,16 };

int LineariseTemp(int adc)
{
	if (adc > 1950 || DISABLE_TEMPS) return 0; // Temporary, sometimes unplugged temp gives like -30degC instead of 0
	
	for (int n=0; n<20; n++)
		if (adc <= tempData[n] && adc > tempData[n+1]) // We're between samples  
			return n*10 + 10*(adc - tempData[n])/(tempData[n+1] - tempData[n]); // Calculate linear interpolation

	return 0; // i.e "not available" if the algorithm below doesn't find it (-40 to 160degC)
}

void SetupPorts()
{
	DDRB = 0b11001000; // PB3 = SCKI, PB6 = CSBI2, PB7 = SDI2
	DDRC = 0b01100010; // PC1 = GREEN, PC5 = CSBI, PC6 = SDI
	DDRD = 0b00001001; // PD0 = VIA2, PD3 = RED

	// Pull-ups for module ID selectors
	PORTB = 0b00000100;
	PORTC = 0b00000000;
	PORTD = 0b11100000;

	// CAN init stuff. Further info on page 203 of ATmega16M1 manual
	CANGCON = (1<<SWRES); // Software reset
	CANTCON = 0; // CAN timing prescaler set to 0

	if (CAN_BAUD_RATE == 1000)
		CANBT1 = 0x00;
	else if (CAN_BAUD_RATE == 500)
		CANBT1 = 0x02;
	else if (CAN_BAUD_RATE == 250)
		CANBT1 = 0x06;
	else
		CANBT1 = 0x0E;
	CANBT2 = 0x04;
	if (CAN_BAUD_RATE == 1000)
		CANBT3 = 0x12;
	else
		CANBT3 = 0x13;

	for (int8_t mob=0; mob<6; mob++)
	{
		CANPAGE = (mob<<4); // Select MOB 0-5
		CANCDMOB = 0x00; // Disable MOB
		CANSTMOB = 0x00; // Clear MOB status register
	}

	CANPAGE = (1<<MOBNB0); // Select MOB1
	CANIE2 = (1<<IEMOB1); // Enable interrupts on MOB1 for reception and transmission
	CANGIE = (1<<ENIT) | (1<<ENRX); // Enable interrupts on receive and transmit
	CANIDM1 = CANIDM2 = CANIDM3 = CANIDM4 = 0x00; // CAN ID mask, zero will let all IDs pass
	CANCDMOB = (1<<CONMOB1) | (8<<DLC0) | ((1<<IDE)*USE_29BIT_IDS); // Enable reception, 11-bit IDE, 8-bit data length
	CANGCON |= (1<<ENASTB); // Enable mode. CAN channel enters enable mode after 11 recessive bits have been read
}

void GetModuleID()
{
	int rotarySwitch = 0;

	if (!MOD_ID_NUM1) rotarySwitch += 1;
	if (!MOD_ID_NUM2) rotarySwitch += 2;
	if (!MOD_ID_NUM4) rotarySwitch += 4;
	if (!MOD_ID_NUM8) rotarySwitch += 8;
	moduleID = BASE_ID+rotarySwitch*10; // Atomic assignment in case of interrupt
}

void WriteSPIByte(unsigned char byte)
{
	unsigned short mask = 0b10000000; // MSB first
	for (int n=0; n<8; n++) // 8 bits = 1 byte
	{
		// Prepare pin
		if ((byte & mask) > 0) // Bit is on
			SDI_PORT |= SDI;
		else
			SDI_PORT &= ~SDI;

		_delay_us(2);
		SCKI_PORT |= SCKI; // Clock goes high = register SDI state
		_delay_us(2);
		SCKI_PORT &= ~SCKI; // Clock goes low
		mask = mask>>1;
	}
}

unsigned char ReadSPIByte()
{
	unsigned char byte = 0;
	unsigned char mask = 0b10000000;
	for (int n=0; n<8; n++)
	{
		_delay_us(2);
		SCKI_PORT |= SCKI; // Clock goes high = register SDI state
		_delay_us(2);
		if (SDO_PORT & SDO) byte += mask;
		SCKI_PORT &= ~SCKI; // Clock goes low
		mask = mask>>1;
	}
	return byte;
}

void WriteSPIByte2(unsigned char byte)
{
	unsigned short mask = 0b10000000; // MSB first
	for (int n=0; n<8; n++) // 8 bits = 1 byte
	{
		// Prepare pin
		if ((byte & mask) > 0) // Bit is on
			SDI2_PORT |= SDI2;
		else
			SDI2_PORT &= ~SDI2;

		_delay_us(2);
		SCKI2_PORT |= SCKI2; // Clock goes high = register SDI state
		_delay_us(2);
		SCKI2_PORT &= ~SCKI2; // Clock goes low
		mask = mask>>1;
	}
}

unsigned char ReadSPIByte2()
{
	unsigned char byte = 0;
	unsigned char mask = 0b10000000;
	for (int n=0; n<8; n++)
	{
		_delay_us(2);
		SCKI2_PORT |= SCKI2; // Clock goes high = register SDI state
		_delay_us(2);
		if (SDO2_PORT & SDO2) byte += mask;
		SCKI2_PORT &= ~SCKI2; // Clock goes low
		mask = mask>>1;
	}
	return byte;
}
