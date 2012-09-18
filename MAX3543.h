/*
 * MAX3543.h
 *
 * Copyright (c) 2012, Gabriel Fournier <gabriel@gaftech.fr>
 *
 * This file is part of ArduinoMAX3543 project.
 * Pleased read attached LICENSE file.
 *
 *  Created on: 1 juil. 2012
 *      Author: Gabriel Fournier
 */

#include <Arduino.h>

#ifndef MAX3543_H_
#define MAX3543_H_

/*******
 * TYPES
 */
typedef  unsigned short UINT_16; /* compiler type for 16 bit unsigned integer */
typedef  unsigned long	UINT_32; /* compiler type for 32 bit unsigned integer */
typedef	 byte			UINT_8;

/****************
 * MISC CONSTANTS
 */

/* Functions and methods return codes */
#define MAX3543_SUCCESS		0
#define MAX3543_ERROR		1
#define MAX3543_MHZ			1000000
#define MAX3543_2POW20		1048576

/* Frequency params (Hz) */
#ifndef MAX3543_IF_FREQUENCY
#define MAX3543_IF_FREQUENCY	36000000
#endif
#ifndef MAX3543_REF_FREQUENCY
#define MAX3543_REF_FREQUENCY	16000000
#endif

/************
 * CHIP BYTES
 */

/* Reference divider */
#define MAX3543_RDIV_1				0
#define MAX3543_RDIV_2				1
#define MAX3543_RDIV_AUTO			2
#ifndef MAX3543_RDIV
#define MAX3543_RDIV				MAX3543_RDIV_1
#endif

/* XO divider (REFOUT) */
#define MAX3543_XODIV_4				0	// divide by 4
#define MAX3543_XODIV_1				1
#ifndef MAX3543_XODIV
#define MAX3543_XODIV				MAX3543_XODIV_4
#endif

/* Band */
#define MAX3543_VHF_L	0
#define MAX3543_VHF_H	1
#define MAX3543_UHF 	2
/* Mode control */
#define MAX3543_LNA2G_LOW			0
#define MAX3543_LNA2G_NOM			1
#ifndef MAX3543_LNA2G
	#define MAX3543_LNA2G			MAX3543_LNA2G_NOM
#endif
#define MAX3543_BW_7MHZ				0
#define MAX3543_BW_8MHZ				1
#ifndef MAX3543_BW
	#define MAX3543_BW				MAX3543_BW_7MHZ
#endif
#define MAX3543_IFSEL_DTV_DIFF		0
#define MAX3543_IFSEL_DTV_SE		1
#define MAX3543_IFSEL_ATV			2
#ifndef MAX3543_IFSEL
	#define	MAX3543_IFSEL			MAX3543_IFSEL_DTV_DIFF
#endif
/* Tracking filter */
#define MAX3543_SER0 0
#define MAX3543_SER1 1
#define MAX3543_PAR0 2
#define MAX3543_PAR1 3
/* IF VGA shutdown */
#define MAX3543_SDIFVG_ENABLED		0
#define MAX3543_SDIFVG_DISABLED		1
#ifndef MAX3543_SDIFVG
	#define MAX3543_SDIFVG			MAX3543_SDIFVG_ENABLED
#endif
/* VAS */
#if (MAX3543_REF_FREQUENCY >= 28 * MAX3543_MHZ)
#define MAX3543_LFDIV			2
#elif (MAX3543_REF_FREQUENCY >= 20 * MAX3543_MHZ)
#define MAX3543_LFDIV			1
#else
#define MAX3543_LFDIV 			0
#endif
#define MAX3543_VASS_0			0
#define MAX3543_VASS_1			1
#ifndef MAX3543_VASS
#define MAX3543_VASS			MAX3543_VASS_0
#endif
#define MAX3543_VAS				1
#define MAX3543_ADL_DISABLE		0
#define MAX3543_ADL_LATCH		1
#ifndef MAX3543_ADL
#define MAX3543_ADL				MAX3543_ADL_DISABLE
#endif
#define MAX3543_ADE_DISABLE		0
#define MAX3543_ADE_ENABLE		1
#ifndef MAX3543_ADE
#define	MAX3543_ADE				MAX3543_ADE_ENABLE
#endif
#define MAX3543_LTC				2 // Note: datasheet says it must be 2 but Maxim eval soft set this to 3
/* POWER DETECTOR */
#define MAX3543_DWPD_ENABLE	0
#define MAX3543_DWPD_DISABLE	1
#ifndef MAX3543_DWPD
#define MAX3543_DWPD			MAX3543_DWPD_ENABLE
#endif
#define MAX3543_WPDA_MIN		0
#define MAX3543_WPDA_NOM		4
#define MAX3543_WPDA_MAX		7
#ifndef MAX3543_WPDA
#define MAX3543_WPDA			MAX3543_WPDA_NOM
#endif
#define MAX3543_DNPD_ENABLE		0
#define MAX3543_DNPD_DISABLE	1
#ifndef MAX3543_DNPD
#define MAX3543_DNPD			MAX3543_DNPD_ENABLE
#endif
#define MAX3543_NPDA_MIN		0
#define	MAX3543_NPDA_NOM		3
#define MAX3543_NPDA_MAX		7
#ifndef MAX3543_NPDA
#define MAX3543_NPDA			MAX3543_NPDA_NOM
#endif
#define MAX3543_RFIFD_600MV		0
#define MAX3543_RFIFD_950MV		1
#define MAX3543_RFIFD_1300MV	2
#define MAX3543_RFIFD_OFF		3
#ifndef MAX3543_RFIFD
#define MAX3543_RFIFD			MAX3543_RFIFD_950MV
#endif
/* BIAS adjustment */
#define MAX3543_MIXGM_ATV		0
#define MAX3543_MIXGM_DTV 		1
#ifndef MAX3543_MIXGM
	#define MAX3543_MIXGM		MAX3543_MIXGM_DTV
#endif
#define MAX3543_LNA2B_NOM 		1
#define MAX3543_LNA2B_HIGH 		3
#ifndef MAX3543_LNA2B
	#define MAX3543_LNA2B 		MAX3543_LNA2B_NOM
#endif
#define MAX3543_MIXB_NOM 		1
#define MAX3543_MIXB_HIGH 		3
#ifndef MAX3543_MIXB
	#define MAX3543_MIXB 		MAX3543_MIXB_NOM
#endif
#define MAX3543_FILTB			1
#define MAX3543_IFVGAB_DEFAULT 	0
#define MAX3543_IFVGAB_HIGH 	1
#ifndef MAX3543_IFVGAB
	#define MAX3543_IFVGAB 		MAX3543_IFVGAB_DEFAULT
#endif

/*************
 * MISC PARAMS
 */

/* Debug & tests */

/* Calc mode */

// Note: by default, frequency calculations use floats.

// Alternate frequency calculation modes: Following flags can be defined to override this behaviour:
//#define MAX3543_INT_MODE: perform frequency calculations using integer.
//#define MAX3543_MAXIM_MODE: perform frequency calculations the way Maxim's driver does.

// Integer mode params
#ifndef MAX3543_FRAC_SCALE
#define MAX3543_FRAC_SCALE		976		// Fractional scale as calculated in max3543_int_calc.ods
#endif

// Maxim mode params
#ifndef MAX3543_LOSCALE
#define MAX3543_LOSCALE			65		// Scaling factor for the IF and RF frequencies.
#endif
#ifndef MAX3543_XTALSCALE
#define MAX3543_XTALSCALE		32		// Scaling factor for Xtal frequency.
#endif



// I2C params
#define MAX3543_I2C_ADDRESS		0x61	// 7 bits i2c address

class MAX3543
{
public:

	MAX3543();
	byte init();

	/****************************
	 * Direct register accessors.
	 *
	 * Setters affect both register cache and device registers
	 * Getters update cache then return value
	 */

	/* tune device at `freq` Hz */
	byte tune(UINT_32 freq);

	unsigned long readFrequency();

	bool pllLocked();

	/************************
	 * Frequency calculations
	 */

	/* put in `res` appropriate register values to tune device at `freq` Hz */
	byte computeTuneRegisters(byte res[8], UINT_32 freq);

	/* returns current frequency in Hz */
	UINT_32 computeFrequency(const byte regs[8]);

	/**************************
	 * Register cache accessors
	 *
	 * Return cached values (i.e. no i2c transfer performed)
	 */

	UINT_32 getFrequency();

	byte getRegister(byte address);

	byte getRegisters(byte * regs, byte start, byte stop);

	void debugRegisters(byte * regs, byte start, byte stop);

	/***********************
	 * Register cache update
	 */

	/* Read all registers */
	byte readRegisters();

	/* Read one register, at specified address */
	byte readRegister(byte address);

	/* Read registers from `start` to `stop` */
	byte readRegisters(byte start, byte stop);



private:
	byte registers[23];
	byte tfRomCoefs[3][4];

	/***********************
	 * Frequency calculation
	 */

	/*
	 * Calculate approximation for Max3540 tracking filter using integer math only.
	 *
	 * Almost copy/paste from Maxim driver
	 */
	byte tfs_i(byte S0, byte S1, UINT_32 FreqRF, const byte c[5]);

	/***************************
	 * Direct register accessors
	 */

	byte writeRegister(byte address, byte value);

	/*****************
	 * Cache accessors
	 */

	/* Updates register value array (0x00 to 0x07) to match an RF input of `freq` Hz. */
	byte setTuneRegisters(UINT_32 frequency);

	/******************************
	 * Registers-to-cache accessors
	 */
	byte writeRegisters(byte byteAddress);
	byte writeRegisters(byte start, byte stop);
	byte readROM();



};

#endif /* MAX3543_H_ */
