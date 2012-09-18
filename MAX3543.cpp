/*
 * MAX3543.cpp
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
#include "MAX3543.h"

#include <Wire.h>
#include <new.h>

/* table of fixed coefficients for the tracking filter equations. (Maxim's driver) */
static const UINT_8 MAX3543_TF_COEFS[6][5] =	{
		{ 26, 6,  68, 20, 45 }, /* VHF LO TFS */
		{ 16, 8,  88, 40, 0 },  /* VHF LO TFP */
		{ 27, 10, 54, 30, 20 }, /* VHF HI TFS */
		{ 18, 10, 41, 20, 0 },  /* VHF HI TFP */
		{ 32, 10, 34, 8,  10 }, /* UHF TFS */
		{ 13, 15, 21, 16, 0 } 	/* UHF TFP */
};

MAX3543::MAX3543() {
}

byte MAX3543::init() {
	Wire.begin();
	// Reading initial registers
	if ( readRegisters(0x00, 0x15) != MAX3543_SUCCESS ) return MAX3543_ERROR;

	// Setting registers R08 to R0C
	registers[0x08] = MAX3543_SDIFVG << 3;
	registers[0x09] = 0x0A | MAX3543_XODIV;
	registers[0x0A] = (MAX3543_LFDIV <<6) |(MAX3543_VASS << 5)
			| (MAX3543_VAS << 4) | (MAX3543_ADL << 3) | (MAX3543_ADE << 2) | MAX3543_LTC;
	registers[0x0B] = (MAX3543_DWPD << 7) | (MAX3543_WPDA << 4) | (MAX3543_DNPD << 3) | MAX3543_NPDA;
	registers[0x0C] = MAX3543_RFIFD;

	if ( writeRegisters(0x08, 0x0C) != MAX3543_SUCCESS ) return MAX3543_ERROR;

	// R0D to R0F. Init registers (and write them) and vars from ROM values.
	if ( readROM() != MAX3543_SUCCESS ) return MAX3543_ERROR;

	// R13: Bias adjutments
	registers[0x13] = ((MAX3543_MIXGM << 6) | (MAX3543_LNA2B << 4) |
			(MAX3543_MIXB << 2) | (MAX3543_FILTB << 1) | MAX3543_IFVGAB);

	if ( writeRegisters(0x13) != MAX3543_SUCCESS ) return MAX3543_ERROR;

	return MAX3543_SUCCESS;
}

byte MAX3543::tune(unsigned long freq) {
	if ( setTuneRegisters(freq) != MAX3543_SUCCESS )
		return MAX3543_ERROR;
	if ( writeRegisters(0x00, 0x07) != MAX3543_SUCCESS )
		return MAX3543_ERROR;
	return MAX3543_SUCCESS;
}

unsigned long MAX3543::readFrequency() {
	if (readRegisters(0x0, 0x7) != MAX3543_SUCCESS)
		return 0;
	return getFrequency();
}

bool MAX3543::pllLocked() {
	byte vcoadc;
	if (readRegister(0x12) != MAX3543_SUCCESS)
		return false;
	vcoadc = registers[0x12] & 0x7;
	return vcoadc > 0 && vcoadc < 7;
}

byte MAX3543::writeRegister(byte address, byte value) {
	registers[address] = value;
	return writeRegisters(address);
}

UINT_32 MAX3543::getFrequency() {
    return computeFrequency(registers);
}

byte MAX3543::getRegister(byte address) {
	return registers[address];
}

byte MAX3543::getRegisters(byte *regs, byte start, byte stop) {
	if (start < 0x00 || stop > 0x15 || stop < start) return MAX3543_ERROR;
	for (byte i = start; i <= stop; ++i)
		regs[i] = registers[i];
	return MAX3543_SUCCESS;
}

void MAX3543::debugRegisters(byte * regs, byte start, byte stop) {
	for (byte i = start; i <= stop; ++i) {
		Serial.print("0x");
		Serial.print(i, HEX);
		Serial.print(": 0x");
		Serial.println(regs[i], HEX);
	}
}


byte MAX3543::setTuneRegisters(UINT_32 Frf) {
	return computeTuneRegisters(registers, Frf);
}

UINT_32 MAX3543::computeFrequency(const byte regs[8]) {
    byte VdivBin = regs[0x00] & 0x3;
    byte Vdiv = 1 << (VdivBin+2);
    byte NdivI = regs[0x01];
    byte Rdiv = ((regs[0x2] >> 4) & 0x3) + 1;

    UINT_32 NdivF = ((UINT_32)(regs[0x2] & 0xf) << 16) | ((UINT_32)regs[0x3] << 8) | regs[0x4];
    float Ndiv = float(NdivI) + float(NdivF) / MAX3543_2POW20;
    UINT_32 Fvco_kHz = (UINT_32) MAX3543_REF_FREQUENCY / 1000 * Rdiv * Ndiv * 4;
    UINT_32 Flo = ((float)Fvco_kHz / Vdiv) * 1000;
    UINT_32 Frf = Flo - MAX3543_IF_FREQUENCY;

    return Frf;
}

byte MAX3543::computeTuneRegisters(byte res[8], UINT_32 Frf) {

	UINT_32 Flo;
	byte Rdiv;
	byte Vdiv;
	byte VdivBin;
	byte NdivInt;
	UINT_32 NdivFrac;
	byte tfb;
	byte tfs;
	byte tfp;

	/* Flo */
	Flo = Frf + MAX3543_IF_FREQUENCY;

	/* Rdiv */
	Rdiv = 1;
	if ( (MAX3543_RDIV == MAX3543_RDIV_AUTO && Flo <= 275 * MAX3543_MHZ) || (MAX3543_RDIV == MAX3543_RDIV_2) )
		Rdiv = 2;

	/* Vdiv: VCO divider */
	if (Flo < 137.5 * MAX3543_MHZ)
		VdivBin = 3;
	else if (Flo < 275 * MAX3543_MHZ)
		VdivBin = 2;
	else if (Flo < 550 * MAX3543_MHZ)
		VdivBin = 1;
	else
		VdivBin = 0;
	Vdiv = 1 << (VdivBin+2);

	/* Ndiv (PLL divider): int and frac part caclulation */

#if defined(MAX3543_MAXIM_MODE)
	UINT_32 Flo_scaled;
	UINT_32 Num;
	UINT_32 Denom;
	UINT_32 Rem;
	UINT_32 fracscale;
	UINT_32 XtalRef;

	Flo_scaled = (float) Flo / MAX3543_MHZ * MAX3543_LOSCALE;
	XtalRef = (float) MAX3543_REF_FREQUENCY / MAX3543_MHZ * MAX3543_XTALSCALE;

	Denom = XtalRef * 4 * MAX3543_LOSCALE;
	Num = Flo_scaled * Rdiv * Vdiv * MAX3543_XTALSCALE;

	NdivInt = (UINT_16) ( Num / (UINT_32) Denom );

	/* Calculate whole number remainder from division of Num by denom */
	Rem = Num - Denom * NdivInt;

	/* FracN = Rem * 2^20/Denom, Scale 2^20/Denom 2048 X larger for more accuracy. */
	/* fracscale = 2^31/denom.  2048 = 2^31/2^20  */
	fracscale = 2147483648 / Denom;
	NdivFrac = ( Rem * fracscale ) / 2048;
#elif defined(MAX3543_INT_MODE)
	unsigned long Num;
	unsigned long Denom;
	unsigned long NdivFracScaled;
	unsigned long Flo_kHz;

	Flo_kHz = Flo / 1000;
	Denom = ((unsigned long) MAX3543_REF_FREQUENCY / 1000) / Rdiv * 4;

	NdivInt = (Flo_kHz * Vdiv) / Denom;
	Num = Flo_kHz * Vdiv * MAX3543_FRAC_SCALE;

	NdivFracScaled = (Num / Denom - NdivInt * MAX3543_FRAC_SCALE) * MAX3543_2POW20; // MAX3543_2POW20 = 2^20
	NdivFrac = NdivFracScaled / MAX3543_FRAC_SCALE;

#else
	/*
	 * N divider Calculation using floats.
	 * We work with Flo and Fref in kHz to avoid
	 * overflow with Flo * Vdiv.
	 * (example @ Frf = 98.25 MHz: Flo * Vdiv = 134.25M * 32 > 2^32-1)
	 */

	float Ndiv;
	UINT_16 fscale = 1000;

	Ndiv = ( (float)Flo / fscale ) * Vdiv / ( (float) MAX3543_REF_FREQUENCY / fscale / Rdiv * 4 );
	NdivInt = byte(Ndiv);
	NdivFrac = (Ndiv - NdivInt) * MAX3543_2POW20;
#endif

	/*
	 * TFS and TFP calculation.
	 *
	 * Adapted from Maxim's driver code.
	 *
	 *  Calculate the series and parallel capacitor values for the given frequency
	 *  band.  These values are then written to the registers.  This causes the
	 *  MAX3543's internal series and parallel capacitors to change thus tuning the
	 *  tracking filter to the proper frequency.
	 */

	/*  Set the tfb Bits (Tracking Filter Band) for the given frequency. */
	if (Frf < 196 * MAX3543_MHZ)			/* VHF Low Band */
        tfb = MAX3543_VHF_L;
	else if (Frf < 440 * MAX3543_MHZ)		/* VHF High  196-440 MHz */
        tfb = MAX3543_VHF_H;
	else							/* UHF */
        tfb = MAX3543_UHF;

	tfs = tfs_i(tfRomCoefs[tfb][MAX3543_SER0], tfRomCoefs[tfb][MAX3543_SER1],  Frf, MAX3543_TF_COEFS[tfb*2]);
	tfp = tfs_i(tfRomCoefs[tfb][MAX3543_PAR0], tfRomCoefs[tfb][MAX3543_PAR1],  Frf, MAX3543_TF_COEFS[(tfb*2)+1]);
	if (tfp > 63)   /* 63 = 6 bits of TFP  */
		tfp = 63;

	res[0x00] = (res[0x00] & 0xf0) | VdivBin;
	res[0x01] = NdivInt;
	res[0x02] = 0x80 | (res[0x02] & 0x40) | ((Rdiv-1) << 4) | (NdivFrac >> 16);
	res[0x03] = (NdivFrac >> 8) & 0xff;
	res[0x04] = NdivFrac & 0xff;

	/* R05: MODE CTRL */
	res[0x05] = 0x00;
	res[0x05] |= MAX3543_LNA2G << 7;
	res[0x05] |= (Frf > 345 * MAX3543_MHZ) << 6;
	res[0x05] |= (Frf < 110 * MAX3543_MHZ) << 5;
	res[0x05] |= MAX3543_BW << 4;
	res[0x05] |= tfb << 2;
	res[0x05] |= MAX3543_IFSEL;

	/* R06 and 07: tfs/tfp */

	res[0x06] = tfs;
	res[0x07] = tfp & 0x3f;

	return MAX3543_SUCCESS;
}

byte MAX3543::tfs_i(byte S0, byte S1, UINT_32 Frf, const byte c[5]) {
	UINT_32 i, y, add;
	UINT_32 res;

	UINT_16 Frf_MHz = Frf / MAX3543_MHZ;

	y = (UINT_32) 4 * ((UINT_32) (64 * c[0]) + (UINT_32) (c[1] * S0))
			- ((UINT_32) (64 * c[2]) - (UINT_32) (S1 * c[3])) * Frf_MHz / 250;

	y = (10 * y) / 111; /* approximation for nom*10*LN(10)/256 */

	add = y;
	res = 100 + y;
	for (i = 2; i < 12; i++) {
		add = (add * y) / (i * 100); /* this only works with 32bit math */
		res += add;

	}

   if (((UINT_32) res + 50 * 1) > ((UINT_32) 100 * c[4]))
		res = (res + 50 * 1) / 100 - c[4];
	else
		res = 0;

	if (res < 255)
		return (byte) res;
	else
		return 255;
}

byte MAX3543::readRegister(byte start) {
	return readRegisters(start, start);
}

byte MAX3543::readRegisters() {
	return readRegisters(0x00, 0x15);
}



byte MAX3543::readRegisters(byte start, byte stop) {
	if (start < 0x00 || stop > 0x15 || stop < start) return MAX3543_ERROR;

	Wire.beginTransmission(MAX3543_I2C_ADDRESS);
	Wire.write(start);
//	if ( Wire.endTransmission() != 0 ) return MAX3543_ERROR;
	Wire.requestFrom((uint8_t) MAX3543_I2C_ADDRESS, (uint8_t) (stop-start+1));
	for (byte i = start; i<= stop; i++) {
		if ( Wire.available() ) {
			registers[i] = Wire.read();
		}
		else {
			Wire.endTransmission();
			return MAX3543_ERROR;
		}
	}
	if ( Wire.endTransmission() != 0 ) return MAX3543_ERROR;
	return MAX3543_SUCCESS;
}

byte MAX3543::readROM() {

	byte rom_data[13];
	for (byte i = 0x0 ; i <= 0xC ; i++) {
		if ( writeRegister(0x0E,i) != MAX3543_SUCCESS ) return MAX3543_ERROR;
		if ( readRegister(0x10) != MAX3543_SUCCESS ) return MAX3543_ERROR;
		rom_data[i] = registers[0x10];
	}

	/* assemble the broken up word pairs from the ROM table  into complete ROM coefficients:  */
	tfRomCoefs[MAX3543_VHF_L][MAX3543_SER0] = (rom_data[1] & 0xFC) >> 2;  /*'LS0 )*/
	tfRomCoefs[MAX3543_VHF_L][MAX3543_SER1] = ((rom_data[1] & 0x3 ) << 4) + ((rom_data[2] & 0xf0) >> 4);  /* 'LS1*/
	tfRomCoefs[MAX3543_VHF_L][MAX3543_PAR0] = ((rom_data[2] & 0xf) << 2) + ((rom_data[3] & 0xc0) >> 6);  /*'LP0*/
	tfRomCoefs[MAX3543_VHF_L][MAX3543_PAR1] = rom_data[3] & 0x3f;  /*LP1 */

	tfRomCoefs[MAX3543_VHF_H][MAX3543_SER0] = ((rom_data[4] & 0xfc) >> 2);  /*'HS0 */
	tfRomCoefs[MAX3543_VHF_H][MAX3543_SER1] = ((rom_data[4] & 0x3) << 4) + ((rom_data[5] & 0xF0) >> 4);  /*'HS1 */
	tfRomCoefs[MAX3543_VHF_H][MAX3543_PAR0] = ((rom_data[5] & 0xf) << 2) + ((rom_data[6] & 0xc0) >> 6);  /*'HP0 */
	tfRomCoefs[MAX3543_VHF_H][MAX3543_PAR1] = rom_data[6] & 0x3F;  /*'HP1 */

	tfRomCoefs[MAX3543_UHF][MAX3543_SER0] =  ((rom_data[7]  & 0xFC) >> 2);  /*'US0 */
	tfRomCoefs[MAX3543_UHF][MAX3543_SER1] = ((rom_data[7] & 0x3) << 4) + ((rom_data[8] & 0xf0) >> 4 );  /*'US1 */
	tfRomCoefs[MAX3543_UHF][MAX3543_PAR0] = ((rom_data[8] & 0xF) << 2) + ((rom_data[9] & 0xc0) >> 6);  /*'UP0 */
	tfRomCoefs[MAX3543_UHF][MAX3543_PAR1] = rom_data[9] & 0x3f;  /*'UP1 */

	registers[0x0D] = rom_data[0xA] & 0x3f;
	registers[0x0E] = 0;
	registers[0x0F] = rom_data[0xB];

	return  writeRegisters(0x0D, 0x0F);
}

byte MAX3543::writeRegisters(byte start) {
	return writeRegisters(start, start);
}

byte MAX3543::writeRegisters(byte start, byte stop) {

	if (start < 0x00 || stop > 0x15 || stop < start) return MAX3543_ERROR;

	Wire.beginTransmission(MAX3543_I2C_ADDRESS);
	Wire.write(start);
	for (byte i = start ; i <= stop ; i++) {
		Wire.write(registers[i]);
	}

	if (Wire.endTransmission() != 0) return MAX3543_ERROR;

	return MAX3543_SUCCESS;
}

