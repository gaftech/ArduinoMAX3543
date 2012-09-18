/*
 * MAX3543_Tests.ino
 *
 * Copyright (c) 2012, Gabriel Fournier <gabriel@gaftech.fr>
 *
 * This file is part of ArduinoMAX3543 project.
 * Pleased read attached LICENSE file. 
 *
 *  Created on: 1 juil. 2012
 *      Author: Gabriel Fournier
 */

#include <MAX3543.h>
#include <TestHelpers.h>

#include <Wire.h>
#include <Arduino.h>

MAX3543 tuner;



/*
 * Compare register values @ 107.7 MHz with the ones we got with maxim software
 * and our default params.
 *
 * TODO: Same thing with other frequencies.
 */
void testTuneRegisterCalculation() {
	Serial.println("MAX3543 frequency calculation tests...");

	byte regs[8];
	tuner.computeTuneRegisters(regs, 107700000);
	assertEqual("R00 @ 107.7 MHz", regs[0x00] & 0x3, 0x2);
	assertEqual("R01 @ 107.7 MHz", regs[0x01], 0x23 );
	assertEqual("R02 @ 107.7 MHz", regs[0x02], 0x8E );
	assertEqual("R03 @ 107.7 MHz", regs[0x03], 0xCC );
	assertEqual("R04 @ 107.7 MHz", regs[0x04], 0xCC );

	//TODO: R05 depends on frequency independent params, so this test should be adapted (or removed ;-)
	assertEqual("R05 @ 107.7 MHz", regs[0x05], 0xA0 );
	// TFS/TFP: I didn't exactly understand the way it should be calculated, so I use the algorithm
	// provided by Maxim in their driver code, but the results differs a little (1 @ 107.7) from
	// the one given by the evaluation board software. Ideally, I would like to get and understand
	// the equation to get the results and try to implement it using floats and then compare
	// results and speed (in fact just for fun, it would differ much I guess...).
	assertAlmostEqual("R06 @ 107.7 MHz", regs[0x06], 0x86, 1 );
	assertAlmostEqual("R07 @ 107.7 MHz", regs[0x07], 0x14, 1 );
}

/*
 * Tune device @ 107.7 MHz then read registers (forces i2c read, does not rely on cache).
 * Compare results with known values from Maxim's evaluation software.
 */
void testTune() {

	bool success;
	byte regs[20];

	success = tuner.tune(107700000) == MAX3543_SUCCESS;
	ATS_PrintTestStatus("Tune success", success);

	success = tuner.readRegisters() == MAX3543_SUCCESS;
	ATS_PrintTestStatus("Read success", success);

	tuner.getRegisters(regs, 0x00, 0x14);

	assertEqual( "R00 @ 107.7 MHz", regs[0x00] & 0x3, 0x2 );
	assertEqual( "R01 @ 107.7 MHz", regs[0x01], 0x23 );
	assertEqual( "R02 @ 107.7 MHz", regs[0x02], 0x8E );
	assertEqual( "R03 @ 107.7 MHz", regs[0x03], 0xCC );
	assertEqual( "R04 @ 107.7 MHz", regs[0x04], 0xCC );

	//TODO: R05 depends on frequency independent params, so this test should be adapted (or removed ;-)
	assertEqual( "R05 @ 107.7 MHz", regs[0x05], 0xA0 );
	// TFS/TFP: I didn't exactly understand the way it should be calculated, so I use the algorithm
	// provided by Maxim in their driver code, but the results differs a little (1 @ 107.7) from
	// the one given by the evaluation board software. Ideally, I would like to get and understand
	// the equation to get the results and try to implement it using floats and then compare
	// results and speed (in fact just for fun, it would differ much I guess...).
	assertAlmostEqual( "R06 @ 107.7 MHz", regs[0x06], 0x86, 1 );
	assertAlmostEqual( "R07 @ 107.7 MHz", regs[0x07], 0x14, 1 );
	assertEqual( "R08 @ 107.7 MHz", regs[0x08], 0x00 );
	assertEqual( "R09 @ 107.7 MHz", regs[0x09], 0x0A);
	assertEqual( "R0A @ 107.7 MHz", regs[0x0A], 0x16);
	assertEqual( "R0B @ 107.7 MHz", regs[0x0B], 0x43);
	assertEqual( "R0C @ 107.7 MHz", regs[0x0C], 0x01);
	assertEqual( "R0D @ 107.7 MHz", regs[0x0D], 0x24);
	assertEqual( "R0E @ 107.7 MHz", regs[0x0E], 0x00);
	assertEqual( "R0F @ 107.7 MHz", regs[0x0F], 0x80);
	// skiping R10 to R12 (ROM readback and status indicators)
	assertEqual( "R13 @ 107.7 MHz", regs[0x13], 0x56);
	assertEqual( "R14 @ 107.7 MHz", regs[0x14], 0x40);
}
/*
 * Performs a write registers/read registers for a given frequency (in Hz)
 * and compare this given frequency with the one computed from register values.
 *
 * Note: it's certainly a not enough rigorous approach to evaluate tuning
 * calculation accuracy. Maybe we should code some script, not on arduino,
 * to make exact calculation (using 64 bits integer) and compare it with
 * results of our float or 32 bits integer methods. Hmmm, I guess
 * also that with some math skills, I could have a better approach.
 */
long getFrequencyError(unsigned long freq) {
	byte regs[8];
	unsigned long res;

	tuner.computeTuneRegisters(regs, freq);
	res = tuner.computeFrequency(regs);

	if (freq > res)
		return freq - res;
	else
		return res - freq;
}

void testFrequencyError(unsigned long start = 47000000, unsigned long stop =
		862000000, unsigned long step = 50000, unsigned long delta = 1000,
		bool breakOnFirstError=1) {

	char startStr[13];
	char stopStr[13];
	char stepStr[13];
	dtostrf((float) start / MAX3543_MHZ, 0, 3, startStr);
	dtostrf((float) stop / MAX3543_MHZ, 0, 3, stopStr);
	dtostrf((float) step / 1000, 0, 3, stepStr);
	debugPrintf(
			"Testing frequency error between %s and %s MHz / step = %s kHz...",
			startStr, stopStr, stepStr);

	bool success = 1;
	long err = 0;
	long maxError = 0;
	long maxErrorFreq = 0;
	unsigned long freq;
	for (freq = start; freq <= stop; freq += step) {
		err = getFrequencyError(freq);

		if (abs(err) > maxError) {
			maxError = err;
			maxErrorFreq = freq;
		}

		if (abs(err) > abs(delta)) {
			success = 0;
			if (breakOnFirstError)
				break;
		}
	}

	renderTestResult(success, "Frequency calculation error");

	char freqStr[13];
	dtostrf((float) maxErrorFreq / MAX3543_MHZ, 0, 3, freqStr);
	debugPrintf("max delta @ %s MHz: %d Hz", freqStr, maxError);


}

void setup() {
	Serial.begin(9600);
	ATS_begin("Gaftech", "MAX3543 tests");

	bool initSuccess = tuner.init() == MAX3543_SUCCESS;
	renderTestResult(initSuccess, "device init");

	if ( ! initSuccess ) {
		Serial.println("ERROR: MAX3543 init failed"); // No need to go further
	}
	else {
		testTuneRegisterCalculation();
		testTune();
		testFrequencyError();
	}
	ATS_end();
}

void loop() {

}
