/*
 * tasta - simple USB keyboard for ATtiny85
 * Copyright (C) 2015 Christian Garbs <mitch@cgarbs.de>
 * Licensed under GNU GPL v2 or v3
 *
 * based upon HIDKeys example from V-USB/obdev:
 *
 *
 * Name: main.c
 * Project: HID-Test
 * Author: Christian Starkjohann
 * Creation Date: 2006-02-02
 * Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2  (or GNU GPL v3 as the complete License.txt says)
 * Homepage: http://www.obdev.at/vusb/
 * Source: https://www.obdev.at/downloads/vusb/HIDKeys.2012-12-08.tar.gz
 */

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdlib.h>

#include "usbdrv/usbdrv.h"

/* ----------------------- hardware I/O abstraction ------------------------ */

/* pin assignments:
PB0	LED (output, but acts as a sink: deactivate pullup = LED on)
PB1     USB data -
PB2     USB data +
PB3     button 2
PB4     button 1
PB5     reset (in hardware; unused here)
*/

#define BUTTON_PORT     PORTB       /* PORTx - register for buttons (pullups) */
#define BUTTON_PIN      PINB        /* PINx  - register for buttons (input state) */
#define LED_PORT        PORTB       /* PORTx - register for LED (pullups) */
#define LED_DDR         DDRB        /* DDRx  - register for LED (set to output) */

#define BUTTON1_BIT     PB4         /* bit for button 1 in button register */
#define BUTTON2_BIT     PB3         /* bit for button 2 in button register */
#define LED_BIT         PB0         /* bit for LED in LED register */

#define KEY1            (1 << 0)    /* bitmask for key 1 */
#define KEY2            (1 << 1)    /* bitmask for key 2 */

/* LED is controlled via pull-up: no pullup = acts as sink = LED on */
#define LED_ON      (LED_PORT &= ~_BV(LED_BIT))
#define LED_OFF     (LED_PORT |=  _BV(LED_BIT))

#define GET_BIT(pin,bit) (pin & _BV(bit))


static void hardwareInit(void)
{
	uchar i;
	uchar calibrationValue;

	calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
	if (calibrationValue != 0xff)
	{
		OSCCAL = calibrationValue;
	}

	usbInit();
	usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
	i = 0;
	while(--i){             /* fake USB disconnect for > 250 ms */
		wdt_reset();
		_delay_ms(1);
	}
	usbDeviceConnect();

	wdt_enable(WDTO_1S);

	/* activate pull-ups for the buttons */
	BUTTON_PORT |= _BV(BUTTON1_BIT) | _BV(BUTTON2_BIT);

	/* initialize LED output */
	LED_DDR |= _BV(LED_BIT);
	LED_ON;

	/* select clock: 16.5M/1k -> overflow rate = 16.5M/256k = 62.94 Hz (~11ms) */
	TCCR1 = 0x0b;
}

/* USB IDLE is counted in 4ms - how many are there in one timer overflow (~11ms)? */
#define OVERFLOWS_4MS 3
/* TODO: this is slightly too much, would 2 be better? */

/* ------------------------------------------------------------------------- */

#define NUM_KEYS 2

/* The following function returns an index for the first key pressed. It
 * returns 0 if no key is pressed.
 *
 * TODO: make both keys work independently from each other (don't let button 1
 *       'overshadow' button 2)
 */
static uchar keyPressed(void)
{
	uchar keystate = 0;

	/* 
	 * look out (I _always_ stumble over this):
	 * as the buttons short to GND on closing, the pin value is reversed:
	 * button bit = 0 -> button is pressed
	 * button bit = 1 -> button is not pressed (pull-up active)
	 */
	if (GET_BIT(BUTTON_PIN, BUTTON1_BIT) == 0)
	{
		keystate |= KEY1;
	}
	if (GET_BIT(BUTTON_PIN, BUTTON2_BIT) == 0)
	{
		keystate |= KEY2;
	}



	/*********************************************/
	/* EDIT BELOW FOR YOUR OWN LED CONFIGURATION */

	if (keystate == 0)
	{
		LED_OFF;
	}
	else
	{
		LED_ON;
	}

	/* EDIT ABOVE FOR YOUR OWN LED CONFIGURATION */
	/*********************************************/



	return keystate;
}

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

static uchar reportBuffer[3];    /* buffer for HID reports */
static uchar idleRate;           /* in 4 ms units */

#define KEYS_IN_REPORT 2   /* modifier does not count, only slots for real keys (the REPORT_COUNT of the second INPUT below) */

const PROGMEM char usbHidReportDescriptor[35] = {   /* USB report descriptor */
	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x06,                    // USAGE (Keyboard)
	0xa1, 0x01,                    // COLLECTION (Application)
	0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
	0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
	0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
	0x75, 0x01,                    //   REPORT_SIZE (1)
	0x95, 0x08,                    //   REPORT_COUNT (8)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	0x95, 0x02,                    //   REPORT_COUNT (2)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
	0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
	0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
	0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
	0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* Keyboard usage values, see usb.org's HID-usage-tables document, chapter
 * 10 Keyboard/Keypad Page for more codes.
 */
#define MOD_CONTROL_LEFT    (1<<0)
#define MOD_SHIFT_LEFT      (1<<1)
#define MOD_ALT_LEFT        (1<<2)
#define MOD_GUI_LEFT        (1<<3)  /* "Windows" key */
#define MOD_CONTROL_RIGHT   (1<<4)
#define MOD_SHIFT_RIGHT     (1<<5)
#define MOD_ALT_RIGHT       (1<<6)
#define MOD_GUI_RIGHT       (1<<7)  /* "Windows" key */

#define KEY_ERROR_ROLLOVER  1       /* error condition "too many keys at once pressed" */

#define KEY_A       4
#define KEY_B       5
#define KEY_C       6
#define KEY_D       7
#define KEY_E       8
#define KEY_F       9
#define KEY_G       10
#define KEY_H       11
#define KEY_I       12
#define KEY_J       13
#define KEY_K       14
#define KEY_L       15
#define KEY_M       16
#define KEY_N       17
#define KEY_O       18
#define KEY_P       19
#define KEY_Q       20
#define KEY_R       21
#define KEY_S       22
#define KEY_T       23
#define KEY_U       24
#define KEY_V       25
#define KEY_W       26
#define KEY_X       27
#define KEY_Y       28
#define KEY_Z       29
#define KEY_1       30
#define KEY_2       31
#define KEY_3       32
#define KEY_4       33
#define KEY_5       34
#define KEY_6       35
#define KEY_7       36
#define KEY_8       37
#define KEY_9       38
#define KEY_0       39

#define KEY_ENTER   40

#define KEY_F1      58
#define KEY_F2      59
#define KEY_F3      60
#define KEY_F4      61
#define KEY_F5      62
#define KEY_F6      63
#define KEY_F7      64
#define KEY_F8      65
#define KEY_F9      66
#define KEY_F10     67
#define KEY_F11     68
#define KEY_F12     69

static void buildReport(uchar key)
{
	uchar modifiers = 0;
	uchar keypos = 0;

	/*********************************************/
	/* EDIT BELOW FOR YOUR OWN KEY CONFIGURATION */

	/* Key test examples follow:

	if (key & KEY1)
	{
		// one modifier, no keys
		modifiers |= MOD_GUI_LEFT;
	}

	if (key & KEY2)
	{
		// *two* keys, no modifiers
		reportBuffer[++keypos] = KEY_A;
		reportBuffer[++keypos] = KEY_B;
	}

	*/

	if (key & KEY1)
	{
		// one modifier, no keys
		modifiers |= MOD_GUI_LEFT;
	}

	if (key & KEY2)
	{
		// one key, no modifiers
		reportBuffer[++keypos] = KEY_ENTER;
	}

	/* EDIT ABOVE FOR YOUR OWN KEY CONFIGURATION */
	/*********************************************/



	reportBuffer[0] = modifiers;
	
	if (keypos > KEYS_IN_REPORT)
	{
		/* Keyboard ErrorRollOver condition (too many keys pressed) */
		keypos = 0;
		while (keypos < KEYS_IN_REPORT)
		{
			reportBuffer[++keypos] = KEY_ERROR_ROLLOVER;
		}
	}
	else
	{
		/* fill remaining/unused keys in report with zero */
		while (keypos < KEYS_IN_REPORT)
		{
			reportBuffer[++keypos] = 0;
		}
	}
}

uchar usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (void *)data;

	usbMsgPtr = reportBuffer;
	if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) /* class request type */
	{
		if (rq->bRequest == USBRQ_HID_GET_REPORT) /* wValue: ReportType (highbyte), ReportID (lowbyte) */
		{
			/* we only have one report type, so don't look at wValue */
			buildReport(keyPressed());
			return sizeof(reportBuffer);
		}
		else if(rq->bRequest == USBRQ_HID_GET_IDLE)
		{
			usbMsgPtr = &idleRate;
			return 1;
		}
		else if(rq->bRequest == USBRQ_HID_SET_IDLE)
		{
			idleRate = rq->wValue.bytes[1];
		}
	}
	else
	{
		/* no vendor specific requests implemented */
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET.
 */
static void calibrateOscillator(void)
{
	int frameLength, targetLength = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
	int bestDeviation = 9999;
	uchar trialCal, bestCal, step, region;

	/* do a binary search in regions 0-127 and 128-255 to get optimum OSCCAL */
	for (region = 0; region <= 1; region++)
	{
		frameLength = 0;
		trialCal = (region == 0) ? 0 : 128;
        
		for (step = 64; step > 0; step >>= 1)
		{
			if (frameLength < targetLength) /* true for initial iteration */
			{
				trialCal += step; /* frequency too low */
			}
			else
			{
				trialCal -= step; /* frequency too high */
			}
                
			OSCCAL = trialCal;
			frameLength = usbMeasureFrameLength();
			
			if (abs(frameLength-targetLength) < bestDeviation)
			{
				bestCal = trialCal; /* new optimum found */
				bestDeviation = abs(frameLength -targetLength);
			}
		}
	}

	OSCCAL = bestCal;
}

void usbEventResetReady(void)
{
	calibrateOscillator();
	eeprom_write_byte(0, OSCCAL); /* store the calibrated value in EEPROM */
}

/* ------------------------------------------------------------------------- */

int main(void)
{
	uchar key, lastKey = 0, keyDidChange = 0;
	uchar idleCounter = 0;

	hardwareInit();
	sei();
	for (;;) /* main event loop */
	{
		wdt_reset();
		usbPoll();
		key = keyPressed();
		if (lastKey != key)
		{
			lastKey = key;
			keyDidChange = 1;
		}
		if (TIFR & (1<<TOV0)) /* ~63 ms timer */
		{
			TIFR = 1<<TOV0; /* clear overflow */
			if (idleRate != 0)
			{
				if (idleCounter >= OVERFLOWS_4MS)
				{
					idleCounter -= OVERFLOWS_4MS;
				}
				else
				{
					/* USB HID poll timer reached
					 * send current state regardless of real key change */
					idleCounter = idleRate;
					keyDidChange = 1;
				}
			}
		}
		if (keyDidChange && usbInterruptIsReady())
		{
			keyDidChange = 0;
			/* use last key and not current key status in order to avoid lost
			   changes in key status. */
			buildReport(lastKey);
			usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
		}
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
