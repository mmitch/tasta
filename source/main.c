/*
 * tasta - simple USB keyboard for ATtiny85
 * Copyright (C) 2015 Christian Garbs <mitch@cgarbs.de>
 *
 * based upon HIDKeys example from V-USB/obdev:
 *
 *
 * Name: main.c
 * Project: HID-Test
 * Author: Christian Starkjohann
 * Creation Date: 2006-02-02
 * Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2
 * Homepage: http://www.obdev.at/vusb/
 * Source: https://www.obdev.at/downloads/vusb/HIDKeys.2012-12-08.tar.gz
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "usbdrv.h"

/* ----------------------- hardware I/O abstraction ------------------------ */

/* pin assignments:
PB0	LED (input, sink, low=on)
PB1     USB data -
PB2     USB data +
PB3     button 2
PB4     button 1
PB5     reset (in hardware; unused here)
*/

#define BUTTON_PORT     PORTB       /* PORTx - register for buttons */
#define BUTTON_PIN      PINB        /* PINx  - register for buttons */
#define LED_DDR         DDRB        /* DDRx  - register for LED */

#define BUTTON1_BIT     PB4         /* bit for button 1 in button register */
#define BUTTON2_BIT     PB3         /* bit for button 2 in button register */
#define LED_BIT         PB0         /* bit for LED in LED register */


/* some bitbanging magic */

#define LED_ON      (LED_PORT &= ~_BV(LED_BIT))
#define LED_OFF     (LED_PORT |=  _BV(LED_BIT))

#define GET_BIT(pin,bit) (pin & _BV(bit))


static void hardwareInit(void)
{
	uchar i;
	uchar   calibrationValue;

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
	if (GET_BIT(BUTTON_PIN, BUTTON1_BIT))
	{
		LED_ON;
		return 1;
	}
	if (GET_BIT(BUTTON_PIN, BUTTON2_BIT))
	{
		LED_ON;
		return 2;
	}
	LED_OFF;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */

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
	0x95, 0x01,                    //   REPORT_COUNT (1)
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

static const uchar keyReport[NUM_KEYS + 1][2] PROGMEM = {
	/* none */  {0, 0},                     /* no key pressed */
	/*  1 */    {MOD_GUI_LEFT,  0},         /* left windows key (modifier only) */
	/*  2 */    {MOD_GUI_RIGHT, 0},         /* right windows key (modifier only) */
};

static void buildReport(uchar key)
{
/* This (not so elegant) cast saves us 10 bytes of program memory */
	*(int *)reportBuffer = pgm_read_word(keyReport[key]);
}

uchar usbFunctionSetup(uchar data[8])
{
	usbRequest_t    *rq = (void *)data;

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
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
	uchar step = 128;
	uchar trialValue = 0, optimumValue;
	int   x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

	/* do a binary search: */
	do
	{
		OSCCAL = trialValue + step;
		x = usbMeasureFrameLength(); /* proportional to current real frequency */
		if (x < targetValue)         /* frequency still too low */
		{
			trialValue += step;
		}
		step >>= 1;
	}
	while (step > 0);
	/* We have a precision of +/- 1 for optimum OSCCAL here */
	/* now do a neighborhood search for optimum value */
	optimumValue = trialValue;
	optimumDev = x; /* this is certainly far away from optimum */
	for (OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++)
	{
		x = usbMeasureFrameLength() - targetValue;
		if (x < 0)
		{
			x = -x;
		}
		if(x < optimumDev)
		{
			optimumDev = x;
			optimumValue = OSCCAL;
		}
	}
	OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

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

	wdt_enable(WDTO_2S);
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
				if (idleCounter >= OVERFLOW_4MS)
				{
					idleCounter -= OVERFLOW_4MS;
				}
				else
				{
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

		}
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
