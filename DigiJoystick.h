/*
 * Based on Obdev's AVRUSB code and under the same license.
 *
 * TODO: Make a proper file header. :-)
 * Modified for Digispark by Digistump
 * And now modified by Sean Murphy (duckythescientist) from a keyboard device to a joystick device
 * And now modified by Bluebie to have better code style, not ruin system timers, and have delay() function
 * Most of the credit for the joystick code should go to Rapha�l Ass�nat
 */
#ifndef __DigiJoystick_h__
#define __DigiJoystick_h__
 
#define GCN64_REPORT_SIZE 8 //0x20  

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <string.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "usbdrv.h"
//#include "devdesc.h"
#include "oddebug.h"
#include "usbconfig.h"

#define ONBOARD_LED	1 
 
static uchar *rt_usbHidReportDescriptor=NULL;
static uchar rt_usbHidReportDescriptorSize=0;
static uchar *rt_usbDeviceDescriptor=NULL;
static uchar rt_usbDeviceDescriptorSize=0;

// TODO: Work around Arduino 12 issues better.
//#include <WConstants.h>
//#undef int()

//typedef uint8_t byte;

/* What was most recently read from the controller */
unsigned char last_built_report[GCN64_REPORT_SIZE];

/* What was most recently sent to the host */
unsigned char last_sent_report[GCN64_REPORT_SIZE];

uchar		 reportBuffer[GCN64_REPORT_SIZE];

// report frequency set to default of 50hz
#define DIGIJOYSTICK_DEFAULT_REPORT_INTERVAL 20
static unsigned char must_report = 0;
static unsigned char idle_rate = DIGIJOYSTICK_DEFAULT_REPORT_INTERVAL / 4; // in units of 4ms
// new minimum report frequency system:
static unsigned long last_report_time = 0;

/* ERV
typedef struct
{
	uint8_t startByte; //Always 0x00
	uint8_t bLength;
	uint8_t dButtons;	// D-Pad = 0 or 1 bitwise - DU, DN, DL, DR, TL, TR, start, back
	uint8_t reserved;
	uint8_t A; 				// A button from 0 - 255 depending on pressure
	uint8_t B;				// B button from 0 - 255 depending on pressure
	uint8_t X;				// X button from 0 - 255 depending on pressure
	uint8_t Y;				// Y button from 0 - 255 depending on pressure
	uint8_t BLACK;			// Black button from 0 - 255 depending on pressure
	uint8_t WHITE;			// White button from 0 - 255 depending on pressure
	uint8_t L;				// Left trigger from 0 - 255 depending on pressure
	uint8_t R;				// Right trigger from 0 - 255 depending on pressure
	int16_t leftStickX; 	// Left Stick X from 0 - 32768
	int16_t leftStickY; 	// Left Stick X from 0 - 32768
	int16_t rightStickX;	// Left Stick X from 0 - 32768
	int16_t rightStickY;	// Left Stick X from 0 - 32768
	//These last few values aren't part of the xbox controller HID report, but are added here by me to store extra stuff.
	uint8_t left_actuator;
	uint8_t right_actuator;
	uint8_t rumbleUpdate;
} USB_XboxGamepad_Data_t;
*/

#if 0 
// Xbox Controller configuration descriptor
static unsigned char gcn64_usbHidConfigurationDescriptor[] = {
	
	//Configuration Descriptor//
	0x09,			//bLength of config descriptor
	0x02,			//bDescriptorType, 2=Configuration Descriptor
	0x20, 0x00,		//wTotalLength 2-bytes, total length (including interface and endpoint descriptors)
	0x01,			//bNumInterfaces, just 1
	0x01,			//bConfigurationValue
	0x00,			//iConfiguration - index to string descriptors. we dont use them
	0x80,			//bmAttributes - 0x80 = USB Bus Powered
	0xFA,			//bMaxPower - maximum power in 2mA units. 0xFA=500mA. Genuine OG controller is normally 100mA (0x32)
	
	//Interface Descriptor//
	0x09, //bLength of interface descriptor
	0x04, //bDescriptorType, 4=Interface  Descriptor
	0x00, //bInterfaceNumber
	0x00, //bAlternateSetting
	0x02, //bNumEndpoints - we have two endpoints (IN for button presses, and OUT for rumble values)
	0x58, //bInterfaceClass - From OG Xbox controller
	0x42, //bInterfaceSubClass - From OG Xbox controller
	0x00, //bInterfaceProtocol
	0x00, //iInterface - index to string descriptors. we dont use them

	//Endpoint Descriptor (IN)//
	0x07, //bLength of endpoint descriptor
	0x05, //bDescriptorType, 5=Endpoint Descriptor
	0x81, //bEndpointAddress, Address=1, Direction IN
	0x03, //bmAttributes, 3=Interrupt Endpoint
	0x20, 0x00, //wMaxPacketSize
	0x04, //bInterval, Interval for polling the interrupt endpoint. 4ms
	
	//Endpoint Descriptor (OUT)//
	0x07, //bLength of endpoint descriptor
	0x05, //bDescriptorType, 5=Endpoint Descriptor
	0x02, //bEndpointAddress, Address=2, Direction OUT
	0x03, //bmAttributes, 3=Interrupt Endpoint
	0x20, 0x00, //wMaxPacketSize
	0x04 //bInterval, Interval for polling the interrupt endpoint. 4ms
};
#endif

// back up of original descriptor
const unsigned char gcn64_usbHidReportDescriptor[] PROGMEM = {
		0x05, 0x01,					// USAGE_PAGE (Generic Desktop) 1 = Device Descriptor
		0x09, 0x05,					// USAGE (Gamepad)
		0xa1, 0x01,					// COLLECTION (Application)
	
		0x09, 0x01,					//		USAGE (Pointer)		 
		0xa1, 0x00,					//		COLLECTION (Physical)
		0x05, 0x01,					//		USAGE_PAGE (Generic Desktop)
		0x09, 0x30,	   			 	//		USAGE (X)
		0x09, 0x31,					//		USAGE (Y)
	
		0x09, 0x33,					//		USAGE (Rx)
		0x09, 0x34,					//		USAGE (Ry)

		0x09, 0x35,					//		USAGE (Rz)	
		0x09, 0x36,					//		USAGE (Slider)	

		0x15, 0x00,					//		LOGICAL_MINIMUM (0)
		0x26, 0xFF, 0x00,			//		LOGICAL_MAXIMUM (255)
		0x75, 0x08,					//		REPORT_SIZE (8)
		0x95, 0x06,					//		REPORT_COUNT (6)
		0x81, 0x02,					//		INPUT (Data,Var,Abs)
		0xc0,						//		END_COLLECTION (Physical)

		0x05, 0x09,					//		USAGE_PAGE (Button)
		0x19, 0x01,					//		USAGE_MINIMUM (Button 1)
		0x29, 0x10,					//		USAGE_MAXIMUM (Button 14)
		0x15, 0x00,					//		LOGICAL_MINIMUM (0)
		0x25, 0x01,					//		LOGICAL_MAXIMUM (1)
		0x75, 0x01,					//		REPORT_SIZE (1)
		0x95, 0x10,					//		REPORT_COUNT (16)
		0x81, 0x02,					//		INPUT (Data,Var,Abs)

		0xc0													 // END_COLLECTION (Application)
};

#define USBDESCR_DEVICE				1

// back of the original descriptor
const unsigned char usbDescrDevice[] PROGMEM = {		/* USB device descriptor */
		18,					/* sizeof(usbDescrDevice): length of descriptor in bytes */
		USBDESCR_DEVICE,		/* descriptor type */
		0x01, 0x01, /* USB version supported */
		USB_CFG_DEVICE_CLASS,
		USB_CFG_DEVICE_SUBCLASS,
		0,					/* protocol */
		8,					/* max packet size */
		USB_CFG_VENDOR_ID,	/* 2 bytes */
		USB_CFG_DEVICE_ID,	/* 2 bytes */
		USB_CFG_DEVICE_VERSION, /* 2 bytes */
#if USB_CFG_VENDOR_NAME_LEN
		1,					/* manufacturer string index */
#else
		0,					/* manufacturer string index */
#endif
#if USB_CFG_DEVICE_NAME_LEN
		2,					/* product string index */
#else
		0,					/* product string index */
#endif
#if USB_CFG_SERIAL_NUMBER_LENGTH
		3,					/* serial number string index */
#else
		0,					/* serial number string index */
#endif
		1,					/* number of configurations */
};


#if 0
// HID class driver callback function for the creation of HID reports to the host.
bool CreateHIDReport(void* ReportData){

	USB_XboxGamepad_Data_t* DukeReport = (USB_XboxGamepad_Data_t*)ReportData;

	DukeReport->startByte = 0x00;
	DukeReport->bLength = 20;
	DukeReport->dButtons = 0; //XboxOGDuke[0].dButtons;
	DukeReport->reserved = 0x00;
	DukeReport->A = 0; //XboxOGDuke[0].A;
	DukeReport->B = 0 && 0x02; //XboxOGDuke[0].B;
	DukeReport->X = last_built_report[0]; //XboxOGDuke[0].X;
	DukeReport->Y = last_built_report[1]; //XboxOGDuke[0].Y;
	DukeReport->BLACK = 0; //XboxOGDuke[0].BLACK;
	DukeReport->WHITE = 0; //XboxOGDuke[0].WHITE;
	DukeReport->L = 0; //XboxOGDuke[0].L;
	DukeReport->R = 0; //XboxOGDuke[0].R;
	DukeReport->leftStickX = 0; //XboxOGDuke[0].leftStickX;
	DukeReport->leftStickY = 0; //XboxOGDuke[0].leftStickY;
	DukeReport->rightStickX = 0; // XboxOGDuke[0].rightStickX;
	DukeReport->rightStickY = 0; //XboxOGDuke[0].rightStickY;
	
	return false;
}
#endif

void gamecubeBuildReport(unsigned char *reportBuf) {
	if (reportBuf != NULL) {
		//CreateHIDReport(last_built_report);
		memcpy(reportBuf, last_built_report, GCN64_REPORT_SIZE);
	}
	
	memcpy(last_sent_report, last_built_report, GCN64_REPORT_SIZE); 
}

int getGamepadReport(unsigned char *dstbuf) {
	gamecubeBuildReport(dstbuf);
	return GCN64_REPORT_SIZE;
}

 
class DigiJoystickDevice {
 public:
	DigiJoystickDevice () {

		rt_usbHidReportDescriptor = (uchar *)gcn64_usbHidReportDescriptor;
		rt_usbHidReportDescriptorSize = sizeof(gcn64_usbHidReportDescriptor);

		rt_usbDeviceDescriptor = (uchar *)usbDescrDevice;
		rt_usbDeviceDescriptorSize = sizeof(usbDescrDevice);

		// Initialize the on board LED
		 pinMode(ONBOARD_LED,OUTPUT);
    	 digitalWrite(ONBOARD_LED, LOW);

		cli();
		usbDeviceDisconnect();
		_delay_ms(250);
		usbDeviceConnect();
		
		usbInit();
		
		sei();
		
		last_report_time = millis();
	}
	
	void update() {
		usbPoll();
		
		// instead of above code, use millis arduino system to enforce minimum reporting frequency
		unsigned long time_since_last_report = millis() - last_report_time;
		if (time_since_last_report >= (idle_rate * 4 /* in units of 4ms - usb spec stuff */)) {
			last_report_time += idle_rate * 4;
			must_report = 1;
		}
		
		// if the report has changed, try force an update anyway
		if (memcmp(last_built_report, last_sent_report, GCN64_REPORT_SIZE)) {
			must_report = 1;
		}
	
		// if we want to send a report, signal the host computer to ask us for it with a usb 'interrupt'
		if (must_report) {
			if (usbInterruptIsReady()) {
				must_report = 0;
				
				gamecubeBuildReport(reportBuffer);
				usbSetInterrupt(reportBuffer, GCN64_REPORT_SIZE);
			}
		}
	}
	
	// delay while updating until we are finished delaying
	void delay(long milli) {
		unsigned long last = millis();
	  while (milli > 0) {
	    unsigned long now = millis();
	    milli -= now - last;
	    last = now;
	    update();
	  }
	}
	
	void setX(byte value) {
		last_built_report[0] = value;
	}
	
	void setY(byte value) {
		last_built_report[1] = value;
	}
	
	void setXROT(byte value) {
		last_built_report[2] = value;
	}
	
	void setYROT(byte value) {
		last_built_report[3] = value;
	}
	
	void setZROT(byte value) {
		last_built_report[4] = value;
	}
	
	void setSLIDER(byte value) {
		last_built_report[5] = value;
	}
	
	void setX(char value) {
		setX(*(reinterpret_cast<byte *>(&value)));
	}
	
	void setY(char value) {
		setY(*(reinterpret_cast<byte *>(&value)));
	}
	
	void setXROT(char value) {
		setXROT(*(reinterpret_cast<byte *>(&value)));
	}
	
	void setYROT(char value) {
		setYROT(*(reinterpret_cast<byte *>(&value)));
	}
	
	void setZROT(char value) {
		setZROT(*(reinterpret_cast<byte *>(&value)));
	}
	void setSLIDER(char value) {
		setSLIDER(*(reinterpret_cast<byte *>(&value)));
	}
	
	void setButtons(unsigned char low, unsigned char high) {
		last_built_report[6] = low;
		last_built_report[7] = high;
	}
	
	void setButtons(char low,char high) {
		setButtons(*reinterpret_cast<unsigned char *>(&low),*reinterpret_cast<unsigned char *>(&high));
	}
	
	void setValues(unsigned char values[]) {
		memcpy(last_built_report, values, GCN64_REPORT_SIZE);
	}
	
	void setValues(char values[]) {
		unsigned char *foo = reinterpret_cast<unsigned char *>(values);//preserves bit values in cast
		memcpy(last_built_report, foo, GCN64_REPORT_SIZE);
	}
};

// Create global singleton object for users to make use of
DigiJoystickDevice DigiJoystick = DigiJoystickDevice();

#ifdef __cplusplus
extern "C"{
#endif 
	// USB_PUBLIC uchar usbFunctionSetup
	
	uchar usbFunctionSetup(uchar data[8]) {
		usbRequest_t *rq = (usbRequest_t *)data;

		usbMsgPtr = reportBuffer;
		if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) { // class request type
			if (rq->bRequest == USBRQ_HID_GET_REPORT){ // wValue: ReportType (highbyte), ReportID (lowbyte)
				// we only have one report type, so don't look at wValue
				//curGamepad->buildReport(reportBuffer);
				//return curGamepad->report_size;
				return GCN64_REPORT_SIZE;
			} else if (rq->bRequest == USBRQ_HID_GET_IDLE) {
				usbMsgPtr = &idle_rate;
				return 1;
			} else if (rq->bRequest == USBRQ_HID_SET_IDLE) {
				idle_rate = rq->wValue.bytes[1];
			}
		} else {
			/* no vendor specific requests implemented */
		}
		return 0;
	}

	uchar usbFunctionDescriptor(struct usbRequest *rq) {
		if ((rq->bmRequestType & USBRQ_TYPE_MASK) != USBRQ_TYPE_STANDARD) {
			return 0;
		}


		if (rq->bRequest == USBRQ_GET_DESCRIPTOR) {
			// USB spec 9.4.3, high byte is descriptor type
			switch (rq->wValue.bytes[1]) {
				case USBDESCR_DEVICE:	
					digitalWrite(ONBOARD_LED, HIGH);			
					delay(100);
					digitalWrite(ONBOARD_LED, LOW);		
					usbMsgPtr = rt_usbDeviceDescriptor;
					return rt_usbDeviceDescriptorSize;
					break;
					
				case USBDESCR_HID_REPORT:
					usbMsgPtr = rt_usbHidReportDescriptor;
					return  rt_usbHidReportDescriptorSize;
					break;				
			}
		}
		
		return 0;
	}
	
#ifdef __cplusplus
} // extern "C"
#endif


#endif // __DigiKeyboard_h__
