/* ============================================================================
 * PROJECT: TC9560
 * Copyright (C) 2018  Toshiba Electronic Devices & Storage Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ========================================================================= */

/*! History:   
 *		18 July 2016 : 
 */

#ifndef __DWC_OTG_PCD_LINUX_H__
#define __DWC_OTG_PCD_LINUX_H__
#include "tc9560_common.h"
#include "neu_os.h"

#define ETIMEDOUT						145		/* Connection timed out */
#define ECONNRESET						131		/* Connection reset by peer */
#define ENOTSUPP						524		/* Operation is not supported */
#define ENOMEM							12		/* Out of Memory */
#define IRQF_SHARED						0x00000080
#define IRQF_DISABLED					0x00000020
#define ENODEV							19		/* No such device */
#define EAGAIN							35		/* Try again */
#define ERANGE							34		/* Math result not representable */
#define USB_DT_ENDPOINT 				0x05
#define EBUSY							16		/* Device or resource busy */				
/* There is only one configuration. */
#define	CONFIG_VALUE					1
#define USB_DT_CONFIG_SIZE				9
#define USB_DT_INTERFACE_SIZE			9
#define USB_DT_ENDPOINT_SIZE			7
#define USB_ENDPOINT_XFER_BULK			2
/* Big enough to hold our biggest descriptor */
#define EP0_BUFSIZE						256
/* these descriptors are modified based on what controller we find */
#define	STRINGID_MFGR					1
#define	STRINGID_PRODUCT				2
#define	STRINGID_SERIAL					3
#define	STRINGID_CONFIG					4
#define	STRINGID_INTERFACE				5
/*
 * USB directions
 *
 * This bit flag is used in endpoint descriptors' bEndpointAddress field.
 * It's also one of three fields in control requests bRequestType.
 */
#define USB_DIR_OUT						0				/* to device */
#define USB_DIR_IN						0x80			/* to host */

/*
 * USB types, the second of three bRequestType fields
 */
#define USB_TYPE_MASK					(0x03 << 5)
#define USB_TYPE_STANDARD				(0x00 << 5)
#define USB_TYPE_CLASS					(0x01 << 5)
#define USB_TYPE_VENDOR					(0x02 << 5)
#define USB_TYPE_RESERVED				(0x03 << 5)

/*
 * USB recipients, the third of three bRequestType fields
 */
#define USB_RECIP_MASK					0x1f
#define USB_RECIP_DEVICE				0x00
#define USB_RECIP_INTERFACE				0x01
#define USB_RECIP_ENDPOINT				0x02
#define USB_RECIP_OTHER					0x03

/*
 * Standard requests, for the bRequest field of a SETUP packet.
 *
 * These are qualified by the bRequestType field, so that for example
 * TYPE_CLASS or TYPE_VENDOR specific feature flags could be retrieved
 * by a GET_STATUS request.
 */
#define USB_REQ_GET_STATUS				0x00
#define USB_REQ_CLEAR_FEATURE			0x01
#define USB_REQ_SET_FEATURE				0x03
#define USB_REQ_SET_ADDRESS				0x05
#define USB_REQ_GET_DESCRIPTOR			0x06
#define USB_REQ_SET_DESCRIPTOR			0x07
#define USB_REQ_GET_CONFIGURATION		0x08
#define USB_REQ_SET_CONFIGURATION		0x09
#define USB_REQ_GET_INTERFACE			0x0A
#define USB_REQ_SET_INTERFACE			0x0B
#define USB_REQ_SYNCH_FRAME				0x0C
#define USB_REQ_SET_SEL					0x30
/*
 * Descriptor types ... USB 2.0 spec table 9.5
 */
#define USB_DT_DEVICE					0x01
#define USB_DT_CONFIG					0x02
#define USB_DT_STRING					0x03
#define USB_DT_INTERFACE				0x04
#define USB_DT_ENDPOINT					0x05
#define USB_DT_DEVICE_QUALIFIER			0x06
#define USB_DT_OTHER_SPEED_CONFIG		0x07
#define USB_DT_INTERFACE_POWER			0x08
/* these are from a minor usb 2.0 revision (ECN) */
#define USB_DT_OTG						0x09
#define USB_DT_DEBUG					0x0a
#define USB_DT_INTERFACE_ASSOCIATION	0x0b
/*
 * Device and/or Interface Class codes
 * as found in bDeviceClass or bInterfaceClass
 * and defined by www.usb.org documents
 */
#define USB_CLASS_PER_INTERFACE			0		/* for DeviceClass */
#define USB_CLASS_CDC_DATA				0x0a 
#define TC9560_VENDOR_ID 				0x04C6
#define TC9560_PRODUCT_ID				0xB0AB 
#define	TC9560_STRING_MANUFACTURER		0
#define	TC9560_STRING_MANUFACTURER		0
#define	TC9560_STRING_SERIAL			0
/* from config descriptor bmAttributes */
#define USB_CONFIG_ATT_ONE				(1 << 7)		/* must be set */
#define USB_CONFIG_ATT_SELFPOWER		(1 << 6)		/* self powered */
#define USB_CONFIG_ATT_WAKEUP			(1 << 5)		/* can wakeup */
#define USB_CONFIG_ATT_BATTERY			(1 << 4)		/* battery powered */
#define REGISTERED						0
#define IGNORE_BULK_OUT					1
#define SUSPENDED						2
#define EP_IN_NUMBER					4
#define EP_OUT_NUMBER					4
#define AUTOCONF_FAIL					1
#define OUT_NEU_BIND					2
/*************** STRUCTURES *****************************/
struct usb_descriptor_header {
	uint8_t bLength;
	uint8_t bDescriptorType;
}__attribute__ ((packed));

/* USB_DT_DEVICE: Device descriptor */ // TODO: ref - ch9.h
struct usb_device_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;

	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
}__attribute__ ((packed));

struct usb_config_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;

	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
}__attribute__ ((packed));

/* USB_DT_INTERFACE: Interface descriptor */
struct usb_interface_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;

	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
}__attribute__ ((packed));

static struct usb_interface_descriptor neu_intf_desc;

struct usb_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;

	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;

	/* NOTE:  these two are _only_ in audio endpoints. */
	/* use USB_DT_ENDPOINT*_SIZE in bLength, not sizeof. */
	uint8_t bRefresh;
	uint8_t bSynchAddress;
}__attribute__ ((packed));

/* USB_DT_SS_ENDPOINT_COMP: SuperSpeed Endpoint Companion descriptor */
struct usb_ss_ep_comp_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;

	uint8_t bMaxBurst;
	uint8_t bmAttributes;
	uint16_t wBytesPerInterval;
}__attribute__ ((packed));

enum usb_device_state {
	/* NOTATTACHED isn't in the USB spec, and this state acts
	 * the same as ATTACHED ... but it's clearer this way.
	 */
	USB_STATE_NOTATTACHED = 0,

	/* chapter 9 and authentication (wireless) device states */
	USB_STATE_ATTACHED, USB_STATE_POWERED, /* wired */
	USB_STATE_RECONNECTING, /* auth */
	USB_STATE_UNAUTHENTICATED, /* auth */
	USB_STATE_DEFAULT, /* limited function */
	USB_STATE_ADDRESS, USB_STATE_CONFIGURED, /* most functions */

	USB_STATE_SUSPENDED

/* NOTE:  there are actually four different SUSPENDED
 * states, returning to POWERED, DEFAULT, ADDRESS, or
 * CONFIGURED respectively when SOF tokens flow again.
 * At this level there's no difference between L1 and L2
 * suspend states.	(L2 being original USB 1.1 suspend.)
 */
};

/**
 * struct usb_gadget - represents a usb slave device
 * @work: (internal use) Workqueue to be used for sysfs_notify()
 * @ops: Function pointers used to access hardware-specific operations.
 * @ep0: Endpoint zero, used when reading or writing responses to
 *		driver setup() requests
 * @ep_list: List of other endpoints supported by the device.
 * @speed: Speed of current connection to USB host.
 * @max_speed: Maximal speed the UDC can handle.  UDC must support this
 *		and all slower speeds.
 * @state: the state we are now (attached, suspended, configured, etc)
 * @name: Identifies the controller hardware type.	Used in diagnostics
 *		and sometimes configuration.
 * @dev: Driver model state for this abstract device.
 * @out_epnum: last used out ep number
 * @in_epnum: last used in ep number
 * @sg_supported: true if we can handle scatter-gather
 * @is_otg: True if the USB device port uses a Mini-AB jack, so that the
 *		gadget driver must provide a USB OTG descriptor.
 * @is_a_peripheral: False unless is_otg, the "A" end of a USB cable
 *		is in the Mini-AB jack, and HNP has been used to switch roles
 *		so that the "A" device currently acts as A-Peripheral, not A-Host.
 * @a_hnp_support: OTG device feature flag, indicating that the A-Host
 *		supports HNP at this port.
 * @a_alt_hnp_support: OTG device feature flag, indicating that the A-Host
 *		only supports HNP on a different root port.
 * @b_hnp_enable: OTG device feature flag, indicating that the A-Host
 *		enabled HNP support.
 * @quirk_ep_out_aligned_size: epout requires buffer size to be aligned to
 *		MaxPacketSize.
 *
 * Gadgets have a mostly-portable "gadget driver" implementing device
 * functions, handling all usb configurations and interfaces.  Gadget
 * drivers talk to hardware-specific code indirectly, through ops vectors.
 * That insulates the gadget driver from hardware details, and packages
 * the hardware endpoints through generic i/o queues.  The "usb_gadget"
 * and "usb_ep" interfaces provide that insulation from the hardware.
 *
 * Except for the driver data, all fields in this structure are
 * read-only to the gadget driver.	That driver data is part of the
 * "driver model" infrastructure in 2.6 (and later) kernels, and for
 * earlier systems is grouped in a similar structure that's not known
 * to the rest of the kernel.
 *
 * Values of the three OTG device feature flags are updated before the
 * setup() call corresponding to USB_REQ_SET_CONFIGURATION, and before
 * driver suspend() calls.	They are valid only when is_otg, and when the
 * device is acting as a B-Peripheral (so is_a_peripheral is false).
 */
 
struct usb_gadget {

	/* readonly to gadget driver */
	const struct usb_gadget_ops * ops;
	struct usb_ep * ep0;
	struct list_head ep_list; /* of usb_ep */
	enum usb_device_speed speed;
	enum usb_device_speed max_speed;
	enum usb_device_state state;
	const char_t * name;
	struct device dev;
	USIGN out_epnum;
	USIGN in_epnum;

	USIGN sg_supported :1;
	USIGN is_otg :1;
	USIGN is_a_peripheral :1;
	USIGN b_hnp_enable :1;
	USIGN a_hnp_support :1;
	USIGN a_alt_hnp_support :1;
	USIGN quirk_ep_out_aligned_size :1;
};

struct usb_gadget_ops {
	int32_t (* get_frame)(struct usb_gadget const *);
	int32_t (* wakeup)(struct usb_gadget const *);
	int32_t (* set_selfpowered)(struct usb_gadget *, int32_t is_selfpowered);
	int32_t (* vbus_session)(struct usb_gadget *, int32_t is_active);
	int32_t (* vbus_draw)(struct usb_gadget *, USIGN mA);
	int32_t (* pullup)(struct usb_gadget *, int32_t is_on);
	int32_t (* ioctl)(struct usb_gadget *, USIGN code, ULONG param);
	int32_t (* udc_stop)(struct usb_gadget *);
};

/**
 * struct usb_string - wraps a C string and its USB id
 * @id: the (nonzero) ID for this string
 * @s: the string, in UTF-8 encoding
 *
 * If you're using usb_gadget_get_string(), use this to wrap a string
 * together with its ID.
 */
struct usb_string {
	uint8_t id;
	const uint8_t * s;
};

/**
 * struct usb_gadget_strings - a set of USB strings in a given language
 * @language: identifies the strings' language (0x0409 for en-us)
 * @strings: array of strings with their ids
 *
 * If you're using usb_gadget_get_string(), use this to wrap all the
 * strings for a given language.
 */
struct usb_gadget_strings {
	uint16_t language; /* 0x0409 for en-us */
	struct usb_string * strings;
};

/**
 * struct usb_ctrlrequest - SETUP data for a USB device control request
 * @bRequestType: matches the USB bmRequestType field
 * @bRequest: matches the USB bRequest field
 * @wValue: matches the USB wValue field (le16 byte order)
 * @wIndex: matches the USB wIndex field (le16 byte order)
 * @wLength: matches the USB wLength field (le16 byte order)
 *
 * This structure is used to send control requests to a USB device.	 It matches
 * the different fields of the USB 2.0 Spec section 9.3, table 9-2.	 See the
 * USB spec for a fuller description of the different fields, and what they are
 * used for.
 *
 * Note that the driver for any interface can issue control requests.
 * For most devices, interfaces don't coordinate with each other, so
 * such requests may be made at any time.
 */
struct usb_ctrlrequest {
	uint8_t bRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
}__attribute__ ((packed));

/*
 * STANDARD DESCRIPTORS ... as returned by GET_DESCRIPTOR, or
 * (rarely) accepted by SET_DESCRIPTOR.
 *
 * Note that all multi-byte values here are encoded in little endian
 * byte order "on the wire".  Within the kernel and when exposed
 * through the Linux-USB APIs, they are not converted to cpu byte
 * order; it is the responsibility of the client code to do this.
 * The single exception is when device and configuration descriptors (but
 * not other descriptors) are read from usbfs (i.e. /proc/bus/usb/BBB/DDD);
 * in this case the fields are converted to host endianness by the kernel.
 */

enum fsg_state {

	/* This one isn't used anywhere */
	NEU_STATE_COMMAND_PHASE = -10,
	NEU_STATE_DATA_PHASE,
	NEU_STATE_STATUS_PHASE,

	NEU_STATE_IDLE = 0,
	NEU_STATE_ABORT_BULK_OUT,
	NEU_STATE_RESET,
	NEU_STATE_INTERFACE_CHANGE,
	NEU_STATE_CONFIG_CHANGE,
	NEU_STATE_DISCONNECT,
	NEU_STATE_EXIT,
	NEU_STATE_TERMINATED
};

enum data_direction {
	DATA_DIR_UNKNOWN = 0, DATA_DIR_FROM_HOST, DATA_DIR_TO_HOST, DATA_DIR_NONE
};

#ifdef CONFIG_USB_OTG
static struct usb_otg_descriptor
neu_otg_desc = {
	.bLength = sizeof neu_otg_desc,
	.bDescriptorType = USB_DT_OTG,

	.bmAttributes = USB_OTG_SRP,
};
#endif

struct neu_dev {
	struct usb_gadget *gadget;

	struct usb_ep *ep0; // Handy copy of gadget->ep0
	struct usb_request *ep0req; // For control responses
	uint32_t  bulk_out_maxpacket;
	enum fsg_state state; // For exception handling

	uint8_t config, new_config;

	uint32_t running :1;
	uint32_t bulk_in_enabled :1;
	uint32_t bulk_out_enabled :1;
	
	uint32_t bulk_out1_enabled :1;
	uint32_t bulk_out2_enabled :1;
	uint32_t bulk_out3_enabled :1;
	uint32_t bulk_out4_enabled :1;
	uint32_t bulk_in5_enabled :1;
	uint32_t bulk_in6_enabled :1;
	uint32_t bulk_in7_enabled :1;
	uint32_t bulk_in8_enabled :1;
	uint32_t phase_error :1;
	uint32_t short_packet_received :1;

	struct usb_ep *bulk_in;
	struct usb_ep *bulk_out;
	
	struct usb_ep *bulk_out1;
	struct usb_ep *bulk_out2;
	struct usb_ep *bulk_out3;
	struct usb_ep *bulk_out4;
	struct usb_ep *bulk_in5;
	struct usb_ep *bulk_in6;
	struct usb_ep *bulk_in7;
	struct usb_ep *bulk_in8;
	struct usb_ep *intr_in;

	enum data_direction data_dir;
	uint32_t data_size;

	/* Must be the last entry */
	struct neu_buffhd buffhds[NEU_NUMBER_BUFFHD];
};
/* There is only one interface. */
#if 1
///////////////////////////////////////////////////////////////////////

void neu_intf_desc_call()  { 
	neu_intf_desc.bLength 				= sizeof (neu_intf_desc);
	neu_intf_desc.bDescriptorType 		= USB_DT_INTERFACE;
	neu_intf_desc.bInterfaceNumber 		= 0x00;
	neu_intf_desc.bAlternateSetting 	= 0x00;
	neu_intf_desc.bNumEndpoints 		= 2; // * Adjusted during neu_bind() *
	neu_intf_desc.bInterfaceClass 		= 0xFF;
	neu_intf_desc.bInterfaceSubClass 	= 0xFF;	  // Adjusted during neu_bind() *
	neu_intf_desc.bInterfaceProtocol 	= 0x00;	  // Adjusted during neu_bind() *
	neu_intf_desc.iInterface 			= 0x00;
}
////////////////////////////////////////////////////////////////////
#endif
/*
 * USB 2.0 devices need to expose both high speed and full speed
 * descriptors, unless they only run at full speed.
 *
 * That means alternate endpoint descriptors (bigger packets)
 * and a "device qualifier" ... plus more construction options
 * for the configuration descriptor.
 */

/**
 * struct usb_gadget_driver - driver for usb 'slave' devices
 * @function: String describing the gadget's function
 * @max_speed: Highest speed the driver handles.
 * @setup: Invoked for ep0 control requests that aren't handled by
 *		the hardware level driver. Most calls must be handled by
 *		the gadget driver, including descriptor and configuration
 *		management.	 The 16 bit members of the setup data are in
 *		USB byte order. Called in_interrupt; this may not sleep.  Driver
 *		queues a response to ep0, or returns negative to stall.
 * @disconnect: Invoked after all transfers have been stopped,
 *		when the host is disconnected.	May be called in_interrupt; this
 *		may not sleep.	Some devices can't detect disconnect, so this might
 *		not be called except as part of controller shutdown.
 * @bind: the driver's bind callback
 * @unbind: Invoked when the driver is unbound from a gadget,
 *		usually from rmmod (after a disconnect is reported).
 *		Called in a context that permits sleeping.
 * @suspend: Invoked on USB suspend.  May be called in_interrupt.
 * @resume: Invoked on USB resume.	May be called in_interrupt.
 * @reset: Invoked on USB bus reset. It is mandatory for all gadget drivers
 *		and should be called in_interrupt.
 * @driver: Driver model state for this driver.
 *
 * Devices are disabled till a gadget driver successfully bind()s, which
 * means the driver will handle setup() requests needed to enumerate (and
 * meet "chapter 9" requirements) then do some useful work.
 *
 * If gadget->is_otg is true, the gadget driver must provide an OTG
 * descriptor during enumeration, or else fail the bind() call.	 In such
 * cases, no USB traffic may flow until both bind() returns without
 * having called usb_gadget_disconnect(), and the USB host stack has
 * initialized.
 *
 * Drivers use hardware-specific knowledge to configure the usb hardware.
 * endpoint addressing is only one of several hardware characteristics that
 * are in descriptors the ep0 implementation returns from setup() calls.
 *
 * Except for ep0 implementation, most driver code shouldn't need change to
 * run on top of different usb controllers.	 It'll use endpoints set up by
 * that ep0 implementation.
 *
 * The usb controller driver handles a few standard usb requests.  Those
 * include set_address, and feature flags for devices, interfaces, and
 * endpoints (the get_status, set_feature, and clear_feature requests).
 *
 * Accordingly, the driver's setup() callback must always implement all
 * get_descriptor requests, returning at least a device descriptor and
 * a configuration descriptor.	Drivers must make sure the endpoint
 * descriptors match any hardware constraints. Some hardware also constrains
 * other descriptors. (The pxa250 allows only configurations 1, 2, or 3).
 *
 * The driver's setup() callback must also implement set_configuration,
 * and should also implement set_interface, get_configuration, and
 * get_interface.  Setting a configuration (or interface) is where
 * endpoints should be activated or (config 0) shut down.
 *
 * (Note that only the default control endpoint is supported.  Neither
 * hosts nor devices generally support control traffic except to ep0.)
 *
 * Most devices will ignore USB suspend/resume operations, and so will
 * not provide those callbacks.	 However, some may need to change modes
 * when the host is not longer directing those activities.	For example,
 * local controls (buttons, dials, etc) may need to be re-enabled since
 * the (remote) host can't do that any longer; or an error state might
 * be cleared, to make the device behave identically whether or not
 * power is maintained.
 */
struct usb_gadget_driver {
	char_t * function;
	enum usb_device_speed max_speed;
	int32_t (* bind)(struct usb_gadget *gadget);
	void (* unbind)(struct usb_gadget *);
	int32_t (* setup)(struct usb_gadget const *, const struct usb_ctrlrequest *);
	void (* disconnect)(struct usb_gadget *);
	void (* suspend)(struct usb_gadget *);
	void (* resume)(struct usb_gadget *);
	void (* reset)(struct usb_gadget *);
};

static struct gadget_wrapper {
	dwc_otg_pcd_t * pcd;
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct usb_ep ep0;
	struct usb_ep in_ep[EP_IN_NUMBER];
	struct usb_ep out_ep[EP_OUT_NUMBER];
}*gadget_wrapper;

/*****************************function declaration*********************/
static inline uint32_t usb_endpoint_maxp(const struct usb_endpoint_descriptor * epd);
static void bulk_in_complete(struct usb_ep const * ep, struct usb_request const * req);
static void bulk_in6_complete(struct usb_ep const * ep, struct usb_request const * req);
static void bulk_in7_complete(struct usb_ep const * ep, struct usb_request const * req);
static void bulk_in8_complete(struct usb_ep const * ep, struct usb_request const * req);
static void bulk_out0_complete(struct usb_ep const * ep, struct usb_request * req);
static void bulk_out1_complete(struct usb_ep const * ep, struct usb_request * req);
static void bulk_out2_complete(struct usb_ep const * ep, struct usb_request * req);
static void bulk_out3_complete(struct usb_ep const * ep, struct usb_request * req);
static int32_t ep_enable(struct usb_ep * usb_ep,const struct usb_endpoint_descriptor * ep_desc);
static int32_t ep_disable(struct usb_ep const * usb_ep);
int32_t ep_queue(struct usb_ep const * usb_ep, struct usb_request * usb_req, gfp_t gfp_flags);

static int32_t get_frame_number(struct usb_gadget const * gadget);
static int32_t wakeup(struct usb_gadget const * gadget);
static int32_t setup(dwc_otg_pcd_t const * pcd, uint8_t const * bytes);
static int32_t complete(dwc_otg_pcd_t const * pcd, void const * ep_handle, void * req_handle, int32_t status, uint32_t actual);
static int32_t connect(dwc_otg_pcd_t const * pcd, int32_t speed);
#if(HSIC_FW_UNTESTED_FEATURE == ENABLED)
static int32_t disconnect(dwc_otg_pcd_t const * pcd);
static int32_t resume(dwc_otg_pcd_t const * pcd);
#endif
static int32_t suspend(dwc_otg_pcd_t const * pcd);
static int32_t int_hnp_changed(dwc_otg_pcd_t const * pcd);
static int32_t reset(dwc_otg_pcd_t const * pcd);
static struct gadget_wrapper * alloc_wrapper(struct lm_device const * _dev);
static int32_t do_set_config(struct neu_dev * neu, uint8_t new_config);
extern cb_handler_t cb_array[2];
/* extern void (* global_isr_table[])(void); */
extern void usb_start_transmit(uint8_t const * p_data, struct usb_request * req, uint8_t bulk_out_num);
/* void neu_ep_queue(uint32_t buf_addr, const char_t* ep_name); */
uint8_t free_request = 0;
uint8_t * ptr = NULL;
void neu_ep_queue_out(uint32_t req_num, const char_t * ep_name,uint8_t out_ep);
int32_t neu_ep_queue_in(uint8_t * buf_addr, uint8_t bulk_in_num, uint32_t size);
char_t debug_ping = 0;
extern void dump_msg(const uint8_t * buf, uint32_t length);
int32_t neu_bind(struct usb_gadget * gadget);
static void neu_unbind(struct usb_gadget * gadget);
static struct usb_request * dwc_otg_pcd_alloc_request(struct usb_ep const * ep,
		int32_t gfp_flags);
static void dwc_otg_pcd_free_request(struct usb_ep const * ep, struct usb_request * req);
struct neu_dev * the_neu;
#endif
