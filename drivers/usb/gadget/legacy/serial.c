// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * serial.c -- USB gadget serial driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 */

#include <common.h>
#include <errno.h>
#include <init.h>
#include <linux/err.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>
#include <linux/usb/usbserial.h>
#include <asm/byteorder.h>

#include "u_serial.h"


/* Defines */

#define GS_VERSION_STR			"v2.4"
#define GS_VERSION_NUM			0x2400

#define GS_LONG_NAME			"Gadget Serial"
#define GS_VERSION_NAME			GS_LONG_NAME " " GS_VERSION_STR

/*-------------------------------------------------------------------------*/
static struct usb_composite_overwrite coverwrite;

/* Thanks to NetChip Technologies for donating this product ID.
*
* DO NOT REUSE THESE IDs with a protocol-incompatible driver!!  Ever!!
* Instead:  allocate your own, using normal USB-IF procedures.
*/
#define GS_VENDOR_ID			0x0525	/* NetChip */
#define GS_PRODUCT_ID			0xa4a6	/* Linux-USB Serial Gadget */
#define GS_CDC_PRODUCT_ID		0xa4a7	/* ... as CDC-ACM */
#define GS_CDC_OBEX_PRODUCT_ID		0xa4a9	/* ... as CDC-OBEX */

/* string IDs are assigned dynamically */

#define STRING_DESCRIPTION_IDX		USB_GADGET_FIRST_AVAIL_IDX

static struct usb_string strings_dev[] = {
	[USB_GADGET_MANUFACTURER_IDX].s = "",
	[USB_GADGET_PRODUCT_IDX].s = GS_VERSION_NAME,
	[USB_GADGET_SERIAL_IDX].s = "",
	[STRING_DESCRIPTION_IDX].s = NULL /* updated; f(use_acm) */,
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength =		USB_DT_DEVICE_SIZE,
	.bDescriptorType =	USB_DT_DEVICE,
	.bcdUSB =		cpu_to_le16(0x0200),
	/* .bDeviceClass = f(use_acm) */
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	/* .bMaxPacketSize0 = f(hardware) */
	/* .idProduct =	f(use_acm) */
	.bcdDevice = cpu_to_le16(GS_VERSION_NUM),
	/* .iManufacturer = DYNAMIC */
	/* .iProduct = DYNAMIC */
	.bNumConfigurations =	1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/* REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

/*-------------------------------------------------------------------------*/

/* Module */
MODULE_DESCRIPTION(GS_VERSION_NAME);
MODULE_AUTHOR("Al Borchers");
MODULE_AUTHOR("David Brownell");
MODULE_LICENSE("GPL");

static bool use_acm = true;

static bool use_obex = false;

static unsigned n_ports = 1;

/*-------------------------------------------------------------------------*/

static struct usb_configuration serial_config_driver = {
	/* .label = f(use_acm) */
	/* .bConfigurationValue = f(use_acm) */
	/* .iConfiguration = DYNAMIC */
	.bmAttributes	= USB_CONFIG_ATT_SELFPOWER,
};

static struct usb_function_instance *fi_serial[MAX_U_SERIAL_PORTS];
static struct usb_function *f_serial[MAX_U_SERIAL_PORTS];

static int serial_register_ports(struct usb_composite_dev *cdev,
		struct usb_configuration *c, const char *f_name)
{
	int i;
	int ret;

	ret = usb_add_config_only(cdev, c);
	if (ret)
		goto out;

	for (i = 0; i < n_ports; i++) {

		fi_serial[i] = usb_get_function_instance(f_name);
		if (IS_ERR(fi_serial[i])) {
			ret = PTR_ERR(fi_serial[i]);
			goto fail;
		}

		f_serial[i] = usb_get_function(fi_serial[i]);
		if (IS_ERR(f_serial[i])) {
			ret = PTR_ERR(f_serial[i]);
			goto err_get_func;
		}

		ret = usb_add_function(c, f_serial[i]);
		if (ret)
			goto err_add_func;
	}

	return 0;

err_add_func:
	usb_put_function(f_serial[i]);
err_get_func:
	usb_put_function_instance(fi_serial[i]);

fail:
	i--;
	while (i >= 0) {
		usb_remove_function(c, f_serial[i]);
		usb_put_function(f_serial[i]);
		usb_put_function_instance(fi_serial[i]);
		i--;
	}
out:
	return ret;
}

static int __init gs_bind(struct usb_composite_dev *cdev)
{
	int			status;
	struct usb_gadget	*gadget = cdev->gadget;

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */

	if (gadget->vendor_id && gadget->product_id) {
		device_desc.idVendor = cpu_to_le16(gadget->vendor_id);
		device_desc.idProduct = cpu_to_le16(gadget->product_id);
	} else {
		device_desc.idVendor = cpu_to_le16(GS_VENDOR_ID);
		if (use_acm)
			device_desc.idProduct = cpu_to_le16(GS_CDC_PRODUCT_ID);
		else
			device_desc.idProduct = cpu_to_le16(GS_PRODUCT_ID);
	}

	strings_dev[USB_GADGET_MANUFACTURER_IDX].s = gadget->manufacturer;
	strings_dev[USB_GADGET_PRODUCT_IDX].s = gadget->productname;

	status = usb_string_ids_tab(cdev, strings_dev);
	if (status < 0)
		goto fail;
	device_desc.iManufacturer = strings_dev[USB_GADGET_MANUFACTURER_IDX].id;
	device_desc.iProduct = strings_dev[USB_GADGET_PRODUCT_IDX].id;
	status = strings_dev[STRING_DESCRIPTION_IDX].id;
	serial_config_driver.iConfiguration = status;

	if (gadget_is_otg(cdev->gadget)) {
		serial_config_driver.descriptors = otg_desc;
		serial_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	/* register our configuration */
	if (use_acm) {
		status  = serial_register_ports(cdev, &serial_config_driver,
				"acm");
		usb_ep_autoconfig_reset(cdev->gadget);
	} else if (use_obex)
		status = serial_register_ports(cdev, &serial_config_driver,
				"obex");
	else {
		status = serial_register_ports(cdev, &serial_config_driver,
				"gser");
	}
	if (status < 0)
		goto fail;

	usb_composite_overwrite_options(cdev, &coverwrite);
	INFO(cdev, "%s\n", GS_VERSION_NAME);

	return 0;

fail:
	return status;
}

static int gs_unbind(struct usb_composite_dev *cdev)
{
	int i;

	for (i = 0; i < n_ports; i++) {
		usb_put_function(f_serial[i]);
		usb_put_function_instance(fi_serial[i]);
	}
	return 0;
}

static struct usb_composite_driver gserial_driver = {
	.name		= "g_serial",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.max_speed	= USB_SPEED_SUPER,
	.bind		= gs_bind,
	.unbind		= gs_unbind,
};

int usb_serial_register(struct usb_serial_pdata *pdata)
{
	/* We *could* export two configs; that'd be much cleaner...
	 * but neither of these product IDs was defined that way.
	 */

	use_acm = pdata->acm;

	/*
	 * PXA CPU suffer a silicon bug which prevents them from being a
	 * compound device, forbiding the ACM configurations.
	 */

	if (IS_ENABLED(CONFIG_ARCH_PXA2XX))
		use_acm = 0;

	if (use_acm) {
		serial_config_driver.label = "CDC ACM config";
		serial_config_driver.bConfigurationValue = 2;
		device_desc.bDeviceClass = USB_CLASS_COMM;
	} else {
		serial_config_driver.label = "Generic Serial config";
		serial_config_driver.bConfigurationValue = 1;
		device_desc.bDeviceClass = USB_CLASS_VENDOR_SPEC;
	}

	return usb_composite_probe(&gserial_driver);
}

void usb_serial_unregister(void)
{
	usb_composite_unregister(&gserial_driver);
}
