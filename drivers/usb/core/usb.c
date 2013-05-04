/*
 *
 * Most of this source has been derived from the Linux USB
 * project:
 * (C) Copyright Linus Torvalds 1999
 * (C) Copyright Johannes Erdfelt 1999-2001
 * (C) Copyright Andreas Gal 1999
 * (C) Copyright Gregory P. Smith 1999
 * (C) Copyright Deti Fliegl 1999 (new USB architecture)
 * (C) Copyright Randy Dunlap 2000
 * (C) Copyright David Brownell 2000 (kernel hotplug, usb_device_id)
 * (C) Copyright Yggdrasil Computing, Inc. 2000
 *     (usb_device_id matching changes by Adam J. Richter)
 *
 * Adapted for barebox:
 * (C) Copyright 2001 Denis Peter, MPL AG Switzerland
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 */

/*
 * How it works:
 *
 * Since this is a bootloader, the devices will not be automatic
 * (re)configured on hotplug, but after a restart of the USB the
 * device should work.
 *
 * For each transfer (except "Interrupt") we wait for completion.
 */
#include <common.h>
#include <command.h>
#include <malloc.h>
#include <driver.h>
#include <linux/ctype.h>
#include <asm/byteorder.h>
#include <xfuncs.h>
#include <init.h>
#include <dma.h>

#include <usb/usb.h>

/* #define USB_DEBUG */

#ifdef	USB_DEBUG
#define	USB_PRINTF(fmt, args...)	printf(fmt , ##args)
#else
#define USB_PRINTF(fmt, args...)
#endif

#define USB_BUFSIZ	512

static int dev_index;
static int asynch_allowed;

static int usb_hub_probe(struct usb_device *dev, int ifnum);
static int hub_port_reset(struct usb_device *dev, int port,
			  unsigned short *portstat);

static LIST_HEAD(host_list);
static LIST_HEAD(usb_device_list);

static void print_usb_device(struct usb_device *dev)
{
	printf("Bus %03d Device %03d: ID %04x:%04x %s\n",
		dev->host->busnum, dev->devnum,
		dev->descriptor->idVendor,
		dev->descriptor->idProduct,
		dev->prod);
}

static int host_busnum = 1;

int usb_register_host(struct usb_host *host)
{
	list_add_tail(&host->list, &host_list);
	host->busnum = host_busnum++;
	asynch_allowed = 1;
	return 0;
}

/**
 * set configuration number to configuration
 */
static int usb_set_configuration(struct usb_device *dev, int configuration)
{
	int res;
	USB_PRINTF("set configuration %d\n", configuration);
	/* set setup command */
	res = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				USB_REQ_SET_CONFIGURATION, 0,
				configuration, 0,
				NULL, 0, USB_CNTL_TIMEOUT);
	if (res == 0) {
		dev->toggle[0] = 0;
		dev->toggle[1] = 0;
		return 0;
	} else
		return -1;
}

/* The routine usb_set_maxpacket_ep() is extracted from the loop of routine
 * usb_set_maxpacket(), because the optimizer of GCC 4.x chokes on this routine
 * when it is inlined in 1 single routine. What happens is that the register r3
 * is used as loop-count 'i', but gets overwritten later on.
 * This is clearly a compiler bug, but it is easier to workaround it here than
 * to update the compiler (Occurs with at least several GCC 4.{1,2},x
 * CodeSourcery compilers like e.g. 2007q3, 2008q1, 2008q3 lite editions on ARM)
 */
static void  noinline
usb_set_maxpacket_ep(struct usb_device *dev, struct usb_endpoint_descriptor *ep)
{
	int b;

	b = ep->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;

	if ((ep->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
						USB_ENDPOINT_XFER_CONTROL) {
		/* Control => bidirectional */
		dev->epmaxpacketout[b] = ep->wMaxPacketSize;
		dev->epmaxpacketin[b] = ep->wMaxPacketSize;
		USB_PRINTF("##Control EP epmaxpacketout/in[%d] = %d\n",
			   b, dev->epmaxpacketin[b]);
	} else {
		if ((ep->bEndpointAddress & 0x80) == 0) {
			/* OUT Endpoint */
			if (ep->wMaxPacketSize > dev->epmaxpacketout[b]) {
				dev->epmaxpacketout[b] = ep->wMaxPacketSize;
				USB_PRINTF("##EP epmaxpacketout[%d] = %d\n",
					   b, dev->epmaxpacketout[b]);
			}
		} else {
			/* IN Endpoint */
			if (ep->wMaxPacketSize > dev->epmaxpacketin[b]) {
				dev->epmaxpacketin[b] = ep->wMaxPacketSize;
				USB_PRINTF("##EP epmaxpacketin[%d] = %d\n",
					   b, dev->epmaxpacketin[b]);
			}
		} /* if out */
	} /* if control */
}

/*
 * set the max packed value of all endpoints in the given configuration
 */
static int usb_set_maxpacket(struct usb_device *dev)
{
	int i, ii;

	for (i = 0; i < dev->config.bNumInterfaces; i++)
		for (ii = 0; ii < dev->config.if_desc[i].bNumEndpoints; ii++)
			usb_set_maxpacket_ep(dev,
					  &dev->config.if_desc[i].ep_desc[ii]);

	return 0;
}

/**
 * Parse the config, located in buffer, and fills the dev->config structure.
 * Note that all little/big endian swapping are done automatically.
 */
static int usb_parse_config(struct usb_device *dev, unsigned char *buffer, int cfgno)
{
	struct usb_descriptor_header *head;
	int index, ifno, epno, curr_if_num;
	int i;
	unsigned char *ch;

	ifno = -1;
	epno = -1;
	curr_if_num = -1;

	dev->configno = cfgno;
	head = (struct usb_descriptor_header *) &buffer[0];
	if (head->bDescriptorType != USB_DT_CONFIG) {
		printf(" ERROR: NOT USB_CONFIG_DESC %x\n",
			head->bDescriptorType);
		return -1;
	}
	memcpy(&dev->config, buffer, buffer[0]);
	le16_to_cpus(&(dev->config.wTotalLength));
	dev->config.no_of_if = 0;

	index = dev->config.bLength;
	/* Ok the first entry must be a configuration entry,
	 * now process the others */
	head = (struct usb_descriptor_header *) &buffer[index];
	while (index + 1 < dev->config.wTotalLength) {
		switch (head->bDescriptorType) {
		case USB_DT_INTERFACE:
			if (((struct usb_interface_descriptor *) \
			     &buffer[index])->bInterfaceNumber != curr_if_num) {
				/* this is a new interface, copy new desc */
				ifno = dev->config.no_of_if;
				/* if ifno > USB_MAXINTERFACES, then
				 * next memcpy() will corrupt dev->config
				 */
				if (ifno > USB_MAXINTERFACES) {
					printf("ifno = %d > "
						"USB_MAXINTERFACES = %d !\n",
						ifno,
						USB_MAXINTERFACES);
					break;
				}
				dev->config.no_of_if++;
				memcpy(&dev->config.if_desc[ifno],
					&buffer[index], buffer[index]);
				dev->config.if_desc[ifno].no_of_ep = 0;
				dev->config.if_desc[ifno].num_altsetting = 1;
				curr_if_num =
				     dev->config.if_desc[ifno].bInterfaceNumber;
			} else {
				/* found alternate setting for the interface */
				dev->config.if_desc[ifno].num_altsetting++;
			}
			break;
		case USB_DT_ENDPOINT:
			epno = dev->config.if_desc[ifno].no_of_ep;
			/* found an endpoint */
			dev->config.if_desc[ifno].no_of_ep++;
			memcpy(&dev->config.if_desc[ifno].ep_desc[epno],
				&buffer[index], buffer[index]);
			le16_to_cpus(&(dev->config.if_desc[ifno].ep_desc[epno].\
							       wMaxPacketSize));
			USB_PRINTF("if %d, ep %d\n", ifno, epno);
			break;
		default:
			if (head->bLength == 0)
				return 1;

			USB_PRINTF("unknown Description Type : %x\n",
				   head->bDescriptorType);

			{
				ch = (unsigned char *)head;
				for (i = 0; i < head->bLength; i++)
					USB_PRINTF("%02X ", *ch++);
				USB_PRINTF("\n\n\n");
			}
			break;
		}
		index += head->bLength;
		head = (struct usb_descriptor_header *)&buffer[index];
	}
	return 1;
}

/**
 * set address of a device to the value in dev->devnum.
 * This can only be done by addressing the device via the default address (0)
 */
static int usb_set_address(struct usb_device *dev)
{
	int res;

	USB_PRINTF("set address %d\n", dev->devnum);
	res = usb_control_msg(dev, usb_snddefctrl(dev),
				USB_REQ_SET_ADDRESS, 0,
				(dev->devnum), 0,
				NULL, 0, USB_CNTL_TIMEOUT);
	return res;
}

static int usb_get_descriptor(struct usb_device *dev, unsigned char type,
			unsigned char index, void *buf, int size)
{
	int res;
	res = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
			(type << 8) + index, 0,
			buf, size, USB_CNTL_TIMEOUT);
	return res;
}

/*
 * By the time we get here, the device has gotten a new device ID
 * and is in the default state. We need to identify the thing and
 * get the ball rolling..
 *
 * Returns 0 for success, != 0 for error.
 */
static int usb_new_device(struct usb_device *dev)
{
	int addr, err;
	int tmp;
	void *buf;
	struct usb_device_descriptor *desc;
	int port = -1;
	struct usb_device *parent = dev->parent;
	unsigned short portstatus;
	char str[16];

	buf = dma_alloc(USB_BUFSIZ);

	/* We still haven't set the Address yet */
	addr = dev->devnum;
	dev->devnum = 0;

	/* This is a Windows scheme of initialization sequence, with double
	 * reset of the device (Linux uses the same sequence)
	 * Some equipment is said to work only with such init sequence; this
	 * patch is based on the work by Alan Stern:
	 * http://sourceforge.net/mailarchive/forum.php?
	 * thread_id=5729457&forum_id=5398
	 */

	/* send 64-byte GET-DEVICE-DESCRIPTOR request.  Since the descriptor is
	 * only 18 bytes long, this will terminate with a short packet.  But if
	 * the maxpacket size is 8 or 16 the device may be waiting to transmit
	 * some more, or keeps on retransmitting the 8 byte header. */

	desc = buf;
	dev->descriptor->bMaxPacketSize0 = 64;	    /* Start off at 64 bytes  */
	/* Default to 64 byte max packet size */
	dev->maxpacketsize = PACKET_SIZE_64;
	dev->epmaxpacketin[0] = 64;
	dev->epmaxpacketout[0] = 64;

	err = usb_get_descriptor(dev, USB_DT_DEVICE, 0, desc, 64);
	if (err < 0) {
		USB_PRINTF("%s: usb_get_descriptor() failed with %d\n", __func__, err);
		goto err_out;
	}

	dev->descriptor->bMaxPacketSize0 = desc->bMaxPacketSize0;

	/* find the port number we're at */
	if (parent) {
		int j;

		for (j = 0; j < parent->maxchild; j++) {
			if (parent->children[j] == dev) {
				port = j;
				break;
			}
		}
		if (port < 0) {
			printf("%s: cannot locate device's port.\n", __func__);
			err = -ENODEV;
			goto err_out;
		}

		/* reset the port for the second time */
		err = hub_port_reset(dev->parent, port, &portstatus);
		if (err < 0) {
			printf("\n     Couldn't reset port %i\n", port);
			goto err_out;
		}
	}

	dev->epmaxpacketin[0] = dev->descriptor->bMaxPacketSize0;
	dev->epmaxpacketout[0] = dev->descriptor->bMaxPacketSize0;
	switch (dev->descriptor->bMaxPacketSize0) {
	case 8:
		dev->maxpacketsize  = PACKET_SIZE_8;
		break;
	case 16:
		dev->maxpacketsize = PACKET_SIZE_16;
		break;
	case 32:
		dev->maxpacketsize = PACKET_SIZE_32;
		break;
	case 64:
		dev->maxpacketsize = PACKET_SIZE_64;
		break;
	}
	dev->devnum = addr;

	err = usb_set_address(dev); /* set address */

	if (err < 0) {
		printf("\n      USB device not accepting new address " \
			"(error=%lX)\n", dev->status);
		goto err_out;
	}

	wait_ms(10);	/* Let the SET_ADDRESS settle */

	tmp = sizeof(*dev->descriptor);

	err = usb_get_descriptor(dev, USB_DT_DEVICE, 0,
				 dev->descriptor, sizeof(*dev->descriptor));
	if (err < tmp) {
		if (err < 0)
			printf("unable to get device descriptor (error=%d)\n",
			       err);
		else
			printf("USB device descriptor short read " \
				"(expected %i, got %i)\n", tmp, err);
		goto err_out;
	}
	/* correct le values */
	le16_to_cpus(&dev->descriptor->bcdUSB);
	le16_to_cpus(&dev->descriptor->idVendor);
	le16_to_cpus(&dev->descriptor->idProduct);
	le16_to_cpus(&dev->descriptor->bcdDevice);
	/* only support for one config for now */
	usb_get_configuration_no(dev, buf, 0);
	usb_parse_config(dev, buf, 0);
	usb_set_maxpacket(dev);
	/* we set the default configuration here */
	if (usb_set_configuration(dev, dev->config.bConfigurationValue)) {
		printf("failed to set default configuration " \
			"len %d, status %lX\n", dev->act_len, dev->status);
		goto err_out;
	}
	USB_PRINTF("new device: Mfr=%d, Product=%d, SerialNumber=%d\n",
		   dev->descriptor->iManufacturer, dev->descriptor->iProduct,
		   dev->descriptor->iSerialNumber);
	memset(dev->mf, 0, sizeof(dev->mf));
	memset(dev->prod, 0, sizeof(dev->prod));
	memset(dev->serial, 0, sizeof(dev->serial));
	if (dev->descriptor->iManufacturer)
		usb_string(dev, dev->descriptor->iManufacturer,
			   dev->mf, sizeof(dev->mf));
	if (dev->descriptor->iProduct)
		usb_string(dev, dev->descriptor->iProduct,
			   dev->prod, sizeof(dev->prod));
	if (dev->descriptor->iSerialNumber)
		usb_string(dev, dev->descriptor->iSerialNumber,
			   dev->serial, sizeof(dev->serial));
	/* now prode if the device is a hub */
	usb_hub_probe(dev, 0);

	sprintf(dev->dev.name, "usb%d-%d", dev->host->busnum, dev->devnum);

	print_usb_device(dev);

	register_device(&dev->dev);
	sprintf(str, "%d", dev->descriptor->iManufacturer);
	dev_add_param_fixed(&dev->dev, "iManufacturer", str);
	sprintf(str, "%d", dev->descriptor->iProduct);
	dev_add_param_fixed(&dev->dev, "iProduct", str);
	sprintf(str, "%d", dev->descriptor->iSerialNumber);
	dev_add_param_fixed(&dev->dev, "iSerialNumber", str);
	dev_add_param_fixed(&dev->dev, "Manufacturer", dev->mf);
	dev_add_param_fixed(&dev->dev, "Product", dev->prod);
	dev_add_param_fixed(&dev->dev, "SerialNumber", dev->serial);
	sprintf(str, "%04x", le16_to_cpu(dev->descriptor->idVendor));
	dev_add_param_fixed(&dev->dev, "idVendor", str);
	sprintf(str, "%04x", le16_to_cpu(dev->descriptor->idProduct));
	dev_add_param_fixed(&dev->dev, "idProduct", str);
	list_add_tail(&dev->list, &usb_device_list);

	err = 0;

err_out:
	dma_free(buf);
	return err;
}

static struct usb_device *usb_alloc_new_device(void)
{
	struct usb_device *usbdev = xzalloc(sizeof (*usbdev));

	if (!usbdev)
		return NULL;

	usbdev->devnum = dev_index + 1;
	usbdev->maxchild = 0;
	usbdev->dev.bus = &usb_bus_type;
	usbdev->setup_packet = dma_alloc(sizeof(*usbdev->setup_packet));
	usbdev->descriptor = dma_alloc(sizeof(*usbdev->descriptor));

	dev_index++;

	return usbdev;
}

void usb_rescan(void)
{
	struct usb_device *dev, *tmp;
	struct usb_host *host;
	int ret;

	list_for_each_entry_safe(dev, tmp, &usb_device_list, list) {
		list_del(&dev->list);
		unregister_device(&dev->dev);
		if (dev->hub)
			free(dev->hub);
		dma_free(dev->setup_packet);
		dma_free(dev->descriptor);
		free(dev);
	}

	printf("USB: scanning bus for devices...\n");
	dev_index = 0;

	list_for_each_entry(host, &host_list, list) {
		ret = host->init(host);
		if (ret)
			continue;

		dev = usb_alloc_new_device();
		dev->host = host;
		usb_new_device(dev);
	}

	printf("%d USB Device(s) found\n", dev_index);
}

/*
 * disables the asynch behaviour of the control message. This is used for data
 * transfers that uses the exclusiv access to the control and bulk messages.
 */
void usb_disable_asynch(int disable)
{
	asynch_allowed = !disable;
}


/*-------------------------------------------------------------------
 * Message wrappers.
 *
 */

/*
 * submits an Interrupt Message
 */
int usb_submit_int_msg(struct usb_device *dev, unsigned long pipe,
			void *buffer, int transfer_len, int interval)
{
	struct usb_host *host = dev->host;

	return host->submit_int_msg(dev, pipe, buffer, transfer_len, interval);
}

/*
 * submits a control message and waits for completion (at least timeout * 1ms)
 * If timeout is 0, we don't wait for completion (used as example to set and
 * clear keyboards LEDs). For data transfers, (storage transfers) we don't
 * allow control messages with 0 timeout, by previously resetting the flag
 * asynch_allowed (usb_disable_asynch(1)).
 * returns the transfered length if OK or -1 if error. The transfered length
 * and the current status are stored in the dev->act_len and dev->status.
 */
int usb_control_msg(struct usb_device *dev, unsigned int pipe,
			unsigned char request, unsigned char requesttype,
			unsigned short value, unsigned short index,
			void *data, unsigned short size, int timeout)
{
	struct usb_host *host = dev->host;
	int ret;
	struct devrequest *setup_packet = dev->setup_packet;

	if ((timeout == 0) && (!asynch_allowed)) {
		/* request for a asynch control pipe is not allowed */
		return -1;
	}

	/* set setup command */
	setup_packet->requesttype = requesttype;
	setup_packet->request = request;
	setup_packet->value = cpu_to_le16(value);
	setup_packet->index = cpu_to_le16(index);
	setup_packet->length = cpu_to_le16(size);
	USB_PRINTF("usb_control_msg: request: 0x%X, requesttype: 0x%X, " \
		   "value 0x%X index 0x%X length 0x%X\n",
		   request, requesttype, value, index, size);
	dev->status = USB_ST_NOT_PROC; /*not yet processed */

	ret = host->submit_control_msg(dev, pipe, data, size, setup_packet,
			timeout);
	if (ret)
		return ret;

	return dev->act_len;
}

/*-------------------------------------------------------------------
 * submits bulk message, and waits for completion. returns 0 if Ok or
 * -1 if Error.
 * synchronous behavior
 */
int usb_bulk_msg(struct usb_device *dev, unsigned int pipe,
			void *data, int len, int *actual_length, int timeout)
{
	struct usb_host *host = dev->host;
	int ret;

	if (len < 0)
		return -1;

	dev->status = USB_ST_NOT_PROC; /* not yet processed */
	ret = host->submit_bulk_msg(dev, pipe, data, len, timeout);
	if (ret)
		return ret;

	*actual_length = dev->act_len;

	return (dev->status == 0) ? 0 : -1;
}


/*-------------------------------------------------------------------
 * Max Packet stuff
 */

/*
 * returns the max packet size, depending on the pipe direction and
 * the configurations values
 */
int usb_maxpacket(struct usb_device *dev, unsigned long pipe)
{
	/* direction is out -> use emaxpacket out */
	if ((pipe & USB_DIR_IN) == 0)
		return dev->epmaxpacketout[((pipe>>15) & 0xf)];
	else
		return dev->epmaxpacketin[((pipe>>15) & 0xf)];
}

/***********************************************************************
 * Clears an endpoint
 * endp: endpoint number in bits 0-3;
 * direction flag in bit 7 (1 = IN, 0 = OUT)
 */
int usb_clear_halt(struct usb_device *dev, int pipe)
{
	int result;
	int endp = usb_pipeendpoint(pipe)|(usb_pipein(pipe)<<7);

	result = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				 USB_REQ_CLEAR_FEATURE, USB_RECIP_ENDPOINT, 0,
				 endp, NULL, 0, USB_CNTL_TIMEOUT * 3);

	/* don't clear if failed */
	if (result < 0)
		return result;

	/*
	 * NOTE: we do not get status and verify reset was successful
	 * as some devices are reported to lock up upon this check..
	 */

	usb_endpoint_running(dev, usb_pipeendpoint(pipe), usb_pipeout(pipe));

	/* toggle is reset on clear */
	usb_settoggle(dev, usb_pipeendpoint(pipe), usb_pipeout(pipe), 0);
	return 0;
}

/**********************************************************************
 * gets configuration cfgno and store it in the buffer
 */
int usb_get_configuration_no(struct usb_device *dev,
			     unsigned char *buffer, int cfgno)
{
	int result;
	unsigned int tmp;
	struct usb_config_descriptor *config;


	config = (struct usb_config_descriptor *)&buffer[0];
	result = usb_get_descriptor(dev, USB_DT_CONFIG, cfgno, buffer, 9);
	if (result < 9) {
		if (result < 0)
			printf("unable to get descriptor, error %lX\n",
				dev->status);
		else
			printf("config descriptor too short " \
				"(expected %i, got %i)\n", 9, result);
		return -1;
	}
	tmp = le16_to_cpu(config->wTotalLength);

	if (tmp > USB_BUFSIZ) {
		USB_PRINTF("usb_get_configuration_no: failed to get " \
			   "descriptor - too long: %d\n", tmp);
		return -1;
	}

	result = usb_get_descriptor(dev, USB_DT_CONFIG, cfgno, buffer, tmp);
	USB_PRINTF("get_conf_no %d Result %d, wLength %d\n",
		   cfgno, result, tmp);
	return result;
}

/********************************************************************
 * set interface number to interface
 */
int usb_set_interface(struct usb_device *dev, int interface, int alternate)
{
	struct usb_interface_descriptor *if_face = NULL;
	int ret, i;

	for (i = 0; i < dev->config.bNumInterfaces; i++) {
		if (dev->config.if_desc[i].bInterfaceNumber == interface) {
			if_face = &dev->config.if_desc[i];
			break;
		}
	}
	if (!if_face) {
		printf("selecting invalid interface %d", interface);
		return -1;
	}
	/*
	 * We should return now for devices with only one alternate setting.
	 * According to 9.4.10 of the Universal Serial Bus Specification
	 * Revision 2.0 such devices can return with a STALL. This results in
	 * some USB sticks timeouting during initialization and then being
	 * unusable in barebox.
	 */
	if (if_face->num_altsetting == 1)
		return 0;

	ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				USB_REQ_SET_INTERFACE, USB_RECIP_INTERFACE,
				alternate, interface, NULL, 0,
				USB_CNTL_TIMEOUT * 5);
	if (ret < 0)
		return ret;

	return 0;
}

/********************************************************************
 * set protocol to protocol
 */
int usb_set_protocol(struct usb_device *dev, int ifnum, int protocol)
{
	return usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
		USB_REQ_SET_PROTOCOL, USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		protocol, ifnum, NULL, 0, USB_CNTL_TIMEOUT);
}

/********************************************************************
 * set idle
 */
int usb_set_idle(struct usb_device *dev, int ifnum, int duration, int report_id)
{
	return usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
		USB_REQ_SET_IDLE, USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		(duration << 8) | report_id, ifnum, NULL, 0, USB_CNTL_TIMEOUT);
}

/********************************************************************
 * get report
 */
int usb_get_report(struct usb_device *dev, int ifnum, unsigned char type,
		   unsigned char id, void *buf, int size)
{
	return usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			USB_REQ_GET_REPORT,
			USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			(type << 8) + id, ifnum, buf, size, USB_CNTL_TIMEOUT);
}

/********************************************************************
 * get class descriptor
 */
int usb_get_class_descriptor(struct usb_device *dev, int ifnum,
		unsigned char type, unsigned char id, void *buf, int size)
{
	return usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
		USB_REQ_GET_DESCRIPTOR, USB_RECIP_INTERFACE | USB_DIR_IN,
		(type << 8) + id, ifnum, buf, size, USB_CNTL_TIMEOUT);
}

/********************************************************************
 * get string index in buffer
 */
static int usb_get_string(struct usb_device *dev, unsigned short langid,
		   unsigned char index, void *buf, int size)
{
	int i;
	int result;

	for (i = 0; i < 3; ++i) {
		/* some devices are flaky */
		result = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			USB_REQ_GET_DESCRIPTOR, USB_DIR_IN,
			(USB_DT_STRING << 8) + index, langid, buf, size,
			USB_CNTL_TIMEOUT);

		if (result > 0)
			break;
	}

	return result;
}


static void usb_try_string_workarounds(unsigned char *buf, int *length)
{
	int newlength, oldlength = *length;

	for (newlength = 2; newlength + 1 < oldlength; newlength += 2)
		if (!isprint(buf[newlength]) || buf[newlength + 1])
			break;

	if (newlength > 2) {
		buf[0] = newlength;
		*length = newlength;
	}
}


static int usb_string_sub(struct usb_device *dev, unsigned int langid,
		unsigned int index, unsigned char *buf)
{
	int rc;

	/* Try to read the string descriptor by asking for the maximum
	 * possible number of bytes */
	rc = usb_get_string(dev, langid, index, buf, 255);

	/* If that failed try to read the descriptor length, then
	 * ask for just that many bytes */
	if (rc < 2) {
		rc = usb_get_string(dev, langid, index, buf, 2);
		if (rc == 2)
			rc = usb_get_string(dev, langid, index, buf, buf[0]);
	}

	if (rc >= 2) {
		if (!buf[0] && !buf[1])
			usb_try_string_workarounds(buf, &rc);

		/* There might be extra junk at the end of the descriptor */
		if (buf[0] < rc)
			rc = buf[0];

		rc = rc - (rc & 1); /* force a multiple of two */
	}

	if (rc < 2)
		rc = -1;

	return rc;
}


/********************************************************************
 * usb_string:
 * Get string index and translate it to ascii.
 * returns string length (> 0) or error (< 0)
 */
int usb_string(struct usb_device *dev, int index, char *buf, size_t size)
{
	unsigned char mybuf[USB_BUFSIZ];
	unsigned char *tbuf;
	int err;
	unsigned int u, idx;

	if (size <= 0 || !buf || !index)
		return -1;
	buf[0] = 0;
	tbuf = &mybuf[0];

	/* get langid for strings if it's not yet known */
	if (!dev->have_langid) {
		err = usb_string_sub(dev, 0, 0, tbuf);
		if (err < 0) {
			USB_PRINTF("error getting string descriptor 0 " \
				   "(error=%lx)\n", dev->status);
			return -1;
		} else if (tbuf[0] < 4) {
			USB_PRINTF("string descriptor 0 too short\n");
			return -1;
		} else {
			dev->have_langid = -1;
			dev->string_langid = tbuf[2] | (tbuf[3] << 8);
				/* always use the first langid listed */
			USB_PRINTF("USB device number %d default " \
				   "language ID 0x%x\n",
				   dev->devnum, dev->string_langid);
		}
	}

	err = usb_string_sub(dev, dev->string_langid, index, tbuf);
	if (err < 0)
		return err;

	size--;		/* leave room for trailing NULL char in output buffer */
	for (idx = 0, u = 2; u < err; u += 2) {
		if (idx >= size)
			break;
		if (tbuf[u+1])			/* high byte */
			buf[idx++] = '?';  /* non-ASCII character */
		else
			buf[idx++] = tbuf[u];
	}
	buf[idx] = 0;
	err = idx;
	return err;
}

/****************************************************************************
 * HUB "Driver"
 * Probes device for being a hub and configurate it
 */

#undef	USB_HUB_DEBUG

#ifdef	USB_HUB_DEBUG
#define	USB_HUB_PRINTF(fmt, args...)	printf(fmt , ##args)
#else
#define USB_HUB_PRINTF(fmt, args...)
#endif

static int usb_get_hub_descriptor(struct usb_device *dev, void *data, int size)
{
	return usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
		USB_REQ_GET_DESCRIPTOR, USB_DIR_IN | USB_RT_HUB,
		USB_DT_HUB << 8, 0, data, size, USB_CNTL_TIMEOUT);
}

static int usb_clear_port_feature(struct usb_device *dev, int port, int feature)
{
	return usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				USB_REQ_CLEAR_FEATURE, USB_RT_PORT, feature,
				port, NULL, 0, USB_CNTL_TIMEOUT);
}

static int usb_set_port_feature(struct usb_device *dev, int port, int feature)
{
	return usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
				USB_REQ_SET_FEATURE, USB_RT_PORT, feature,
				port, NULL, 0, USB_CNTL_TIMEOUT);
}

static int usb_get_hub_status(struct usb_device *dev, void *data)
{
	return usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_HUB, 0, 0,
			data, sizeof(struct usb_hub_status), USB_CNTL_TIMEOUT);
}

static int usb_get_port_status(struct usb_device *dev, int port, void *data)
{
	return usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
			USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_PORT, 0, port,
			data, sizeof(struct usb_hub_status), USB_CNTL_TIMEOUT);
}


static void usb_hub_power_on(struct usb_hub_device *hub)
{
	int i;
	struct usb_device *dev;

	dev = hub->pusb_dev;
	/* Enable power to the ports */
	USB_HUB_PRINTF("enabling power on all ports\n");
	for (i = 0; i < dev->maxchild; i++) {
		usb_set_port_feature(dev, i + 1, USB_PORT_FEAT_POWER);
		USB_HUB_PRINTF("port %d returns %lX\n", i + 1, dev->status);
	}
	/* power on is encoded in 2ms increments -> times 2 for the actual delay */
	mdelay(hub->desc.bPwrOn2PwrGood*2);
}

#define MAX_TRIES 5

static inline char *portspeed(int portstatus)
{
	if (portstatus & (1 << USB_PORT_FEAT_HIGHSPEED))
		return "480 Mb/s";
	else if (portstatus & (1 << USB_PORT_FEAT_LOWSPEED))
		return "1.5 Mb/s";
	else
		return "12 Mb/s";
}

static int hub_port_reset(struct usb_device *dev, int port,
			unsigned short *portstat)
{
	int tries;
	struct usb_port_status portsts;
	unsigned short portstatus, portchange;

	USB_HUB_PRINTF("hub_port_reset: resetting port %d...\n", port);
	for (tries = 0; tries < MAX_TRIES; tries++) {

		usb_set_port_feature(dev, port + 1, USB_PORT_FEAT_RESET);
		wait_ms(200);

		if (usb_get_port_status(dev, port + 1, &portsts) < 0) {
			USB_HUB_PRINTF("get_port_status failed status %lX\n",
					dev->status);
			return -1;
		}
		portstatus = le16_to_cpu(portsts.wPortStatus);
		portchange = le16_to_cpu(portsts.wPortChange);

		USB_HUB_PRINTF("portstatus %x, change %x, %s\n",
				portstatus, portchange,
				portspeed(portstatus));

		USB_HUB_PRINTF("STAT_C_CONNECTION = %d STAT_CONNECTION = %d" \
			       "  USB_PORT_STAT_ENABLE %d\n",
			(portchange & USB_PORT_STAT_C_CONNECTION) ? 1 : 0,
			(portstatus & USB_PORT_STAT_CONNECTION) ? 1 : 0,
			(portstatus & USB_PORT_STAT_ENABLE) ? 1 : 0);

		if ((portchange & USB_PORT_STAT_C_CONNECTION) ||
		    !(portstatus & USB_PORT_STAT_CONNECTION))
			return -1;

		if (portstatus & USB_PORT_STAT_ENABLE)
			break;

		wait_ms(200);
	}

	if (tries == MAX_TRIES) {
		USB_HUB_PRINTF("Cannot enable port %i after %i retries, " \
				"disabling port.\n", port + 1, MAX_TRIES);
		USB_HUB_PRINTF("Maybe the USB cable is bad?\n");
		return -1;
	}

	usb_clear_port_feature(dev, port + 1, USB_PORT_FEAT_C_RESET);
	*portstat = portstatus;
	return 0;
}


static void usb_hub_port_connect_change(struct usb_device *dev, int port)
{
	struct usb_device *usb;
	struct usb_port_status portsts;
	unsigned short portstatus, portchange;

	/* Check status */
	if (usb_get_port_status(dev, port + 1, &portsts) < 0) {
		USB_HUB_PRINTF("get_port_status failed\n");
		return;
	}

	portstatus = le16_to_cpu(portsts.wPortStatus);
	portchange = le16_to_cpu(portsts.wPortChange);
	USB_HUB_PRINTF("portstatus %x, change %x, %s\n",
			portstatus, portchange, portspeed(portstatus));

	/* Clear the connection change status */
	usb_clear_port_feature(dev, port + 1, USB_PORT_FEAT_C_CONNECTION);

	/* Disconnect any existing devices under this port */
	if (((!(portstatus & USB_PORT_STAT_CONNECTION)) &&
	     (!(portstatus & USB_PORT_STAT_ENABLE))) || (dev->children[port])) {
		USB_HUB_PRINTF("usb_disconnect(&hub->children[port]);\n");
		/* Return now if nothing is connected */
		if (!(portstatus & USB_PORT_STAT_CONNECTION))
			return;
	}
	wait_ms(200);

	/* Reset the port */
	if (hub_port_reset(dev, port, &portstatus) < 0) {
		printf("cannot reset port %i!?\n", port + 1);
		return;
	}

	wait_ms(200);

	/* Allocate a new device struct for it */
	usb = usb_alloc_new_device();
	usb->host = dev->host;

	if (portstatus & USB_PORT_STAT_HIGH_SPEED)
		usb->speed = USB_SPEED_HIGH;
	else if (portstatus & USB_PORT_STAT_LOW_SPEED)
		usb->speed = USB_SPEED_LOW;
	else
		usb->speed = USB_SPEED_FULL;

	dev->children[port] = usb;
	usb->parent = dev;
	/* Run it through the hoops (find a driver, etc) */
	if (usb_new_device(usb)) {
		/* Woops, disable the port */
		USB_HUB_PRINTF("hub: disabling port %d\n", port + 1);
		usb_clear_port_feature(dev, port + 1, USB_PORT_FEAT_ENABLE);
	}
}


static int usb_hub_configure(struct usb_device *dev)
{
	unsigned char buffer[USB_BUFSIZ], *bitmap;
	struct usb_hub_descriptor *descriptor;
	struct usb_hub_status *hubsts;
	int i;
	struct usb_hub_device *hub;

	hub = xzalloc(sizeof (*hub));
	dev->hub = hub;

	hub->pusb_dev = dev;
	/* Get the the hub descriptor */
	if (usb_get_hub_descriptor(dev, buffer, 4) < 0) {
		USB_HUB_PRINTF("%s: failed to get hub " \
				   "descriptor, giving up %lX\n", __func__, dev->status);
		return -1;
	}
	descriptor = (struct usb_hub_descriptor *)buffer;

	/* silence compiler warning if USB_BUFSIZ is > 256 [= sizeof(char)] */
	i = descriptor->bLength;
	if (i > USB_BUFSIZ) {
		USB_HUB_PRINTF("%s: failed to get hub " \
				"descriptor - too long: %d\n", __func__,
				descriptor->bLength);
		return -1;
	}

	if (usb_get_hub_descriptor(dev, buffer, descriptor->bLength) < 0) {
		USB_HUB_PRINTF("%s: failed to get hub " \
				"descriptor 2nd giving up %lX\n", __func__, dev->status);
		return -1;
	}
	memcpy((unsigned char *)&hub->desc, buffer, descriptor->bLength);
	/* adjust 16bit values */
	hub->desc.wHubCharacteristics =
				le16_to_cpu(descriptor->wHubCharacteristics);
	/* set the bitmap */
	bitmap = (unsigned char *)&hub->desc.DeviceRemovable[0];
	/* devices not removable by default */
	memset(bitmap, 0xff, (USB_MAXCHILDREN+1+7)/8);
	bitmap = (unsigned char *)&hub->desc.PortPowerCtrlMask[0];
	memset(bitmap, 0xff, (USB_MAXCHILDREN+1+7)/8); /* PowerMask = 1B */

	for (i = 0; i < ((hub->desc.bNbrPorts + 1 + 7)/8); i++)
		hub->desc.DeviceRemovable[i] = descriptor->DeviceRemovable[i];

	for (i = 0; i < ((hub->desc.bNbrPorts + 1 + 7)/8); i++)
		hub->desc.DeviceRemovable[i] = descriptor->PortPowerCtrlMask[i];

	dev->maxchild = descriptor->bNbrPorts;
	USB_HUB_PRINTF("%d ports detected\n", dev->maxchild);

	switch (hub->desc.wHubCharacteristics & HUB_CHAR_LPSM) {
	case 0x00:
		USB_HUB_PRINTF("ganged power switching\n");
		break;
	case 0x01:
		USB_HUB_PRINTF("individual port power switching\n");
		break;
	case 0x02:
	case 0x03:
		USB_HUB_PRINTF("unknown reserved power switching mode\n");
		break;
	}

	if (hub->desc.wHubCharacteristics & HUB_CHAR_COMPOUND)
		USB_HUB_PRINTF("part of a compound device\n");
	else
		USB_HUB_PRINTF("standalone hub\n");

	switch (hub->desc.wHubCharacteristics & HUB_CHAR_OCPM) {
	case 0x00:
		USB_HUB_PRINTF("global over-current protection\n");
		break;
	case 0x08:
		USB_HUB_PRINTF("individual port over-current protection\n");
		break;
	case 0x10:
	case 0x18:
		USB_HUB_PRINTF("no over-current protection\n");
		break;
	}

	USB_HUB_PRINTF("power on to power good time: %dms\n",
			descriptor->bPwrOn2PwrGood * 2);
	USB_HUB_PRINTF("hub controller current requirement: %dmA\n",
			descriptor->bHubContrCurrent);

	for (i = 0; i < dev->maxchild; i++)
		USB_HUB_PRINTF("port %d is%s removable\n", i + 1,
			hub->desc.DeviceRemovable[(i + 1) / 8] & \
					   (1 << ((i + 1) % 8)) ? " not" : "");

	if (sizeof(struct usb_hub_status) > USB_BUFSIZ) {
		USB_HUB_PRINTF("%s: failed to get Status - " \
				"too long: %d\n", __func__, descriptor->bLength);
		return -1;
	}

	if (usb_get_hub_status(dev, buffer) < 0) {
		USB_HUB_PRINTF("%s: failed to get Status %lX\n", __func__,
				dev->status);
		return -1;
	}

	hubsts = (struct usb_hub_status *)buffer;
	USB_HUB_PRINTF("get_hub_status returned status %X, change %X\n",
			le16_to_cpu(hubsts->wHubStatus),
			le16_to_cpu(hubsts->wHubChange));
	USB_HUB_PRINTF("local power source is %s\n",
		(le16_to_cpu(hubsts->wHubStatus) & HUB_STATUS_LOCAL_POWER) ? \
		"lost (inactive)" : "good");
	USB_HUB_PRINTF("%sover-current condition exists\n",
		(le16_to_cpu(hubsts->wHubStatus) & HUB_STATUS_OVERCURRENT) ? \
		"" : "no ");
	usb_hub_power_on(hub);

	for (i = 0; i < dev->maxchild; i++) {
		struct usb_port_status portsts;
		unsigned short portstatus, portchange;

		if (usb_get_port_status(dev, i + 1, &portsts) < 0) {
			USB_HUB_PRINTF("get_port_status failed\n");
			continue;
		}

		portstatus = le16_to_cpu(portsts.wPortStatus);
		portchange = le16_to_cpu(portsts.wPortChange);
		USB_HUB_PRINTF("Port %d Status %X Change %X\n",
				i + 1, portstatus, portchange);

		if (portchange & USB_PORT_STAT_C_CONNECTION) {
			USB_HUB_PRINTF("port %d connection change\n", i + 1);
			usb_hub_port_connect_change(dev, i);
		}
		if (portchange & USB_PORT_STAT_C_ENABLE) {
			USB_HUB_PRINTF("port %d enable change, status %x\n",
					i + 1, portstatus);
			usb_clear_port_feature(dev, i + 1,
						USB_PORT_FEAT_C_ENABLE);

			/* EM interference sometimes causes bad shielded USB
			 * devices to be shutdown by the hub, this hack enables
			 * them again. Works at least with mouse driver */
			if (!(portstatus & USB_PORT_STAT_ENABLE) &&
			     (portstatus & USB_PORT_STAT_CONNECTION) &&
			     ((dev->children[i]))) {
				USB_HUB_PRINTF("already running port %i "  \
						"disabled by hub (EMI?), " \
						"re-enabling...\n", i + 1);
					usb_hub_port_connect_change(dev, i);
			}
		}
		if (portstatus & USB_PORT_STAT_SUSPEND) {
			USB_HUB_PRINTF("port %d suspend change\n", i + 1);
			usb_clear_port_feature(dev, i + 1,
						USB_PORT_FEAT_SUSPEND);
		}

		if (portchange & USB_PORT_STAT_C_OVERCURRENT) {
			USB_HUB_PRINTF("port %d over-current change\n", i + 1);
			usb_clear_port_feature(dev, i + 1,
						USB_PORT_FEAT_C_OVER_CURRENT);
			usb_hub_power_on(hub);
		}

		if (portchange & USB_PORT_STAT_C_RESET) {
			USB_HUB_PRINTF("port %d reset change\n", i + 1);
			usb_clear_port_feature(dev, i + 1,
						USB_PORT_FEAT_C_RESET);
		}
	} /* end for i all ports */

	return 0;
}

static int usb_hub_probe(struct usb_device *dev, int ifnum)
{
	struct usb_interface_descriptor *iface;
	struct usb_endpoint_descriptor *ep;
	int ret;

	iface = &dev->config.if_desc[ifnum];
	/* Is it a hub? */
	if (iface->bInterfaceClass != USB_CLASS_HUB)
		return 0;
	/* Some hubs have a subclass of 1, which AFAICT according to the */
	/*  specs is not defined, but it works */
	if ((iface->bInterfaceSubClass != 0) &&
	    (iface->bInterfaceSubClass != 1))
		return 0;
	/* Multiple endpoints? What kind of mutant ninja-hub is this? */
	if (iface->bNumEndpoints != 1)
		return 0;
	ep = &iface->ep_desc[0];
	/* Output endpoint? Curiousier and curiousier.. */
	if (!(ep->bEndpointAddress & USB_DIR_IN))
		return 0;
	/* If it's not an interrupt endpoint, we'd better punt! */
	if ((ep->bmAttributes & 3) != 3)
		return 0;
	/* We found a hub */
	USB_HUB_PRINTF("USB hub found\n");
	ret = usb_hub_configure(dev);
	return ret;
}

int usb_driver_register(struct usb_driver *drv)
{
	drv->driver.name = drv->name;
	drv->driver.bus = &usb_bus_type;
	return register_driver(&drv->driver);
}

/* returns 0 if no match, 1 if match */
static int usb_match_device(struct usb_device *dev, const struct usb_device_id *id)
{
	if ((id->match_flags & USB_DEVICE_ID_MATCH_VENDOR) &&
	    id->idVendor != le16_to_cpu(dev->descriptor->idVendor))
		return 0;

	if ((id->match_flags & USB_DEVICE_ID_MATCH_PRODUCT) &&
	    id->idProduct != le16_to_cpu(dev->descriptor->idProduct))
		return 0;

	return 1;
}


/* returns 0 if no match, 1 if match */
static int usb_match_one_id(struct usb_device *usbdev,
		     const struct usb_device_id *id)
{
	int ifno;

	/* proc_connectinfo in devio.c may call us with id == NULL. */
	if (id == NULL)
		return 0;

	if (!usb_match_device(usbdev, id))
		return 0;

	/* The interface class, subclass, and protocol should never be
	 * checked for a match if the device class is Vendor Specific,
	 * unless the match record specifies the Vendor ID. */
	if (usbdev->descriptor->bDeviceClass == USB_CLASS_VENDOR_SPEC &&
			!(id->match_flags & USB_DEVICE_ID_MATCH_VENDOR) &&
			(id->match_flags & USB_DEVICE_ID_MATCH_INT_INFO))
		return 0;

	if ( (id->match_flags & USB_DEVICE_ID_MATCH_INT_INFO) ) {
		/* match any interface */
		for (ifno=0; ifno<usbdev->config.no_of_if; ifno++) {
			struct usb_interface_descriptor *intf;
			intf = &usbdev->config.if_desc[ifno];

			if ((id->match_flags & USB_DEVICE_ID_MATCH_INT_CLASS) &&
			    (id->bInterfaceClass != intf->bInterfaceClass))
				continue;

			if ((id->match_flags & USB_DEVICE_ID_MATCH_INT_SUBCLASS) &&
			    (id->bInterfaceSubClass != intf->bInterfaceSubClass))
				continue;

			if ((id->match_flags & USB_DEVICE_ID_MATCH_INT_PROTOCOL) &&
			    (id->bInterfaceProtocol != intf->bInterfaceProtocol))
				continue;
			break;
		}
		if (ifno >= usbdev->config.no_of_if)
			return 0;
	}

	return 1;
}
EXPORT_SYMBOL(usb_match_one_id);

static const struct usb_device_id *usb_match_id(struct usb_device *usbdev,
					 const struct usb_device_id *id)
{
	/* proc_connectinfo in devio.c may call us with id == NULL. */
	if (id == NULL)
		return NULL;

	/* It is important to check that id->driver_info is nonzero,
	   since an entry that is all zeroes except for a nonzero
	   id->driver_info is the way to create an entry that
	   indicates that the driver want to examine every
	   device and interface. */
	for (; id->idVendor || id->idProduct || id->bDeviceClass ||
	       id->bInterfaceClass || id->driver_info; id++) {
		if (usb_match_one_id(usbdev, id))
			return id;
	}

	return NULL;
}
EXPORT_SYMBOL(usb_match_id);

static int usb_match(struct device_d *dev, struct driver_d *drv)
{
	struct usb_device *usbdev = container_of(dev, struct usb_device, dev);
	struct usb_driver *usbdrv = container_of(dev->driver, struct usb_driver, driver);
	const struct usb_device_id *id;

	debug("matching: 0x%04x 0x%04x\n", usbdev->descriptor->idVendor,
			usbdev->descriptor->idProduct);

	id = usb_match_id(usbdev, usbdrv->id_table);
	if (id) {
		debug("match: 0x%04x 0x%04x\n", id->idVendor, id->idProduct);
		return 0;
	}
	return 1;
}

static int usb_probe(struct device_d *dev)
{
	struct usb_device *usbdev = container_of(dev, struct usb_device, dev);
	struct usb_driver *usbdrv = container_of(dev->driver, struct usb_driver, driver);
	const struct usb_device_id *id;

	id = usb_match_id(usbdev, usbdrv->id_table);

	return usbdrv->probe(usbdev, id);
}

static void usb_remove(struct device_d *dev)
{
	struct usb_device *usbdev = container_of(dev, struct usb_device, dev);
	struct usb_driver *usbdrv = container_of(dev->driver, struct usb_driver, driver);

	usbdrv->disconnect(usbdev);
}

struct bus_type usb_bus_type = {
	.name	= "usb",
	.match	= usb_match,
	.probe	= usb_probe,
	.remove	= usb_remove,
};

static int usb_bus_init(void)
{
	return bus_register(&usb_bus_type);
}
pure_initcall(usb_bus_init);
