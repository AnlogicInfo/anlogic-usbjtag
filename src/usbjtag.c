/*
 * This file is part of anlogic-usbjtag project
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2017 Zhiyuan Wan <h@iloli.bid>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

/* A internal function for setting endpoint_address manually, it is pretty ugly, but it could works */
/* I have no idea for better way. So just wait them */
void st_usbfs_ep_setup_core(usbd_device *dev, uint8_t phy_addr, uint8_t ep_addr, uint8_t type,
		uint16_t max_size,
		void (*callback) (usbd_device *usbd_dev,
		uint8_t ep));

int _write(int file, char *ptr, int len);

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0, /* Custom device */
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor =  0x0547,
	.idProduct = 0x1002,
	.bcdDevice = 0x0000,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};


static const struct usb_endpoint_descriptor endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x06,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
},{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x08,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};


static const struct usb_interface_descriptor iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 3,
	.bInterfaceClass = 0xff,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Anlogic",
	"USB-JTAG-Cable"
};

/* usbd_device_handler */
usbd_device *usbd_dev_handler;

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

/* Parameter for JTAG configuration */
uint16_t speed = 0;

/* USB configuration state */
unsigned char configured = 0;

/* USB interrupt handler */
void usb_wakeup_isr(void)
	__attribute__ ((alias ("usb_int_relay")));

void usb_hp_can_tx_isr(void)
	__attribute__ ((alias ("usb_int_relay")));

void usb_lp_can_rx0_isr(void)
	__attribute__ ((alias ("usb_int_relay")));

static void usb_int_relay(void) {
  /* Need to pass a parameter... otherwise just alias it directly. */
  usbd_poll(usbd_dev_handler);
}
/* For bit-banging usage */
int rx_count = 0;
uint8_t rx_buf[512];

/* USB function */
static int usbjtag_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;
	(void)len;

	switch (req->bRequest) {

	}
	return 0;
}

static void usbjtag_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{/* This function provided data transmit routine */
 /* data format: 4bit as 1 frame, bit 2 = TCK, bit 1 means TDI, bit 0 means TMS */
	(void)ep;
	(void)usbd_dev;

	char buf[64];
	int i = 0;
	
	int len = usbd_ep_read_packet(usbd_dev, 0x06, buf, 64);

	if (len) {
		for(i = 0; i < 128; i++)
		{
			uint8_t io_state = 0;
			if(i & 0x01)
				rx_buf[(i >> 1) + rx_count * 64] |= (GPIOA_IDR & 0x01) << 4;
			else
				rx_buf[(i >> 1) + rx_count * 64] =  (GPIOA_IDR & 0x01);
				
			io_state = ((buf[i >> 1] >> ((i & 0x01) ? 4: 0)) & 0x0f) << 1;
			GPIOA_BSRR = io_state + (((~(io_state)) & 0x0f) << 16);
		}
		rx_count ++;
	}
	if(rx_count >= 8)
	{
		while(usbd_ep_write_packet(usbd_dev_handler, 0x82, &rx_buf[0], 64) == 0);
		while(usbd_ep_write_packet(usbd_dev_handler, 0x82, &rx_buf[64], 64) == 0);
		while(usbd_ep_write_packet(usbd_dev_handler, 0x82, &rx_buf[128], 64) == 0);
		while(usbd_ep_write_packet(usbd_dev_handler, 0x82, &rx_buf[192], 64) == 0);
		while(usbd_ep_write_packet(usbd_dev_handler, 0x82, &rx_buf[256], 64) == 0);
		while(usbd_ep_write_packet(usbd_dev_handler, 0x82, &rx_buf[320], 64) == 0);
		while(usbd_ep_write_packet(usbd_dev_handler, 0x82, &rx_buf[384], 64) == 0);
		while(usbd_ep_write_packet(usbd_dev_handler, 0x82, &rx_buf[448], 64) == 0);
		rx_count = 0;
		printf("EP2: 512 bytes\r\n");
	}
	printf("EP6: %d bytes\r\n", len);
	
}

static void usbjtag_data_rx_st(usbd_device *usbd_dev, uint8_t ep)
{/* This function provided, maybe, setting routine */
	(void)ep;
	(void)usbd_dev;

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x07, buf, 64);
	if(len == 2)
		speed = buf[0] | buf[1] << 8;
	printf("EP8: %d bytes\r\n", len);
	rx_count = 0;
	
}

static void usbjtag_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;
	(void)usbd_dev;

	usbd_ep_setup(usbd_dev, 0x06, USB_ENDPOINT_ATTR_BULK, 64, usbjtag_data_rx_cb);
	st_usbfs_ep_setup_core(usbd_dev, 0x07, 0x08, USB_ENDPOINT_ATTR_BULK, 64, usbjtag_data_rx_st);/* TODO: a better way to assign address manually */
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usbjtag_control_request);
	configured = 1;
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_set(GPIOB, GPIO12);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	printf("USB: configuration done\r\n");
}

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++)
			usart_send_blocking(USART1, ptr[i]);
		return i;
	}

	errno = EIO;
	return -1;
}

static void usart_setup(void)
{
	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port B for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 230400);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

int main(void)
{
	int i;
	uint8_t buf[64] = {0xca, 0xfe};

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART1);

	usart_setup();
	
	gpio_clear(GPIOA, GPIO8);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);

	/* JTAG IO setup */
	/* PA3 = TCK, PA2 = TDI, PA1 = TMS*/
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO1 | GPIO2 | GPIO3);
	/* PA0 = TDO */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
			
	usbd_dev_handler = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 2, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev_handler, usbjtag_set_config);

	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	nvic_enable_irq(NVIC_USB_WAKEUP_IRQ);
  
	for (i = 0; i < 0x80000; i++)
		__asm__("nop");

	gpio_set(GPIOA, GPIO8);
	printf("Main entry\r\n");
	i = 0;
	while (1)
	{
		/*if(rx_count >= 7)
		{
			for(i = 0; i < 8; i++)
			{
				int j = 0;
				while(j == 0)
				{
					j = usbd_ep_write_packet(usbd_dev_handler, 0x82, buf, 64);
				}
				printf("EP2: %d bytes\r\n", j);
			}
			rx_count = 0;
		}*/
	}
}
