/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2018 Kalray Inc.
 */

#include <init.h>
#include <console.h>

#define SERIAL_MAX_BUFFERING	128

struct k1c_scall_console_data {
	char buffer[SERIAL_MAX_BUFFERING];
	int cur_off;
	struct console_device cdev;
};

struct pollfd {
	int fd;			/* File descriptor to poll.  */
	short int events;	/* Types of events poller cares about.  */
	short int revents;	/* Types of events that actually occurred.  */
};

static int k1c_scall_console_poll(struct pollfd  *fds,
				unsigned int nfds, unsigned int timeout)
{
	register struct pollfd *arg1 asm("r0") = fds;
	register unsigned int arg2 asm ("r1") = nfds;
	register unsigned int arg3 asm ("r2") = timeout;

	asm volatile ("scall 0xfdb\n\t;;"
			: "+r"(arg1)
			: "r"(arg2), "r"(arg3)
			: "r3", "r4", "r5", "r6", "r7", "r8", "memory");
	return (long) arg1;
}

static int k1c_scall_console_inbyte(char *buf)
{
	register char *arg1 asm("r0") = buf;

	asm volatile ("scall 0xfda\n\t;;"
			: "+r"(arg1)
			: : "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "memory");
	return (long) arg1;
}

static int k1c_scall_console_puts(struct console_device *cdev, const char *s)
{
	int len = strlen(s);
	register const char *arg1 asm("r0") = s;
	register unsigned arg2 asm ("r1") = len;

	asm volatile ("scall 0xffe\n\t;;"
			: : "r"(arg1), "r"(arg2)
			: "r2", "r3", "r4", "r5", "r6", "r7", "r8", "memory");
	return len;
}

static void k1c_scall_console_putc(struct console_device *cdev, char c)
{
	struct k1c_scall_console_data *data =
		container_of(cdev, struct k1c_scall_console_data, cdev);
	
	data->buffer[data->cur_off] = c;
	data->cur_off++;

	if (c == '\n' || data->cur_off == SERIAL_MAX_BUFFERING - 1) {
		data->buffer[data->cur_off] = '\0';
		data->cur_off = 0;
		k1c_scall_console_puts(cdev, data->buffer);
	}
}

static int k1c_scall_console_getc(struct console_device *cdev)
{
	char c;

	k1c_scall_console_inbyte(&c);

	return c;
}

static int k1c_scall_console_tstc(struct console_device *cdev)
{
	struct pollfd fds;
	int poll_ret;

	fds.fd = 0;
	fds.events = 0x001;
	fds.revents = 0;

	poll_ret = k1c_scall_console_poll(&fds, 1, 0);
	if (poll_ret <= 0)
		return 0;

	if ((fds.revents & 0x001) == 0)
		return 0;

	return 1;
}

static int k1c_scall_console_probe(struct device_d *dev)
{
	struct console_device *cdev;
	struct k1c_scall_console_data *data;

	data = xzalloc(sizeof(struct k1c_scall_console_data));

	cdev = &data->cdev;
	cdev->dev = dev;
	cdev->putc = k1c_scall_console_putc;
	cdev->puts = k1c_scall_console_puts;
	cdev->tstc = k1c_scall_console_tstc;
	cdev->getc = k1c_scall_console_getc;
	cdev->devname = "scall";

	console_register(&data->cdev);

	return 0;
}

static __maybe_unused struct of_device_id k1c_scall_console[] = {
	{
		.compatible = "kalray,k1c-scall-console",
	}, {
	}
};

static struct driver_d k1c_scall_console_driver = {
	.name  = "k1c_scall_console",
	.probe = k1c_scall_console_probe,
	.of_compatible = DRV_OF_COMPAT(k1c_scall_console),
};

console_platform_driver(k1c_scall_console_driver);

