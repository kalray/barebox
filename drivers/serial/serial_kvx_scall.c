/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2018 Kalray Inc.
 */

#include <init.h>
#include <console.h>
#include <linux/string.h>

#define SERIAL_MAX_BUFFERING	128

struct kvx_scall_console_data {
	char buffer[SERIAL_MAX_BUFFERING];
	int cur_off;
	bool tx_only;
	struct console_device cdev;
};

struct pollfd {
	int fd;			/* File descriptor to poll.  */
	short int events;	/* Types of events poller cares about.  */
	short int revents;	/* Types of events that actually occurred.  */
};

static int kvx_scall_console_poll(struct pollfd  *fds,
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

static int kvx_scall_console_inbyte(char *buf)
{
	register char *arg1 asm("r0") = buf;

	asm volatile ("scall 0xfda\n\t;;"
			: "+r"(arg1)
			: : "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "memory");
	return (long) arg1;
}

static int kvx_scall_puts(const char *s, size_t nbytes)
{
	register const char *arg1 asm("r0") = s;
	register unsigned arg2 asm ("r1") = nbytes;
	register unsigned fd asm ("r2") = 1;

	asm volatile ("scall 0xffe\n\t;;"
			: : "r"(arg1), "r"(arg2), "r"(fd)
			: "r3", "r4", "r5", "r6", "r7", "r8", "memory");
	return nbytes;
}

static void kvx_scall_console_flush(struct console_device *cdev)
{
	struct kvx_scall_console_data *data =
		container_of(cdev, struct kvx_scall_console_data, cdev);

	if (data->cur_off > 0) {
		data->buffer[data->cur_off] = '\0';
		kvx_scall_puts(data->buffer, data->cur_off);
		data->cur_off = 0;
	}
}

static int kvx_scall_console_puts(struct console_device *cdev, const char *s,
			   size_t nbytes)
{
	/* Flush any leftovers from previous putc */
	kvx_scall_console_flush(cdev);

	return kvx_scall_puts(s, nbytes);
}

static void kvx_scall_console_putc(struct console_device *cdev, char c)
{
	struct kvx_scall_console_data *data =
		container_of(cdev, struct kvx_scall_console_data, cdev);

	data->buffer[data->cur_off] = c;
	data->cur_off++;

	if (c == '\n' || data->cur_off == SERIAL_MAX_BUFFERING - 1)
		kvx_scall_console_flush(cdev);
}

static int kvx_scall_console_getc(struct console_device *cdev)
{
	char c;

	kvx_scall_console_inbyte(&c);

	return c;
}

static int kvx_scall_console_tstc(struct console_device *cdev)
{
	struct pollfd fds;
	int poll_ret;
	struct kvx_scall_console_data *data =
		container_of(cdev, struct kvx_scall_console_data, cdev);

	if (data->tx_only)
		return 0;

	fds.fd = 0;
	fds.events = 0x001;
	fds.revents = 0;

	poll_ret = kvx_scall_console_poll(&fds, 1, 0);
	if (poll_ret <= 0)
		return 0;

	if ((fds.revents & 0x001) == 0)
		return 0;

	return 1;
}

static int kvx_scall_console_probe(struct device_d *dev)
{
	struct console_device *cdev;
	struct kvx_scall_console_data *data;

	data = xzalloc(sizeof(struct kvx_scall_console_data));
	data->tx_only = false;

	if (of_property_read_bool(dev->device_node, "kalray,serial-tx-only"))
		data->tx_only = true;

	cdev = &data->cdev;
	cdev->dev = dev;
	cdev->putc = kvx_scall_console_putc;
	cdev->puts = kvx_scall_console_puts;
	cdev->tstc = kvx_scall_console_tstc;
	cdev->getc = kvx_scall_console_getc;
	cdev->flush = kvx_scall_console_flush;
	cdev->devname = "scall";

	console_register(&data->cdev);

	return 0;
}

static __maybe_unused struct of_device_id kvx_scall_console[] = {
	{
		.compatible = "kalray,kvx-scall-console",
	}, {
	}
};

static struct driver_d kvx_scall_console_driver = {
	.name  = "kvx_scall_console",
	.probe = kvx_scall_console_probe,
	.of_compatible = DRV_OF_COMPAT(kvx_scall_console),
};

console_platform_driver(kvx_scall_console_driver);

