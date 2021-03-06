/*
 * Designware GPIO support functions
 *
 * Copyright (C) 2012 Altera
 * Copyright (C) 2020 kalray
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <common.h>
#include <errno.h>
#include <io.h>
#include <gpio.h>
#include <init.h>
#include <linux/err.h>
#include <pinctrl.h>

#define DW_GPIO_DR 		0x0
#define DW_GPIO_DDR		0x4
#define DW_GPIO_CTL		0x8
#define DW_GPIO_EXT 		0x50
#define DW_GPIO_CONFIG2		0x70
#define DW_GPIO_CONFIG1		0x74

#define DW_GPIO_CONFIG2_WIDTH(val, port)	(((val) >> ((port) * 4) & 0x1f) + 1)
#define DW_GPIO_CONFIG1_NPORTS(val)		(((val) >> 2 & 0x3) + 1)

struct dw_gpio {
	void __iomem *regs;
};

struct dw_gpio_instance {
	struct pinctrl_device pdev;
	struct dw_gpio *parent;
	struct gpio_chip chip;
	u32 gpio_state;		/* GPIO state shadow register */
	u32 gpio_dir;		/* GPIO direction shadow register */
};

static inline struct dw_gpio_instance *to_dw_gpio(struct gpio_chip *gc)
{
	return container_of(gc, struct dw_gpio_instance, chip);
}

static int dw_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct dw_gpio_instance *chip = to_dw_gpio(gc);
	struct dw_gpio *parent = chip->parent;

	return (readl(parent->regs + DW_GPIO_EXT) >> offset) & 1;
}

static void dw_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct dw_gpio_instance *chip = to_dw_gpio(gc);
	struct dw_gpio *parent = chip->parent;
	u32 data_reg;

	data_reg = readl(parent->regs + DW_GPIO_DR);
	data_reg = (data_reg & ~(1<<offset)) | (value << offset);
	writel(data_reg, parent->regs + DW_GPIO_DR);
}

static int dw_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct dw_gpio_instance *chip = to_dw_gpio(gc);
	struct dw_gpio *parent = chip->parent;
	u32 gpio_ddr;

	/* Set pin as input, assumes software controlled IP */
	gpio_ddr = readl(parent->regs + DW_GPIO_DDR);
	gpio_ddr &= ~(1 << offset);
	writel(gpio_ddr, parent->regs + DW_GPIO_DDR);

	return 0;
}

static int dw_gpio_direction_output(struct gpio_chip *gc,
		unsigned offset, int value)
{
	struct dw_gpio_instance *chip = to_dw_gpio(gc);
	struct dw_gpio *parent = chip->parent;
	u32 gpio_ddr;

	dw_gpio_set(gc, offset, value);

	/* Set pin as output, assumes software controlled IP */
	gpio_ddr = readl(parent->regs + DW_GPIO_DDR);
	gpio_ddr |= (1 << offset);
	writel(gpio_ddr, parent->regs + DW_GPIO_DDR);

	return 0;
}

static int dw_gpio_get_direction(struct gpio_chip *gc, unsigned offset)
{
	struct dw_gpio_instance *chip = to_dw_gpio(gc);
	struct dw_gpio *parent = chip->parent;

	return (readl(parent->regs + DW_GPIO_DDR) & (1 << offset)) ?
		GPIOF_DIR_OUT : GPIOF_DIR_IN;
}

static struct gpio_ops dw_gpio_ops = {
	.direction_input = dw_gpio_direction_input,
	.direction_output = dw_gpio_direction_output,
	.get_direction = dw_gpio_get_direction,
	.get = dw_gpio_get,
	.set = dw_gpio_set,
};

static int dw_pinctrl_set_state(struct pinctrl_device *pdev,
				struct device_node *group)
{
	int num_pins = 0, i, ret, pin_id;
	const char *func, *pin;
	u32 gpio_ctl;
	struct dw_gpio_instance *chip = container_of(pdev,
						     struct dw_gpio_instance,
						     pdev);
	struct dw_gpio *parent = chip->parent;

	ret = of_property_read_string(group, "function", &func);
	if (ret) {
		dev_err(pdev->dev, "Missing \"function\" property: %d\n",
			ret);
		return ret;
	}

	num_pins = of_property_count_strings(group, "pins");
	if (!num_pins) {
		dev_err(pdev->dev, "Missing \"pins\" property: %d\n", ret);
		return -EINVAL;
	}

	for (i = 0; i < num_pins; i++) {
		ret = of_property_read_string_index(group, "pins", i, &pin);
		if (ret) {
			dev_err(pdev->dev, "Failed to read pins[%d]: %d\n",
				i, ret);
			return ret;
		}
		if (strncmp(pin, "pin", 3)) {
			dev_err(pdev->dev, "Invalid pin description: %s\n",
				pin);
			return -EINVAL;
		}

		ret = kstrtoint(&pin[3], 10, &pin_id);
		if (ret) {
			dev_err(pdev->dev, "Invalid pin index: %d\n", ret);
			return ret;
		}

		dev_dbg(pdev->dev, "Setting function %s on pin %d\n", func,
			pin_id);

		gpio_ctl = readl(parent->regs + DW_GPIO_CTL);
		if (strcmp(func, "hw") == 0) {
			gpio_ctl |= (1 << pin_id);
		} else if (strcmp(func, "sw") == 0) {
			gpio_ctl &= ~(1 << pin_id);
		} else {
			dev_err(pdev->dev, "Invalid function: %s\n", func);
			return -EINVAL;
		}
		writel(gpio_ctl, parent->regs + DW_GPIO_CTL);
	}

	return 0;
}

static struct pinctrl_ops dw_pinctrl_ops = {
	.set_state = dw_pinctrl_set_state,
};

static int dw_gpio_add_port(struct device_d *dev, struct device_node *node,
			    struct dw_gpio *parent)
{
	struct dw_gpio_instance *chip;
	uint32_t config1, config2;
	int ngpio, ret;

	chip = xzalloc(sizeof(*chip));

	chip->chip.ops = &dw_gpio_ops;
	if (dev->id < 0)
		chip->chip.base = DEVICE_ID_DYNAMIC;
	else
		chip->chip.base = dev->id * 32;

	config2 = readl(parent->regs + DW_GPIO_CONFIG2);
	config1 = readl(parent->regs + DW_GPIO_CONFIG1);
	ngpio = DW_GPIO_CONFIG2_WIDTH(config2, 0);

	if (DW_GPIO_CONFIG1_NPORTS(config1) > 1)
		dev_info(dev, "ignoring ports B-D\n");

	chip->parent = parent;
	chip->chip.ngpio = ngpio;
	chip->chip.dev = add_generic_device("dw-port", DEVICE_ID_DYNAMIC, NULL,
					    dev->resource[0].start,
					    resource_size(&dev->resource[0]),
					    IORESOURCE_MEM, NULL);

	if (!chip->chip.dev) {
		dev_err(dev, "unable to add device\n");
		return -ENODEV;
	}

	chip->chip.dev->device_node = node;

	if (of_get_next_available_child(node, NULL)) {
		chip->pdev.dev = dev;
		chip->pdev.ops = &dw_pinctrl_ops;
		ret = pinctrl_register(&chip->pdev);
		if (ret) {
			dev_dbg(dev, "pinctrl_register failed: (%d)\n", ret);
			return ret;
		}
	}

	ret = gpiochip_add(&chip->chip);
	if (ret)
		return ret;

	dev_dbg(chip->chip.dev, "probed gpiochip with %d gpios, base %d\n",
			chip->chip.ngpio, chip->chip.base);

	return 0;
}

static int dw_gpio_probe(struct device_d *dev)
{
	struct resource *iores;
	struct dw_gpio *gpio;
	struct device_node *node;

	gpio = xzalloc(sizeof(*gpio));

	iores = dev_request_mem_resource(dev, 0);
	if (IS_ERR(iores))
		return PTR_ERR(iores);
	gpio->regs = IOMEM(iores->start);

	for_each_child_of_node(dev->device_node, node)
		dw_gpio_add_port(dev, node, gpio);

	return 0;
}

static __maybe_unused struct of_device_id dwgpio_match[] = {
	{
		.compatible = "snps,dw-apb-gpio",
	}, {
		/* sentinel */
	},
};

static struct driver_d dwgpio_driver = {
	.name = "dw-apb-gpio",
	.probe = dw_gpio_probe,
	.of_compatible = DRV_OF_COMPAT(dwgpio_match),
};

static int __init dwgpio_init(void)
{
	return platform_driver_register(&dwgpio_driver);
}
postcore_initcall(dwgpio_init);
