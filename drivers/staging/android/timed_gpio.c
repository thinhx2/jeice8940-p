/* drivers/misc/timed_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/gpio.h>

#ifdef CONFIG_TOWA_PRODUCT
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#endif
#endif
#include "timed_output.h"
#include "timed_gpio.h"


struct timed_gpio_data {
	struct timed_output_dev dev;
	struct hrtimer timer;
	spinlock_t lock;
	unsigned gpio;
	int max_timeout;
	u8 active_low;
};

static enum hrtimer_restart gpio_timer_func(struct hrtimer *timer)
{
	struct timed_gpio_data *data =
		container_of(timer, struct timed_gpio_data, timer);

	gpio_direction_output(data->gpio, data->active_low ? 1 : 0);
	return HRTIMER_NORESTART;
}

static int gpio_get_time(struct timed_output_dev *dev)
{
	struct timed_gpio_data *data;
	struct timeval t;

	data = container_of(dev, struct timed_gpio_data, dev);

	if (!hrtimer_active(&data->timer))
		return 0;

	t = ktime_to_timeval(hrtimer_get_remaining(&data->timer));

	return t.tv_sec * 1000 + t.tv_usec / 1000;
}

static void gpio_enable(struct timed_output_dev *dev, int value)
{
	struct timed_gpio_data	*data =
		container_of(dev, struct timed_gpio_data, dev);
	unsigned long	flags;

	spin_lock_irqsave(&data->lock, flags);

	/* cancel previous timer and set GPIO according to value */
	hrtimer_cancel(&data->timer);
	gpio_direction_output(data->gpio, data->active_low ? !value : !!value);

	if (value > 0) {
		if (value > data->max_timeout)
			value = data->max_timeout;

		hrtimer_start(&data->timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&data->lock, flags);
}

#ifdef CONFIG_TOWA_PRODUCT
#ifdef CONFIG_OF
static struct timed_gpio_platform_data *
timed_gpio_get_devtree_pdata(struct device *dev)
{
	struct device_node *node = NULL;
	struct device_node *pp = NULL;
	struct timed_gpio_platform_data *pdata = NULL;
	struct timed_gpio *new_gpio =NULL;
	int error = EPROBE_DEFER;
	int gpio_count = 0;
	int i = 0;

	node = dev->of_node;
	if (!node){
		dev_err(dev,"Failed to get node\n");
		return ERR_PTR(-ENODEV);
	}

	gpio_count = of_get_child_count(node);
	if (gpio_count == 0){
		dev_err(dev,"Failed to get gpio_count\n");
		return ERR_PTR(-ENODEV);
	}

	pdata = devm_kzalloc(dev,
			sizeof(*pdata) + gpio_count * sizeof(*new_gpio),
			GFP_KERNEL);
	if (!pdata){
		dev_err(dev,"Failed to get pdata\n");
		return ERR_PTR(-ENOMEM);
	}
	pdata->gpios = (struct timed_gpio *)(pdata + 1);
	pdata->num_gpios = gpio_count;

	for_each_child_of_node(node, pp) {
		int gpio;
		enum of_gpio_flags flags;

		if (!of_find_property(pp, "gpios", NULL)) {
			pdata->num_gpios--;
			dev_warn(dev, "Can't find gpios\n");
			continue;
		}

		gpio = of_get_gpio_flags(pp, 0, &flags);
		if (gpio < 0) {
			error = gpio;
			if (error != -EPROBE_DEFER)
				dev_err(dev,"Failed to get gpio flags, error: %d\n",
					error);
			return ERR_PTR(error);
		}

		new_gpio = &pdata->gpios[i++];
		new_gpio->gpio = gpio;
		new_gpio->active_low = flags & OF_GPIO_ACTIVE_LOW;

		if (of_property_read_u32(pp, "max_timeout", &new_gpio->max_timeout)) {
			dev_err(dev, "Time_gpio without max_timeout: 0x%x\n",
				new_gpio->gpio);
			return ERR_PTR(-EINVAL);
		}

		new_gpio->name = of_get_property(pp, "label", NULL);
	}

	if (pdata->num_gpios == 0){
		dev_err(dev,"Failed to get num_gpios 0\n");
		return ERR_PTR(-EINVAL);
	}
	return pdata;
}
static const struct of_device_id timed_gpio_of_match[] = {
	{ .compatible = "timed-gpio", },
	{ },
};
MODULE_DEVICE_TABLE(of, timed_gpio_of_match);
#endif
#endif
static int timed_gpio_probe(struct platform_device *pdev)
{
#ifdef CONFIG_TOWA_PRODUCT
#ifdef CONFIG_OF
	struct device *dev = &pdev->dev;
#endif
#endif
	struct timed_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct timed_gpio *cur_gpio;
	struct timed_gpio_data *gpio_data, *gpio_dat;
	int i, ret;
#ifdef CONFIG_TOWA_PRODUCT
#ifdef CONFIG_OF
	pdata = timed_gpio_get_devtree_pdata(dev);
#endif
#endif

	if (!pdata)
		return -EBUSY;

	gpio_data = devm_kzalloc(&pdev->dev,
			sizeof(struct timed_gpio_data) * pdata->num_gpios,
			GFP_KERNEL);
	if (!gpio_data)
		return -ENOMEM;

	for (i = 0; i < pdata->num_gpios; i++) {
		cur_gpio = &pdata->gpios[i];
		gpio_dat = &gpio_data[i];

		hrtimer_init(&gpio_dat->timer, CLOCK_MONOTONIC,
				HRTIMER_MODE_REL);
		gpio_dat->timer.function = gpio_timer_func;
		spin_lock_init(&gpio_dat->lock);

		gpio_dat->dev.name = cur_gpio->name;
		gpio_dat->dev.get_time = gpio_get_time;
		gpio_dat->dev.enable = gpio_enable;
		ret = gpio_request(cur_gpio->gpio, cur_gpio->name);
		if (ret < 0)
			goto err_out;
		ret = timed_output_dev_register(&gpio_dat->dev);
		if (ret < 0) {
			gpio_free(cur_gpio->gpio);
			goto err_out;
		}

		gpio_dat->gpio = cur_gpio->gpio;
		gpio_dat->max_timeout = cur_gpio->max_timeout;
		gpio_dat->active_low = cur_gpio->active_low;
		gpio_direction_output(gpio_dat->gpio, gpio_dat->active_low);
	}

	platform_set_drvdata(pdev, gpio_data);

	return 0;

err_out:
	while (--i >= 0) {
		timed_output_dev_unregister(&gpio_data[i].dev);
		gpio_free(gpio_data[i].gpio);
	}

	return ret;
}

static int timed_gpio_remove(struct platform_device *pdev)
{
	struct timed_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct timed_gpio_data *gpio_data = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < pdata->num_gpios; i++) {
		timed_output_dev_unregister(&gpio_data[i].dev);
		gpio_free(gpio_data[i].gpio);
	}

	return 0;
}

static struct platform_driver timed_gpio_driver = {
	.probe		= timed_gpio_probe,
	.remove		= timed_gpio_remove,
	.driver		= {
		.name		= TIMED_GPIO_NAME,
		.owner		= THIS_MODULE,
#ifdef CONFIG_TOWA_PRODUCT
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(timed_gpio_of_match),
#endif
#endif
	},
};

module_platform_driver(timed_gpio_driver);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("timed gpio driver");
MODULE_LICENSE("GPL");
