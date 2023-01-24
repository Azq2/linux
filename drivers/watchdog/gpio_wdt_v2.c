// SPDX-License-Identifier: GPL-2.0-or-later
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>

#define SOFT_TIMEOUT_MIN	1
#define SOFT_TIMEOUT_DEF	60

struct gpio_wdt_v2_priv {
	struct gpio_desc *gpiod;
	bool state;
	bool always_running;
	struct watchdog_device wdd;
};

static int gpio_wdt_v2_ping(struct watchdog_device *wdd) {
	struct gpio_wdt_v2_priv *priv = watchdog_get_drvdata(wdd);
	priv->state = !priv->state;
	gpiod_set_value(priv->gpiod, priv->state);
	return 0;
}

static int gpio_wdt_v2_start(struct watchdog_device *wdd) {
	struct gpio_wdt_v2_priv *priv = watchdog_get_drvdata(wdd);
	
	if (gpiod_get_direction(priv->gpiod) == 0) {
		priv->state = gpiod_get_value(priv->gpiod);
	} else {
		priv->state = 0;
		gpiod_direction_output(priv->gpiod, priv->state);
	}
	
	set_bit(WDOG_HW_RUNNING, &wdd->status);
	
	return gpio_wdt_v2_ping(wdd);
}

static int gpio_wdt_v2_stop(struct watchdog_device *wdd) {
	set_bit(WDOG_HW_RUNNING, &wdd->status);
	return 0;
}

static const struct watchdog_info gpio_wdt_v2_ident = {
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity	= "GPIO Watchdog V2",
};

static const struct watchdog_ops gpio_wdt_v2_ops = {
	.owner		= THIS_MODULE,
	.start		= gpio_wdt_v2_start,
	.stop		= gpio_wdt_v2_stop,
	.ping		= gpio_wdt_v2_ping,
};

static int gpio_wdt_v2_probe(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct gpio_wdt_v2_priv *priv;
	u32 min_time, max_time;
	int ret;
	
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	
	platform_set_drvdata(pdev, priv);
	
	priv->gpiod = devm_gpiod_get(dev, NULL, 0);
	if (IS_ERR(priv->gpiod))
		return PTR_ERR(priv->gpiod);
	
	ret = of_property_read_u32(np, "wdt-min-time", &min_time);
	if (ret)
		return ret;
	
	ret = of_property_read_u32(np, "wdt-max-time", &max_time);
	if (ret)
		return ret;
	
	watchdog_set_drvdata(&priv->wdd, priv);
	
	priv->wdd.info			= &gpio_wdt_v2_ident;
	priv->wdd.ops			= &gpio_wdt_v2_ops;
	priv->wdd.parent		= dev;
	priv->wdd.min_timeout	= SOFT_TIMEOUT_MIN;
	priv->wdd.timeout		= SOFT_TIMEOUT_DEF;
	
	priv->wdd.max_hw_heartbeat_ms = max_time;
	priv->wdd.min_hw_heartbeat_ms = min_time;
	
	watchdog_init_timeout(&priv->wdd, 0, dev);
	watchdog_set_nowayout(&priv->wdd, true);
	watchdog_stop_on_reboot(&priv->wdd);
	gpio_wdt_v2_start(&priv->wdd);
	
	return devm_watchdog_register_device(dev, &priv->wdd);
}

static const struct of_device_id gpio_wdt_v2_dt_ids[] = {
	{ .compatible = "linux,wdt-gpio-v2", },
	{ }
};
MODULE_DEVICE_TABLE(of, gpio_wdt_v2_dt_ids);

static struct platform_driver gpio_wdt_v2_driver = {
	.driver	= {
		.name		= "gpio-wdt-v2",
		.of_match_table	= gpio_wdt_v2_dt_ids,
	},
	.probe	= gpio_wdt_v2_probe,
};

static int __init gpio_wdt_v2_init(void) {
	return platform_driver_register(&gpio_wdt_v2_driver);
}
arch_initcall(gpio_wdt_v2_init);

MODULE_DESCRIPTION("GPIO Watchdog V2");
MODULE_LICENSE("GPL");
