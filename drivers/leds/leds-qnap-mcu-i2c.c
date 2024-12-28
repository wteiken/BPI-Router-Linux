// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for i2c driven LEDs found on QNAP MCU devices
 *
 * Copyright (C) 2024 Wilfried Teiken <wteiken@teiken.org>
 */

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <uapi/linux/uleds.h>

enum qnap_mcu_led_mode {
  QNAP_MCU_LED_ON = 0x10,
  QNAP_MCU_LED_OFF = 0x30,
  QNAP_MCU_LED_BLINK_FAST = 0x50,
  QNAP_MCU_LED_BLINK_SLOW = 0x90,
  QNAP_MCU_LED_BLINK_MEDIUM = 0xd0,
};

struct qnap_mcu_led {
  struct i2c_client *client;
  struct led_classdev cdev;
  char name[LED_MAX_NAME_SIZE];
  u8 num;
  u8 mode;
};

static int set_led_state(const struct qnap_mcu_led* led) {
  return i2c_smbus_write_byte_data(led->client, led->mode | led->num, 0xff);
}

static inline struct qnap_mcu_led *
		cdev_to_qnap_mcu_led(struct led_classdev *led_cdev)
{
  return container_of(led_cdev, struct qnap_mcu_led, cdev);
}

static int qnap_mcu_led_set(struct led_classdev *led_cdev,
			    enum led_brightness brightness)
{
  struct qnap_mcu_led *led = cdev_to_qnap_mcu_led(led_cdev);
  /* Don't disturb a possible set blink-mode if LED stays on */
  if (brightness != 0 && led->mode >= QNAP_MCU_LED_BLINK_FAST)
    return 0;

  led->mode = brightness ? QNAP_MCU_LED_ON : QNAP_MCU_LED_OFF;

  return set_led_state(led);
}

static int qnap_mcu_led_blink_set(struct led_classdev *led_cdev,
					unsigned long *delay_on,
					unsigned long *delay_off)
{
  struct qnap_mcu_led *led = cdev_to_qnap_mcu_led(led_cdev);

  /* LED is off, nothing to do */
  if (led->mode == QNAP_MCU_LED_OFF)
    return 0;

  *delay_on = 250;
  *delay_off = 250;
  // TODO what about other blink modes?
  led->mode = QNAP_MCU_LED_BLINK_SLOW;

  return set_led_state(led);
}

static int qnap_mcu_register_led(struct device *dev, struct i2c_client *client, const char* name, int num)
{
  struct qnap_mcu_led *led;
  int ret;

  led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
  if (!led)
    return -ENOMEM;

  led->client = client;
  led->mode = QNAP_MCU_LED_OFF;
  led->num = num;
  strncpy(led->name, name, sizeof(led->name));
  led->name[sizeof(led->name)-1]=0;
  led->cdev.name = led->name;
  led->cdev.brightness_set_blocking = qnap_mcu_led_set;
  led->cdev.blink_set = qnap_mcu_led_blink_set;
  led->cdev.brightness = 0;
  led->cdev.max_brightness = 1;

  ret = devm_led_classdev_register(dev, &led->cdev);
  if (ret)
      return ret;
  
  return qnap_mcu_led_set(&led->cdev, 0);
}

static int qnap_mcu_i2c_leds_probe(struct i2c_client *client)
{
  int num_sata_drives;
  u32 sata_drives[32];
  int num_m2_drives;
  u32 m2_drives[32];
  int ret;


  num_sata_drives = device_property_count_u32(&client->dev, "sata-drives");
  if (num_sata_drives < 0 || num_sata_drives > 32)
    return dev_err_probe(&client->dev, num_sata_drives ? : -EINVAL,
			 "Invalid/missing 'sata-drives' property\n");
  if ((ret = device_property_read_u32_array(&client->dev, "sata-drives",
					     sata_drives, num_sata_drives)) != 0) 
    return dev_err_probe(&client->dev, ret, "Failed to read 'sata-drives' property\n");
  
  for (int i = 0; i < num_sata_drives; i++) {
    char name_buf[LED_MAX_NAME_SIZE];
    snprintf(name_buf, sizeof(name_buf), "sata-%d:green:indicator", i+1);
    ret = qnap_mcu_register_led(&client->dev, client, name_buf, 2*sata_drives[i]);
    if (ret)
      return dev_err_probe(&client->dev, ret,
			   "failed to register green sata LED %d\n", i);
    snprintf(name_buf, sizeof(name_buf), "sata-%d:red:indicator", i+1);
    ret = qnap_mcu_register_led(&client->dev, client, name_buf, 2*sata_drives[i]+1);
    if (ret)
      return dev_err_probe(&client->dev, ret,
			   "failed to register red sata LED %d\n", i);
  }

  num_m2_drives = device_property_count_u32(&client->dev, "m2-drives");
  if (num_m2_drives < 0 || num_m2_drives > 32)
    return dev_err_probe(&client->dev, num_m2_drives ? : -EINVAL,
			 "Invalid/missing 'm2-drives' property\n");
  if ((ret = device_property_read_u32_array(&client->dev, "m2-drives",
					     m2_drives, num_m2_drives)) != 0) 
    return dev_err_probe(&client->dev, ret, "Failed to read 'm2-drives' property\n");
  
  for (int i = 0; i < num_m2_drives; i++) {
    char name_buf[LED_MAX_NAME_SIZE];
    snprintf(name_buf, sizeof(name_buf), "m2-%d:green:indicator", i+1);
    ret = qnap_mcu_register_led(&client->dev, client, name_buf, 2*m2_drives[i]);
    if (ret)
      return dev_err_probe(&client->dev, ret,
			   "failed to register green m2 LED %d\n", i);
    snprintf(name_buf, sizeof(name_buf), "m2-%d:red:indicator", i+1);
    ret = qnap_mcu_register_led(&client->dev, client, name_buf, 2*m2_drives[i]+1);
    if (ret)
      return dev_err_probe(&client->dev, ret,
			   "failed to register red m2 LED %d\n", i);
  }

  return 0;
}

static void qnap_mcu_i2c_leds_remove(struct i2c_client *client)
{
}

static const struct of_device_id of_qnap_i2c_leds_match[] = {
	{ .compatible = "qnap,ts435xeu-mcu" },
	{},
};

MODULE_DEVICE_TABLE(of, of_qnap_i2c_leds_match);

static struct i2c_driver qnap_mcu_i2c_driver = {
	.driver = {
		.name	= "qnap-mcu-i2c-leds",
		.of_match_table = of_qnap_i2c_leds_match,
	},
	.probe		= qnap_mcu_i2c_leds_probe,
	.remove		= qnap_mcu_i2c_leds_remove,
};

module_i2c_driver(qnap_mcu_i2c_driver);

MODULE_ALIAS("platform:qnap-mcu-i2c-leds");
MODULE_AUTHOR("Wilfried Teiken <wteiken@teiken.org>");
MODULE_DESCRIPTION("QNAP MCU i2c LEDs driver");
MODULE_LICENSE("GPL");
