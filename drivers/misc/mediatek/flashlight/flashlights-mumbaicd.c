/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif
#ifdef OPLUS_FEATURE_CAMERA_COMMON
#include <soc/oplus/system/oplus_project.h>
#endif

#include "flashlight-core.h"
#include "flashlight-dt.h"
#include "flashlight.h"
/* device tree should be defined in flashlight-dt.h */
#ifndef MUMBAICD_DTNAME
#define MUMBAICD_DTNAME "mediatek,flashlights_mumbaicd"
#endif
#ifndef MUMBAICD_DTNAME_I2C
#define MUMBAICD_DTNAME_I2C   "mediatek,strobe_main"
#endif

#define MUMBAICD_NAME "flashlights_mumbaicd"

/* define registers */
#define MUMBAICD_REG_ENABLE           (0x01)
#define MUMBAICD_MASK_ENABLE_LED1     (0x01)
#define MUMBAICD_MASK_ENABLE_LED2     (0x02)
#define MUMBAICD_DISABLE              (0x00)
#define MUMBAICD_TORCH_MODE           (0x08)
#define MUMBAICD_FLASH_MODE           (0x0C)
#define MUMBAICD_ENABLE_LED1          (0x01)
#define MUMBAICD_ENABLE_LED1_TORCH    (0x09)
#define MUMBAICD_ENABLE_LED1_FLASH    (0x0D)
#define MUMBAICD_ENABLE_LED2          (0x02)
#define MUMBAICD_ENABLE_LED2_TORCH    (0x0A)
#define MUMBAICD_ENABLE_LED2_FLASH    (0x0E)

#define MUMBAICD_REG_TORCH_LEVEL_LED1 (0x05)
#define MUMBAICD_REG_FLASH_LEVEL_LED1 (0x03)
#define MUMBAICD_REG_TORCH_LEVEL_LED2 (0x06)
#define MUMBAICD_REG_FLASH_LEVEL_LED2 (0x04)

#define MUMBAICD_REG_TIMING_CONF      (0x08)
#define MUMBAICD_TORCH_RAMP_TIME      (0x10)
#define MUMBAICD_FLASH_TIMEOUT        (0x0F)

#define MUMBAICD_AW36515_SOFT_RESET_ENABLE (0x80)
#define MUMBAICD_AW36515_REG_BOOST_CONFIG (0x07)


/* define channel, level */
#define MUMBAICD_CHANNEL_NUM          2
#define MUMBAICD_CHANNEL_CH1          0
#define MUMBAICD_CHANNEL_CH2          1
/* define level */
#define MUMBAICD_LEVEL_NUM 28
#define MUMBAICD_LEVEL_TORCH 7

#define MUMBAICD_HW_TIMEOUT 400 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(mumbaicd_mutex);
static struct work_struct mumbaicd_work_ch1;
static struct work_struct mumbaicd_work_ch2;

/* define pinctrl */
#define MUMBAICD_PINCTRL_PIN_HWEN 0
#define MUMBAICD_PINCTRL_PINSTATE_LOW 0
#define MUMBAICD_PINCTRL_PINSTATE_HIGH 1
#define MUMBAICD_PINCTRL_STATE_HWEN_HIGH "mumbaicd_hwen_high"
#define MUMBAICD_PINCTRL_STATE_HWEN_LOW  "mumbaicd_hwen_low"

/* define device id */
#define USE_AW36515_IC  0x1111
#define USE_OCP81378_IC 0x2222

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct i2c_client *mumbaicd_flashlight_client;

static struct pinctrl *mumbaicd_pinctrl;
static struct pinctrl_state *mumbaicd_hwen_high;
static struct pinctrl_state *mumbaicd_hwen_low;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *mumbaicd_i2c_client;

/* platform data */
struct mumbaicd_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* mumbaicd chip data */
struct mumbaicd_chip_data {
	struct i2c_client *client;
	struct mumbaicd_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

enum FLASHLIGHT_DEVICE {
	AW36515_SM = 0x02,
	OCP81378_SM = 0x3A,
};

/******************************************************************************
 * Pinctrl configuration
 *****************************************************************************/
static int mumbaicd_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("mumbaicd_pinctrl_init start\n");
	/* get pinctrl */
	mumbaicd_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(mumbaicd_pinctrl)) {
		pr_err("Failed to get flashlight pinctrl.\n");
		ret = PTR_ERR(mumbaicd_pinctrl);
		return ret;
	}

	/* Flashlight HWEN pin initialization */
	mumbaicd_hwen_high = pinctrl_lookup_state(mumbaicd_pinctrl, MUMBAICD_PINCTRL_STATE_HWEN_HIGH);
	if (IS_ERR(mumbaicd_hwen_high)) {
		pr_err("Failed to init (%s)\n", MUMBAICD_PINCTRL_STATE_HWEN_HIGH);
		ret = PTR_ERR(mumbaicd_hwen_high);
		return ret;
	}

	mumbaicd_hwen_low = pinctrl_lookup_state(mumbaicd_pinctrl, MUMBAICD_PINCTRL_STATE_HWEN_LOW);
	if (IS_ERR(mumbaicd_hwen_low)) {
		pr_err("Failed to init (%s)\n", MUMBAICD_PINCTRL_STATE_HWEN_LOW);
		ret = PTR_ERR(mumbaicd_hwen_low);
		return ret;
	}
	pr_info("pinctrl_lookup_state mumbaicd_hwen_low finish\n");

	return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int mumbaicd_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct mumbaicd_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

/* i2c wrapper function */
static int mumbaicd_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct mumbaicd_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (val < 0)
		pr_err("failed read at 0x%02x\n", reg);

	return val;
}

static int mumbaicd_pinctrl_set(int pin, int state)
{
	int ret = 0;

	if (IS_ERR(mumbaicd_pinctrl)) {
		pr_err("pinctrl is not available\n");
		return -1;
	}

	switch (pin) {
	case MUMBAICD_PINCTRL_PIN_HWEN:
		if (state == MUMBAICD_PINCTRL_PINSTATE_LOW && !IS_ERR(mumbaicd_hwen_low)) {
			ret = pinctrl_select_state(mumbaicd_pinctrl, mumbaicd_hwen_low);
			//pinctrl_select_state(mumbaicd_pinctrl, mumbaicd_hwen_low);//rm to keep HWEN high
		}
		else if (state == MUMBAICD_PINCTRL_PINSTATE_HIGH && !IS_ERR(mumbaicd_hwen_high)) {
			ret = pinctrl_select_state(mumbaicd_pinctrl, mumbaicd_hwen_high);
		}
		else {
			pr_err("set err, pin(%d) state(%d)\n", pin, state);
		}
		break;
	default:
		pr_err("set err, pin(%d) state(%d)\n", pin, state);
		break;
	}
	pr_info("pin(%d) state(%d)\n", pin, state);

	return ret;
}

/******************************************************************************
 * mumbaicd operations
 *****************************************************************************/

static const int *mumbaicd_current;
static const unsigned char *mumbaicd_torch_level;
static const unsigned char *mumbaicd_flash_level;

static const int AW36515_current[MUMBAICD_LEVEL_NUM] = {
	24,   46,   70,   93,   116,  141,  164,  199,  246,  304,
	351,  398,  445,  504,  551,  598,  656,  703,  750,  797,
	856,  903,  949,  996,  1055, 1100, 1147, 1202
};

/*Offset: 0.98mA(00000000)
Step:1.96mA
Range: 0.98mA(00000000)~500mA(11111111)*/
static const unsigned char AW36515_torch_level[MUMBAICD_LEVEL_NUM] = {
	0x0C, 0x17, 0x23, 0x31, 0x3B, 0x47, 0x53, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*Offset: 3.91mA(00000000)
Step:7.83mA
Range: 3.91 mA(00000000)~2.0 A(11111111)*/
static const unsigned char AW36515_flash_level[MUMBAICD_LEVEL_NUM] = {
	0x03, 0x05, 0x08, 0x0C, 0x0E, 0x12, 0x14, 0x19, 0x1F, 0x26,
	0x2C, 0x32, 0x38, 0x40, 0x46, 0x4C, 0x53, 0x59, 0x5F, 0x65,
	0x6D, 0x73, 0x79, 0x7F, 0x86, 0x8C, 0x92, 0x99
};

static const int OCP81378_current[MUMBAICD_LEVEL_NUM] = {
	26,   45,   68,   98,   118,  141,  164,  202,  250,  296,
	343,  397,  437,  500,  547,  594,  656,  703,  750,  797,
	860,  907,  954,  1000,  1063, 1095, 1157, 1204
};

/*Offset: 0.98mA(00000000)
Step:3.84mA
Range: 0.98mA(00000000)~500mA(11111111)*/
static const unsigned char OCP81378_torch_level[MUMBAICD_LEVEL_NUM] = {
	0x06, 0x0B, 0x11, 0x15, 0x1E, 0x24, 0x2A, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*Offset: 15mA(00000000)
Step:15.65mA
Range: 15 mA(00000000)~2.0 A(11111111)*/
static const unsigned char OCP81378_flash_level[MUMBAICD_LEVEL_NUM] = {
	0x01, 0x02, 0x03, 0x05, 0x06, 0x08, 0x09, 0x0C, 0x0F, 0x12,
	0x15, 0x18, 0x1B, 0x1F, 0x22, 0x25, 0x29, 0x2C, 0x2F, 0x32,
	0x36, 0x39, 0x3C, 0x3F, 0x43, 0x45, 0x48, 0x4C
};

static volatile unsigned char mumbaicd_reg_enable;
static volatile int mumbaicd_level_ch1 = -1;
static volatile int mumbaicd_level_ch2 = -1;

static int mumbaicd_is_torch(int level)
{
	if (level >= MUMBAICD_LEVEL_TORCH)
		return -1;

	return 0;
}

static int mumbaicd_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= MUMBAICD_LEVEL_NUM)
		level = MUMBAICD_LEVEL_NUM - 1;

	return level;
}

/* flashlight enable function */
static int mumbaicd_enable_ch1(void)
{
	unsigned char reg, val;

	reg = MUMBAICD_REG_ENABLE;
	if (!mumbaicd_is_torch(mumbaicd_level_ch1)) {
		/* torch mode */
		mumbaicd_reg_enable |= MUMBAICD_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		mumbaicd_reg_enable |= MUMBAICD_ENABLE_LED1_FLASH;
	}
	val = mumbaicd_reg_enable;

	return mumbaicd_write_reg(mumbaicd_i2c_client, reg, val);
}

static int mumbaicd_enable_ch2(void)
{
	unsigned char reg, val;

	reg = MUMBAICD_REG_ENABLE;
	if (!mumbaicd_is_torch(mumbaicd_level_ch2)) {
		/* torch mode */
		mumbaicd_reg_enable |= MUMBAICD_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		mumbaicd_reg_enable |= MUMBAICD_ENABLE_LED2_FLASH;
	}
	val = mumbaicd_reg_enable;

	return mumbaicd_write_reg(mumbaicd_i2c_client, reg, val);
}

static int mumbaicd_enable(int channel)
{
	if (channel == MUMBAICD_CHANNEL_CH1)
		mumbaicd_enable_ch1();
	else if (channel == MUMBAICD_CHANNEL_CH2)
		mumbaicd_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}
	return 0;
}

/* flashlight disable function */
static int mumbaicd_disable_ch1(void)
{
	unsigned char reg, val;

	reg = MUMBAICD_REG_ENABLE;
	if (mumbaicd_reg_enable & MUMBAICD_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		mumbaicd_reg_enable &= (~MUMBAICD_ENABLE_LED1);
	} else {
		/* if LED 2 is enable, disable LED 1 and clear mode */
		mumbaicd_reg_enable &= (~MUMBAICD_ENABLE_LED1_FLASH);
	}
	val = mumbaicd_reg_enable;
	return mumbaicd_write_reg(mumbaicd_i2c_client, reg, val);
}

static int mumbaicd_disable_ch2(void)
{
	unsigned char reg, val;

	reg = MUMBAICD_REG_ENABLE;
	if (mumbaicd_reg_enable & MUMBAICD_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		mumbaicd_reg_enable &= (~MUMBAICD_ENABLE_LED2);
	} else {
		/* if LED 1 is enable, disable LED 2 and clear mode */
		mumbaicd_reg_enable &= (~MUMBAICD_ENABLE_LED2_FLASH);
	}
	val = mumbaicd_reg_enable;

	return mumbaicd_write_reg(mumbaicd_i2c_client, reg, val);
}

static int mumbaicd_disable(int channel)
{
	if (channel == MUMBAICD_CHANNEL_CH1) {
		mumbaicd_disable_ch1();
		pr_info("MUMBAICD_CHANNEL_CH1\n");
	} else if (channel == MUMBAICD_CHANNEL_CH2) {
		mumbaicd_disable_ch2();
		pr_info("MUMBAICD_CHANNEL_CH2\n");
	} else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int mumbaicd_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = mumbaicd_verify_level(level);

	/* set torch brightness level */
	reg = MUMBAICD_REG_TORCH_LEVEL_LED1;
	val = mumbaicd_torch_level[level];
	ret = mumbaicd_write_reg(mumbaicd_i2c_client, reg, val);

	mumbaicd_level_ch1 = level;

	/* set flash brightness level */
	reg = MUMBAICD_REG_FLASH_LEVEL_LED1;
	val = mumbaicd_flash_level[level];
	ret = mumbaicd_write_reg(mumbaicd_i2c_client, reg, val);

	return ret;
}

int mumbaicd_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = mumbaicd_verify_level(level);

	/* set torch brightness level */
	reg = MUMBAICD_REG_TORCH_LEVEL_LED2;
	val = mumbaicd_torch_level[level];
	ret = mumbaicd_write_reg(mumbaicd_i2c_client, reg, val);

	mumbaicd_level_ch2 = level;

	/* set flash brightness level */
	reg = MUMBAICD_REG_FLASH_LEVEL_LED2;
	val = mumbaicd_flash_level[level];
	ret = mumbaicd_write_reg(mumbaicd_i2c_client, reg, val);

	return ret;
}

static int mumbaicd_set_level(int channel, int level)
{
	if (channel == MUMBAICD_CHANNEL_CH1)
		mumbaicd_set_level_ch1(level);
	else if (channel == MUMBAICD_CHANNEL_CH2)
		mumbaicd_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}
/* flashlight init */
int mumbaicd_init(void)
{
	int ret;
	unsigned char reg, val, reg_val;
	int chip_id;

	chip_id = mumbaicd_read_reg(mumbaicd_i2c_client, 0x0C);
	msleep(2);
	pr_info("flashlight chip id: reg:0x0c, chip_id 0x%x",chip_id);
	if ( chip_id == AW36515_SM ) {
		reg_val = mumbaicd_read_reg(mumbaicd_i2c_client, MUMBAICD_AW36515_REG_BOOST_CONFIG);
		reg_val |= MUMBAICD_AW36515_SOFT_RESET_ENABLE;
		pr_info("flashlight chip id: reg:0x0c, data:0x%x;boost confgiuration: reg:0x07, reg_val: 0x%x", chip_id, reg_val);
		ret = mumbaicd_write_reg(mumbaicd_i2c_client, MUMBAICD_AW36515_REG_BOOST_CONFIG, reg_val);
		msleep(2);
	} else {
		mumbaicd_pinctrl_set(MUMBAICD_PINCTRL_PIN_HWEN, MUMBAICD_PINCTRL_PINSTATE_HIGH);
		reg_val = mumbaicd_read_reg(mumbaicd_i2c_client, MUMBAICD_AW36515_REG_BOOST_CONFIG);
		reg_val |= MUMBAICD_AW36515_SOFT_RESET_ENABLE;
		pr_info("flashlight chip id: reg:0x0c, data:0x%x;boost confgiuration: reg:0x07, reg_val: 0x%x", chip_id, reg_val);
		ret = mumbaicd_write_reg(mumbaicd_i2c_client, MUMBAICD_AW36515_REG_BOOST_CONFIG, reg_val);
		msleep(2);
	}
	/* clear enable register */
	reg = MUMBAICD_REG_ENABLE;
	val = MUMBAICD_DISABLE;
	ret = mumbaicd_write_reg(mumbaicd_i2c_client, reg, val);

	mumbaicd_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = MUMBAICD_REG_TIMING_CONF;
	val = MUMBAICD_TORCH_RAMP_TIME | MUMBAICD_FLASH_TIMEOUT;
	ret = mumbaicd_write_reg(mumbaicd_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int mumbaicd_uninit(void)
{
	mumbaicd_disable(MUMBAICD_CHANNEL_CH1);
	mumbaicd_disable(MUMBAICD_CHANNEL_CH2);
	mumbaicd_pinctrl_set(MUMBAICD_PINCTRL_PIN_HWEN, MUMBAICD_PINCTRL_PINSTATE_LOW);
	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer mumbaicd_timer_ch1;
static struct hrtimer mumbaicd_timer_ch2;
static unsigned int mumbaicd_timeout_ms[MUMBAICD_CHANNEL_NUM];

static void mumbaicd_work_disable_ch1(struct work_struct *data)
{
	pr_info("ht work queue callback\n");
	mumbaicd_disable_ch1();
}

static void mumbaicd_work_disable_ch2(struct work_struct *data)
{
	pr_info("lt work queue callback\n");
	mumbaicd_disable_ch2();
}

static enum hrtimer_restart mumbaicd_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&mumbaicd_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart mumbaicd_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&mumbaicd_work_ch2);
	return HRTIMER_NORESTART;
}

int mumbaicd_timer_start(int channel, ktime_t ktime)
{
	if (channel == MUMBAICD_CHANNEL_CH1)
		hrtimer_start(&mumbaicd_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == MUMBAICD_CHANNEL_CH2)
		hrtimer_start(&mumbaicd_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

int mumbaicd_timer_cancel(int channel)
{
	if (channel == MUMBAICD_CHANNEL_CH1)
		hrtimer_cancel(&mumbaicd_timer_ch1);
	else if (channel == MUMBAICD_CHANNEL_CH2)
		hrtimer_cancel(&mumbaicd_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int mumbaicd_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= MUMBAICD_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_info("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		mumbaicd_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		mumbaicd_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (mumbaicd_timeout_ms[channel]) {
				ktime = ktime_set(mumbaicd_timeout_ms[channel] / 1000,
						(mumbaicd_timeout_ms[channel] % 1000) * 1000000);
				mumbaicd_timer_start(channel, ktime);
			}
			mumbaicd_enable(channel);
		} else {
			mumbaicd_disable(channel);
			mumbaicd_timer_cancel(channel);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = MUMBAICD_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = MUMBAICD_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = mumbaicd_verify_level(fl_arg->arg);
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = mumbaicd_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_info("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = MUMBAICD_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int mumbaicd_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int mumbaicd_release(void)
{
	/* uninit chip and clear usage count */

	mutex_lock(&mumbaicd_mutex);
	use_count = 0;
	mumbaicd_uninit();
	mutex_unlock(&mumbaicd_mutex);

	pr_info("Release: %d\n", use_count);

	return 0;
}

static int mumbaicd_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&mumbaicd_mutex);
	if (set) {
		if (!use_count) {
			ret = mumbaicd_init();
		}
		use_count++;
		pr_info("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count) {
			ret = mumbaicd_uninit();
		}
		if (use_count < 0) {
			use_count = 0;
		}
		mumbaicd_pinctrl_set(MUMBAICD_PINCTRL_PIN_HWEN, MUMBAICD_PINCTRL_PINSTATE_LOW);
		pr_info("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&mumbaicd_mutex);

	return ret;
}

static ssize_t mumbaicd_strobe_store(struct flashlight_arg arg)
{
	mumbaicd_set_driver(1);
	mumbaicd_set_level(arg.channel, arg.level);
	mumbaicd_timeout_ms[arg.channel] = 0;
	mumbaicd_enable(arg.channel);
	msleep(arg.dur);
	mumbaicd_disable(arg.channel);
	//mumbaicd_release(NULL);
	mumbaicd_set_driver(0);
	return 0;
}

static struct flashlight_operations mumbaicd_ops = {
	mumbaicd_open,
	mumbaicd_release,
	mumbaicd_ioctl,
	mumbaicd_strobe_store,
	mumbaicd_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int mumbaicd_chip_init(struct mumbaicd_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * mumbaicd_init();
	 */

	return 0;
}

static int mumbaicd_parse_dt(struct device *dev,
		struct mumbaicd_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				MUMBAICD_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int mumbaicd_chip_id(void)
{
	int chip_id;
	int reg00_id = -1;
	msleep(1);
	chip_id = mumbaicd_read_reg(mumbaicd_i2c_client, 0x0C);
	pr_info("flashlight chip id: reg:0x0c, data:0x%x", chip_id);
	if (chip_id == AW36515_SM) {
		reg00_id = mumbaicd_read_reg(mumbaicd_i2c_client, 0x00);
		pr_info("flashlight chip id: reg:0x00, data:0x%x", reg00_id);
		if (reg00_id == 0x30) {
			chip_id = AW36515_SM;
			pr_info("flashlight reg00_id = 0x%x, set chip_id to AW36515_SM", reg00_id);
		}
	} else {
		mumbaicd_pinctrl_set(MUMBAICD_PINCTRL_PIN_HWEN, MUMBAICD_PINCTRL_PINSTATE_HIGH);
		if(mumbaicd_read_reg(mumbaicd_i2c_client, 0x0C) == OCP81378_SM) {
			chip_id = OCP81378_SM;
			pr_info("flashlight reg00_id = 0x%x, set chip_id to OCP81378_SM", reg00_id);
		}
		mumbaicd_pinctrl_set(MUMBAICD_PINCTRL_PIN_HWEN, MUMBAICD_PINCTRL_PINSTATE_LOW);
	}
	if (chip_id == AW36515_SM) {
		pr_info(" the device's flashlight driver IC is AW36515\n");
		return USE_AW36515_IC;
	} else if(chip_id == OCP81378_SM) {
		pr_info(" the device's flashlight driver IC is OCP81378\n");
		// pr_err(" the device's flashlight driver IC is not used in our project!\n");
		return USE_OCP81378_IC;
	} else {
		pr_err(" the device's flashlight driver IC is not used in our project!\n");
		return USE_AW36515_IC;
	}
}

static int mumbaicd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mumbaicd_chip_data *chip;
	struct mumbaicd_platform_data *pdata = client->dev.platform_data;
	int err;
	int i;
	int chip_id;
	bool curProject = false;
	pr_info("mumbaicd_i2c_probe Probe start.\n");
	curProject = is_project(25618) || is_project(25619) || is_project(25720) || is_project(25721) || is_project(25722) || is_project(25730);

	if (!curProject) {
		err = -ENODEV;
		pr_err("[%s] it is not our project!\n", __func__);
		goto err_out;
	}

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct mumbaicd_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pr_err("Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct mumbaicd_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free_chip;
		}
		client->dev.platform_data = pdata;
		err = mumbaicd_parse_dt(&client->dev, pdata);
		if (err) {
			goto err_free_pdata;
		}
		chip->no_pdata = 1; // Mark that pdata is dynamically allocated
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	mumbaicd_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&mumbaicd_work_ch1, mumbaicd_work_disable_ch1);
	INIT_WORK(&mumbaicd_work_ch2, mumbaicd_work_disable_ch2);

	/* init timer */
	hrtimer_init(&mumbaicd_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mumbaicd_timer_ch1.function = mumbaicd_timer_func_ch1;
	hrtimer_init(&mumbaicd_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mumbaicd_timer_ch2.function = mumbaicd_timer_func_ch2;
	mumbaicd_timeout_ms[MUMBAICD_CHANNEL_CH1] = 100;
	mumbaicd_timeout_ms[MUMBAICD_CHANNEL_CH2] = 100;

	/* init chip hw */
	mumbaicd_chip_init(chip);
	chip_id = mumbaicd_chip_id();
	if (chip_id == USE_AW36515_IC) {
		mumbaicd_current = AW36515_current;
		mumbaicd_torch_level = AW36515_torch_level;
		mumbaicd_flash_level = AW36515_flash_level;
	} else if (chip_id == USE_OCP81378_IC) {
		mumbaicd_current = OCP81378_current;
		mumbaicd_torch_level = OCP81378_torch_level;
		mumbaicd_flash_level = OCP81378_flash_level;
	}

	/* register flashlight operations */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&mumbaicd_ops)) {
				pr_err("Failed to register flashlight device.\n");
				err = -EFAULT;
				goto err_free_pdata;
			}
	} else {
		if (flashlight_dev_register(MUMBAICD_NAME, &mumbaicd_ops)) {
			pr_err("Failed to register flashlight device.\n");
			err = -EFAULT;
			goto err_free_pdata;
		}
	}

	//mumbaicd_create_sysfs(client);

	pr_info("Probe done.\n");

	return 0;

err_free_pdata:
	if (chip->no_pdata)
		kfree(chip->pdata);
err_free_chip:
	kfree(chip);
err_out:
	return err;
}

static int mumbaicd_i2c_remove(struct i2c_client *client)
{
	struct mumbaicd_platform_data *pdata = dev_get_platdata(&client->dev);
	struct mumbaicd_chip_data *chip = i2c_get_clientdata(client);
	int i;

	pr_info("Remove start.\n");
	mumbaicd_pinctrl_set(MUMBAICD_PINCTRL_PIN_HWEN, MUMBAICD_PINCTRL_PINSTATE_LOW);

	client->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(MUMBAICD_NAME);

	/* flush work queue */
	flush_work(&mumbaicd_work_ch1);
	flush_work(&mumbaicd_work_ch2);

	/* unregister flashlight operations */
	flashlight_dev_unregister(MUMBAICD_NAME);

	/* free resource */
	if (chip && chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	pr_info("Remove done.\n");

	return 0;
}

static const struct i2c_device_id mumbaicd_i2c_id[] = {
	{MUMBAICD_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id mumbaicd_i2c_of_match[] = {
	{.compatible = MUMBAICD_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, mumbaicd_i2c_of_match);
#endif

static struct i2c_driver mumbaicd_i2c_driver = {
	.driver = {
		   .name = MUMBAICD_NAME,
#ifdef CONFIG_OF
		   .of_match_table = mumbaicd_i2c_of_match,
#endif
		   },
	.probe = mumbaicd_i2c_probe,
	.remove = mumbaicd_i2c_remove,
	.id_table = mumbaicd_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int mumbaicd_probe(struct platform_device *dev)
{
	pr_info("Probe start %s.\n", MUMBAICD_DTNAME_I2C);

	/* init pinctrl */
	if (mumbaicd_pinctrl_init(dev)) {
		pr_err("Failed to init pinctrl.\n");
		return -1;
	}

	if (i2c_add_driver(&mumbaicd_i2c_driver)) {
		pr_err("Failed to add i2c driver.\n");
		return -1;
	}

	pr_info("Probe done.\n");

	return 0;
}

static int mumbaicd_remove(struct platform_device *dev)
{
	pr_info("Remove start.\n");

	i2c_del_driver(&mumbaicd_i2c_driver);

	pr_info("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mumbaicd_of_match[] = {
	{.compatible = MUMBAICD_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, mumbaicd_of_match);
#else
static struct platform_device mumbaicd_platform_device[] = {
	{
		.name = MUMBAICD_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, mumbaicd_platform_device);
#endif

static struct platform_driver mumbaicd_platform_driver = {
	.probe = mumbaicd_probe,
	.remove = mumbaicd_remove,
	.driver = {
		.name = MUMBAICD_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mumbaicd_of_match,
#endif
	},
};

static int __init flashlight_mumbaicd_init(void)
{
	int ret;

	pr_info("flashlight_mumbaicd-Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&mumbaicd_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&mumbaicd_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_info("flashlight_mumbaicd Init done.\n");

	return 0;
}

static void __exit flashlight_mumbaicd_exit(void)
{
	pr_info("flashlight_mumbaicd-Exit start.\n");

	platform_driver_unregister(&mumbaicd_platform_driver);

	pr_info("flashlight_mumbaicd Exit done.\n");
}


module_init(flashlight_mumbaicd_init);
module_exit(flashlight_mumbaicd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight MUMBAICD Driver");

