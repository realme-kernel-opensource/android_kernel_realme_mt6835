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
#ifndef CHOPARDB_DTNAME
#define CHOPARDB_DTNAME "mediatek,flashlights_chopardb"
#endif
#ifndef CHOPARDB_DTNAME_I2C
#define CHOPARDB_DTNAME_I2C   "mediatek,strobe_main"
#endif

#define CHOPARDB_NAME "flashlights_chopardb"

/* define registers */
#define CHOPARDB_REG_ENABLE           (0x01)
#define CHOPARDB_MASK_ENABLE_LED1     (0x01)
#define CHOPARDB_MASK_ENABLE_LED2     (0x02)
#define CHOPARDB_DISABLE              (0x00)
#define CHOPARDB_TORCH_MODE           (0x08)
#define CHOPARDB_FLASH_MODE           (0x0C)
#define CHOPARDB_ENABLE_LED1          (0x01)
#define CHOPARDB_ENABLE_LED1_TORCH    (0x09)
#define CHOPARDB_ENABLE_LED1_FLASH    (0x0D)
#define CHOPARDB_ENABLE_LED2          (0x02)
#define CHOPARDB_ENABLE_LED2_TORCH    (0x0A)
#define CHOPARDB_ENABLE_LED2_FLASH    (0x0E)

#define CHOPARDB_REG_TORCH_LEVEL_LED1 (0x05)
#define CHOPARDB_REG_FLASH_LEVEL_LED1 (0x03)
#define CHOPARDB_REG_TORCH_LEVEL_LED2 (0x06)
#define CHOPARDB_REG_FLASH_LEVEL_LED2 (0x04)

#define CHOPARDB_REG_TIMING_CONF      (0x08)
#define CHOPARDB_TORCH_RAMP_TIME      (0x10)
#define CHOPARDB_FLASH_TIMEOUT        (0x0F)

#define CHOPARDB_AW36515_SOFT_RESET_ENABLE (0x80)
#define CHOPARDB_AW36515_REG_BOOST_CONFIG (0x07)


/* define channel, level */
#define CHOPARDB_CHANNEL_NUM          2
#define CHOPARDB_CHANNEL_CH1          0
#define CHOPARDB_CHANNEL_CH2          1
/* define level */
#define CHOPARDB_LEVEL_NUM 28
#define CHOPARDB_LEVEL_TORCH 7

#define CHOPARDB_HW_TIMEOUT 400 /* ms */

/* define mutex and work queue */
static DEFINE_MUTEX(chopardb_mutex);
static struct work_struct chopardb_work_ch1;
static struct work_struct chopardb_work_ch2;

/* define pinctrl */
#define CHOPARDB_PINCTRL_PIN_HWEN 0
#define CHOPARDB_PINCTRL_PINSTATE_LOW 0
#define CHOPARDB_PINCTRL_PINSTATE_HIGH 1
#define CHOPARDB_PINCTRL_STATE_HWEN_HIGH "chopardb_hwen_high"
#define CHOPARDB_PINCTRL_STATE_HWEN_LOW  "chopardb_hwen_low"

/* define device id */
#define USE_AW36515_IC  0x1111

struct i2c_client *chopardb_flashlight_client;

/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *chopardb_i2c_client;

/* platform data */
struct chopardb_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

/* chopardb chip data */
struct chopardb_chip_data {
	struct i2c_client *client;
	struct chopardb_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

enum FLASHLIGHT_DEVICE {
	AW36515_SM = 0x02,
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int chopardb_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct chopardb_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		pr_err("failed writing at 0x%02x\n", reg);

	return ret;
}

/* i2c wrapper function */
static int chopardb_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct chopardb_chip_data *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (val < 0)
		pr_err("failed read at 0x%02x\n", reg);

	return val;
}


/******************************************************************************
 * chopardb operations
 *****************************************************************************/

static const int *chopardb_current;
static const unsigned char *chopardb_torch_level;
static const unsigned char *chopardb_flash_level;

static const int AW36515_current[CHOPARDB_LEVEL_NUM] = {
	24,   50,   75,   99,   125,  150,  162,  200,  310,  380,
	450,  520,  590,  660,  730,  800,  870,  940,  1010, 1080,
	1150, 1220, 1290, 1360, 1430, 1500, 1570, 1640
};

/*Offset: 0.98mA(00000000)
Step:1.96mA
Range: 0.98mA(00000000)~500mA(11111111)*/
static const unsigned char AW36515_torch_level[CHOPARDB_LEVEL_NUM] = {
	0x0C, 0x19, 0x26, 0x32, 0x3F, 0x4C, 0x52, 0x66, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*Offset: 3.91mA(00000000)
Step:7.83mA
Range: 3.91 mA(00000000)~2.0 A(11111111)*/
static const unsigned char AW36515_flash_level[CHOPARDB_LEVEL_NUM] = {
	0x03, 0x06, 0x09, 0x0C, 0x0F, 0x13, 0x15, 0x19, 0x26, 0x2F,
	0x38, 0x41, 0x4A, 0x53, 0x5C, 0x65, 0x6E, 0x77, 0x80, 0x89,
	0x92, 0x9B, 0xA4, 0xAD, 0xB6, 0xBF, 0xC8, 0xD1
};


static volatile unsigned char chopardb_reg_enable;
static volatile int chopardb_level_ch1 = -1;
static volatile int chopardb_level_ch2 = -1;

static int chopardb_is_torch(int level)
{
	if (level >= CHOPARDB_LEVEL_TORCH)
		return -1;

	return 0;
}

static int chopardb_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= CHOPARDB_LEVEL_NUM)
		level = CHOPARDB_LEVEL_NUM - 1;

	return level;
}

/* flashlight enable function */
static int chopardb_enable_ch1(void)
{
	unsigned char reg, val;

	reg = CHOPARDB_REG_ENABLE;
	if (!chopardb_is_torch(chopardb_level_ch1)) {
		/* torch mode */
		chopardb_reg_enable |= CHOPARDB_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		chopardb_reg_enable |= CHOPARDB_ENABLE_LED1_FLASH;
	}
	val = chopardb_reg_enable;

	return chopardb_write_reg(chopardb_i2c_client, reg, val);
}

static int chopardb_enable_ch2(void)
{
	unsigned char reg, val;

	reg = CHOPARDB_REG_ENABLE;
	if (!chopardb_is_torch(chopardb_level_ch2)) {
		/* torch mode */
		chopardb_reg_enable |= CHOPARDB_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		chopardb_reg_enable |= CHOPARDB_ENABLE_LED2_FLASH;
	}
	val = chopardb_reg_enable;

	return chopardb_write_reg(chopardb_i2c_client, reg, val);
}

static int chopardb_enable(int channel)
{
	if (channel == CHOPARDB_CHANNEL_CH1)
		chopardb_enable_ch1();
	else if (channel == CHOPARDB_CHANNEL_CH2)
		chopardb_enable_ch2();
	else {
		pr_err("Error channel\n");
		return -1;
	}
	return 0;
}

/* flashlight disable function */
static int chopardb_disable_ch1(void)
{
	unsigned char reg, val;

	reg = CHOPARDB_REG_ENABLE;
	if (chopardb_reg_enable & CHOPARDB_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		chopardb_reg_enable &= (~CHOPARDB_ENABLE_LED1);
	} else {
		/* if LED 2 is enable, disable LED 1 and clear mode */
		chopardb_reg_enable &= (~CHOPARDB_ENABLE_LED1_FLASH);
	}
	val = chopardb_reg_enable;
	return chopardb_write_reg(chopardb_i2c_client, reg, val);
}

static int chopardb_disable_ch2(void)
{
	unsigned char reg, val;

	reg = CHOPARDB_REG_ENABLE;
	if (chopardb_reg_enable & CHOPARDB_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		chopardb_reg_enable &= (~CHOPARDB_ENABLE_LED2);
	} else {
		/* if LED 1 is enable, disable LED 2 and clear mode */
		chopardb_reg_enable &= (~CHOPARDB_ENABLE_LED2_FLASH);
	}
	val = chopardb_reg_enable;

	return chopardb_write_reg(chopardb_i2c_client, reg, val);
}

static int chopardb_disable(int channel)
{
	if (channel == CHOPARDB_CHANNEL_CH1) {
		chopardb_disable_ch1();
		pr_info("CHOPARDB_CHANNEL_CH1\n");
	} else if (channel == CHOPARDB_CHANNEL_CH2) {
		chopardb_disable_ch2();
		pr_info("CHOPARDB_CHANNEL_CH2\n");
	} else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int chopardb_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = chopardb_verify_level(level);

	/* set torch brightness level */
	reg = CHOPARDB_REG_TORCH_LEVEL_LED1;
	val = chopardb_torch_level[level];
	ret = chopardb_write_reg(chopardb_i2c_client, reg, val);

	chopardb_level_ch1 = level;

	/* set flash brightness level */
	reg = CHOPARDB_REG_FLASH_LEVEL_LED1;
	val = chopardb_flash_level[level];
	ret = chopardb_write_reg(chopardb_i2c_client, reg, val);

	return ret;
}

int chopardb_set_level_ch2(int level)
{
	int ret;
	unsigned char reg, val;

	level = chopardb_verify_level(level);

	/* set torch brightness level */
	reg = CHOPARDB_REG_TORCH_LEVEL_LED2;
	val = chopardb_torch_level[level];
	ret = chopardb_write_reg(chopardb_i2c_client, reg, val);

	chopardb_level_ch2 = level;

	/* set flash brightness level */
	reg = CHOPARDB_REG_FLASH_LEVEL_LED2;
	val = chopardb_flash_level[level];
	ret = chopardb_write_reg(chopardb_i2c_client, reg, val);

	return ret;
}

static int chopardb_set_level(int channel, int level)
{
	if (channel == CHOPARDB_CHANNEL_CH1)
		chopardb_set_level_ch1(level);
	else if (channel == CHOPARDB_CHANNEL_CH2)
		chopardb_set_level_ch2(level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}
/* flashlight init */
int chopardb_init(void)
{
	int ret;
	unsigned char reg, val, reg_val;
	int chip_id;


	chip_id = chopardb_read_reg(chopardb_i2c_client, 0x0c);
	msleep(2);
	pr_info("flashlight chip id: reg:0x0c, chip_id 0x%x",chip_id);
	if ( chip_id == AW36515_SM ) {
		reg_val = chopardb_read_reg(chopardb_i2c_client, CHOPARDB_AW36515_REG_BOOST_CONFIG);
		reg_val |= CHOPARDB_AW36515_SOFT_RESET_ENABLE;
		pr_info("flashlight chip id: reg:0x0c, data:0x%x;boost confgiuration: reg:0x07, reg_val: 0x%x", chip_id, reg_val);
		ret = chopardb_write_reg(chopardb_i2c_client, CHOPARDB_AW36515_REG_BOOST_CONFIG, reg_val);
		if (ret < 0) {
			pr_err("Failed to write to boost configuration register\n");
			return ret;
		}
		msleep(2);
	}
	/* clear enable register */
	reg = CHOPARDB_REG_ENABLE;
	val = CHOPARDB_DISABLE;
	ret = chopardb_write_reg(chopardb_i2c_client, reg, val);
	if (ret < 0) {
		pr_err("Failed to write to enable register\n");
		return ret;
	}

	chopardb_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = CHOPARDB_REG_TIMING_CONF;
	val = CHOPARDB_TORCH_RAMP_TIME | CHOPARDB_FLASH_TIMEOUT;
	ret = chopardb_write_reg(chopardb_i2c_client, reg, val);
	if (ret < 0) {
		pr_err("Failed to write to timing configuration register\n");
		return ret;
	}

	return ret;
}

/* flashlight uninit */
int chopardb_uninit(void)
{
	chopardb_disable(CHOPARDB_CHANNEL_CH1);
	chopardb_disable(CHOPARDB_CHANNEL_CH2);

	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer chopardb_timer_ch1;
static struct hrtimer chopardb_timer_ch2;
static unsigned int chopardb_timeout_ms[CHOPARDB_CHANNEL_NUM];

static void chopardb_work_disable_ch1(struct work_struct *data)
{
	pr_info("ht work queue callback\n");
	chopardb_disable_ch1();
}

static void chopardb_work_disable_ch2(struct work_struct *data)
{
	pr_info("lt work queue callback\n");
	chopardb_disable_ch2();
}

static enum hrtimer_restart chopardb_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&chopardb_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart chopardb_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&chopardb_work_ch2);
	return HRTIMER_NORESTART;
}

int chopardb_timer_start(int channel, ktime_t ktime)
{
	if (channel == CHOPARDB_CHANNEL_CH1)
		hrtimer_start(&chopardb_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == CHOPARDB_CHANNEL_CH2)
		hrtimer_start(&chopardb_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

int chopardb_timer_cancel(int channel)
{
	if (channel == CHOPARDB_CHANNEL_CH1)
		hrtimer_cancel(&chopardb_timer_ch1);
	else if (channel == CHOPARDB_CHANNEL_CH2)
		hrtimer_cancel(&chopardb_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int chopardb_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= CHOPARDB_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}
	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_info("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		chopardb_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		chopardb_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (chopardb_timeout_ms[channel]) {
				ktime = ktime_set(chopardb_timeout_ms[channel] / 1000,
						(chopardb_timeout_ms[channel] % 1000) * 1000000);
				chopardb_timer_start(channel, ktime);
			}
			chopardb_enable(channel);
		} else {
			chopardb_disable(channel);
			chopardb_timer_cancel(channel);
		}
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_info("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = CHOPARDB_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_info("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = CHOPARDB_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = chopardb_verify_level(fl_arg->arg);
		pr_info("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = chopardb_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_info("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = CHOPARDB_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int chopardb_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int chopardb_release(void)
{
	/* uninit chip and clear usage count */
/*
	mutex_lock(&chopardb_mutex);
	use_count--;
	if (!use_count)
		chopardb_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&chopardb_mutex);

	pr_info("Release: %d\n", use_count);
*/
	return 0;
}

static int chopardb_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&chopardb_mutex);
	if (set) {
		if (!use_count)
			ret = chopardb_init();
		use_count++;
		pr_info("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = chopardb_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_info("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&chopardb_mutex);

	return ret;
}

static ssize_t chopardb_strobe_store(struct flashlight_arg arg)
{
	int channel = arg.channel;
	int level = arg.level;
	int dur = arg.dur;
	chopardb_set_driver(1);
	chopardb_set_level(channel, level);
	chopardb_timeout_ms[channel] = 0;
	chopardb_enable(channel);
	msleep(dur);
	chopardb_disable(channel);
	//chopardb_release(NULL);
	chopardb_set_driver(0);
	return 0;
}

static struct flashlight_operations chopardb_ops = {
	chopardb_open,
	chopardb_release,
	chopardb_ioctl,
	chopardb_strobe_store,
	chopardb_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int chopardb_chip_init(struct chopardb_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * chopardb_init();
	 */

	return 0;
}

static int chopardb_parse_dt(struct device *dev,
		struct chopardb_platform_data *pdata)
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
				CHOPARDB_NAME);
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

static int chopardb_chip_id(void)
{
	int chip_id;
	int reg00_id = -1;
	msleep(1);
	chip_id = chopardb_read_reg(chopardb_i2c_client, 0x0c);
	pr_info("flashlight chip id: reg:0x0c, data:0x%x", chip_id);
	if (chip_id == AW36515_SM) {
		reg00_id = chopardb_read_reg(chopardb_i2c_client, 0x00);
		pr_info("flashlight chip id: reg:0x00, data:0x%x", reg00_id);
		if (reg00_id == 0x30) {
			chip_id = AW36515_SM;
			pr_info("flashlight reg00_id = 0x%x, set chip_id to AW36515_SM", reg00_id);
		}
	}
    if (chip_id == AW36515_SM){
		pr_info(" the device's flashlight driver IC is AW36515\n");
		return USE_AW36515_IC;
	} else {
		pr_err(" the device's flashlight driver IC is not used in our project!\n");
		return USE_AW36515_IC;
	}
}

static int chopardb_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct chopardb_chip_data *chip;
    struct chopardb_platform_data *pdata = client->dev.platform_data;
    int err;
    int i;
    int chip_id;
    bool curProject = false;
    pr_info("chopardb_i2c_probe Probe start.\n");
    curProject = is_project(25617) || is_project(25716) || is_project(25717) || is_project(25719);

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
    chip = kzalloc(sizeof(struct chopardb_chip_data), GFP_KERNEL);
    if (!chip) {
        err = -ENOMEM;
        goto err_out;
    }
    chip->client = client;

    /* init platform data */
    if (!pdata) {
        pr_err("Platform data does not exist\n");
        pdata = kzalloc(sizeof(struct chopardb_platform_data), GFP_KERNEL);
        if (!pdata) {
            err = -ENOMEM;
            goto err_free_chip;
        }
        client->dev.platform_data = pdata;
        err = chopardb_parse_dt(&client->dev, pdata);
        if (err) {
            goto err_free_pdata;
        }
        chip->no_pdata = 1; // Mark that pdata is dynamically allocated
    }
    chip->pdata = pdata;
    i2c_set_clientdata(client, chip);
    chopardb_i2c_client = client;

    /* init mutex and spinlock */
    mutex_init(&chip->lock);

    /* init work queue */
    INIT_WORK(&chopardb_work_ch1, chopardb_work_disable_ch1);
    INIT_WORK(&chopardb_work_ch2, chopardb_work_disable_ch2);

    /* init timer */
    hrtimer_init(&chopardb_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    chopardb_timer_ch1.function = chopardb_timer_func_ch1;
    hrtimer_init(&chopardb_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    chopardb_timer_ch2.function = chopardb_timer_func_ch2;
    chopardb_timeout_ms[CHOPARDB_CHANNEL_CH1] = 100;
    chopardb_timeout_ms[CHOPARDB_CHANNEL_CH2] = 100;

    /* init chip hw */
    chopardb_chip_init(chip);
    chip_id = chopardb_chip_id();
    if (chip_id == USE_AW36515_IC){
        chopardb_current = AW36515_current;
        chopardb_torch_level = AW36515_torch_level;
        chopardb_flash_level = AW36515_flash_level;
    }

    /* register flashlight operations */
    if (pdata->channel_num) {
        for (i = 0; i < pdata->channel_num; i++)
            if (flashlight_dev_register_by_device_id(
                        &pdata->dev_id[i],
                        &chopardb_ops)) {
                pr_err("Failed to register flashlight device.\n");
                err = -EFAULT;
                goto err_free_pdata;
            }
    } else {
        if (flashlight_dev_register(CHOPARDB_NAME, &chopardb_ops)) {
            pr_err("Failed to register flashlight device.\n");
            err = -EFAULT;
            goto err_free_pdata;
        }
    }

    //chopardb_create_sysfs(client);

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

static int chopardb_i2c_remove(struct i2c_client *client)
{
    struct chopardb_platform_data *pdata = dev_get_platdata(&client->dev);
    struct chopardb_chip_data *chip = i2c_get_clientdata(client);
    int i;

    pr_info("Remove start.\n");

    client->dev.platform_data = NULL;

    /* unregister flashlight device */
    if (pdata && pdata->channel_num)
        for (i = 0; i < pdata->channel_num; i++)
            flashlight_dev_unregister_by_device_id(
                    &pdata->dev_id[i]);
    else
        flashlight_dev_unregister(CHOPARDB_NAME);

    /* flush work queue */
    flush_work(&chopardb_work_ch1);
    flush_work(&chopardb_work_ch2);

    /* unregister flashlight operations */
    flashlight_dev_unregister(CHOPARDB_NAME);

    /* free resource */
    if (chip && chip->no_pdata)
        kfree(chip->pdata);
    kfree(chip);

    pr_info("Remove done.\n");

    return 0;
}

static const struct i2c_device_id chopardb_i2c_id[] = {
	{CHOPARDB_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id chopardb_i2c_of_match[] = {
	{.compatible = CHOPARDB_DTNAME_I2C},
	{},
};
MODULE_DEVICE_TABLE(of, chopardb_i2c_of_match);
#endif

static struct i2c_driver chopardb_i2c_driver = {
	.driver = {
		   .name = CHOPARDB_NAME,
#ifdef CONFIG_OF
		   .of_match_table = chopardb_i2c_of_match,
#endif
		   },
	.probe = chopardb_i2c_probe,
	.remove = chopardb_i2c_remove,
	.id_table = chopardb_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int chopardb_probe(struct platform_device *dev)
{
	pr_info("Probe start %s.\n", CHOPARDB_DTNAME_I2C);

	if (i2c_add_driver(&chopardb_i2c_driver)) {
		pr_err("Failed to add i2c driver.\n");
		return -1;
	}

	pr_info("Probe done.\n");

	return 0;
}

static int chopardb_remove(struct platform_device *dev)
{
	pr_info("Remove start.\n");

	i2c_del_driver(&chopardb_i2c_driver);

	pr_info("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id chopardb_of_match[] = {
	{.compatible = CHOPARDB_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, chopardb_of_match);
#else
static struct platform_device chopardb_platform_device[] = {
	{
		.name = CHOPARDB_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, chopardb_platform_device);
#endif

static struct platform_driver chopardb_platform_driver = {
	.probe = chopardb_probe,
	.remove = chopardb_remove,
	.driver = {
		.name = CHOPARDB_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = chopardb_of_match,
#endif
	},
};

static int is_feature_disable(void)
{
	struct device_node *node;
	int ret = 0;

	node = of_find_compatible_node(NULL, NULL, "mediatek,flashlights_chopardb");
	if (node == NULL) {
		pr_err("Can't mediatek,flashlights_chopardb\n");
		goto out;
	}
	ret = of_property_read_bool(node,"feature-disable");
	pr_err("feature-disable is %d\n", ret);
	of_node_put(node);

out:
	return ret;
}

static int __init flashlight_chopardb_init(void)
{
	int ret;

	pr_info("flashlight_chopardb-Init start.\n");
	if (is_feature_disable()) {
		return -ENODEV;
	}
#ifndef CONFIG_OF
	ret = platform_device_register(&chopardb_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&chopardb_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_info("flashlight_chopardb Init done.\n");

	return 0;
}

static void __exit flashlight_chopardb_exit(void)
{
	pr_info("flashlight_chopardb-Exit start.\n");

	platform_driver_unregister(&chopardb_platform_driver);

	pr_info("flashlight_chopardb Exit done.\n");
}


module_init(flashlight_chopardb_init);
module_exit(flashlight_chopardb_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight CHOPARDB Driver");

