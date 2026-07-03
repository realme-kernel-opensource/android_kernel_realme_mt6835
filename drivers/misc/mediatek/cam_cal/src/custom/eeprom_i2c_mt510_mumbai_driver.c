/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__
#define OTP_NAME "MT510_OTP"
#define LOG_INF(format, args...)                                               \
pr_debug(OTP_NAME " [%s] " format, __func__, ##args)

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cam_cal_list.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/compat.h>
#endif
#include "kd_camera_typedef.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
#include "eeprom_i2c_mt510_mumbai_driver.h"

#define EEPROM_I2C_MSG_SIZE_READ 2
#define MAX_EEPROM_BYTE 0x17FF
#define READ_FLAG_ADDR 0x17FE
#define MT510_OTP_RET_FAIL -1
//size
#define MT510_SN_LENS 25
#define MT510_LRC_LENS 6
#define MT510_LSC_LENS 1870
#define MT510_OTP_AWB_LENS 18
#define MT510_OTP_MODULE_LENS 17
//flag
#define MT510_GROUP1_FLAG 0x01
#define MT510_GROUP2_FLAG 0x13
#define MT510_OTP_GROUP_FLAGADDR 0x0605
//module info
#define MT510_OTP_MODULE_GROUP1_STARTADDR 0x0606
#define MT510_OTP_MODULE_GROUP2_STARTADDR 0x0DB5
//module sn
#define MT510_OTP_SN_GROUP1_STARTADDR 0x0647
#define MT510_OTP_SN_GROUP2_STARTADDR 0x0DF6
//module lsc
#define MT510_OTP_LSC_GROUP1_STARTADDR 0x0661
#define MT510_OTP_LSC_GROUP2_STARTADDR 0x0E10
//awb5000
#define MT510_OTP_AWB5000K_GROUP1_STARTADDR 0x0617
#define MT510_OTP_AWB5000K_GROUP2_STARTADDR 0x0DC6
//awb2850
#define MT510_OTP_AWB2850K_GROUP1_STARTADDR 0x0629
#define MT510_OTP_AWB2850K_GROUP2_STARTADDR 0x0DD8
//lrc5000
#define MT510_OTP_LRC5000K_GROUP1_STARTADDR 0x063B
#define MT510_OTP_LRC5000K_GROUP2_STARTADDR 0x0DEA
//lrc2580
#define MT510_OTP_LRC2850K_GROUP1_STARTADDR 0x0641
#define MT510_OTP_LRC2850K_GROUP2_STARTADDR 0x0DF0

static DEFINE_MUTEX(mt510_otp_mutex);
static int GroupFlag = 0;
static struct i2c_client *g_pstI2CclientG;
char mt510_otp_buf[MAX_EEPROM_BYTE] = {0};
/************************************************************
 * I2C read function (Custom)
 * Customer's driver can put on here
 * Below is an example
 ************************************************************/
static int iReadRegI2C(struct i2c_client *client,
		u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData)
{
	int i4RetValue = 0;
	struct i2c_msg msg[EEPROM_I2C_MSG_SIZE_READ];

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = a_sizeSendData;
	msg[0].buf = a_pSendData;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = a_sizeRecvData;
	msg[1].buf = a_pRecvData;

	i4RetValue = i2c_transfer(client->adapter, msg,
				EEPROM_I2C_MSG_SIZE_READ);

	if (i4RetValue != EEPROM_I2C_MSG_SIZE_READ) {
		LOG_INF("I2C read failed!!\n");
		return -1;
	}
	return 0;
}

static int sensor_reg_read(struct i2c_client *client, const u16 addr)
{
	u8 buf[2] = {addr >> 8, addr & 0xff};
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr  = client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = buf,
		}, {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 2,
			.buf   = buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	if (ret < 0) {
		LOG_INF("reading register %x from %x failed\n", addr, client->addr);
		return ret;
	}

	return buf[0] & 0xff; /* no sign-extension */
}

static int sensor_reg_write (struct i2c_client *client, const u16 addr, const u8 data)
{
	struct i2c_msg msg;
	unsigned char tx[3];
	int ret;

	msg.addr = client->addr;
	msg.buf = tx;
	msg.len = 3;
	msg.flags = 0;

	tx[0] = addr >> 8;
	tx[1] = addr & 0xff;
	tx[2] = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret == 1)
		return 0;
	else {
		LOG_INF("write register %x from %x failed\n", addr, client->addr);
		return -EIO;
	}
}

static int read_otp_group_flag(uint8_t *data, uint32_t start_addr)
{
	uint16_t flag_addr = MT510_OTP_GROUP_FLAGADDR;
	uint32_t offset = MT510_OTP_GROUP_FLAGADDR - start_addr;//0605
    uint8_t flag_value = data[offset];

    LOG_INF("GroupFlag at 0x%04X (offset %u): 0x%02X\n",
           flag_addr, offset, flag_value);

    return (int)flag_value;  // return flag
}

static int mt510_sensor_otp_read_data(unsigned int ui4_offset,
					unsigned int ui4_length, unsigned char *pinputdata, u8 *data)
{
	int sum = 0, i = 0;
	for (i = 0; i < ui4_length; i++) {
		pinputdata[i] = data[ui4_offset++];

		if (i < ui4_length - 2)
			sum += pinputdata[i];
		pr_debug("dbgmsg - func: %s, pinputdata[%d] = 0x%02x\n", __FUNCTION__, i, pinputdata[i]);
	}

	pr_debug("sum = %x, sum %% 255 = %x, checksum = %x\n", sum, sum % 255, pinputdata[i - 1]);
	if (sum % 255 != pinputdata[i - 1])
		return -1;
	return 0;
}

static int mt510_read_otp(struct i2c_client *client,
				u32 start_addr, u8 *data, u32 otp_len)
{
	int rtn = 0;
	u8 cnt = 0;
	u16 max_len_once = 0x1000;
	u16 addr_offset = 0x1000;
	u16 map_addr = start_addr + addr_offset;

	u8 pu_send_cmd[2] = { (u8) (map_addr >> 8), (u8) (map_addr & 0xFF) };

	if (otp_len > 6144) {
		LOG_INF("otp read len can not over 6K");
		rtn = -1;
		return rtn;
	}
	do {
		/* 1. Enable otp work*/
		sensor_reg_write(client, 0x0d01, 0x00);
		sensor_reg_write(client, 0x0c00, 0x01);
		/* 2. Normal read */
		sensor_reg_write(client, 0x0c19, 0x00);
		/* 3. Set otp read act addr */
		sensor_reg_write(client, 0x0c13, (start_addr >> 8) & 0xff);
		sensor_reg_write(client, 0x0c14, start_addr & 0xff);
		/* 4. Set otp read mode and triggle*/
		sensor_reg_write(client, 0x0c16, 0x50);
		sensor_reg_write(client, 0x0c16, 0x70);
		/* 5. Wait for read complete status setting.*/
		do {
			if (sensor_reg_read(client, 0x0c1a) == 0x01)
				break;
			cnt++;
		} while (cnt < 10);
		if (cnt > 9) {
			LOG_INF("otp read data fail");
			rtn = -1;
			break;
		}
		/* 6. Read otp data once or twice */
		if (otp_len > max_len_once) {
			rtn = iReadRegI2C(client, pu_send_cmd, 2, data, max_len_once);
			pu_send_cmd[0] += (max_len_once >> 8);
			sensor_reg_write(client, 0x0c13, ((start_addr + max_len_once) >> 8) & 0xff);
			sensor_reg_write(client, 0x0c14, (start_addr + max_len_once) & 0xff);
			sensor_reg_write(client, 0x0c16, 0x50);
			sensor_reg_write(client, 0x0c16, 0x70);
			do {
				if (sensor_reg_read(client, 0x0c1a) == 0x01)
					break;
				cnt++;
			} while (cnt < 10);
			if (cnt > 9) {
				LOG_INF("otp read data fail");
				rtn = -1;
				break;
			}
			rtn = iReadRegI2C(client, pu_send_cmd, 2, data + max_len_once, otp_len - max_len_once);
		}
		else
			rtn = iReadRegI2C(client, pu_send_cmd, 2, data, otp_len);
	} while(0);
	/* 7. Exist otp working status */
	sensor_reg_write(client, 0x0c17, 0x01);
	/* 8. Disable otp work */
	sensor_reg_write(client, 0x0c00, 0x00);

	LOG_INF("start_addr = %x size = %d data read = %d\n", start_addr, otp_len, rtn);

	return rtn;
}


static int mt510_sensor_otp_read_module_info(uint8_t *data, uint16_t start_addr, unsigned char *pinputdata)
{
	int ret = MT510_OTP_RET_FAIL;

	//read group flag
	GroupFlag = read_otp_group_flag(data, start_addr);
	LOG_INF("GroupFlag = %d\n", GroupFlag);

	if (GroupFlag == MT510_GROUP1_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_MODULE_GROUP1_STARTADDR - start_addr,
			MT510_OTP_MODULE_LENS, pinputdata,data);
		LOG_INF("mt510_otp group1 ret = %d!\n", ret);
	} else if (GroupFlag == MT510_GROUP2_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_MODULE_GROUP2_STARTADDR - start_addr,
			MT510_OTP_MODULE_LENS, pinputdata,data);
		LOG_INF("mt510_otp group2 ret = %d!\n", ret);
	} else {
		pr_err("mt510_otp invalid flag :0x%x\n",
				GroupFlag);
	}

	return ret;
}

static int mt510_sensor_otp_read_awb_5000k_info(uint8_t *data, uint16_t start_addr, unsigned char *pinputdata)
{
	int ret = MT510_OTP_RET_FAIL;

	if (GroupFlag == MT510_GROUP1_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_AWB5000K_GROUP1_STARTADDR - start_addr,
			MT510_OTP_AWB_LENS, pinputdata,data);
		LOG_INF("mt510_otp group1 ret = %d!\n", ret);
	} else if (GroupFlag == MT510_GROUP2_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_AWB5000K_GROUP2_STARTADDR - start_addr,
			MT510_OTP_AWB_LENS, pinputdata,data);
		LOG_INF("mt510_otp group2 ret = %d!\n", ret);
	} else {
		pr_err("mt510_otp invalid flag :0x%x\n",
				GroupFlag);
	}

	return ret;
}

static int mt510_sensor_otp_read_awb_2850k_info(uint8_t *data, uint16_t start_addr, unsigned char *pinputdata)
{
	int ret = MT510_OTP_RET_FAIL;

	if (GroupFlag == MT510_GROUP1_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_AWB2850K_GROUP1_STARTADDR - start_addr,
			MT510_OTP_AWB_LENS, pinputdata,data);
		LOG_INF("mt510_otp group1 ret = %d!\n", ret);
	} else if (GroupFlag == MT510_GROUP2_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_AWB2850K_GROUP2_STARTADDR - start_addr,
			MT510_OTP_AWB_LENS, pinputdata,data);
		LOG_INF("mt510_otp group2 ret = %d!\n", ret);
	} else {
		pr_err("mt510_otp invalid flag :0x%x\n",
				GroupFlag);
	}

	return ret;
}

static int mt510_sensor_otp_read_lrc_5000k_info(uint8_t *data, uint16_t start_addr, unsigned char *pinputdata)
{
	int ret = MT510_OTP_RET_FAIL;

	if (GroupFlag == MT510_GROUP1_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_LRC5000K_GROUP1_STARTADDR - start_addr,
			MT510_LRC_LENS, pinputdata,data);
		LOG_INF("mt510_otp group1 ret = %d!\n", ret);
	} else if (GroupFlag == MT510_GROUP2_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_LRC5000K_GROUP2_STARTADDR - start_addr,
			MT510_LRC_LENS, pinputdata,data);
		LOG_INF("mt510_otp group2 ret = %d!\n", ret);
	} else {
		pr_err("mt510_otp invalid flag :0x%x\n",
				GroupFlag);
	}

	return ret;
}

static int mt510_sensor_otp_read_lrc_2850k_info(uint8_t *data, uint16_t start_addr, unsigned char *pinputdata)
{
	int ret = MT510_OTP_RET_FAIL;

	if (GroupFlag == MT510_GROUP1_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_LRC2850K_GROUP1_STARTADDR - start_addr,
			MT510_LRC_LENS, pinputdata,data);
		LOG_INF("mt510_otp group1 ret = %d!\n", ret);
	} else if (GroupFlag == MT510_GROUP2_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_LRC2850K_GROUP2_STARTADDR - start_addr,
			MT510_LRC_LENS, pinputdata,data);
		LOG_INF("mt510_otp group2 ret = %d!\n", ret);
	} else {
		pr_err("mt510_otp invalid flag :0x%x\n",
				GroupFlag);
	}

	return ret;
}

static int mt510_sensor_otp_read_sn_info(uint8_t *data, uint16_t start_addr, unsigned char *pinputdata)
{
	int ret = MT510_OTP_RET_FAIL;

	if (GroupFlag == MT510_GROUP1_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_SN_GROUP1_STARTADDR - start_addr,
			MT510_SN_LENS, pinputdata,data);
		LOG_INF("mt510_otp group1 ret = %d!\n", ret);
	} else if (GroupFlag == MT510_GROUP2_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_SN_GROUP2_STARTADDR - start_addr,
			MT510_SN_LENS, pinputdata,data);
		LOG_INF("mt510_otp group2 ret = %d!\n", ret);
	} else {
		pr_err("mt510_otp invalid flag :0x%x\n",
				GroupFlag);
	}

	return ret;
}

static int mt510_sensor_otp_read_lsc_info(uint8_t *data, uint16_t start_addr, unsigned char *pinputdata)
{
	int ret = MT510_OTP_RET_FAIL;

	if (GroupFlag == MT510_GROUP1_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_LSC_GROUP1_STARTADDR - start_addr,
			MT510_LSC_LENS, pinputdata,data);
		LOG_INF("mt510_otp group1 ret = %d!\n", ret);
	} else if (GroupFlag == MT510_GROUP2_FLAG) {
		ret = mt510_sensor_otp_read_data(MT510_OTP_LSC_GROUP2_STARTADDR - start_addr,
			MT510_LSC_LENS, pinputdata,data);
		LOG_INF("mt510_otp group2 ret = %d!\n", ret);
	} else {
		pr_err("mt510_otp invalid flag :0x%x\n",
				GroupFlag);
	}

	return ret;
}

static int mt510_sensor_otp_read_by_group()
{
	int ret = MT510_OTP_RET_FAIL;
	u16 addr = 0x00bc;
	unsigned char *temp_buf = NULL;

    temp_buf = kzalloc(6 * 1024, GFP_KERNEL);
	if(mt510_read_otp(g_pstI2CclientG, addr, temp_buf, MAX_EEPROM_BYTE) == 0) {
		LOG_INF("mt510 read otp in data\n");
	} else {
		goto err_exit;
	}

	ret = mt510_sensor_otp_read_module_info(temp_buf,addr,mt510_otp_buf);
	if (ret == MT510_OTP_RET_FAIL) {
		pr_err("mt510_otp read module info fail\n");
		goto err_exit;
	}

	ret = mt510_sensor_otp_read_awb_5000k_info(temp_buf,addr,&mt510_otp_buf[17]);
	if (ret == MT510_OTP_RET_FAIL) {
		pr_err("mt510_otp read awb info fail\n");
		goto err_exit;
	}

	ret = mt510_sensor_otp_read_awb_2850k_info(temp_buf,addr,&mt510_otp_buf[35]);
	if (ret == MT510_OTP_RET_FAIL) {
		pr_err("mt510_otp read awb info fail\n");
		goto err_exit;
	}

	ret = mt510_sensor_otp_read_lrc_5000k_info(temp_buf,addr,&mt510_otp_buf[53]);
	if (ret == MT510_OTP_RET_FAIL) {
		pr_err("mt510_otp read lrc info fail\n");
		goto err_exit;
	}

	ret = mt510_sensor_otp_read_lrc_2850k_info(temp_buf,addr,&mt510_otp_buf[59]);
	if (ret == MT510_OTP_RET_FAIL) {
		pr_err("mt510_otp read lrc info fail\n");
		goto err_exit;
	}

	ret = mt510_sensor_otp_read_sn_info(temp_buf,addr,&mt510_otp_buf[65]);
	if (ret == MT510_OTP_RET_FAIL) {
		pr_err("mt510_otp read sn info fail\n");
		goto err_exit;
	}

	ret = mt510_sensor_otp_read_lsc_info(temp_buf,addr,&mt510_otp_buf[90]);
	if (ret == MT510_OTP_RET_FAIL) {
		pr_err("mt510_otp read lsc info fail\n");
		goto err_exit;
	}

	kfree(temp_buf);
	temp_buf = NULL;
	return 0;

err_exit:
	kfree(temp_buf);
	return ret;

}

unsigned int mt510_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	/* otp_offset need update when otp data write offset confirm */
	int ret = MT510_OTP_RET_FAIL;
	g_pstI2CclientG = client;

	LOG_INF("mt510 mt510_read_region addr = %x, size = %d, data = %p\n", addr, size, data);

	if (mt510_otp_buf[READ_FLAG_ADDR]) {
		pr_info("read otp data from mt510_read_region: addr 0x%x, size %d", addr, size);
		memcpy((void *)data, &mt510_otp_buf[addr], size);
		return size;
	}

	ret = mt510_sensor_otp_read_by_group();

	if (!ret) {
		mt510_otp_buf[READ_FLAG_ADDR] = 1;
	}

	if (NULL != data) {
		memcpy((void *)data, &mt510_otp_buf[addr], size);
	}
	return ret;
}