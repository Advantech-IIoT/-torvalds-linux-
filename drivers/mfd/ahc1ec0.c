// SPDX-License-Identifier: GPL-2.0-only
/*****************************************************************************
 *                 Copyright (c) 2018, Advantech Automation Corp.
 *     THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
 *              INFORMATION WHICH IS THE PROPERTY OF ADVANTECH AUTOMATION CORP.
 *
 *   ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
 *              ADVANTECH AUTOMATION CORP., IS STRICTLY PROHIBITED.
 *****************************************************************************
 *
 * File:        ahc1ec0.c
 * Version:     1.00  <10/10/2014>
 * Author:      Sun.Lang
 *
 * Description: The ahc1ec0 is multifunctional driver for controlling EC chip.
 *
 *
 * Status:      working
 *
 * Change Log:
 *              Version 1.00 <10/10/2014> Sun.Lang
 *              - Initial version
 *              Version 1.01 <11/05/2015> Jiangwei.Zhu
 *              - Modify read_ad_value() function.
 *              - Add smbus_read_byte() function.
 *              - Add smbus_write_byte() function.
 *              - Add wait_smbus_protocol_finish() function.
 *              Version 1.02 <03/04/2016> Jiangwei.Zhu
 *              - Add smbus_read_word() function.
 *              Version 1.03 <01/22/2017> Ji.Xu
 *              - Add detect to Advantech porduct name "ECU".
 *              Version 1.04 <09/20/2017> Ji.Xu
 *              - Update to support detect Advantech product name in UEFI
 *                BIOS(DMI).
 *              Version 1.05 <11/02/2017> Ji.Xu
 *              - Fixed issue: Cache coherency error when exec 'ioremap_uncache()'
 *                in kernel-4.10.
 *              Version 1.06 <10/16/2020> Shihlun.Lin
 *              - Update: Replace ioremap_nocache() with ioremap_uc() since
 *                ioremap_uc() was used on the entire PCI BAR.
 ******************************************************************************/

#include <linux/kconfig.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/mfd/ahc1ec0.h>
#include <linux/acpi.h>
#include <linux/mod_devicetable.h>
#include <linux/mfd/core.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define ADVANTECH_EC_NAME      "ahc1ec0"
#define ADVANTECH_EC_MFD_VER    "2.0.0"
#define ADVANTECH_EC_MFD_DATE   "10/16/2020"

struct mutex lock;

enum {
	ADVEC_SUBDEV_BRIGHTNESS = 0,
	ADVEC_SUBDEV_EEPROM,
	ADVEC_SUBDEV_GPIO,
	ADVEC_SUBDEV_HWMON,
	ADVEC_SUBDEV_LED,
	ADVEC_SUBDEV_WDT,
	ADVEC_SUBDEV_MAX,
};

static int wait_ibf(void)
{
	int i;

	for (i = 0; i < EC_MAX_TIMEOUT_COUNT; i++) {
		if ((inb(EC_COMMAND_PORT) & 0x02) == 0)
			return 0;

		udelay(EC_UDELAY_TIME);
	}

	return -ETIMEDOUT;
}

/* Wait OBF (Output buffer full) set */
static int wait_obf(void)
{
	int i;

	for (i = 0; i < EC_MAX_TIMEOUT_COUNT; i++) {
		if ((inb(EC_COMMAND_PORT) & 0x01) != 0)
			return 0;

		udelay(EC_UDELAY_TIME);
	}

	return -ETIMEDOUT;
}

/* Read data from EC HW ram */
static int read_hw_ram(uchar addr, uchar *data)
{
	int ret;

	mutex_lock(&lock);

	/* Step 0. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 1. Send "read EC HW ram" command to EC Command port */
	outb(EC_HW_RAM_READ, EC_COMMAND_PORT);

	/* Step 2. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 3. Send "EC HW ram" address to EC Data port */
	outb(addr, EC_STATUS_PORT);

	/* Step 4. Wait OBF set */
	ret = wait_obf();
	if (ret)
		goto error;

	/* Step 5. Get "EC HW ram" data from EC Data port */
	*data = inb(EC_STATUS_PORT);

	mutex_unlock(&lock);

	return ret;

error:
	mutex_unlock(&lock);
	pr_err("%s: Wait for IBF or OBF too long. line: %d", __func__, __LINE__);
	return ret;
}

/* Write data to EC HW ram */
int write_hw_ram(uchar addr, uchar data)
{
	int ret;

	mutex_lock(&lock);
	/* Step 0. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 1. Send "write EC HW ram" command to EC command port */
	outb(EC_HW_RAM_WRITE, EC_COMMAND_PORT);

	/* Step 2. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 3. Send "EC HW ram" address to EC Data port */
	outb(addr, EC_STATUS_PORT);

	/* Step 4. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 5. Send "EC HW ram" data to EC Data port */
	outb(data, EC_STATUS_PORT);

	mutex_unlock(&lock);

	return 0;

error:
	mutex_unlock(&lock);
	pr_err("%s: Wait for IBF too long. line: %d", __func__, __LINE__);
	return ret;
}
EXPORT_SYMBOL_GPL(write_hw_ram);

static int wait_smbus_protocol_finish(void)
{
	uchar addr, data;
	int retry = 1000;

	do {
		addr = EC_SMBUS_PROTOCOL;
		data = 0xFF;
		if (!read_hw_ram(addr, &data))
			return 0;

		if (data == 0)
			return 0;

		udelay(EC_UDELAY_TIME);
	} while (retry-- > 0);

	return -ETIMEDOUT;
}

/* Get dynamic control table */
static int adv_get_dynamic_tab(struct adv_ec_platform_data *pdata)
{
	int i, ret;
	uchar pin_tmp, device_id;

	mutex_lock(&lock);

	for (i = 0; i < EC_MAX_TBL_NUM; i++) {
		pdata->dym_tbl[i].DeviceID = 0xff;
		pdata->dym_tbl[i].HWPinNumber = 0xff;
	}

	for (i = 0; i < EC_MAX_TBL_NUM; i++) {
		/* Step 0. Wait IBF clear */
		ret = wait_ibf();
		if (ret)
			goto error;

		/*
		 *  Step 1. Write 0x20 to 0x29A
		 *  Send write item number into index command
		 */
		outb(EC_TBL_WRITE_ITEM, EC_COMMAND_PORT);

		/* Step 2. Wait IBF clear */
		ret = wait_ibf();
		if (ret)
			goto error;

		/*
		 *  Step 3. Write item number to 0x299
		 *  Write item number to index. Item number is limited in range 0 to 31
		 */
		outb(i, EC_STATUS_PORT);

		/* Step 4. Wait OBF set */
		ret = wait_obf();
		if (ret)
			goto error;

		/*
		 *  Step 5. Read 0x299 port
		 *  If item is defined, EC will return item number.
		 *  If table item is not defined, EC will return 0xFF.
		 */
		if (inb(EC_STATUS_PORT) == 0xff) {
			mutex_unlock(&lock);
			return -EINVAL;
		}

		/* Step 6. Wait IBF clear */
		ret = wait_ibf();
		if (ret)
			goto error;

		/*
		 *  Step 7. Write 0x21 to 0x29A
		 *  Send read HW pin number command
		 */
		outb(EC_TBL_GET_PIN, EC_COMMAND_PORT);

		/* Step 8. Wait OBF set */
		ret = wait_obf();
		if (ret)
			goto error;

		/*
		 *  Step 9. Read 0x299 port
		 *  EC will return current item HW pin number
		 */
		pin_tmp = inb(EC_STATUS_PORT) & 0xff;

		/* Step 10. Wait IBF clear */
		ret = wait_ibf();
		if (ret)
			goto error;

		if (pin_tmp == 0xff) {
			mutex_unlock(&lock);
			return -EINVAL;
		}

		/*
		 *  Step 11. Write 0x22 to 0x29A
		 *  Send read device id command
		 */
		outb(EC_TBL_GET_DEVID, EC_COMMAND_PORT);

		/* Step 12. Wait OBF set */
		ret = wait_obf();
		if (ret)
			goto error;

		/*
		 *  Step 13. Read 0x299 port
		 *  EC will return current item Device ID
		 */
		device_id = inb(EC_STATUS_PORT) & 0xff;

		/* Step 14. Save data to a database */
		pdata->dym_tbl[i].DeviceID = device_id;
		pdata->dym_tbl[i].HWPinNumber = pin_tmp;
	}

	mutex_unlock(&lock);
	return 0;

error:
	mutex_unlock(&lock);
	pr_err("%s: Wait for IBF or OBF too long. line: %d", __func__, __LINE__);
	return ret;
}

int read_ad_value(uchar hwpin, uchar multi)
{
	int ret;
	u32 ret_val;
	uchar LSB, MSB;

	mutex_lock(&lock);
	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_AD_INDEX_WRITE, EC_COMMAND_PORT);
	ret = wait_ibf();
	if (ret)
		goto error;

	outb(hwpin, EC_STATUS_PORT);
	ret = wait_obf();
	if (ret)
		goto error;

	if (inb(EC_STATUS_PORT) == 0xff) {
		mutex_unlock(&lock);
		return -1;
	}

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_AD_LSB_READ, EC_COMMAND_PORT);
	ret = wait_obf();
	if (ret)
		goto error;

	LSB = inb(EC_STATUS_PORT);
	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_AD_MSB_READ, EC_COMMAND_PORT);
	ret = wait_obf();
	if (ret)
		goto error;

	MSB = inb(EC_STATUS_PORT);
	ret_val = ((MSB << 8) | LSB) & 0x03FF;
	ret_val = ret_val * multi * 100;

	mutex_unlock(&lock);
	return ret_val;

error:
	mutex_unlock(&lock);
	pr_warn("%s: Wait for IBF or OBF too long. line: %d", __func__, __LINE__);
	return ret;
}
EXPORT_SYMBOL_GPL(read_ad_value);

int read_acpi_value(uchar addr, uchar *pvalue)
{
	int ret;
	uchar value;

	mutex_lock(&lock);

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_ACPI_RAM_READ, EC_COMMAND_PORT);
	ret = wait_ibf();
	if (ret)
		goto error;

	outb(addr, EC_STATUS_PORT);
	ret = wait_obf();
	if (ret)
		goto error;

	value = inb(EC_STATUS_PORT);
	*pvalue = value;
	mutex_unlock(&lock);

	return 0;

error:
	mutex_unlock(&lock);
	pr_warn("%s: Wait for IBF or OBF too long. line: %d", __func__, __LINE__);
	return ret;
}
EXPORT_SYMBOL_GPL(read_acpi_value);

int write_acpi_value(uchar addr, uchar value)
{
	int ret;

	mutex_lock(&lock);

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_ACPI_DATA_WRITE, EC_COMMAND_PORT);

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(addr, EC_STATUS_PORT);

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(value, EC_STATUS_PORT);

	mutex_unlock(&lock);
	return 0;

error:
	mutex_unlock(&lock);
	pr_warn("%s: Wait for IBF  too long. line: %d", __func__, __LINE__);
	return ret;
}

int read_gpio_status(uchar PinNumber, uchar *pvalue)
{
	int ret;

	uchar gpio_status_value;

	mutex_lock(&lock);

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_GPIO_INDEX_WRITE, EC_COMMAND_PORT);

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(PinNumber, EC_STATUS_PORT);

	ret = wait_obf();
	if (ret)
		goto error;

	if (inb(EC_STATUS_PORT) == 0xff) {
		pr_err("%s: Read Pin Number error!!", __func__);
		mutex_unlock(&lock);
		return -1;
	}

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_GPIO_STATUS_READ, EC_COMMAND_PORT);

	ret = wait_obf();
	if (ret)
		goto error;

	gpio_status_value = inb(EC_STATUS_PORT);
	*pvalue = gpio_status_value;
	mutex_unlock(&lock);
	return 0;

error:
	mutex_unlock(&lock);
	pr_warn("%s: Wait for IBF or OBF too long. line: %d", __func__, __LINE__);
	return ret;
}

int write_gpio_status(uchar PinNumber, uchar value)
{
	int ret;

	mutex_lock(&lock);

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_GPIO_INDEX_WRITE, EC_COMMAND_PORT);
	ret = wait_ibf();
	if (ret)
		goto error;

	outb(PinNumber, EC_STATUS_PORT);
	ret = wait_obf();
	if (ret)
		goto error;

	if (inb(EC_STATUS_PORT) == 0xff) {
		pr_err("%s: Read Pin Number error!!", __func__);
		mutex_unlock(&lock);
		return -1;
	}

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_GPIO_STATUS_WRITE, EC_COMMAND_PORT);
	ret = wait_ibf();
	if (ret)
		goto error;

	outb(value, EC_STATUS_PORT);
	mutex_unlock(&lock);
	return 0;

error:
	mutex_unlock(&lock);
	pr_warn("%s: Wait for IBF or OBF too long. line: %d", __func__, __LINE__);
	return ret;
}

int read_gpio_dir(uchar PinNumber, uchar *pvalue)
{
	int ret;
	uchar gpio_dir_value;

	mutex_lock(&lock);

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_GPIO_INDEX_WRITE, EC_COMMAND_PORT);
	ret = wait_ibf();
	if (ret)
		goto error;

	outb(PinNumber, EC_STATUS_PORT);
	ret = wait_obf();
	if (ret)
		goto error;

	if (inb(EC_STATUS_PORT) == 0xff) {
		pr_err("%s: Read Pin Number error!!", __func__);
		mutex_unlock(&lock);
		return -1;
	}

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_GPIO_DIR_READ, EC_COMMAND_PORT);
	ret = wait_obf();
	if (ret)
		goto error;

	gpio_dir_value = inb(EC_STATUS_PORT);
	*pvalue = gpio_dir_value;
	mutex_unlock(&lock);
	return 0;

error:
	mutex_unlock(&lock);
	pr_warn("%s: Wait for IBF or OBF too long. line: %d", __func__, __LINE__);
	return ret;
}

int write_gpio_dir(uchar PinNumber, uchar value)
{
	int ret;

	mutex_lock(&lock);

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_GPIO_INDEX_WRITE, EC_COMMAND_PORT);
	ret = wait_ibf();
	if (ret)
		goto error;

	outb(PinNumber, EC_STATUS_PORT);
	ret = wait_obf();
	if (ret)
		goto error;

	if (inb(EC_STATUS_PORT) == 0xff) {
		pr_err("%s: Read Pin Number error!!", __func__);
		mutex_unlock(&lock);
		return -1;
	}

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(EC_GPIO_DIR_WRITE, EC_COMMAND_PORT);
	ret = wait_ibf();
	if (ret)
		goto error;

	outb(value, EC_STATUS_PORT);
	mutex_unlock(&lock);
	return 0;

error:
	mutex_unlock(&lock);
	pr_warn("%s: Wait for IBF or OBF too long. line: %d", __func__, __LINE__);
	return ret;
}

/* Write data to EC HW ram */
int write_hw_extend_ram(uchar addr, uchar data)
{
	int ret;

	mutex_lock(&lock);
	/* Step 0. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 1. Send "write EC HW ram" command to EC command port */
	outb(EC_HW_EXTEND_RAM_WRITE, EC_COMMAND_PORT);

	/* Step 2. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 3. Send "EC HW ram" address to EC Data port */
	outb(addr, EC_STATUS_PORT);

	/* Step 4. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 5. Send "EC HW ram" data to EC Data port */
	outb(data, EC_STATUS_PORT);

	mutex_unlock(&lock);

	return 0;

error:
	mutex_unlock(&lock);
	pr_debug("%s: Wait for IBF too long. line: %d", __func__, __LINE__);
	return ret;
}

int write_hwram_command(uchar data)
{
	int ret;

	mutex_lock(&lock);

	ret = wait_ibf();
	if (ret)
		goto error;

	outb(data, EC_COMMAND_PORT);
	mutex_unlock(&lock);
	return 0;

error:
	mutex_unlock(&lock);
	pr_debug("%s: Wait for IBF too long. line: %d", __func__, __LINE__);
	return ret;
}
EXPORT_SYMBOL_GPL(write_hwram_command);

int smbus_read_word(struct EC_SMBUS_WORD_DATA *ptr_ec_smbus_word_data)
{
	int ret;

	uchar sm_ready, LSB, MSB, addr, data;
	unsigned short Value = 0;

	/*  Step 1. Select SMBus channel */
	addr = EC_SMBUS_CHANNEL;
	data = ptr_ec_smbus_word_data->Channel;
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Select SMBus channel Failed");
		goto error;
	}

	/* Step 2. Set SMBUS device address EX: 0x98 */
	addr = EC_SMBUS_SLV_ADDR;
	data = (ptr_ec_smbus_word_data->Address);
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Select SMBus dev addr:0x%02X Failed", ptr_ec_smbus_word_data->Address);
		goto error;
	}

	/* Step 3. Set Chip (EX: INA266) Register Address */
	addr = EC_SMBUS_CMD;
	data = ptr_ec_smbus_word_data->Register;
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Select Chip Register Addr:0x%02X Failed", ptr_ec_smbus_word_data->Register);
		goto error;
	}

	/* Step 4. Set EC SMBUS read word Mode */
	addr = EC_SMBUS_PROTOCOL;
	data = SMBUS_WORD_READ;
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Set EC SMBUS read Byte Mode Failed");
		goto error;
	}

	/* Step 5. Check EC Smbus states */
	ret = wait_smbus_protocol_finish();
	if (ret) {
		pr_err("Wait SmBus Protocol Finish Failed!!");
		goto error;
	}

	addr = EC_SMBUS_STATUS;
	ret = read_hw_ram(addr, &data);
	if (ret) {
		pr_err("Check EC Smbus states Failed");
		goto error;
	}
	sm_ready = data;

	/* check no error */
	if (sm_ready != 0x80) {
		pr_err("SMBUS ERR:0x%02X", sm_ready);
		goto error;
	}

	/* Step 6. Get Value */
	addr = EC_SMBUS_DAT_OFFSET(0);
	ret = read_hw_ram(addr, &data);
	if (ret) {
		pr_err("Get Value Failed");
		goto error;
	}
	MSB = data;

	addr = EC_SMBUS_DAT_OFFSET(1);
	ret = read_hw_ram(addr, &data);
	if (ret) {
		pr_err("Get Value Failed");
		goto error;
	}
	LSB = data;

	Value = (MSB << 8) | LSB;
	ptr_ec_smbus_word_data->Value = Value;

	return 0;

error:
	pr_err("%s: Exception!", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(smbus_read_word);

int smbus_read_byte(struct EC_SMBUS_READ_BYTE *ptr_ec_smbus_read_byte)
{
	int ret;
	uchar sm_ready, addr, data;

	/* CHECK_PARAMETER */
	if (ptr_ec_smbus_read_byte == NULL)
		return -EINVAL;

	/* Step 1. Select SMBus channel */
	addr = EC_SMBUS_CHANNEL;
	data = ptr_ec_smbus_read_byte->Channel;
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Select SMBus channel Failed");
		goto error;
	}

	/* Step 2. Set SMBUS device address EX: 0x98 */
	addr = EC_SMBUS_SLV_ADDR;
	data = (ptr_ec_smbus_read_byte->Address);
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Select SMBus dev addr:0x%02X Failed", ptr_ec_smbus_read_byte->Address);
		goto error;
	}

	/* Step 3. Set Chip (EX: MCP23008) Register Address */
	addr = EC_SMBUS_CMD;
	data = ptr_ec_smbus_read_byte->Register;
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Select Chip Register Addr:0x%02X Failed", ptr_ec_smbus_read_byte->Register);
		goto error;
	}

	/* Step 4. Set EC SMBUS read Byte Mode */
	addr = EC_SMBUS_PROTOCOL;
	data = SMBUS_BYTE_READ;
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Set EC SMBUS read Byte Mode Failed");
		goto error;
	}

	/* Step 5. Check EC Smbus states */
	ret = wait_smbus_protocol_finish();
	if (ret) {
		pr_err("Wait SmBus Protocol Finish Failed!!");
		goto error;
	}

	addr = EC_SMBUS_STATUS;
	ret = read_hw_ram(addr, &data);
	if (ret) {
		pr_err("Check EC Smbus states Failed");
		goto error;
	}
	sm_ready = data;

	/* check no error */
	if (sm_ready != 0x80) {
		pr_err("SMBUS ERR:(0x%02X)", sm_ready);
		goto error;
	}

	/* Step 6. Get Value */
	addr = EC_SMBUS_DATA;
	ret = read_hw_ram(addr, &data);
	if (ret) {
		pr_err("Get Value Failed");
		goto error;
	}

	ptr_ec_smbus_read_byte->Data = (data & 0xFF);
	return 0;

error:
	pr_err("%s: Exception!", __func__);
	return ret;
}
EXPORT_SYMBOL_GPL(smbus_read_byte);

int smbus_write_byte(struct EC_SMBUS_WRITE_BYTE *ptr_ec_smbus_write_byte)
{
	int ret;
	uchar sm_ready, addr, data;

	/* Step 1. Select SMBus channel */
	addr = EC_SMBUS_CHANNEL;
	data = ptr_ec_smbus_write_byte->Channel;
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Select SMBus channel Failed");
		goto error;
	}

	/* Step 2. Set SMBUS device address EX: 0x98 */
	addr = EC_SMBUS_SLV_ADDR;
	data = (ptr_ec_smbus_write_byte->Address);
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Select SMBus dev addr:0x%02X Failed", ptr_ec_smbus_write_byte->Address);
		goto error;
	}

	/* Step 3. Set Chip (EX: MCP23008) Register Address */
	addr = EC_SMBUS_CMD;
	data = ptr_ec_smbus_write_byte->Register;
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Select Chip reg addr:0x%02X Failed", ptr_ec_smbus_write_byte->Register);
		goto error;
	}

	/* Step 4. Set Data to SMBUS */
	addr = EC_SMBUS_DATA;
	data = ptr_ec_smbus_write_byte->Data;
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Set Data 0x%02X to SMBUS Failed", ptr_ec_smbus_write_byte->Data);
		goto error;
	}

	/* Step 5. Set EC SMBUS write Byte Mode */
	addr = EC_SMBUS_PROTOCOL;
	data = SMBUS_BYTE_WRITE;
	ret = write_hw_ram(addr, data);
	if (ret) {
		pr_err("Set EC SMBUS write Byte Mode Failed");
		goto error;
	}

	/* Step 6. Check EC Smbus states */
	ret = wait_smbus_protocol_finish();
	if (ret) {
		pr_err("Wait SmBus Protocol Finish Failed!!");
		goto error;
	}

	addr = EC_SMBUS_STATUS;
	ret = read_hw_ram(addr, &data);
	if (ret) {
		pr_err("Check EC Smbus states Failed");
		goto error;
	}
	sm_ready = data;

	/* check no error */
	if (sm_ready != 0x80) {
		pr_err("SMBUS ERR:(0x%02X)", sm_ready);
		goto error;
	}
	mutex_unlock(&lock);
	return 0;

error:
	pr_err("%s: Exception!", __func__);
	mutex_unlock(&lock);
	return ret;
}

/* Get One Key Recovery status */
int read_onekey_status(uchar addr, uchar *pdata)
{
	int ret;

	mutex_lock(&lock);

	/* Init return value */
	*pdata = 0;

	/* Step 0. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 1. Send "One Key Recovery" command to EC Command port */
	outb(EC_ONE_KEY_FLAG, EC_COMMAND_PORT);

	/* Step 2. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 3. Send "One Key Recovery function" address to EC Data port */
	outb(addr, EC_STATUS_PORT);

	/* Step 4. Wait OBF set */
	ret = wait_obf();
	if (ret)
		goto error;

	/* Step 5. Get "One Key Recovery function" data from EC Data port */
	*pdata = inb(EC_STATUS_PORT);

	pr_debug("%s: data= %d, line: %d", __func__, *pdata, __LINE__);
	mutex_unlock(&lock);
	return 0;

error:
	mutex_unlock(&lock);
	pr_warn("%s: Wait for IBF or OBF too long. line: %d", __func__, __LINE__);
	return ret;
}

/* Set One Key Recovery status */
int write_onekey_status(uchar addr)
{
	int ret;

	mutex_lock(&lock);

	/* Step 0. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 1. Send "One Key Recovery" command to EC Command port */
	outb(EC_ONE_KEY_FLAG, EC_COMMAND_PORT);

	/* Step 2. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 3. Send "One Key Recovery function" address to EC Data port */
	outb(addr, EC_STATUS_PORT);

	mutex_unlock(&lock);

	pr_debug("%s: addr= %d, line: %d", __func__, addr, __LINE__);
	return 0;

error:
	mutex_unlock(&lock);
	pr_debug("%s: Wait for IBF too long. line: %d", __func__, __LINE__);
	return ret;
}

/* EC OEM get status */
int ec_oem_get_status(uchar addr, uchar *pdata)
{
	int ret;

	mutex_lock(&lock);

	/* Init return value */
	*pdata = 0;

	/* Step 0. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 1. Send "ASG OEM" command to EC Command port */
	outb(EC_ASG_OEM, EC_COMMAND_PORT);

	/* Step 2. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 3. Send "ASG OEM STATUS READ" address to EC Data port */
	outb(EC_ASG_OEM_READ, EC_STATUS_PORT);

	/* Step 4. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 5. Send "OEM STATUS" address to EC Data port */
	outb(addr, EC_STATUS_PORT);

	/* Step 6. Wait OBF set */
	ret = wait_obf();
	if (ret)
		goto error;

	/* Step 7. Get "OEM STATUS" data from EC Data port */
	*pdata = inb(EC_STATUS_PORT);

	pr_debug("%s: data= %d, line: %d", __func__, *pdata, __LINE__);
	mutex_unlock(&lock);
	return 0;

error:
	mutex_unlock(&lock);
	pr_warn("%s: Wait for IBF or OBF too long. line: %d", __func__, __LINE__);
	return ret;
}

/* EC OEM set status */
int ec_oem_set_status(uchar addr, uchar pdata)
{
	int ret;

	mutex_lock(&lock);

	/* Step 0. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 1. Send "ASG OEM" command to EC Command port */
	outb(EC_ASG_OEM, EC_COMMAND_PORT);

	/* Step 2. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 3. Send "ASG OEM STATUS WRITE" address to EC Data port */
	outb(EC_ASG_OEM_WRITE, EC_STATUS_PORT);

	/* Step 4. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 5. Send "OEM STATUS" address to EC Data port */
	outb(addr, EC_STATUS_PORT);

	/* Step 6. Wait IBF clear */
	ret = wait_ibf();
	if (ret)
		goto error;

	/* Step 7. Send "OEM STATUS" status to EC Data port */
	outb(pdata, EC_STATUS_PORT);

	pr_debug("%s: data= %d, line: %d", __func__, pdata, __LINE__);
	mutex_unlock(&lock);
	return 0;

error:
	mutex_unlock(&lock);
	pr_warn("%s: Wait for IBF or OBF too long. line: %d", __func__, __LINE__);

	return ret;
}

static int adv_ec_get_productname(char *product)
{
	static unsigned char *uc_ptaddr;
	static unsigned char *uc_epsaddr;
	int index = 0, eps_table;
	int i = 0;
	int length = 0;
	int type0_str = 0;
	int type1_str = 0;
	int is_advantech = 0;

	uc_ptaddr = ioremap(AMI_UEFI_ADVANTECH_BOARD_NAME_ADDRESS,
			AMI_UEFI_ADVANTECH_BOARD_NAME_LENGTH);
	if (!uc_ptaddr) {
		pr_err("Error: ioremap_uc()");
		return -ENXIO;
	}

	/* Try to Read the product name from UEFI BIOS(DMI) EPS table */
	for (index = 0; index < AMI_UEFI_ADVANTECH_BOARD_NAME_LENGTH; index++) {
		if (uc_ptaddr[index] == '_'
				&& uc_ptaddr[index+0x1] == 'S'
				&& uc_ptaddr[index+0x2] == 'M'
				&& uc_ptaddr[index+0x3] == '_'
				&& uc_ptaddr[index+0x10] == '_'
				&& uc_ptaddr[index+0x11] == 'D'
				&& uc_ptaddr[index+0x12] == 'M'
				&& uc_ptaddr[index+0x13] == 'I'
				&& uc_ptaddr[index+0x14] == '_'
				) {
			eps_table = 1;
			break;
		}
	}

	/* If EPS table exist, read type1(system information) */
	if (eps_table) {
		uc_epsaddr =
			(char *)ioremap(((unsigned int *)&uc_ptaddr[index+0x18])[0],
			((unsigned short *)&uc_ptaddr[index+0x16])[0]);
		if (!uc_epsaddr) {
			uc_epsaddr =
				(char *)ioremap_uc(((unsigned int *)&uc_ptaddr[index+0x18])[0],
				((unsigned short *)&uc_ptaddr[index+0x16])[0]);
			if (!uc_epsaddr) {
				pr_err("Error: both ioremap() and ioremap_uc() exec failed!");
				return -ENXIO;
			}
		}

		type0_str = (int)uc_epsaddr[1];
		for (i = type0_str; i < (type0_str+512); i++) {
			if (uc_epsaddr[i] == 0 && uc_epsaddr[i+1] == 0 && uc_epsaddr[i+2] == 1) {
				type1_str = i + uc_epsaddr[i+3];
				break;
			}
		}
		for (i = type1_str; i < (type1_str+512); i++) {
			if (!strncmp(&uc_epsaddr[i], "Advantech", 9))
				is_advantech = 1;

			if (uc_epsaddr[i] == 0) {
				i++;
				type1_str = i;
				break;
			}
		}
		length = 2;
		while ((uc_epsaddr[type1_str + length] != 0)
				&& (length < AMI_UEFI_ADVANTECH_BOARD_NAME_LENGTH)) {
			length += 1;
		}
		memmove(product, &uc_epsaddr[type1_str], length);
		iounmap((void *)uc_epsaddr);
		if (is_advantech) {
			iounmap((void *)uc_ptaddr);
			return 0;
		}
	}

	/* It is an old BIOS, read from 0x000F0000 */
	for (index = 0; index < (AMI_UEFI_ADVANTECH_BOARD_NAME_LENGTH - 3); index++) {
		if (!strncmp(&uc_ptaddr[index], "TPC", 3)
				|| !strncmp(&uc_ptaddr[index], "UNO", 3)
				|| !strncmp(&uc_ptaddr[index], "ITA", 3)
				|| !strncmp(&uc_ptaddr[index], "MIO", 3)
				|| !strncmp(&uc_ptaddr[index], "ECU", 3)
				|| !strncmp(&uc_ptaddr[index], "APAX", 4))
			break;
	}

	if (index == (AMI_UEFI_ADVANTECH_BOARD_NAME_LENGTH - 3)) {
		pr_err("%s: Can't find the product name, line: %d", __func__, __LINE__);
		product[0] = '\0';
		iounmap((void *)uc_ptaddr);
		return -ENODATA;
	}

	/* Use char "Space" (ASCII code: 32) to check the end of the Product Name. */
	for (i = 0; (uc_ptaddr[index+i] != 32) && (i < 31); i++)
		product[i] = uc_ptaddr[index+i];

	product[i] = '\0';
	pr_info("%s: BIOS Product Name = %s, line: %d", __func__, product, __LINE__);

	iounmap((void *)uc_ptaddr);

	return 0;
}

static const struct mfd_cell adv_ec_sub_cells[] = {
	{ .name = "adv-ec-brightness", },
	{ .name = "adv-ec-eeprom", },
	{ .name = "adv-ec-gpio", },
	{ .name = "adv-ec-hwmon", },
	{ .name = "adv-ec-led", },
	{ .name = "adv-ec-wdt", },
};

static int adv_ec_init_ec_data(struct adv_ec_platform_data *pdata)
{
	int ret;

	pdata->sub_dev_mask = 0;
	pdata->sub_dev_nb = 0;
	pdata->dym_tbl = NULL;
	pdata->bios_product_name = NULL;

	/* Get product name */
	pdata->bios_product_name = kmalloc(AMI_ADVANTECH_BOARD_ID_LENGTH, GFP_KERNEL);
	if (!pdata->bios_product_name)
		return -ENOMEM;

	memset(pdata->bios_product_name, 0, AMI_ADVANTECH_BOARD_ID_LENGTH);
	ret = adv_ec_get_productname(pdata->bios_product_name);
	if (ret)
		return ret;

	/* Get pin table */
	pdata->dym_tbl = kmalloc(EC_MAX_TBL_NUM*sizeof(struct Dynamic_Tab), GFP_KERNEL);
	if (!pdata->dym_tbl)
		return -ENOMEM;

	ret = adv_get_dynamic_tab(pdata);

	return 0;
}

static int adv_ec_parse_prop(struct adv_ec_platform_data *pdata)
{
	int i, ret;
	u32 nb, sub_dev[ADVEC_SUBDEV_MAX];

	ret = device_property_read_u32(pdata->dev, "advantech,sub-dev-nb", &nb);
	if (ret < 0) {
		dev_err(pdata->dev, "get sub-dev-nb failed! (%d)", ret);
		return ret;
	}
	pdata->sub_dev_nb = nb;

	ret = device_property_read_u32_array(pdata->dev, "advantech,sub-dev", sub_dev, nb);
	if (ret < 0) {
		dev_err(pdata->dev, "get sub-dev failed! (%d)", ret);
		return ret;
	}

	for (i = 0; i < nb; i++) {
		switch (sub_dev[i]) {
		case ADVEC_SUBDEV_BRIGHTNESS:
		case ADVEC_SUBDEV_EEPROM:
		case ADVEC_SUBDEV_GPIO:
		case ADVEC_SUBDEV_HWMON:
		case ADVEC_SUBDEV_LED:
		case ADVEC_SUBDEV_WDT:
			pdata->sub_dev_mask |= BIT(sub_dev[i]);
			break;
		default:
			dev_err(pdata->dev, "invalid prop value(%d)!", sub_dev[i]);
		}
	}
	dev_info(pdata->dev, "sub-dev mask = 0x%x", pdata->sub_dev_mask);

	return 0;
}

static int adv_ec_probe(struct platform_device *pdev)
{
	int ret, i;
	struct device *dev = &pdev->dev;
	struct adv_ec_platform_data *adv_ec_data;

	adv_ec_data = kmalloc(sizeof(struct adv_ec_platform_data), GFP_KERNEL);
	if (!adv_ec_data) {
		ret = -ENOMEM;
		goto err_plat_data;
	}

	dev_set_drvdata(dev, (void *)adv_ec_data);
	adv_ec_data->dev = dev;

	mutex_init(&lock);

	ret = adv_ec_init_ec_data(adv_ec_data);
	if (ret)
		goto err_init_data;

	ret = adv_ec_parse_prop(adv_ec_data);
	if (ret)
		goto err_prop;

	for (i = 0; i < ARRAY_SIZE(adv_ec_sub_cells); i++) {
		if (adv_ec_data->sub_dev_mask & BIT(i)) {
			ret = mfd_add_hotplug_devices(dev, &adv_ec_sub_cells[i], 1);
			if (ret)
				dev_err(dev, "failed to add %s subdevice: %d",
					adv_ec_sub_cells[i].name, ret);
		}
	}

	dev_info(&pdev->dev, "Ver:%s, Data:%s, probe done",
			ADVANTECH_EC_MFD_VER, ADVANTECH_EC_MFD_DATE);

	return 0;

err_prop:
err_init_data:
	kfree(adv_ec_data->dym_tbl);
	kfree(adv_ec_data->bios_product_name);
	kfree(adv_ec_data);

err_plat_data:
	return ret;
}

static int adv_ec_remove(struct platform_device *pdev)
{
	struct adv_ec_platform_data *adv_ec_data;

	adv_ec_data = (struct adv_ec_platform_data *)dev_get_drvdata(&pdev->dev);

	kfree(adv_ec_data->dym_tbl);
	kfree(adv_ec_data->bios_product_name);
	kfree(adv_ec_data);

	mfd_remove_devices(&pdev->dev);
	mutex_destroy(&lock);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id adv_ec_of_match[] = {
	{ .compatible = "advantech,ahc1ec0", },
	{}
};
MODULE_DEVICE_TABLE(of, adv_ec_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id adv_ec_acpi_match[] = {
	{"AHC1EC0", },
	{ },
};
MODULE_DEVICE_TABLE(acpi, adv_ec_acpi_match);
#endif

static const struct platform_device_id adv_ec_id[] = {
	{ ADVANTECH_EC_NAME, },
	{}
};
MODULE_DEVICE_TABLE(platform, adv_ec_id);

static struct platform_driver adv_ec_driver = {
	.driver = {
		.name = ADVANTECH_EC_NAME,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(adv_ec_of_match),
#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(adv_ec_acpi_match),
#endif
	},
	.id_table = adv_ec_id,
	.probe = adv_ec_probe,
	.remove = adv_ec_remove,
};
module_platform_driver(adv_ec_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("sun.lang");
MODULE_DESCRIPTION("Advantech EC MFD Driver.");
