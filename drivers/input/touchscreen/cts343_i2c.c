/*
* Advanced Silicon CTS343 TouchScreen I2C driver
*
* Copyright (c) 2021  Advanced Silicon S.A.
* Info <info@advancedsilicon.com>
*
* This software is licensed under the terms of the GNU General Public
* License, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*/

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/input/mt.h>
#include <asm/unaligned.h>
#include <linux/timer.h>
#include <linux/gpio/consumer.h>
#define CMD_ID_WD_RESET			(0xA9) // TOOD: remove
#define CMD_WD_RESET_KEY		(5610) // TODO: remove
#define TYPE_CMD				(0x01)
#define TYPE_PARAMETERS			(0x02)

#define WDOG_TIME				2000
#define WDOG_ERR_CNT			5
#define CTS_XOR(i1,i2)	((i1)^(i2))

#define	I2C_RP_ST					0x01
#define I2C_REQUEST_GET_REPORT		0x02
#define I2C_REQUEST_SET_REPORT		0x03
#define I2C_REQUEST_SET_POWER		0x08
#define REPORTID_DEVICE_INFO 		0xF2
#define REPORTID_I2C_INPUT_PREVIOUS 	0x26
#define REPORTID_I2C_INPUT		0x24

#define CTS343_NAME				"cts343_i2c"
#define CTS343_DRV_VER			"0.0.23"
#define I2C_HID_DSCR_ID			0x20
#define REPORTLEN_I2C_HID_DSCR	30
#define	ST_REPORT				0x01
#define	RPT_ID_TOUCH			0x01

#define REPORTLEN_DEVICEINFO	44

#define CTS_MAX_FINGER			10
#define CTS_RAW_BUF_COUNT		60
#define CTS_PCK_LEN_COUNT		2

#define MAX_UNIT_AXIS			0x7FFF

/* the finger definition of the report event */
#define FINGER_EV_OFFSET_ID		0
#define FINGER_EV_OFFSET_X		1
#define FINGER_EV_OFFSET_Y		3
#define FINGER_EV_SIZE			5

/* The definition of a report packet */
#define TOUCH_PK_OFFSET_REPORT_ID	2
#define TOUCH_PK_OFFSET_EVENT		3
#define TOUCH_PK_OFFSET_SCAN_TIME	53
#define TOUCH_PK_OFFSET_FNGR_NUM	55
#define TOUCH_PK_RPT_CNT		56
#define TOUCH_PK_CHECKSUM		58
/* The definition of the firmware id string */
#define FW_ID_OFFSET_FW_ID			1
#define FW_ID_OFFSET_N_TCH_PKT		13
#define	FW_ID_OFFSET_N_BT_TCH		14

/* Controller requires minimum 300us in get_feature */
#define CTS_COMMAND_DELAY_MS		1
static union devinfo DevInfo;
static int wchdog_err = 0;
static uint16_t last_report_count = 0;
static int initialized = 0;
static int reportid_i2c_command = 0x22; // updated from descriptor

static u8 saved_packet[CTS_RAW_BUF_COUNT];
static int last_packet_missed = -1;
static int packet_missed_number = 0;
static int saved_packet_index = -1;
static int current_report_count = 0;

struct device_info {
	__le16 len;
	u8 ReportID;
	__le32 FirmwareID;
	__le32 HardwareID;
	__le32 SerialNO;
	__le32 HASH[5];
	u8 Modif;
	__le32 XMLS_ID;
	__le16 LCD_W;
	__le16 LCD_H;
} __packed;

union devinfo{
	struct device_info devinfo;
	u8 data[REPORTLEN_DEVICEINFO];
};

struct cts343_param {
	u16 HIDDescLength;
	u16 bcdVersion;
	u16 ReportDescLength;
	u16 ReportDescRegister;
	u16 InputRegister;
	u16 MaxInputLength;
	u16 OutputRegister;
	u16 MaxOutputLength;
	u16 CommandRegister;
	u16 DataRegister;
	u16 VendorID;
	u16 ProductID;
	u16 VersionID;
	u16 rsvd0;
	u16 rsvd1;
	u16 max_x;
	u16 max_y;
	u16	phy_w;
	u16	phy_h;
	u16	scaling_factor;
} __packed;

struct cts343_data;

typedef	int(*LPFUNC_report_type) (struct cts343_data *cts);

struct cts343_data {
	struct i2c_client		*client;
	struct input_dev		*input_mt;
	/* Mutex for fw watchdog to prevent concurrent access */
	struct mutex			fw_mutex;
	struct timer_list		watchdog_timer;
	struct work_struct		watchdog_work;
	struct cts343_param		param;
	struct gpio_desc		*gpio_reset;
	u8						phys[32];
	u32						state;
	bool					wake_irq_enabled;

	u32						report_type;
	u32						dev_status;
	LPFUNC_report_type		func_report_type[1];
	u32						fngr_state;
	u32						rpt_scantime;
	bool					flipx;
	bool					flipy;
	bool					report_coordinates;
};

static int cts343_hw_reset(struct cts343_data *cts);
static void dsp_sw_reset(struct cts343_data *cts);
static int get_feature(struct i2c_client *client, u8 report_id, u8 *buf, u16 len);
static void cts343_watchdog_timer(struct timer_list *t);
static void cts343_watchdog_work(struct work_struct *work);
static void cts343_start_wd(struct cts343_data *cts);
static void cts343_stop_wd(struct cts343_data *cts);
static int set_feature(struct i2c_client *client, u8 report_id, u8 *buf, u16 len){
	u8 tx_buf[64];
	int ret;
	int i;
	struct i2c_msg msg = {
			.addr = client->addr,
			.flags = 0,
			.len = len + 9,
			.buf = tx_buf,
		};
	for (i = 0; i < len; i++){
		tx_buf[i + 9] = buf[i];
	}
	tx_buf[0] = reportid_i2c_command & 0xFF; // Command Register
	tx_buf[1] = reportid_i2c_command >> 8;
	tx_buf[2] = (0x3 << 4) + 0xF; // Feature type & report_id = 15
	tx_buf[3] = I2C_REQUEST_SET_REPORT & 0xF;
	tx_buf[4] = report_id; // Report ID
	tx_buf[5] = 0 & 0xFF; // Data Register
	tx_buf[6] = 0 >> 8;
	tx_buf[7] = (len + 2) & 0xFF; // Length
	tx_buf[8] = (len + 2) >> 8;

	ret = i2c_transfer(client->adapter, &msg, 1);
	return ret;
}


static int cts343_mt_release_contacts(struct cts343_data *cts)
{
	struct input_dev *input = cts->input_mt;
	int i;
	for (i = 0; i < CTS_MAX_FINGER; i++) {
		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, 0);
	}

	input_mt_sync_frame(input);
	input_sync(input);
	return 0;
}

static int send_cmd(struct i2c_client *client, u8 report_id , u8 value){
	u8 tx_buf[4];
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(tx_buf),
			.buf = tx_buf,
		},
	};
	tx_buf[0] = reportid_i2c_command & 0xFF; // Command Register
	tx_buf[1] = reportid_i2c_command >> 8;
	tx_buf[2] = value & 0x1;
	tx_buf[3] = report_id & 0xF;
	
	ret = i2c_transfer(client->adapter, &msgs[0], 1);
	return (ret < 0) ? ret: 0;
}

static int __maybe_unused cts343_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cts343_data *cts = i2c_get_clientdata(client);
	int error;

	disable_irq(client->irq);
	dev_info(&client->dev, "CTS343: suspend\n");

	if (device_may_wakeup(dev)) {
		dev_info(&client->dev, "CTS343: wakeup enabled\n");
		error = enable_irq_wake(client->irq);
		dev_info(&client->dev, "CTS343: wakeup return %d \n", error);
		cts->wake_irq_enabled = (error == 0);
		dev_info(&client->dev, "CTS343: wakeup %d \n", cts->wake_irq_enabled);
	}
	error = send_cmd(client, I2C_REQUEST_SET_POWER, 1);
	cts343_stop_wd(cts);
	if (error) {
		enable_irq(client->irq);
		dev_err(&client->dev, "SUSPEND CMD Failed: %d\n", error);
		return error;
	}

	return 0;
}

static int __maybe_unused cts343_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cts343_data *cts = i2c_get_clientdata(client);
	int error;
	if (device_may_wakeup(dev)) {
		dev_info(&client->dev, "cts343 resume: wakeup\n");
		if (cts->wake_irq_enabled)
			disable_irq_wake(client->irq);
	}
	error = send_cmd(client, I2C_REQUEST_SET_POWER, 0);
	if (error)
		dev_err(&client->dev, "Resume Failed: %d", error);

	enable_irq(client->irq);
	cts343_mt_release_contacts(cts);
	dev_info(&client->dev, "end cts343 resume\n");
	cts343_start_wd(cts);
	return 0;
}

static SIMPLE_DEV_PM_OPS(cts343_pm_ops, cts343_suspend, cts343_resume);

static uint16_t misr(uint16_t currentChecksumValue, uint16_t newValue)
{
	uint16_t a, b;
	a = ( CTS_XOR(CTS_XOR(CTS_XOR(CTS_XOR(CTS_XOR(CTS_XOR(CTS_XOR(CTS_XOR( (newValue >> 0), (currentChecksumValue >> 0)), (currentChecksumValue >> 1)), (currentChecksumValue >> 2)), (currentChecksumValue >> 4)), (currentChecksumValue >> 5)), (currentChecksumValue >> 7)), (currentChecksumValue >> 11)), (currentChecksumValue >> 15)) ) & 1;
	b = CTS_XOR( (currentChecksumValue << 1), newValue ) & 0xFFFE;
	return a | b;
}

static uint16_t checksum_compute(uint8_t const *buffer, uint16_t count, uint16_t initialValue)
{
	uint16_t checksum;
	int i = 0;
	checksum = misr(0, initialValue);
	for (i = 0; i < count; ++i)
	{
		checksum = misr(checksum, *buffer++);
	}
	return checksum;
}

static int cts343_i2c_rx(struct i2c_client *client,	void *rxdata, size_t rxlen, void *txdata, size_t txlen)
{
	int ret;
	struct i2c_msg wr_msg = {
		.addr = client->addr,
		.flags = 0,
		.len = txlen,
		.buf = txdata,
	};
	struct i2c_msg rd_msg = {
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = rxlen,
		.buf = rxdata,
	};
	
	ret = i2c_transfer(client->adapter, &wr_msg, 1);
	if(ret)
	{
		ret = i2c_transfer(client->adapter, &rd_msg, 1);
	}
	return (ret == 1) ? rxlen : ret;
}

static int get_feature(struct i2c_client *client, u8 report_id, u8 *buf, u16 len){
	u8 tx_buf[7];
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(tx_buf),
			.buf = tx_buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	tx_buf[0] = reportid_i2c_command & 0xFF; // Command Register
	tx_buf[1] = reportid_i2c_command >> 8;
	tx_buf[2] = (0x3 << 4) + 0xF; // Feature type & report_id = 15
	tx_buf[3] = I2C_REQUEST_GET_REPORT & 0xF;
	tx_buf[4] = report_id; // Report ID
	tx_buf[5] = 0 & 0xFF; // Data Register
	tx_buf[6] = 0 >> 8;
	
	ret = i2c_transfer(client->adapter, &msgs[0], 1);
	mdelay(CTS_COMMAND_DELAY_MS);
	if(ret < 0)
		return ret;
	ret = i2c_transfer(client->adapter, &msgs[1], 1);
	return ret;
}

static int cts343_get_param(struct cts343_data *cts)
{
	u8 txbuf[] = {0};
	int error;
	char modified;
	struct i2c_client *client = cts->client;
	struct cts343_param *param = &cts->param;
	txbuf[0] = I2C_HID_DSCR_ID;
	error = cts343_i2c_rx(client, (u8*) param , REPORTLEN_I2C_HID_DSCR, txbuf, 1);

	if ((param->HIDDescLength != REPORTLEN_I2C_HID_DSCR) | (error < 0)){
		dev_err(&client->dev, "Error: i2c_hid_dscr %d \n", param->HIDDescLength);
		return error;
	}
	
	reportid_i2c_command = param->CommandRegister;

	get_feature(client, REPORTID_DEVICE_INFO, DevInfo.data , REPORTLEN_DEVICEINFO);

	modified = (DevInfo.devinfo.Modif == 1) ? '*': ' ';

	dev_info(&client->dev, "driver version: %s ", CTS343_DRV_VER);

	dev_info(&client->dev, "fw_id: V %08x \n",	DevInfo.devinfo.FirmwareID);
	dev_info(&client->dev, 
			"HASH: %08x %08x %08x %08x %08x %c",
			DevInfo.devinfo.HASH[4], DevInfo.devinfo.HASH[3], 
			DevInfo.devinfo.HASH[2], DevInfo.devinfo.HASH[1],
			DevInfo.devinfo.HASH[0], modified
	);

	param->max_x = MAX_UNIT_AXIS;
	param->max_y = MAX_UNIT_AXIS;
	param->phy_w = DevInfo.devinfo.LCD_W;
	param->phy_h = DevInfo.devinfo.LCD_H;
	cts->flipx = false;
	cts->flipy = false;
	cts->report_coordinates = false;
	dev_info(&client->dev,
		"pid: %04x, vid: %04x, w: %d, h: %d\n",
		param->VendorID, param->ProductID, param->phy_w, param->phy_h);
	
	dev_info(&client->dev, "xmls_id: %08x\n", DevInfo.devinfo.XMLS_ID);
	INIT_WORK(&cts->watchdog_work, cts343_watchdog_work);
	// Start watchdog timer
	cts343_start_wd(cts);

	return 0;
}

static ssize_t fw_hash_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%08x\n", DevInfo.devinfo.HASH[4]);
}

static ssize_t fw_version_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%x\n", DevInfo.devinfo.FirmwareID);
}

static ssize_t hardware_id_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%8x\n", DevInfo.devinfo.HardwareID);
}

static ssize_t xmls_id_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%08x\n", DevInfo.devinfo.XMLS_ID);
}

static ssize_t flipx_store(struct device *dev,	struct device_attribute *attr, const char *buf, size_t count)
{
	int flipx = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct cts343_data *cts = i2c_get_clientdata(client);
	if (sscanf(buf, "%d", &flipx) != 1 || flipx < -1 || flipx > 2)
		return -EINVAL;
	cts->flipx = (bool) flipx;
	dev_info(&client->dev, "flip x = %d\n", cts->flipx);
	return count;
}

static ssize_t flipy_store(struct device *dev,	struct device_attribute *attr, const char *buf, size_t count)
{
	int flipy = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct cts343_data *cts = i2c_get_clientdata(client);
	if (sscanf(buf, "%d", &flipy)!=1 || flipy < -1 || flipy > 2)
		return -EINVAL;
	cts->flipy = (bool) flipy;
	dev_info(&client->dev, "flip y = %d\n", cts->flipy);
	return count;
}

static ssize_t report_coordinate_store(struct device *dev,	struct device_attribute *attr, const char *buf, size_t count)
{
	int report = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct cts343_data *cts = i2c_get_clientdata(client);
	if (sscanf(buf, "%d", &report)!=1 && ((report == 0) || (report == 1)))
		return -EINVAL;
	cts->report_coordinates = (bool) report;
	dev_info(&client->dev, "report coordinate = %d\n", cts->report_coordinates);
	return count;
}

static ssize_t cts343_hw_reset_store(struct device *dev,	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cts343_data *cts = i2c_get_clientdata(client);
	int error;
	error = cts343_hw_reset(cts);
	error = (error < 0) ? error: count;
	return count;
}

static int cts343_hw_reset(struct cts343_data *cts)
{
	struct i2c_client *client = cts->client;
	if(!IS_ERR_OR_NULL(cts->gpio_reset)){
		cts343_stop_wd(cts);
		disable_irq(cts->client->irq);
		dsp_sw_reset(cts); // TODO: remove
		dev_info(&client->dev, "hw reset \n");
		gpiod_set_value_cansleep(cts->gpio_reset, 1); // perfrom a hw reset on pin "reset" high 1ms then low
		mdelay(1);
		gpiod_set_value_cansleep(cts->gpio_reset, 0);
		msleep(3000);   // wait for digitizer to come back
		cts343_start_wd(cts);
		enable_irq(cts->client->irq);
		current_report_count = 0;
		return 0;
	}
	return -1;
}
static ssize_t suspend_store(struct device *dev,	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	dev_info(&client->dev, "suepend cmd\n");
	cts343_suspend(dev);
	return count;
}

static ssize_t resume_store(struct device *dev,	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	dev_info(&client->dev, "resume cmd\n");
	cts343_resume(dev);
	return count;
}
static void dsp_sw_reset(struct cts343_data *cts)
{
	struct i2c_client *client = cts->client;
	unsigned char cmdbuf[] = {
		0xc2,
		TYPE_CMD,	//p type
		CMD_ID_WD_RESET,	// sub type : cmd 
		CMD_WD_RESET_KEY & 0xFF,
		CMD_WD_RESET_KEY >> 8,
	};
	set_feature(client, 0xc2, cmdbuf, 5);
}

static DEVICE_ATTR(fw_hash, S_IRUGO, fw_hash_show, NULL);
static DEVICE_ATTR(xmls_id, S_IRUGO, xmls_id_show, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, fw_version_show, NULL);
static DEVICE_ATTR(hw_id, S_IRUGO, hardware_id_show, NULL);
static DEVICE_ATTR(gpio_reset, S_IWUSR, NULL, cts343_hw_reset_store);
static DEVICE_ATTR(report_coordinate, S_IWUSR, NULL, report_coordinate_store);
static DEVICE_ATTR(flipx, S_IWUSR, NULL, flipx_store);
static DEVICE_ATTR(flipy, S_IWUSR, NULL, flipy_store);
static DEVICE_ATTR(msuspend, S_IWUSR, NULL, suspend_store); // TODO: remove this 
static DEVICE_ATTR(mresume, S_IWUSR, NULL, resume_store); // TODO: remove this 

static struct attribute *cts343_attrs[] = {
	&dev_attr_fw_hash.attr,
	&dev_attr_xmls_id.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_hw_id.attr,
	&dev_attr_msuspend.attr,
	&dev_attr_mresume.attr,
	&dev_attr_gpio_reset.attr,
	&dev_attr_flipx.attr,
	&dev_attr_flipy.attr,
	&dev_attr_report_coordinate.attr,
	NULL
};

static const struct attribute_group cts343_attr_group = {
	.attrs = cts343_attrs,
};

static void cts343_report_contact(struct cts343_data *cts, struct cts343_param *param, u8 *buf)
{
	struct input_dev *input = cts->input_mt;
	int fngr_id;
	u32 x, y;

	fngr_id = (buf[FINGER_EV_OFFSET_ID] >> 1) - 1;
	if (fngr_id < 0)
		return;
	if (!(buf[FINGER_EV_OFFSET_ID] & 0x1)) {
		cts->fngr_state &= ~(0x1 << fngr_id);
		return;
	}

	x = get_unaligned_le16(buf + FINGER_EV_OFFSET_X);
	y = get_unaligned_le16(buf + FINGER_EV_OFFSET_Y);

	if(cts->flipx){
		x = param->max_x - x;
	}
	if(cts->flipy){
		y = param->max_y - y;
	}
	/* Refuse incorrect coordinates */
	if (x > param->max_x || y > param->max_y)
	{
		return;
	}
	if(cts->report_coordinates){
		dev_info(&cts->client->dev, "t(%d), x(%d), y(%d),", fngr_id, x, y);
	}
	input_mt_slot(input, fngr_id);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, 1);
	input_report_abs(input, ABS_MT_POSITION_X, x);
	input_report_abs(input, ABS_MT_POSITION_Y, y);
	cts->fngr_state |= (0x1 << fngr_id);
}

static int cts343_get_previous_touch_packet(struct i2c_client *client, void *buf){
	int error;
	int report_count;
	int expected_checksum, checksum;

	error = get_feature(client, REPORTID_I2C_INPUT_PREVIOUS, buf, CTS_RAW_BUF_COUNT);
	if(error < 0)
	{
		return -1;
	}
	expected_checksum = get_unaligned_le16(buf + TOUCH_PK_CHECKSUM); 
	checksum = checksum_compute(buf + CTS_PCK_LEN_COUNT, TOUCH_PK_CHECKSUM - CTS_PCK_LEN_COUNT, 0);
	dev_dbg(&client->dev, "pre report checksum %x :: %x", expected_checksum, checksum);
	if(expected_checksum == checksum)
	{
		report_count = *((uint16_t*)(buf + TOUCH_PK_RPT_CNT));
		return report_count;
	}
	return -1;
}

static int generate_reports(struct cts343_data *cts, u8* tbuf)
{
	int i, fngrs, report_count;
	fngrs = tbuf[TOUCH_PK_OFFSET_FNGR_NUM];
	if (!fngrs) {
		cts->fngr_state = 0;
		return 0;
	}
	if((fngrs > CTS_MAX_FINGER ) || (tbuf[TOUCH_PK_OFFSET_REPORT_ID] != RPT_ID_TOUCH))
	{
		return 0;
	}
	if(cts->report_coordinates)
	{
		report_count =*((uint16_t*)(tbuf + TOUCH_PK_RPT_CNT));
		dev_info(&cts->client->dev, "packet(%d)", report_count );
	}
	for (i = 0; i < fngrs; i++)
		cts343_report_contact(cts, &cts->param,	&tbuf[TOUCH_PK_OFFSET_EVENT + i * FINGER_EV_SIZE]);
	wchdog_err = 0; // reset wchdog_err counter		
	return 1;
}

static int cts343_report_parallel(struct cts343_data *cts)
{
	struct i2c_client *client = cts->client;
	int report_count_pre = 0;
	int error;
	int checksum = 0;
	int expected_checksum = 0;
	int send_report = false;
	int send_saved_paket = false;
	
	u8 raw_buf[CTS_RAW_BUF_COUNT];
	u8 tx_buf[1] = { REPORTID_I2C_INPUT };
	memset(raw_buf, 0, CTS_RAW_BUF_COUNT);

	if(last_packet_missed >= 0)
	{
		report_count_pre = cts343_get_previous_touch_packet(client, raw_buf);
		if(report_count_pre < 0)
		{
			dev_err(&client->dev, "error happened %d !", report_count_pre);
			return 0;
		}
		else if (!initialized)
		{
			current_report_count = report_count_pre;
			last_report_count = current_report_count;
			packet_missed_number = -1;
			send_report = true;
		}
		else if(packet_missed_number == report_count_pre){
			last_packet_missed = -1;
			if (report_count_pre > last_report_count) 
			{
				last_report_count = report_count_pre;
				send_report = true;
				send_saved_paket = true;
			}
			error = 0;
		}
		else if(packet_missed_number < report_count_pre)
		{
			last_packet_missed = -1;
			error = 0;
			last_report_count = (u8) (report_count_pre);
			send_report = true;
			send_saved_paket = true;
		}
		else if(packet_missed_number > report_count_pre)
		{
			last_packet_missed = -1;
			error =0;
		}
		dev_dbg(&client->dev, "count (%d) ", report_count_pre);
	}
	else{
		send_saved_paket = false;
		send_report = false;
		error = cts343_i2c_rx(client, raw_buf,  CTS_RAW_BUF_COUNT , tx_buf, 1);
		expected_checksum = *((uint16_t*)(raw_buf + TOUCH_PK_CHECKSUM));
		checksum = checksum_compute(raw_buf + CTS_PCK_LEN_COUNT, TOUCH_PK_CHECKSUM - CTS_PCK_LEN_COUNT, 0);
		if((checksum == expected_checksum) & (raw_buf[0] == CTS_RAW_BUF_COUNT))
		{
			current_report_count =*((uint16_t*)(raw_buf + TOUCH_PK_RPT_CNT));
			if(!initialized)
			{
				last_report_count = current_report_count;
				send_report = true; 
				initialized = 1;
			}
			else
			{
				if(((u16)(current_report_count - last_report_count)) == 1 )
				{
					last_report_count = current_report_count;
					packet_missed_number = -1;
					send_report = true;
				}
				else if(((u16)(current_report_count - last_report_count)) == 2 )
				{
					last_packet_missed = 1;
					// Store Packet and get previous then send this one
					packet_missed_number = (u16) (last_report_count + 1);
					dev_dbg(&client->dev, "packet missed, diff = %d,  cur = %d, last = %d", packet_missed_number, current_report_count, last_report_count);
					saved_packet_index = current_report_count;
					memcpy(saved_packet, raw_buf, CTS_RAW_BUF_COUNT);
					send_report = false;
				}
				else {

					last_report_count = current_report_count;
					send_report = true;
				}
			}
		}

		if ((error < 0) || (checksum != expected_checksum))
		{
			if (error < 0)
			{
				dev_err(&client->dev, "read raw data failed: %d\n", error);
			}
			if(checksum !=expected_checksum)
			{
				dev_err(&client->dev, "checksum wrong");
			}
			last_packet_missed = 1;
			packet_missed_number = current_report_count + 1;
			saved_packet_index = -1;
			send_report = false;
		}
	}

	if(!send_report)
	{
		return 0;
	}
	
	error = generate_reports(cts, (u8*) raw_buf);
	if(send_saved_paket & (saved_packet_index >=0))
	{
		saved_packet_index = -1;
		memcpy(raw_buf, saved_packet, CTS_RAW_BUF_COUNT);
		generate_reports(cts, (u8*) raw_buf);
		send_saved_paket = false;
	}
	return error;
}

static irqreturn_t cts343_ts_interrupt(int irq, void *dev_id)
{
	struct cts343_data *cts = dev_id;
	if (cts->func_report_type[0](cts)) {
		input_mt_sync_frame(cts->input_mt);
		input_sync(cts->input_mt);
	}
	
	return IRQ_HANDLED;
}

static int cts343_ts_create_input_device_mt(struct cts343_data *cts)
{
	struct device *dev = &cts->client->dev;
	struct input_dev *input;
	unsigned int res = DIV_ROUND_CLOSEST(MAX_UNIT_AXIS, cts->param.phy_w);
	int error;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	cts->input_mt = input;

	input->name = "CTS343 Touchscreen";
	input->id.bustype = BUS_I2C;
	input->id.vendor = cts->param.VendorID;
	input->id.product = cts->param.ProductID;
	input->phys = cts->phys;

	__set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, cts->param.max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, cts->param.max_y, 0, 0);
	input_abs_set_res(input, ABS_MT_POSITION_X, res);
	input_abs_set_res(input, ABS_MT_POSITION_Y, res);

	error = input_mt_init_slots(input, CTS_MAX_FINGER, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error) {
		dev_err(dev, "Unable to set up slots, err: %d\n", error);
		return error;
	}
	error = input_register_device(input);
	if (error) {
		dev_err(dev, "failed to register input mt: %d\n", error);
		return error;
	}
	return 0;
}

static void cts343_watchdog_timer(struct timer_list *t){
	struct cts343_data *cts =from_timer(cts, t, watchdog_timer);
	schedule_work(&cts->watchdog_work);
}

static void cts343_watchdog_work(struct work_struct *work){

	struct cts343_data *cts =container_of(work, struct cts343_data, watchdog_work);
	u8 dvinfo[7];
	u32 fwid;
	//error = mutex_lock_interruptible(&cts->fw_mutex);
	mutex_lock(&cts->fw_mutex);
	disable_irq(cts->client->irq);
	get_feature(cts->client, REPORTID_DEVICE_INFO, dvinfo , sizeof(dvinfo));
	fwid = get_unaligned_le32(dvinfo + 3);
	if(DevInfo.devinfo.FirmwareID == fwid)
	{
		wchdog_err = 0;
		dev_info(&cts->client->dev, "** fwid: %x", fwid);
	}
	else
	{
		wchdog_err += 1;
		if (wchdog_err >= WDOG_ERR_CNT)
		{
			// restart the digitizer by gpio
			enable_irq(cts->client->irq);
			mutex_unlock(&cts->fw_mutex);
			cts343_hw_reset(cts);
			return ;
		}
	}

	enable_irq(cts->client->irq);
	mutex_unlock(&cts->fw_mutex);
	mod_timer(&cts->watchdog_timer, jiffies + msecs_to_jiffies(WDOG_TIME));
}

static int cts343_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cts343_data *cts;
	int error;

	dev_info(&client->dev, "adapter=%d, client irq: %d\n", client->adapter->nr, client->irq);

	/* Check if the I2C function is ok in this adaptor */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENXIO;

	cts = devm_kzalloc(&client->dev, sizeof(*cts), GFP_KERNEL);
	if (!cts)
		return -ENOMEM;

	cts->client = client;
	mutex_init(&cts->fw_mutex);
	i2c_set_clientdata(client, cts);

	snprintf(cts->phys, sizeof(cts->phys), "i2c-%u-%04x/input0",
		client->adapter->nr, client->addr);

	cts->func_report_type[0] = cts343_report_parallel;

	error = cts343_get_param(cts);
	if (error < 0)
		return error;

	error = cts343_ts_create_input_device_mt(cts);
	if (error)
		return error;

	dev_info(&client->dev, "%s: configured irq=%d\n", __func__, client->irq);

	irq_set_irq_type(client->irq, IRQ_TYPE_LEVEL_LOW);

	error = devm_request_threaded_irq(&client->dev, client->irq, NULL, cts343_ts_interrupt,	IRQF_ONESHOT, client->name, cts);
	if (error) {
		dev_err(&client->dev, "request irq failed: %d\n", error);
		return error;
	}

	cts->gpio_reset = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_LOW);
	if(IS_ERR(cts->gpio_reset)){
		error = PTR_ERR(cts->gpio_reset);
		return error;
	}
	error = device_init_wakeup(&client->dev, true);
	if (error) {
		dev_err(&client->dev, "init wakeup failed: %d\n", error);
		return error;
	}

	error = sysfs_create_group(&client->dev.kobj, &cts343_attr_group);
	if (error) {
		dev_err(&client->dev, "create sysfs failed: %d\n", error);
		return error;
	}
	
	cts->state = ST_REPORT;
	return 0;
}

static void cts343_start_wd(struct cts343_data *cts)
{
	timer_setup(&cts->watchdog_timer, cts343_watchdog_timer, 0);
	mod_timer(&cts->watchdog_timer, jiffies + msecs_to_jiffies(10000));
}

static void cts343_stop_wd(struct cts343_data *cts)
{
	del_timer_sync(&cts->watchdog_timer);
	cancel_work_sync(&cts->watchdog_work);
	del_timer_sync(&cts->watchdog_timer);
}

static int cts343_ts_remove(struct i2c_client *client)
{
	struct cts343_data *cts = i2c_get_clientdata(client);
	cts343_stop_wd(cts);
	sysfs_remove_group(&client->dev.kobj, &cts343_attr_group);
	return 0;
}

static const struct i2c_device_id cts343_dev_id[] = {
	{ CTS343_NAME, 0 },
	{}
};

#ifdef	CONFIG_OF
static struct of_device_id cts343_of_ids[] = {
	{ .compatible = "as,cts343_i2c" },
	{}
};
#endif

static struct i2c_driver cts343_driver = {
	.probe = cts343_ts_probe,
	.remove = cts343_ts_remove,
	.id_table = cts343_dev_id,
	.driver = {
	.name = CTS343_NAME,
	.pm = &cts343_pm_ops,
#ifdef	CONFIG_OF
	.of_match_table = of_match_ptr(cts343_of_ids),
#endif
},

};

static int __init cts343_driver_init(void)
{
	int ret = 0;
	pr_debug("Init Module \n");
	ret =  i2c_add_driver(&cts343_driver);
	return 0;
}

static void __exit cts343_driver_exit(void)
{
	i2c_del_driver(&cts343_driver);
	pr_debug("Module removed \n");
}


module_init(cts343_driver_init);
module_exit(cts343_driver_exit);
MODULE_AUTHOR("Info <info@advancedsilicon.com>");
MODULE_DESCRIPTION("Advanced Silicon CTS343 TouchScreen I2C driver");
MODULE_VERSION(CTS343_DRV_VER);
MODULE_LICENSE("GPL");
