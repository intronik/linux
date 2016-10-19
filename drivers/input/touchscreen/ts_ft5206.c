/*
 * FocalTech FT5206 Touchscreen Controller Driver
 * Copyright (C) 2015 Intronik GmbH
 *
 * Base of the kernel driver:
 *  - st1232.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "ts_ft5206.h"

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_data/nh_ft5x06_pdata.h>

#define FT5206_TS_NAME "ft5206-ts"
#define    FTS_PACKET_LENGTH        128
#define MAX_FINGERS	1

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char       FTS_BYTE;   //8 bit
typedef unsigned short      FTS_WORD;   //16 bit
typedef unsigned int        FTS_DWRD;   //16 bit
typedef unsigned char       FTS_BOOL;   //8 bit
typedef unsigned char       U8;

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE               0x0

#define I2C_CTPM_ADDRESS       0x70

static FTS_BYTE firmware[] = {
    #include "ts_ft5206.inc"
};

#define CONFIG_FT5X0X_MULTITOUCH 1

struct ft5206_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct dev_pm_qos_request low_latency_req;
	int touch_gpio;
};

void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}

static int ft5x0x_i2c_txdata(struct i2c_client *client, char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(client, buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }

    return 0;
}

FTS_BOOL i2c_read_interface(struct i2c_client *client, FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;

    ret=i2c_master_recv(client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[FTS]i2c_read_interface error\n");
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

FTS_BOOL i2c_write_interface(struct i2c_client *client, FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[FTS]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

FTS_BOOL cmd_write(struct i2c_client *client, FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(client, I2C_CTPM_ADDRESS, write_cmd, num);
}

FTS_BOOL byte_write(struct i2c_client *client, FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{

    return i2c_write_interface(client, I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

FTS_BOOL byte_read(struct i2c_client *client, FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(client, I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}

E_UPGRADE_ERR_TYPE ft5206_ts_load_fw(struct i2c_client *client, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth) {
   FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD	packet_number;
    FTS_DWRD	j;
    FTS_DWRD	temp;
    FTS_DWRD	lenght;
    FTS_BYTE	packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE	auc_i2c_write_buf[10];
    FTS_BYTE	bt_ecc;
    int			i_ret;

    //Step 1:Reset  CTPM
    /*write 0xaa to register 0xfc*/
    ft5x0x_write_reg(client,0xfc,0xaa);
    delay_qt_ms(50);
     /*write 0x55 to register 0xfc*/
    ft5x0x_write_reg(client,0xfc,0x55);
    printk("[FTS] Step 1: Reset CTPM test\n");

    delay_qt_ms(30);


    //Step 2:Enter upgrade mode
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(client, auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );

    //Step 3:check READ-ID
    cmd_write(client, 0x90,0x00,0x00,0x00,4);
    byte_read(client, reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
        //i_is_new_protocol = 1;
    }

    cmd_write(client, 0xcd,0x0,0x00,0x00,1);
    byte_read(client, reg_val,1);
    printk("[FTS] bootloader version = 0x%x\n", reg_val[0]);

    //Step 4:erase app and panel paramenter area
    cmd_write(client, 0x61,0x00,0x00,0x00,1);  //erase app area
    delay_qt_ms(1500);
    cmd_write(client, 0x63,0x00,0x00,0x00,1);  //erase panel parameter area
    delay_qt_ms(100);
    printk("[FTS] Step 4: erase. \n");

    //Step 5:write firmware(FW) to ctpm flash
    bt_ecc = 0;
    printk("[FTS] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(client, &packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[FTS] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(client, &packet_buf[0],temp+6);
        delay_qt_ms(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i];
        bt_ecc ^= packet_buf[6];

        byte_write(client, &packet_buf[0],7);
        delay_qt_ms(20);
    }

	//Step 6: read out checksum
    //send the opration head
    cmd_write(client, 0xcc,0x00,0x00,0x00,1);
    byte_read(client, reg_val,1);
    printk("[FTS] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    return ERR_OK;
}

static int ft5206_ts_identify(struct i2c_client *client) {
    //get the flash option
    const char *fwflash;
    if (of_property_read_string(client->dev.of_node, "flash-firmware", &fwflash) == 0) {  //property found
        if (strcmp("yes",fwflash)==0){
            ft5206_ts_load_fw(client, firmware, sizeof(firmware));
        }
        //else printk("[FTS]: no firmware flash\n");
    }

    //Step 7: reset the FW
    cmd_write(client, 0x07,0x00,0x00,0x00,1);

    msleep(300);  //make sure CTP startup normally
	return 0;
}

static irqreturn_t ft5206_ts_irq_handler(int irq, void *dev_id) {
	struct ft5206_ts_data *ts = dev_id;
	struct input_dev *input_dev = ts->input_dev;
	struct i2c_client *client = ts->client;
	struct i2c_msg msg[2];
	u8 start_reg;
	u8 buf[6*MAX_FINGERS+1];
	u8* prawfinger;
	int error;
	int count = 0;
	unsigned i;
	unsigned c;

	/* read touchscreen data from FT5206 */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x02;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	error = i2c_transfer(client->adapter, msg, 2);
	if (error < 0)
		goto end;


	c = buf[0];			// get number of fingers
	if (c>MAX_FINGERS)		// limit that value
		c=MAX_FINGERS;
	prawfinger = &buf[1];		// set start pointer of buffer to first finger struct

	/* multi touch protocol A
		see /Documentation/input/multi-touch-protocol.txt
	*/
	for (i=0; i<buf[0]; i++)
	{
		u16 x = ((prawfinger[0]&0xF)<<8)|(prawfinger[1]);
		u16 y = ((prawfinger[2]&0xF)<<8)|(prawfinger[3]);
		u8 ev = prawfinger[0]>>6;	// 2 bit event flag
		//u8 id = prawfinger[2]>>4;	// 4bit touch id
		prawfinger+=6;			// advance to next finger in raw data
		// only report contact events, since touch down events are sometimes buggy
		if (ev==0x02)
		{
			input_report_abs(input_dev, ABS_X, x);
			input_report_abs(input_dev, ABS_Y, y);
			count++;
		}
	}
	input_report_key(input_dev, BTN_TOUCH, count>0);

	/* SYN_REPORT */
	input_sync(input_dev);
end:
	return IRQ_HANDLED;
}

static int ft5206_ts_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	struct ft5206_ts_data *ts;
	struct nh_ft5x06_pdata *pdata = dev_get_platdata(&client->dev);
	struct input_dev *input_dev;
	struct pinctrl *pinctrl;
	int error=0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -EIO;
	}

    ft5206_ts_identify(client);

	pinctrl = devm_pinctrl_get_select_default(&client->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&client->dev, "pins are not configured\n");

	/*if (ft5206_ts_identify(client))
		return -ENODEV;*/

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev)
		return -ENOMEM;

	dev_info(&client->dev, "loading ft5206 ctp");

	ts->client = client;
	ts->input_dev = input_dev;

	if (pdata)
		ts->touch_gpio = pdata->touch_gpio;
	else if (client->dev.of_node)
		ts->touch_gpio = of_get_named_gpio(client->dev.of_node, "touch-gpio", 0);
	else
		ts->touch_gpio = -ENODEV;

	if (gpio_is_valid(ts->touch_gpio)) {
		if (client->irq>0) {
			dev_err(&client->dev,
				"Configuration conflict, do not specify both interrupt %d and touch gpio %d.\n",
				client->irq, ts->touch_gpio);
				return error;
		}
		error = devm_gpio_request_one(&client->dev, ts->touch_gpio, GPIOF_IN|GPIOF_EXPORT, "ft5206-touch");
		if (error) {
			dev_err(&client->dev,
				"Unable to request GPIO pin %d.\n",
				ts->touch_gpio);
				return error;
		}
		client->irq = gpio_to_irq(ts->touch_gpio);
		if (client->irq < 0) {
			error = client->irq;
			dev_err(&client->dev,
				"Unable to get irq number for GPIO %d, error %d\n",
				ts->touch_gpio, error);
			return error;
		}
	}

	if (!client->irq) {
		dev_err(&client->dev, "no IRQ?\n");
		return -EINVAL;
	}

	input_dev->name = "FocalTech-ft5206-touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

//	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, MAX_FINGERS, 0, 0);
	input_set_abs_params(input_dev, ABS_X, MIN_X, MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, MIN_Y, MAX_Y, 0, 0);
//	input_set_abs_params(input_dev, ABS_MT_POSITION_X, MIN_X, MAX_X, 0, 0);
//	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, MIN_Y, MAX_Y, 0, 0);

	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, ft5206_ts_irq_handler,
					  IRQF_ONESHOT|IRQF_TRIGGER_FALLING,
					  client->name, ts);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		return error;
	}

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&client->dev, "Unable to register %s input device\n",
			input_dev->name);
		return error;
	}

	i2c_set_clientdata(client, ts);
	device_init_wakeup(&client->dev, 1);

	return 0;
}

static int ft5206_ts_remove(struct i2c_client *client) {
	device_init_wakeup(&client->dev, 0);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ft5206_ts_suspend(struct device *dev) {
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev)) {
		enable_irq_wake(client->irq);
	} else {
		disable_irq(client->irq);
	}
	return 0;
}

static int ft5206_ts_resume(struct device *dev) {
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev)) {
		disable_irq_wake(client->irq);
	} else {
		enable_irq(client->irq);
	}
	return 0;
}

#endif

static SIMPLE_DEV_PM_OPS(ft5206_ts_pm_ops,
			 ft5206_ts_suspend, ft5206_ts_resume);

static const struct i2c_device_id ft5206_ts_id[] = {
	{ FT5206_TS_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft5206_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id ft5206_ts_dt_ids[] = {
	{ .compatible = "focaltech,ft5206", },
	{ }
};
MODULE_DEVICE_TABLE(of, ft5206_ts_dt_ids);
#endif

static struct i2c_driver ft5206_ts_driver = {
	.probe		= ft5206_ts_probe,
	.remove		= ft5206_ts_remove,
	.id_table	= ft5206_ts_id,
	.driver = {
		.name	= FT5206_TS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(ft5206_ts_dt_ids),
		.pm	= &ft5206_ts_pm_ops,
	},
};

module_i2c_driver(ft5206_ts_driver);

MODULE_AUTHOR("RN <info@intronik.de>");
MODULE_DESCRIPTION("FOCALTECH FT5206 Touchscreen Controller Driver");
MODULE_LICENSE("GPL");
