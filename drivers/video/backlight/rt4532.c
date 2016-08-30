#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>

static struct i2c_client *rt4532_client;
int bl_en_gpio;

void rt4532_I2C_read(struct i2c_client *client, u8 address)
{
	
	int ret = -1;
	struct i2c_msg msg;
	unsigned char buf[1];

	buf[0] = address;
	msg.addr = client->addr;
	msg.flags = 0; //Write
	msg.len = 1;
	msg.buf = (unsigned char *)buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if( ret == 1) { // 1 msg sent OK
        	unsigned char r_buf[1];
		// Delay 1 ms, wait for f/w device data ready
		mdelay(2);
		//read back device information
		msg.addr = client->addr;
		msg.flags = I2C_M_RD; //Read
		msg.len = 1;
		msg.buf = (unsigned char *)r_buf;
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1) 
			printk(" %s: address(0x%x)=0x%04xh austin++++++++!\n", __FUNCTION__, address, r_buf[0]);	
	} else
		printk(" %s: write failed !\n", __FUNCTION__);
	return;
}


void rt4532_I2C_readDeviceInfo(struct i2c_client *client)
{
	
	int ret = -1;
	struct i2c_msg msg;
	unsigned char buf[1];

	buf[0] = 0x00;
	msg.addr = client->addr;
	msg.flags = 0; //Write
	msg.len = 1;
	msg.buf = (unsigned char *)buf;
	printk("%s:: austin (%x) austin++\n", __func__, msg.addr);
	ret = i2c_transfer(client->adapter, &msg, 1);
	if( ret == 1) { // 1 msg sent OK
        	unsigned char r_buf[1];
		// Delay 1 ms, wait for f/w device data ready
        	printk(" %s: write OKKK austin++++++++!\n", __FUNCTION__);
		mdelay(2);
		//read back device information
		msg.addr = client->addr;
		msg.flags = I2C_M_RD; //Read
		msg.len = 1;
		msg.buf = (unsigned char *)r_buf;
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1) 
			printk(" %s: read OKKK REVID=0x%02xh austin++++++++!\n", __FUNCTION__,r_buf[0]);	
	}
	return;
}

void rt4532_I2C_PWMmode_low_active(struct i2c_client *client)
{
	int ret = -1;
	struct i2c_msg msg;
	unsigned char buf[2];

	//sent address 0x10 to Pmic >>>>>
	buf[0] = 0x02;
	buf[1] = 0xE7;
	msg.addr = client->addr;
	msg.flags = 0; //Write
	msg.len = 2;
	msg.buf = (unsigned char *)buf;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1)		
		printk("%s:: wrtite 0x02 to 0xE7 failed austin\n", __func__);

}

void rt4532_suspend(void)
{
	gpio_direction_output(bl_en_gpio,0);

	return;
}
EXPORT_SYMBOL(rt4532_suspend);

void rt4532_resume(void)
{
	gpio_direction_output(bl_en_gpio,1);

	/*rt4532_I2C_read(rt4532_client, 0x02);*/
	return;
}
EXPORT_SYMBOL(rt4532_resume);

static int rt4532_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int addr;
	
	rt4532_client = client;
	addr = client->addr; //Slave Addr

	bl_en_gpio = of_get_named_gpio_flags(client->dev.of_node, "rt,bl-en-gpio",0, NULL);
	if (gpio_request(bl_en_gpio, "bl_en_gpio") != 0)
		printk("%s::: austin request bl_en_gpio failed \n", __func__);
	else	
	   gpio_direction_output(bl_en_gpio,1);

	printk(" %s::Slaveaddr[%x] BL_EN(%d)\n", __func__, addr, bl_en_gpio);

	rt4532_I2C_read(client, 0x02);

	return 0;
}

static struct of_device_id rt4532_i2c_table[] = {
	{ .compatible = "rt,4532"}, //Compatible node must match dts
	{ },
};

static const struct i2c_device_id rt4532_id[] = {
	{ "rt4532", 0 },
	{ },
};

//I2C Driver Info
static struct i2c_driver rt4532_driver = {
	.driver = {
		.name = "rt4532",
		.owner = THIS_MODULE,
		.of_match_table = rt4532_i2c_table,
	},
	.probe = rt4532_probe,
	.id_table = rt4532_id,
};


static int __init rt4532_I2C_init(void)
{
	int ret = 0;
	printk("%s:: austin+++\n",__func__);
	ret = i2c_add_driver(&rt4532_driver);
	
	return ret;
}

static void __exit rt4532_I2C_exit(void)
{
	return;
}

module_init(rt4532_I2C_init);
module_exit(rt4532_I2C_exit);


//Easy wrapper to do driver init
//module_i2c_driver(rt4532_driver);
MODULE_DESCRIPTION("rt4532");
MODULE_LICENSE("GPL v2");
