#include <linux/module.h>

#include <linux/workqueue.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>


#include <asm/uaccess.h>
#include <linux/of_irq.h>

//+++ ASUS_BSP Gauss_Li "fix  need 10 seconds to wakeup  when  just suspend"
#define WAKE_LOCK_TIME	(800)
struct wake_lock hall_sensor_wake_lock; 

struct det_gpio {
	/* Configuration parameters */
	unsigned int code;	/* input event code (KEY_*, SW_*) */
	int gpio;		/* -1 if this key does not support gpio */
	int active_low;
	const char *desc;
	unsigned int type;	/* input event type (EV_KEY, EV_SW, EV_ABS) */
	int wakeup;		/* configure the button as a wake-up source */
	int debounce_interval;	/* debounce ticks interval in msecs */
	bool can_disable;
	int value;		/* axis value for EV_ABS */
	unsigned int irq;	/* Irq number in case of interrupt keys */
};

struct hall_sensor_dev {
	struct input_dev *input;	
	struct delayed_work hall_work;
	int value;
	int keep_value;
	struct det_gpio datas[0];
};



struct hall_sensor_platform_data {
	struct det_gpio *gpios;
	int ngpios;
	unsigned int poll_interval;	/* polling interval in msecs -
					   for polling driver only */
	unsigned int rep:1;		/* enable input subsystem auto repeat */
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	const char *name;		/* input device name */
};

struct workqueue_struct *hall_detect_wq = NULL;

static int hall_status = -1;

static ssize_t read_hall(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	if(hall_status < 0)
		return sprintf(buf, "Hall sensor get hall status error\n");

	return sprintf(buf, "%d\n", hall_status);
}
static DEVICE_ATTR(status, S_IRUGO, read_hall, NULL);

static struct attribute *hall_sensor_attrs[] = {
	&dev_attr_status.attr,
	NULL,
};

static struct attribute_group hall_sensor_attr_group = {
       .name="hall_sensor",
	.attrs = hall_sensor_attrs,
};


static void hall_sensor_work(struct work_struct *work)
{	
	//int times;
	struct hall_sensor_dev *sensor_dev = container_of(work, struct hall_sensor_dev, hall_work.work);

	msleep(50);
	sensor_dev->keep_value = gpio_get_value(sensor_dev->datas->gpio);

	hall_status = sensor_dev->keep_value;

	/*for(times = 0 ; times < 2; times++)
	{		
		msleep(50);
		if(sensor_dev->keep_value != gpio_get_value(sensor_dev->datas->gpio))
		{
			queue_delayed_work(hall_detect_wq, &sensor_dev->hall_work, msecs_to_jiffies(50));
			return;
		}
	}


	if(sensor_dev->value == sensor_dev->keep_value)
		return;
	else
		sensor_dev->value = sensor_dev->keep_value;*/
		
	sensor_dev->value = sensor_dev->keep_value;

	printk("type = %d, code = %d,gpio value = %d\n",sensor_dev->datas->type, sensor_dev->datas->code, sensor_dev->value);
    if(sensor_dev->value == 1)
    {
        input_report_switch(sensor_dev->input, sensor_dev->datas->code, 0);
    }
    else
    {
        input_report_switch(sensor_dev->input, sensor_dev->datas->code, 1);
    }

	input_sync(sensor_dev->input);
}


static irqreturn_t hall_det_irq(int irq, void *dev_id)
{
	struct hall_sensor_dev *sensor_dev;
	int GPIO_value; 
	
	sensor_dev = (struct hall_sensor_dev *)dev_id;
	
	//+++ ASUS_BSP Gauss_Li "fix  need 10 seconds to wakeup  when  just suspend"
	if (gpio_get_value(sensor_dev->datas->gpio) > 0) GPIO_value = 1;
      else GPIO_value = 0;		
	
	printk("before enter hall_det_irq gpio = %d\n",sensor_dev->value);
	
	if(GPIO_value != hall_status){
		queue_delayed_work(hall_detect_wq, &sensor_dev->hall_work, msecs_to_jiffies(50));
		wake_lock_timeout(&hall_sensor_wake_lock, msecs_to_jiffies(WAKE_LOCK_TIME));
	}	
	//--- ASUS_BSP Gauss_Li "fix  need 10 seconds to wakeup  when  just suspend"
	
	return IRQ_HANDLED; 
}


static int hall_sensor_get_devtree_pdata(struct device *dev, struct hall_sensor_platform_data *pdata)
{
	
	struct device_node *node, *pp = NULL;
	struct det_gpio *hall_det_gpios;
	int i;
	unsigned int reg;
	node = dev->of_node;
	
	hall_det_gpios = pdata->gpios;
	pdata->name = of_get_property(node, "input-name", NULL);


	pp = NULL;
	i = 0;

	while ((pp = of_get_next_child(node, pp))) {
		enum of_gpio_flags flags;

		if (!of_find_property(pp, "gpios", NULL)) {
			pdata->ngpios--;
			printk("Found detgpio without gpios\n");
			continue;
		}
		hall_det_gpios[i].gpio = of_get_gpio_flags(pp, 0, &flags);
		printk("[hall sensor]   gpio=%d\n",hall_det_gpios[i].gpio);
		hall_det_gpios[i].active_low = flags & OF_GPIO_ACTIVE_LOW;

		if (of_property_read_u32(pp, "linux,code", &reg)) {
			printk("detgpio without keycode: 0x%x\n", hall_det_gpios[i].gpio);
			goto out_fail;
		}
		hall_det_gpios[i].code = reg;

		hall_det_gpios[i].desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,input-type", &reg) == 0)
			hall_det_gpios[i].type = reg;
		else
			goto out_fail;

		if (of_property_read_u32(pp, "debounce-interval", &reg) == 0)
			hall_det_gpios[i].debounce_interval = reg;
		else
			hall_det_gpios[i].debounce_interval = 50;

		i++;
	}
	
	return 0;

	out_fail:
	return -ENODEV;
}



static int  hall_sensor_probe(struct platform_device *pdev)
{
	struct hall_sensor_platform_data *pdata = pdev->dev.platform_data, alt_pdata;
	struct device_node *node, *pp = NULL;

	struct input_dev *hall_input;
	struct device *dev = &pdev->dev;
	struct det_gpio *hall_det_gpios;
	struct hall_sensor_dev *sensor_dev;
		
	int error;
	node = dev->of_node;
	if (node == NULL)
		return -ENODEV;
	
	
	alt_pdata.ngpios = 0;
	while ((pp = of_get_next_child(node, pp)))
		alt_pdata.ngpios++;
	

	if(alt_pdata.ngpios != 1)
		return -ENODEV;
	



	sensor_dev = kzalloc(sizeof(struct hall_sensor_dev) + alt_pdata.ngpios * sizeof(struct det_gpio), GFP_KERNEL);
	if(!sensor_dev)
		return -ENOMEM;
	platform_set_drvdata(pdev, sensor_dev);
	
	wake_lock_init(&hall_sensor_wake_lock, WAKE_LOCK_SUSPEND, "HallSensor_wake_lock");            //+++ ASUS_BSP Gauss_Li "fix  need 10 seconds to wakeup  when  just suspend"
	alt_pdata.gpios = sensor_dev->datas;

	if (!pdata) {
		error = hall_sensor_get_devtree_pdata(dev, &alt_pdata);
		if (error)
			return error;
		pdata = &alt_pdata;
	}
	hall_det_gpios = pdata->gpios;
	error= gpio_request(hall_det_gpios->gpio, hall_det_gpios->desc);	
	if (error < 0){
		printk("hall sensor: gpio_request fail gpio=%d!\n", hall_det_gpios->gpio);
		goto fail0;
	}

	error = gpio_direction_input(hall_det_gpios->gpio); 
	if (error < 0){
		printk("hall sensor: gpio_direction_input fail gpio=%d!\n", hall_det_gpios->gpio);
		goto fail1;
	}



	hall_input = input_allocate_device();

	if (!hall_input ) {
		printk("failed to allocate state\n");
		goto fail1;
	}

	hall_input->name =  pdata->name;
	hall_input->id.bustype = BUS_HOST;
	hall_input->dev.parent = &pdev->dev;
	

	input_set_capability(hall_input, hall_det_gpios->type, hall_det_gpios->code);
	
	error = input_register_device(hall_input);
	if (error) {
		printk("Unable to register input device, error: %d\n",error);
		goto fail2;
	}
	
	sensor_dev->input = hall_input;
	sensor_dev->value = 1;
	hall_status = 1;
	
    hall_detect_wq = create_singlethread_workqueue("hall_detect_wq");
	INIT_DELAYED_WORK(&sensor_dev->hall_work, hall_sensor_work);
    queue_delayed_work(hall_detect_wq, &sensor_dev->hall_work, 0); //fix node status error when reboot

	error = request_irq(gpio_to_irq(hall_det_gpios->gpio), hall_det_irq,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED, hall_det_gpios->desc, sensor_dev);


	if (error < 0){
		printk("hall sensor: request_irq fail gpio=%d!\n", hall_det_gpios->gpio);
		goto fail3;
	}

	error = sysfs_create_group(&pdev->dev.kobj, &hall_sensor_attr_group);
	if (error) 
		printk("create dev attr error!\n");

	device_rename(&pdev->dev, "hall_sensor");

	return 0;
	
	fail3:
		input_unregister_device(hall_input);
	fail2:		
		input_free_device(hall_input);
	fail1:
		gpio_free(hall_det_gpios->gpio);
	fail0:
		kfree(sensor_dev);
		
	return error;	
		
}


static int  hall_sensor_remove(struct platform_device *pdev)
{
	struct hall_sensor_dev *sensor_dev = platform_get_drvdata(pdev);
	struct input_dev *input = sensor_dev->input;
	struct det_gpio *hall_det_gpios = sensor_dev->datas;

	free_irq(gpio_to_irq(hall_det_gpios->gpio), sensor_dev);
	gpio_free(hall_det_gpios->gpio);
	input_unregister_device(input);
	input_free_device(input);
	kfree(sensor_dev);
	sysfs_remove_group(&pdev->dev.kobj, &hall_sensor_attr_group);
	return 0;
	
}

static struct of_device_id hall_sensor_of_match[] = {
	{ .compatible = "hall-sensor", },
	{ },
};

static int hall_sensor_suspend(struct device *dev)
{
	struct hall_sensor_dev *sensor_dev = dev_get_drvdata(dev);
	//printk("hall_sensor_suspend\n");
	enable_irq_wake(gpio_to_irq(sensor_dev->datas->gpio));
	return 0;
}

static int hall_sensor_resume(struct device *dev)
{
	struct hall_sensor_dev *sensor_dev = dev_get_drvdata(dev);
	//printk("hall_sensor_resume\n");
	disable_irq_wake(gpio_to_irq(sensor_dev->datas->gpio));
	return 0;
}

static SIMPLE_DEV_PM_OPS(hall_sensor_pm_ops, hall_sensor_suspend, hall_sensor_resume);


static struct platform_driver hall_sensor_driver = {
	.probe		= hall_sensor_probe,
	.remove		= hall_sensor_remove,
	.driver		= {
		.name	= "hall-sensor",
		.owner	= THIS_MODULE,
		.pm	= &hall_sensor_pm_ops,
		.of_match_table = hall_sensor_of_match,
	}
};


static int __init hall_init(void)
{
	printk("hall_init\n");
	return platform_driver_register(&hall_sensor_driver);
}


static void __exit hall_exit(void)
{
	wake_lock_destroy(&hall_sensor_wake_lock);                                     //+++ ASUS_BSP Gauss_Li "fix  need 10 seconds to wakeup  when  just suspend"
	return platform_driver_unregister(&hall_sensor_driver);
}


module_init(hall_init);  
module_exit(hall_exit); 

MODULE_AUTHOR("ASUS");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hall sensor driver");

