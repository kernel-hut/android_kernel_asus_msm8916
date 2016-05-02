#include <linux/module.h>
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
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ProximityBasic.h>
#include "ProximitySensorCore.h"
#include "ProximitySource.h"
#include "source/ProximitySourceFactory.h"
#include "algo/ProximityAlgoFactory.h"

#define __devinit        __section(.devinit.text) __cold notrace
#define __devexit        __section(.devexit.text) __exitused __cold notrace
#if defined(MODULE) || defined(CONFIG_HOTPLUG)
#define __devexit_p(x) x
#else
#define __devexit_p(x) NULL
#endif

typedef struct ProximitySensorCore{
    bool inited;
    struct mutex source_lock;
    struct list_head source;
    ProximityEventWatcher watch;
}ProximitySensorCore;
static int proximity_sensor_suspend(struct device *dev)
{
	//struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);

	return 0;
}

static int proximity_sensor_resume(struct device *dev)
{
	//struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);

	return 0;
}

static void init_core_data(ProximitySensorCore *core)
{
    mutex_init(&core->source_lock);
    INIT_LIST_HEAD(&core->source);
}
static void onEventChanged(struct ProximitySource *apSource, PROXIMITY_EVENT event)
{
    printk("%s\n", __FUNCTION__);

    if(NULL != apSource->getAlgo(apSource)){

        apSource->getAlgo(apSource)->handle(
            apSource->getAlgo(apSource),
            event,
            apSource->getIndex(apSource)
        );
    }
}

static ProximitySensorCore gSensorCore= {
    .inited = false,
    .watch = {
        .onEventChanged = onEventChanged,
    }
};

static int __devinit proximity_sensor_probe(struct platform_device *pdev)
{
    int i;

    int size;

    proximity_platform_data *pdata = pdev->dev.platform_data; 

    printk("%s+++\n", __FUNCTION__);

    init_core_data(&gSensorCore);

    if(pdata){

        if(pdata->nResource > MAX_POXIMITY_SENSOR_SIZE){

            printk(KERN_ERR "%sExceed the sensor size!\n", __FUNCTION__);

        }

        size = (pdata->nResource > MAX_POXIMITY_SENSOR_SIZE)?MAX_POXIMITY_SENSOR_SIZE:pdata->nResource;

        for (i = 0; i < size; i++) {

            ProximitySource *source = NULL;

            proximity_resource *resource = &pdata->resource[i];  

            printk("Proximity source :%s created...\n",resource->name);

            source = getProximitySource(
                resource,
                &gSensorCore.watch,
                getProximityAlgo(resource->algo_type));

            if(NULL != source){
                
                list_add(&source->list,&gSensorCore.source);

            }
        }

    }

    printk("%s---\n", __FUNCTION__);

    return 0;
}
static int __devexit proximity_sensor_remove(struct platform_device *pdev)
{

	return 0;
}

static const struct dev_pm_ops proximity_sensor_pm_ops = {
	.suspend	        = proximity_sensor_suspend,
	.resume		= proximity_sensor_resume,
};

static struct platform_driver proximity_sensor_device_driver = {
	.probe		= proximity_sensor_probe,
	.remove		= __devexit_p(proximity_sensor_remove),
	.driver		= {
		.name	= "proximity-core-sensor",
		.owner	= THIS_MODULE,
		.pm	= &proximity_sensor_pm_ops,
		//.of_match_table = proximity_sensor_of_match,
	}
};
static int __init proximity_sensor_init(void)
{
    printk("%s+++\n", __FUNCTION__);

    return platform_driver_register(&proximity_sensor_device_driver);
}

static void __exit proximity_sensor_exit(void)
{
    printk("%s+++\n", __FUNCTION__);

    platform_driver_unregister(&proximity_sensor_device_driver);
}
module_init(proximity_sensor_init);
module_exit(proximity_sensor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Victor FU <victor_fu@asus.com>");
MODULE_DESCRIPTION("Proximity sensor");
//MODULE_ALIAS("platform:proximity_sensor");

