/*
 * Gas Gauge driver for TI's BATTERY VPM
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/adv_vpm_comm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/reboot.h>

struct battery_vpm_info {
	struct battery_vpm_pinfo *pdata;
	struct device		*dev;
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	struct delayed_work work;
	bool is_present;
	int irq;
	struct hrtimer timer;
	ktime_t debounce_time;
	struct power_supply *bat;
	struct power_supply_desc bat_desc;
	struct power_supply	detect_usb;
};

typedef struct {
	u32 voltage;
	u32 percent;
} battery_capacity , *pbattery_capacity;

enum {
	VPM_CAPACITY,
	VPM_TIME_TO_EMPTY,
	VPM_TIME_TO_FULL,
	VPM_FLAGS,
	VPM_TEMPERATURE,
	VPM_VOLTAGE,
	VPM_CURRENT,
};

#define POLL_INTERVAL			30
#define CHARGE_FLAG_CHANGE_INTERVAL	3

enum {
	VPM_READ_FLAG = 0,
	VPM_WRITE_FLAG,
};



static struct platform_device *vpm_battery = NULL;

#define BATTERY_VPM_DATA(_psp, _addr, _min_value, _max_value) { \
	.psp = _psp, \
	.addr = _addr, \
	.min_value = _min_value, \
	.max_value = _max_value, \
}

static const struct battery_vpm_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} battery_vpm_data[] = {
	[VPM_CAPACITY] =				BATTERY_VPM_DATA(POWER_SUPPLY_PROP_CAPACITY, 0x90, 0, 100), //0x1:Primary, 0x2: Secondary
	[VPM_TIME_TO_EMPTY] =			BATTERY_VPM_DATA(POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG, 0x91, 0, 65535), //min.
	[VPM_TIME_TO_FULL] =			BATTERY_VPM_DATA(POWER_SUPPLY_PROP_TIME_TO_FULL_AVG, 0x96, 0, 65535), //min.
	[VPM_FLAGS] =					BATTERY_VPM_DATA(POWER_SUPPLY_PROP_STATUS, 0x9F, 0, 65535), 
	[VPM_TEMPERATURE] =				BATTERY_VPM_DATA(POWER_SUPPLY_PROP_TEMP, 0x93, 0, 65535), //0.1K C=K-237.15
	[VPM_VOLTAGE] =					BATTERY_VPM_DATA(POWER_SUPPLY_PROP_VOLTAGE_NOW, 0x94, 0, 65535), //mV
	[VPM_CURRENT] = 			    BATTERY_VPM_DATA(POWER_SUPPLY_PROP_CURRENT_NOW, 0x95, -32768, 32767),//mA
};

static enum power_supply_property vpm_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


static enum power_supply_property battery_vpm_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};



//struct battery_vpm_info *battery_vpm_device;

static int battery_vpm_read_command(int vpm_cmd)
{
	struct adv_vpm_data vpm_data;
	s32 ret;
	u16 event_interrupt = 0, shift_high = 0;
	
	vpm_data.wlen = 2;
	vpm_data.rlen = 3;
	vpm_data.data[0] = vpm_cmd;
	vpm_data.data[1] = vpm_cmd;
	
	
	if (vpm_cmd > 0) {
        //If vpm is in bootloader mode, we are forbidden to access vpm.
        if (vpm_is_bootloader_mode()) {
            return battery_vpm_data[VPM_VOLTAGE].max_value;
        } else {
        
        	mutex_lock(&vpm_pack_mutex);
			ret = adv_vpm_tf(&vpm_data);
			mutex_unlock(&vpm_pack_mutex);
			
			shift_high = vpm_data.data[0];
			event_interrupt = (shift_high << 8) + vpm_data.data[1];
			
			//printk("VPM (%d.%d)= %d\n", vpm_data.data[0], vpm_data.data[1], event_interrupt);

            if(ret == 0) {
				return event_interrupt;	//get data from vpm
            }
        }
	}

	return -1;
}

static int battery_vpmread(int vpm_cmd)
{
	s32 ret;
	
	ret = battery_vpm_read_command(vpm_cmd);

	return ret;
}

static int battery_vpm_get_present(union power_supply_propval *val)
{
	s32 ret;

	ret = battery_vpmread(VPM_BATTERY_PACK_STATE_OF_CHARGE);
	
	//printk("battery_vpm_get_present: 0x%4X\n",ret);
	
	ret = ret & 0x00FF;

	if (ret < 0)
		return ret;

	if (ret == BATT_DISATTACHED) {
		//printk("vpm: battery un-plug \n");
		val->intval = 0;
	} else {
        //printk("vpm: battery plug in\n");
        val->intval = 1;
	}

	return 0;
}

static int battery_vpm_get_charged_bit(union power_supply_propval *val)
{
	s32 ret = 0;
	
	ret = battery_vpmread(VPM_BATTERY_PACK_FLAGS);
	printk("1. battery_vpm_get_status: 0x%4X\n",ret);
	
	ret = ret & 0x0020 ;
	printk("2. battery_vpm_get_status: 0x%4X\n",ret);

	if (ret < 0)
		return ret;
	
	val->intval = ret;
	
}

static void  battery_vpm_unit_adjustment(
	enum power_supply_property psp, union power_supply_propval *val)
{
#define BASE_UNIT_CONVERSION		1000
#define BATTERY_MODE_CAP_MULT_WATT	(10 * BASE_UNIT_CONVERSION)
#define TIME_UNIT_CONVERSION		600
#define TEMP_KELVIN_TO_CELCIUS		2731
	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:	
		/* battery_vpm spec says that this can be >100 %
		* even if max value is 100 % */
		val->intval = min(val->intval, 100);
		break;
//	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
//	case POWER_SUPPLY_PROP_CURRENT_NOW:
//		val->intval *= BASE_UNIT_CONVERSION;
//		break;
	case POWER_SUPPLY_PROP_TEMP:
		/* battery_vpm provides battery tempreture in 0.1K
		 * so convert it to 0.1C */
		val->intval -= TEMP_KELVIN_TO_CELCIUS;
		break;

//	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
//	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
//		val->intval *= TIME_UNIT_CONVERSION;
//		break;

	default:
		//printk(KERN_DEBUG "%s: no need for unit conversion %d\n", __func__, psp);
		break;
	}
}

static int battery_vpm_get_battery_property(
	int reg_offset, enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;

	ret = battery_vpmread(battery_vpm_data[reg_offset].addr);
	//printk(KERN_INFO "battery_vpm: [%d]  0x%2X = 0x%4X\n", psp,battery_vpm_data[reg_offset].addr,ret);

	if (ret < 0)
		return ret;

	/* returned values are 16 bit */
	if (battery_vpm_data[reg_offset].min_value < 0)
		ret = (s16)ret;

	if (ret >= battery_vpm_data[reg_offset].min_value &&
	    ret <= battery_vpm_data[reg_offset].max_value) {
		val->intval = ret;

		battery_vpm_unit_adjustment(psp, val);
		
		if (psp == POWER_SUPPLY_PROP_STATUS) {
			ret = ret >> 8;
			switch (ret) {
				case BATT_CHARGING:
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
					break;
				case BATT_FULL_CHARGED:
					val->intval = POWER_SUPPLY_STATUS_FULL;
					break;
				case BATT_DC_OUT:
					val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
					break;	
				default:
					val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
					break;					
			}
		}
	} 

	return 0;
}

static int vpm_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct battery_vpm_info *data = psy->drv_data;
	int ret, state;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:

		// Use Battery Pack State (0x9F) command to check AC online property
		//   0x00 - Battery disattached. (DC mode)
		//   0x01 - Battery not fully charged
		//   0x02 - Battery fully charged
		//   0x03 - DC out, and Battery attached. (Battery mode)
		
		ret = battery_vpmread(VPM_BATTERY_PACK_STATE_OF_CHARGE);

		state = ret & 0x0F;
		
		if (state == 0x03)  //Battery mode
			val->intval = 0;
		else
			val->intval = 1;

		//printk(KERN_DEBUG "%s, state = %x, online = %d\n", __func__, state, val->intval);
		
		break;
	default:
		return -EINVAL;
	}

	return 0;
}



static int battery_vpm_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int count;
	int ret;
	union power_supply_propval val_check;
	
	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		battery_vpm_get_present(val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
	
		if ((psp == POWER_SUPPLY_PROP_CAPACITY) || (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW) || \
			(psp == POWER_SUPPLY_PROP_CURRENT_NOW) || (psp == POWER_SUPPLY_PROP_TEMP) || \
			(psp == POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG) || (psp == POWER_SUPPLY_PROP_TIME_TO_FULL_AVG))
		{	
			battery_vpm_get_present(val);
			if(val->intval == 0)
			{
				val->intval = 0xFFFFF; // battery unplug-in
				return 0;
			}
		}
	
		for (count = 0; count < ARRAY_SIZE(battery_vpm_data); count++) {
			if (psp == battery_vpm_data[count].psp)
				break;
		}

		ret = battery_vpm_get_battery_property(count, psp, val);
		
		// *** Double check full charged -> CAPACITY=100%
		if (psp == POWER_SUPPLY_PROP_CAPACITY)
		{	
			battery_vpm_get_charged_bit(&val_check);
			//printk("cap = %d, status = %d\n", val->intval, val_check.intval );
			
			if(val_check.intval)
			{
				val->intval = 100; 
				return 0;
			}
		}
		// ***
		
		
		if (ret)
			return ret;

		break;

	default:
		printk(KERN_ERR "%s: INVALID property\n", __func__);
		return -EINVAL;
	}

	return 0;
}


static const struct of_device_id vpm_dt_ids[] = {
        { .compatible = "fsl,vpm-battery", },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vpm_dt_ids);


static int battery_vpm_probe(struct platform_device *pdev)
{
	struct battery_vpm_info *data;
	struct device *dev = &pdev->dev;
	struct battery_vpm_pinfo *pdata = pdev->dev.platform_data;
	struct power_supply_config psy_cfg = {};
	int rc;

	printk("%s \n", __func__);

	
	data = devm_kzalloc(dev, sizeof(struct battery_vpm_info), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	
	pdata = pdev->dev.platform_data;

	data->pdata = pdata;
	data->dev = dev;
	platform_set_drvdata(pdev, data);
	data->psy_desc.name = "ac";
	data->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
	data->psy_desc.get_property = vpm_charger_get_property;
	data->psy_desc.properties = vpm_charger_properties;
	data->psy_desc.num_properties = ARRAY_SIZE(vpm_charger_properties);
	
	psy_cfg.drv_data = data;
	
	
	data->psy = power_supply_register(dev, &data->psy_desc, &psy_cfg);
	if (IS_ERR(data->psy)) {
		printk("%s failed: power supply register.\n", __func__);
		//goto err_psy;
	}
	
	data->bat_desc.name = "battery";
	data->bat_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	data->bat_desc.get_property = battery_vpm_get_property;
	data->bat_desc.properties = battery_vpm_properties;
	data->bat_desc.num_properties = ARRAY_SIZE(battery_vpm_properties);
	
	psy_cfg.drv_data = data;
	
	
	data->psy = power_supply_register(&pdev->dev, &data->bat_desc, &psy_cfg);
	if (IS_ERR(data->psy)) {
		printk("%s failed: power supply register.\n", __func__);
		//goto err_psy;
	}

	return 0;
}

static int battery_vpm_remove(struct platform_device *pdev)
{
	struct battery_vpm_info *data = platform_get_drvdata(pdev);
	struct battery_vpm_pinfo *pdata = data->pdata;

	power_supply_unregister(data->psy);
	
	kfree(data);
//	battery_vpm_device = NULL;
	return 0;
}
/*
#if defined CONFIG_PM

static int battery_vpm_suspend(pm_message_t state)
{
	hrtimer_cancel(&battery_vpm_device->timer);
	return 0;
}

static int battery_vpm_resume(pm_message_t state)
{
	hrtimer_start(&battery_vpm_device->timer, battery_vpm_device->debounce_time, HRTIMER_MODE_REL); 
	return 0;
}
*/
//#else
#define battery_vpm_suspend		NULL
#define battery_vpm_resume		NULL
//#endif
/*
static int  battery_vpm_shutdown(pm_message_t state)
{

	//battery_vpm_write_command(VPM_STATUS_INTERRUPT, VPM_I2C_INTERRUPT_DISABLE);
      printk("battery_vpm_shutdown\n");

	hrtimer_cancel(&battery_vpm_device->timer);
  	power_supply_unregister(&battery_vpm_device->power_supply);
	kfree(battery_vpm_device);
	battery_vpm_device = NULL;

}
*/

/* any smbus transaction will wake up battery_vpm */

static struct platform_driver battery_vpm_battery_driver = {
	.probe		= battery_vpm_probe,
	.remove		= battery_vpm_remove,
	.suspend	= battery_vpm_suspend,
	.resume		= battery_vpm_resume,
	//.shutdown		= battery_vpm_shutdown,
	.driver = {
		.name	= "battery-vpm",
		.owner	= THIS_MODULE,
		.of_match_table = vpm_dt_ids,//aaronlin
	},
};



static int __init battery_vpm_battery_init(void)
{
	int err;

	if((err = platform_driver_register(&battery_vpm_battery_driver)))
		return err;

	if ((vpm_battery = platform_device_alloc("battery-vpm", 0)) == NULL) {
		err = -ENOMEM;
		goto exit_driver_unregister;
	}

	if ((err = platform_device_add(vpm_battery)))
		goto exit_free_vpm_battery;

	//if (err = register_reboot_notifier(&battery_vpm_notifier))
	//	goto exit_free_vpm_battery_nofifier;

	return 0;

exit_free_vpm_battery:
	platform_device_put(vpm_battery);

exit_driver_unregister:
	platform_driver_unregister(&battery_vpm_battery_driver);
	
	return err;
}
module_init(battery_vpm_battery_init);

static void __exit battery_vpm_battery_exit(void)
{
	platform_device_unregister(vpm_battery);
	platform_driver_unregister(&battery_vpm_battery_driver);	

}
module_exit(battery_vpm_battery_exit);

MODULE_DESCRIPTION("VPM battery driver");
MODULE_LICENSE("GPL");
