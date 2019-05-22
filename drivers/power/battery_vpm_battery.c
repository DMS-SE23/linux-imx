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
	VPM_FLAGS,
	VPM_TIME_TO_FULL,
	VPM_HEALTH,
	VPM_CHARGE_NOW,
	VPM_FULL_CHARGE_FULL,
	VPM_DEVICE_NAME,
	VPM_SERIAL_NUMBER,
	VPM_MANUFACTURER_NAME,
	VPM_CYCLE_COUNT,
	VPM_STATUS,
	VPM_TEMPERATURE,
	VPM_VOLTAGE,
	VPM_CURRENT,
	VPM_ATRATE_READ,
	VPM_ATRATE_WRITE,
	VPM_ATRATE_TIME_TO_EMPTY
	};

#define POLL_INTERVAL			30
#define CHARGE_FLAG_CHANGE_INTERVAL	3

enum {
	VPM_READ_FLAG = 0,
	VPM_WRITE_FLAG,
};



static struct platform_device *vpm_battery = NULL;

#define BATTERY_VPM_DATA(_psp, _addr, _min_value, _max_value, _i2c_length) { \
	.psp = _psp, \
	.addr = _addr, \
	.min_value = _min_value, \
	.max_value = _max_value, \
	.i2c_length = _i2c_length, \
}

static const struct battery_vpm_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
	int i2c_length;
} battery_vpm_data[] = {
	[VPM_CAPACITY] =				BATTERY_VPM_DATA(POWER_SUPPLY_PROP_CAPACITY, 0x90, 0, 100, 3), //0x1:Primary, 0x2: Secondary
	[VPM_TIME_TO_EMPTY] =			BATTERY_VPM_DATA(POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG, 0x91, 0, 65535, 3), //min.
	[VPM_FLAGS] =			        BATTERY_VPM_DATA(POWER_SUPPLY_PROP_FLAGS, 0x92, 0, 65535, 3),
	[VPM_TIME_TO_FULL] =			BATTERY_VPM_DATA(POWER_SUPPLY_PROP_TIME_TO_FULL_AVG, 0x96, 0, 65535, 3), //min.
	[VPM_HEALTH] =					BATTERY_VPM_DATA(POWER_SUPPLY_PROP_HEALTH, 0x97, 0, 100, 3), 
	[VPM_CHARGE_NOW] =			    BATTERY_VPM_DATA(POWER_SUPPLY_PROP_CHARGE_NOW, 0x98, 0, 65536, 3), 
	[VPM_FULL_CHARGE_FULL] =	    BATTERY_VPM_DATA(POWER_SUPPLY_PROP_CHARGE_FULL, 0x99, 0, 65536, 3), 
	[VPM_DEVICE_NAME] =	            BATTERY_VPM_DATA(POWER_SUPPLY_PROP_MODEL_NAME, 0x9A, 0, 65536, 20), 
	[VPM_SERIAL_NUMBER] =	        BATTERY_VPM_DATA(POWER_SUPPLY_PROP_SERIAL_NUMBER, 0x9E, 0, 65536, 10), 
	[VPM_MANUFACTURER_NAME] =	    BATTERY_VPM_DATA(POWER_SUPPLY_PROP_MANUFACTURER, 0x9C, 0, 65536, 20), 
	[VPM_CYCLE_COUNT] = 			BATTERY_VPM_DATA(POWER_SUPPLY_PROP_CYCLE_COUNT, 0x9D, 0, 65535, 3),
	[VPM_STATUS] =					BATTERY_VPM_DATA(POWER_SUPPLY_PROP_STATUS, 0x9F, 0, 65535, 3), 
	[VPM_TEMPERATURE] =				BATTERY_VPM_DATA(POWER_SUPPLY_PROP_TEMP, 0x93, 0, 65535, 3), //0.1K C=K-237.15
	[VPM_VOLTAGE] =					BATTERY_VPM_DATA(POWER_SUPPLY_PROP_VOLTAGE_NOW, 0x94, 0, 65535, 3), //mV
	[VPM_CURRENT] = 			    BATTERY_VPM_DATA(POWER_SUPPLY_PROP_CURRENT_NOW, 0x95, -32768, 32767, 3),//mA
	[VPM_ATRATE_READ] = 			    BATTERY_VPM_DATA(POWER_SUPPLY_PROP_ATRATE_READ, 0x80, -32768, 32767, 3),//mA
	[VPM_ATRATE_WRITE] = 			    BATTERY_VPM_DATA(POWER_SUPPLY_PROP_ATRATE_WRITE, 0x81, -32768, 32767, 3),//mA
	[VPM_ATRATE_TIME_TO_EMPTY] = 	BATTERY_VPM_DATA(POWER_SUPPLY_PROP_ATRATE_TIME_TO_EMPTY, 0x82, 0, 65536, 3),//min
};

static enum power_supply_property vpm_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


static enum power_supply_property battery_vpm_properties[] = {
	POWER_SUPPLY_PROP_FLAGS,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ATRATE_READ,
	POWER_SUPPLY_PROP_ATRATE_WRITE,
	POWER_SUPPLY_PROP_ATRATE_TIME_TO_EMPTY,
};


static int battery_vpm_write_command(struct adv_vpm_data *vpm_data)
{
	s32 ret;

	if (vpm_data->data[0] > 0) {
        //If vpm is in bootloader mode, we are forbidden to access vpm.
        if (vpm_is_bootloader_mode()) {
            return NULL;
        } else {
        	mutex_lock(&vpm_pack_mutex);
			ret = adv_vpm_tf(vpm_data);
			mutex_unlock(&vpm_pack_mutex);

			return 0;
        }
	}
	return -1;
}

//struct battery_vpm_info *battery_vpm_device;

static int battery_vpm_read_command(struct adv_vpm_data *vpm_data)
{
	//struct adv_vpm_data vpm_data;
	s32 ret;
	u16 event_interrupt = 0, shift_high = 0;
	
	if (vpm_data->data[0] > 0) {
        //If vpm is in bootloader mode, we are forbidden to access vpm.
        if (vpm_is_bootloader_mode()) {
            return NULL;
        } else {
        
        	mutex_lock(&vpm_pack_mutex);
			ret = adv_vpm_tf(vpm_data);
			mutex_unlock(&vpm_pack_mutex);
			
			shift_high = vpm_data->data[0];
			event_interrupt = (shift_high << 8) + vpm_data->data[1];
			
			//printk("VPM (%d.%d.%d.%d)\n", vpm_data->data[0], vpm_data->data[1], vpm_data->data[2], vpm_data->data[3]);

            if(ret == 0) {
				return event_interrupt;	//get data from vpm
            }
        }
	}

	return -1;
}

//static int battery_vpmread(struct adv_vpm_data vpm_data)
//{
//	s32 ret;
//	
//	ret = battery_vpm_read_command(vpm_data);
//
//	return ret;
//}

static int battery_vpm_get_present(union power_supply_propval *val)
{
	s32 ret;
	struct adv_vpm_data vpm_data;

	vpm_data.wlen = 2;
	vpm_data.rlen = 3;
	vpm_data.data[0] = VPM_BATTERY_PACK_STATE_OF_CHARGE;
	vpm_data.data[1] = VPM_BATTERY_PACK_STATE_OF_CHARGE;

	ret = battery_vpm_read_command(&vpm_data);
	//ret = battery_vpmread(VPM_BATTERY_PACK_STATE_OF_CHARGE);
	
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
	struct adv_vpm_data vpm_data;

	vpm_data.wlen = 2;
	vpm_data.rlen = 3;
	vpm_data.data[0] = VPM_BATTERY_PACK_FLAGS;
	vpm_data.data[1] = VPM_BATTERY_PACK_FLAGS;
	
	ret = battery_vpm_read_command(&vpm_data);
	//ret = battery_vpmread(VPM_BATTERY_PACK_FLAGS);
	//printk("1. battery_vpm_get_status: 0x%4X\n",ret)
	ret = ret & 0x0020 ;
	//printk("2. battery_vpm_get_status: 0x%4X\n",ret);

	if (ret < 0)
		return ret;

	val->intval = ret;
}

static int battery_vpm_get_time_to_empty(union power_supply_propval *val)
{
	int ret = 0, state = 0;
	s32 Remain_capacity = 0, Current = 0;
	struct adv_vpm_data vpm_data;

	vpm_data.wlen = 2;
	vpm_data.rlen = 3;
	vpm_data.data[0] = VPM_BATTERY_PACK_STATE_OF_CHARGE;
	vpm_data.data[1] = VPM_BATTERY_PACK_STATE_OF_CHARGE;

	ret = battery_vpm_read_command(&vpm_data);

	state = ret & 0x0F;

	if (state != 0x03)  //AC-IN mode
		val->intval = 65535;
	else
	{
		vpm_data.wlen = 2;
		vpm_data.rlen = battery_vpm_data[VPM_CHARGE_NOW].i2c_length;
		vpm_data.data[0] = battery_vpm_data[VPM_CHARGE_NOW].addr;
		vpm_data.data[1] = battery_vpm_data[VPM_CHARGE_NOW].addr;

		Remain_capacity = battery_vpm_read_command(&vpm_data);
		//printk("Remain_capacity: %d\n", Remain_capacity);

		vpm_data.wlen = 2;
		vpm_data.rlen = battery_vpm_data[VPM_CURRENT].i2c_length;
		vpm_data.data[0] = battery_vpm_data[VPM_CURRENT].addr;
		vpm_data.data[1] = battery_vpm_data[VPM_CURRENT].addr;

		Current = battery_vpm_read_command(&vpm_data);

		if (battery_vpm_data[VPM_CURRENT].min_value < 0)
			Current = (s16)Current;

		//printk("Current: %d\n", Current);

		Current = (Current > 0)? Current : Current*(-1);

		ret = (int)Remain_capacity * 60 / Current;

		if (ret < 0)
			return ret;
		val->intval = ret;
	}
}
/*
Error Report
bit0: Unknown
bit1: Unspecified failure
bit2: Over-charged
bit3: Over-temperature
bit4: Dead
*/
static void battery_vpm_error_bit(s32 error_flag, union power_supply_propval *val)
{
	int i, j = 0, i2c_error_code = 0;
	static char adv_char[16] = {0}, adv_flag[8] = {0}, adv_flag_report[8] = {0}, i2c_error[4] = {0};
	char *ap = adv_char, *ap_report = adv_flag_report;
	union power_supply_propval val_check;

	//printk("battery_vpm_error_bit: %d\n", error_flag);

	for( i = 0; i < Battery_Err_Total; i++)// Clean error status
		adv_flag[i] = 0;

	battery_vpm_get_present(&val_check);
	if(! val_check.intval) // error report : battery unplug-in
			adv_flag[Battery_Err_Unknown] = 1;
	else
	{
		for(i = 32768; i ; i >>= 1 )
		{
			//printk("%d-%d ",(16-j), (error_flag&i)?1:0);
			adv_char[j++] = (error_flag&i)?1:0;
			if(error_flag&i)
			{
				//printk("bit=%d\n", (16-j));
				switch (16-j) {
					case Battery_Err0:
					case Battery_Err1:
					case Battery_Err2:
					case Battery_Err3:
						i2c_error[16-j] = 1;
						break;
					case Battery_Overtemperature_Alarm:
						adv_flag[Battery_Err_Over_Heat] = 1;
						break;
					case Battery_Overcharged_Alarm:
						adv_flag[Battery_Err_Over_Voltage] = 1;
						break;
					//case Battery_Terminate_Discharge_Alarm:
					//case Battery_Terminate_Charge_Alarm:
					//	adv_flag[Battery_Err_Unspecified_Fail] = 1;
					//	break;
				}
			}
		}
	}

	for(i = 0; i <= 3; i++)
		i2c_error_code += i2c_error[i] << i;

	switch (i2c_error_code) {
		case I2C_UnknownError:
		case I2C_Badsize:
		case I2C_Overflow_Underflow:
		case I2C_Unsupported_Command:
			adv_flag[Battery_Err_Unspecified_Fail] = 1;
			printk("i2c_error_code=%d\n", i2c_error_code);
			break;
		}

	for( i = 1; i <= Battery_Err_Total; i++)
		ap_report += sprintf(ap_report, "%d", adv_flag[Battery_Err_Total-i]);

	val->strval = adv_flag_report;

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
	static char adv_char[20] = {0};
	char *ap = adv_char;
	int i = 0;
	struct adv_vpm_data vpm_data;

	vpm_data.wlen = 2;
	vpm_data.rlen = battery_vpm_data[reg_offset].i2c_length;
	vpm_data.data[0] = battery_vpm_data[reg_offset].addr;
	vpm_data.data[1] = battery_vpm_data[reg_offset].addr;

	ret = battery_vpm_read_command(&vpm_data);

	if (ret < 0)
		return ret;

	/* returned values are 16 bit */
	if (battery_vpm_data[reg_offset].min_value < 0)
		ret = (s16)ret;

	if (ret >= battery_vpm_data[reg_offset].min_value &&
	    ret <= battery_vpm_data[reg_offset].max_value) {
		val->intval = ret;
		
		battery_vpm_unit_adjustment(psp, val);
		switch (psp) {
			case POWER_SUPPLY_PROP_FLAGS:
				battery_vpm_error_bit(ret, val);
				break;
			case POWER_SUPPLY_PROP_STATUS:
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
				break;
			case POWER_SUPPLY_PROP_MANUFACTURER:
			case POWER_SUPPLY_PROP_MODEL_NAME:
				for(i = 1; i <= vpm_data.data[0]; i++ )
					ap += sprintf(ap, "%c", vpm_data.data[i]);

				val->strval = adv_char;
				break;
			case POWER_SUPPLY_PROP_SERIAL_NUMBER:
				for(i = 0; i < vpm_data.rlen; i++) {
					if(((vpm_data.data[i] >= '0') && (vpm_data.data[i] <= '9')) || \
					   ((vpm_data.data[i] >= 'A') && (vpm_data.data[i] <= 'Z')) || \
					   ((vpm_data.data[i] >= 'a') && (vpm_data.data[i] <= 'z'))) {

						ap += sprintf(ap, "%c", vpm_data.data[i]);
					} else {
						pr_info("[BATT_SN] : unknown data[%d] = 0x%x\n", i, vpm_data.data[i]);

						//fill with char '*'
						memset(adv_char, 0x2A, vpm_data.rlen);
						break;
					}
				}
				pr_info("[BATT_SN] : %s\n", adv_char);
				
				val->strval = adv_char;
				break;
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
	struct adv_vpm_data vpm_data;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:

		// Use Battery Pack State (0x9F) command to check AC online property
		//   0x00 - Battery disattached. (DC mode)
		//   0x01 - Battery not fully charged
		//   0x02 - Battery fully charged
		//   0x03 - DC out, and Battery attached. (Battery mode)
		
		vpm_data.wlen = 2;
		vpm_data.rlen = 3;
		vpm_data.data[0] = VPM_BATTERY_PACK_STATE_OF_CHARGE;
		vpm_data.data[1] = VPM_BATTERY_PACK_STATE_OF_CHARGE;

		ret = battery_vpm_read_command(&vpm_data);

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
	case POWER_SUPPLY_PROP_HEALTH:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_MODEL_NAME:
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
	case POWER_SUPPLY_PROP_MANUFACTURER:
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
	case POWER_SUPPLY_PROP_CHARGE_NOW:
	case POWER_SUPPLY_PROP_FLAGS:
	case POWER_SUPPLY_PROP_ATRATE_READ:
	case POWER_SUPPLY_PROP_ATRATE_TIME_TO_EMPTY:
	
	if ((psp == POWER_SUPPLY_PROP_CAPACITY) || (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW) || \
			(psp == POWER_SUPPLY_PROP_CURRENT_NOW) || (psp == POWER_SUPPLY_PROP_TEMP) || \
			(psp == POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG) || (psp == POWER_SUPPLY_PROP_TIME_TO_FULL_AVG) || \
			(psp == POWER_SUPPLY_PROP_HEALTH) || (psp == POWER_SUPPLY_PROP_CHARGE_FULL) || \
			(psp == POWER_SUPPLY_PROP_MODEL_NAME) || (psp == POWER_SUPPLY_PROP_SERIAL_NUMBER) || \
			(psp == POWER_SUPPLY_PROP_MANUFACTURER) || (psp == POWER_SUPPLY_PROP_CYCLE_COUNT) || \
			(psp == POWER_SUPPLY_PROP_CHARGE_NOW) || (psp == POWER_SUPPLY_PROP_ATRATE_READ) || \
			(psp == POWER_SUPPLY_PROP_ATRATE_TIME_TO_EMPTY))
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
		
		switch (psp) {
		case POWER_SUPPLY_PROP_CAPACITY:
			// *** Double check full charged -> CAPACITY=100%
			battery_vpm_get_charged_bit(&val_check);
			//printk("cap = %d, status = %d\n", val->intval, val_check.intval );
			if(val_check.intval)
			{
				val->intval = 100;
				return 0;
			}
			break;
		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
			battery_vpm_get_time_to_empty(val);
			break;
		}
		if (ret)
			return ret;

		break;
	default:
		printk(KERN_ERR "%s: INVALID property\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int battery_vpm_set_atrate(int val)
{
	struct adv_vpm_data vpm_data;
	s16 ret;
	char val_high, val_low;

	ret = (s16)val;
	val_high = ret >> 8;
	val_low = (ret << 8) >> 8;

	//printk("battery_vpm_set_atrate:0x%4X: %d (%d,%d)", VPM_BATTERY_PACK_ATRATE_SET, val, val_high, val_low);

	vpm_data.wlen = 4;
	vpm_data.rlen = 0;
	vpm_data.data[0] = VPM_BATTERY_PACK_ATRATE_SET;
	vpm_data.data[1] = val_high;
	vpm_data.data[2] = val_low;
	vpm_data.data[3] = VPM_BATTERY_PACK_ATRATE_SET ^ val_high ^ val_low;

	ret = battery_vpm_write_command(&vpm_data);
	return 0;
}

static int battery_vpm_set_property(struct power_supply *ps,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	int count;
	int ret;
	union power_supply_propval val_check;
	
	switch (prop) {
	case POWER_SUPPLY_PROP_ATRATE_WRITE:
			battery_vpm_set_atrate(val->intval);
			break;
	default:
			printk(KERN_ERR "%s: INVALID property\n", __func__);
			return -EINVAL;
	}
	return 0;
}


static int battery_vpm_property_is_writeable(struct power_supply *psy,
	enum power_supply_property psp)
{
	int ret = 0;
	switch (psp) {
	case POWER_SUPPLY_PROP_ATRATE_WRITE:
		ret = 1;
		break;
	default:
		ret = 0;
	}
	return ret;
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
	struct power_supply_config psy_cfg = {}, bat_cfg = {};
	int rc, err = -1;

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
	data->bat_desc.properties = battery_vpm_properties;
	data->bat_desc.num_properties = ARRAY_SIZE(battery_vpm_properties);
	data->bat_desc.get_property = battery_vpm_get_property;
	data->bat_desc.set_property	= battery_vpm_set_property;
	data->bat_desc.property_is_writeable= battery_vpm_property_is_writeable;
	
	bat_cfg.drv_data = data;
	
	data->bat = power_supply_register(dev, &data->bat_desc, &bat_cfg);

	if (IS_ERR(data->bat)) {
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
