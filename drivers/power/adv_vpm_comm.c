/*
 * Advantech <-> VPM driver
 */
#include <linux/types.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/adv_vpm_comm.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
/* input */
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define ADV_VPM_DEBUG 1

#if ADV_VPM_DEBUG
#define dbg(fmt, ...) printk(KERN_DEBUG "[%s] : " fmt, __func__, ##__VA_ARGS__)
#define err(fmt, ...) printk(KERN_ERR "[%s] : " fmt, __func__, ##__VA_ARGS__)
#else
#define dbg(fmt, ...)
#define err(fmt, ...)
#endif

#define info(fmt, ...) printk(KERN_INFO "[%s] : " fmt, __func__, ##__VA_ARGS__)

#define DRIVER_NAME "adv_vpm_comm"
#define DRIVER_VERSION "0.01"

#define ADV_PIC_MINOR	MISC_DYNAMIC_MINOR
#define I2C_Read 1
#define I2C_Write 2

#define ADV_VPM_I2C_ID		0x78

#define MAX_RETRIES 5
#define	ERREVENTID		-1	/* error event id */

#if VPM_KEYPAD_EVENT_SUPPORTED

#define LINUX_Key1							105
#define LINUX_Key2							106
#define LINUX_Key3							28
#define LINUX_Key4							108
#define LINUX_Key5				   			103

#define ADV_POWER_OFF_KEYCODE				KEY_POWER

char event_data;

static u16 hotkey_matrix [] = {
	LINUX_Key1, LINUX_Key2, LINUX_Key3, LINUX_Key4, LINUX_Key5,
};
char hotkey_status [] = {0, 0, 0, 0, 0};
char hotkey_status_previous [] = {0, 0, 0, 0, 0};

#endif

static struct input_dev *tbtnDev = NULL;
//static DEFINE_MUTEX(vpm_mutex);
struct adv_vpm_management_data *data = NULL;
static char *d_rec;
static struct i2c_client *client_vpm = NULL;
static const struct i2c_device_id adv_vpm_id[] = {
	{ "adv_vpm", 0},
	{ }
};

struct adv_vpm_management_data {
	struct work_struct work;
	struct workqueue_struct *workqueue;
#if VPM_DOCKING_SUPPORTED
	struct work_struct docking_work;
	struct workqueue_struct *docking_workqueue;
	struct adv_vpm_platform_data platform_data;
#endif
};

unsigned char module_control_h;
unsigned char module_control_l;
unsigned char bootloaderMode = 0;
unsigned char check_mode_status = 0;
u32 irq_gpio;

//array of vpm event handler function pointers
eventhandler vpm_event_handler_array[VPM_EVENT_TOTAL]= {0};
 //extern int m25p80_get_config(const char* name, char *buf);
 //extern int m25p80_set_config(const char* name, char *buf);

/**
 *	RegisterVPMEventFunc - register handler callback function for the specific interrupt event
 */
void RegisterVPMEventFunc(enum_vpm_int_event event_id, eventhandler handlerfunc)
{
	if( vpm_event_handler_array[event_id] == 0)
	{
		vpm_event_handler_array[event_id] = handlerfunc;
	}
	else
	{
		printk("*** VPM ERROR: Event handler has set already!! WILL NOT SET AGAIN. event_id = %d \n", event_id);
	}
}

void vpm_no_event_handler_func(void)
{
	dbg("VPM: This is vpm_no_event_handler for VPM_NO_EVENT \n");
	dbg("VPM: Process the VPM_NO_EVENT send from VPM... \n");
}


void vpm_powersource_event_handler_func(void)
{
	dbg("VPM: This is vpm_powersource_event_handler_func\n");
}

void vpm_wakeup_event_handler_func(void)
{
	dbg("VPM: This is vpm_wakeup_event_handler_func \n");
}

void vpm_battery_event_handler_func(void)
{
	dbg("VPM: This is vpm_battery_event_handler_func \n");
}

void vpm_gpi_event_handler_func(void)
{
	dbg("VPM: This is vpm_gpi_event_handler_func \n");
}

#if VPM_CRADLE_EVENT_SUPPORTED
void vpm_cradle_event_handler_func(void)
{
	dbg("VPM: This is vpm_cradle_event_handler_func \n");
}
#endif

void vpm_bypass_event_handler_func(void)
{
	adv_vpm_readkey(); // Bypass everything
}

#if VPM_KEYPAD_EVENT_SUPPORTED
static int LAST_KEY = -1;

void vpm_keypad_event_handler_func(void)
{
	char key_id = event_data, i = 0;
	char hotkey_xor[] = {0,0,0,0,0};

	//printk("VPM: key:%d \n", key_id);

	for(i=0; i<5; i++)
	{
		hotkey_status[i] = (char)(key_id << (7-i)) >> 7;
		hotkey_xor[i] = hotkey_status[i] ^ hotkey_status_previous[i];
	}

	for(i=0; i<5; i++)
	{
		if (hotkey_xor[i] == 1)
		{
			if(hotkey_status_previous[i] == 0)
			{
				input_report_key(tbtnDev, hotkey_matrix[i], 1);
				input_sync(tbtnDev);
				printk("VPM key[%d]=%d down \n", i, hotkey_matrix[i]);
			}
			else
			{
				input_report_key(tbtnDev, hotkey_matrix[i], 0);
				input_sync(tbtnDev);
				printk("VPM key[%d]=%d release \n", i, hotkey_matrix[i]);
			}
		}
		
		hotkey_status_previous[i] = hotkey_status[i];
	}
}
#endif

static void vpm_power_off_event_handler_func(void)
{
	__set_bit(ADV_POWER_OFF_KEYCODE, tbtnDev->keybit);

	//printk("VPM: vpm_power_off_event_handler_func %d\n", event_data);

	if (event_data == 1)
	{
		printk("VPM: power_off %d press down\n", ADV_POWER_OFF_KEYCODE);
		input_report_key(tbtnDev, ADV_POWER_OFF_KEYCODE, 1);
		input_sync(tbtnDev);
	}
	else
	{
		printk("VPM: power_off %d release\n", ADV_POWER_OFF_KEYCODE);
		input_report_key(tbtnDev, ADV_POWER_OFF_KEYCODE, 0);
		input_sync(tbtnDev);
	}
}

int adv_i2c_tf(struct adv_vpm_data *tf_data)
{
	int err=0, ret=0;
	struct i2c_msg xfer;

	if(NULL == client_vpm)
	{
		err=-1;
		printk("august_debug null client");
		goto exit_tf;
	}

	mutex_lock(&vpm_mutex);

	if(tf_data->wlen > 0)
	{
		xfer.addr = client_vpm->addr;
		xfer.flags = 0;
		xfer.len = tf_data->wlen;
		xfer.buf = tf_data->data;

		//printk("vpm write(%d) =[%d, %d, %d, %d]\n", tf_data->wlen, tf_data->data[0], tf_data->data[1], tf_data->data[2], tf_data->data[3]);

		if(i2c_transfer(client_vpm->adapter, &xfer, 1) != 1)
		{
			printk(" %s: VPM i2c transfer fail\n", __func__);
			goto exit_tf;
		}
		else
		{
			//printk(" %s: VPM i2c transfer write success\n", __func__);
		}
	}

	if(tf_data->rlen > 0)
	{
		msleep(20);
		xfer.addr = client_vpm->addr;
		xfer.flags = I2C_M_RD;
		xfer.len = tf_data->rlen;
		xfer.buf = tf_data->data;

		if(i2c_transfer(client_vpm->adapter, &xfer, 1) != 1)
		{
			printk(" %s: VPM i2c transfer fail\n", __func__);
			goto exit_tf;
		}

		//printk("VPM read = [%d,%d,%d,%d,%d]\n", xfer.buf[0], xfer.buf[1], xfer.buf[2], xfer.buf[3], xfer.buf[4]);

		memcpy((void*)&tf_data->data,(void*)xfer.buf,tf_data->rlen);
	}

exit_tf:
	mutex_unlock(&vpm_mutex);
	return err;
}

int adv_vpm_tf(struct adv_vpm_data *tf_data)
{
	if (bootloaderMode)
	{
		printk("The system is in the bootloader mode");
		return -1;
	}
	
	return adv_i2c_tf(tf_data);
}

static irqreturn_t adv_vpm_irq_handler(int irq, void *handle)
{
	struct adv_vpm_management_data *data = handle;
	unsigned long start;

	start = jiffies;

	queue_work(data->workqueue, &data->work);

	return IRQ_HANDLED;
}

#if VPM_DOCKING_SUPPORTED
static irqreturn_t adv_vpm_docking_handler(int irq, void *handle)
{
	struct adv_vpm_management_data *data = handle;
	queue_work(data->docking_workqueue, &data->docking_work);
	return IRQ_HANDLED;
}
#endif


int vpm_get_module_control(void)
{
	struct adv_vpm_data tp = {0};

	tp.wlen = 1;
	tp.rlen = 1;
	tp.data[0] = 0x50;
	adv_vpm_tf(&tp);
	module_control_h=tp.data[0];

	tp.wlen = 1;
	tp.rlen = 1;
	tp.data[0] = 0x51;
	adv_vpm_tf(&tp);
	module_control_l=tp.data[0];

	return 0;
}

int vpm_get_version(void)
{
	int ret = -1;
	struct adv_vpm_data tp = {0};
	int major_version;
	int minor_version;

	tp.wlen = 2;
	tp.rlen = 3;
	tp.data[0] = 0xF0;
	tp.data[1] = 0xF0;
	adv_vpm_tf(&tp);

	printk("VPM Version: %d.%d\n", tp.data[0], tp.data[1]);

	ret = 0;

	return ret;
}

void vpm_set_interrupt_status(int status)
{
	struct adv_vpm_data tp = {0};

	tp.wlen = 3;
	tp.rlen = 0;
	tp.data[0] = VPM_SET_INTERRUPT_STATUS;
	tp.data[1] = status;
	tp.data[2] = VPM_SET_INTERRUPT_STATUS ^ status;
	adv_vpm_tf(&tp);
}

int vpm_get_interrupt_status(void)
{
	struct adv_vpm_data tp = {0};

	tp.wlen = 2;
	tp.rlen = 2;
	tp.data[0] = VPM_GET_INTERRUPT_STATUS;
	tp.data[1] = VPM_GET_INTERRUPT_STATUS;
	adv_vpm_tf(&tp);

	printk("VPM Interrupt Status: %d\n", tp.data[0]);

	return (int)(tp.data[0]);
}

int vpm_get_current_sense(void)
{
	struct adv_vpm_data tp = {0};

	tp.wlen = 2;
	tp.rlen = 2;
	tp.data[0] = VPM_GET_BACKLIGHT_CURRENT_SENSE;
	tp.data[1] = VPM_GET_BACKLIGHT_CURRENT_SENSE;
	adv_vpm_tf(&tp);

	int shift_data = 0;

	shift_data = tp.data[0];
	shift_data = (shift_data << 8) + tp.data[1];

	shift_data = (shift_data * 3300)/8192;
	//printk("VPM Current Sense: %d\n", shift_data);

	return shift_data;
}

void vpm_set_amp_mute_status(int status)
{
	struct adv_vpm_data tp = {0};

	tp.wlen = 3;
	tp.rlen = 0;
	tp.data[0] = 0x51;
	tp.data[1] = status;
	tp.data[2] = 0x51 ^ status;
	adv_vpm_tf(&tp);
}

int vpm_get_amp_mute_status(void)
{
	struct adv_vpm_data tp = {0};

	tp.wlen = 2;
	tp.rlen = 2;
	tp.data[0] = 0x50;
	tp.data[1] = 0x50;
	adv_vpm_tf(&tp);

	printk("VPM amp mute status: %d\n", tp.data[0]);

	return (int)(tp.data[0]);
}

int vpm_get_mode(void)
{
	int ret = -1;
	struct adv_vpm_data tp = {0};

	tp.wlen = 2;
	tp.rlen = 2;
	tp.data[0] = 0xFE;
	tp.data[1] = 0xFE;
	adv_vpm_tf(&tp);

	printk("vpm_get_download_mode: %d\n", tp.data[0]);

	return tp.data[0];
}

int vpm_is_bootloader_mode(void)
{
	if (!check_mode_status)
	{
		check_mode_status = 1;
		int mode = vpm_get_mode();
		//It return 0xAA, in bootloader mode.
		//It return 0x55, in user program mode.
		if (mode == 0xaa)
			bootloaderMode = 1;
		else
			bootloaderMode = 0;
	}

	return bootloaderMode;
}

#if VPM_WWAN_SUPPORTED
int vpm_set_wwan_on_off(int on_off)
{
	int ret = -1;
	struct adv_vpm_data tp = {0};

	tp.wlen = 2;
	tp.rlen = 0;
	tp.data[0] = 0x50;

	if(on_off == 1)
	{
		printk("VPM : switch WWAN on  \n");
		//switch WWAN on
		tp.data[1] = 0x04;
	}
	else
	{
		printk("VPM : switch WWAN off  \n");
		//switch WWAN off
		tp.data[1] = 0x00;
	}

	adv_vpm_tf(&tp);

	ret = 0;

	return ret;
}

int vpm_get_wwan_on_off(void)
{
	int ret = -1;
	struct adv_vpm_data tp = {0};

	tp.wlen = 1;
	tp.rlen = 1;
	tp.data[0] = 0x50;

	adv_vpm_tf(&tp);

	dbg("VPM WWAN status = [%d]  \n", tp.data[0]);

	if(tp.data[0] == 0x04)
	{
		printk("VPM : WWAN is on \n");
		ret = 1;
	}
	else
	{
		printk("VPM : WWAN is off  \n");
		ret = 0;
	}

	return ret;
}
#endif

#if VPM_WIFI_MAC_ADDR_SUPPORTED
int vpm_get_wifi_mac_addr(unsigned char *pdata)
{
	struct adv_vpm_data vd;
	unsigned char wifi_mac[6];
	int i;

	if(pdata == NULL)
		return -1;

	for(i=0; i<sizeof(wifi_mac); i++)
	{
		vd.wlen = 1;
		vd.rlen = 1;
		vd.data[0] = VPM_WLAN_MAC_ADDR_BYTE_1 + i;

		adv_vpm_tf(&vd);

		wifi_mac[i] = vd.data[0];
	}

	memcpy(pdata, wifi_mac, sizeof(wifi_mac));

	return 0;
}
#endif

#if VPM_KEYPAD_EVENT_SUPPORTED
int adv_vpm_readkey(void)
{
	int result_key = -1;
	struct adv_vpm_data tp = {0};

	tp.wlen = 1;
	tp.rlen = 1;
	tp.data[0] = VPM_INT_INDEX;
	adv_vpm_tf(&tp);
	result_key = tp.data[0];

	return result_key;
}
#endif

static int adv_vpm_write_power_off(void)
{
	int ret = -1;
	struct adv_vpm_data tp = {0};
	tp.wlen = 2;
	tp.rlen = 1;
	tp.data[0] = 0x39;
	tp.data[1] = 0x39;
	adv_vpm_tf(&tp);

	return tp.data[0];
}

static int adv_vpm_read_event(void)
{
	u16 event_interrupt = 0, shift_high = 0;
	struct adv_vpm_data tp = {0};

	tp.wlen = 2;
	tp.rlen = 3;
	tp.data[0] = VPM_GET_INTERRUPT_EVENT;
	tp.data[1] = VPM_GET_INTERRUPT_EVENT;
	adv_vpm_tf(&tp);
	shift_high = tp.data[0];
	event_interrupt = (shift_high << 8) + tp.data[1];

	//printk("vpm_get_mode: %d, %d, %d\n", tp.data[0], tp.data[1], event_interrupt);

	return event_interrupt;
}

/**
 *	adv_vpm_event_work - process the event when VPM interrupt occurs.
 */
static void adv_vpm_event_work(struct work_struct *work)
{
	int event_interrupt = 0, event_id = 0;

	while(gpio_get_value(irq_gpio))
	{
		//printk("vpm irq gpio%d = %d",irq_gpio, gpio_get_value(irq_gpio));
		event_data = 0, event_interrupt = 0, event_id = 0;

		mutex_lock(&vpm_pack_mutex);

		event_interrupt = adv_vpm_read_event();

		event_id = event_interrupt >> 8;
		event_data = (char)((event_interrupt<<8)>>8);

		if( event_id == ERREVENTID )
		{
			printk("*** VPM ERROR : WRONG EVENT ID \n");
			//1023 return;
		}

		if( vpm_event_handler_array[event_id] == 0)
		{
			printk("*** VPM ERROR: EVENT HANDDLER NOT REGISTERED. event_id = %d \n", event_id);
			//return;
		}

		switch (event_id) {
		case VPM_NO_EVENT:
			dbg("VPM INTERRUPT: VPM_NO_EVENT \n");
			// (*vpm_event_handler_array[VPM_NO_EVENT])();
			break;
		//case VPM_POWER_SOURCE_CHANGED_EVENT:
		//	dbg("VPM INTERRUPT: VPM_POWER_SOURCE_CHANGED_EVENT \n");
		//	(*vpm_event_handler_array[VPM_POWER_SOURCE_CHANGED_EVENT])();
		//	break;
		//case VPM_WAKE_UP_EVENT:
		//	dbg("VPM INTERRUPT: VPM_WAKE_UP_EVENT \n");
		//	(*vpm_event_handler_array[VPM_WAKE_UP_EVENT])();
		//	break;
		//case VPM_BATTERY_EVENT:
		//	dbg("VPM INTERRUPT: VPM_BATTERY_EVENT \n");
		//	(*vpm_event_handler_array[VPM_BATTERY_EVENT])();
		//	break;
		case VPM_GPI_EVENT:
			dbg("VPM INTERRUPT: VPM_GPI_EVENT \n");
			(*vpm_event_handler_array[VPM_GPI_EVENT])();
			break;

#if VPM_CRADLE_EVENT_SUPPORTED	//adv_vpm_comm_0630
		case VPM_CRADLE_EVENT:
			dbg("VPM INTERRUPT: VPM_CRADLE_EVENT \n");
			(*vpm_event_handler_array[VPM_CRADLE_EVENT])();
			break;
#endif	//adv_vpm_comm_0630

#if VPM_KEYPAD_EVENT_SUPPORTED	//adv_vpm_comm_0630
		case VPM_KEYPAD_EVENT:
			dbg("VPM INTERRUPT: VPM_KEYPAD_EVENT \n");
			//printk("VPM INTERRUPT: VPM_KEYPAD_EVENT \n");
			(*vpm_event_handler_array[VPM_KEYPAD_EVENT])();
			break;
#endif	//adv_vpm_comm_0630
		case VPM_POWER_OFF_EVENT:
			(*vpm_event_handler_array[VPM_POWER_OFF_EVENT])();
			//printk("VPM INTERRUPT: VPM_POWER_OFF_EVENT \n");
			break;
		default:
			dbg("VPM INTERRUPT: OTHER EVENT \n");
			printk("VPM INTERRUPT: ByPass EVENT \n");
			(*vpm_event_handler_array[VPM_ByPass_EVENT])();
			//return;
		}

		mutex_unlock(&vpm_pack_mutex);

		msleep(20);
	}
}

#if VPM_DOCKING_SUPPORTED
static void adv_vpm_docking_work(struct work_struct *work)
{
	struct adv_vpm_management_data *data =
		container_of(work, struct adv_vpm_management_data, docking_work);
	struct adv_vpm_platform_data *pdata = &data->platform_data;

	if(pdata->docking_callback != NULL)
		pdata->docking_callback();

	return;
}
#endif



static ssize_t adv_vpm_vpmintmode_write(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	int ret = -1;
	int onoff = 0;

	if ('0' == buf[0])
	{
		vpm_set_interrupt_status(VPM_I2C_INTERRUPT_DISABLE);
		printk("VPM Interrupt off\n");
	}
	else if ('1' == buf[0])
	{
		vpm_set_interrupt_status(VPM_I2C_INTERRUPT_ENABLE);
		printk("VPM Interrupt on\n");
	}

	return count;
}

static ssize_t adv_vpm_vpmintmode_read(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	int onoff = -1;

	onoff = vpm_get_interrupt_status();

	return sprintf(buf, "%d\n", onoff);
}

static ssize_t adv_vpm_curr_sense_read(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	int current_sense = -1;

	current_sense = vpm_get_current_sense();

	return sprintf(buf, "%d\n", current_sense);
}

static ssize_t adv_vpm_amp_mute_write(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	int ret = -1;
	int onoff = 0;

	if ('0' == buf[0])
	{
		vpm_set_amp_mute_status(0x00);
		printk("Disable VPM amp mute\n");
	}
	else if ('1' == buf[0])
	{
		vpm_set_amp_mute_status(0x01);
		printk("Enable VPM amp mute\n");
	}

	return count;
}

static ssize_t adv_vpm_amp_mute_read(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	int onoff = -1;

	onoff = vpm_get_amp_mute_status();

	return sprintf(buf, "%d\n", onoff);
}



static DEVICE_ATTR(vpmintmode, S_IRUGO | S_IWUSR, adv_vpm_vpmintmode_read, adv_vpm_vpmintmode_write); // Enable/Disable VPM Interrupt
static DEVICE_ATTR(vpmbl_curr_sense, S_IRUGO, adv_vpm_curr_sense_read, NULL); // Read backlight current sense
static DEVICE_ATTR(vpm_amp_mute, S_IRUGO | S_IWUSR, adv_vpm_amp_mute_read, adv_vpm_amp_mute_write); // Enable/Disable VPM Interrupt

static struct attribute *adv_vpm_attrs[] = {
	&dev_attr_vpmintmode.attr,
	&dev_attr_vpmbl_curr_sense.attr,
	&dev_attr_vpm_amp_mute.attr,
	NULL
};

static const struct attribute_group adv_vpm_attr_group = {
	.attrs = adv_vpm_attrs,
};


//[DMSST05]Add ioctl operation support - begin
#define IOC_MAGIC '\x66'
#define IOCTL_VALSET _IOW(IOC_MAGIC, 0, struct adv_vpm_data)
#define IOCTL_VALGET _IOR(IOC_MAGIC, 1, struct adv_vpm_data)
#define IOCTL_FW_VALSET _IOR(IOC_MAGIC, 2, struct adv_vpm_data)
#define IOCTL_FW_VALGET _IOR(IOC_MAGIC, 3, struct adv_vpm_data)
#define IOCTL_HOTKEY_VALSET _IOW(IOC_MAGIC, 4, struct adv_vpm_data)
#define IOCTL_HOTKEY_VALGET _IOW(IOC_MAGIC, 5, struct adv_vpm_data)
static unsigned int ioctl_major = 0;
static unsigned int num_of_dev = 1;
static struct cdev ioctl_cdev;
static int ioctl_num = 0;
static struct class * vpm_class;
static char * devicename = "vpm";
static char * classname = "vpm_class";

static int vpm_ioctl_open(struct inode *inode, struct file *filp)
{
	//printk(KERN_ALERT "%s call.\n", __func__);
	return 0;
}

static int vpm_ioctl_close(struct inode *inode, struct file *filp)
{
	//printk(KERN_ALERT "%s call.\n", __func__);
	return 0;
}

static long vpm_ioctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	//printk(KERN_ALERT "%s call %d\n", __func__, cmd);
	int retval = 0;
	unsigned char val;
	struct adv_vpm_data tp;

	memset(&tp, 0, sizeof(tp));
	switch (cmd) {
	case IOCTL_VALSET:
		if (copy_from_user(&tp, (int __user *)arg, sizeof(tp))) 
		{
			retval = -EFAULT;
			goto done;
		}

		if (tp.data[0] == 0x0b && tp.data[1] == 0x05)
		{
			bootloaderMode =1;
			retval = adv_i2c_tf(&tp);
		}
		else
		{
			retval = adv_vpm_tf(&tp);
		}
		break;
    case IOCTL_VALGET:
		if (copy_from_user(&tp, (int __user *)arg, sizeof(tp)))
		{
			retval = -EFAULT;
			goto done;
		}

		if (tp.data[0] == 0x0b && tp.data[1] == 0x05)
		{
			bootloaderMode =1;
			retval = adv_i2c_tf(&tp);
		}
		else
		{
			retval = adv_vpm_tf(&tp);
		}

		if (copy_to_user((int __user *)arg, &tp, sizeof(tp)) )
		{
			retval = -EFAULT;
			goto done;
		}
		break;
	case IOCTL_FW_VALSET:
		if (copy_from_user(&tp, (int __user *)arg, sizeof(tp)))
		{
			retval = -EFAULT;
			goto done;
		}
		bootloaderMode = 1;
		retval = adv_i2c_tf(&tp);
		break;
	case IOCTL_FW_VALGET:
		if (copy_from_user(&tp, (int __user *)arg, sizeof(tp)))
		{
			retval = -EFAULT;
			goto done;
		}
		bootloaderMode = 1;
		retval = adv_i2c_tf(&tp);
		if (copy_to_user((int __user *)arg, &tp, sizeof(tp)) )
		{
			retval = -EFAULT;
			goto done;
		}
		break;
    case IOCTL_HOTKEY_VALSET:
		if (copy_from_user(&tp, (int __user *)arg, sizeof(tp)))
		{
			retval = -EFAULT;
			goto done;
		}
		//adv_Hotkey_Set(&tp);
	case IOCTL_HOTKEY_VALGET:
		if (copy_from_user(&tp, (int __user *)arg, sizeof(tp))) 
		{
			retval = -EFAULT;
			goto done;
		}
		//adv_Hotkey_Get(&tp);
		if (copy_to_user((int __user *)arg, &tp, sizeof(tp)) )
		{
			retval = -EFAULT;
			goto done;
		}
		break;
	default:
		retval = -ENOTTY;
	}
done:
	return retval;
}

ssize_t vpm_ioctl_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	//printk(KERN_ALERT "%s call.\n", __func__);
	return 0;
}

static ssize_t vpm_ioctl_write(struct file *pfile, const char *user_buf, size_t len, loff_t *off)
{
	//printk(KERN_ALERT "%s call.\n", __func__);
	return 0;
}

struct file_operations fops = {
	.owner = THIS_MODULE,
	.open = vpm_ioctl_open,
	.release = vpm_ioctl_close,
	.read = vpm_ioctl_read,
	.write = vpm_ioctl_write,
	.unlocked_ioctl = vpm_ioctl_ioctl,
};

static int adv_vpm_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int alloc_ret = 0;
	int cdev_ret = 0;
	int j = 0;
	bool is_empty = true;

	struct device_node *devnode = client->dev.of_node;;

	int i, err = -1;

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
	if(!(data = kzalloc(sizeof(struct adv_vpm_management_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit_i2c;
	}
	if(!(d_rec=kzalloc(32, GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit_free;
	}

	i2c_set_clientdata(client, &data);

#if VPM_KEYPAD_EVENT_SUPPORTED
	if(devnode)
	{
		irq_gpio = of_get_named_gpio(devnode, "int-gpios", 0);
		printk(KERN_INFO "vpm int gpio = %d", irq_gpio);
	}

	char tmpbuf[64] = {0};
	char tmpbuf_get[64] = {0};
	char tmpbuf_set[10] = {0};
	int shift = 0, test = 0;

	static const char zero[64] = {0};
	static const char FF[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	is_empty = ((!memcmp(zero, tmpbuf, 10))|| (!memcmp(FF, tmpbuf, 10)));

	if(is_empty)
	{
		printk(KERN_INFO "Default VPM hotkey_keycode\n");
		for( j=0; j<5; j++)
		{
			tmpbuf[j*2] =(hotkey_matrix[j] >> 8);
			tmpbuf[j*2+1] = (hotkey_matrix[j] << 8) >> 8;
		}
	}
	else
	{
		//printk(KERN_INFO "From SPI ROM: VPM hotkey_keycode = ");
		for(j=0; j<5; j++)
		{
			shift = tmpbuf[j*2]  ;
			hotkey_matrix[j] = (shift << 8) + tmpbuf[j*2+1] ;
		}
	}

	tbtnDev = input_allocate_device();
	if(!tbtnDev)
	{
		printk("vpm error: input_allocate_device.\n");
		err = -ENOMEM;
		goto exit_free;
	}

	tbtnDev->name = "adv_vpm_key";
	tbtnDev->id.bustype = BUS_I2C;
	tbtnDev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) | BIT_MASK(EV_REP);
	tbtnDev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	//tbtnDev -> evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_SYN);
	set_bit(EV_KEY, tbtnDev->evbit);
	set_bit(EV_SYN, tbtnDev->evbit);
	set_bit(EV_REP, tbtnDev->evbit);

	for(i = 0; i < ARRAY_SIZE(hotkey_matrix); i++)
	{
		__set_bit(hotkey_matrix[i], tbtnDev->keybit);
		set_bit(hotkey_matrix[i], tbtnDev->keybit);
		input_set_capability(tbtnDev, EV_KEY, hotkey_matrix[i]);
	}

	err = input_register_device(tbtnDev);

	tbtnDev->rep[REP_DELAY] = 10;   /* input layer default: 250 */
	tbtnDev->rep[REP_PERIOD] = 10; /* input layer default: 33 */

	if(err)
	{
		err("failed to register device.\n");
		printk("vpm failed to register device.\n");
		input_free_device(tbtnDev);
		goto exit_free;
	}
#endif
	/* set irq type to edge falling */
	irq_set_irq_type(client->irq, (IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND));
	err = request_irq(client->irq, adv_vpm_irq_handler, 0, client->dev.driver->name, data);
	if (err < 0)
	{
		dev_err(&client->dev, "failed to register irq %d!\n",client->irq);
		goto exit_free_2;
	}

	data->workqueue = create_singlethread_workqueue("adv_vpm_comm_w");
	INIT_WORK(&data->work, adv_vpm_event_work);
	if (data->workqueue == NULL)
	{
		dev_err(&client->dev, "couldn't create workqueue\n");
		err = -ENOMEM;
		goto exit_free_interrupt;
	}

	client_vpm=client;
	dbg("...\n");

#if VPM_DOCKING_SUPPORTED
	// docking callback
	if(pdata->docking_pin != 0)
	{
		irq_set_irq_type(pdata->docking_pin, (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND));
		err = request_irq(pdata->docking_pin, adv_vpm_docking_handler, 0, client->dev.driver->name, data);
		if (err < 0)
		{
			dev_err(&client->dev, "failed to register irq %d!\n", pdata->docking_pin);
			goto exit_free_interrupt;
		}

		data->docking_workqueue = create_singlethread_workqueue("adv_vpm_comm_docking_workqueue");
		INIT_WORK(&data->docking_work, adv_vpm_docking_work);
		if (data->docking_workqueue == NULL)
		{
			dev_err(&client->dev, "couldn't create workqueue\n");
			err = -ENOMEM;
			goto exit_free_docking_interrupt;
		}

		data->platform_data.docking_pin = pdata->docking_pin;
		data->platform_data.docking_callback = pdata->docking_callback;
	}
#endif

	//VPM event handler example:
	//printk("VPM : register VPM_NO_EVENT handler func...\n");
	RegisterVPMEventFunc(VPM_NO_EVENT, vpm_no_event_handler_func);

	//RegisterVPMEventFunc(VPM_READ_INDEX, vpm_read_index_handler_func);

#if VPM_POWER_SOURCE_EVENT_SUPPORTED
	RegisterVPMEventFunc(VPM_POWER_SOURCE_CHANGED_EVENT, vpm_powersource_event_handler_func);
#endif

#if VPM_CAR_POWER_EVENT_SUPPORTED
	RegisterVPMEventFunc(VPM_WAKE_UP_EVENT, vpm_wakeup_event_handler_func);
#endif

#if VPM_BATTERY_EVENT_SUPPORTED
	RegisterVPMEventFunc(VPM_BATTERY_EVENT, vpm_battery_event_handler_func);
#endif

#if VPM_GPI_DI_EVENT_SUPPORTED
	RegisterVPMEventFunc(VPM_GPI_EVENT, vpm_gpi_event_handler_func);
#endif

#if VPM_CRADLE_EVENT_SUPPORTED
	RegisterVPMEventFunc(VPM_CRADLE_EVENT, vpm_cradle_event_handler_func);
#endif

#if VPM_KEYPAD_EVENT_SUPPORTED
	RegisterVPMEventFunc(VPM_KEYPAD_EVENT, vpm_keypad_event_handler_func);
#endif

#if VPM_POWER_OFF_EVENT_SUPPORTED
	//printk("VPM : register VPM_KEYPAD_EVENT handler func...\n");
	RegisterVPMEventFunc(VPM_POWER_OFF_EVENT, vpm_power_off_event_handler_func);
#endif

	RegisterVPMEventFunc(VPM_ByPass_EVENT, vpm_bypass_event_handler_func);

	vpm_get_version();

	//vpm_set_interrupt_status(VPM_I2C_INTERRUPT_ENABLE);

	err = sysfs_create_group(&client->dev.kobj, &adv_vpm_attr_group);
	if (err)
	{
		goto exit_free_interrupt;
	}

	dev_t dev = MKDEV(ioctl_major, 0);
	alloc_ret = alloc_chrdev_region(&dev, 0, num_of_dev, DRIVER_NAME);
	if (alloc_ret)
		goto exit_ioctl;

	ioctl_major = MAJOR(dev);

	cdev_init(&ioctl_cdev, &fops);
	cdev_ret = cdev_add(&ioctl_cdev, dev, num_of_dev);
	if (cdev_ret)
		goto exit_ioctl;

	vpm_class = class_create(THIS_MODULE, classname);

	if(IS_ERR(vpm_class))
	{
		printk("Failed at class_create().Please exec [mknod] before operate the device/n");
	}
	else
	{
		device_create(vpm_class, NULL, dev, NULL, devicename);
	}

	printk(KERN_ALERT "%s driver(major: %d) installed.\n", DRIVER_NAME, ioctl_major);

	return 0;

#if VPM_DOCKING_SUPPORTED
exit_free_docking_interrupt:
	free_irq(pdata->docking_pin, data);
#endif

exit_free_interrupt:
	free_irq(client->irq, data);

exit_free_2:
#if VPM_KEYPAD_EVENT_SUPPORTED
	input_unregister_device(tbtnDev);
#endif
	kfree(d_rec);

exit_free:
	kfree(data);

exit_i2c:
	i2c_set_clientdata(client, NULL);

exit_ioctl:
	if (cdev_ret == 0)
		cdev_del(&ioctl_cdev);
	if (alloc_ret == 0)
		unregister_chrdev_region(dev, num_of_dev);

	return err;
}

static int adv_vpm_remove(struct i2c_client *client)
{
	struct adv_vpm_management_data *data = i2c_get_clientdata(client);

#if VPM_DOCKING_SUPPORTED
	struct adv_vpm_platform_data *pdata = &(data->platform_data);
#endif

	client_vpm=NULL;
	cancel_work_sync(&data->work);
	destroy_workqueue(data->workqueue);

#if VPM_KEYPAD_EVENT_SUPPORTED
	input_unregister_device(tbtnDev);
#endif

#if VPM_DOCKING_SUPPORTED
	free_irq(pdata->docking_pin, data);
#endif

	free_irq(client->irq, data);
	kfree(data);
	kfree(d_rec);
	i2c_set_clientdata(client, NULL);
	dev_t dev = MKDEV(ioctl_major, 0);
	cdev_del(&ioctl_cdev);
	device_destroy(vpm_class, dev);
	class_destroy(vpm_class);
	unregister_chrdev_region(dev, num_of_dev);

	return 0;
}

//ask vpm to execute suspend from android ui
int adv_vpm_cmd_exe_suspend(void)
{
	struct adv_vpm_data tp={0};
	tp.wlen = 2;
	tp.rlen = 0;
	tp.data[0] = 0x0a;
	tp.data[1] = 0x02;

	printk("adv_vpm_cmd_exe_suspend: cmd %d, %d \n",tp.data[0],tp.data[1]);
	adv_vpm_tf(&tp);
	return 0;;
}
//EXPORT_SYMBOL(adv_vpm_cmd_suspend);

//ask vpm to change mode to off mode
int adv_vpm_cmd_off_mode(void)
{
	struct adv_vpm_data tp={0};
	tp.wlen = 2;
	tp.rlen = 0;
	tp.data[0] = 0x0a;
	tp.data[1] = 0x01;

	printk("adv_vpm_cmd_off_mode: cmd %d, %d \n",tp.data[0],tp.data[1]);
	adv_vpm_tf(&tp);
	return 0;;
}

// OS resume/on successful
int adv_vpm_cmd_os_resumeon_ping(void)
{
	struct adv_vpm_data tp={0};
	tp.wlen = 2;
	tp.rlen = 0;
	tp.data[0] = 0x0d;
	tp.data[1] = 0x01;

	printk("adv_vpm_cmd_os_resumeon_ping: cmd %d, %d \n",tp.data[0],tp.data[1]);
	adv_vpm_tf(&tp);
	return 0;;
}

static int adv_vpm_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("------------------->vpm_suspend\n");
	adv_vpm_cmd_exe_suspend();
	disable_irq(client->irq);//august
	return 0;
}

static int adv_vpm_resume(struct i2c_client *client)
{
	printk("------------------->vpm_resume\n");
	enable_irq(client->irq);//august
	adv_vpm_cmd_os_resumeon_ping();

	return 0;
}
static struct of_device_id vpm_match_table[] = {
	{ .compatible = "adv_vpm",},
	{ },
};
static struct i2c_driver adv_vpm_driver = {
	.driver = {
		.name = "adv_vpm",
		.of_match_table = vpm_match_table,
		.owner = THIS_MODULE,
	},
//#if defined(VPM_PLATFROM_DMSST05)						//adv_vpm_comm_0626
	.probe		= adv_vpm_probe,
	.remove		= adv_vpm_remove,
	.id_table	= adv_vpm_id,
	//.suspend	= adv_vpm_suspend,
	//.resume		= adv_vpm_resume,
//#endif	//VPM_PLATFROM_DMSST05
};

static int battery_vpm_notify_shutdown(struct notifier_block *this,
       unsigned long code, void *x)
{
	//printk("battery_vpm_notify_shutdown: %d\n", code);
	if ((code == SYS_POWER_OFF))
	{
		adv_vpm_write_power_off();
		vpm_set_amp_mute_status(0x01);
		printk("Enable VPM amp mute\n");
	}
	else if((code == SYS_RESTART))
	{
		vpm_set_amp_mute_status(0x01);
		printk("Enable VPM amp mute\n");
	}

	return NOTIFY_DONE;
}

static struct notifier_block battery_vpm_notifier = {
	.notifier_call  = battery_vpm_notify_shutdown,
	.next       = NULL,
	.priority   = INT_MAX, /* should be > ssd pri's and disk dev pri's */
};

static int __init adv_vpm_init(void)
{
	int err;

	if (err = register_reboot_notifier(&battery_vpm_notifier))
		goto exit_free_vpm_battery_nofifier;

	return i2c_add_driver(&adv_vpm_driver);

exit_free_vpm_battery_nofifier:
	unregister_reboot_notifier(&battery_vpm_notifier);
	return err;
}

static void __exit adv_vpm_exit(void)
{
	unregister_reboot_notifier(&battery_vpm_notifier);
	i2c_del_driver(&adv_vpm_driver);
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ADV <-> VPM DRIVER");

module_init(adv_vpm_init);
//device_initcall_sync(adv_vpm_init);
//late_initcall(adv_vpm_init);
module_exit(adv_vpm_exit);
