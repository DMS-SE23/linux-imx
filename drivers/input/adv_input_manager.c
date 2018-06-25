/*
 *  linux/driver/input/adv_input_manager.c
 *  Copyright (C) 2017 Advantech
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/adv_autobl.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/fs.h>

#define DEFAULT_LUX200_CAL  200
#define DEFAULT_CONTROL_BL    1
#define DEBUG_FU              0
#define DEBUG_VA              0

static int adv_bl_levels[LEVELS_ARRAY_SIZE][2];
static int adv_levels_size = DEFAULT_LEVELS_SIZE;
static int adv_lux200 = DEFAULT_LUX200_CAL;
static int control_bl = DEFAULT_CONTROL_BL;
static struct proc_dir_entry *proc_input_manager_dir;
struct delayed_work	read_config_work;

static void adv_input_manager_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	if(DEBUG_FU) printk("%s\n",__FUNCTION__);
	if(DEBUG_VA) printk( "Event. Dev: %s, Type: %d, Code: %d, Value: %d\n" ,
	       dev_name(&handle->dev->dev), type, code, value);
	switch (type) {
	case EV_KEY:
		if(DEBUG_VA) printk( "EV_KEY\n");
		break;
	case EV_REL:
		if(DEBUG_VA) printk( "EV_REL\n");
		break;
	case EV_ABS:
		if(DEBUG_VA) printk( "EV_ABS\n");
		if(code==ABS_MISC) {
			if(DEBUG_VA) printk( "ABS_MISC\n");
			if(control_bl == 1) {
				adv_set_brightness(adv_bl_levels,(unsigned int)(value*200/adv_lux200),BACKLIGHT_PATH,&adv_levels_size);
			}
		}
		break;
	case EV_MSC:
		if(DEBUG_VA) printk( "EV_MSC\n");
		break;
	case EV_LED:
		if(DEBUG_VA) printk( "EV_LED\n");
		break;
	case EV_SND:
		if(DEBUG_VA) printk( "EV_SND\n");
		break;
	case EV_FF:
		if(DEBUG_VA) printk( "EV_FF\n");
		break;
	case EV_SW:
		if(DEBUG_VA) printk( "EV_SW\n");
		break;
	default:
		if(DEBUG_VA) printk( "default\n");
		break;
	}
}

static int adv_input_manager_connect(struct input_handler *handler, struct input_dev *dev,
			 const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;
	if(DEBUG_FU) printk("%s\n",__FUNCTION__);
	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "adv_input_manager";

	error = input_register_handle(handle);
	if (error)
		goto err_free_handle;

	error = input_open_device(handle);
	if (error)
		goto err_unregister_handle;

	if(DEBUG_VA) printk( "Connected device: %s (%s at %s)\n" ,
	       dev_name(&dev->dev),
	       dev->name ?: "unknown",
	       dev->phys ?: "unknown");

	return 0;

 err_unregister_handle:
	input_unregister_handle(handle);
 err_free_handle:
	kfree(handle);
	return error;
}

static void adv_input_manager_disconnect(struct input_handle *handle)
{
	if(DEBUG_FU) printk("%s\n",__FUNCTION__);
	if(DEBUG_VA) printk( "Disconnected device: %s\n", dev_name(&handle->dev->dev));
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id adv_input_manager_ids[] = {
	//{ .driver_info = 1 },	/* Matches all devices */
	{
		.flags = INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
	},
	{ },
};

MODULE_DEVICE_TABLE(input, input_manager_ids);

static struct input_handler input_manager_handler = {
	.event =	adv_input_manager_event,
	.connect =	adv_input_manager_connect,
	.disconnect =	adv_input_manager_disconnect,
	.name =		"adv_input_manager",
	.id_table =	adv_input_manager_ids,
};

static int userbuf_to_value(const char __user *userbuf,size_t count) {
	int  val;
	char buf[32];

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;
	sscanf(buf, "%d", &val);
	return val;
}

static ssize_t light_control_bl_read(struct file *file, char __user *buf,
			       size_t count, loff_t *ppos)
{
	char str[32];
	sprintf(str,"%d\n", control_bl);
	return simple_read_from_buffer(buf, count, ppos, str, strlen(str));
}


static ssize_t light_control_bl_write(struct file *file, const char __user *userbuf,
			   size_t count, loff_t *ppos)
{
	int  val = userbuf_to_value(userbuf,count);
	if(val==0 || val==1)
		control_bl = val;
	return count;
}

static const struct file_operations light_control_bl_fileops = {
	.owner		= THIS_MODULE,
	.write		= light_control_bl_write,
	.read		= light_control_bl_read,
};

static ssize_t light_lux200_read(struct file *file, char __user *buf,
			       size_t count, loff_t *ppos) 
{
	char str[32];
	sprintf(str,"%d\n", adv_lux200);
	return simple_read_from_buffer(buf, count, ppos, str, strlen(str));
}

static ssize_t light_lux200_write(struct file *file, const char __user *userbuf,
			   size_t count, loff_t *ppos)
{
	int  val = userbuf_to_value(userbuf,count);
	if(val > 0)
		adv_lux200 = val;
	return count;
}

static const struct file_operations light_lux200_fileops = {
	.owner		= THIS_MODULE,
	.write		= light_lux200_write,
	.read		= light_lux200_read,
};

static ssize_t light_levels_read(struct file *file, char __user *buf,
			       size_t count, loff_t *ppos)
{
	int index = 0;
	char str[50];
	char all_str[500]="";
	ssize_t len;
	for(index = 0;index < adv_levels_size;index++) {
		sprintf(str, "[%d,%d]", adv_bl_levels[index][0],adv_bl_levels[index][1]);
		strcat(all_str, str);
	}
	strcat(all_str, "\n");
	len = strlen(all_str);
	return simple_read_from_buffer(buf, count, ppos, all_str, len);
}

static ssize_t light_levels_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	int ret=0;
	char str[500];
	count = min_t(size_t, count, (sizeof(str)-1));
	if (copy_from_user(str, buf, count))
		return -EFAULT;

	str[count] = 0;
	ret = adv_parser_levels(adv_bl_levels, str, &adv_levels_size);
	if(ret == 0){
		printk( "levels table update\n");
	}else {
		printk( "wrong levels table\n");
	}
	return count;
}

static const struct file_operations light_levels_fileops = {
	.owner		= THIS_MODULE,
	.write		= light_levels_write,
	.read		= light_levels_read,
};

static void init_levels(void) {
	int index = 0;
	int luxs = 50;
	int bl_level_base = 30;
	for(index = 0;index < DEFAULT_LEVELS_SIZE;index++) {
		adv_bl_levels[index][0] = luxs;
		adv_bl_levels[index][1] = bl_level_base;
		luxs = luxs*2;
		bl_level_base = bl_level_base + 25;
	}
}

static void light_setting_config(struct work_struct *work)
{
	int ret = 0, fd = 0;
	if(DEBUG_FU) printk("%s\n",__FUNCTION__);

	adv_lux200 = 200;
	control_bl = 0;
	adv_bl_levels[0][0]=100;
	adv_bl_levels[0][1]=50;
	adv_bl_levels[1][0]=200;
	adv_bl_levels[1][1]=100;
	adv_bl_levels[2][0]=400;
	adv_bl_levels[2][1]=200;
	adv_bl_levels[3][0]=800;
	adv_bl_levels[3][1]=250;
	adv_levels_size=4;

	//symlink for driver/iio/light/tsl2563
	set_fs(KERNEL_DS);
	fd = sys_open("/sys/devices/soc0/soc/2100000.aips-bus/21a8000.i2c/i2c-2/2-0029/iio:device0/events", O_RDONLY, 0);
	if(DEBUG_VA) printk("fd:%d", fd);
	if(fd >= 0)
	{
		sys_close(fd);

		proc_symlink("light_en",
				proc_input_manager_dir,
				 "/sys/devices/soc0/soc/2100000.aips-bus/21a8000.i2c/i2c-2/2-0029/iio:device0/events/in_intensity_both_thresh_rising_en");
		proc_symlink("threshold_range" ,
				proc_input_manager_dir,
				 "/sys/devices/soc0/soc/2100000.aips-bus/21a8000.i2c/i2c-2/2-0029/iio:device0/threshold_range");
		proc_symlink("lux" ,
				proc_input_manager_dir,
				 "/sys/devices/soc0/soc/2100000.aips-bus/21a8000.i2c/i2c-2/2-0029/iio:device0/lux");
	}

	//symlink for driver/video/backlight/pwm_bl
	fd = sys_open("/sys/class/backlight/backlight/brightness", O_RDONLY, 0);
	if(DEBUG_VA) printk("fd:%d", fd);
	if(fd >= 0)
	{
		proc_symlink("backlight" ,
	             proc_input_manager_dir,
				 "/sys/class/backlight/backlight/brightness");
	}
}

static int __init adv_input_manager_init(void)
{
	struct proc_dir_entry *entry;
	if(DEBUG_FU) printk("%s\n",__FUNCTION__);
	init_levels();

	proc_input_manager_dir = proc_mkdir("adv_input_manager", NULL);
	if (!proc_input_manager_dir)
		return -ENOMEM;

	entry = proc_create("levels", 0, proc_input_manager_dir,
			    &light_levels_fileops);
	if (!entry)
		goto fail1;

	entry = proc_create("lux200", 0, proc_input_manager_dir,
			    &light_lux200_fileops);
	if (!entry)
		goto fail2;

	entry = proc_create("control_bl", 0, proc_input_manager_dir,
				&light_control_bl_fileops);
	if (!entry)
		goto fail3;

	INIT_DELAYED_WORK(&read_config_work, light_setting_config);
	schedule_delayed_work(&read_config_work, 6 * HZ);//waitting fs

	return input_register_handler(&input_manager_handler);

 fail3:
	remove_proc_entry("threshold_range", proc_input_manager_dir);
 fail2:
	remove_proc_entry("levels", proc_input_manager_dir);
 fail1:
	remove_proc_entry("adv_input_manager", NULL);

	return -1;
}

static void __exit adv_input_manager_exit(void)
{
	if(DEBUG_FU) printk("%s\n",__FUNCTION__);
	remove_proc_entry("control_bl", proc_input_manager_dir);
	remove_proc_entry("lux200", proc_input_manager_dir);
	remove_proc_entry("levels", proc_input_manager_dir);
	remove_proc_entry("adv_input_manager", NULL);
	input_unregister_handler(&input_manager_handler);
}

late_initcall_sync(adv_input_manager_init);
module_exit(adv_input_manager_exit);

MODULE_AUTHOR("Daniel Chan <daniel.chan@advantech.com>");
MODULE_DESCRIPTION("Input driver event manager module");
MODULE_LICENSE("GPL");