/*
 *  linux/lib/adv_autobl.c
 *  Copyright (C) 2017 Advantech
 */

#include <linux/types.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/bug.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/file.h>

#define LEVELS_ARRAY_SIZE 100

void adv_get_file_value(int *data,char *path)
{
	int fd,val,buff_size=100;
	u8  Buff[buff_size];
	mm_segment_t old_fs = get_fs();
	memset(Buff, 0x00, sizeof(Buff));
	set_fs(KERNEL_DS);
	fd = sys_open(path, O_RDONLY, 0);
	if(fd>=0)
	{
		sys_read(fd, Buff, buff_size);
		printk("%s: %s", path, Buff);
		sys_close(fd);
		set_fs(old_fs);
		if(sscanf(Buff, "%d", &val ) == 1) {
			if(val >=0) {
				*data = val;
			} else {
				printk("%s wrong value\n", path);
				return;
			}
		} else {
			printk("%s wrong value\n", path);
			return;
		}
	} else{
		printk("cannot find %s\n", path);
	}
	set_fs(old_fs);
}
EXPORT_SYMBOL(adv_get_file_value);


void adv_set_brightness(int levels[LEVELS_ARRAY_SIZE][2],int lux,char *path,int *levels_size)
{
	mm_segment_t old_fs;
	struct file *fp;
	char *buf = "1";
	int ret = 0,level = 255,index;
	loff_t pos = 0x00;
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	for(index=0;index<*levels_size;index++) {
		if(lux < levels[index][0]){
			level = levels[index][1];
			index = *levels_size;
		}
	}

	sprintf(buf, "%d", level);
	fp = filp_open(path, O_RDWR|O_CREAT, 0666);
	if (IS_ERR(fp)) {
		printk("cannot find %s\n",path);
		ret = -1;
		goto exit;
	}
	fp->f_op->write(fp, buf, sizeof(buf), &pos);

exit:

	if (fp)
		filp_close(fp, NULL);

	set_fs(old_fs);
}
EXPORT_SYMBOL(adv_set_brightness);

int adv_parse_levels(int bl_levels[LEVELS_ARRAY_SIZE][2],char *str ,int *levels_size)
{
	int temp_size = 0;
	int temp_lux = 0;
	int temp_bl = 0;
	int temp_value = 0;
	int temp_levels[LEVELS_ARRAY_SIZE][2];
    char delim[] = " [,]";
    char *token;
    for(token = strsep(&str, delim); token != NULL; token = strsep(&str, delim)) {
		temp_value = 0;
		sscanf(token, "%d",  &temp_value );
		if(temp_value != 0 && temp_lux == 0) {
			temp_lux = temp_value;
			temp_levels[temp_size][0] = temp_lux;
		} else if(temp_value != 0 && temp_bl == 0 ) {
			temp_bl = temp_value;
			temp_levels[temp_size][1] = temp_bl;
			temp_size++;
			temp_bl = 0;
			temp_lux = 0;
		}
    }
	if(temp_size <= 0 || temp_size > 100){
		return -1;
	}
	memcpy(bl_levels,temp_levels,LEVELS_ARRAY_SIZE*2*sizeof(int));
	*levels_size = temp_size;
	return 0;
}
EXPORT_SYMBOL(adv_parse_levels);

void adv_get_levels(int bl_levels[LEVELS_ARRAY_SIZE][2] ,char *path,int *levels_size)
{
	int ret = 0, buff_size = LEVELS_ARRAY_SIZE*2*8;
	char Buff[buff_size];
	int fd;
	mm_segment_t old_fs = get_fs();
	memset(Buff, 0x00, sizeof(Buff));
	set_fs(KERNEL_DS);
	fd = sys_open(path, O_RDONLY, 0);
	if(fd >= 0)
	{
		sys_read(fd, Buff, buff_size);
		sys_close(fd);
	} else{
		goto error;
	}
	set_fs(old_fs);
	ret = adv_parse_levels(bl_levels,Buff,levels_size);
	if(ret == 0){
		printk( "levels table update\n");
	}else {
		printk( "wrong levels table\n");
	}
	return;
error:
	printk(  "cannot find %s\n", path);
}
EXPORT_SYMBOL(adv_get_levels);
