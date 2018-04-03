#ifndef __ADV_VPM_COMM_H__
#define __ADV_VPM_COMM_H__

//adv_vpm_comm_0630+
//*******************************************************************************
//event class supported
//define 1 => support this event, define 0 => not support.
#define VPM_POWER_SOURCE_EVENT_SUPPORTED										1
#define VPM_CAR_POWER_EVENT_SUPPORTED											1
#define VPM_BATTERY_EVENT_SUPPORTED												1
#define VPM_GPI_DI_EVENT_SUPPORTED												1
#define VPM_CRADLE_EVENT_SUPPORTED												0
#define VPM_KEYPAD_EVENT_SUPPORTED												1
#define VPM_POWER_OFF_EVENT_SUPPORTED                                                                    1
//-------------------------------------------------------------------------------
#define VPM_DOCKING_SUPPORTED													0
#define VPM_WWAN_SUPPORTED														0
#define VPM_WIFI_MAC_ADDR_SUPPORTED												0
#define VPM_GPSMODE_SUPPORTED													0


#define VPM_PLATFROM_DMSST05													1		
//*******************************************************************************

static DEFINE_MUTEX(vpm_mutex);
static DEFINE_MUTEX(vpm_pack_mutex);

/*
 * VPM command address
 */
#define VPM_INT_EVENT															0x00
#define VPM_INT_INDEX															0x01
#define VPM_SYS_INFO_H															0x02
#define VPM_SYS_INFO_L															0x03
#define VPM_WAKEUP_EVENT_SRC_H													0x04
#define VPM_WAKEUP_EVENT_SRC_L													0x05
#define VPM_CONTROL_STATUS														0x08	//adv_vpm_comm_0626
#define VPM_SYSTEM_POWER_STATUS													0x09
#define VPM_STATUS_INTERRUPT                 								                   0x0A  	//Enable/Disable the I2C INTERRUPT
#define VPM_FORCE_VPM_WORKING_MODE_CHANGE										0x0B
#define VPM_LAST_WAKEUP_EVENT_SRC												0X0C	//adv_vpm_comm_0626
#define VPM_GET_CURRENT_STATE_H													0x0E	//adv_vpm_comm_0626
#define VPM_GET_CURRENT_STATE_L													0x0F	//adv_vpm_comm_0626

#define VPM_RTC_YEAR_ADDR														0x10
#define VPM_RTC_MONTH_ADDR														0x11
#define VPM_RTC_DAY_ADDR														0x12
#define VPM_RTC_DAY_OF_WEEK_ADDR												0x13
#define VPM_RTC_HOUR_ADDR														0x14
#define VPM_RTC_MINUTE_ADDR														0x15
#define VPM_RTC_SECOND_ADDR														0x16
#define VPM_ALARM_DAY_OF_WEEK													0x17	//adv_vpm_comm_0626
#define VPM_ALARM_HOUR															0x18	//adv_vpm_comm_0626
#define VPM_ALARM_MINUTE														0x19	//adv_vpm_comm_0626

#define VPM_GET_INTERRUPT_EVENT 												0x20
//#define VPM_WATCHDOG_COUNT_DOWN_TIMER_H											0x20	//adv_vpm_comm_0626
//#define VPM_WATCHDOG_COUNT_DOWN_TIMER_L											0x21	//adv_vpm_comm_0626
#define VPM_WATCHDOG_CURRENT_COUNTER_H											0x22	//adv_vpm_comm_0626
#define VPM_WATCHDOG_CURRENT_COUNTER_L											0x23	//adv_vpm_comm_0626
#define VPM_WATCHDOG_CURRENT_COUNTER											0x24	//adv_vpm_comm_0626

#define VPM_EEPROM_DEFAULT														0x30	//adv_vpm_comm_0626
#define VPM_GET_INTERRUPT_STATUS												0x37
#define VPM_SET_INTERRUPT_STATUS												0x38
#define VPM_GET_BACKLIGHT_CURRENT_SENSE											0x40
#define VPM_KEY_PAD_BACK_LIGHT_CONTROL_NO										0x41
#define VPM_KEY_PAD_BACK_LIGHT_BRIGHTNESS										0x42
#define VPM_LCD_BACKLIGHT_CTRL													0x43
#define VPM_LIGHT_LUX_H															0x44
#define VPM_LIGHT_LUX_L															0x45
#define VPM_REAR_VIEW_STATUS													0x46	//adv_vpm_comm_0626

//#define VPM_MODULE_CONTROL_H													0x50	//adv_vpm_comm_0626
//#define VPM_MODULE_CONTROL_L													0x51	//adv_vpm_comm_0626
#define VPM_DI_STATUS															0x52
#define VPM_CAR_POWER_LOW_EVENT_DELAY_H											0x54	//adv_vpm_comm_0626
#define VPM_CAR_POWER_LOW_EVENT_DELAY_L											0x55	//adv_vpm_comm_0626
#define VPM_CAR_POWER_LOW_HARD_DELAY_H											0x56
#define VPM_CAR_POWER_LOW_HARD_DELAY_L											0x57	//adv_vpm_comm_0626
#define VPM_IGNITION_ON_DELAY_H													0x58	//adv_vpm_comm_0626
#define VPM_IGNITION_ON_DELAY_L													0x59	//adv_vpm_comm_0626
#define VPM_IGNITION_OFF_EVENT_DELAY_H											0x5A	//adv_vpm_comm_0626
#define VPM_IGNITION_OFF_EVENT_DELAY_L											0x5B	//adv_vpm_comm_0626
#define VPM_IGNITION_OFF_HARD_DELAY_H											0x5C	//adv_vpm_comm_0626
#define VPM_IGNITION_OFF_HARD_DELAY_L											0x5D	//adv_vpm_comm_0626
#define VPM_POST_BOOT_POWER_CHECK_DELAY_H										0x5E	//adv_vpm_comm_0626
#define VPM_POST_BOOT_POWER_CHECK_DELAY_L										0x5F	//adv_vpm_comm_0626

#define VPM_CAR_POWER_VOLTAGE_H													0x60	//adv_vpm_comm_0626
#define VPM_CAR_POWER_VOLTAGE_L													0x61	//adv_vpm_comm_0626
#define VPM_12V_SYSTEM_RANGE_MINIUM_H											0x62	//adv_vpm_comm_0626
#define VPM_12V_SYSTEM_RANGE_MINIUM_L											0x63    //adv_vpm_comm_0626
#define VPM_12V_SYSTEM_RANGE_DEFAULT_H											0x64    //adv_vpm_comm_0626
#define VPM_12V_SYSTEM_RANGE_DEFAULT_L											0x65    //adv_vpm_comm_0626
#define VPM_12V_SYSTEM_RANGE_MAXIMUM_H											0x66    //adv_vpm_comm_0626
#define VPM_12V_SYSTEM_RANGE_MAXIMUM_L											0x67    //adv_vpm_comm_0626

#define VPM_24V_SYSTEM_RANGE_MINIUM_H											0x68	//adv_vpm_comm_0626
#define VPM_24V_SYSTEM_RANGE_MINIUM_L                   						0x69    //adv_vpm_comm_0626
#define VPM_24V_SYSTEM_RANGE_DEFAULT_H											0x6A    //adv_vpm_comm_0626
#define VPM_24V_SYSTEM_RANGE_DEFAULT_L											0x6B    //adv_vpm_comm_0626
#define VPM_24V_SYSTEM_RANGE_MAXIMUM_H											0x6C    //adv_vpm_comm_0626
#define VPM_24V_SYSTEM_RANGE_MAXIMUM_L											0x6D    //adv_vpm_comm_0626

#define VPM_12V_SYSTEM_PRE_BOOT_VOLTAGE_VALUE_H									0x6E	//adv_vpm_comm_0626
#define VPM_12V_SYSTEM_PRE_BOOT_VOLTAGE_VALUE_L									0x6F    //adv_vpm_comm_0626
#define VPM_12V_SYSTEM_POST_BOOT_VOLTAGE_VALUE_H								0x70    //adv_vpm_comm_0626
#define VPM_12V_SYSTEM_POST_BOOT_VOLTAGE_VALUE_L								0x71    //adv_vpm_comm_0626

#define VPM_24V_SYSTEM_PRE_BOOT_VOLTAGE_VALUE_H									0x72	//adv_vpm_comm_0626
#define VPM_24V_SYSTEM_PRE_BOOT_VOLTAGE_VALUE_L         						0x73    //adv_vpm_comm_0626
#define VPM_24V_SYSTEM_POST_BOOT_VOLTAGE_VALUE_H								0x74    //adv_vpm_comm_0626
#define VPM_24V_SYSTEM_POST_BOOT_VOLTAGE_VALUE_L								0x75    //adv_vpm_comm_0626
#define VPM_CAR_POWER_VOLTAGE_LOW_PROTECTION_LOAD_D4							0x76    //adv_vpm_comm_0626

#define VPM_BATTERY_PACK_FLAGS													0x92 
#define VPM_BATTERY_PACK_STATE_OF_CHARGE										0x9F 
#define VPM_BATTERY_PACK_TIME_TO_EMPTY_H										0x82
#define VPM_BATTERY_PACK_TIME_TO_EMPTY_L										0x83
#define VPM_BATTERY_PACK_TIME_TO_FULL_H											0x84
#define VPM_BATTERY_PACK_TIME_TO_FULL_L											0x85
#define VPM_BATTERY_PACK_FLAGS_H												0x86
#define VPM_BATTERY_PACK_FLAGS_L												0x87
#define VPM_BATTERY_PACK_TEMPERATURE_H											0x88
#define VPM_BATTERY_PACK_TEMPERATURE_L											0x89
#define VPM_BATTERY_PACK_VOLTAGE_H												0x8A
#define VPM_BATTERY_PACK_VOLTAGE_L												0x8B
#define VPM_BATTERY_PACK_CURRENT_H												0x8C
#define VPM_BATTERY_PACK_CURRENT_L												0x8D
#define VPM_BATTERY_PACK_AVERAGE_CURRENT_H										0x8E
#define VPM_BATTERY_PACK_AVERAGE_CURRENT_L										0x8F

#define VPM_BATTERY_PACK_ATRATE_H												0x90
#define VPM_BATTERY_PACK_ATRATE_L												0x91
#define VPM_BATTERY_PACK_ATRATE_TIME_TO_FULL_H									0x92
#define VPM_BATTERY_PACK_ATRATE_TIME_TO_FULL_L									0x93
#define VPM_BATTERY_PACK_ATRATE_TIME_TO_EMPTY_H									0x94
#define VPM_BATTERY_PACK_ATRATE_TIME_TO_EMTPY_L									0x95

//#define VPM_WLAN_MAC_ADDR_BYTE_1												0xA0	//adv_vpm_comm_0626
//#define VPM_WLAN_MAC_ADDR_BYTE_2												0xA1    //adv_vpm_comm_0626
//#define VPM_WLAN_MAC_ADDR_BYTE_3												0xA2    //adv_vpm_comm_0626
//#define VPM_WLAN_MAC_ADDR_BYTE_4												0xA3    //adv_vpm_comm_0626
//#define VPM_WLAN_MAC_ADDR_BYTE_5												0xA4    //adv_vpm_comm_0626
//#define VPM_WLAN_MAC_ADDR_BYTE_6												0xA5    //adv_vpm_comm_0626

#define VPM_GET_IGNITION_ON_GENERATE_POWER_ON_EVENT_COUNTB0						0xB0	//adv_vpm_comm_0626
#define VPM_GET_IGNITION_ON_GENERATE_POWER_ON_EVENT_COUNTB1						0xB1    //adv_vpm_comm_0626
#define VPM_GET_IGNITION_ON_GENERATE_POWER_ON_EVENT_COUNTB2						0xB2    //adv_vpm_comm_0626
#define VPM_GET_IGNITION_ON_GENERATE_POWER_ON_EVENT_COUNTB3						0xB3    //adv_vpm_comm_0626
#define VPM_GET_CAR_POWER_LOW_GENERATE_POWER_OFF_EVENT_COUNTB0					0xB4    //adv_vpm_comm_0626
#define VPM_GET_CAR_POWER_LOW_GENERATE_POWER_OFF_EVENT_COUNTB1					0xB5    //adv_vpm_comm_0626
#define VPM_GET_CAR_POWER_LOW_GENERATE_POWER_OFF_EVENT_COUNTB2					0xB6    //adv_vpm_comm_0626
#define VPM_GET_CAR_POWER_LOW_GENERATE_POWER_OFF_EVENT_COUNTB3					0xB7    //adv_vpm_comm_0626
#define VPM_GET_IGNITION_OFF_GENERATE_POWER_OFF_EVENT_COUNTB0					0xB8    //adv_vpm_comm_0626
#define VPM_GET_IGNITION_OFF_GENERATE_POWER_OFF_EVENT_COUNTB1					0xB9    //adv_vpm_comm_0626
#define VPM_GET_IGNITION_OFF_GENERATE_POWER_OFF_EVENT_COUNTB2					0xBA    //adv_vpm_comm_0626
#define VPM_GET_IGNITION_OFF_GENERATE_POWER_OFF_EVENT_COUNTB3					0xBB    //adv_vpm_comm_0626
#define VPM_GET_BATERY_OVER_TEMPERATURE_COUNTB0									0xBC    //adv_vpm_comm_0626
#define VPM_GET_BATERY_OVER_TEMPERATURE_COUNTB1									0xBD    //adv_vpm_comm_0626
#define VPM_GET_BATERY_OVER_TEMPERATURE_COUNTB2									0xBE    //adv_vpm_comm_0626
#define VPM_GET_BATERY_OVER_TEMPERATURE_COUNTB3									0xBF    //adv_vpm_comm_0626

#define VPM_GET_PREBOOT_CAR_POWER_CHECK_GENERATE_POWER_OFF_COUNTB0				0xC0	//adv_vpm_comm_0626
#define VPM_GET_PREBOOT_CAR_POWER_CHECK_GENERATE_POWER_OFF_COUNTB1				0xC1	//adv_vpm_comm_0626
#define VPM_GET_PREBOOT_CAR_POWER_CHECK_GENERATE_POWER_OFF_COUNTB2				0xC2	//adv_vpm_comm_0626
#define VPM_GET_PREBOOT_CAR_POWER_CHECK_GENERATE_POWER_OFF_COUNTB3				0xC3	//adv_vpm_comm_0626
#define VPM_GET_POSTBOOT_CAR_POWER_CHECK_GENERATE_POWER_OFF_COUNTB0				0xC4	//adv_vpm_comm_0626
#define VPM_GET_POSTBOOT_CAR_POWER_CHECK_GENERATE_POWER_OFF_COUNTB1				0xC5	//adv_vpm_comm_0626
#define VPM_GET_POSTBOOT_CAR_POWER_CHECK_GENERATE_POWER_OFF_COUNTB2				0xC6	//adv_vpm_comm_0626
#define VPM_GET_POSTBOOT_CAR_POWER_CHECK_GENERATE_POWER_OFF_COUNTB3				0xC7	//adv_vpm_comm_0626
#define VPM_GET_WATCHDOG_RESET_SYSTEM_COUNTB0									0xC8	//adv_vpm_comm_0626
#define VPM_GET_WATCHDOG_RESET_SYSTEM_COUNTB1									0xC9	//adv_vpm_comm_0626
#define VPM_GET_WATCHDOG_RESET_SYSTEM_COUNTB2									0xCA	//adv_vpm_comm_0626
#define VPM_GET_WATCHDOG_RESET_SYSTEM_COUNTB3									0xCB	//adv_vpm_comm_0626
#define VPM_GET_KEEP_ALIVEON_AND_IGNITIONON_GENERATE_POWER_ON_EVENT_COUNTB0		0xCC	//adv_vpm_comm_0626
#define VPM_GET_KEEP_ALIVEON_AND_IGNITIONON_GENERATE_POWER_ON_EVENT_COUNTB1		0xCD	//adv_vpm_comm_0626
#define VPM_GET_KEEP_ALIVEON_AND_IGNITIONON_GENERATE_POWER_ON_EVENT_COUNTB2		0xCE	//adv_vpm_comm_0626
#define VPM_GET_KEEP_ALIVEON_AND_IGNITIONON_GENERATE_POWER_ON_EVENT_COUNTB3		0xCF	//adv_vpm_comm_0626

#define VPM_FIRMWARE_PLATFORM_ID_H												0xF0
#define VPM_FIRMWARE_PLATFORM_ID_L												0xF1
#define VPM_FIRMWARE_VERSION_MAJOR												0xF2
#define VPM_FIRMWARE_VERSION_MINOR												0xF3
//#define VPM_PCB_VERSION														0xF4	//adv_vpm_comm_0626
//#define VPM_OS_KERNEL_VERSION													0xF5    //adv_vpm_comm_0626
#define VPM_GET_FIRMWARE_CHECKSUM_VERIFY_METHOD									0xF8	//adv_vpm_comm_0626


#if VPM_KEYPAD_EVENT_SUPPORTED	
#define HOTKEY_BUTTON_UP 								0x00
#define HOTKEY_HOME										0x10
#define HOTKEY_KEY_MENU									0x01
#define HOTKEY_BACK										0x04
#define HOTKEY_DOWN										0x08
#define HOTKEY_UP										0x02
//#define HOTKEY_SW1	0x1
//#define HOTKEY_SW2	0x2
//#define HOTKEY_SW3	0x4
//#define HOTKEY_SW4	0x8
//#define HOTKEY_SW5	0x10
#define HOTKEY_SW6										0x20
#define HOTKEY_SW7										0x40
#define HOTKEY_SW8										0x80
#endif	


#define POWER_IGNITION_OFF 								0x01
#define POWER_CAR_POWER_LOW								0x02
#define POWER_BATTERY_LOW									0x04
#define POWER_SHUTDOWN_FLAG_ENABLE						0x08

//VPM_BATTERY_PACK_STATE_OF_CHARGE (0x9F)
#define BATT_DISATTACHED 								0x00
#define BATT_CHARGING 									0x01
#define BATT_FULL_CHARGED 								0x02
#define BATT_DC_OUT 									0x03



#define VPM_I2C_INTERRUPT_DISABLE 0x00
#define VPM_I2C_INTERRUPT_ENABLE  0x01


struct adv_vpm_data {
	int wlen;
	int rlen;
	u16 scancode;
	u16 code;
	u16 status; //Read only, write not support status.
	char data[32];
};

/* 0422 begin*/
typedef void (* eventhandler) (void);
typedef enum
{
	VPM_NO_EVENT = 0,					//0
	VPM_POWER_SOURCE_CHANGED_EVENT,		//1
	VPM_WAKE_UP_EVENT,					//2
	VPM_BATTERY_EVENT,					//3
	VPM_GPI_EVENT,						//4
	VPM_CRADLE_EVENT,					//5
	VPM_KEYPAD_EVENT,					//6
	VPM_BOARD_EVENT,					//7
	VPM_POWER_OFF_EVENT,				//8
	VPM_ByPass_EVENT,				    //9
	VPM_EVENT_TOTAL						
} enum_vpm_int_event;

/**
 *  RegisterVPMEventFunc - register handler callback function for the specific event
 *
 *  1. Write the event handler for specific interrupt event, example: pic_no_event_handler_func()
 *  2. Register the handler function to the event, example:
 *  ==> RegisterVPMEventFunc(VPM_NO_EVENT, vpm_no_event_handler_func);
 *  The event hander will be called at ti_vpm_event_work work queue when VPM interrupt occurs.
 *  (*vpm_event_handler_array[VPM_NO_EVENT])();
 */
void RegisterVPMEventFunc(enum_vpm_int_event event_id, eventhandler func);

/*0422 end*/

#if VPM_DOCKING_SUPPORTED	//adv_vpm_comm_0630
struct adv_vpm_platform_data {
	int docking_pin;
	void (*docking_callback)(void);
};
#endif	//adv_vpm_comm_0630

extern int adv_vpm_tf(struct adv_vpm_data *);
extern int adv_vpm_cmd_suspend(void);
extern int adv_vpm_cmd_off_mode(void);
extern int adv_vpm_cmd_os_resumeon_ping(void);
extern int adv_vpm_cmd_readmode(void);

#if VPM_WWAN_SUPPORTED	//adv_vpm_comm_0630
extern int vpm_get_wwan_on_off(void);
extern int vpm_set_wwan_on_off(int on_off);
#endif	//adv_vpm_comm_0630

#if VPM_WIFI_MAC_ADDR_SUPPORTED	//adv_vpm_comm_0630
extern int vpm_get_wifi_mac_addr(unsigned char *pdata);	// 6 bytes of WiFi MAC address
#endif	//adv_vpm_comm_0630

#if VPM_KEYPAD_EVENT_SUPPORTED	//adv_vpm_comm_0630
extern int adv_vpm_readkey(void);
#endif	//adv_vpm_comm_0630

extern int adv_vpm_read_poweroff_source(void);

//common function
extern int vpm_get_version(void);
extern int vpm_is_bootloader_mode(void);
#endif
struct battery_vpm_pinfo {
	/*
	 * GPIOs
	 * cen, chg, flt, and usus are optional.
	 * dok, dcm, and uok are not optional depending on the status of
	 * dc_valid and usb_valid.
	 */
	int cen;	/* Charger Enable input */
	int dok;	/* DC(Adapter) Power OK output */
	int uok;	/* USB Power OK output */
	int chg;	/* Charger status output */
	int flt;	/* Fault output */
	int dcm;	/* Current-Limit Mode input (1: DC, 2: USB) */
	int usus;	/* USB Suspend Input (1: suspended) */
	int feature_flag;/* battery capacity feature(0:enable, 1:disable) */

	/*
	 * DCM wired to Logic High Set this true when DCM pin connect to
	 * Logic high.
	 */
	bool dcm_always_high;

	/*
	 * DC(Adapter/TA) is wired
	 * When dc_valid is true,
	 *	dok and dcm should be valid.
	 *
	 * At least one of dc_valid or usb_valid should be true.
	 */
	bool dc_valid;
	/*
	 * USB is wired
	 * When usb_valid is true,
	 *	uok should be valid.
	 */
	bool usb_valid;
};
