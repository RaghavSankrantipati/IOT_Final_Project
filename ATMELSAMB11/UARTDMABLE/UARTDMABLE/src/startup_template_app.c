/**
 * \file
 *
 * \brief BLE Startup Template
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel
 * Support</a>
 */

/**
 * \mainpage
 * \section preface Preface
 * This is the reference manual for the Startup Template
 */
/*- Includes ---------------------------------------------------------------*/
#include <asf.h>
#include "platform.h"
#include "at_ble_api.h"
#include "console_serial.h"
#include "timer_hw.h"
#include "ble_manager.h"
#include "ble_utils.h"
#include "button.h"
#include "gpio.h"
#include "startup_template_app.h"
#include <dma_sam_b.h>
//#include <asf.h>
#include <conf_console_serial.h>
#include <string.h>
#include <stdarg.h>
//	#include "console_serial.h"
#include <samb11_xplained_pro.h>
//#include <pio_samb11g18a.h>
//#include "btlc1000wlcsp.h"
#include "battery.h"
#include "battery_info.h"
#include "hr_sensor_app.h"
#include "hr_sensor.h"

static void enable_uart(void);

volatile at_ble_status_t status;
at_ble_handle_t htpt_conn_handle;
volatile bool Timer_Flag = false;
volatile bool Temp_Notification_Flag = false;
static volatile bool transfer_is_done = false;
static volatile bool state=true;
#define BUFFER_LEN    6
static uint8_t string[BUFFER_LEN];


struct uart_module uart_instance;
struct dma_resource uart_dma_resource_rx;
struct dma_descriptor example_descriptor_rx;

//dma_start_transfer_job(&uart_dma_resource_rx)
static void htp_temperature_send(void);
/////////////////////////////////////////////////////////////

#define BATTERY_UPDATE_INTERVAL	(1) //1 second
#define BATTERY_MAX_LEVEL		(100)
#define BATTERY_MIN_LEVEL		(0)

uint8_t db_mem[1024] = {0};
bat_gatt_service_handler_t bas_service_handler;

bool volatile timer_cb_done = false;
bool volatile flag = true;

at_ble_handle_t bat_connection_handle;


uint16_t *pressure_ptr;
uint16_t pressure;

void resume_cb(void);

///////////////////////////////////////////////////////////////////////////

volatile bool app_state = 0 ; /*!< flag to represent the application state*/
volatile bool start_advertisement = 0; /*!< flag to start advertisement*/
volatile bool advertisement_flag = false;/*!< to check if the device is in advertisement*/
volatile bool notification_flag = false; /*!< flag to start notification*/
volatile bool disconnect_flag = false;	/*!< flag for disconnection*/
volatile bool hr_initializer_flag = 1; /*!< flag for initialization of hr for each category*/
volatile bool notification_sent = true;
uint8_t second_counter = 0;	/*!< second_counter to count the time*/
uint16_t energy_expended_val = ENERGY_EXP_NORMAL; /*!< to count the energy expended*/
uint16_t energy_incrementor ;	/*!< energy incrementor for various heart rate values*/
uint16_t heart_rate_value = HEART_RATE_MIN_NORM; /*!< to count the heart rate value*/
uint16_t rr_interval_value = RR_VALUE_MIN; /*!< to count the rr interval value*/
uint8_t activity = 0; /*!< activiy which will determine the */
uint8_t prev_activity = DEFAULT_ACTIVITY;/*!< previous activity */
int8_t inc_changer	= 1;/*!< increment operator to change heart rate */
int8_t time_operator ;/*!< operator to change the seconds */
uint8_t hr_min_value;/*!<the minimum heart rate value*/
uint8_t hr_max_value;/*!<the maximum heart rate value*/
uint8_t energy_inclusion = 0;/*!<To check for including the energy in hr measurement*/
uint8_t i = 0;
/////////////////////////////////////////////////////////////////////


static at_ble_status_t battery_service_advertise(void)
{
 status = AT_BLE_FAILURE;
	
	if((status = ble_advertisement_data_set()) != AT_BLE_SUCCESS)
	{
		DBG_LOG("advertisement data set failed reason :%d",status);
		return status;
	}
	
	/* Start of advertisement */
	if((status = at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED, AT_BLE_ADV_GEN_DISCOVERABLE, NULL, AT_BLE_ADV_FP_ANY, APP_BAS_FAST_ADV, APP_BAS_ADV_TIMEOUT, 0)) == AT_BLE_SUCCESS)
	{
		DBG_LOG("BLE Started Adv");
		return AT_BLE_SUCCESS;
	}
	else
	{
		DBG_LOG("BLE Adv start Failed reason :%d",status);
	}
	return status;
}

/* Callback registered for AT_BLE_PAIR_DONE event from stack */
static at_ble_status_t ble_paired_app_event(void *param)
{
	timer_cb_done = false;
	ALL_UNUSED(param);
	return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_DISCONNECTED event from stack */
static at_ble_status_t ble_disconnected_app_event(void *param)
{
	timer_cb_done = false;
	flag = true;
	
	
	battery_service_advertise();
	ALL_UNUSED(param);
	return AT_BLE_SUCCESS;
}

static at_ble_status_t ble_connected_app_event(void *param)
{
	at_ble_connected_t *connected = (at_ble_connected_t *)param;
	bat_connection_handle = connected->handle;
	#if !BLE_PAIR_ENABLE
	ble_paired_app_event(param);
	#else
	ALL_UNUSED(param);
	#endif
	return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_NOTIFICATION_CONFIRMED event from stack */
static at_ble_status_t ble_notification_confirmed_app_event(void *param)
{
	at_ble_cmd_complete_event_t *notification_status = (at_ble_cmd_complete_event_t *)param;
	if(!notification_status->status)
	{
		flag = true;
		DBG_LOG_DEV("sending notification to the peer success");
	}
	return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_CHARACTERISTIC_CHANGED event from stack */
static at_ble_status_t ble_char_changed_app_event(void *param)
{
	uint16_t device_listening;
	at_ble_characteristic_changed_t *char_handle = (at_ble_characteristic_changed_t *)param;

	if(bas_service_handler.serv_chars.client_config_handle == char_handle->char_handle)
	{
		device_listening = char_handle->char_new_value[1]<<8| char_handle->char_new_value[0];
		
	}
	return bat_char_changed_event(char_handle->conn_handle,&bas_service_handler, char_handle, &flag);
}

static const ble_event_callback_t battery_app_gap_cb[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	ble_connected_app_event,
	ble_disconnected_app_event,
	NULL,
	NULL,
	ble_paired_app_event,
	NULL,
	NULL,
	NULL,
	NULL,
	ble_paired_app_event,
	NULL,
	NULL,
	NULL,
	NULL
};

static const ble_event_callback_t battery_app_gatt_server_cb[] = {
	ble_notification_confirmed_app_event,
	NULL,
	ble_char_changed_app_event,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};


//////////////////////////////////////////////////////////////////////////////

static const ble_event_callback_t app_gap_handle[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	app_connected_event_handler,
	app_disconnected_event_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

static const ble_event_callback_t app_gatt_server_handle[] = {
	app_notification_cfm_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

static void app_notification_handler(uint8_t notification_enable)
{
	if (notification_enable == true) {
		DBG_LOG("Notification Enabled");
		hw_timer_start(NOTIFICATION_INTERVAL);
	} else {
		hw_timer_stop();
		notification_flag = false;
		DBG_LOG("Notification Disabled");
	}
}

/** @brief hr_notification_confirmation_handler called by ble manager 
 *	to give the status of notification sent
 *  @param[in] at_ble_cmd_complete_event_t address of the cmd completion
 */	
at_ble_status_t app_notification_cfm_handler(void *params)
{
	at_ble_cmd_complete_event_t event_params;
	memcpy(&event_params,params,sizeof(at_ble_cmd_complete_event_t));
	if (event_params.status == AT_BLE_SUCCESS) {
		DBG_LOG_DEV("App Notification Successfully sent over the air");
		notification_sent = true;
	} else {
		DBG_LOG_DEV("Sending Notification over the air failed");
		notification_sent = false;
	}
	return AT_BLE_SUCCESS;
}

/** @brief energy expended handler called by profile to reset the energy values
 *
 */

static void app_reset_handler(void)
{
	energy_expended_val = 0;
	DBG_LOG("Energy Expended is made '0'on user Reset");
}

/** @brief heart_rate_value_init will initializes the heart rate values
 *	 for simulation.
 *	 Based on the time different heart rate values are chosen to indicate
 *	 different activity.
 */
static void heart_rate_value_init(void)
{
	activity = second_counter / 40;

	if (activity != prev_activity) {		
		switch(activity) {
		case ACTIVITY_NORMAL:
			hr_min_value = HEART_RATE_MIN_NORM;
			hr_max_value = HEART_RATE_MAX_NORM;
			heart_rate_value = hr_min_value;
			energy_incrementor = ENERGY_EXP_NORMAL;
			break;
			
		case ACTIVITY_WALKING:
			hr_min_value = HEART_RATE_MIN_WALKING;
			hr_max_value = HEART_RATE_MAX_WALKING;
			heart_rate_value = hr_min_value;
			energy_incrementor = ENERGY_EXP_WALKING;
			break;
			
		case ACTIVITY_BRISK_WALKING:
			hr_min_value = HEART_RATE_MIN_BRISK_WALK;
			hr_max_value = HEART_RATE_MAX_BRISK_WALK;
			heart_rate_value = hr_min_value;
			energy_incrementor = ENERGY_EXP_BRISK_WALKING;
			break;
			
		case ACTIVITY_RUNNING:
			hr_min_value = HEART_RATE_MIN_RUNNING;
			hr_max_value = HEART_RATE_MAX_RUNNING;
			heart_rate_value = hr_min_value;
			energy_incrementor = ENERGY_EXP_RUNNING;
			break;
			
		case ACTIVITY_FAST_RUNNING:
			hr_min_value = HEART_RATE_MIN_FAST_RUNNING;
			hr_max_value = HEART_RATE_MAX_FAST_RUNNING;
			heart_rate_value = hr_min_value;
			energy_incrementor = ENERGY_EXP_FAST_RUNNING;
			break;
		}
		prev_activity = activity;
	}
	
	if (heart_rate_value == hr_max_value) {
		inc_changer = -1;
	} else if (heart_rate_value == hr_min_value) {
		inc_changer = 1;
	}
}

/** @brief connected state handler
 *  @param[in] status of the application
 */
static at_ble_status_t app_connected_event_handler(void *params)
{
	app_state = true;
	LED_On(LED0);
	DBG_LOG("Enable the notification in app to listen "
	"heart rate or press the button to disconnect");
	advertisement_flag = false;
	notification_sent = true;
        ALL_UNUSED(params);
	return AT_BLE_SUCCESS;
}

static at_ble_status_t app_disconnected_event_handler(void *params)
{
	app_state = false;
	hw_timer_stop();
	notification_flag = false;
	energy_expended_val = ENERGY_EXP_NORMAL;
	second_counter = 0;
	activity = ACTIVITY_NORMAL;
	prev_activity = DEFAULT_ACTIVITY;
	energy_inclusion = 0;
	heart_rate_value_init();
	LED_Off(LED0);
	DBG_LOG("Press button to advertise");
	ALL_UNUSED(params);
	return AT_BLE_SUCCESS;
}

/**
 * @brief Button Press Callback
 */
static void button_cb(void)
{
	if (app_state) {
		DBG_LOG_DEV("Going to disconnect ");
		disconnect_flag = true;
	} else if (app_state == false && advertisement_flag == false) {
		/* To check if the device is in advertisement */
		DBG_LOG_DEV("Going to advertisement");
		start_advertisement = true;
		advertisement_flag = true;	
	}
	send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
}

/** @brief hr_measurment_send sends the notifications after adding the hr values
 *	heart rate values starts @60bpm increments by 1 goes upto 255 bpm and
 *	restarts @60
 *  Energy Expended will be sent on every 10th notification,it starts @ 0 and
 *	increments by 20
 *  rr interval values, two rr interval values will be sent in every
 *	notification
 */
static void hr_measurment_send(void)
{
	uint8_t hr_data[HR_CHAR_VALUE_LEN];
	uint8_t idx = 0;
	
	if ((energy_expended_val == ENERGY_RESET) || (second_counter % 10 == energy_inclusion)) {
		hr_data[idx] = (RR_INTERVAL_VALUE_PRESENT | ENERGY_EXPENDED_FIELD_PRESENT);
		
		/* To send energy expended after 10 notifications after reset */
		if (energy_expended_val == ENERGY_RESET) {
			energy_inclusion = second_counter % 10 ;
		}
	} else {
		hr_data[idx] = RR_INTERVAL_VALUE_PRESENT ;
	}
	idx += 1;			
	DBG_LOG("Heart Rate: %d bpm", heart_rate_value);
	heart_rate_value += (inc_changer);
	hr_data[idx++] = string[5];
	/* Heart Rate Value 8bit*/
	//hr_data[idx++] = (uint8_t)heart_rate_value ;
	if (hr_data[0] & ENERGY_EXPENDED_FIELD_PRESENT) {
		memcpy(&hr_data[idx], &energy_expended_val, 2);
		idx += 2;	
	}
	
	/* Appending RR interval values*/	
	if (rr_interval_value >= RR_VALUE_MAX) {
		rr_interval_value = (uint8_t) RR_VALUE_MIN; 
	}	
	DBG_LOG_CONT("\tRR Values:(%d,%d)msec",
				rr_interval_value, rr_interval_value + 200);
	memcpy(&hr_data[idx], &rr_interval_value, 2);
	idx += 2;
	rr_interval_value += 200;
	memcpy(&hr_data[idx], &rr_interval_value, 2);
	idx += 2;
	rr_interval_value += 200;
	
	/*printing the user activity,simulation*/
	switch(activity) {
	case ACTIVITY_NORMAL:
		DBG_LOG_CONT(" User Status:Idle");
		break;
		
	case ACTIVITY_WALKING:
		DBG_LOG_CONT(" User Status:Walking");
		break;
		
	case ACTIVITY_BRISK_WALKING:
		DBG_LOG_CONT(" User status:Brisk walking");
		break;
		
	case ACTIVITY_RUNNING:
		DBG_LOG_CONT(" User status:Running");
		break;
		
	case ACTIVITY_FAST_RUNNING:
		DBG_LOG_CONT(" User Status:Fast Running");
		break;	
	}
	
	/* Printing the energy*/
	if ((hr_data[0] & ENERGY_EXPENDED_FIELD_PRESENT)) {
		DBG_LOG("Energy Expended :%d KJ\n", energy_expended_val);
		energy_expended_val += energy_incrementor;
	}
	
	/* Sending the data for notifications*/
	hr_sensor_send_notification(hr_data, idx);
}

/**
 * \brief Timer callback handler called on timer expiry
 */
static void timer_callback_handler(void)
{
	if (second_counter == START_OF_FIRST_ACTIVITY) {
		time_operator = 1;
	} else if (second_counter == END_OF_LAST_ACTIVITY) {
		time_operator = -1;
	}
	second_counter += (time_operator);
	heart_rate_value_init();
	notification_flag = true;
	Timer_Flag = true;
	
	send_plf_int_msg_ind(USER_TIMER_CALLBACK, TIMER_EXPIRED_CALLBACK_TYPE_DETECT, NULL, 0);
}

/* to make app executing continuously*/
bool app_exec = true;
/**
 * \brief Heart Rate Sensor Application main function
 */

//////////////////////////////////////////////////////////////////////////////////

static void transfer_done_rx(struct dma_resource* const resource )
{
// 		
// 		if(string[0]==3)
// 		{
// 			if(state==true)
// 			{
// 				gpio_pin_toggle_output_level(LED_0_PIN);
// 				state=false;
// 			}
// 			
// 
// 		}
// 		else if(string[0]==2)
// 		{
// 			if(state==false)
// 			{
// 			gpio_pin_toggle_output_level(LED_0_PIN);
// 			state=true;
// 			}
// 		}
	
	dma_start_transfer_job(&uart_dma_resource_rx);
	
}

static void configure_dma_resource_rx(struct dma_resource *resource)
{
	struct dma_resource_config config;

	dma_get_config_defaults(&config);

	config.src.periph = UART0RX_DMA_PERIPHERAL;
	config.src.enable_inc_addr = false;
	config.src.periph_delay = 1;

	dma_allocate(resource, &config);
}

static void setup_transfer_descriptor_rx(struct dma_descriptor *descriptor)
{
	dma_descriptor_get_config_defaults(descriptor);

	descriptor->buffer_size = BUFFER_LEN;
    descriptor->read_start_addr =
    (uint32_t)(&(uart_instance.hw->RECEIVE_DATA.reg));
	descriptor->write_start_addr = (uint32_t)string	;

}


static void configure_usart(void)
{
	struct uart_config config_uart;

	uart_get_config_defaults(&config_uart);
	
	config_uart.baud_rate = 9600;
config_uart.pin_number_pad[0] = EDBG_CDC_SERCOM_PIN_PAD0;
config_uart.pin_number_pad[1] = EDBG_CDC_SERCOM_PIN_PAD1;
config_uart.pin_number_pad[2] = EDBG_CDC_SERCOM_PIN_PAD2;
config_uart.pin_number_pad[3] = EDBG_CDC_SERCOM_PIN_PAD3;
config_uart.pinmux_sel_pad[0] = EDBG_CDC_SERCOM_MUX_PAD0;
config_uart.pinmux_sel_pad[1] = EDBG_CDC_SERCOM_MUX_PAD1;
config_uart.pinmux_sel_pad[2] = EDBG_CDC_SERCOM_MUX_PAD2;
config_uart.pinmux_sel_pad[3] = EDBG_CDC_SERCOM_MUX_PAD3;


	while (uart_init(&uart_instance,
	EDBG_CDC_MODULE, &config_uart) != STATUS_OK) {
	}

	//uart_enable_transmit_dma(&uart_instance);
	uart_enable_receive_dma(&uart_instance);
}

static void configure_dma_callback(void)
{
	dma_register_callback(&uart_dma_resource_rx, transfer_done_rx, DMA_CALLBACK_TRANSFER_DONE);
	dma_enable_callback(&uart_dma_resource_rx, DMA_CALLBACK_TRANSFER_DONE);

	NVIC_EnableIRQ(PROV_DMA_CTRL0_IRQn);
}

/* Timer callback */
// static void timer_callback_handler(void)
// {
// 	/* Stop timer */
// 	hw_timer_stop();
// 	/* Set timer Alarm flag */
// 	Timer_Flag = true;
// 	/* Restart Timer */
// 	hw_timer_start(10);
// }

/* Sending the temperature value after reading it from IO1 Xplained Pro */
static void htp_temperature_send(void) {
	at_ble_prf_date_time_t timestamp;
//	enable_uart();
	float *temperature;
	//float temperature_value;
	//uart_rx_value;
	/* Read Temperature Value from IO1 Xplained Pro */
// 	temperature = at30tse_read_temperature();
// #ifdef HTPT_FLAG_FAHRENHEIT
// 	temperature = (((temperature * 9.0)/5.0) + 32.0);
// #endif
	/* Read Temperature Value from IO1 Xplained Pro */
	timestamp.day = 1;
	timestamp.hour = 9;
	timestamp.min = 2;
	timestamp.month = 8;
	timestamp.sec = 36;
	timestamp.year = 15;
	temperature = (float *)&string[0];
	pressure_ptr = (uint16_t *)&string[4];
	pressure = *pressure_ptr;
	//uint16_t temperature_value = *temperature;

	
		
	/* Read Temperature Value from IO1 Xplained Pro */
	if(at_ble_htpt_temp_send(convert_ieee754_ieee11073_float(*temperature),
	&timestamp,
	(at_ble_htpt_temp_flags)(HTPT_FLAG_CELSIUS | HTPT_FLAG_TYPE),
	HTP_TYPE_MOUTH,
	1
	) == AT_BLE_SUCCESS)
	{
// #ifdef HTPT_FLAG_FAHRENHEIT
// 		printf("Temperature: %d Fahrenheit", (uint16_t)string);
// 		printf("\nBaud rate is fine");
// #else
// 		printf("Temperature: %d Deg Celsius", *temperature);
// 		printf("\nBaud rate is fine");
// #endif
	}
	
}

static at_ble_status_t  app_htpt_cfg_indntf_ind_handler(void *params)
{
	at_ble_htpt_cfg_indntf_ind_t htpt_cfg_indntf_ind_params;
	memcpy((uint8_t *)&htpt_cfg_indntf_ind_params, params, sizeof(at_ble_htpt_cfg_indntf_ind_t));
	if (htpt_cfg_indntf_ind_params.ntf_ind_cfg == 1) {
		printf("Started HTP Temperature Notification");
		Temp_Notification_Flag = true;
	}
	else {
		printf("HTP Temperature Notification Stopped");
		Temp_Notification_Flag = false;
	}
	return AT_BLE_SUCCESS;
}

static const ble_event_callback_t app_htpt_handle[] = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	app_htpt_cfg_indntf_ind_handler,
	NULL,
	NULL,
	NULL
};


static void htp_init (void){
	printf("\nAssignment 4.1: Init Health temperature service ");
	/* Create htp service in GATT database*/
	status = at_ble_htpt_create_db(
	HTPT_TEMP_TYPE_CHAR_SUP,
	HTP_TYPE_ARMPIT,
	1,
	30,
	1,
	HTPT_AUTH,
	&htpt_conn_handle);
	if (status != AT_BLE_SUCCESS){
		printf("HTP Data Base creation failed");
		while(1);
	}
}


static void ble_advertise (void)
{
	printf("\nAssignment 2.1 : Start Advertising");
	status = ble_advertisement_data_set();
	if(status != AT_BLE_SUCCESS)
	{
		printf("\n\r## Advertisement data set failed : error %x",status);
		while(1);
	}
	/* Start of advertisement */
	status = at_ble_adv_start(AT_BLE_ADV_TYPE_UNDIRECTED,\
	AT_BLE_ADV_GEN_DISCOVERABLE,\
	NULL,\
	AT_BLE_ADV_FP_ANY,\
	1000,\
	655,\
	0);
	if(status != AT_BLE_SUCCESS)
	{
		printf("\n\r## Advertisement data set failed : error %x",status);
		while(1);
	}
}

/* Callback registered for AT_BLE_CONNECTED event*/
static at_ble_status_t ble_paired_cb (void *param)
{
	at_ble_pair_done_t *pair_params = param;
	printf("\nAssignment 3.2: Application paired ");
	/* Enable the HTP Profile */
	printf("\nAssignment 4.1: enable health temperature service ");
	status = at_ble_htpt_enable(pair_params->handle,
	HTPT_CFG_STABLE_MEAS_IND);
	if(status != AT_BLE_SUCCESS){
		printf("*** Failure in HTP Profile Enable");
		while(1);
	}
	ALL_UNUSED(param);
	return AT_BLE_SUCCESS;
}

/* Callback registered for AT_BLE_DISCONNECTED event */
static at_ble_status_t ble_disconnected_cb (void *param)
{
	printf("\nAssignment 3.2: Application disconnected ");
	ble_advertise();
	ALL_UNUSED(param);return AT_BLE_SUCCESS;
}

static const ble_event_callback_t app_gap_cb[] = {
	NULL, // AT_BLE_UNDEFINED_EVENT
	NULL, // AT_BLE_SCAN_INFO
	NULL, // AT_BLE_SCAN_REPORT
	NULL, // AT_BLE_ADV_REPORT
	NULL, // AT_BLE_RAND_ADDR_CHANGED
	NULL, // AT_BLE_CONNECTED
	ble_disconnected_cb, // AT_BLE_DISCONNECTED
	NULL, // AT_BLE_CONN_PARAM_UPDATE_DONE
	NULL, // AT_BLE_CONN_PARAM_UPDATE_REQUEST
	ble_paired_cb, // AT_BLE_PAIR_DONE
	NULL, // AT_BLE_PAIR_REQUEST
	NULL, // AT_BLE_SLAVE_SEC_REQUEST
	NULL, // AT_BLE_PAIR_KEY_REQUEST
	NULL, // AT_BLE_ENCRYPTION_REQUEST
	NULL, // AT_BLE_ENCRYPTION_STATUS_CHANGED
	NULL, // AT_BLE_RESOLV_RAND_ADDR_STATUS
	NULL, // AT_BLE_SIGN_COUNTERS_IND
	NULL, // AT_BLE_PEER_ATT_INFO_IND
	NULL  // AT_BLE_CON_CHANNEL_MAP_IND
};

static void register_ble_callbacks (void)
{
	/* Register GAP Callbacks */
	printf("\nAssignment 3.2: Register bluetooth events callbacks");
	status = ble_mgr_events_callback_handler(REGISTER_CALL_BACK,\
	BLE_GAP_EVENT_TYPE,app_gap_cb);
	if (status != true) {
		printf("\n##Error when Registering  SAMB11 gap callbacks");
	}
	status = ble_mgr_events_callback_handler(REGISTER_CALL_BACK,\
	BLE_GATT_HTPT_EVENT_TYPE,app_htpt_handle);
	if (status != true) {
		printf("\n##Error when Registering  SAMB11 htpt callbacks");
	}
}



static void configure_gpio_pins(void)
 {
			struct gpio_config config_gpio_pin;

			gpio_get_config_defaults(&config_gpio_pin);

			config_gpio_pin.direction  = GPIO_PIN_DIR_INPUT;
			config_gpio_pin.input_pull = GPIO_PIN_PULL_NONE;

			gpio_pin_set_config(BUTTON_0_PIN, &config_gpio_pin);

			config_gpio_pin.direction = GPIO_PIN_DIR_OUTPUT;

			gpio_pin_set_config(LED_0_PIN, &config_gpio_pin);
			gpio_pin_toggle_output_level(LED_0_PIN);
 }

static void enable_uart(void)
{
		system_clock_config(CLOCK_RESOURCE_XO_26_MHZ, CLOCK_FREQ_26_MHZ);
		
		configure_usart();
		configure_dma_resource_rx(&uart_dma_resource_rx);
		setup_transfer_descriptor_rx(&example_descriptor_rx);
		dma_add_descriptor(&uart_dma_resource_rx, &example_descriptor_rx);
		configure_dma_callback();
		dma_start_transfer_job(&uart_dma_resource_rx);
}

int main (void)
{
	
		app_state = 0;  /*!< flag to represent the application state*/
		start_advertisement = 0; /*!< flag to start advertisement*/
		advertisement_flag = false; /*!< to check if the device is in advertisement*/
		notification_flag = false; /*!< flag to start notification*/
		disconnect_flag = false;        /*!< flag for disconnection*/
		hr_initializer_flag = 1; /*!< flag for initialization of hr for each category*/
		second_counter = 0;     /*!< second_counter to count the time*/
		energy_expended_val = ENERGY_EXP_NORMAL; /*!< to count the energy expended*/
		energy_incrementor = 0;
		heart_rate_value = HEART_RATE_MIN_NORM; /*!< to count the heart rate value*/
		rr_interval_value = RR_VALUE_MIN; /*!< to count the rr interval value*/
		activity = 0; /*!< activiy which will determine the */
		prev_activity = DEFAULT_ACTIVITY; /*!< previous activity */
		inc_changer     = 1; /*!< increment operator to change heart rate */
		time_operator = 0; /*!< operator to change the seconds */
		hr_min_value = 0; /*!<the minimum heart rate value*/
		hr_max_value = 0; /*!<the maximum heart rate value*/

		app_exec = true;
		//at_ble_status_t status;
		uint8_t battery_level = BATTERY_MIN_LEVEL;
	platform_driver_init();
	acquire_sleep_lock();
	
		gpio_init();
		button_init();
		button_register_callback(button_cb);
		
		//led_init();
	
	configure_gpio_pins();
	/* Initialize serial console */
	serial_console_init();
	/* Initialize the hardware timer */
	hw_timer_init();
	/* Register the callback */
	hw_timer_register_callback(timer_callback_handler);
	/* Start timer */
	hw_timer_start(1);
	//configure_gpio_pins();
	//enable_uart();
	/* initialize the BLE chip and Set the Device Address */
	ble_device_init(NULL);
	
	hr_sensor_init(NULL);
	htp_init();
	/* Register Bluetooth events Callbacks */
	register_ble_callbacks();
		/* Registering the app_notification_handler with the profile */
		register_hr_notification_handler(app_notification_handler);

		/* Registering the app_reset_handler with the profile */
		register_hr_reset_handler(app_reset_handler);
		
		bat_init_service(&bas_service_handler, &battery_level);
		
		/* Define the primary service in the GATT server database */
		if((status = bat_primary_service_define(&bas_service_handler))!= AT_BLE_SUCCESS)
		{
			DBG_LOG("defining battery service failed %d", status);
		}
	
	battery_service_advertise();

    ble_advertise();
	
		ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
		BLE_GAP_EVENT_TYPE,
		battery_app_gap_cb);
		
		/* Register callbacks for gatt server related events */
		ble_mgr_events_callback_handler(REGISTER_CALL_BACK,
		BLE_GATT_SERVER_EVENT_TYPE,
		battery_app_gatt_server_cb);
	
	enable_uart();
	//at_ble_tx_power_set(AT_BLE_TX_PWR_LVL_NEG_20_DB);
	
	while(1) {
		i++;
		
		if (start_advertisement == true || disconnect_flag == true) {
			/* button debounce delay*/
			/*delay_ms(350);*/
		}
		
		/* Flag to start advertisement */
		if (start_advertisement) {
			hr_sensor_adv();
			start_advertisement = false;
		}

		/* Flag to start notification */
		if (notification_flag) {
			//LED_Toggle(LED0);
			if (notification_sent) {
				hr_measurment_send();
				} else {
				DBG_LOG("Previous notification not sent");
			}
			
			notification_flag = false;
		}

		/* Flag to disconnect with the peer device */
		if (disconnect_flag) {
			hr_sensor_disconnect();
			app_state = false;
			disconnect_flag = false;
		}
					ble_event_task(655);  
					htp_temperature_send();
					bat_update_char_value(bat_connection_handle,&bas_service_handler, string[4], &flag);
				
	}
}


