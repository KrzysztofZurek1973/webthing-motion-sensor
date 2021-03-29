/*
 * thing_motion_sensor.c
 *
 *  Created on: Mar 25, 2021
 *      Author: Krzysztof Zurek
 *		e-mail: krzzurek@gmail.com
 */
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "simple_web_thing_server.h"
#include "webthing_motion_sensor.h"

#define MAX_MOTION 5000

//button GPIO
#define GPIO_INPUT		    (CONFIG_GPIO_MOTION_IN)
#define GPIO_INPUT_MASK		(1ULL << GPIO_INPUT)

xSemaphoreHandle DRAM_ATTR thing_sem;
xSemaphoreHandle DRAM_ATTR thing_mux;
static int DRAM_ATTR irq_counter = 0;

thing_t *motion_sensor = NULL;
property_t *prop_motion, *prop_counter, *prop_alarm_active, *prop_alarm_counter;
property_t *prop_last_alarm;
event_t *alarm_event;

at_type_t motion_sensor_type, motion_prop_type, counter_prop_type;
at_type_t alarm_prop_type, alarm_cnt_prop_type, last_alarm_prop_type;

static bool DRAM_ATTR motion_detected = false;
static bool DRAM_ATTR alarm_activated = false;
static char DRAM_ATTR last_alarm_buff[25];
static int32_t DRAM_ATTR alarm_counter = 0;
static time_t DRAM_ATTR time_now = 0, prev_alarm_time = 0;

char *last_alarm_model_jsonize(property_t *p);
char *last_alarm_value_jsonize(property_t *p);

/* ************************************************************
 *
 * button interrupt
 *
 * ***********************************************************/
static void IRAM_ATTR gpio_isr_handler(void* arg){
	static portBASE_TYPE xHigherPriorityTaskWoken;

	xSemaphoreGiveFromISR(thing_sem, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR();
}


/*************************************************************
 *
 * main button function
 *
 * ************************************************************/
void sensor_fun(void *pvParameter){

	printf("Button task is ready\n");
	
	//button_ready = true;

	for(;;){
		//wait sensor to be trrigered
		xSemaphoreTake(thing_sem, portMAX_DELAY);
		
		//test
		//printf("irq_fun: %i\n", irq_counter);
		//test end
		
		int sensor_value = gpio_get_level(GPIO_INPUT);

		if (sensor_value == 1){
			//motion detected
			motion_detected = true;
			irq_counter++;
			if (irq_counter > MAX_MOTION){
				irq_counter = 1;
			}
			inform_all_subscribers_prop(prop_motion);
			inform_all_subscribers_prop(prop_counter);
			
			if (alarm_activated == true){
				struct tm timeinfo;
				char *time_buffer;
				
				time(&time_now);
				
				if ((time_now - prev_alarm_time) > 300){
					prev_alarm_time = time_now; 
					time_buffer = malloc(25);
					localtime_r(&time_now, &timeinfo);
					strftime(time_buffer, 25, "%Y-%m-%d %H:%M:%S", &timeinfo);
					//printf("send event, time: %s\n", time_buffer);
					memset(last_alarm_buff, 0, 25);
					strcpy(last_alarm_buff, time_buffer);
				
					time_buffer[10] = 'T';
					emit_event(motion_sensor -> thing_nr, "motion", (void *)time_buffer);
					inform_all_subscribers_prop(prop_last_alarm);
					alarm_counter++;
					inform_all_subscribers_prop(prop_alarm_counter);
				}
			}
		}
		else{
			motion_detected = false;
			inform_all_subscribers_prop(prop_motion);
		}
	}
}


/*******************************************************************
*
* set ON/OFF state
*
*******************************************************************/
int16_t alarm_activate(char *new_value_str){
	int8_t res = 1;
	
	xSemaphoreTake(thing_mux, portMAX_DELAY);
	if (strcmp(new_value_str, "true") == 0){
		alarm_activated = true;
		printf("alarm active\n");
		//gpio_set_level(GPIO_LED, 1);
	}
	else{
		alarm_activated = false;
		printf("alarm NOT active\n");
		//gpio_set_level(GPIO_LED, 0);
	}

	xSemaphoreGive(thing_mux);
	
	return res;
}


/*******************************************************************
 *
 * initialize button's GPIO
 *
 * ******************************************************************/
void init_button_io(void){
	gpio_config_t io_conf;

	//interrupt on both edges
	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	//bit mask of the pins, use GPIO4/5 here
	io_conf.pin_bit_mask = GPIO_INPUT_MASK;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	io_conf.pull_down_en = 0;
	gpio_config(&io_conf);

	gpio_isr_handler_add(GPIO_INPUT, gpio_isr_handler, NULL);
}


/*****************************************************************
 *
 * Initialize button thing and all it's properties and event
 *
 * ****************************************************************/
thing_t *init_motion_sensor(void){
	//thing_t *motion_sensor = NULL;

	vSemaphoreCreateBinary(thing_sem);
	xSemaphoreTake(thing_sem, 0);
	init_button_io();
	
	thing_mux = xSemaphoreCreateMutex();
	//create button thing
	motion_sensor = thing_init();
	
	motion_sensor -> id = "motion_sensor";
	motion_sensor -> at_context = things_context;
	motion_sensor -> model_len = 2000;
	//set @type
	motion_sensor_type.at_type = "MotionSensor";
	motion_sensor_type.next = NULL;
	set_thing_type(motion_sensor, &motion_sensor_type);
	motion_sensor -> description = "WebThing: motion sensor";
	
	//create motion property
	prop_motion = property_init(NULL, NULL);
	prop_motion -> id = "motion";
	prop_motion -> description = "motion";
	motion_prop_type.at_type = "MotionProperty";
	motion_prop_type.next = NULL;
	prop_motion -> at_type = &motion_prop_type;
	prop_motion -> type = VAL_BOOLEAN;
	prop_motion -> value = &motion_detected;
	prop_motion -> title = "Motion";
	prop_motion -> read_only = true;
	prop_motion -> set = NULL;
	prop_motion -> mux = thing_mux;

	add_property(motion_sensor, prop_motion); //add property to thing
	
	//create counter property
	prop_counter = property_init(NULL, NULL);
	prop_counter -> id = "counter";
	prop_counter -> description = "event counter";
	counter_prop_type.at_type = "LevelProperty";
	counter_prop_type.next = NULL;
	prop_counter -> at_type = &counter_prop_type;
	prop_counter -> type = VAL_INTEGER;
	prop_counter -> value = &irq_counter;
	prop_counter -> max_value.int_val = MAX_MOTION;
	prop_counter -> min_value.int_val = 0;
	prop_counter -> unit = "events";
	prop_counter -> title = "Counter-1";
	prop_counter -> read_only = true;
	prop_counter -> set = NULL;
	prop_counter -> mux = thing_mux;

	add_property(motion_sensor, prop_counter); //add property to thing
	
	//create "alarm_activated" property
	prop_alarm_active = property_init(NULL, NULL);
	prop_alarm_active -> id = "alarm";
	prop_alarm_active -> description = "alarm is activated";
	alarm_prop_type.at_type = "OnOffProperty";
	alarm_prop_type.next = NULL;
	prop_alarm_active -> at_type = &alarm_prop_type;
	prop_alarm_active -> type = VAL_BOOLEAN;
	prop_alarm_active -> value = &alarm_activated;
	prop_alarm_active -> title = "Alarm ON/OFF";
	prop_alarm_active -> read_only = false;
	prop_alarm_active -> set = alarm_activate;
	prop_alarm_active -> mux = thing_mux;

	add_property(motion_sensor, prop_alarm_active); //add property to thing
	
	//create "last_alarm" property
	prop_last_alarm = property_init(NULL, NULL);
	prop_last_alarm -> id = "last-alarm";
	prop_last_alarm -> description = "last alarm";
	last_alarm_prop_type.at_type = "StringProperty";
	last_alarm_prop_type.next = NULL;
	prop_last_alarm -> at_type = &last_alarm_prop_type;
	prop_last_alarm -> type = VAL_STRING;
	prop_last_alarm -> value = last_alarm_buff;
	prop_last_alarm -> unit = "---";
	prop_last_alarm -> title = "Last alarm";
	prop_last_alarm -> read_only = true;
	prop_last_alarm -> set = NULL;
	prop_last_alarm -> model_jsonize = last_alarm_model_jsonize;
	prop_last_alarm -> value_jsonize = last_alarm_value_jsonize;
	prop_last_alarm -> mux = thing_mux;

	add_property(motion_sensor, prop_last_alarm); //add property to thing
	strcpy(last_alarm_buff, "NO ALARM");

	//create "alarm_counter" property
	prop_alarm_counter = property_init(NULL, NULL);
	prop_alarm_counter -> id = "alarm-cnt";
	prop_alarm_counter -> description = "alarm counter";
	alarm_cnt_prop_type.at_type = "LevelProperty";
	alarm_cnt_prop_type.next = NULL;
	prop_alarm_counter -> at_type = &alarm_cnt_prop_type;
	prop_alarm_counter -> type = VAL_INTEGER;
	prop_alarm_counter -> value = &alarm_counter;
	prop_alarm_counter -> max_value.int_val = 99;
	prop_alarm_counter -> min_value.int_val = 0;
	prop_alarm_counter -> unit = "alarms";
	prop_alarm_counter -> title = "Counter-2";
	prop_alarm_counter -> read_only = true;
	prop_alarm_counter -> set = NULL;
	prop_alarm_counter -> mux = thing_mux;

	add_property(motion_sensor, prop_alarm_counter); //add property to thing
	
	//event "motion detected"
	alarm_event = event_init();
	alarm_event -> id = "motion";
	alarm_event -> title = "motion alarm";
	alarm_event -> description = "motion detected";
	alarm_event -> type = VAL_STRING;
	alarm_event -> at_type = "AlarmEvent";
	alarm_event -> unit = "pcs";

	add_event(motion_sensor, alarm_event); //add event to thing
	
	//read current state
	if (gpio_get_level(GPIO_INPUT) == 1){
		motion_detected = true;
	}
	else{
		motion_detected = false;
	}
	
	if (thing_sem != NULL){
		xTaskCreate(&sensor_fun, "pir_sensor_task",
					configMINIMAL_STACK_SIZE * 4, NULL, 0, NULL);
	}

	return motion_sensor;
}


/****************************************************
 *
 * jsonize model of last alarm property
 *
 * **************************************************/
char *last_alarm_model_jsonize(property_t *p){
	char *buff;

	//only unit printed in model, is it enough?
	buff = malloc(12 + strlen(p -> unit));
	sprintf(buff, "\"unit\":\"%s\",", p -> unit);

	return buff;
}


/****************************************************
 *
 * last_alarm jsonize
 *
 * **************************************************/
char *last_alarm_value_jsonize(property_t *p){
	char *buff;

	buff = malloc(40);
	memset(buff, 0, 40);
	strcpy(buff, "\"last-alarm\":\"");
	strcat(buff, p -> value);
	strcat(buff, "\"");

	return buff;
}
