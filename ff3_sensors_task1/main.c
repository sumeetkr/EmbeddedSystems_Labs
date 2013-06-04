/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  Contributing Authors (specific to this file):
*  Zane Starr
*******************************************************************************/


#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>

NRK_STK Stack1[NRK_APP_STACKSIZE];
NRK_STK Stack2[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
nrk_task_type TaskTwo;

void Collect_Audio_Data(void);
uint16_t calculate_rms(uint16_t audios [], int8_t size, int8_t window_size);
void Analyze_Audio_Data(void);

void nrk_create_taskset();
void nrk_register_drivers();
uint8_t kill_stack(uint8_t val);
uint16_t * audio_data;
int8_t audio_data_size;

int
main ()
{
  uint8_t t;
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);


  printf( PSTR("starting...\r\n") );
  audio_data_size = 30;
  audio_data = malloc(audio_data_size * (sizeof audio_data[0]));

  nrk_init();
  nrk_time_set(0,0);

  nrk_register_drivers();
  nrk_create_taskset ();
  nrk_start();
  
  return 0;
}


void 
Collect_Audio_Data(){

int8_t fd, val, index;
uint16_t buf;

// Open ADC device as read 
fd=nrk_open(FIREFLY_3_SENSOR_BASIC,READ);
if(fd==NRK_ERROR) nrk_kprintf(PSTR("Failed to open sensor driver\r\n"));

	while(1)
	{
		val=nrk_set_status(fd,SENSOR_SELECT,AUDIO_P2P);
		nrk_spin_wait_us(60000);
		val=nrk_read(fd,&buf,2);
		printf( " audio=%d\r\n",buf);

		nrk_led_clr(BLUE_LED);
		nrk_led_clr(RED_LED);
		nrk_led_toggle(RED_LED);

		audio_data[index] = buf;
		index++;

		if(index == audio_data_size)
		{
			index =0;
		}

		nrk_wait_until_next_period();
	}
}

uint16_t
calculate_rms(uint16_t audios [], int8_t size, int8_t window_size)
{
	uint8_t index = 0;
	uint8_t window_count = 0;

	while(index + window_size <= size)
	{
		uint16_t rawrms  =0;
		uint16_t rms  = 0;

		for(uint8_t count = index ; count < index + window_size ; count++ )
		{
			rawrms += audios[count] * audios[count] ;
		}

		rms = rawrms / window_size ;
		rms = sqrt(rms) ;

		printf( "window no =%d", window_count);
		printf( " has rms value =%d\r\n",rms);

		window_count++;
		index = index + window_size;
	}

	return 0;
}

void
Analyze_Audio_Data()
{
 int16_t cnt;
  printf( "Task2 PID=%u\r\n",nrk_get_pid());
  cnt=0;
  while(1) {

  	nrk_led_clr(BLUE_LED);
  	nrk_led_clr(RED_LED);
	nrk_led_toggle(BLUE_LED);
	// printf( "Task2 signed cnt=%d\r\n",cnt );
  	printf( "%u\r\n", 2);

  	uint16_t rms = calculate_rms(audio_data, audio_data_size, 10);
	//nrk_stats_display_pid(nrk_get_pid());
	nrk_wait_until_next_period();
	cnt--;
	}
}

void
nrk_create_taskset()
{
 nrk_task_set_entry_function( &TaskOne, Collect_Audio_Data);
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 2*NANOS_PER_MS; //*NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 1;
  TaskOne.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);

  nrk_task_set_entry_function( &TaskTwo, Analyze_Audio_Data);
  nrk_task_set_stk( &TaskTwo, Stack2, NRK_APP_STACKSIZE);
  TaskTwo.prio = 2;
  TaskTwo.FirstActivation = TRUE;
  TaskTwo.Type = BASIC_TASK;
  TaskTwo.SchType = PREEMPTIVE;
  TaskTwo.period.secs = 0;
  TaskTwo.period.nano_secs = 2500*NANOS_PER_MS;
  TaskTwo.cpu_reserve.secs = 0;
  TaskTwo.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
  TaskTwo.offset.secs = 0;
  TaskTwo.offset.nano_secs= 0;
  nrk_activate_task (&TaskTwo);

}

void nrk_register_drivers()
{
int8_t val;

// Register the Basic FireFly Sensor device driver
// Make sure to add: 
//     #define NRK_MAX_DRIVER_CNT  
//     in nrk_cfg.h
// Make sure to add: 
//     SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM_TYPE)/source/ff_basic_sensor.c
//     in makefile
val=nrk_register_driver( &dev_manager_ff3_sensors,FIREFLY_3_SENSOR_BASIC);
if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to load my ADC driver\r\n") );

}


