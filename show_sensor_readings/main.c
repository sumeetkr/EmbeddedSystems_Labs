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
nrk_task_type TaskOne;
void print_sensor_readings(void);

nrk_task_type TaskTwo;
NRK_STK Stack2[NRK_APP_STACKSIZE];
void collect_Audio_Data(void);

uint16_t * audio_data;
int8_t audio_data_size;
uint16_t calculate_rms(uint16_t audios [], int8_t window_size);

void nrk_create_taskset();
void nrk_register_drivers();
uint8_t kill_stack(uint8_t val);

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
nrk_create_taskset()
{
  TaskOne.task = print_sensor_readings;
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 2;
  TaskOne.period.nano_secs = 0;
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs = 0;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);

  nrk_task_set_entry_function( &TaskTwo, collect_Audio_Data);
  nrk_task_set_stk( &TaskTwo, Stack2, NRK_APP_STACKSIZE);
  TaskTwo.prio = 2;
  TaskTwo.FirstActivation = TRUE;
  TaskTwo.Type = BASIC_TASK;
  TaskTwo.SchType = PREEMPTIVE;
  TaskTwo.period.secs = 2;
  TaskTwo.period.nano_secs = 0;
  TaskTwo.cpu_reserve.secs = 1;
  TaskTwo.cpu_reserve.nano_secs = 0;
  TaskTwo.offset.secs = 0;
  TaskTwo.offset.nano_secs= 0;
  nrk_activate_task (&TaskTwo);
}


void print_sensor_readings()
{
uint16_t cnt;
int8_t i,fd,val;
uint16_t buf;
uint16_t buf_last = 0;
uint16_t motions[] = {0,0};
uint16_t index = 0;
uint64_t bbuf;

  printf( "My node's address is %d\r\n",NODE_ADDR );

  printf( "Task1 PID=%d\r\n",nrk_get_pid());

  
  	// Open ADC device as read 
  	fd=nrk_open(FIREFLY_3_SENSOR_BASIC,READ);
  	if(fd==NRK_ERROR) nrk_kprintf(PSTR("Failed to open sensor driver\r\n"));

	cnt=0;
	while(1) 
	{

		// humidity = 37% temperature(A) = 59.7F temperature(D) = 77.1F pressure= 29.85inHg  light=259lm  acc_x=510 acc_y=490 motion=detected  audio=30
		// Example of setting a sensor 
		val=nrk_set_status(fd,SENSOR_SELECT,HUMIDITY);
		val=nrk_read(fd,&bbuf,4); //percent humidity
		// Relative Humidity (%)  linear          |  1 |    0 |         0 |       100 |
		printf( "Percent humidity (in %%)= %lu, ",bbuf);

	    // | digital_temp | Temperature (digital sensor, F) | linear          |              0.18 |   32 |        40 |        90
		val=nrk_set_status(fd,SENSOR_SELECT,TEMP2);
		val=nrk_read(fd,&bbuf,4); 
		uint16_t temp_converted = ((9*bbuf)/5)+ 32;
		printf( " Digital Temperature (in F)= %d, ", temp_converted/10);

		val=nrk_set_status(fd,SENSOR_SELECT,LIGHT);
		val=nrk_read(fd,&buf,2);
		// light        | Incident Illumination (lumens)  | linear          |                -1 | 1024 |         0 |      1024 
	    char *light_type;

		if(buf > 1000){
			light_type= "dark \r";
		}
		else
		{
			if(buf < 580){
				light_type= "daylight \r";
			}
			else
			{
				light_type= "room-light \r";
			}
		}

	    
		printf( " Ambient light (in lumens)= %d, ",-1*buf + 1024);
	    printf( " Light type detected as %s\r\n", light_type);
		
		val=nrk_set_status(fd,SENSOR_SELECT,PRESS);
		val=nrk_read(fd,&bbuf,4);
		// pressure     | Barometric Pressure (in. Hg)    | linear          | 0.000295780903035 |    0 |        28 |        31 |
		printf( "Barometric Pressure (in. Hg)= %d, ",295*(bbuf)/1000000);
		
		// | acc_x        | Acceleration - X (ft / sec^2)   | linear          |                 1 |    0 |         0 |        50 |
		// | acc_y        | Acceleration - Y (ft / sec^2)   | linear          |                 1 |    0 |         0 |        50 |
		// | acc_z        | Acceleration - Z (ft / sec^2)   | linear          |                 1 |    0 |         0 |        50
		val=nrk_set_status(fd,SENSOR_SELECT, ACC_X);
		val=nrk_read(fd,&buf,2);
		printf( " Acceleration - X (ft / sec^2) =%d, ",buf);
		val=nrk_set_status(fd,SENSOR_SELECT, ACC_Y);
		val=nrk_read(fd,&buf,2);
		printf( " Acceleration - Y (ft / sec^2) =%d\r\n",buf);


		// motion       | Motion Detected (binary)        | threshhold      |               950 |    0 |         0 |         1
		val=nrk_set_status(fd,SENSOR_SELECT, MOTION);		
		val=nrk_read(fd,&buf,2);
		// printf( " Motion value =%d\r\n",buf);
		if(buf> 990){
			printf( "Motion based on Infrared = Yes ");
		}
		else
		{
			printf( "Motion based on Infrared = No ");
		}

		uint16_t sum = 0;
	    int size = sizeof(motions)/sizeof(motions[0]);
		for ( i = 0; i < size; i++ )
	     {
	      sum = sum + motions[i];
	     }

		uint16_t avg = sum/size;

	    nrk_led_clr(RED_LED);
	    nrk_led_clr(GREEN_LED);

	    printf( ":");
		if( abs(buf - avg) >10)
		//if( abs(buf - buf_last) >2)
			{ printf(" Motion based on light change = Yes \r\n" );
		      nrk_led_toggle(RED_LED);
		  	}
		else 
		{
			printf(" Motion based on light change = No \r\n" );
			nrk_led_toggle(GREEN_LED);
		}

		motions[index] = buf;

		buf_last = buf; 


		nrk_wait_until_next_period();
		cnt++;

		index++;
		if(index == size) index =0;
	}
}

void 
collect_Audio_Data(){
printf( "Task PID=%u\r\n",nrk_get_pid());

int8_t fd, val, index;
uint16_t buf;


// Open ADC device as read 
fd=nrk_open(FIREFLY_3_SENSOR_BASIC,READ);
if(fd==NRK_ERROR) nrk_kprintf(PSTR("Failed to open sensor driver\r\n"));

	while(1)
	{
		val=nrk_set_status(fd,SENSOR_SELECT,AUDIO);
    	//8 KHZ sensor
		nrk_spin_wait_us(125);
		val=nrk_read(fd,&buf,2);

    	// printf( " audio=%d\r\n", buf);

		nrk_led_clr(BLUE_LED);
		nrk_led_clr(RED_LED);
		nrk_led_toggle(RED_LED);

		audio_data[index] = buf;
		index++;

		if(index == audio_data_size)
		{
			index =0;
			calculate_rms(audio_data, audio_data_size);
			nrk_wait_until_next_period();
		}
	}
}

uint16_t
calculate_rms(uint16_t audios [], int8_t window_size)
{
  uint8_t index = 0;
  uint32_t rawrms  =0;
  uint32_t rms  = 0;

  while(index  < window_size)
  {
    rawrms += (uint32_t)audios[index] * (uint32_t)audios[index];
    index ++;
  }

  rms = rawrms / window_size ;
  rms = sqrt(rms) ;

    // No conversion needed as a =1 and b =0 in linear transformation
    // convert to dB = 20 * log(average / (2 ^ (BitDepth - 1))) 
    printf( "Audio RMS value =%ld\r\n",rms);

  return 0;
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


