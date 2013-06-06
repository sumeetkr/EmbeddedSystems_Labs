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
void Task1(void);


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

  nrk_init();
  nrk_time_set(0,0);

  nrk_register_drivers();
  nrk_create_taskset ();
  nrk_start();
  
  return 0;
}


void Task1()
{
uint16_t cnt;
int8_t i,fd,val;
uint16_t buf;
uint16_t buf_last = 0;
// uint16_t motions[] = {0,0,0,0,0,0,0,0,0,0};
uint16_t motions[] = {0,0};
uint16_t index = 0;
uint64_t bbuf;

  printf( "My node's address is %d\r\n",NODE_ADDR );

  printf( "Task1 PID=%d\r\n",nrk_get_pid());

  
  	// Open ADC device as read 
  	fd=nrk_open(FIREFLY_3_SENSOR_BASIC,READ);
  	if(fd==NRK_ERROR) nrk_kprintf(PSTR("Failed to open sensor driver\r\n"));
  cnt=0;
  while(1) {

	// Example of setting a sensor 
	val=nrk_set_status(fd,SENSOR_SELECT,HUMIDITY);
	val=nrk_read(fd,&bbuf,4); //percent humidity
	// Relative Humidity (%)  linear          |  1 |    0 |         0 |       100 |
	printf( "Percent humidity = %lu %s",bbuf, "%");

	val=nrk_set_status(fd,SENSOR_SELECT,TEMP2);
    // 	val=nrk_read(fd,&bbuf,4); //degrees F
    // 	 temp         | Temperature (analog sensor, F)  | linear          |           0.07506 |   32 |        40 |        90 |
    // | digital_temp | Temperature (digital sensor, F) | linear          |              0.18 |   32 |        40 |        90
	printf( " Temperature = %lu%s ",bbuf/10, "degrees");
	
	val=nrk_set_status(fd,SENSOR_SELECT,PRESS);
	val=nrk_read(fd,&bbuf,4);
	// pressure     | Barometric Pressure (in. Hg)    | linear          | 0.000295780903035 |    0 |        28 |        31 |
	printf( " Barometric Pressure (in. Hg)= %lu ",0.000295780903035*bbuf);
	
	val=nrk_set_status(fd,SENSOR_SELECT,LIGHT);
	val=nrk_read(fd,&buf,2);
	// light        | Incident Illumination (lumens)  | linear          |                -1 | 1024 |         0 |      1024 
	printf( " Incident Illumination (lumens) =%d",-1*buf+1024);
	
	uint16_t sum = 0;
    int size = sizeof(motions)/sizeof(motions[0]);
	for ( i = 0; i < size; i++ )
     {
      sum = sum + motions[i];
     }

	uint16_t avg = sum/size;

    nrk_led_clr(RED_LED);
    nrk_led_clr(GREEN_LED);

    printf( " Motion based on ligth change :\r\n");
	if( abs(buf - avg) >10)
	//if( abs(buf - buf_last) >2)
		{ printf("%s","Motion Detected " );
	      nrk_led_toggle(RED_LED);
	  	}
	else 
	{
		printf("%s","Motion Not Detected " );
		nrk_led_toggle(GREEN_LED);
	}

	// if(cnt<10)
	// {
		motions[index] = buf;
	// }
	buf_last = buf; 

	// printf( " Avg light=%d",avg);

	// val=nrk_set_status(fd,SENSOR_SELECT,TEMP);
	// val=nrk_read(fd,&buf,2); //degrees F
	// printf( " Temp in degrees=%03d",buf/10);

// 	| acc_x        | Acceleration - X (ft / sec^2)   | linear          |                 1 |    0 |         0 |        50 |
// | acc_y        | Acceleration - Y (ft / sec^2)   | linear          |                 1 |    0 |         0 |        50 |
// | acc_z        | Acceleration - Z (ft / sec^2)   | linear          |                 1 |    0 |         0 |        50
	val=nrk_set_status(fd,SENSOR_SELECT,ACC_X);
	val=nrk_read(fd,&buf,2);
	printf( " Acceleration - X (ft / sec^2) =%d",buf);
	val=nrk_set_status(fd,SENSOR_SELECT,ACC_Y);
	val=nrk_read(fd,&buf,2);
	printf( " Acceleration - Y (ft / sec^2) =%d",buf);
	val=nrk_set_status(fd,SENSOR_SELECT,ACC_Z);
	
	val=nrk_read(fd,&buf,2); //motion detected/no motion detected
	printf( " Acceleration - Z (ft / sec^2) =%d\r\n",buf);

 //  	val=nrk_set_status(fd,SENSOR_SELECT,AUDIO_P2P);
	// nrk_spin_wait_us(60000);
	// val=nrk_read(fd,&buf,2);
	// printf( " Audio=%d\r\n",buf);
	//nrk_close(fd);
	nrk_wait_until_next_period();
	cnt++;

	index++;
	if(index == size)
		index =0;
	}
}


void
nrk_create_taskset()
{
  TaskOne.task = Task1;
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 100*NANOS_PER_MS; //*NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs =  200*NANOS_PER_MS;
  TaskOne.offset.secs = 0;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);

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


