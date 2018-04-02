// A menu based UART program for the testing purposes of I2C communication with DS3231 

/*

Here is the procedure to be followed for impolementing the above details : 

As the program is based on UART protocol : 
1. Configuring the UART interface for the MCU 
2.Writing the lower level functions for sending & receiving of bytes  ( Already provided by the RTOS -- missing in the case of MSP430 )
3.Presenting the Menu form this UART interface 

*/

#include"driver/gpio.h"
#include"stdlib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_log.h"
#include"driver/uart.h"
#include"esp_err.h"
#include"string.h"
#include"driver/i2c.h"
#include"config.h"
#include"fonts.h"


void uart_config();
void send_data();
char * bcd_char(uint8_t  *arr,int length_arr);
uint8_t *convert_bcd(uint8_t *arr);
char * DS3231_get_time();
void DS3231_update_time(uint8_t *addr_register_write,uint8_t *data_register_write);
i2c_cmd_handle_t SSD_stat_comm();
void SSD_command(uint8_t command);
void SSD_initialize();
void SSD_push_mat();
void set_GDRAM();


char *opt_msg = "\r\n\r\n****SSD1306 Testing ****\r\n1.Initialize Display\r\n2.Turn ON the Display ??\r\n3.Turn  OFF the Display \r\n4.Send Data '1'\r\n5.Send Data '0'\r\n7.Send a char to Display \r\n";
 
 struct Time 
 {

     char seconds[2];
     char min[2];
     char hour[2];
     char day[2];
     char date[2];
     char month[2];
     char year[2];
     char century[2];

 };


struct Time time;



void update_time(void *args)
{

    uart_config();
    configure_I2C();

    while(1)
    {

        /* Now receiving the string containing the current time , here is the description about the contents of the string : 

            Every field is of 2 chracters  ( means Seconds , Min , Hours each will be represented by two-two characters each  )

            First 2 Characters : Seconds 
            Next 2 Characters : Minuets 
            Next 2 Characters : Hour 
            Next 2 Chracters : Day 
            Next 2 Characters : Date 
            Next 2 Characters : Month 
            Next 2 Characters : Year 



            Note : There needs to applied extreme level of error handling on the I2C side plus pointer handling side so that segmentation fault doesn't occur more over 
            any I2C based errors are also reported . 

        */

       char *Time = DS3231_get_time();

       time.seconds[0]  = Time[0];
       time.seconds[1] = Time[1];

       time.min[0]  = Time[2];
       time.min[1] = Time[3];

       time.hour[0]  = Time[4];
       time.hour[1] = Time[5];

       time.day[0]  = Time[6];
       time.day[1] = Time[7];

       time.date[0]  = Time[8];
       time.date[1] = Time[9];

       time.month[0] = Time[10];
       time.month[1] = Time[11];

       time.year[0] = Time[12];
       time.year[1] = Time[13];

       time.century[0] = '2';
       time.century[1] = '0';
       
     //  printf(" TIme is : %c%c",time.hour[0],time.hour[1]);


       vTaskDelay(300/portTICK_RATE_MS);

    }

}


void display(void *args)
{
    SSD_initialize();

    while(1)
    {

        draw_message("Time",64,10,strlen("Time"));
        
         //char *Date_msg = time.date[0] + time.date[1] + ' \\ ' + time.month[0] + time.month[1] + ' \\ ' + time.century[0] + time.century[1] + time.year[0] + time.year[1] +'\0' ;

        char *sample_msg = "Kartik";

        char *time_msg = (char *) malloc(sizeof(char) * 8);
        char *date_msg = (char *) malloc(sizeof(char) * 10);

        time_msg[0] = time.hour[0] ; 
        time_msg[1] = time.hour[1] ; 
        time_msg[2] = ':' ;
        time_msg[3] = time.min[0] ; 
        time_msg[4] = time.min[1] ; 
        time_msg[5] = ':' ;
        time_msg[6] = time.seconds[0] ; 
        time_msg[7] = time.seconds[1] ; 

        date_msg[0] = time.date[0];
        date_msg[1] = time.date[1];
        date_msg[2] = '/';
        date_msg[3] = time.month[0];
        date_msg[4] = time.month[1];
        date_msg[5] = '/';
        date_msg[6] = time.century[0];
        date_msg[7] = time.century[1];
        date_msg[8] = time.year[0];
        date_msg[9] = time.year[1];


         clear_mat();
         draw_message("Time & Date ",10,10,strlen("Time & Date "));
         draw_message(time_msg,30,34,8);
         draw_message(date_msg,30,54,10);

   //printf("\n\nTIme is :  %s",time_msg);
        // printf("\n\nTIme is :  %c%c",Time_msg[0],Time_msg[1]);
       //  printf("\nDate is : %14c",Date_msg);

        SSD_push_mat();
   vTaskDelay(500/portTICK_RATE_MS);



    }

}




void app_main()
{
  xTaskCreate(update_time,"Receiving Time from DS3231",2048,NULL,10,NULL);
  xTaskCreate(display,"Displaying Task",2048,NULL,8,NULL);
  

}



/*
NOTES : 

1. In the ESP-32 microcontroller , Pull Resistor have been found to be sufficient for the comm. purposes .
2. ALso there is serious need to create & Delete the handle otherwise many times it has lead to the bugs.

DS3231 Reading fucnition is compelete & now for the writing function , proto is compelete 
, final yet needs to be written . 


Further Updates :


1. Getting the value of Status Register or all register & display it via UART 
2. Making it more User friendly  ( improvement s in current menu ) 
3. Error Reporting : Debugging Errors ( wheere each errors like : Wire not connected , Pull- Ups missising should be given by the system ) 
4.Updating the current variable names ( especially of the Acknowledgment which is curently called with the help of 0 or 1 ) 

Eroor Reporting to be Priority 1 ( as the Segmentation fault error would be occuring & more errors to occur ) , so error reporting is must for designing a 
robust system . 


*/