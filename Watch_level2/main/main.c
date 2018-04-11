// A menu based UART program for the testing purposes of I2C communication with DS3231 

/*

Here is the procedure to be followed for impolementing the above details : 

As the program is based on UART protocol : 
1. Configuring the UART interface for the MCU 
2.Writing the lower level functions for sending & receiving of bytes  ( Already provided by the RTOS -- missing in the case of MSP430 )
3.Presenting the Menu form this UART interface 

 Here are the following pins used for Touch : 

TOUCH_PAD_NUM5 --> GPIO12 ( Used for Turning ON the LED ) 
TOUCH_PAD_NUM4 --> GPIO13 ( Used for Turning OFF the LED )
TOCUH_PAD_NUM7 --> GPIO27 (Used for Initiating a task to TURN ON & TURN OFF the LED ) 
TOUCH_PAD_NUM8 --> GPIO 33

*/

// Interrupt is a combined interrupt for all the GPIOs ,so we will be chekcing out the GPIO which was touched inside the interrupt & taking the action based upon it . 


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
#include"small_symbols.h"
#include"big_symbols.h"
#include"fonts.h"
#include"DS3231.h"
#include"SSD.h"
#include"touch.h"

void uart_config();
void send_data();
char * bcd_char(uint8_t  *arr,int length_arr);
uint8_t *convert_bcd(uint8_t *arr);
char * DS3231_get_time();
void DS3231_update_time(uint8_t *addr_register_write,uint8_t *data_register_write);
int time_display(int touch);
i2c_cmd_handle_t SSD_stat_comm();
void SSD_command(uint8_t command);
void SSD_initialize();
void SSD_push_mat();
void set_GDRAM();
int compare_arr(uint32_t *arr1 , uint32_t *arr2, int len);
void configure_gpio(int arr[],int len_arr);
void show_char(char ch , int x , int y , lv_font_t *symbols);
void draw_line(int s_x,int s_y , int e_x, int e_y );
void draw_lines(int s_x,int s_y , int e_x, int e_y , int ptx);
int sub_menu(int touch);

QueueHandle_t touch ;

enum Touch_State {SELECT , RIGHT_SLIDE , LEFT_SLIDE};
enum STATUS{TIME , MENU};
int status = TIME ;

int (*func_ptr[])(int touch) = {time_display,sub_menu};


char *opt_msg = "\r\n\r\n****SSD1306 Testing ****\r\n1.Initialize Display\r\n2.Turn ON the Display ??\r\n3.Turn  OFF the Display \r\n4.Send Data '1'\r\n5.Send Data '0'\r\n7.Send a char to Display \r\n";

int a = 0 ;
#define HIGH 1
#define LOW 0

int curr_pos = 1;
int prev_pos = 0 ;
int next_pos = 2 ;


// The value of the Pin No correspoinding to Touch_PAD No has been taken from Library itself , here these are used for providing user friendly names ( reffereable through Pin No )
#define TOUCH_PIN12 TOUCH_PAD_NUM5
#define TOUCH_PIN13 TOUCH_PAD_NUM4
#define TOUCH_PIN27 TOUCH_PAD_NUM7
#define TOUCH_PIN33 TOUCH_PAD_NUM8

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

    free(Time);
       vTaskDelay(300/portTICK_RATE_MS);

    }

}


void display(void *args)
{
    SSD_initialize();
    int * p = (int *) malloc(sizeof(int)) ;

    while(1)
    {


         //printf("\n\nTIme is :  %s",time_msg);
        // printf("\n\nTIme is :  %c%c",Time_msg[0],Time_msg[1]);
       //  printf("\nDate is : %14c",Date_msg);
        
        *p =-1 ;

        if (xQueueReceive(touch,p,1000/portTICK_RATE_MS) !=0)
        {
            printf("\nValue received inside the Queue is : %d",(*p));

        }

        (*func_ptr[status])(*p);
         SSD_push_mat();
  
   vTaskDelay(500/portTICK_RATE_MS);



    }

}

int time_display(int touch)
{

        draw_message("Time",64,10,strlen("Time"));
        
         //char *Date_msg = time.date[0] + time.date[1] + ' \\ ' + time.month[0] + time.month[1] + ' \\ ' + time.century[0] + time.century[1] + time.year[0] + time.year[1] +'\0' ;

        //char *sample_msg = "Kartik";

        if((touch == RIGHT_SLIDE ))
        {
            status = MENU;
            printf("Status has been changed ");
        }
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
        free(time_msg);
        free(date_msg);

        return 0;

}



   /* Task should be eliminated , uptill finger is taken off the Touch Pad 
         Further Optimizations : IncludeLeft SLide also  , if there is any zero in the middle dataset then Task should be exited 
         More Features : Selection Pin , ( TOuch Start at Pin is the Touch Left at that Pin ) 
                         Left Slide Detection as well . 
*/


void slide_task(int *args)
{
    
    configure_gpio((int *)args,4);
    // vTaskSuspend(NULL);                     // For suspending this own task 
    int prev = 0,current = 0,index = 0;
    uint32_t touched_pad[8] ;
    uint32_t check_pad[4] = {1<<(int)args[0] ,1<<(int)args[1],1<<(int)args[2],1<<(int)args[3]};
    uint32_t right_slide[2*4] = {check_pad[0],check_pad[0] | check_pad[1] , check_pad[1] , check_pad[1] | check_pad[2], check_pad[2],check_pad[2] | check_pad[3],check_pad[3] , 0};
    uint32_t left_slide[2*4] = {check_pad[3],check_pad[3] | check_pad[2],check_pad[2],check_pad[2] | check_pad[1],check_pad[1],check_pad[1] | check_pad[0],check_pad[0],0};
    int *status = 0;


    while(1)
    {

         ESP_ERROR_CHECK(touch_pad_intr_disable());
        current= touch_pad_get_status();
        touch_pad_clear_status();
          
        if (current != prev ) 
        {
            touched_pad[index] = current;
            prev = current ; 
            index ++ ;
            printf("Pin Touched :%d\n",current);

             if ( index != 7 && current == 0)
             {   
                   if ( index == 2 || index == 3)
                   {   printf("Select Detected");
                   status = SELECT;
                        xQueueSend(touch,&(status),1000/portTICK_RATE_MS);

                    }

                index = 8;
            
             }

        }


        vTaskDelay(75/portTICK_RATE_MS);                      // To accomodate the filter period , if reading are taken during the filter period ,then it gives 0's 

    if( index == 8) 
        {


            index = 0 ;
            current = 0; 
          printf("\nValue of the array : %d,%d,%d,%d,%d,%d,%d,%d",touched_pad[0],touched_pad[1],touched_pad[2],touched_pad[3],touched_pad[4],touched_pad[5],touched_pad[6],touched_pad[7]);
           if(compare_arr(right_slide,touched_pad,8))
           {
               status = RIGHT_SLIDE;
               xQueueSend(touch,&(status),1000/portTICK_RATE_MS);
           }
            if(compare_arr(left_slide,touched_pad,8))
             {  
                   status = LEFT_SLIDE ;
                   xQueueSend(touch,&(status),1000/portTICK_RATE_MS);
                   printf("\nLEFT SLide Detected");
             }

            vTaskDelay(100/portTICK_RATE_MS);               // Giving a one second delay , so as to prevent the task from re-running after one slide has occured 
           
            ESP_ERROR_CHECK(touch_pad_intr_enable());       // Now enabling back the interrupts
             vTaskSuspend(NULL);                          // WHen the 7 readings are taken then we suspend the task 


        }
     


    }

}


int  sub_menu(int touch)
{
	// C : Messages , F : Games , N : Heart Rate Sensor , U : Calls , V : Settings , i : Bluetooth , m : Whatsapp , u : Flashlight , W : Left Arrow for Pointing
		char list_seq[] = {'F','C','m','N','i','u','V'};

			if ( touch == RIGHT_SLIDE)
			{
				curr_pos ++;
				prev_pos ++;
				next_pos ++;

			}else if (touch == LEFT_SLIDE)
			{
				curr_pos --;
				prev_pos --;
				next_pos --;
			}else if ( touch == SELECT)
			{
				status = TIME ;
			}

			clear_mat();
			show_char(list_seq[prev_pos], 95 , 5,&small_symbols);
			show_char(list_seq[curr_pos], 95 , 25,&small_symbols);
			show_char(list_seq[next_pos], 95 , 45,&small_symbols);
			show_char('W',120,26,&small_symbols);
			show_char(list_seq[curr_pos], 10, 14,&big_symbols);
			draw_lines(70,0,85,60,3);

    return 0 ;
}




 void app_main(void)
{
    int arr[4] = {TOUCH_PIN12,TOUCH_PIN13,TOUCH_PIN27,TOUCH_PIN33};
  xTaskCreate(update_time,"Receiving Time from DS3231",2048,NULL,10,NULL);
 xTaskCreate(display,"Displaying Task",2048,NULL,8,NULL);
  xTaskCreate(slide_task,"Touch Interface",2048,arr,5,&task_handle);
  //symbol_task();
 // xTaskCreate(symbol_task,"Symbol Testing",2048,NULL,5,NULL);
 touch = xQueueCreate(5,sizeof(int));  

}



/*

Already Tested : 
void symbol_task()
{
    SSD_initialize();
    uart_config();
    uint8_t * ch = ( uint8_t *) malloc(sizeof(uint8_t));
    while(1)
    {

     uart_write_bytes(UART_NUM_1,"\r\nEnter the Charcter you want to display on screen ",strlen("Enter the Charcter you want to display on screen "));
     uart_read_bytes(UART_NUM_1,ch,1,portMAX_DELAY);
     clear_mat();
     create_symbol(*ch,10,10);
     SSD_push_mat();


    }

}

////////////////////


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

Error Reporting to be Priority 1 ( as the Segmentation fault error would be occuring & more errors to occur ) , so error reporting is must for designing a 
robust system . 


*/


/*

 For the purpose of the state machine design : 
 Follow the Game State , Enum video 
 & also refer to the follwing link for more clear explanation : https://stackoverflow.com/questions/1371460/state-machines-tutorials

 Things Learned so far : 
 1. There needs to be two array : One for Storing the current State
                                  Another for storing the pointer to the function ( which needs to be executed for that state ) 
2. There may be furhter support of the structure in the current system , to support the backward tracking of the states ( if reqd .)

                    
A good tutorial , relating the State Machine Design with the reference to the micro controllers ( GPIO INterface )
http://blog.mbedded.ninja/programming/general/control-methodology/a-function-pointer-based-state-machine

Learned from the above Page : 

1.THere is structuer made for the State , Event , Function pointer . 
    Based upon the matching of the State , Event . ( Function pointer is called ) 
    For Example : Current State ( LED OFF , Event : Switch is Pressed for OFF ) then the LED OFF function would be called 
                  Current State (LED ON , Event : Swithc is Pressed for LED ON ) then no function would be called . 


Size of the enum : https://stackoverflow.com/questions/366017/what-is-the-size-of-an-enum-in-c?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa



*/