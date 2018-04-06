// Program to create a touch based interface for Tuning ON / OFF / Blinknig of LED 

/*

    To acheive the above , there will be three GPIOs used for the purpose , Below are the steps the above mentioned : 
    1. Configure the three GPIOs for TOUCH & setting their threshold value
    2. Auto Start of Measuring of the Capacitance of the GPIO 
    3. Set Up Interrupt & writing out the INterrupt handler 
    4. IN the interrupt , we will be examining which GPIO was touched & based upon 
       that one GPIO would be used for Turning ON the LED , Other for Tuning OFF the LED 
       & last one to initiate the task from the queue for the Blinking of the LED .

       One more thing that needs to be kept in mind while designging a touch system is that thickness of the insulating material should be kept 
       less as it effects the sensitivity . 

    Program Working Absolutely fine 
    Also it is observed that if try to resume task that is already running , then no errors are reported & same case holds true for 
    Task suspension . 
    Note : Status Read function should be read with some delay , as the reading process byu internal timer is slow ( by deafult or what ) 
            If reading @ full speed , then it gives false values . 


*/


#include"driver/gpio.h"
#include"stdlib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_log.h"
#include"driver/uart.h"
#include"esp_err.h"
#include"freertos/task.h"


#define LED 17
int a = 0 ;
#define THRESHOLD 600                               // The threshold value has been determined through Polling approach 
#define HIGH 1
#define LOW 0


// The value of the Pin No correspoinding to Touch_PAD No has been taken from Library itself , here these are used for providing user friendly names ( reffereable through Pin No )
#define TOUCH_PIN12 TOUCH_PAD_NUM5
#define TOUCH_PIN13 TOUCH_PAD_NUM4
#define TOUCH_PIN27 TOUCH_PAD_NUM7
#define TOUCH_PIN33 TOUCH_PAD_NUM8


#define CHECK_PIN12 (1<<TOUCH_PAD_NUM5)
#define CHECK_PIN13 (1<<TOUCH_PAD_NUM4)
#define CHECK_PIN27 (1<<TOUCH_PAD_NUM7)
#define CHECK_PIN33 (1<<TOUCH_PAD_NUM8)


int compare_arr(uint32_t *arr1 , uint32_t *arr2, int len);    // function for comparing two arrays 

/* Here are the following pins used for Touch : 

TOUCH_PAD_NUM5 --> GPIO12 ( Used for Turning ON the LED ) 
TOUCH_PAD_NUM4 --> GPIO13 ( Used for Turning OFF the LED )
TOCUH_PAD_NUM7 --> GPIO27 (Used for Initiating a task to TURN ON & TURN OFF the LED ) 
TOUCH_PAD_NUM8 --> GPIO 33

*/

// Interrupt is a combined interrupt for all the GPIOs ,so we will be chekcing out the GPIO which was touched inside the interrupt & taking the action based upon it . 


TaskHandle_t task_handle;                           // Task Handles are used to refer the task from another task or Interrupt , Example Case may be of waking up a task from interrupt .



void touch_handler(void *args)
{

      vTaskResume(task_handle);          // vTaskResume is working absolutely fine , even though being called from Interrupt 

}
 


void configure_gpio(int arr[],int len_arr)
{
    ESP_ERROR_CHECK(touch_pad_init());
    // ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM7,THRESHOLD));
    // ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM5,THRESHOLD)); // Different Touch Pad No corrspond to Different GPIOs & also threshold value need to be set for interrupt 
    // ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM4,THRESHOLD));
    // ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM8,THRESHOLD));

    for(int index =0; index < len_arr ;index ++)
    {

        ESP_ERROR_CHECK(touch_pad_config(arr[index],THRESHOLD));

    }



    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER)); // THis will make the touch measurement start by FSM automatically with the help of hardware timer 

    // for giving out the signal to FSM Mode to start touch measurement , then call touch_pad_sw_start()
 
    ESP_ERROR_CHECK(touch_pad_set_trigger_mode(TOUCH_TRIGGER_BELOW));    // Trigger Mode , decideds whether interrupt would occur if the Read value is larger or smaller than threshold value
    ESP_ERROR_CHECK(touch_pad_isr_register(touch_handler,NULL));        // Name of the FUnction to call when the interrupt is triggered ( Interrupt Handler)
    ESP_ERROR_CHECK(touch_pad_intr_enable());                           // Enabling the Interrupt ( Interrupt will be triggered based upon threshold value set above )


    // Configuration for LED 

    gpio_config_t config = {.pin_bit_mask = 1ULL<<LED ,  .mode = GPIO_MODE_OUTPUT ,.intr_type = GPIO_INTR_DISABLE };
    ESP_ERROR_CHECK(gpio_config( &config ));


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
             index = 8;
        }


        vTaskDelay(75/portTICK_RATE_MS);            // To accomodate the filter period , if reading are taken during the filter period ,then it gives 0's 

    if( index == 8) 
        {


            index = 0 ;
            current = 0; 
          printf("\nValue of the array : %d,%d,%d,%d,%d,%d,%d,%d",touched_pad[0],touched_pad[1],touched_pad[2],touched_pad[3],touched_pad[4],touched_pad[5],touched_pad[6],touched_pad[7]);
           if(compare_arr(right_slide,touched_pad,8))
           printf("\nRight SLide Detected");
            if(compare_arr(left_slide,touched_pad,8))
           printf("\nLEFT SLide Detected");
            vTaskDelay(100/portTICK_RATE_MS);  // Giving a one second delay , so as to prevent the task from re-running after one slide has occured 
            ESP_ERROR_CHECK(touch_pad_intr_enable());       // Now enabling back the interrupts
             vTaskSuspend(NULL);                 // WHen the 7 readings are taken then we suspend the task 


        }
     


    }

}

int compare_arr(uint32_t *arr1 , uint32_t *arr2, int len)
{

    for ( int index = 0 ; index < len ; index ++)
    {

        if(arr1[index] != arr2[index])
        return 0;

    }

return 1 ;

}


void app_main()
{
int arr[4] = {TOUCH_PIN12,TOUCH_PIN13,TOUCH_PIN27,TOUCH_PIN33};
xTaskCreate(slide_task,"Blink LED",2048,arr,5,&task_handle);


}