// To implement touch based controller ,

/*
    To achieve the above , below are the steps to follow : 

    1. Initialization of Touch Driver 
    2. Configuration of GPIO for Touch 
    3. Taking Measurement ( Value of Capacitance ( theoritically either by Oscillation method or Discharge method ))
    4. Filtering of Measurement ( IIR filter ) (optional )
    5. Touch Detection Method   ( Optional )
    6. Interrupt Setting       ( Optional )
    7. Waking from Sleep Mode ( Optional )

*/




#include"driver/gpio.h"
#include"stdlib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_log.h"
#include"driver/uart.h"
#include"esp_err.h"


#define LED 13
int a = 0 ;
#define THRESHOLD 600                               // The threshold value has been determined through Polling approach 


void touch_handler(void *args)
{
    a = !a;
    // Interrupt Handler to Light UP the LED 
gpio_set_level(LED ,!a);
//printf("Interrupt Occured");



}



void configure_gpio()
{
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM5,THRESHOLD)); // Different Touch Pad No corrspond to Different GPIOs & also threshold value need to be set for interrupt 
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER)); // THis will make the touch measurement start by FSM automatically with the help of hardware timer 

    // for giving out the signal to FSM Mode to start touch measurement , then call touch_pad_sw_start()
 
    ESP_ERROR_CHECK(touch_pad_set_trigger_mode(TOUCH_TRIGGER_BELOW));    // Trigger Mode , decideds whether interrupt would occur if the Read value is larger or smaller than threshold value
    ESP_ERROR_CHECK(touch_pad_isr_register(touch_handler,NULL));        // Name of the FUnction to call when the interrupt is triggered ( Interrupt Handler)
    ESP_ERROR_CHECK(touch_pad_intr_enable());                           // Enabling the Interrupt ( Interrupt will be triggered based upon threshold value set above )



    // Configuration for LED 

     gpio_config_t config = {.pin_bit_mask = 1ULL<<LED ,  .mode = GPIO_MODE_OUTPUT ,.intr_type = GPIO_INTR_DISABLE };
    ESP_ERROR_CHECK(gpio_config( &config ));

}


void app_main()
{


       configure_gpio();            // Configuring the GPIO for TOuch , Setting Interrupt , Threshold for detection 

}


/*
NOTE : 

Touch Value as observed by the program is : 

FOr Non-Touch : Around 1400 ( mostly greater than but sometimes greater than 1200 )
For Touch : Around 200 ( most less than )


This program is usefull for determining the threshold value for touch interface , threshold value differes in each case & depend on the temperature , 
closeness to ground , ELectrode thickness , enivronmental noise . 

As our threshold value for TOuch & Non-Touch have a large difference so we may skip applying of filter . 

*/