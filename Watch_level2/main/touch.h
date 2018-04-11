#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_log.h"
#include"driver/uart.h"
#include"esp_err.h"
#include"string.h"
#include"driver/i2c.h"
#include"fonts.h"

#ifndef __TOUCH_H__
#define __TOUCH_H__


#define THRESHOLD 600                               // The threshold value has been determined through Polling approach 
#define LED 17


int compare_arr(uint32_t *arr1 , uint32_t *arr2, int len)
{

    for ( int index = 0 ; index < len ; index ++)
    {

        if(arr1[index] != arr2[index])
        return 0;

    }

return 1 ;

}

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

#endif