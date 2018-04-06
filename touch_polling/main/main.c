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


void configure_gpio()
{
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_PAD_NUM5,200)); // Different Touch Pad No corrspond to Different GPIOs & also threshold value need to be set for interrupt 
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW)); // THis will make the touch measurement start by FSM when signal is given by Software 

    // for giving out the signal to FSM Mode to start touh measurement , then call touch_pad_sw_start()

}

void sense_touch()
{
    configure_gpio();

    while(1)
    {
        uint16_t *touch_value = (uint16_t *) malloc(sizeof(uint16_t));
        ESP_ERROR_CHECK(touch_pad_sw_start());                          // To initiate mesurement , call this function 
            ESP_ERROR_CHECK(touch_pad_read(TOUCH_PAD_NUM5,touch_value));
        printf("Touch Value :%u\n",(*touch_value));
       free(touch_value);
       vTaskDelay(500/portTICK_RATE_MS);

    }

}

    

void app_main()
{


xTaskCreate(sense_touch,"Detecting Touch",2048,NULL,5,NULL);

        
}


/*
NOTE : 

Touch Value as observed by the program is : 

FOr Non-Touch : Around 1400 ( mostly greater than but sometimes greater than 1200 )
For Touch : Around 200 ( most less than )


This program is usefull for determining the threshold value for touch interface , threshold value differes in each case & depend on the temperature , 
closeness to ground , ELectrode thickness , enivronmental noise . 

As our threshold value for Touch & Non-Touch have a large difference so we may skip applying of filter . 

There is also observation that , when our feet touches ground then capacitance value reduces to zero , also threshold value for not so good touches is varies 
over a range of 200- 600 . 


It is also noted that , when Capacitive Pads ( Large Copper Clad surfaces are used , then there is signinficant improvement in result ) . 

*/