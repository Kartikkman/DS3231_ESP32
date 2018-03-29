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



void uart_config();
void send_data();
char * bcd_char(uint8_t  *arr,int length_arr);
uint8_t *convert_bcd(uint8_t *arr);
char * DS3231_get_time();
void DS3231_update_time(uint8_t *addr_register_write,uint8_t *data_register_write);

char *opt_msg = "\r\n\r\n****DS3231 Testing ****\r\n1.What's Time ??\r\n2.Update REG @ DS3231\r\n\r\n";

static void test()
{
    const int bytes_read_i2c = 6;                                                                        // Bytes Read at ONE GO from DS3231
    const int bytes_read_uart = 1;

    uint8_t *receive_data_uart = (uint8_t *) malloc(sizeof(uint8_t) * 8);

    int choice = 0;
    uart_config();
    configure_I2C();
      
      
   while(1)
    {
    
        uart_write_bytes(UART_NUM_1, opt_msg,strlen(opt_msg));                                           // Sending the Option selection message through UART 
        uart_flush(UART_NUM_1);
        
        int response = uart_read_bytes(UART_NUM_1,receive_data_uart,1 ,portMAX_DELAY);                    // Receiving 1 byte of character  
        char *delimiter = ",";


        if( response >0)
        {
            choice = receive_data_uart[0] - '0';

            switch(choice)
            {
                case 1 :
                {

                        char * result = DS3231_get_time();                                                  // Getting a pointer to the Time received in string format 

                            for(int value =0 ; value <= bytes_read_i2c*2;value = value + 2) 
                            {

                                uart_write_bytes(UART_NUM_1,result+value ,2);   
                                uart_write_bytes(UART_NUM_1,delimiter,1);

                            } 

                            free(result);

                        
                }break;
                
                case 2:{

                            char *response = "\r\nSend Address where Data is to be written ";
                            uart_write_bytes(UART_NUM_1,response,strlen(response));                                       // Sending the feedback Message ( Response )                  
                            uart_read_bytes(UART_NUM_1,receive_data_uart,bytes_read_uart ,portMAX_DELAY);                 // Reading byte of data    
                            uint8_t addr_register_write = receive_data_uart[0];
                            addr_register_write = (addr_register_write) - '0';

                            response = "\r\n Send the Data to be written ";                                               // Receiving the Data from the UART , to be sent over I2C 
                             uart_write_bytes(UART_NUM_1,response,strlen(response));                                      // Sending the feedback Message ( Response )                  
                             uart_read_bytes(UART_NUM_1,receive_data_uart,bytes_read_uart + 1,portMAX_DELAY);                              // Reading byte of data    
                            uint8_t *data_register_write = receive_data_uart;

                            data_register_write = convert_bcd(data_register_write);

                            DS3231_update_time(&addr_register_write,data_register_write);

                         }break;


                default :{
                             char *response = "Invalid Option";
                             uart_write_bytes(UART_NUM_1,response,strlen(response));
                        }
                        break;
            }

 
    
        }
    }


 }




void app_main()
{
     
  xTaskCreate(test,"UART Task",2048,NULL,10,NULL);
}



/*
NOTES : 

1. In the ESP-32 microcontroller , Pull Resistor have been found to be sufficient for the comm. purposes .
2. ALso there is serious need to create & Delete the handle otherwise many times it has lead to the bugs.

DS3231 Reading fucnition is compelete & now for the writing function , proto is compelete 
, final yet needs to be written . 


*/