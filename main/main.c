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

static void test()
{
   // const int bytes_read_i2c = 6;                                                                        // Bytes Read at ONE GO from DS3231
   // const int bytes_read_uart = 1;

    uint8_t *receive_data_uart = (uint8_t *) malloc(sizeof(uint8_t) * 8);

    int choice = 0;
    uart_config();
    configure_I2C();
    int row= 0, col = 0 ,colum = 0;
    
      
   while(1)
    {
    
        uart_write_bytes(UART_NUM_1, opt_msg,strlen(opt_msg));                                           // Sending the Option selection message through UART 
        uart_flush(UART_NUM_1);
        
        int response = uart_read_bytes(UART_NUM_1,receive_data_uart,1 ,portMAX_DELAY);                    // Receiving 1 byte of character  
        //char *delimiter = ",";



        if( response >0)
        {
            choice = receive_data_uart[0] - '0';

            switch(choice)
            {
                case 1 :
                {
                      // i2c_cmd_handle_t handle = SSD_stat_comm();                                           // this gives use the Handle for I2C , furhter function calls would be based upon this ( as they would be rquiring the handle ) 
                       SSD_initialize();
                      // i2c_cmd_link_delete(handle);                                                                 // Putting out the DIsplay ON command 

                }break;
                
                case 2:{

                       //i2c_cmd_handle_t handle = SSD_stat_comm();                                           // this gives use the Handle for I2C , furhter function calls would be based upon this ( as they would be rquiring the handle ) 
                       SSD_command(DISPLAY_ON);
                       //i2c_cmd_link_delete(handle);    
                            

                         }break;
                case 3: {

                        SSD_command(DISPLAY_OFF);


                        }break;

                case 4:{

                        // Initialising the matrix with zeroes 
                        int row= 0 , col = 0; 
                        for ( row = 0 ; row <= 7 ; row ++)
                        {
                            for( col =0 ; col < 128 ; col ++)
                            {
                                mat[row][col] = 255 ; 

                            }

                        }

                        SSD_push_mat();

                         }break;
                case 5:{

                        // int row= 0 , col = 0; 
                        // for ( row = 0 ; row <= 7 ; row ++)
                        // {
                        //     for( col =0 ; col < 128 ; col ++)
                        //     {
                        //         mat[row][col] = 0 ; 

                        //     }
                        // }
                        int index =0;
                        for (int index =0 ;index < 8; index ++)
                        mat[0][index] =font_3[index + 344];

                        SSD_push_mat();

                }break;
                case 6 :{

                            uint8_t *data = (uint8_t *) malloc(sizeof(uint8_t));
                            uart_read_bytes(UART_NUM_1,data,1,portMAX_DELAY);
                            data[0] = data[0] - '0';
                            SSD_push_data(*data);

                }break;
                case 7:{
                            
                            
                            uint8_t *data = (uint8_t *) malloc(sizeof(uint8_t));
                            uart_read_bytes(UART_NUM_1,data,1,portMAX_DELAY);
                            
                            // Chracter K is locatex at pos 424 to 431 , &  currently using 8*8 fonts then we will be displaying those
                            int font_index = 0; 
                            font_index = (data[0] - 32) * 8;
                            int col_bound = col + 8 ;

                            if(col_bound > 128)
                            {
                                col = 0 ; 
                                col_bound = 8;
                                row ++ ;
                            }

                            for(colum = col ; colum < col_bound;colum ++)
                            {
                                mat[row][colum] = font_3[font_index];
                                font_index ++ ;

                            }

                            col = colum + 1;            // Plus 1 is for adding some space b/w two characters . 
                            SSD_push_mat();

                }break;
                case 8:
                {
                            // For making the GDRAM point to the starting location 
                            set_GDRAM();



                }break;
                case 9:
                {

                        SSD_INVERT();
                        

                }break;
                case 10:
                {

                        SSD_NORMAL();

                }break;
                default :{

                          draw_message("Watch Dog  ",5,5,strlen("Watch Dog  "));
                          draw_message("Work in Progress...",5,15,strlen("Work in Progress..."));
                          draw_message("Launching Soon..",5,25,strlen("Launching Soon"));
                          draw_message("Hackster.io",5,35,strlen("Hackster.io"));
                          
                          
                          SSD_push_mat();
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

/* 

Further Updates :


1. Getting the value of Status Register or all register & display it via UART 
2. Making it more User friendly  ( improvement s in current menu ) 
3. Error Reporting : Debugging Errors ( wheere each errors like : Wire not connected , Pull- Ups missising should be given by the system ) 
4.Updating the current variable names ( especially of the Acknowledgment which is curently called with the help of 0 or 1 ) 



*/