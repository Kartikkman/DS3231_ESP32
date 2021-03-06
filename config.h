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



// UART Constants

#define TX 12
#define RX 13 
#define RTS 14
#define CTS 15
#define UART_BUF_SIZE 1024
#define BAUD_RATE 115200


//I2C Constants 

#define CLK_SPEED 100000
#define SDA 4
#define SCL 5 
#define SLAVE_ADDR 104
#define BUFFER_SIZE 1024




void uart_config() {

    uart_config_t config ={ .baud_rate = BAUD_RATE , .data_bits = UART_DATA_8_BITS , .parity = UART_PARITY_DISABLE ,.stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
    uart_param_config(UART_NUM_1 ,&config);         //first step ( Configuration of UART)
    uart_set_pin(UART_NUM_1, TX, RX, RTS , CTS);    // Second Step ( Allocating Pins)     uart_driver_install(UART_NUM_1,UART_BUF_SIZE,0,0 ,NULL,0);
    uart_driver_install(UART_NUM_1,UART_BUF_SIZE ,UART_BUF_SIZE, 0, NULL, 0);

}

void configure_I2C()
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t config = { .mode = I2C_MODE_MASTER , .sda_io_num = SDA, .sda_pullup_en = 1, .scl_io_num = SCL, .scl_pullup_en = 1 , .master ={.clk_speed = CLK_SPEED } };
    i2c_param_config(I2C_NUM_0 , &config);
    i2c_driver_install(I2C_NUM_0 , I2C_MODE_MASTER, 0, 0, 0);
    // printf("Driver has been Installed ");
}

char * bcd_char(uint8_t *arr,int length_arr)
{

     char *ch = (char *) malloc(sizeof(char) *length_arr * 2);                     // Fetching out the no's separetly
     char first_digit_char, second_digit_char;
     int index = 0;

    for(int loop = 0; loop < length_arr;loop ++)
    {

        int first_digit = arr[loop] & (0b00001111);
        int second_digit = (arr[loop] & (0b11110000) )>>4;

        first_digit_char = first_digit + '0';
        second_digit_char = second_digit + '0'  ;                                      // Converting the no into char & then forming a string


        ch[index] = second_digit_char;
        ch[index + 1] = first_digit_char;
        index = index + 2 ;                             
    }

    return ch;

}

uint8_t *convert_bcd(uint8_t *arr)
{
    // Data received in the above array will contain characters , whihc will be fisrt converted into the integer ( no - '0' ) , then BCD no will be formed

    uint8_t *bcd_no = (uint8_t *) malloc(sizeof(uint8_t));
    arr[0] = arr[0] - '0';
    arr[1] = arr[1] - '0';
    *bcd_no = ( arr[0]<<4 ) + arr[1];

    return bcd_no;


}

char * DS3231_get_time()
{

        const int bytes_read_i2c = 6;                                                                        // Bytes Read at ONE GO from DS3231
    const int bytes_read_uart = 1;
    uint8_t start_addr_read = 0;

    uint8_t *receive_data_uart = (uint8_t *) malloc(sizeof(uint8_t) * 8);
    uint8_t *receive_data_i2c = (uint8_t *) malloc(sizeof(uint8_t) * 8);

        char *msg = "\r\nChoice received";
        uart_write_bytes(UART_NUM_1,msg,strlen(msg));
        
            // using Repeated Start Condtition , for the simulation purposes , as well as BCD Testing 

        i2c_cmd_handle_t handle =  i2c_cmd_link_create();                                            // Cerating Handle for I2C Communication
        ESP_ERROR_CHECK(i2c_master_start(handle));                                                   // Start Bit Transmit 
        ESP_ERROR_CHECK(i2c_master_write_byte(handle ,(SLAVE_ADDR << 1) | I2C_MASTER_WRITE,0));      // Sending the Address of the Slave along with the Write Condition
        ESP_ERROR_CHECK(i2c_master_write(handle,&start_addr_read,1,1));                              // Address of the REgister inside the DS3231 ,where we will begin Reading from
        //ESP_ERROR_CHECK(i2c_master_stop(handle));
        ESP_ERROR_CHECK(i2c_master_start(handle));                                                   // Repeated Start Condition 
        ESP_ERROR_CHECK(i2c_master_write_byte(handle ,(SLAVE_ADDR << 1) | I2C_MASTER_READ,0));      // Sending the Address of the Slave along with the Read Condition
        ESP_ERROR_CHECK(i2c_master_read(handle,receive_data_i2c,bytes_read_i2c,0));                  // Reaciving Bytes along with Acknowledgement Sending 
        ESP_ERROR_CHECK(i2c_master_read_byte(handle,(receive_data_i2c + bytes_read_i2c),1));             // Receiving the Last Byte without the Acknowledgement                   
        i2c_master_stop(handle);                                                                    // Sending the stop condition 
        
        int error_code =  i2c_master_cmd_begin(I2C_NUM_0,handle,portMAX_DELAY);                     // Actual Transmission of above commands begin
        i2c_cmd_link_delete(handle);                                                                // Deleting the driver acquired

        if( error_code != 0)
         {   
            error_code +='0';    
            char *response = (char *)"\r\nError received is  :";
            uart_write_bytes(UART_NUM_1,(char *)response,strlen((char*)response));
            uart_write_bytes(UART_NUM_1,(char *)(&error_code),1);
            return 'E';
       
        }
    else{

            char *result = bcd_char(receive_data_i2c,7);                                                   // Converting the BCD received data into the charact"
            free(receive_data_uart);
            free(receive_data_i2c);
            return result;
         }

}

void DS3231_update_time(uint8_t *addr_register_write,uint8_t *data_register_write)
{


        i2c_cmd_handle_t handle =  i2c_cmd_link_create();                                            // Creating new Link 
        ESP_ERROR_CHECK(i2c_master_start(handle));                                                   // Start Bit Transmit 
        ESP_ERROR_CHECK(i2c_master_write_byte(handle ,(SLAVE_ADDR << 1) | I2C_MASTER_WRITE,0));      // Sending the Address of the Slave along with the Write Condition
        ESP_ERROR_CHECK(i2c_master_write(handle,addr_register_write,1,0));                          // Writing out the Address of the register on which Data has to be written 
        ESP_ERROR_CHECK(i2c_master_write(handle,data_register_write,1,1));                          // Writing out the Data on the above specified register 
        ESP_ERROR_CHECK(i2c_master_stop(handle));                                                    // Sending the Stop condition 
        int error_code =  i2c_master_cmd_begin(I2C_NUM_0,handle,portMAX_DELAY);                      // Actual Transmission of above commands begin 
        i2c_cmd_link_delete(handle);                                                                 // Deleting the Handle 
                                                                                

        if(error_code !=0)
        {
                error_code +='0'; 
               char *response = "\r\nError received is  :";
                uart_write_bytes(UART_NUM_1,response,strlen(response));
                uart_write_bytes(UART_NUM_1,(char *)(&error_code),1);

        }       

  //      free(addr_register_write);
        free(data_register_write);            

}