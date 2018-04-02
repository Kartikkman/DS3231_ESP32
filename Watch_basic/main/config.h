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
#include"fonts.h"

// Aknowledgement
#define ACK_EN 0x00                     // Sending the Acknowledgement after receiving the byte 
#define ACK_DIS 0x1                     // For NOt sending the Acknowledgement after receivign the Byte 

// UART Constants

#define TX 12
#define RX 13 
#define RTS 14
#define CTS 15
#define UART_BUF_SIZE 1024
#define BAUD_RATE 115200


//I2C Constants 

#define CLK_SPEED 390000                // Running the SSD1306 Display @ max speed 
#define SDA 4                           
#define SCL 5 
#define SLAVE_ADDR 104
#define BUFFER_SIZE 1024
#define SSD1306_ADDR 0x3C

// OLED Display ( 128 * 64 ) Commands , Commands of Page 28 of Data sheet

#define ENTIRE_DISPLAY 0xA5
#define RAM_DISPLAY 0xA4

#define NORMAL_DISPLAY 0xA6
#define INVERT_DISPLAY 0xA7

#define DISPLAY_ON 0xAF
#define DISPLAY_OFF 0xAE


#define PUMP 0x8D
#define CHARGE 0x14

#define PRECHARGE 0xD9


#define MUX_RAT 0xA8
#define MUX 0x3F

#define OSC_FR 0xD5
#define OSC 0x80

#define CONTROL_BYTE 0b10000000          // Control Byte for non-continuous , COmmand Mode
#define CONTROL_BYTE1 0b00000000         // COntrol Byte for continuous , Command Mode

#define CONTROL_BYTE2 0b1100000000      // Control Byte for non-continuous , Data Mode
#define CONTROL_BYTE3 0x40      // Control Byte for continuous , Data Mode


#define OFFSET 0xD3
#define START_LINE 0x40

#define CONTRAST_CONTROL 0x81

#define ADD_MODE 0x20

#define HORIZ_ADD 0b00000000
#define PAGE_ADD 0b0000001
#define VERTI_ADD 0b000010

#define S_COL_ADD 0x21
#define S_PAGE_ADD 0x22

#define COL_START 0
#define PAGE_START 0

#define COL_END 127
#define PAGE_END 7

#define NOP 0xE3

uint8_t mat[8][128] ;


void uart_config() {

    uart_config_t config ={ .baud_rate = BAUD_RATE , .data_bits = UART_DATA_8_BITS , .parity = UART_PARITY_DISABLE ,.stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
    uart_param_config(UART_NUM_1 ,&config);         //first step ( Configuration of UART)
    uart_set_pin(UART_NUM_1, TX, RX, RTS , CTS);    // Second Step ( Allocating Pins)     uart_driver_install(UART_NUM_1,UART_BUF_SIZE,0,0 ,NULL,0);
    uart_driver_install(UART_NUM_1,UART_BUF_SIZE ,UART_BUF_SIZE, 0, NULL, 0);

}

void configure_I2C()
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t config = { .mode = I2C_MODE_MASTER , .sda_io_num = SDA, .sda_pullup_en = 0, .scl_io_num = SCL, .scl_pullup_en = 0 , .master ={.clk_speed = CLK_SPEED } };
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
    uint8_t start_addr_read = 0;

   // uint8_t *receive_data_uart = (uint8_t *) malloc(sizeof(uint8_t) * 8);
    uint8_t *receive_data_i2c = (uint8_t *) malloc(sizeof(uint8_t) * 8);

        // char *msg = "\r\nChoice received";
        // uart_write_bytes(UART_NUM_1,msg,strlen(msg));
        
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

void SSD_command(uint8_t command)
{
          i2c_cmd_handle_t handle = i2c_cmd_link_create();                                           
        ESP_ERROR_CHECK(i2c_master_start(handle));                                                    // Placing the Start Bit on the Line ( SDA --> HIGH to LOW when  SCL ---> HIGH )
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,(SSD1306_ADDR<<1)|I2C_MASTER_WRITE,ACK_EN));    // Placing the Adress on the Line 
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,CONTROL_BYTE,ACK_EN));                          // Placing out the Control Byte on I2C ( for Non Continous transmission) of COmmand 
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,command,ACK_EN));                                                // Placing the Actual Command 
        //ESP_ERROR_CHECK(i2c_master_write_byte(handle,NOP,ACK_EN));                                                   // Placing the No-Operation command on the bus , ( it is done so as the Master Holds the bus if there is 
        ESP_ERROR_CHECK(i2c_master_stop(handle));  
        int error_code = i2c_master_cmd_begin(I2C_NUM_0,handle,portMAX_DELAY);                                                                                           // No data avaialble or one needs to send the STOP command ), that could have lead to Not execution of 
        i2c_cmd_link_delete(handle);                                                                                             // previous command until there is some other data placed on the bus        

        if(  error_code != 0)
        {
            printf("Error occured is : %d",error_code);
        }

}

void SSD_commands(uint8_t *commands , uint8_t no_commands)
{
          i2c_cmd_handle_t handle = i2c_cmd_link_create();                                           
        ESP_ERROR_CHECK(i2c_master_start(handle));                                                    // Placing the Start Bit on the Line ( SDA --> HIGH to LOW when  SCL ---> HIGH )
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,(SSD1306_ADDR<<1)|I2C_MASTER_WRITE,ACK_EN));    // Placing the Adress on the Line 
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,CONTROL_BYTE1,ACK_EN));                          // Placing out the Control Byte on I2C ( for Continous transmission) of COommand 
        ESP_ERROR_CHECK(i2c_master_write(handle,commands,no_commands,ACK_EN));                          // Placing the Actual Command 
        //ESP_ERROR_CHECK(i2c_master_write_byte(handle,NOP,ACK_EN));                                   // Placing the No-Operation command on the bus , ( it is done so as the Master Holds the bus if there is 
        ESP_ERROR_CHECK(i2c_master_stop(handle));  
        int error_code = i2c_master_cmd_begin(I2C_NUM_0,handle, portMAX_DELAY);                        // No data avaialble or one needs to send the STOP command ), that could have lead to Not execution of 
        i2c_cmd_link_delete(handle);                                                                   // previous command until there is some other data placed on the bus        

        if(  error_code != 0)
        {
            printf("Error occured is : %d",error_code);
        }

}

void set_GDRAM()
{
    uint8_t commands[6] = {S_COL_ADD,0,127,S_PAGE_ADD,0,7};

    SSD_commands(commands,6);


}


void SSD_initialize()
{

          i2c_cmd_handle_t handle = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(handle));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,(SSD1306_ADDR<<1)|I2C_MASTER_WRITE,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,CONTROL_BYTE1,ACK_EN));                        // Control Byte indicating that , there will be transmission of Commands befor the stop of this comm.

        // Now writing out the commands 
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,DISPLAY_OFF,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,MUX_RAT,ACK_EN));
       
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,MUX,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,OFFSET,ACK_EN));
        
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,0,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,START_LINE,ACK_EN));

        // COM Pin Configuration
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,0xDA,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,0x12,ACK_EN));

        ESP_ERROR_CHECK(i2c_master_write_byte(handle,OSC_FR,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,OSC,ACK_EN));

        ESP_ERROR_CHECK(i2c_master_write_byte(handle,PUMP,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,CHARGE,ACK_EN));

        ESP_ERROR_CHECK(i2c_master_write_byte(handle,ADD_MODE,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,HORIZ_ADD,ACK_EN)); 
        
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,0xA0,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,0xC0,ACK_EN));

        ESP_ERROR_CHECK(i2c_master_write_byte(handle,CONTRAST_CONTROL,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,0,ACK_EN));


        ESP_ERROR_CHECK(i2c_master_write_byte(handle,PRECHARGE,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,0xF1,ACK_EN));

        ESP_ERROR_CHECK(i2c_master_write_byte(handle,0xDB,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,0x49,ACK_EN));

        ESP_ERROR_CHECK(i2c_master_write_byte(handle,RAM_DISPLAY,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,NORMAL_DISPLAY,ACK_EN));

        ESP_ERROR_CHECK(i2c_master_write_byte(handle,0x2E,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,DISPLAY_ON,ACK_EN));

        ESP_ERROR_CHECK(i2c_master_stop(handle));
        
        int error_code = i2c_master_cmd_begin(I2C_NUM_0,handle,portMAX_DELAY);
        if(error_code != 0)
        {
            char *response = "Couldn't Send the Data";
            uart_write_bytes(UART_NUM_1,response,strlen(response));

        }

        i2c_cmd_link_delete(handle);

}

void SSD_push_data(uint8_t data)
{
        i2c_cmd_handle_t handle = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(handle));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,(SSD1306_ADDR<<1)|I2C_MASTER_WRITE,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,CONTROL_BYTE3,ACK_EN));                    // Control Byte , indicating that there will be continous stream of Data passing befire the stop condition arises 
        
        int index = 0 ; 
        for(index =0; index < 1024; index ++)
        {
            ESP_ERROR_CHECK(i2c_master_write_byte(handle,data,ACK_EN));

        }
        
        ESP_ERROR_CHECK(i2c_master_stop(handle));
       int error_code =  i2c_master_cmd_begin(I2C_NUM_0,handle,portMAX_DELAY);

        if(error_code != 0)
        {
           char *msg = "An Error Occured ";
            uart_write_bytes(UART_NUM_1,msg,strlen(msg));

        }

        i2c_cmd_link_delete(handle);

}

void SSD_push_mat()
{

       i2c_cmd_handle_t handle = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(handle));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,(SSD1306_ADDR<<1)|I2C_MASTER_WRITE,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write_byte(handle,CONTROL_BYTE3,ACK_EN));                    // Control Byte , indicating that there will be continous stream of Data passing befire the stop condition arises 
       
        // Loading our matrix onto the Dis

        ESP_ERROR_CHECK(i2c_master_write(handle,mat[0],128,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write(handle,mat[1],128,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write(handle,mat[2],128,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write(handle,mat[3],128,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write(handle,mat[4],128,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write(handle,mat[5],128,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write(handle,mat[6],128,ACK_EN));
        ESP_ERROR_CHECK(i2c_master_write(handle,mat[7],128,ACK_EN));

         ESP_ERROR_CHECK(i2c_master_stop(handle));

        int error_code =  i2c_master_cmd_begin(I2C_NUM_0,handle,portMAX_DELAY);

        if(error_code != 0)
        {
           char *msg = "An Error Occured ";
            uart_write_bytes(UART_NUM_1,msg,strlen(msg));

        }

        i2c_cmd_link_delete(handle);


}

void SSD_INVERT()
{

    SSD_command((uint8_t)INVERT_DISPLAY);

}

void SSD_NORMAL()
{

   SSD_command((uint8_t)NORMAL_DISPLAY);

}




void set_pixel(int x , int y , int val)
{

	int page = y / 8 ;
	int bit = y % 8 ;

	mat[page][x] |= (val<<bit);


}

void draw_char(char ch , int x , int y)
{
	int font_index = (ch -32 )* 8;
	int y_limit = y + 8 , x_limit = x + 8;

	for( int x_pos = x ; x_pos < x_limit ; x_pos ++)
	{
		int bit_pos = 0 ;

		for(int y_pos = y ; y_pos < y_limit ; y_pos ++)
		{
			int temp = (( font_3[font_index ] ) & ( 1 << bit_pos ));
			if(temp)
			set_pixel(x_pos , y_pos , 1);
			else
			set_pixel(x_pos , y_pos , 0);

			bit_pos ++ ;

		}

		font_index ++ ;

	}

}


void draw_message(char *str, int x ,int y , int len)
{

	for(int index =0 ; index < len ; index ++)
	{

		draw_char(str[index],(x + (index*8) )%128,y + x/128);

	}


}

void clear_mat()
{

    for(int page = 0 ; page < 8; page ++)
    {

        for(int col = 0; col < 128 ; col ++)
        {

            mat[page][col] = 0;


        }

    }


}
