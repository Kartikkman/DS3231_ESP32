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
TaskHandle_t task_handle;                           // Task Handles are used to refer the task from another task or Interrupt , Example Case may be of waking up a task from interrupt .


typedef struct
{
	int w_px;
	int glyph_index;

}lv_font_glyph_dsc_t;



typedef struct
{
	int unicode_first;
	int unicode_last;
	int h_px;
	const uint8_t *glyph_bitmap;
	const lv_font_glyph_dsc_t *glyph_dsc;
	int unicode_list;


}lv_font_t;



// Aknowledgement
#define ACK_EN 0x00                     // Sending the Acknowledgement after receiving the byte 
#define ACK_DIS 0x1                     // For NOt sending the Acknowledgement after receivign the Byte 

// UART Constants

#define TX 21
#define RX 22 
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
