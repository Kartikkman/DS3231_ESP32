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
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"


typedef struct 
{
int category;                                                   // Categories are : Email , Call , Message , Missed Call , SMS , 
int no_notify ; 
char text[18];             // Maximum Length of the string inside the Alert is 18 

}Alert;

Alert trial[10];
char alert_categories[6][15] = {"SIMPLE_ALERT","EMAIL","NEWS","CALL","MISSED_CALL","SMS"};



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
void led_gui(int touch);

void configure_BLE();
void parse_alert(uint8_t *value , uint16_t len );
void sort_queue(uint8_t *value , uint16_t len ,uint8_t handle);
void print_alerts();
void alert_gui(int touch);


TaskHandle_t xHandle;
esp_gatt_if_t interface;
uint16_t conn_id;



QueueHandle_t touch ;

enum Touch_State {SELECT , RIGHT_SLIDE , LEFT_SLIDE};
enum STATUS{TIME , MENU,LED_GUI,ALERT_GUI};
int status = TIME ;

int (*func_ptr[])(int touch) = {time_display,sub_menu,led_gui,alert_gui};


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

        if (xQueueReceive(touch,p,10/portTICK_RATE_MS) !=0)
        {
            printf("\nValue received inside the Queue is : %d",(*p));

        }

        (*func_ptr[status])(*p);
         SSD_push_mat();
  
   vTaskDelay(40/portTICK_RATE_MS);



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
                        xQueueSend(touch,&(status),100/portTICK_RATE_MS);

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
               xQueueSend(touch,&(status),100/portTICK_RATE_MS);
           }
            if(compare_arr(left_slide,touched_pad,8))
             {  
                   status = LEFT_SLIDE ;
                   xQueueSend(touch,&(status),100/portTICK_RATE_MS);
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
				printf("Current Position is : %d",curr_pos);
                
                if ( curr_pos == 5)
                {
                    status = LED_GUI ; 
                    printf(" LED status has been selectd ");
                }
                 if ( curr_pos == 1)
                {
                    status = ALERT_GUI ;
                    printf("Alert Status Selected from Menu ");

                }


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

void led_gui(int touch)
{
    clear_mat();
    //At the v: LED ON , w : LED OFF  
    char ch[2] = {'w','v'};
    bool current_state = gpio_get_level(LED);
    show_char(ch[current_state],30,10,&big_symbols);

    if ( touch == SELECT)
    {
        current_state = (!current_state) ;
        gpio_set_level(LED,current_state);
        printf("LED status Now %d",current_state);
    }


}


void alert_gui(int touch)
{
    if( touch == SELECT)
    status = TIME;
    
    
    char *alert_msg = "ALERTS !!!";

    clear_mat();
    draw_message(alert_msg,20,10,strlen(alert_msg));

    char *alert_msgs[60] ; 
    // char ch = trial[0].no_notify + '0';
    // draw_message(&ch,20,19,1);


       for(int alert_no = 1 ; alert_no < 4; alert_no ++)
    {

        draw_message(&alert_categories[alert_no],20,( alert_no -1 )*8 + 30,strlen(alert_categories[alert_no]));
        char ch = trial[alert_no].no_notify + '0' ;
        draw_message(&ch,60,( alert_no -1 )*8 + 30,1);
        draw_message(trial[alert_no].text,70,( alert_no -1 )*8 + 30,strlen(trial[alert_no].text));


    }





    // Raw Code for printing the msgs on Screen , as snprintf not giving desired results ( Need to Debug it later !! ) 
/*


       for(int alert_no = 1 ; alert_no < 5; alert_no ++)
    {

        snprintf(alert_msgs,60,"%s : %d ; %s ",alert_categories[alert_no],trial[alert_no].no_notify ,trial[alert_no].text);
        draw_message(alert_msgs,20,alert_no*8 + 30,strlen(alert_msg));

    }

    */


}





//////////////////////////////////////////
 //          BLE LAYER 
/////////////////////////////////////////

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM                 1                           // There can be many profiles in an app
#define PROFILE_APP_IDX             0                           
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "WATCH_DOG"
#define SVC_INST_ID                 0



/* The max length of characteristic value. When the gatt client write or prepare write, 
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX. 
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))


uint16_t handle_table[HRS_IDX_NB];

static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x20,
    .max_interval        = 0x40,
    .appearance          = 193,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = false,
    .include_txpower     = false,
    // .min_interval        = 0x20,
    // // .max_interval        = 0x40,
    // .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 16,
    // .p_service_uuid      = service_uuid,
    // .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};



static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST      = 0x1811;        // UUID set for Alert Notification Service 
static const uint16_t GATTS_CHAR_UUID_TEST_A       = 0x2A46;        // UUID for New Alert ( Characterstic ) 
static const uint16_t GATTS_CHAR_UUID_TEST_B       = 0x2A3D;        // UUID for String 
static const uint16_t GATTS_CHAR_UUID_TEST_C       = 0xFF03;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write          = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint32_t char_value = 0 ;


/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    /* Characteristic Declaration */
    [IDX_CHAR_A]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}},

    [IDX_CHAR_CFG_A] = 
    {{ESP_GATT_AUTO_RSP},{ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

    [IDX_CHAR_B] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},
 
     [IDX_CHAR_VAL_B] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)&char_value}},

};

static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
   char *key_str = NULL;
   switch(key_type) {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;

   }

   return key_str;
}


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
       
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
                esp_ble_gap_start_advertising(&adv_params);
            break;
        
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
     
            break;
       
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        
        case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
                 ESP_LOGE(GATTS_TABLE_TAG, "The passkey Notify number:%d", param->ble_security.key_notif.passkey);
            break;
        
        case ESP_GAP_BLE_KEY_EVT:
                ESP_LOGI(GATTS_TABLE_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
            break;
        
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
              {  esp_bd_addr_t bd_addr;
                memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr,sizeof(esp_bd_addr_t));
                ESP_LOGI(GATTS_TABLE_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) +   
                bd_addr[3],  
                (bd_addr[4] << 8) + bd_addr[5]);
                ESP_LOGI(GATTS_TABLE_TAG, "address type = %d",   
                param->ble_security.auth_cmpl.addr_type);  
                ESP_LOGI(GATTS_TABLE_TAG, "pair status = %s",  
                param->ble_security.auth_cmpl.success ? "success" : "fail");
              }
              break;

        case ESP_GAP_BLE_SEC_REQ_EVT:
                 esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
              break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}



static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }

            interface = gatts_if ;
              //config scan response data
            ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&scan_rsp_data));
   
               //config adv data
            ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));

            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
           { 
               
                ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT , handle is :%d , Device Address :%d:%d:%d:%d:%d:%d",param->read.handle,param->read.bda[0],param->read.bda[1],param->read.bda[2],param->read.bda[3],param->read.bda[4],param->read.bda[5]);

           }

       	    break;
        case ESP_GATTS_WRITE_EVT:
           {
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d", param->write.handle, param->write.len);
               // ESP_LOGI(GATTS_TABLE_TAG,"Value of Data Written to the Handle is : %s",param->write.value);
                sort_queue(param->write.value, param->write.len,param->write.handle);
           }
            break;
        case ESP_GATTS_EXEC_WRITE_EVT: 
            // the length of gattc prapare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX. 
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");

            // example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d", param->conf.status);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
           
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            conn_id = param->connect.conn_id ;
             esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);  
            // vTaskResume( xHandle );
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = %d", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                ESP_LOGI(GATTS_TABLE_TAG,"Handles Are : %d , %d , %d",param->add_attr_tab.handles[0],param->add_attr_tab.handles[1],param->add_attr_tab.handles[2]);
                memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
                ESP_LOGI(GATTS_TABLE_TAG,"Handle Value for the ALert Characterstic is : %d & for WhatsApp String is : %d",handle_table[IDX_CHAR_VAL_A],handle_table[IDX_CHAR_VAL_B]);
                esp_ble_gatts_start_service(handle_table[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_SET_ATTR_VAL_EVT:
        {
                ESP_LOGI(GATTS_TABLE_TAG,"Attribute Value Set , ")


        }break;
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }

}

void update(void *args)
{
    vTaskSuspend(NULL);         // Suspending the self task , waiting for the notification from the Create Table Event 
    uint32_t time = 0 ;

    while(1)
    {
        time ++;      
        ESP_ERROR_CHECK(esp_ble_gatts_set_attr_value(handle_table[IDX_CHAR_VAL_A],sizeof(char_value),(uint8_t *)&time));
        ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(interface,conn_id,handle_table[IDX_CHAR_VAL_A],sizeof(time),(uint8_t *)&time,false));
        vTaskDelay(1000/portTICK_RATE_MS);

    }

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
xTaskCreate(&update,"Update Value",2048,NULL,5,&xHandle);
     configure_BLE();


}



void configure_BLE()
{

 esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_profile_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

        // Constants & Parameters for BLE Security 

    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE ;              // IO Capability to None ( which means NO Screen / No Keyboard etc ) 
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;         //bonding with peer device after authentication
    uint8_t key_size = 16;                                   //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;      // For TLK & IRK Key 
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));



    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }


}

void sort_queue(uint8_t *value , uint16_t len ,uint8_t handle)
{

    if ( handle == handle_table[IDX_CHAR_VAL_A])
    {

        ESP_LOGI(GATTS_TABLE_TAG,"Alert Characterstic Change Detected");
        parse_alert(value,len);
        print_alerts();

    }else if( handle == handle_table[IDX_CHAR_VAL_B])
    {
        ESP_LOGI(GATTS_TABLE_TAG,"Message Change Detected");
        ESP_LOGI(GATTS_TABLE_TAG,"Message Received is : %s",value);

    }

}

void parse_alert(uint8_t *value , uint16_t len )
{

    // Category ID , New Alerts are of unsigned 8 bits so their value would be contained in the first two position of array 
    uint8_t category = value[0] ; 
    uint8_t no_notify = value[1];
    
    trial[category].category = category;
    trial[category].no_notify = no_notify ; 
    memcpy(trial[category].text,(value + 2 ),(len -2 ));

}

void print_alerts()
{

    for(int alert_no = 0 ; alert_no < 6; alert_no ++)
    {

        printf("\n%s : %d  , Value : %s ",alert_categories[alert_no],trial[alert_no].no_notify ,trial[alert_no].text);

    }

}



/*

To change the vlaue fo the Attribute localy , we need to set the attribute value according to the handle , 
To disable the notification , indication .


Inside the New Alert Service  : 

Format : Category( 1 Byte ) , No_Notofication , Last_Name


In the Defining of the Structure for the Alerts , ther could have been two approaches : 
1.Creating array of Objects , each object has Properties : ID , No_Alert , Name 
  User Layer would access that API by deferencing the array 

2. Creating array of categories & Alerts inside the Structure 
    & Defrencing the value of array inside that Structure 




*/




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


For the purpose of Structure bsed design of the State machine , the Structure can be used to store the function pointer easily  ( inspite the fact that 
it can't store the function inside them ) .z



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


Now , Adding up the Flash Light Feature : 
1. TURN ON the LED 
2. TURN OFF the LED 

Delays Inside the Task should be minimum , as it makes the UI sluggish as the Task is put into blocking mode 

For GUI design of the Flash Light : 

*/