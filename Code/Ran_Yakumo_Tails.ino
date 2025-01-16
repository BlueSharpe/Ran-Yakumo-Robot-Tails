/*
*******************************************************************************
*******************************************************************************
*******************************************************************************
*******************************************************************************

Note: comments and details will be added when the project reaches its final stages  
This project is open source.


*******************************************************************************
*******************************************************************************
*******************************************************************************
*******************************************************************************
*/
#include "PCA9685.h"
#include <Wire.h>
#include <PWMServo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#define  TOTAL_MOTOR_AMMOUNT 49u

#define  LCD_BUFFER_SIZE 32u

#define  SCREEN_WIDTH 128u
#define  SCREEN_HEIGHT 32u

#define  BATTERY_MIN 750u
#define  BATTERY_MAX 940u

#define PCA9685_PWM_FREQUENCY 400 //2500µs = 400 hz

#define BATTMON_READINGS 50u

#define ANMETER_MEASUREMENT_DEPTH 20
#define ANMETER_MIN_THRESHOLD 512
#define ANMETER_MAX_THRESHOLD 1024

#define HEARTRATE_THRESHOLD_PERCENTAGE 0.8
#define HEARTRATE_MINIMUM_CYCLE_LENGTH 10
#define HEARTRATE_MINUTE_TO_MS 60000
#define HEARTRATE_CALIBRATION_LENGTH 5000

#define TAIL_COUNT 9 // in case we decide to make chen as well

#define IMU_REPETITIONS 200
#define IMU_TORSO_ADRESS_ID 0x69
#define IMU_HEAD_ADRESS_ID 0x68

#define UPPER_MOTOR_ANGLE_LIMIT 135
#define LOWER_MOTOR_ANGLE_LIMIT -135


#define PCA9685_ADDR_1 0x40
#define PCA9685_ADDR_2 0x41
#define PCA9685_ADDR_3 0x42
#define PCA9685_COUNT 3

#define DIAG_MENU_MAX_CURSOR 0
#define RELAY_MENU_MAX_CURSOR 9 
#define POWER_MENU_MAX_CURSOR 0
#define ERROR_MENU_MAX_CURSOR 12
#define ANIM_MENU_MAX_CURSOR 12
#define MOTOR_MENU_MAX_CURSOR 8
#define MAIN_MENU_MAX_CURSOR  6

// warining this will greatly increase the dynamic memory usage, try to keep it below 75% otherwise the whole thign will crash mid execution
// alternatively a different appoach may be taken where where animations are loaded exclusively from flash memory when needed
#define ANIMATION_BUFFER_SIZE 15
#define ANIMATION_SPPED 1 

#define IMU_TORSO_ID 0
#define IMU_HEAD_ID 1


/****************** Digital Pinouts **********/
//D0  - Free
//D1  - Free
//D2  - Free
//D3  - Free
//D4  - Free
//D5  - Free
//D6  - Free
//D7  - Free
//D8  - Free
//D9  - Free
//D10  - Free
//D11  - Free
//D12  - Free
//D13  - Free
//D14  - Free
//D15  - Free
//D16 - Free
//D17 - Free
//D18 - Free
//D19 - Free
//D20 - Free
//D21 - Free
//D22 - Free
//D23 - Free
//D24 - Free
//D25 - Free
#define BUZZER_PIN 26
#define RESET_BUTTON 37 //D27 - Free-- emergecy button
#define BUTTON_MATRIX_COLUMN_1 28
#define BUTTON_MATRIX_COLUMN_2 29
#define BUTTON_MATRIX_COLUMN_3 30
#define BUTTON_MATRIX_COLUMN_4 31
#define BUTTON_MATRIX_ROW_1 32
#define BUTTON_MATRIX_ROW_2 33
#define BUTTON_MATRIX_ROW_3 34
#define BUTTON_MATRIX_ROW_4 35
//D36 - Free
//D37 - Free
//D38 - Free
//D39 - Free
//D40 - unusable?
//D41 - Free
//D42 - Free
//D43 - Free
#define DIRECT_PWM_PIN 44
#define RELAY_CH_8_PIN 45 
#define RELAY_CH_9_PIN 46 
#define RELAY_CH_7_PIN 47 
#define RELAY_CH_6_PIN 48 
#define RELAY_CH_5_PIN 49 
#define RELAY_CH_4_PIN 50 
#define RELAY_CH_3_PIN 51 
#define RELAY_CH_2_PIN 52 
#define RELAY_CH_1_PIN 53 


/****************** Analog Pinouts **********/

#define Ammeter_CH_1_PIN  A0  // Current sensor 1
#define Ammeter_CH_2_PIN  A1  // Current sensor 2
#define Ammeter_CH_3_PIN  A2  // Current sensor 3
#define Ammeter_CH_4_PIN  A3  // Current sensor 4
#define Ammeter_CH_5_PIN  A4  // Current sensor 5
#define Ammeter_CH_6_PIN  A5  // Current sensor 6
#define Ammeter_CH_7_PIN  A6  // Current sensor 7
//A7 -  old broken bat monitor
#define Ammeter_CH_8_PIN  A8  // Current sensor 8
#define Ammeter_CH_9_PIN  A9  // Current sensor 9
#define BATTMON_PIN A10 
#define HEART_MONITOR_ANALOG_PIN A11 // Heart monitor
#define HEART_MONITOR_LEFT_PAD_PIN A12
#define HEART_MONITOR_RIGHT_PAD_PIN A13
//A14 - Free
//A15 - Free

/****************** Enumerations *****************/


enum TAIL_CONTROL_STATE_EN 
{
SM_STARTUP_EN=0,
SM_IDLE_EN=1,
SM_HUG_EN=2,
SM_WAG_INDIV_EN=3,
SM_WAG_COMBI_EN=4,
SM_CROWD_EN=5,
SM_POSE_1_EN=6,
SM_POSE_2_EN=7,
SM_POSE_3_EN=8
}; 

enum GUI_MENU_LIST
{
  DIAG_MENU_E=1,
  RELAY_MENU_E=2,
  POWER_MENU_E=3,
  ERROR_MENU_E=4,
  ANIM_MENU_E=5,
  MOTOR_MENU_E=6,
  MAIN_MENU_E=7
};

enum MOTOR_MENU_LIST
{
  MOTOR_ID_UP_E=1,
  MOTOR_ID_DOWN_E=2,
  POSITION_DIGIT_1_UP_E=3,
  POSITION_DIGIT_1_DOWN_E=4,
  POSITION_DIGIT_2_UP_E=5,
  POSITION_DIGIT_2_DOWN_E=6,
  POSITION_DIGIT_3_UP_E=7,
  POSITION_DIGIT_3_DOWN_E=8
};
  
enum GUI_INPUT_LIST
{
  UP_INPUT_E=1,
  DOWN_INPUT_E=2,
  ENTER_INPUT_E=3,
  EXIT_INPUT_E=4
};

uint8_t Ammeter_Pin_AU8[TAIL_COUNT]=
{
  Ammeter_CH_1_PIN,
  Ammeter_CH_2_PIN,
  Ammeter_CH_3_PIN,
  Ammeter_CH_4_PIN,
  Ammeter_CH_5_PIN,
  Ammeter_CH_6_PIN,
  Ammeter_CH_7_PIN,
  Ammeter_CH_8_PIN,
  Ammeter_CH_9_PIN
};

uint8_t Relay_Pin_AU8[TAIL_COUNT]=
{
  RELAY_CH_1_PIN,
  RELAY_CH_2_PIN,
  RELAY_CH_3_PIN,
  RELAY_CH_4_PIN,
  RELAY_CH_5_PIN,
  RELAY_CH_6_PIN,
  RELAY_CH_7_PIN,
  RELAY_CH_8_PIN,
  RELAY_CH_9_PIN
};

enum ERROR_CODES_ENUM
{
LEFT_HEART_ELECTRODE_ISSUE_E=1,
RIGHT_HEART_ELECTRODE_ISSUE_E=2,
LOW_BATTERY_WARNING_E=3,
OVER_CURRENT_PROTECTION_1_E=4,
OVER_CURRENT_PROTECTION_2_E=5,
OVER_CURRENT_PROTECTION_3_E=6,
OVER_CURRENT_PROTECTION_4_E=7,
OVER_CURRENT_PROTECTION_5_E=8,
OVER_CURRENT_PROTECTION_6_E=9,
OVER_CURRENT_PROTECTION_7_E=10,
OVER_CURRENT_PROTECTION_8_E=11,
OVER_CURRENT_PROTECTION_9_E=12,
ERROR_CODE_COUNT_E=13
};

enum PWM_OUTPUT_MODE_EN 
{
  EXTERNAL_CH1_E=0,
  EXTERNAL_CH2_E=1,
  EXTERNAL_CH3_E=2,
  INTERNAL_PWM_E=3
}; 

enum ANIMATION_STATUS_LIST
{
  ANIMATION_MOTOR_POSITION_EN=0,
  ANIMATION_STEP_SIZE_EN=1,
  ANIMATION_STEP_COUNT_EN=2,
  ANIMATION_STATUS_LIST_COUNT_EN=3
};

enum ANIMATION_DATA_LIST
{
  ANIMATION_DATA_TARGET_ANGLE_EN=0,
  ANIMATION_DATA_ANIMATION_DURATION_EN=1,
  ANIMATION_DATA_LIST_COUNT_EN=2
};

enum ANIMATION_BUFFER_LIST
{
  ANIMATION_QUEUE_CURRENT_POSITION_EN=0,
  ANIMATION_QUEUE_NEXT_POSITION_EN=1,
  ANIMATION_BUFFER_LIST_COUNT=2
};

/****************** Structures *****************/

struct Motor_Instance_S
{
  uint8_t            Pin_U8 ;
  PWM_OUTPUT_MODE_EN Output_mode_U8;
} Motor_Control_S[TOTAL_MOTOR_AMMOUNT];

struct IMU_Data {
  float X_Axis_F;
  float Y_Axis_F;
};

struct Animation_Status_S
{
  uint16_t  Motor_Position;
  int16_t  Step_size;
  uint16_t  Step_Count;
};


/****************** Variables *****************/

// this matrix holds information for current position, step size(this is dergees done in a single step) and step count(Remaining number steps for each animation)
// step count and step size are to be caluclated from data found in the animation queue which has speed and duration

Animation_Status_S Motor_Animation_Status_AU16 [TOTAL_MOTOR_AMMOUNT];
uint16_t Animation_Queue_Data_AU16 [TOTAL_MOTOR_AMMOUNT][ANIMATION_BUFFER_SIZE][ANIMATION_DATA_LIST_COUNT_EN];// this 3D matrix is a circular buffer and contains animaiton data for upcoming animations for each animation. it holds values for each motor and has a predefined depth (ANIMATION_BUFFER_SIZE) the infomration held is target angle and duration
uint8_t  Animation_Queue_Positions_U8[TOTAL_MOTOR_AMMOUNT][ANIMATION_BUFFER_LIST_COUNT];//this matrix holds current and next buffer positions with values going from 0 to max buffer size
bool Motor_Animation_Loaded_ABO[TOTAL_MOTOR_AMMOUNT];//vector to indicate that a animation for a motor is loaded

uint8_t   Ammeter_Results_U8[TAIL_COUNT];
         
PCA9685 PCA9685_Module_1(PCA9685_ADDR_1);
PCA9685 PCA9685_Module_2(PCA9685_ADDR_2);
PCA9685 PCA9685_Module_3(PCA9685_ADDR_3);

PWMServo Direct_PWM_Motor;
uint16_t  PCA9685_Motor_Angle_To_Be_Written_MU16[PCA9685_COUNT][16]={}; // this buffer holds the values that are about to be sent to the PCA modules
uint16_t  Direct_PWM_Motor_Angle_U16; // this is a lone variable to hold the angle of the 1 motor that is connected directly to the µC PWM
uint8_t   Motor_Lookup_table_MU16[PCA9685_COUNT][16];

//task request booleans, these are used for tasks which need to be executed based on the tasking system..
//but due to problems with the I2C or other problems.. they cannot be executed on interupt.
//so they are executed only in the Loop and they are triggered from interrupt
bool      OLED_Update_BO=true;
bool      Motor_Update_BO=true;
bool      Startup_Sequence_BO=true;
bool      Buzzer_Status_BO=false;
bool      HeartRate_Update_BO=true;

bool      HeartRate_Calibration=true;

uint8_t   Buzzer_timer_U8=0;
uint8_t   Startup_Counter_U8=200;
uint16_t  OLED_Timer_U16=100;
uint8_t   OLED_Connection_Status_U8=1;
// used only at startup to keep the logo on screen for 2 seconds( 100*20 ms)
 
uint8_t   Minimum_Recorded_Battery_U8 =100;

uint16_t  HeartRate_Remaining_Data_U16 =HEARTRATE_CALIBRATION_LENGTH; // number of readings to be done for calibration
uint16_t  HeartRate_Threshhold_U16 ;// the threshold value where a hearthbeat is being considered (this is a calculated value during calibration)
uint16_t  HeartRate_Result_U16 ;

uint16_t  Ammeter_Average_data_AU16 [TAIL_COUNT]; // hold the result of the anmeter calulations

bool     Tail_Active_AU8[TAIL_COUNT]; // this array holds the status of tails
bool     Relay_flip_ABO[TAIL_COUNT]={true,true,true,true,true,true,true,true,true}; // this is needed when mixing simple relays with relays+optocouplers, not the case here

uint8_t OLED_cursor_Position=1;
uint8_t Max_Cursor=MAIN_MENU_MAX_CURSOR;
uint8_t Motor_Menu_Selected_Motor=0;
GUI_MENU_LIST current_menu_EN=MAIN_MENU_E;

float elapsedTime, time, timePrev;        //Variables for time control

//const int16_t Motor_Angle_Adjustment[TOTAL_MOTOR_AMMOUNT]= {0,0,120,120,60,210,250,-20,-110,30,120,130,-70,60,-20,0,-100,120,90,70,-40,130,40,40,160,-150,190,40,-30,100,-100,100,-50,-70,170,-70,0,30,130,-140,110,-110,-30,-90,-140,0,0,0,0};
//const int16_t Motor_Angle_Adjustment[TOTAL_MOTOR_AMMOUNT]= {110,-110,-30,-90,-140,-70,0,30,130,-140,-100,100,-50,-70,170,-150,190,40,-30,100,-40,130,40,40,160,0,-100,120,90,70,120,130,-70,60,-20,210,250,-20,-110,30,0,0,120,120,60,0,0,0,0};
const int16_t Motor_Angle_Adjustment[TOTAL_MOTOR_AMMOUNT]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

float Rad_to_Deg=180/3.141592654;

IMU_Data Gyro_error[2];
IMU_Data Accel_error[2];
IMU_Data Gyro_Data[2];
IMU_Data Accel_Data[2];

TAIL_CONTROL_STATE_EN Current_State_EN=SM_STARTUP_EN;
TAIL_CONTROL_STATE_EN Next_State_EN=SM_STARTUP_EN;
uint16_t Loaded_Animation_Duration_U16=0;

byte  Button_Matrix_Row_Pins_AU8 [4]=   {BUTTON_MATRIX_ROW_1    ,BUTTON_MATRIX_ROW_2    ,BUTTON_MATRIX_ROW_3    ,BUTTON_MATRIX_ROW_4   };
byte  Button_Matrix_Column_Pins_AU8 [4]={BUTTON_MATRIX_COLUMN_1 ,BUTTON_MATRIX_COLUMN_2 ,BUTTON_MATRIX_COLUMN_3 ,BUTTON_MATRIX_COLUMN_4};

uint8_t  Row_Counter_U8 ;
uint8_t  Column_Counter_U8 ;

uint8_t  Motor_ID_U8 =0;
uint8_t  Direct_Servo_id_U8 =0;
  
bool     Error_flag_ABO[ERROR_CODE_COUNT_E];
  
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/****************** Function Declaration *******/
void     MOD_SERV_Motor_Manager_init();
void     MOD_SERV_Motor_Manager_Create_motor( uint8_t  Param_Pin_U8 , PWM_OUTPUT_MODE_EN Param_Enum_EN);
uint16_t MOD_SERV_Motor_Manager_Get_Current_Angle(uint8_t  Param_Motor_ID_U8);
void     MOD_SERV_Motor_Manager_Adjust_Angle_From_GUI(uint8_t  Param_Motor_ID_U8 ,int16_t  Param_motor_Angle_S16);
void     MOD_SERV_Motor_Manager_Write_To_All_Motors();
uint16_t MOD_SERV_Motor_Angle_Converter(uint16_t Input_Value_U16_Param, uint8_t Motor_ID_U8_Param);

void     MOD_SERV_PCA9685_Init();

uint16_t MOD_MCAL_Battery_Get_Raw();
void     MOD_SERV_Battery_Get_Percentage();

void     MOD_SERV_OLED_Main_Screen();
void     MOD_SERV_OLED_Anmeter_Screen();
void     MOD_SERV_OLED_Diagnostic_Screen();
void     MOD_SERV_OLED_Relay_Control_Screen();
void     MOD_SERV_OLED_Animation_Control_Screen();
void     MOD_SERV_OLED_Error_Screen();
void     MOD_SERV_OLED_Motor_Screen();
void     MOD_SERV_OLED_Manager_Init();
void     MOD_SERV_OLED_Load_Base_UI();
void     MOD_SERV_OLED_Manager_Print_Text(String Message_ST,uint8_t  X_Cord_U8 ,uint8_t  Y_Cord_U8 );
void     MOD_SERV_OLED_Manager_Draw_Bitmap(const unsigned char Bitmap[],uint8_t  bitmap_Width_U8 ,uint8_t  bitmap_Height_U8 );
void     MOD_SERV_OLED_Screen_Manager();
void     MOD_SERV_OLED_Input_up();
void     MOD_SERV_OLED_Input_down();
void     MOD_SERV_OLED_input_Manager(GUI_INPUT_LIST Received_input_EN);

void     MOD_MCAL_IMU_Configuration(uint8_t I2C_address_U8);
void     MOD_MCAL_IMU_Accel_Measure_Error(uint8_t I2C_address_U8,uint8_t IMU_ID);
void     MOD_MCAL_IMU_Gyro_Measure_Error(uint8_t I2C_address_U8,uint8_t IMU_ID);
void     MOD_MCAL_IMU_Gyro_Read_Data(uint8_t I2C_address_U8,uint8_t IMU_ID);
void     MOD_MCAL_IMU_Accel_Read_Data(uint8_t I2C_address_U8,uint8_t IMU_ID);
float    MOD_SERV_IMU_Get_Angle();

void     MOD_SERV_Animation_Add_Into_Queue(uint8_t Motor_ID_U8_Param,uint16_t Target_angle_U16_Param,uint16_t Animation_Duration_U16_Param);
void     MOD_SERV_Animation_Load_From_Queue(uint8_t  Motor_ID_U8_Param);
void     MOD_SERV_Animation_Run(uint8_t  Motor_ID_U8_Param);
void     MOD_APPL_Animation_Manager();
void     MOD_SERV_Animation_State_Machine();

void     MOD_SERV_State_Machine_STARTUP_ACTION();
void     MOD_SERV_State_Machine_IDLE_ACTION();
void     MOD_SERV_State_Machine_HUG_ACTION();
void     MOD_SERV_State_Machine_WAG_INDIV_ACTION();
void     MOD_SERV_State_Machine_WAG_COMBI_ACTION();
void     MOD_SERV_State_Machine_CROWD_ACTION();
void     MOD_SERV_State_Machine_POSE_1_ACTION();
void     MOD_SERV_State_Machine_POSE_2_ACTION();
void     MOD_SERV_State_Machine_POSE_3_ACTION();

void     MOD_SERV_State_Machine_STARTUP_ENTRY();
void     MOD_SERV_State_Machine_IDLE_ENTRY();
void     MOD_SERV_State_Machine_HUG_ENTRY();
void     MOD_SERV_State_Machine_WAG_INDIV_ENTRY();
void     MOD_SERV_State_Machine_WAG_COMBI_ENTRY();
void     MOD_SERV_State_Machine_CROWD_ENTRY();
void     MOD_SERV_State_Machine_POSE_1_ENTRY();
void     MOD_SERV_State_Machine_POSE_2_ENTRY();
void     MOD_SERV_State_Machine_POSE_3_ENTRY();

void     MOD_SERV_State_Machine_STARTUP_EXIT();
void     MOD_SERV_State_Machine_IDLE_EXIT();
void     MOD_SERV_State_Machine_HUG_EXIT();
void     MOD_SERV_State_Machine_WAG_INDIV_EXIT();
void     MOD_SERV_State_Machine_WAG_COMBI_EXIT();
void     MOD_SERV_State_Machine_CROWD_EXIT();
void     MOD_SERV_State_Machine_POSE_1_EXIT();
void     MOD_SERV_State_Machine_POSE_2_EXIT();
void     MOD_SERV_State_Machine_POSE_3_EXIT();

void     MOD_MCAL_Task_init();

void     MOD_SERV_Power_Manager_Init();
void     MOD_SERV_Power_Manager_Read_Ammeters();
void     MOD_SERV_Emergency_Shutdown(uint8_t  Param_Relay_channel_U8 );

void     MOD_MCAL_Relay_Control(uint8_t  Param_Relay_channel_U8 , bool Param_Status_BO);
void     MOD_MCAL_Relay_Toggle(uint8_t  Param_Relay_channel_U8);
void     MOD_MCAL_Relay_Init();

void     MOD_SERV_HeartRate_Monitor();
void     MOD_SERV_HeartRate_Calibration();

uint8_t  MOD_SERV_Button_Matrix_Read();
void     MOD_SERV_Button_Matrix_Init();
void     MOD_SERV_Button_Matrix_Task();

void     MOD_SERV_Error_Set(ERROR_CODES_ENUM Received_Error);
void     MOD_SERV_Error_Reset(ERROR_CODES_ENUM Received_Error);
void     MOD_MCAL_Error_Buzzer_trigger();

void     MOD_SERV_Main_Motor_Update();
void     MOD_SERV_Main_OLED_Update();
void     MOD_SERV_Main_Buzzer_Update();
void     MOD_SERV_Main_HeartMonitor_Update();
void     MOD_SERV_Main_Startup_Sequence();



/****************** Fumo Inside Logo ᗜˬᗜ *************/

static const unsigned char PROGMEM Fumo[]      = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xf0, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0f, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0xf0, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x0c, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x20, 
  0x00, 0x00, 0xf0, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 
  0x1f, 0xff, 0xfc, 0x1f, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 
  0x07, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
  0x07, 0x3f, 0xf0, 0x07, 0x3f, 0xf0, 0x01, 0xfe, 0x30, 0x18, 0x70, 0x03, 0x80, 0x7e, 0x01, 0x00, 
  0x06, 0x1f, 0xf0, 0x06, 0x1f, 0xf0, 0x01, 0xfe, 0x30, 0x18, 0x78, 0x07, 0x81, 0xff, 0x83, 0xe0, 
  0x06, 0x1f, 0xf0, 0x06, 0x1f, 0xf0, 0x01, 0x80, 0x30, 0x18, 0x78, 0x07, 0x83, 0xc1, 0xc0, 0x00, 
  0x07, 0x3f, 0xf0, 0x07, 0x3f, 0xf0, 0x01, 0x80, 0x30, 0x18, 0x6c, 0x0d, 0x83, 0x00, 0xc0, 0x00, 
  0x07, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x01, 0x80, 0x30, 0x18, 0x6c, 0x0d, 0x87, 0x00, 0xe0, 0x00, 
  0x07, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x01, 0x80, 0x30, 0x18, 0x6c, 0x09, 0x86, 0x00, 0x60, 0x00, 
  0x07, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x01, 0xfe, 0x30, 0x18, 0x66, 0x19, 0x86, 0x00, 0x60, 0x00, 
  0x07, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x01, 0xfe, 0x30, 0x18, 0x66, 0x19, 0x86, 0x00, 0x60, 0x00, 
  0x07, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x01, 0x80, 0x30, 0x18, 0x63, 0x31, 0x86, 0x00, 0x60, 0x00, 
  0x07, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x01, 0x80, 0x30, 0x18, 0x63, 0x31, 0x86, 0x00, 0x60, 0x00, 
  0x07, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x01, 0x80, 0x30, 0x18, 0x63, 0x31, 0x87, 0x00, 0xe0, 0x00, 
  0x07, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x01, 0x80, 0x38, 0x38, 0x61, 0xe1, 0x83, 0x00, 0xc0, 0x00, 
  0x07, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x01, 0x80, 0x1c, 0x70, 0x61, 0xe1, 0x83, 0x83, 0xc0, 0x00, 
  0x07, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x01, 0x80, 0x1f, 0xf0, 0x60, 0xc1, 0x81, 0xff, 0x80, 0x00, 
  0x03, 0xff, 0xe0, 0x03, 0xff, 0xe0, 0x01, 0x80, 0x07, 0xc0, 0x60, 0xc1, 0x80, 0x7e, 0x00, 0x00, 
  0x01, 0xff, 0xc0, 0x01, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0xff, 0xc0, 0x01, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0xff, 0x80, 0x00, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x04, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


static const unsigned char PROGMEM UI_test []  = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xe0, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 
  0xc7, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 
  0x8f, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0x9f, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xbf, 0xe8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xa0, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xbf, 0xe8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 
  0xc0, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 
  0xe0, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/*************** MOTOR MANAGER *************/

void MOD_SERV_Motor_Manager_init()
{
  // magic numbers are phisical Pin IDS
  
  // // Tail 0
  //   MOD_SERV_Motor_Manager_Create_motor(1u  ,EXTERNAL_CH1_E); // 150kg motor id 1 not a typo i screwed up the wiring
  //   MOD_SERV_Motor_Manager_Create_motor(0u  ,EXTERNAL_CH1_E); // 150kg motor id 0 not a typo i screwed up the wiring
  //   MOD_SERV_Motor_Manager_Create_motor(2u  ,EXTERNAL_CH1_E); //  80kg motor id 2
  //   MOD_SERV_Motor_Manager_Create_motor(3u  ,EXTERNAL_CH1_E); //  80kg motor id 3
  //   MOD_SERV_Motor_Manager_Create_motor(4u  ,EXTERNAL_CH1_E); //  40kg motor id 4
  // // Tail 1                                                      
  //   MOD_SERV_Motor_Manager_Create_motor(5u  ,EXTERNAL_CH1_E); // 150kg motor id 5
  //   MOD_SERV_Motor_Manager_Create_motor(6u  ,EXTERNAL_CH1_E); // 150kg motor id 6
  //   MOD_SERV_Motor_Manager_Create_motor(7u  ,EXTERNAL_CH1_E); //  80kg motor id 7
  //   MOD_SERV_Motor_Manager_Create_motor(9u  ,EXTERNAL_CH1_E); //  80kg motor id 8
  //   MOD_SERV_Motor_Manager_Create_motor(8u  ,EXTERNAL_CH1_E); //  40kg motor id 9
  // // Tail 2                                                      
  //   MOD_SERV_Motor_Manager_Create_motor(10u ,EXTERNAL_CH1_E); // 150kg motor id 10
  //   MOD_SERV_Motor_Manager_Create_motor(11u ,EXTERNAL_CH1_E); // 150kg motor id 11
  //   MOD_SERV_Motor_Manager_Create_motor(12u ,EXTERNAL_CH1_E); //  80kg motor id 12
  //   MOD_SERV_Motor_Manager_Create_motor(13u ,EXTERNAL_CH1_E); //  80kg motor id 13
  //   MOD_SERV_Motor_Manager_Create_motor(14u ,EXTERNAL_CH1_E); //  40kg motor id 14
  //  // Tail 3                                                     
  //   MOD_SERV_Motor_Manager_Create_motor(0u  ,EXTERNAL_CH2_E); // 150kg motor id 15
  //   MOD_SERV_Motor_Manager_Create_motor(1u  ,EXTERNAL_CH2_E); // 150kg motor id 16
  //   MOD_SERV_Motor_Manager_Create_motor(2u  ,EXTERNAL_CH2_E); //  80kg motor id 17
  //   MOD_SERV_Motor_Manager_Create_motor(3u  ,EXTERNAL_CH2_E); //  80kg motor id 18
  //   MOD_SERV_Motor_Manager_Create_motor(4u  ,EXTERNAL_CH2_E); //  40kg motor id 19
  // // Tail 4                                                     
  //   MOD_SERV_Motor_Manager_Create_motor(5u  ,EXTERNAL_CH2_E); // 150kg motor id 20
  //   MOD_SERV_Motor_Manager_Create_motor(6u  ,EXTERNAL_CH2_E); // 150kg motor id 21
  //   MOD_SERV_Motor_Manager_Create_motor(7u  ,EXTERNAL_CH2_E); //  80kg motor id 22
  //   MOD_SERV_Motor_Manager_Create_motor(8u  ,EXTERNAL_CH2_E); //  80kg motor id 23
  //   MOD_SERV_Motor_Manager_Create_motor(9u  ,EXTERNAL_CH2_E); //  40kg motor id 24
  // // Tail 5                                                      
  //   MOD_SERV_Motor_Manager_Create_motor(10u ,EXTERNAL_CH2_E); // 150kg motor id 25
  //   MOD_SERV_Motor_Manager_Create_motor(11u ,EXTERNAL_CH2_E); // 150kg motor id 26
  //   MOD_SERV_Motor_Manager_Create_motor(12u ,EXTERNAL_CH2_E); //  80kg motor id 27
  //   MOD_SERV_Motor_Manager_Create_motor(13u ,EXTERNAL_CH2_E); //  80kg motor id 28
  //   MOD_SERV_Motor_Manager_Create_motor(14u ,EXTERNAL_CH2_E); //  40kg motor id 29
  //   // Tail 6                                                    
  //   MOD_SERV_Motor_Manager_Create_motor(0u  ,EXTERNAL_CH3_E); // 150kg motor id 30
  //   MOD_SERV_Motor_Manager_Create_motor(1u  ,EXTERNAL_CH3_E); // 150kg motor id 31
  //   MOD_SERV_Motor_Manager_Create_motor(2u  ,EXTERNAL_CH3_E); //  80kg motor id 32
  //   MOD_SERV_Motor_Manager_Create_motor(3u  ,EXTERNAL_CH3_E); //  80kg motor id 33
  //   MOD_SERV_Motor_Manager_Create_motor(4u  ,EXTERNAL_CH3_E); //  40kg motor id 34
  // // Tail 7                                                      
  //   MOD_SERV_Motor_Manager_Create_motor(5u  ,EXTERNAL_CH3_E); // 150kg motor id 35
  //   MOD_SERV_Motor_Manager_Create_motor(6u  ,EXTERNAL_CH3_E); // 150kg motor id 36
  //   MOD_SERV_Motor_Manager_Create_motor(7u  ,EXTERNAL_CH3_E); //  80kg motor id 37
  //   MOD_SERV_Motor_Manager_Create_motor(8u  ,EXTERNAL_CH3_E); //  80kg motor id 38
  //   MOD_SERV_Motor_Manager_Create_motor(9u  ,EXTERNAL_CH3_E); //  40kg motor id 39
  // // Tail 8                                                        
  //   MOD_SERV_Motor_Manager_Create_motor(10u ,EXTERNAL_CH3_E); // 150kg motor id 40
  //   MOD_SERV_Motor_Manager_Create_motor(11u ,EXTERNAL_CH3_E); // 150kg motor id 41
  //   MOD_SERV_Motor_Manager_Create_motor(12u ,EXTERNAL_CH3_E); //  80kg motor id 42
  //   MOD_SERV_Motor_Manager_Create_motor(13u ,EXTERNAL_CH3_E); //  80kg motor id 43
  //   MOD_SERV_Motor_Manager_Create_motor(14u ,EXTERNAL_CH3_E); //  40kg motor id 44
  // // Ears prototype                                  
  //   MOD_SERV_Motor_Manager_Create_motor(15u ,EXTERNAL_CH1_E); //  SG90 tower id 45
  //   MOD_SERV_Motor_Manager_Create_motor(15u ,EXTERNAL_CH2_E); //  SG90 tower id 46
  //   MOD_SERV_Motor_Manager_Create_motor(15u ,EXTERNAL_CH3_E); //  SG90 tower id 47
  //   MOD_SERV_Motor_Manager_Create_motor(44u ,INTERNAL_PWM_E); //  SG90 tower id 48


 MOD_SERV_Motor_Manager_Create_motor(14u ,EXTERNAL_CH3_E); // 150kg motor id 0
 MOD_SERV_Motor_Manager_Create_motor(13u ,EXTERNAL_CH3_E); // 150kg motor id 1
 MOD_SERV_Motor_Manager_Create_motor(12u ,EXTERNAL_CH3_E); //  80kg motor id 2
 MOD_SERV_Motor_Manager_Create_motor(10u ,EXTERNAL_CH3_E); //  80kg motor id 3
 MOD_SERV_Motor_Manager_Create_motor(11u ,EXTERNAL_CH3_E); //  40kg motor id 4
 MOD_SERV_Motor_Manager_Create_motor(5u  ,EXTERNAL_CH3_E); // 150kg motor id 5
 MOD_SERV_Motor_Manager_Create_motor(6u  ,EXTERNAL_CH3_E); // 150kg motor id 6
 MOD_SERV_Motor_Manager_Create_motor(7u  ,EXTERNAL_CH3_E); //  80kg motor id 7
 MOD_SERV_Motor_Manager_Create_motor(8u  ,EXTERNAL_CH3_E); //  80kg motor id 8
 MOD_SERV_Motor_Manager_Create_motor(9u  ,EXTERNAL_CH3_E); //  40kg motor id 9
 MOD_SERV_Motor_Manager_Create_motor(0u  ,EXTERNAL_CH3_E); // 150kg motor id 10
 MOD_SERV_Motor_Manager_Create_motor(1u  ,EXTERNAL_CH3_E); // 150kg motor id 11
 MOD_SERV_Motor_Manager_Create_motor(2u  ,EXTERNAL_CH3_E); //  80kg motor id 12
 MOD_SERV_Motor_Manager_Create_motor(3u  ,EXTERNAL_CH3_E); //  80kg motor id 13
 MOD_SERV_Motor_Manager_Create_motor(4u  ,EXTERNAL_CH3_E); //  40kg motor id 14
 MOD_SERV_Motor_Manager_Create_motor(10u ,EXTERNAL_CH2_E); // 150kg motor id 15
 MOD_SERV_Motor_Manager_Create_motor(11u ,EXTERNAL_CH2_E); // 150kg motor id 16
 MOD_SERV_Motor_Manager_Create_motor(12u ,EXTERNAL_CH2_E); //  80kg motor id 17
 MOD_SERV_Motor_Manager_Create_motor(13u ,EXTERNAL_CH2_E); //  80kg motor id 18
 MOD_SERV_Motor_Manager_Create_motor(14u ,EXTERNAL_CH2_E); //  40kg motor id 19
 MOD_SERV_Motor_Manager_Create_motor(5u  ,EXTERNAL_CH2_E); // 150kg motor id 20
 MOD_SERV_Motor_Manager_Create_motor(6u  ,EXTERNAL_CH2_E); // 150kg motor id 21
 MOD_SERV_Motor_Manager_Create_motor(7u  ,EXTERNAL_CH2_E); //  80kg motor id 22
 MOD_SERV_Motor_Manager_Create_motor(8u  ,EXTERNAL_CH2_E); //  80kg motor id 23
 MOD_SERV_Motor_Manager_Create_motor(9u  ,EXTERNAL_CH2_E); //  40kg motor id 24
 MOD_SERV_Motor_Manager_Create_motor(0u  ,EXTERNAL_CH2_E); // 150kg motor id 25
 MOD_SERV_Motor_Manager_Create_motor(1u  ,EXTERNAL_CH2_E); // 150kg motor id 26
 MOD_SERV_Motor_Manager_Create_motor(2u  ,EXTERNAL_CH2_E); //  80kg motor id 27
 MOD_SERV_Motor_Manager_Create_motor(3u  ,EXTERNAL_CH2_E); //  80kg motor id 28
 MOD_SERV_Motor_Manager_Create_motor(4u  ,EXTERNAL_CH2_E); //  40kg motor id 29
 MOD_SERV_Motor_Manager_Create_motor(10u ,EXTERNAL_CH1_E); // 150kg motor id 30
 MOD_SERV_Motor_Manager_Create_motor(11u ,EXTERNAL_CH1_E); // 150kg motor id 31
 MOD_SERV_Motor_Manager_Create_motor(12u ,EXTERNAL_CH1_E); //  80kg motor id 32
 MOD_SERV_Motor_Manager_Create_motor(13u ,EXTERNAL_CH1_E); //  80kg motor id 33
 MOD_SERV_Motor_Manager_Create_motor(14u ,EXTERNAL_CH1_E); //  40kg motor id 34
 MOD_SERV_Motor_Manager_Create_motor(5u  ,EXTERNAL_CH1_E); // 150kg motor id 35
 MOD_SERV_Motor_Manager_Create_motor(6u  ,EXTERNAL_CH1_E); // 150kg motor id 36
 MOD_SERV_Motor_Manager_Create_motor(7u  ,EXTERNAL_CH1_E); //  80kg motor id 37
 MOD_SERV_Motor_Manager_Create_motor(9u  ,EXTERNAL_CH1_E); //  80kg motor id 38
 MOD_SERV_Motor_Manager_Create_motor(8u  ,EXTERNAL_CH1_E); //  40kg motor id 39
 MOD_SERV_Motor_Manager_Create_motor(0u  ,EXTERNAL_CH1_E); // 150kg motor id 40 
 MOD_SERV_Motor_Manager_Create_motor(1u  ,EXTERNAL_CH1_E); // 150kg motor id 41 
 MOD_SERV_Motor_Manager_Create_motor(2u  ,EXTERNAL_CH1_E); //  80kg motor id 42
 MOD_SERV_Motor_Manager_Create_motor(3u  ,EXTERNAL_CH1_E); //  80kg motor id 43
 MOD_SERV_Motor_Manager_Create_motor(4u  ,EXTERNAL_CH1_E); //  40kg motor id 44
   // // Ears prototype                                  
     MOD_SERV_Motor_Manager_Create_motor(15u ,EXTERNAL_CH1_E); //  SG90 tower id 45
     MOD_SERV_Motor_Manager_Create_motor(15u ,EXTERNAL_CH2_E); //  SG90 tower id 46
     MOD_SERV_Motor_Manager_Create_motor(15u ,EXTERNAL_CH3_E); //  SG90 tower id 47
     MOD_SERV_Motor_Manager_Create_motor(44u ,INTERNAL_PWM_E); //  SG90 tower id 48



}
  
void MOD_SERV_Motor_Manager_Create_motor( uint8_t  Param_Pin_U8 , PWM_OUTPUT_MODE_EN Param_Enum_EN)
{

  Motor_Control_S[Motor_ID_U8].Pin_U8         = Param_Pin_U8;
  Motor_Control_S[Motor_ID_U8].Output_mode_U8 = Param_Enum_EN;

  if(Param_Enum_EN!=INTERNAL_PWM_E)Motor_Lookup_table_MU16[Param_Enum_EN][Param_Pin_U8] = Motor_ID_U8;
  if(Param_Enum_EN==INTERNAL_PWM_E) Direct_PWM_Motor.attach(Param_Pin_U8);
  
  Motor_ID_U8++;
 
}

uint16_t MOD_SERV_Motor_Manager_Get_Current_Angle(uint8_t  Param_Motor_ID_U8)
{
 uint16_t Output_S16=0;
  switch(Motor_Control_S[Param_Motor_ID_U8 ].Output_mode_U8)
  {
    case EXTERNAL_CH1_E :
    case EXTERNAL_CH2_E :
    case EXTERNAL_CH3_E : {Output_S16 = PCA9685_Motor_Angle_To_Be_Written_MU16[Motor_Control_S[Param_Motor_ID_U8 ].Output_mode_U8][Motor_Control_S[Param_Motor_ID_U8].Pin_U8]   ; }	break;
    case INTERNAL_PWM_E : {Output_S16 = Direct_PWM_Motor_Angle_U16;                                                                                                               } break;
    default             : {Serial.println("error: motor id:"+(String)Param_Motor_ID_U8+" has unknown output mode:"+(String)Motor_Control_S[Param_Motor_ID_U8 ].Output_mode_U8); }   
  }
  return Output_S16;
}

void MOD_SERV_Motor_Manager_Adjust_Angle_From_GUI(uint8_t  Param_Motor_ID_U8 ,int16_t  Param_motor_Angle_S16)
{
  switch(Motor_Control_S[Param_Motor_ID_U8 ].Output_mode_U8)
  {
    case EXTERNAL_CH1_E :
    case EXTERNAL_CH2_E :
    case EXTERNAL_CH3_E : {PCA9685_Motor_Angle_To_Be_Written_MU16[Motor_Control_S[Param_Motor_ID_U8 ].Output_mode_U8][Motor_Control_S[Param_Motor_ID_U8].Pin_U8]+= Param_motor_Angle_S16; } break;
    case INTERNAL_PWM_E : {Direct_PWM_Motor_Angle_U16+=Param_motor_Angle_S16;                                                                                                             } break;
    default             : {Serial.println("error: motor id:"+(String)Param_Motor_ID_U8+" has unknown output mode:"+(String)Motor_Control_S[Param_Motor_ID_U8 ].Output_mode_U8);         }   
  }  
}

void MOD_SERV_Motor_Manager_Write_To_All_Motors()
{
  PCA9685_Module_1.setChannelsPWM(0, 16, PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E]);
  PCA9685_Module_2.setChannelsPWM(0, 16, PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E]);
  PCA9685_Module_3.setChannelsPWM(0, 16, PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E]);
  
  Direct_PWM_Motor_Angle_U16 = map(Direct_PWM_Motor_Angle_U16, 0, 4095, 0, 180);
  Direct_PWM_Motor.write(Direct_PWM_Motor_Angle_U16);// this library only supports angles(not the greatest fan but it is what it is)
}
  
uint16_t MOD_SERV_Motor_Angle_Converter(int16_t Input_Angle_S16_Param, uint8_t Motor_ID_U8_Param)
{
  uint16_t Output_U16=0;
	if((Input_Angle_S16_Param > LOWER_MOTOR_ANGLE_LIMIT) && (Input_Angle_S16_Param < UPPER_MOTOR_ANGLE_LIMIT))
	{
	  Output_U16=constrain(map(Input_Angle_S16_Param,-135,135,820,4095)+Motor_Angle_Adjustment[Motor_ID_U8_Param],820,4095);
	}
  return Output_U16;
}

/**************** PCA9685_Driver *************/

void MOD_SERV_PCA9685_Init()
{
  PCA9685_Module_1.resetDevices();
  PCA9685_Module_2.resetDevices();
  PCA9685_Module_3.resetDevices();

  PCA9685_Module_1.init();
  PCA9685_Module_2.init();
  PCA9685_Module_3.init();
  
  PCA9685_Module_1.setPWMFrequency(PCA9685_PWM_FREQUENCY);
  PCA9685_Module_2.setPWMFrequency(PCA9685_PWM_FREQUENCY);
  PCA9685_Module_3.setPWMFrequency(PCA9685_PWM_FREQUENCY);

  Loaded_Animation_Duration_U16+=5;
  
  for(uint8_t Motor_Count_U8=0;Motor_Count_U8<TOTAL_MOTOR_AMMOUNT;Motor_Count_U8++) MOD_SERV_Animation_Add_Into_Queue(Motor_Count_U8,0,5);

  for(uint8_t PCA9685_Channel_U8=0;PCA9685_Channel_U8<=EXTERNAL_CH3_E;PCA9685_Channel_U8++)
  for(uint8_t Pin_ID_U8=0;Pin_ID_U8<=15;Pin_ID_U8++)
  {
   PCA9685_Motor_Angle_To_Be_Written_MU16[PCA9685_Channel_U8][Pin_ID_U8]  =  MOD_SERV_Motor_Angle_Converter(0,Motor_Lookup_table_MU16[PCA9685_Channel_U8][Pin_ID_U8]); 
   Serial.println("Channel: "+(String)PCA9685_Channel_U8+" Pin:"+Pin_ID_U8 +" Set to "+(String)PCA9685_Motor_Angle_To_Be_Written_MU16[PCA9685_Channel_U8][Pin_ID_U8]);
   Motor_Animation_Status_AU16[Pin_ID_U8].Motor_Position                  =  PCA9685_Motor_Angle_To_Be_Written_MU16[PCA9685_Channel_U8][Pin_ID_U8];
  }  
}

/*************** BATTERY MONITOR *************/

uint16_t MOD_MCAL_Battery_Get_Raw()
{
 uint16_t  Measured_Average_U16 =0;
 uint16_t  Raw_Value_U16 ;
 
 for(uint16_t  Battery_Readings_U16 =0; Battery_Readings_U16  < BATTMON_READINGS; Battery_Readings_U16++)
    {
     Raw_Value_U16  = analogRead(BATTMON_PIN);
     Measured_Average_U16  += Raw_Value_U16 ;
    }

  Measured_Average_U16 =Measured_Average_U16 / BATTMON_READINGS;

  return Measured_Average_U16 ;
}
  
void MOD_SERV_Battery_Get_Percentage()
{
  uint16_t  Battery_Raw_Received_U16 ;
  uint8_t  Battery_Percentage_U8 ;

  Battery_Raw_Received_U16 =MOD_MCAL_Battery_Get_Raw();
  Battery_Percentage_U8  = map(Battery_Raw_Received_U16 , BATTERY_MIN, BATTERY_MAX, 0, 100); 

  Minimum_Recorded_Battery_U8 = Battery_Percentage_U8 ;

 // the approach below cause some spikes when activating the relays, which would lock you in a lower voltage then you actually are in
 // the Test PSU can only supply 21 Amps , it is possible that this problem will not have that much of an impact using using the 90 Amp battery
  
 //  if(Battery_Percentage_U8 <Minimum_Recorded_Battery_U8 )
 //    {
 //     Minimum_Recorded_Battery_U8 =Battery_Percentage_U8 ;
 //     OLED_Update_BO = true;
 //    }
}

/***************LCD MANAGER*********************/

void MOD_SERV_OLED_Main_Screen()
{
  display.clearDisplay();
  MOD_SERV_OLED_Load_Base_UI();
  MOD_SERV_OLED_Manager_Print_Text(" Diag  Pwr Anim ",15,2);
  MOD_SERV_OLED_Manager_Print_Text(" Relay Err Motor",15,12);
  MOD_SERV_OLED_Manager_Print_Text(" Bat:",15,22);
  MOD_SERV_OLED_Manager_Print_Text(String(Minimum_Recorded_Battery_U8 )+="%",43,22);
  Serial.print("OLED_cursor_Position:");
  Serial.println(OLED_cursor_Position);
   
  switch(OLED_cursor_Position)
  {
    case DIAG_MENU_E  : {MOD_SERV_OLED_Manager_Print_Text(">",15,2);  }break;
    case RELAY_MENU_E : {MOD_SERV_OLED_Manager_Print_Text(">",15,12); }break;
    case POWER_MENU_E : {MOD_SERV_OLED_Manager_Print_Text(">",50,2);  }break;
    case ERROR_MENU_E : {MOD_SERV_OLED_Manager_Print_Text(">",50,12); }break;
    case ANIM_MENU_E  : {MOD_SERV_OLED_Manager_Print_Text(">",75,2);  }break;
    case MOTOR_MENU_E : {MOD_SERV_OLED_Manager_Print_Text(">",75,12); }break;
  }
}

void MOD_SERV_OLED_Anmeter_Screen()
{

  display.clearDisplay();
  MOD_SERV_OLED_Load_Base_UI();
  MOD_SERV_OLED_Manager_Print_Text((String(Ammeter_Results_U8[0]/10)+"."+String(Ammeter_Results_U8[0]%10)+="A | "+String(Ammeter_Results_U8[1]/10)+"."+String(Ammeter_Results_U8[1]%10)+"A | "+String(Ammeter_Results_U8[2]/10)+"."+String(Ammeter_Results_U8[2]%10)+"A"),15,2);  
  MOD_SERV_OLED_Manager_Print_Text((String(Ammeter_Results_U8[3]/10)+"."+String(Ammeter_Results_U8[3]%10)+="A | "+String(Ammeter_Results_U8[4]/10)+"."+String(Ammeter_Results_U8[4]%10)+"A | "+String(Ammeter_Results_U8[5]/10)+"."+String(Ammeter_Results_U8[5]%10)+"A"),15,12);
  MOD_SERV_OLED_Manager_Print_Text((String(Ammeter_Results_U8[6]/10)+"."+String(Ammeter_Results_U8[6]%10)+="A | "+String(Ammeter_Results_U8[7]/10)+"."+String(Ammeter_Results_U8[7]%10)+"A | "+String(Ammeter_Results_U8[8]/10)+"."+String(Ammeter_Results_U8[8]%10)+"A"),15,22);
  OLED_Timer_U16=20;
 // Serial.println("Anm update");
}

void MOD_SERV_OLED_Diagnostic_Screen()// prototype  is active but does not do anything  significant at this moment
{
  unsigned int wADC;
  double temp;
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC
  delay(20);            // wait for voltages to become stable. this needs to be moved to task
  ADCSRA |= _BV(ADSC);  // Start the ADC
  while (bit_is_set(ADCSRA,ADSC));
  wADC = ADCW;
  temp = (wADC+45) / 1.22;

  // extern uint16_t __heap_start;
  // extern uint16_t *__brkval;
  // extern uint16_t __data_start;
  // extern uint16_t __data_end;
  // extern uint16_t __bss_start;
  // extern uint16_t __bss_end;

  // uint16_t  Mem_Heap_Size_U16;
  // uint16_t  Mem_Stack_Size_U16;
  // uint16_t  Mem_Data_Size_U16;
  // uint16_t  Mem_BSS_Size_U16;

  // Mem_Stack_Size_U16 = ( (uint16_t)RAMEND      - (uint16_t)SP            );
  // Mem_Data_Size_U16  = ( (uint16_t)&__data_end - (uint16_t)&__data_start );
  // Mem_BSS_Size_U16   = ( (uint16_t)&__bss_end  - (uint16_t)&__bss_start  );

  // if   (__brkval == NULL)  Mem_Heap_Size_U16=0;
  // else                     Mem_Heap_Size_U16=((uint16_t)__brkval - (uint16_t)&__heap_start);



  uint32_t Ammeter_total_U32;
  float time_remainig;
  Ammeter_total_U32=0;
  for (uint32_t counter=0;counter<9;counter++)
  {
  Ammeter_total_U32+=Ammeter_Results_U8[counter];
  }
  Ammeter_total_U32+=800;
  time_remainig=(float)(285*Minimum_Recorded_Battery_U8/Ammeter_total_U32)*60;

  
  display.clearDisplay();
  MOD_SERV_OLED_Load_Base_UI();
  MOD_SERV_OLED_Manager_Print_Text("Core Temp:",15,2);
  MOD_SERV_OLED_Manager_Print_Text(String(temp),75,2);
  MOD_SERV_OLED_Manager_Print_Text("Amperage:",15,12);
  MOD_SERV_OLED_Manager_Print_Text(String(Ammeter_total_U32)+"mAh",70,12);
  MOD_SERV_OLED_Manager_Print_Text("Bat Left:",15,22);  
  MOD_SERV_OLED_Manager_Print_Text(String((int)time_remainig)+"m",70,22);
}

void MOD_SERV_OLED_Relay_Control_Screen()
{
  
  display.clearDisplay();
  MOD_SERV_OLED_Load_Base_UI();
  MOD_SERV_OLED_Manager_Print_Text(" R1:   R4:   R7:",15,2);  
  MOD_SERV_OLED_Manager_Print_Text(" R2:   R5:   R8:",15,12);
  MOD_SERV_OLED_Manager_Print_Text(" R3:   R6:   R9:",15,22);

  uint8_t Matrix_X;
  uint8_t Matrix_Y;
  uint8_t Tail_Counter;

  Tail_Counter=0;
  for(Matrix_Y=0;Matrix_Y<3;Matrix_Y++)
   {
    for(Matrix_X=0;Matrix_X<3;Matrix_X++)
    {
     if (Tail_Counter!=10)
      {
	   if (Tail_Active_AU8[Tail_Counter]==true )
        {
         MOD_SERV_OLED_Manager_Print_Text("O",38+Matrix_Y*36,2+10*Matrix_X);
        }
       else
        {
         MOD_SERV_OLED_Manager_Print_Text(" ",38+Matrix_Y*36,2+10*Matrix_X);
        }
      }
     Tail_Counter++;
    }
   }
  
 switch(OLED_cursor_Position)
 {
  case 1 :{MOD_SERV_OLED_Manager_Print_Text(">",15,2);  }break;
  case 2 :{MOD_SERV_OLED_Manager_Print_Text(">",15,12); }break;
  case 3 :{MOD_SERV_OLED_Manager_Print_Text(">",15,22); }break;
  case 4 :{MOD_SERV_OLED_Manager_Print_Text(">",50,2);  }break;
  case 5 :{MOD_SERV_OLED_Manager_Print_Text(">",50,12); }break;
  case 6 :{MOD_SERV_OLED_Manager_Print_Text(">",50,22); }break;
  case 7 :{MOD_SERV_OLED_Manager_Print_Text(">",85,2);  }break;
  case 8 :{MOD_SERV_OLED_Manager_Print_Text(">",85,12); }break;
  case 9 :{MOD_SERV_OLED_Manager_Print_Text(">",85,22); }break;
 }
}
  
void MOD_SERV_OLED_Animation_Control_Screen()
{
  
  display.clearDisplay();
  MOD_SERV_OLED_Load_Base_UI();
  MOD_SERV_OLED_Manager_Print_Text(" B1  B4  B7  B10",15,2);  
  MOD_SERV_OLED_Manager_Print_Text(" R2  B5  B8  B11",15,12);
  MOD_SERV_OLED_Manager_Print_Text(" R3  B6  B9  B12",15,22);

  switch(OLED_cursor_Position)
  {
    case 1  :{MOD_SERV_OLED_Manager_Print_Text(">",15,2);  }break;
    case 2  :{MOD_SERV_OLED_Manager_Print_Text(">",15,12); }break;
    case 3  :{MOD_SERV_OLED_Manager_Print_Text(">",15,22); }break;
    case 4  :{MOD_SERV_OLED_Manager_Print_Text(">",39,2);  }break;
    case 5  :{MOD_SERV_OLED_Manager_Print_Text(">",39,12); }break;
    case 6  :{MOD_SERV_OLED_Manager_Print_Text(">",39,22); }break;
    case 7  :{MOD_SERV_OLED_Manager_Print_Text(">",63,2);  }break;
    case 8  :{MOD_SERV_OLED_Manager_Print_Text(">",63,12); }break;
    case 9  :{MOD_SERV_OLED_Manager_Print_Text(">",63,22); }break;
    case 10 :{MOD_SERV_OLED_Manager_Print_Text(">",87,2);  }break;
    case 11 :{MOD_SERV_OLED_Manager_Print_Text(">",87,12); }break;
    case 12 :{MOD_SERV_OLED_Manager_Print_Text(">",87,22); }break;
  }
}
  
void MOD_SERV_OLED_Error_Screen()
{
  
  display.clearDisplay();
  MOD_SERV_OLED_Load_Base_UI(); 

  if( Error_flag_ABO[OVER_CURRENT_PROTECTION_7_E]   ==true) MOD_SERV_OLED_Manager_Print_Text("           R7",15,2);
  if( Error_flag_ABO[OVER_CURRENT_PROTECTION_4_E]   ==true) MOD_SERV_OLED_Manager_Print_Text("        R4",15,2);  
  if( Error_flag_ABO[OVER_CURRENT_PROTECTION_1_E]   ==true) MOD_SERV_OLED_Manager_Print_Text("     R1",15,2);  
  if( Error_flag_ABO[LEFT_HEART_ELECTRODE_ISSUE_E]  ==true) MOD_SERV_OLED_Manager_Print_Text(" HRL",15,2);  

  if( Error_flag_ABO[OVER_CURRENT_PROTECTION_8_E]   ==true) MOD_SERV_OLED_Manager_Print_Text("           R8",15,12);  
  if( Error_flag_ABO[OVER_CURRENT_PROTECTION_5_E]   ==true) MOD_SERV_OLED_Manager_Print_Text("        R5",15,12);  
  if( Error_flag_ABO[OVER_CURRENT_PROTECTION_2_E]   ==true) MOD_SERV_OLED_Manager_Print_Text("     R2",15,12);  
  if( Error_flag_ABO[RIGHT_HEART_ELECTRODE_ISSUE_E] ==true) MOD_SERV_OLED_Manager_Print_Text(" HRR",15,12); 

  if( Error_flag_ABO[OVER_CURRENT_PROTECTION_9_E]   ==true) MOD_SERV_OLED_Manager_Print_Text("           R9",15,22);  
  if( Error_flag_ABO[OVER_CURRENT_PROTECTION_6_E]   ==true) MOD_SERV_OLED_Manager_Print_Text("        R6",15,22);  
  if( Error_flag_ABO[OVER_CURRENT_PROTECTION_3_E]   ==true) MOD_SERV_OLED_Manager_Print_Text("     R3",15,22);  
  if( Error_flag_ABO[LOW_BATTERY_WARNING_E]         ==true) MOD_SERV_OLED_Manager_Print_Text(" BAT",15,22);  


  switch(OLED_cursor_Position)
  {
    case 1  :{MOD_SERV_OLED_Manager_Print_Text(">",15,2);  }break;
    case 2  :{MOD_SERV_OLED_Manager_Print_Text(">",15,12); }break;
    case 3  :{MOD_SERV_OLED_Manager_Print_Text(">",15,22); }break;
    case 4  :{MOD_SERV_OLED_Manager_Print_Text(">",39,2);  }break;
    case 5  :{MOD_SERV_OLED_Manager_Print_Text(">",39,12); }break;
    case 6  :{MOD_SERV_OLED_Manager_Print_Text(">",39,22); }break;
    case 7  :{MOD_SERV_OLED_Manager_Print_Text(">",57,2);  }break;
    case 8  :{MOD_SERV_OLED_Manager_Print_Text(">",57,12); }break;
    case 9  :{MOD_SERV_OLED_Manager_Print_Text(">",57,22); }break;
    case 10 :{MOD_SERV_OLED_Manager_Print_Text(">",75,2);  }break;
    case 11 :{MOD_SERV_OLED_Manager_Print_Text(">",75,12); }break;
    case 12 :{MOD_SERV_OLED_Manager_Print_Text(">",75,22); }break;
  }
}

void MOD_SERV_OLED_Motor_Screen()
{
  display.clearDisplay();
  MOD_SERV_OLED_Load_Base_UI();
  MOD_SERV_OLED_Manager_Print_Text(" M+  I+  X+  C+",15,2);  
  MOD_SERV_OLED_Manager_Print_Text(" M-  I-  X-  C-",15,12);
  MOD_SERV_OLED_Manager_Print_Text(" M:  V:    ",15,22);
  MOD_SERV_OLED_Manager_Print_Text(String(MOD_SERV_Motor_Manager_Get_Current_Angle(Motor_Menu_Selected_Motor)),73,22);
  MOD_SERV_OLED_Manager_Print_Text(String(Motor_Menu_Selected_Motor),30,22);

 switch(OLED_cursor_Position)
  {
    case 1  :{MOD_SERV_OLED_Manager_Print_Text(">",15,2);  }break;
    case 2  :{MOD_SERV_OLED_Manager_Print_Text(">",15,12); }break;
    case 3  :{MOD_SERV_OLED_Manager_Print_Text(">",39,2);  }break;
    case 4  :{MOD_SERV_OLED_Manager_Print_Text(">",39,12); }break;
    case 5  :{MOD_SERV_OLED_Manager_Print_Text(">",63,2);  }break;
    case 6  :{MOD_SERV_OLED_Manager_Print_Text(">",63,12); }break;
    case 7  :{MOD_SERV_OLED_Manager_Print_Text(">",87,2);  }break;
    case 8  :{MOD_SERV_OLED_Manager_Print_Text(">",87,12); }break;
  }
}
  
/************************** OLED MANAGER GENERAL****************************************/

void MOD_SERV_OLED_Manager_Init() 
{
 Serial.println("STARTING OLED INITIALIZATION");
 
  Wire.begin();
  Wire.beginTransmission(0x3C);
  OLED_Connection_Status_U8 = Wire.endTransmission();

    Wire.beginTransmission(0x3C);
  OLED_Connection_Status_U8 = Wire.endTransmission();
  
  Serial.println("OLED_Connection_Status_U8");
   if(OLED_Connection_Status_U8)
   {
   Serial.println("OLED DISPLAY INITIALIZATION FAILED WITH ERROR CODE: "+ (String)OLED_Connection_Status_U8);
   //digitalWrite(BUZZER_PIN,HIGH);
  // delay (500); 
  // digitalWrite(BUZZER_PIN,LOW);
  // delay (500);
   }
  else
   {
	  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE); // Draw white text
    MOD_SERV_OLED_Manager_Draw_Bitmap(Fumo,SCREEN_WIDTH,SCREEN_HEIGHT);
    display.display();
    MOD_SERV_OLED_Screen_Manager();
    OLED_Update_BO = true;
  }
}

void MOD_SERV_OLED_Load_Base_UI()
{  
 display.clearDisplay();
 MOD_SERV_OLED_Manager_Draw_Bitmap(UI_test,SCREEN_WIDTH,SCREEN_HEIGHT);
 
 display.drawLine(3, 25, 3,25-Minimum_Recorded_Battery_U8 /5, SSD1306_WHITE);
 display.drawLine(4, 25, 4,25-Minimum_Recorded_Battery_U8 /5, SSD1306_WHITE);
 display.drawLine(5, 25, 5,25-Minimum_Recorded_Battery_U8 /5, SSD1306_WHITE);
 display.drawLine(6, 25, 6,25-Minimum_Recorded_Battery_U8 /5, SSD1306_WHITE);
 display.drawLine(7, 25, 7,25-Minimum_Recorded_Battery_U8 /5, SSD1306_WHITE);
 display.drawLine(8, 25, 8,25-Minimum_Recorded_Battery_U8 /5, SSD1306_WHITE);
 display.drawLine(9, 25, 9,25-Minimum_Recorded_Battery_U8 /5, SSD1306_WHITE);
}

void MOD_SERV_OLED_Manager_Print_Text(String Message_ST,uint8_t  X_Cord_U8 ,uint8_t  Y_Cord_U8 ) 
{
  display.setCursor(X_Cord_U8 ,Y_Cord_U8 );
  display.print(Message_ST);
}

void MOD_SERV_OLED_Manager_Draw_Bitmap(const unsigned char Bitmap[],uint8_t  bitmap_Width_U8 ,uint8_t  bitmap_Height_U8 )
{
  display.drawBitmap((display.width()  - bitmap_Width_U8  ) / 2,(display.height() - bitmap_Height_U8 ) / 2,Bitmap, bitmap_Width_U8 , bitmap_Height_U8 , 1);
}

void MOD_SERV_OLED_Screen_Manager()
{
 switch(current_menu_EN)
  {
    case MAIN_MENU_E  :{MOD_SERV_OLED_Main_Screen();              }break;
    case POWER_MENU_E :{MOD_SERV_OLED_Anmeter_Screen();           }break;
    case DIAG_MENU_E  :{MOD_SERV_OLED_Diagnostic_Screen();        }break;
    case RELAY_MENU_E :{MOD_SERV_OLED_Relay_Control_Screen();     }break;
    case ANIM_MENU_E  :{MOD_SERV_OLED_Animation_Control_Screen(); }break;
    case ERROR_MENU_E :{MOD_SERV_OLED_Error_Screen();             }break;
    case MOTOR_MENU_E :{MOD_SERV_OLED_Motor_Screen();             }break;
  }
}  

void MOD_SERV_OLED_Input_up()
{
 OLED_cursor_Position--;
 if (OLED_cursor_Position==0)
   { 
   OLED_cursor_Position=Max_Cursor;
   }
   Serial.print("OLED_cursor_Position:");
   Serial.println(OLED_cursor_Position);
}
  
void MOD_SERV_OLED_Input_down()
{
 OLED_cursor_Position++;
 if (OLED_cursor_Position>Max_Cursor)
   { 
   OLED_cursor_Position=1;
   }
      Serial.print("OLED_cursor_Position:");
   Serial.println(OLED_cursor_Position);
}

void MOD_SERV_OLED_input_Manager(GUI_INPUT_LIST Received_input_EN)
{

  switch(current_menu_EN)
  {case MAIN_MENU_E :
   {switch(Received_input_EN)
     {
       case UP_INPUT_E   :{MOD_SERV_OLED_Input_up()  ;}break;
       case DOWN_INPUT_E :{MOD_SERV_OLED_Input_down();}break;
       case ENTER_INPUT_E :
       {
        switch(OLED_cursor_Position)
        {
         case DIAG_MENU_E  : {current_menu_EN=DIAG_MENU_E;  }break;
         case POWER_MENU_E : {current_menu_EN=POWER_MENU_E; }break;
         case ANIM_MENU_E  : {current_menu_EN=ANIM_MENU_E;  Max_Cursor=ANIM_MENU_MAX_CURSOR;  OLED_cursor_Position=1;}break;
         case ERROR_MENU_E : {current_menu_EN=ERROR_MENU_E; Max_Cursor=ERROR_MENU_MAX_CURSOR; OLED_cursor_Position=1;}break;
         case MOTOR_MENU_E : {current_menu_EN=MOTOR_MENU_E; Max_Cursor=MOTOR_MENU_MAX_CURSOR; OLED_cursor_Position=1;}break;
         case RELAY_MENU_E : {current_menu_EN=RELAY_MENU_E; Max_Cursor=RELAY_MENU_MAX_CURSOR; OLED_cursor_Position=1;}break;
         }
       }break;
       case EXIT_INPUT_E : {;}break;
      }
   }break;
   
   case DIAG_MENU_E :
   {switch(Received_input_EN)
     {
       case UP_INPUT_E    :{;}break;// nothing
       case DOWN_INPUT_E  :{;}break;// nothing
       case ENTER_INPUT_E :{;}break;// nothing
       case EXIT_INPUT_E  :{current_menu_EN=MAIN_MENU_E;Max_Cursor=MAIN_MENU_MAX_CURSOR;OLED_cursor_Position=1;}break;
      }
   }break;

   case RELAY_MENU_E :
   {switch(Received_input_EN)
     {
       case UP_INPUT_E    :{MOD_SERV_OLED_Input_up()  ;}break;
       case DOWN_INPUT_E  :{MOD_SERV_OLED_Input_down();}break;
       case ENTER_INPUT_E :{MOD_MCAL_Relay_Toggle(OLED_cursor_Position-1);}break;// nothing
       case EXIT_INPUT_E  :{current_menu_EN=MAIN_MENU_E;Max_Cursor=MAIN_MENU_MAX_CURSOR;OLED_cursor_Position=1;}break;
      }
   }break;
   
   case POWER_MENU_E :
   {switch(Received_input_EN)
     {
       case UP_INPUT_E    :{;}break;// nothing
       case DOWN_INPUT_E  :{;}break;// nothing
       case ENTER_INPUT_E :{;}break;// nothing
       case EXIT_INPUT_E  :{current_menu_EN=MAIN_MENU_E;Max_Cursor=MAIN_MENU_MAX_CURSOR;OLED_cursor_Position=1;}break;
      }
   }break;
   
   case ERROR_MENU_E :
   {switch(Received_input_EN)
     {
       case UP_INPUT_E    :{MOD_SERV_OLED_Input_up()  ;}break;
       case DOWN_INPUT_E  :{MOD_SERV_OLED_Input_down();}break;
       case ENTER_INPUT_E :{MOD_SERV_Error_Reset((ERROR_CODES_ENUM)OLED_cursor_Position);}break;// nothing
       case EXIT_INPUT_E  :{current_menu_EN=MAIN_MENU_E;Max_Cursor=MAIN_MENU_MAX_CURSOR;OLED_cursor_Position=1;}break;
      }
   }break;
   
      case  ANIM_MENU_E :
   {switch(Received_input_EN)
     {
       case UP_INPUT_E    :{MOD_SERV_OLED_Input_up()  ;}break;
       case DOWN_INPUT_E  :{MOD_SERV_OLED_Input_down();}break;
       case ENTER_INPUT_E :{;}break;// to be implemented
       case EXIT_INPUT_E  :{current_menu_EN=MAIN_MENU_E;Max_Cursor=MAIN_MENU_MAX_CURSOR;OLED_cursor_Position=1;}break;
      }
   }


 case  MOTOR_MENU_E :
   {switch(Received_input_EN)
     {
       case UP_INPUT_E    :{MOD_SERV_OLED_Input_up()  ;}break;
       case DOWN_INPUT_E  :{MOD_SERV_OLED_Input_down();}break;
       case ENTER_INPUT_E :
                           {
                            switch (OLED_cursor_Position)
                            { 
                              case MOTOR_ID_UP_E           : {Motor_Menu_Selected_Motor=min(Motor_Menu_Selected_Motor+1,Motor_ID_U8);}break;
                              case MOTOR_ID_DOWN_E         : {Motor_Menu_Selected_Motor=max(Motor_Menu_Selected_Motor-1,0);}break;
                              case POSITION_DIGIT_1_UP_E   : { MOD_SERV_Motor_Manager_Adjust_Angle_From_GUI(Motor_Menu_Selected_Motor , 1);}break;
                              case POSITION_DIGIT_1_DOWN_E : { MOD_SERV_Motor_Manager_Adjust_Angle_From_GUI(Motor_Menu_Selected_Motor , -1);}break;
                              case POSITION_DIGIT_2_UP_E   : { MOD_SERV_Motor_Manager_Adjust_Angle_From_GUI(Motor_Menu_Selected_Motor ,10);}break;
                              case POSITION_DIGIT_2_DOWN_E : { MOD_SERV_Motor_Manager_Adjust_Angle_From_GUI(Motor_Menu_Selected_Motor ,-10);}break;
                              case POSITION_DIGIT_3_UP_E   : { MOD_SERV_Motor_Manager_Adjust_Angle_From_GUI(Motor_Menu_Selected_Motor ,100);}break;
                              case POSITION_DIGIT_3_DOWN_E : { MOD_SERV_Motor_Manager_Adjust_Angle_From_GUI(Motor_Menu_Selected_Motor ,-100);}break;
                              
                            }
        
                            }break;// nothing
       case EXIT_INPUT_E  :{current_menu_EN=MAIN_MENU_E;Max_Cursor=MAIN_MENU_MAX_CURSOR;OLED_cursor_Position=1;}break;
      }
   }
   
  }
  OLED_Update_BO = true;
  Serial.print("O");
}

//*****************IMU MANAGER*****************************
 
void MOD_MCAL_IMU_Configuration(uint8_t I2C_address_U8)
{   
  Wire.begin();                           //begin the wire comunication  
  Wire.beginTransmission(I2C_address_U8);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(I2C_address_U8);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(I2C_address_U8);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true); 
}

void MOD_MCAL_IMU_Accel_Measure_Error(uint8_t I2C_address_U8,uint8_t IMU_ID)
{   
  uint16_t Repetitions_Counter_U16;
  float Raw_X_F, Raw_Y_F, Raw_Z_F;
  
  for(Repetitions_Counter_U16=0; Repetitions_Counter_U16<IMU_REPETITIONS; Repetitions_Counter_U16++)
    {
      Wire.beginTransmission(I2C_address_U8);
      Wire.write(0x3B);                       
      Wire.endTransmission(false);
      Wire.requestFrom((int)I2C_address_U8,6,true); 
      
      Raw_X_F=(Wire.read()<<8|Wire.read())/4096.0 ; 
      Raw_Y_F=(Wire.read()<<8|Wire.read())/4096.0 ;
      Raw_Z_F=(Wire.read()<<8|Wire.read())/4096.0 ;
     
      Accel_error[IMU_ID].X_Axis_F = Accel_error[IMU_ID].X_Axis_F + ((atan((Raw_Y_F)/sqrt(pow((Raw_X_F),2) + pow((Raw_Z_F),2)))*Rad_to_Deg));
      Accel_error[IMU_ID].Y_Axis_F = Accel_error[IMU_ID].Y_Axis_F + ((atan(-1*(Raw_X_F)/sqrt(pow((Raw_Y_F),2) + pow((Raw_Z_F),2)))*Rad_to_Deg)); 
      
      if(Repetitions_Counter_U16==IMU_REPETITIONS-1U)
      {
        Accel_error[IMU_ID].X_Axis_F = Accel_error[IMU_ID].X_Axis_F/IMU_REPETITIONS;
        Accel_error[IMU_ID].Y_Axis_F = Accel_error[IMU_ID].Y_Axis_F/IMU_REPETITIONS;
      }
    }
  
}

void MOD_MCAL_IMU_Gyro_Measure_Error(uint8_t I2C_address_U8,uint8_t IMU_ID)
{   
    uint16_t Repetitions_Counter_U16;
   float Raw_X_F, Raw_Y_F;

    for(Repetitions_Counter_U16=0; Repetitions_Counter_U16<IMU_REPETITIONS; Repetitions_Counter_U16++)
    {
      Wire.beginTransmission(I2C_address_U8);            
      Wire.write(0x43);                        
      Wire.endTransmission(false);
      Wire.requestFrom((int)I2C_address_U8,4,true);           
         
      Raw_X_F=Wire.read()<<8|Wire.read();     
      Raw_Y_F=Wire.read()<<8|Wire.read();
   
      Gyro_error[IMU_ID].X_Axis_F = Gyro_error[IMU_ID].X_Axis_F + (Raw_X_F/32.8); 
      Gyro_error[IMU_ID].Y_Axis_F = Gyro_error[IMU_ID].Y_Axis_F + (Raw_Y_F/32.8);
    
      if(Repetitions_Counter_U16==IMU_REPETITIONS-1)
      {
        Gyro_error[IMU_ID].X_Axis_F = Gyro_error[IMU_ID].X_Axis_F/IMU_REPETITIONS;
        Gyro_error[IMU_ID].Y_Axis_F = Gyro_error[IMU_ID].Y_Axis_F/IMU_REPETITIONS;
      }
    }
}

void MOD_MCAL_IMU_Gyro_Read_Data(uint8_t I2C_address_U8,uint8_t IMU_ID)
{
    float Raw_X_F, Raw_Y_F;

    Wire.beginTransmission(I2C_address_U8);           
    Wire.write(0x43);                      
    Wire.endTransmission(false);
    Wire.requestFrom((int)I2C_address_U8,4,true);          
        
    Raw_X_F=Wire.read()<<8|Wire.read();     
    Raw_Y_F=Wire.read()<<8|Wire.read();

    Raw_X_F = (Raw_X_F/32.8) - Gyro_error[IMU_ID].X_Axis_F; 
    Raw_Y_F = (Raw_Y_F/32.8) - Gyro_error[IMU_ID].Y_Axis_F;
    
     Gyro_Data[IMU_ID].X_Axis_F = Raw_X_F*elapsedTime;
     Gyro_Data[IMU_ID].Y_Axis_F = Raw_Y_F*elapsedTime;
   
  
}

void MOD_MCAL_IMU_Accel_Read_Data(uint8_t I2C_address_U8,uint8_t IMU_ID)
{
  float Raw_X_F, Raw_Y_F, Raw_Z_F;
  float Measured_X_F,Measured_Y_F;
  
  Wire.beginTransmission(I2C_address_U8);    
  Wire.write(0x3B);                 
  Wire.endTransmission(false);      
  Wire.requestFrom((int)I2C_address_U8,6,true);    


  Raw_X_F=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
  Raw_Y_F=(Wire.read()<<8|Wire.read())/4096.0 ;
  Raw_Z_F=(Wire.read()<<8|Wire.read())/4096.0 ; 

  Measured_X_F = (atan((Raw_Y_F)/sqrt(pow((Raw_X_F),2) + pow((Raw_Z_F),2)))*Rad_to_Deg) - Accel_error[IMU_ID].X_Axis_F; 
  Measured_Y_F = (atan(-1*(Raw_X_F)/sqrt(pow((Raw_Y_F),2) + pow((Raw_Z_F),2)))*Rad_to_Deg) - Accel_error[IMU_ID].Y_Axis_F; ;    

  Accel_Data[IMU_ID].X_Axis_F = 0.98 *(Accel_Data[IMU_ID].X_Axis_F + Gyro_Data[IMU_ID].X_Axis_F) + 0.02*Measured_X_F;
  Accel_Data[IMU_ID].Y_Axis_F = 0.98 *(Accel_Data[IMU_ID].Y_Axis_F + Gyro_Data[IMU_ID].Y_Axis_F) + 0.02*Measured_Y_F;
      
}

float MOD_SERV_IMU_Get_Angle()
{
  float measured_angle;

  timePrev = time;                        
  time = millis();                       
  elapsedTime = (time - timePrev) / 1000; 
  
  MOD_MCAL_IMU_Gyro_Read_Data(IMU_TORSO_ADRESS_ID,IMU_TORSO_ID);
  MOD_MCAL_IMU_Accel_Read_Data(IMU_TORSO_ADRESS_ID,IMU_TORSO_ID);
  MOD_MCAL_IMU_Gyro_Read_Data(IMU_HEAD_ADRESS_ID,IMU_HEAD_ID);
  MOD_MCAL_IMU_Accel_Read_Data(IMU_HEAD_ADRESS_ID,IMU_HEAD_ID);

  measured_angle=Accel_Data[IMU_TORSO_ID].Y_Axis_F-Accel_Data[IMU_HEAD_ID].Y_Axis_F;
  return measured_angle;
} 

/***************ANIMATION SYSTEM******************/

void MOD_SERV_Animation_Add_Into_Queue(uint8_t Motor_ID_U8_Param,int16_t Target_angle_S16_Param, uint16_t Animation_Duration_U16_Param)
{
  uint8_t  Target_Position_U8  = Animation_Queue_Positions_U8 [Motor_ID_U8_Param][ANIMATION_QUEUE_NEXT_POSITION_EN];

    if (Target_Position_U8  == ANIMATION_BUFFER_SIZE-1) Animation_Queue_Positions_U8[Motor_ID_U8_Param][ANIMATION_QUEUE_NEXT_POSITION_EN]=0;
    else                                              Animation_Queue_Positions_U8[Motor_ID_U8_Param][ANIMATION_QUEUE_NEXT_POSITION_EN]++;
  
  Animation_Queue_Data_AU16 [Motor_ID_U8_Param] [Target_Position_U8] [ANIMATION_DATA_TARGET_ANGLE_EN]       = MOD_SERV_Motor_Angle_Converter(Target_angle_S16_Param,Motor_ID_U8_Param);
  Animation_Queue_Data_AU16 [Motor_ID_U8_Param] [Target_Position_U8] [ANIMATION_DATA_ANIMATION_DURATION_EN] = Animation_Duration_U16_Param;

  //Serial.println("+Anim M:"+(String)Motor_ID_U8_Param +" T:"+ Animation_Queue_Data_AU16 [Motor_ID_U8_Param] [Target_Position_U8] [ANIMATION_DATA_TARGET_ANGLE_EN]+" D:"+Animation_Queue_Data_AU16 [Motor_ID_U8_Param] [Target_Position_U8] [ANIMATION_DATA_ANIMATION_DURATION_EN]+" P:"+(String)Target_Position_U8);

}

void MOD_SERV_Animation_Load_From_Queue(uint8_t  Motor_ID_U8_Param)
{

 uint16_t  Target_angle_U16 ;
 uint16_t  Anim_Duration_U16 ;
 
    Motor_Animation_Loaded_ABO[Motor_ID_U8_Param ]=true;//this marks that a new animation has been loaded 
    

    Target_angle_U16    =Animation_Queue_Data_AU16 [Motor_ID_U8_Param ] [Animation_Queue_Positions_U8 [Motor_ID_U8_Param ][ANIMATION_QUEUE_CURRENT_POSITION_EN]][ANIMATION_DATA_TARGET_ANGLE_EN];
    Anim_Duration_U16   =Animation_Queue_Data_AU16 [Motor_ID_U8_Param ] [Animation_Queue_Positions_U8 [Motor_ID_U8_Param ][ANIMATION_QUEUE_CURRENT_POSITION_EN]][ANIMATION_DATA_ANIMATION_DURATION_EN];

   
    Motor_Animation_Status_AU16[Motor_ID_U8_Param].Step_size   = (int16_t)((int16_t)Target_angle_U16 -(int16_t)Motor_Animation_Status_AU16[Motor_ID_U8_Param].Motor_Position) /(int16_t)Anim_Duration_U16 ;
    Motor_Animation_Status_AU16[Motor_ID_U8_Param].Step_Count   = Anim_Duration_U16 /ANIMATION_SPPED;// to be properly measured

  
 // if(Motor_ID_U8_Param==10)Serial.println("REMOVE P:"+(String)Animation_Queue_Positions_U8 [Motor_ID_U8_Param ][ANIMATION_QUEUE_CURRENT_POSITION_EN]+" V:"+(String)Target_angle_U16);

    Animation_Queue_Positions_U8 [Motor_ID_U8_Param ][ANIMATION_QUEUE_CURRENT_POSITION_EN]++; 


    if (Animation_Queue_Positions_U8 [Motor_ID_U8_Param][ANIMATION_QUEUE_CURRENT_POSITION_EN] == ANIMATION_BUFFER_SIZE)
        Animation_Queue_Positions_U8 [Motor_ID_U8_Param][ANIMATION_QUEUE_CURRENT_POSITION_EN] = 0;     
}

void MOD_SERV_Animation_Run(uint8_t  Motor_ID_U8_Param )
{
  Motor_Animation_Status_AU16[Motor_ID_U8_Param].Motor_Position+= Motor_Animation_Status_AU16[Motor_ID_U8_Param].Step_size;
  Motor_Animation_Status_AU16[Motor_ID_U8_Param].Step_Count--;

 //Serial.println("M:"+(String)Motor_ID_U8_Param +" T:"+(String)Motor_Animation_Status_AU16[Motor_ID_U8_Param].Motor_Position+" S:"+Motor_Animation_Status_AU16[Motor_ID_U8_Param].Step_Count);
 // if(Motor_ID_U8_Param==10) Serial.println("M ID:"+(String)Motor_ID_U8_Param+" CH:"+(String)Motor_Control_S[Motor_ID_U8_Param].Output_mode_U8+" P:"+Motor_Control_S[Motor_ID_U8_Param].Pin_U8+" VAL:"+(String)PCA9685_Motor_Angle_To_Be_Written_MU16[Motor_Control_S[Motor_ID_U8_Param].Output_mode_U8][Motor_Control_S[Motor_ID_U8_Param].Pin_U8]);

  if (Motor_Control_S[Motor_ID_U8_Param].Output_mode_U8 != INTERNAL_PWM_E)
  PCA9685_Motor_Angle_To_Be_Written_MU16[Motor_Control_S[Motor_ID_U8_Param].Output_mode_U8][Motor_Control_S[Motor_ID_U8_Param].Pin_U8]=Motor_Animation_Status_AU16[Motor_ID_U8_Param].Motor_Position;
  
  if (Motor_Animation_Status_AU16[Motor_ID_U8_Param].Step_Count == 0)  Motor_Animation_Loaded_ABO[Motor_ID_U8_Param]=false;
}

void MOD_APPL_Animation_Manager()
{
  bool Motor_has_Been_Adjusted_This_Cycle_BO=false;
  
  for(uint8_t Motor_ID_Count_U8 =0;Motor_ID_Count_U8 <TOTAL_MOTOR_AMMOUNT;Motor_ID_Count_U8 ++) 
  {
   if (Motor_Animation_Loaded_ABO[Motor_ID_Count_U8 ]== true) 
   {
    MOD_SERV_Animation_Run(Motor_ID_Count_U8 );
    if(Motor_has_Been_Adjusted_This_Cycle_BO==false)
    {
      Motor_has_Been_Adjusted_This_Cycle_BO=true;
      if(Loaded_Animation_Duration_U16!=0) Loaded_Animation_Duration_U16--;
    }
   }
   else if (Animation_Queue_Positions_U8 [Motor_ID_Count_U8][ANIMATION_QUEUE_CURRENT_POSITION_EN] != 
            Animation_Queue_Positions_U8 [Motor_ID_Count_U8][ANIMATION_QUEUE_NEXT_POSITION_EN   ])  
            {
             MOD_SERV_Animation_Load_From_Queue(Motor_ID_Count_U8);
            }
  }
}

/**************Transition FUNCTIONS***********/
// not used
/*********************ON STATES****************/
void MOD_SERV_State_Machine_STARTUP_ACTION()
{
	//this is a static pose, animations should be loded during entry
}
void MOD_SERV_State_Machine_IDLE_ACTION()
{
    //Serial.println("Idle Action");
  Loaded_Animation_Duration_U16+=200;
  //MOD_SERV_Animation_Add_Into_Queue(10,-20,50);
  //MOD_SERV_Animation_Add_Into_Queue(10,20,50);
  //MOD_SERV_Animation_Add_Into_Queue(11,0,50);
  //MOD_SERV_Animation_Add_Into_Queue(11,40,50);
  //MOD_SERV_Animation_Add_Into_Queue(12,-20,50);
  //MOD_SERV_Animation_Add_Into_Queue(12,20,50);
  //MOD_SERV_Animation_Add_Into_Queue(13,-20,50);
  //MOD_SERV_Animation_Add_Into_Queue(13,20,50);
  //MOD_SERV_Animation_Add_Into_Queue(14,20,50);
  //MOD_SERV_Animation_Add_Into_Queue(14,-20,50);

  MOD_SERV_Animation_Add_Into_Queue(0,64,-22);
  MOD_SERV_Animation_Add_Into_Queue(0,77,25);
  MOD_SERV_Animation_Add_Into_Queue(0,59,10);     
  MOD_SERV_Animation_Add_Into_Queue(1,70,-45);
  MOD_SERV_Animation_Add_Into_Queue(1,60,-15);
  MOD_SERV_Animation_Add_Into_Queue(1,70,-30);     
  MOD_SERV_Animation_Add_Into_Queue(2,30,-14);
  MOD_SERV_Animation_Add_Into_Queue(2,47,20);
  MOD_SERV_Animation_Add_Into_Queue(2,77,-22);
  MOD_SERV_Animation_Add_Into_Queue(2,46,0);   
  MOD_SERV_Animation_Add_Into_Queue(3,71,-31);
  MOD_SERV_Animation_Add_Into_Queue(3,129,-60);      
  MOD_SERV_Animation_Add_Into_Queue(4,50,23);
  MOD_SERV_Animation_Add_Into_Queue(4,73,5);
  MOD_SERV_Animation_Add_Into_Queue(4,31,-27);
  MOD_SERV_Animation_Add_Into_Queue(4,46,0);   
  MOD_SERV_Animation_Add_Into_Queue(5,41,-34);
  MOD_SERV_Animation_Add_Into_Queue(5,81,25);
  MOD_SERV_Animation_Add_Into_Queue(5,78,10);    
  MOD_SERV_Animation_Add_Into_Queue(6,100,-45);
  MOD_SERV_Animation_Add_Into_Queue(6,100,-30);      
  MOD_SERV_Animation_Add_Into_Queue(7,50,20);
  MOD_SERV_Animation_Add_Into_Queue(7,90,-15);
  MOD_SERV_Animation_Add_Into_Queue(7,60,10);    
  MOD_SERV_Animation_Add_Into_Queue(8,100,-25);
  MOD_SERV_Animation_Add_Into_Queue(8,100,-60);      
  MOD_SERV_Animation_Add_Into_Queue(9,50,23);
  MOD_SERV_Animation_Add_Into_Queue(9,73,5);
  MOD_SERV_Animation_Add_Into_Queue(9,31,-27);
  MOD_SERV_Animation_Add_Into_Queue(9,46,0);   
  MOD_SERV_Animation_Add_Into_Queue(10,41,-25);
  MOD_SERV_Animation_Add_Into_Queue(10,97,35);
  MOD_SERV_Animation_Add_Into_Queue(10,62,0);    
  MOD_SERV_Animation_Add_Into_Queue(11,100,-45);
  MOD_SERV_Animation_Add_Into_Queue(11,100,-30);       
  MOD_SERV_Animation_Add_Into_Queue(12,52,32);
  MOD_SERV_Animation_Add_Into_Queue(12,94,-38);
  MOD_SERV_Animation_Add_Into_Queue(12,54,0);    
  MOD_SERV_Animation_Add_Into_Queue(13,53,-17);
  MOD_SERV_Animation_Add_Into_Queue(13,83,-55);
  MOD_SERV_Animation_Add_Into_Queue(13,64,-45);    
  MOD_SERV_Animation_Add_Into_Queue(14,64,6);
  MOD_SERV_Animation_Add_Into_Queue(14,41,-10);
  MOD_SERV_Animation_Add_Into_Queue(14,25,-32);
  MOD_SERV_Animation_Add_Into_Queue(14,70,-10);  
  MOD_SERV_Animation_Add_Into_Queue(15,38,-5);
  MOD_SERV_Animation_Add_Into_Queue(15,78,27);
  MOD_SERV_Animation_Add_Into_Queue(15,54,23);
  MOD_SERV_Animation_Add_Into_Queue(15,30,15);   
  MOD_SERV_Animation_Add_Into_Queue(16,100,-7);
  MOD_SERV_Animation_Add_Into_Queue(16,100,8);       
  MOD_SERV_Animation_Add_Into_Queue(17,19,-17);
  MOD_SERV_Animation_Add_Into_Queue(17,44,16);
  MOD_SERV_Animation_Add_Into_Queue(17,70,-15);
  MOD_SERV_Animation_Add_Into_Queue(17,67,0);  
  MOD_SERV_Animation_Add_Into_Queue(18,83,-30);
  MOD_SERV_Animation_Add_Into_Queue(18,117,-55);       
  MOD_SERV_Animation_Add_Into_Queue(19,86,18);
  MOD_SERV_Animation_Add_Into_Queue(19,50,-10);
  MOD_SERV_Animation_Add_Into_Queue(19,64,0);    
  MOD_SERV_Animation_Add_Into_Queue(20,60,-15);
  MOD_SERV_Animation_Add_Into_Queue(20,70,15);
  MOD_SERV_Animation_Add_Into_Queue(20,70,0);    
  MOD_SERV_Animation_Add_Into_Queue(21,60,10);
  MOD_SERV_Animation_Add_Into_Queue(21,80,-10);
  MOD_SERV_Animation_Add_Into_Queue(21,60,0);    
  MOD_SERV_Animation_Add_Into_Queue(22,20,-10);
  MOD_SERV_Animation_Add_Into_Queue(22,49,18);
  MOD_SERV_Animation_Add_Into_Queue(22,72,-10);
  MOD_SERV_Animation_Add_Into_Queue(22,59,13);   
  MOD_SERV_Animation_Add_Into_Queue(23,41,-90);
  MOD_SERV_Animation_Add_Into_Queue(23,74,-48);
  MOD_SERV_Animation_Add_Into_Queue(23,85,-70);    
  MOD_SERV_Animation_Add_Into_Queue(24,29,-30);
  MOD_SERV_Animation_Add_Into_Queue(24,45,14);
  MOD_SERV_Animation_Add_Into_Queue(24,37,9);
  MOD_SERV_Animation_Add_Into_Queue(24,26,-15);
  MOD_SERV_Animation_Add_Into_Queue(24,63,0);
  MOD_SERV_Animation_Add_Into_Queue(25,70,-35);
  MOD_SERV_Animation_Add_Into_Queue(25,51,-5);
  MOD_SERV_Animation_Add_Into_Queue(25,79,-15);    
  MOD_SERV_Animation_Add_Into_Queue(26,40,15);
  MOD_SERV_Animation_Add_Into_Queue(26,92,-15);
  MOD_SERV_Animation_Add_Into_Queue(26,68,0);    
  MOD_SERV_Animation_Add_Into_Queue(27,55,-15);
  MOD_SERV_Animation_Add_Into_Queue(27,45,15);
  MOD_SERV_Animation_Add_Into_Queue(27,32,-12);
  MOD_SERV_Animation_Add_Into_Queue(27,41,26);
  MOD_SERV_Animation_Add_Into_Queue(27,27,0);
  MOD_SERV_Animation_Add_Into_Queue(28,30,-80);
  MOD_SERV_Animation_Add_Into_Queue(28,60,-35);
  MOD_SERV_Animation_Add_Into_Queue(28,57,-77);
  MOD_SERV_Animation_Add_Into_Queue(28,53,-65);  
  MOD_SERV_Animation_Add_Into_Queue(29,54,-20);
  MOD_SERV_Animation_Add_Into_Queue(29,33,20);
  MOD_SERV_Animation_Add_Into_Queue(29,49,-19);
  MOD_SERV_Animation_Add_Into_Queue(29,55,4);
  MOD_SERV_Animation_Add_Into_Queue(29,9,0);
  MOD_SERV_Animation_Add_Into_Queue(30,14,18);
  MOD_SERV_Animation_Add_Into_Queue(30,72,-31);
  MOD_SERV_Animation_Add_Into_Queue(30,54,16);
  MOD_SERV_Animation_Add_Into_Queue(30,60,10);   
  MOD_SERV_Animation_Add_Into_Queue(31,100,-15);
  MOD_SERV_Animation_Add_Into_Queue(31,100,-30);       
  MOD_SERV_Animation_Add_Into_Queue(32,60,-26);
  MOD_SERV_Animation_Add_Into_Queue(32,58,25);
  MOD_SERV_Animation_Add_Into_Queue(32,82,0);    
  MOD_SERV_Animation_Add_Into_Queue(33,44,-45);
  MOD_SERV_Animation_Add_Into_Queue(33,62,-75);
  MOD_SERV_Animation_Add_Into_Queue(33,94,-60);    
  MOD_SERV_Animation_Add_Into_Queue(34,14,10);
  MOD_SERV_Animation_Add_Into_Queue(34,24,-20);
  MOD_SERV_Animation_Add_Into_Queue(34,33,14);
  MOD_SERV_Animation_Add_Into_Queue(34,129,0);   
  MOD_SERV_Animation_Add_Into_Queue(35,36,32);
  MOD_SERV_Animation_Add_Into_Queue(35,53,-23);
  MOD_SERV_Animation_Add_Into_Queue(35,73,23);
  MOD_SERV_Animation_Add_Into_Queue(35,38,10);   
  MOD_SERV_Animation_Add_Into_Queue(36,100,-15);
  MOD_SERV_Animation_Add_Into_Queue(36,100,-30);       
  MOD_SERV_Animation_Add_Into_Queue(37,28,21);
  MOD_SERV_Animation_Add_Into_Queue(37,21,-40);
  MOD_SERV_Animation_Add_Into_Queue(37,57,23);
  MOD_SERV_Animation_Add_Into_Queue(37,44,-30);
  MOD_SERV_Animation_Add_Into_Queue(37,50,0);
  MOD_SERV_Animation_Add_Into_Queue(38,100,-40);
  MOD_SERV_Animation_Add_Into_Queue(38,100,-75);       
  MOD_SERV_Animation_Add_Into_Queue(39,50,23);
  MOD_SERV_Animation_Add_Into_Queue(39,73,5);
  MOD_SERV_Animation_Add_Into_Queue(39,31,-27);
  MOD_SERV_Animation_Add_Into_Queue(39,46,0);  
  MOD_SERV_Animation_Add_Into_Queue(40,55,35);
  MOD_SERV_Animation_Add_Into_Queue(40,54,-10);
  MOD_SERV_Animation_Add_Into_Queue(40,34,23);
  MOD_SERV_Animation_Add_Into_Queue(40,57,10);   
  MOD_SERV_Animation_Add_Into_Queue(41,100,-20);
  MOD_SERV_Animation_Add_Into_Queue(41,100,-30);       
  MOD_SERV_Animation_Add_Into_Queue(42,29,23);
  MOD_SERV_Animation_Add_Into_Queue(42,24,-35);
  MOD_SERV_Animation_Add_Into_Queue(42,44,23);
  MOD_SERV_Animation_Add_Into_Queue(42,60,-15);
  MOD_SERV_Animation_Add_Into_Queue(42,43,0);
  MOD_SERV_Animation_Add_Into_Queue(43,52,-45);
  MOD_SERV_Animation_Add_Into_Queue(43,68,-65);
  MOD_SERV_Animation_Add_Into_Queue(43,80,-60);    
  MOD_SERV_Animation_Add_Into_Queue(44,50,23);
  MOD_SERV_Animation_Add_Into_Queue(44,73,5);
  MOD_SERV_Animation_Add_Into_Queue(44,31,-27);
  MOD_SERV_Animation_Add_Into_Queue(44,46,0);  


}
void MOD_SERV_State_Machine_HUG_ACTION()
{
	//this is a static pose, animations should be loded during entry
}
void MOD_SERV_State_Machine_WAG_INDIV_ACTION()
{
   //TODO animation
}
void MOD_SERV_State_Machine_WAG_COMBI_ACTION()
{
	//TODO animation
}
void MOD_SERV_State_Machine_CROWD_ACTION()
{
	//this is a static pose, animations should be loded during entry
}
void MOD_SERV_State_Machine_POSE_1_ACTION()
{
	 //this is a static pose, animations should be loded during entry
}
void MOD_SERV_State_Machine_POSE_2_ACTION()
{
	//this is a static pose, animations should be loded during entry
}
void MOD_SERV_State_Machine_POSE_3_ACTION()
{
	//this is a static pose, animations should be loded during entry
}
/*********************ON ENTRY****************/
void MOD_SERV_State_Machine_STARTUP_ENTRY()
{
	//no animation here
}
void MOD_SERV_State_Machine_IDLE_ENTRY()
{
//	Serial.println("Idle Entry");
	//TODO animation
}
void MOD_SERV_State_Machine_HUG_ENTRY()
{
	//TODO animation
}
void MOD_SERV_State_Machine_WAG_INDIV_ENTRY()
{
	//TODO animation
}
void MOD_SERV_State_Machine_WAG_COMBI_ENTRY()
{
	//TODO animation
}
void MOD_SERV_State_Machine_CROWD_ENTRY()
{
	//TODO animation
}
void MOD_SERV_State_Machine_POSE_1_ENTRY()
{
	//TODO animation
}
void MOD_SERV_State_Machine_POSE_2_ENTRY()
{
	//TODO animation
}
void MOD_SERV_State_Machine_POSE_3_ENTRY()
{
	//TODO animation
}
/*********************ON EXIT****************/
void MOD_SERV_State_Machine_STARTUP_EXIT()
{
	//	Serial.println("Startup Exit");
	//no animation here
}
void MOD_SERV_State_Machine_IDLE_EXIT()
{
	//no animation here
}
void MOD_SERV_State_Machine_HUG_EXIT()
{
	//TODO animation
}
void MOD_SERV_State_Machine_WAG_INDIV_EXIT()
{
	//TODO animation
}
void MOD_SERV_State_Machine_WAG_COMBI_EXIT()
{
	//TODO animation
}
void MOD_SERV_State_Machine_CROWD_EXIT()
{
	//TODO animation
}
void MOD_SERV_State_Machine_POSE_1_EXIT()
{
	//TODO animation
}
void MOD_SERV_State_Machine_POSE_2_EXIT()
{
	//TODO animation
}
void MOD_SERV_State_Machine_POSE_3_EXIT()
{
	//TODO animation
}

/***************TAIL CONTROL********************/
void MOD_SERV_Animation_State_Machine()
{
  if (Current_State_EN==Next_State_EN )
  { 
   if (Loaded_Animation_Duration_U16==0)
   {   
    switch(Current_State_EN)
    {
    case SM_STARTUP_EN   : {MOD_SERV_State_Machine_STARTUP_ACTION();   }break;
    case SM_IDLE_EN      : {MOD_SERV_State_Machine_IDLE_ACTION();      }break;
    case SM_HUG_EN       : {MOD_SERV_State_Machine_HUG_ACTION();       }break;
    case SM_WAG_INDIV_EN : {MOD_SERV_State_Machine_WAG_INDIV_ACTION(); }break;
    case SM_WAG_COMBI_EN : {MOD_SERV_State_Machine_WAG_COMBI_ACTION(); }break;
    case SM_CROWD_EN     : {MOD_SERV_State_Machine_CROWD_ACTION();     }break;
    case SM_POSE_1_EN    : {MOD_SERV_State_Machine_POSE_1_ACTION();    }break;
    case SM_POSE_2_EN    : {MOD_SERV_State_Machine_POSE_2_ACTION();    }break;
    case SM_POSE_3_EN    : {MOD_SERV_State_Machine_POSE_3_ACTION();    }break;
    }
   }
  }
  else
  {
    switch(Current_State_EN)
    {
      case  SM_STARTUP_EN   : {MOD_SERV_State_Machine_STARTUP_EXIT();   }break;
      case  SM_IDLE_EN      : {MOD_SERV_State_Machine_IDLE_EXIT();      }break;
      case  SM_HUG_EN       : {MOD_SERV_State_Machine_HUG_EXIT();       }break;
      case  SM_WAG_INDIV_EN : {MOD_SERV_State_Machine_WAG_INDIV_EXIT(); }break;
      case  SM_WAG_COMBI_EN : {MOD_SERV_State_Machine_WAG_COMBI_EXIT(); }break;
      case  SM_CROWD_EN     : {MOD_SERV_State_Machine_CROWD_EXIT();     }break;
      case  SM_POSE_1_EN    : {MOD_SERV_State_Machine_POSE_1_EXIT();    }break;
      case  SM_POSE_2_EN    : {MOD_SERV_State_Machine_POSE_2_EXIT();    }break;
      case  SM_POSE_3_EN    : {MOD_SERV_State_Machine_POSE_3_EXIT();    }break;
    }

    switch(Next_State_EN)
    {
      case  SM_STARTUP_EN   : {MOD_SERV_State_Machine_STARTUP_ENTRY ();  }break;
      case  SM_IDLE_EN      : {MOD_SERV_State_Machine_IDLE_ENTRY();      }break;
      case  SM_HUG_EN       : {MOD_SERV_State_Machine_HUG_ENTRY();       }break;
      case  SM_WAG_INDIV_EN : {MOD_SERV_State_Machine_WAG_INDIV_ENTRY(); }break;
      case  SM_WAG_COMBI_EN : {MOD_SERV_State_Machine_WAG_COMBI_ENTRY(); }break;
      case  SM_CROWD_EN     : {MOD_SERV_State_Machine_CROWD_ENTRY();     }break;
      case  SM_POSE_1_EN    : {MOD_SERV_State_Machine_POSE_1_ENTRY();    }break;
      case  SM_POSE_2_EN    : {MOD_SERV_State_Machine_POSE_2_ENTRY();    }break;
      case  SM_POSE_3_EN    : {MOD_SERV_State_Machine_POSE_3_ENTRY();    }break;

    }
  }
 Current_State_EN=Next_State_EN;
}

/**************** Task Manager  *****************/

void MOD_MCAL_Task_init() 
{
    // this code is generated with this tool http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html  also provided in this prject as a downloadable
  cli(); 
  TCCR3A = 0; 
  TCCR3B = 0; 
  TCNT3  = 0;
  OCR3A = 39999; 
  TCCR3B |= (1 << WGM12);
  TCCR3B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  TIMSK3 |= (1 << OCIE3A);
  sei();
}


ISR(TIMER3_COMPA_vect)
{
  if(OLED_Timer_U16 !=0 ) OLED_Timer_U16--;
  if(Buzzer_timer_U8     !=0 ) Buzzer_timer_U8--;
  if(Startup_Counter_U8  !=0 ) Startup_Counter_U8--;
  
  MOD_SERV_Battery_Get_Percentage();
  MOD_SERV_Button_Matrix_Task();
  
  Motor_Update_BO=true;
  Startup_Sequence_BO=true;
  
}


/******************POWER MANAGAER******************/

uint32_t  Ammeter_Data_Queue_AU32 [TAIL_COUNT][ANMETER_MEASUREMENT_DEPTH];

void MOD_SERV_Power_Manager_Init()
{
   for (uint8_t Tail_Counter_U8=0;Tail_Counter_U8<TAIL_COUNT;Tail_Counter_U8++)
    for (uint8_t Measurement_Counter_U8=0;Measurement_Counter_U8<ANMETER_MEASUREMENT_DEPTH;Measurement_Counter_U8++)
     Ammeter_Data_Queue_AU32 [Tail_Counter_U8][Measurement_Counter_U8]=0;
}
        
void MOD_SERV_Power_Manager_Read_Ammeters()
{
  static uint16_t  Ammeter_Calibration_Value_U16[TAIL_COUNT]={0,0,0,0,0,0,0,0,0};
  static uint8_t   Ammeter_Calibration_Counter_U8=100;
  static bool      Ammeter_Calibration_Status_BO=true;
  static uint8_t   Ammeter_Queue_Position_U8   =0;
         uint8_t   Ammeter_counter_U8 ;
         uint16_t  Ammeter_Raw_Data_U16 ;
         float     Float_Calulation;

  for (Ammeter_counter_U8 =0;Ammeter_counter_U8 <TAIL_COUNT;Ammeter_counter_U8 ++)
  {
    Ammeter_Raw_Data_U16 =analogRead(Ammeter_Pin_AU8[Ammeter_counter_U8]);
    Ammeter_Data_Queue_AU32 [Ammeter_counter_U8 ][Ammeter_Queue_Position_U8 ]=min(Ammeter_Raw_Data_U16,512);

    Ammeter_Average_data_AU16 [Ammeter_counter_U8] += Ammeter_Data_Queue_AU32 [Ammeter_counter_U8 ][ Ammeter_Queue_Position_U8                               ];
    Ammeter_Average_data_AU16 [Ammeter_counter_U8] -= Ammeter_Data_Queue_AU32 [Ammeter_counter_U8 ][(Ammeter_Queue_Position_U8+1)% ANMETER_MEASUREMENT_DEPTH ];

    if (Ammeter_Calibration_Status_BO == false)
    {
    Float_Calulation=(((float)Ammeter_Calibration_Value_U16[Ammeter_counter_U8]-(float)Ammeter_Average_data_AU16 [Ammeter_counter_U8])/512)*10;
    Ammeter_Results_U8[Ammeter_counter_U8]=(uint8_t)Float_Calulation;
    if( Ammeter_Results_U8[Ammeter_counter_U8]>200)
    Ammeter_Results_U8[Ammeter_counter_U8]=0;
    }
     
   //if (Ammeter_Average_data_AU16 [Ammeter_counter_U8 ] >=ANMETER_MAX_THRESHOLD) MOD_SERV_Emergency_Shutdown(Ammeter_counter_U8 );
  }

   if      (Ammeter_Calibration_Counter_U8>0) Ammeter_Calibration_Counter_U8--;
    else if (Ammeter_Calibration_Status_BO==true)
    {
      for (uint8_t Itterator_U8 =0;Itterator_U8 <TAIL_COUNT;Itterator_U8++)  
      {
      Ammeter_Calibration_Value_U16[Itterator_U8]=Ammeter_Average_data_AU16 [Itterator_U8];
      }
      Serial.println("calibration value: "+(String)Ammeter_Calibration_Value_U16[6]);
      Ammeter_Calibration_Status_BO = false;
    }

     
         
   Ammeter_Queue_Position_U8 ++;
     
   if (Ammeter_Queue_Position_U8  > ANMETER_MEASUREMENT_DEPTH) Ammeter_Queue_Position_U8  = 0;
}

void MOD_SERV_Emergency_Shutdown(uint8_t  Param_Relay_channel_U8 )
{
  MOD_MCAL_Relay_Control (Param_Relay_channel_U8 ,false);
  // change state machine
  // send data to OLED
}

void MOD_MCAL_Relay_Control(uint8_t  Param_Relay_channel_U8 , bool Param_Status_BO)
{

  if (Relay_flip_ABO[Param_Relay_channel_U8]==true) digitalWrite(Relay_Pin_AU8 [Param_Relay_channel_U8], !Param_Status_BO);
  else                                              digitalWrite(Relay_Pin_AU8 [Param_Relay_channel_U8], Param_Status_BO);

  Tail_Active_AU8 [Param_Relay_channel_U8] = Param_Status_BO;

  Serial.println("relay: "+(String)Param_Relay_channel_U8+" Port: "+(String)Relay_Pin_AU8 [Param_Relay_channel_U8 ]+" Flip value: "+(String)Relay_flip_ABO[Param_Relay_channel_U8]+" status: "+(String)Tail_Active_AU8 [Param_Relay_channel_U8 ]);
}

void MOD_MCAL_Relay_Toggle(uint8_t  Param_Relay_channel_U8)
{
   MOD_MCAL_Relay_Control(Param_Relay_channel_U8,!Tail_Active_AU8 [Param_Relay_channel_U8]);
}

void MOD_MCAL_Relay_Init()
{  
  MOD_MCAL_Relay_Control(0,LOW);
  MOD_MCAL_Relay_Control(1,LOW);
  MOD_MCAL_Relay_Control(2,LOW);
  MOD_MCAL_Relay_Control(3,LOW);
  MOD_MCAL_Relay_Control(4,LOW);
  MOD_MCAL_Relay_Control(5,LOW);
  MOD_MCAL_Relay_Control(6,LOW);
  MOD_MCAL_Relay_Control(7,LOW);
  MOD_MCAL_Relay_Control(8,LOW);

  

PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][0]=MOD_SERV_Motor_Angle_Converter(0,0);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][1]=MOD_SERV_Motor_Angle_Converter(0,1);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][2]=MOD_SERV_Motor_Angle_Converter(0,2);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][3]=MOD_SERV_Motor_Angle_Converter(0,3);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][4]=MOD_SERV_Motor_Angle_Converter(0,4);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][5]=MOD_SERV_Motor_Angle_Converter(0,5);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][6]=MOD_SERV_Motor_Angle_Converter(0,6);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][7]=MOD_SERV_Motor_Angle_Converter(0,7);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][8]=MOD_SERV_Motor_Angle_Converter(0,8);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][9]=MOD_SERV_Motor_Angle_Converter(0,9);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][10]=MOD_SERV_Motor_Angle_Converter(0,10);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][11]=MOD_SERV_Motor_Angle_Converter(0,11);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][12]=MOD_SERV_Motor_Angle_Converter(0,12);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][13]=MOD_SERV_Motor_Angle_Converter(0,13);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][14]=MOD_SERV_Motor_Angle_Converter(0,14);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E][15]=MOD_SERV_Motor_Angle_Converter(0,15);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][0]=MOD_SERV_Motor_Angle_Converter(0,16);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][1]=MOD_SERV_Motor_Angle_Converter(0,17);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][2]=MOD_SERV_Motor_Angle_Converter(0,18);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][3]=MOD_SERV_Motor_Angle_Converter(0,19);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][4]=MOD_SERV_Motor_Angle_Converter(0,20);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][5]=MOD_SERV_Motor_Angle_Converter(0,21);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][6]=MOD_SERV_Motor_Angle_Converter(0,22);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][7]=MOD_SERV_Motor_Angle_Converter(0,23);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][8]=MOD_SERV_Motor_Angle_Converter(0,24);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][9]=MOD_SERV_Motor_Angle_Converter(0,25);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][10]=MOD_SERV_Motor_Angle_Converter(0,26);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][11]=MOD_SERV_Motor_Angle_Converter(0,27);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][12]=MOD_SERV_Motor_Angle_Converter(0,28);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][13]=MOD_SERV_Motor_Angle_Converter(0,29);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][14]=MOD_SERV_Motor_Angle_Converter(0,30);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E][15]=MOD_SERV_Motor_Angle_Converter(0,31);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][0]=MOD_SERV_Motor_Angle_Converter(0,32);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][1]=MOD_SERV_Motor_Angle_Converter(0,33);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][2]=MOD_SERV_Motor_Angle_Converter(0,34);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][3]=MOD_SERV_Motor_Angle_Converter(0,35);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][4]=MOD_SERV_Motor_Angle_Converter(0,36);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][5]=MOD_SERV_Motor_Angle_Converter(0,37);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][6]=MOD_SERV_Motor_Angle_Converter(0,38);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][7]=MOD_SERV_Motor_Angle_Converter(0,39);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][8]=MOD_SERV_Motor_Angle_Converter(0,40);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][9]=MOD_SERV_Motor_Angle_Converter(0,41);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][10]=MOD_SERV_Motor_Angle_Converter(0,42);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][11]=MOD_SERV_Motor_Angle_Converter(0,43);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][12]=MOD_SERV_Motor_Angle_Converter(0,44);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][13]=MOD_SERV_Motor_Angle_Converter(0,42);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][14]=MOD_SERV_Motor_Angle_Converter(0,43);
PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E][15]=MOD_SERV_Motor_Angle_Converter(0,44);


  pinMode (RELAY_CH_1_PIN,OUTPUT);
  pinMode (RELAY_CH_2_PIN,OUTPUT);
  pinMode (RELAY_CH_3_PIN,OUTPUT);
  pinMode (RELAY_CH_4_PIN,OUTPUT);
  pinMode (RELAY_CH_5_PIN,OUTPUT);
  pinMode (RELAY_CH_6_PIN,OUTPUT);
  pinMode (RELAY_CH_7_PIN,OUTPUT);
  pinMode (RELAY_CH_8_PIN,OUTPUT);
  pinMode (RELAY_CH_9_PIN,OUTPUT);

}

/******************HEART RATE MONITOR******************/

void MOD_SERV_HeartRate_Init()
{
  pinMode(HEART_MONITOR_LEFT_PAD_PIN,  INPUT); // Setup for leads off detection LO +
  pinMode(HEART_MONITOR_RIGHT_PAD_PIN, INPUT); // Setup for leads off detection LO -
}

void MOD_SERV_HeartRate_Monitor()
{
    static uint16_t  HeartRate_Cycle_Length_U16 =0;
    uint16_t  HeartRate_Raw_Data_U16 ;

    HeartRate_Raw_Data_U16 =analogRead(HEART_MONITOR_ANALOG_PIN);
     
    if (HeartRate_Raw_Data_U16  < HeartRate_Threshhold_U16 ) HeartRate_Cycle_Length_U16 ++;
    else
    {
      if(HeartRate_Cycle_Length_U16  >HEARTRATE_MINIMUM_CYCLE_LENGTH)
      {
      HeartRate_Result_U16  =HEARTRATE_MINUTE_TO_MS/HeartRate_Cycle_Length_U16 ;
      HeartRate_Cycle_Length_U16 =0;
      }
    }
}

void MOD_SERV_HeartRate_Calibration()
{
         uint16_t  HeartRate_Raw_Data_U16 ;
  static uint16_t  HeartRate_Max_U16 ;
  static uint16_t  HeartRate_Min_U16 ;
  static bool      HeartRate_Reset=true;
  
  if(HeartRate_Remaining_Data_U16 >1)
  {
    HeartRate_Raw_Data_U16 =analogRead(HEART_MONITOR_ANALOG_PIN);
    if(HeartRate_Reset==true)
    {
      HeartRate_Max_U16 =HeartRate_Raw_Data_U16 ;
      HeartRate_Min_U16 =HeartRate_Raw_Data_U16 ;
      HeartRate_Reset=false;
    }
    else
    {
      HeartRate_Max_U16 =max(HeartRate_Raw_Data_U16 ,HeartRate_Max_U16 );
      HeartRate_Min_U16 =min(HeartRate_Raw_Data_U16 ,HeartRate_Min_U16 );
    }
  }
  else
  {
    HeartRate_Reset=true;
    HeartRate_Calibration=false;
    HeartRate_Threshhold_U16 =HeartRate_Min_U16  +((HeartRate_Max_U16 -HeartRate_Min_U16 )*HEARTRATE_THRESHOLD_PERCENTAGE);
    HeartRate_Remaining_Data_U16 =HEARTRATE_CALIBRATION_LENGTH;
  }  
}

  /******************INPUT MANAGER******************/

uint8_t MOD_SERV_Button_Matrix_Read()
{
  static uint8_t Last_Key_U8=0;
  uint8_t Current_Button_Check_U8=1;
  uint8_t Output_Button_U8=0;
  bool Button_Read_BO;
   
  for(int Row_Counter_U8=0;Row_Counter_U8<4;Row_Counter_U8++)
  {
    digitalWrite(Button_Matrix_Row_Pins_AU8[Row_Counter_U8],LOW);
    
    for(int Column_Counter_U8=0;Column_Counter_U8<4;Column_Counter_U8++)
    {
      Current_Button_Check_U8++;
      Button_Read_BO=digitalRead(Button_Matrix_Column_Pins_AU8[Column_Counter_U8]);
      
      if ((Button_Read_BO==LOW) && (Last_Key_U8!=Current_Button_Check_U8))
      {
       Last_Key_U8=Current_Button_Check_U8;       
       Output_Button_U8= Current_Button_Check_U8;
      }
      if(Button_Read_BO==HIGH && Last_Key_U8==Current_Button_Check_U8) 
      {
        Last_Key_U8=0;
      }
     }
   digitalWrite(Button_Matrix_Row_Pins_AU8[Row_Counter_U8],HIGH);
  }
  return Output_Button_U8;
}
  
void MOD_SERV_Button_Matrix_Init()
{
  for(Row_Counter_U8 =0;Row_Counter_U8 <4;Row_Counter_U8 ++)           pinMode(Button_Matrix_Row_Pins_AU8    [Row_Counter_U8]    , OUTPUT       );
  for(Column_Counter_U8 =0;Column_Counter_U8 <4;Column_Counter_U8 ++)  pinMode(Button_Matrix_Column_Pins_AU8 [Column_Counter_U8] , INPUT_PULLUP );
}   
    
void MOD_SERV_Button_Matrix_Task()
{
    uint8_t  Detected_Input_U8 ;
    Detected_Input_U8 = MOD_SERV_Button_Matrix_Read();
    if (Detected_Input_U8!=0)
    {
      Serial.println ("Detected_Input_U8="+(String)Detected_Input_U8);
      }

    // magic numbers to be replaced once the glove is finished
       
    switch(Detected_Input_U8 )
    {
       
      case 2  : {MOD_SERV_OLED_input_Manager(DOWN_INPUT_E);}break; 
      case 6  : {MOD_SERV_OLED_input_Manager(EXIT_INPUT_E);}break;
      case 10 : {MOD_SERV_OLED_input_Manager(ENTER_INPUT_E);}break; 
      case 14 : {MOD_SERV_OLED_input_Manager(UP_INPUT_E);}break;

      case 1  : {}break; 
      case 3  : {}break;  
      case 4  : {}break;  
      case 5  : {}break; 
      case 7  : {}break;  
      case 8  : {}break;  
      case 9  : {}break; 
      case 11 : {}break; 
      case 12 : {}break;
      case 13 : {}break; 
      case 15 : {}break; 
      case 16 : {}break; 
      default : {}

    //1  - idle - *** DONE ***
    //2  - hug
    //3  - crowd
    //4  - tail wag individual - *** DONE ***
    //5  - tail wag combined
    //6  - walking mode - *** DONE ***
    //7  - pose 2 (tail circle aranngement back)
    //8  - twitch
    //9  - ear mode change
    //10 - exitement down
    //11 - exitement up
    //12 - Pause Animation

    }
   
}
    
/******************ERROR MANAGEMENT******************/  
  
void MOD_SERV_Error_Set(ERROR_CODES_ENUM Received_Error)
{
  Serial.println("Error:"+(String)Received_Error);
  if(Error_flag_ABO[Received_Error]==false)  // this has an bug on the heart rate monitor flags
  {
    Error_flag_ABO[Received_Error]=true; 	
    MOD_MCAL_Error_Buzzer_trigger();
    }
}
  
void MOD_SERV_Error_Reset(ERROR_CODES_ENUM Received_Error)
{
    Error_flag_ABO[Received_Error]=false; 
}
  
void MOD_MCAL_Error_Buzzer_trigger()
{
	digitalWrite(BUZZER_PIN,HIGH);
  Buzzer_timer_U8=50;
  Buzzer_Status_BO=true;
}
  
/******************INITIALIZATION******************/


void MOD_SERV_Main_Motor_Update()
{   
 
  //MOD_APPL_Animation_Manager();
  //MOD_SERV_Animation_State_Machine();
  //MOD_SERV_Power_Manager_Read_Ammeters();
  
  //MOD_SERV_Motor_Manager_Write_To_All_Motors();  this will be used in the final version   we are using individual sends to modules for testing and debugging

  PCA9685_Module_1.setChannelsPWM(0,15,PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH1_E]);
  PCA9685_Module_2.setChannelsPWM(0,15,PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH2_E]);
  PCA9685_Module_3.setChannelsPWM(0,15,PCA9685_Motor_Angle_To_Be_Written_MU16[EXTERNAL_CH3_E]);
  
  Motor_Update_BO=false;
  
  
}

void MOD_SERV_Main_OLED_Update()
{
  
	if (OLED_Timer_U16==0 && OLED_Connection_Status_U8==0)
	{
    MOD_SERV_OLED_Screen_Manager();
    display.display();
    if(current_menu_EN!= POWER_MENU_E)
    OLED_Update_BO= false;	
    Serial.print("up false");
	}
}

void MOD_SERV_Main_Buzzer_Update()
{
	if (Buzzer_timer_U8==0)	{	digitalWrite(BUZZER_PIN,LOW); Buzzer_Status_BO= false;}
}

void MOD_SERV_Main_HeartMonitor_Update()
{
       if(digitalRead(HEART_MONITOR_LEFT_PAD_PIN)  ==HIGH) MOD_SERV_Error_Set(LEFT_HEART_ELECTRODE_ISSUE_E);
  else if(digitalRead(HEART_MONITOR_RIGHT_PAD_PIN) ==HIGH) MOD_SERV_Error_Set(RIGHT_HEART_ELECTRODE_ISSUE_E);
  else if(HeartRate_Calibration                    ==true) MOD_SERV_HeartRate_Monitor();
  else                                                     MOD_SERV_HeartRate_Calibration();
}

void MOD_SERV_Main_Startup_Sequence()
{
  // allow some time for Anmeter calibration to finish before starting relays; as of writing this comment that value is 100 itterations and the counter is 200
    
  //if (Startup_Counter_U8 ==80){ MOD_MCAL_Relay_Control(4,HIGH);}
  //if (Startup_Counter_U8 ==60){ MOD_MCAL_Relay_Control(3,HIGH); MOD_MCAL_Relay_Control(5,HIGH);}
  //if (Startup_Counter_U8 ==40){ MOD_MCAL_Relay_Control(2,HIGH); MOD_MCAL_Relay_Control(6,HIGH);}
  //if (Startup_Counter_U8 ==20){ MOD_MCAL_Relay_Control(1,HIGH); MOD_MCAL_Relay_Control(7,HIGH);}
  //if (Startup_Counter_U8 ==0 ){ MOD_MCAL_Relay_Control(0,HIGH); MOD_MCAL_Relay_Control(8,HIGH);}
  
 // if (Startup_Counter_U8 ==80) MOD_MCAL_Relay_Control(6,HIGH);







  if (Startup_Counter_U8 ==0) Next_State_EN=SM_IDLE_EN;
  Startup_Sequence_BO=false;
	//Next_State_EN=SM_IDLE_EN;
}

void MOD_SERV_Main_I2C_Error_Wrapper(uint8_t I2C_Status_U8_Param)
{

  switch(I2C_Status_U8_Param)
  {
    case 1  : {Serial.println("transmit buffer Overflow"); }break;
    case 2  : {Serial.println("Address NAK");              }break;
    case 3  : {Serial.println("Data NAK");                 }break;
    case 4  : {Serial.println("other error");              }break;
    case 5  : {Serial.println("timeout");                  }break;
    default : {Serial.println("unknown error");            }
  }

}

void MOD_SERV_Main_I2C_Connectivity_Test()
{
 Wire.setWireTimeout(5000,true);
 Wire.setClock(400000);

 uint8_t I2C_Status_U8;

 Wire.beginTransmission(0x40);
 I2C_Status_U8 = Wire.endTransmission();
 if (I2C_Status_U8 == 0) Serial.println("(0x40)PCA Module 1 OK");
 else 
  {
   Serial.print("(0x40)PCA Module 1 Fail ");
   MOD_SERV_Main_I2C_Error_Wrapper(I2C_Status_U8);
  }

 Wire.beginTransmission(0x41);
 I2C_Status_U8 = Wire.endTransmission();
 if (I2C_Status_U8 == 0) Serial.println("(0x41)PCA Module 2 OK");
 else 
  {
    Serial.print("(0x41)PCA Module 2 Fail ");
    MOD_SERV_Main_I2C_Error_Wrapper(I2C_Status_U8);
  }

 Wire.beginTransmission(0x42);
 I2C_Status_U8 = Wire.endTransmission();
 if (I2C_Status_U8 == 0) Serial.println("(0x42)PCA Module 3 OK");
 else 
  {
   Serial.print("(0x42)PCA Module 2 Fail ");
   MOD_SERV_Main_I2C_Error_Wrapper(I2C_Status_U8);
  }

 Wire.beginTransmission(0x3C);
 I2C_Status_U8 = Wire.endTransmission();
 if (I2C_Status_U8 == 0) Serial.println("(0x3C)OLED Module OK");
 else
  {
   Serial.print("(0x42)PCA Module 2 Fail ");
   MOD_SERV_Main_I2C_Error_Wrapper(I2C_Status_U8);
  }

 Wire.beginTransmission(0x68);
 I2C_Status_U8 = Wire.endTransmission();
 if (I2C_Status_U8 == 0) Serial.println("(0x68)IMU Module 1 OK");
 else 
  {
    Serial.print("(0x68)IMU Module 1 Fail ");
    MOD_SERV_Main_I2C_Error_Wrapper(I2C_Status_U8);
  }

 Wire.beginTransmission(0x69);
 I2C_Status_U8 = Wire.endTransmission();
 if (I2C_Status_U8 == 0) Serial.println("(0x69)IMU Module 2 OK");
 else 
 {
  Serial.print("(0x69)IMU Module 2 Fail ");
  MOD_SERV_Main_I2C_Error_Wrapper(I2C_Status_U8);
 }

}

void MOD_MCAL_Buzzer_Init()
{ 
  pinMode(BUZZER_PIN,OUTPUT);
  digitalWrite(BUZZER_PIN,HIGH);
  delay (500); 
  digitalWrite(BUZZER_PIN,LOW);
  delay (500); 
}

void setup() 
{

 MOD_MCAL_Buzzer_Init();
 Serial.begin(115200);                  Serial.println("Serial initialization finished");// maybe add an error code in case this fails
 MOD_SERV_Main_I2C_Connectivity_Test(); Serial.println("I2C Connectivity test finished");
 MOD_SERV_OLED_Manager_Init();          Serial.println("Display Initialization finished");
 MOD_MCAL_Relay_Init();                 Serial.println("Relays Initialization finished");
 MOD_SERV_Motor_Manager_init();         Serial.println("Motor_Manager Initialization finished");
 MOD_SERV_PCA9685_Init();               Serial.println("PCA_9685 modules Initialization finished");
 MOD_SERV_Button_Matrix_Init();         Serial.println("Button Matrix finished");
 MOD_MCAL_Task_init();                  Serial.println("Task Initialization finished");
 MOD_SERV_Power_Manager_Init();         Serial.println("Anmeter Initialization finished");

}

void loop()
 { 


   if(Motor_Update_BO     ==true) MOD_SERV_Main_Motor_Update();
   if(OLED_Update_BO      ==true) MOD_SERV_Main_OLED_Update();
   if(Buzzer_Status_BO    ==true) MOD_SERV_Main_Buzzer_Update();
 //if(HeartRate_Update_BO ==true) MOD_SERV_Main_HeartMonitor_Update(); has a problem with error flag not being set correctly and makes the buzzer be on continiously
   if(Startup_Sequence_BO ==true) MOD_SERV_Main_Startup_Sequence();
  }

  
