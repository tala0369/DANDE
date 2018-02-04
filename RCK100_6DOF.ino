//Include Library
//version 1.2
#include <BOLIDE_Player.h>
#include <A1-16.h>
#include <EEPROM.h>
#include "BOLIDE_Board.h"
#include "Mask_Definition.h"
#include "RCK100_USER_MOTION.h"


//== Declare Global Parameters ==
#define Joystick_Therhold 30
#define Joystick_CEN 127
#define SETTING_TIME 0
#define SETTING_TIME_GRIPPER 10
#define SETTING_TIME_HOME 100

#define SERVO_1_YAW_ID 1
#define SERVO_2_PITCH_ID 2
#define SERVO_3_PITCH_ID 3
#define SERVO_4_PITCH_ID 4
#define SERVO_5_ROW_ID 5
#define SERVO_6_GRIPPER_ID 6

#define SERVO_1_YAW_CEN 512
#define SERVO_1_YAW_MAX 1023
#define SERVO_1_YAW_MIN 0
#define SERVO_2_PITCH_CEN 373
#define SERVO_2_PITCH_MAX 742
#define SERVO_2_PITCH_MIN 234
#define SERVO_3_PITCH_CEN 512
#define SERVO_3_PITCH_MAX 790
#define SERVO_3_PITCH_MIN 252
#define SERVO_4_PITCH_CEN 373
#define SERVO_4_PITCH_MAX 782
#define SERVO_4_PITCH_MIN 152
#define SERVO_5_ROW_CEN 790
#define SERVO_5_ROW_MAX 1023
#define SERVO_5_ROW_MIN 0
#define SERVO_6_GRIPPER_MIN 512
#define SERVO_6_GRIPPER_MAX 852

#define SERVO_DELTA_DEFAULT 2
#define SERVO_DELTA_DEFAULT_2 1

#define LOOP_DELTA_TIME 5
#define POS_DELTA 20
#define POS_DELTA_RCU 2
#define MOTOR_NUMBER 6
//Normal Operation Pararmeter
BOLIDE_Player XYZrobot;
static int packet[7];
static boolean torque_release = false, BT_update = false;
//Motion Editor Parameter
static boolean packet_timeout_status = false;
static boolean seq_trigger = false, seq_loop_trigger = false;
static int seq_pSeqCnt = 0xFF, SeqPos = 0x00;
static int poses[max_pose_index][MAX_SERVOS];      // poses [index][servo_id-1], check for the motion index!!
static int pose_index[max_pose_index];
sp_trans_t sequence[max_seq_index];// sequence
static boolean _servo_row_moving = false, _servo_row_dir = false;
static int _servo_1_yaw_pos = SERVO_1_YAW_CEN, _servo_1_yaw_delta = 0;
static int _servo_2_pitch_pos = SERVO_2_PITCH_CEN, _servo_2_pitch_delta = 0;
static int _servo_3_pitch_pos = SERVO_3_PITCH_CEN, _servo_3_pitch_delta = 0;
static int _servo_4_pitch_pos = SERVO_4_PITCH_CEN, _servo_4_pitch_delta = 0;
static int _servo_5_row_pos = SERVO_5_ROW_CEN, _servo_5_row_delta = 0;
static int _servo_6_grip_pos = SERVO_6_GRIPPER_MIN, _servo_6_grip_delta = 0;
//========================= Set up =======================================
void setup(){
  //Configure all basic setting
  Serial.begin(115200);
  AIM_Task_Setup();
  BT_Task_Setup();
  
  Buzzer_Setup();
  Button_Setup();
  Power_Detection_setup();  
  Start_Music();
  Initial_Pose_Setup();
  delay(1000);
  Timer_Task(0);
  Timer_Task_Setup();
}
//========================= Main =======================================
void loop(){
  //USB Communcation motion
  if(Serial.available() > 0){  
    //USB Communcation motion 
    Motion_Editor_Packet_Task();
  }
 
  //play sequence edited by motion editor
  else if(seq_trigger){
    Motion_Editor_Seq_Play();
    _servo_1_yaw_delta = 0;
    _servo_2_pitch_delta = 0;
    _servo_3_pitch_delta = 0;
    _servo_4_pitch_delta = 0;
    _servo_5_row_delta = 0;
    _servo_6_grip_delta = 0;
  }
  
  else{
   if(Serial2.available() > 0){
    //BT Communcation motion
    BT_Packet_Task();
    if(BT_update){
      _servo_1_yaw_delta = 0;
      _servo_2_pitch_delta = 0;
      _servo_3_pitch_delta = 0;
      _servo_4_pitch_delta = 0;
      _servo_5_row_delta = 0;
      _servo_6_grip_delta = 0;
      //==== RCU Command ====
      if(packet[1]!=255 || packet[2]!=1){
              
        //Release Button
        if(packet[5] & RCU_mask_release){
          A1_16_TorqueOff(A1_16_Broadcast_ID);
        }
        
        //Bluetooth Button
        else if(packet[5] & RCU_mask_BT);
        
        //Power Button
        else if(packet[5] & RCU_mask_power){
          Action(255);
        }
        
        //L1 Button
        else if(packet[6] & RCU_mask_L1){
            if(RCU_L1 != 0){Action(RCU_L1);}
            else{
                _servo_5_row_delta = POS_DELTA_RCU; 
            }
        }
        
        //L2 Button
        else if(packet[6] & RCU_mask_L2){
          if(RCU_L2 != 0){Action(RCU_L2);}
          else{
              _servo_5_row_delta = -POS_DELTA_RCU;
          }
        }
        
        //L3 Button
        else if(packet[6] & RCU_mask_L3){
          if(RCU_L3 != 0){Action(RCU_L3);}
          else{
             _servo_1_yaw_pos = 782;
             SetPositionI_JOG(SERVO_1_YAW_ID, 70, _servo_1_yaw_pos);
          }
        }
        
        //R1 Button
        else if(packet[5] & RCU_mask_R1){
          if(RCU_R1 != 0){Action(RCU_R1);}
          else{
              _servo_6_grip_delta = POS_DELTA_RCU;
          }
        }
        
        //R2 Button
        else if(packet[5] & RCU_mask_R2){
          if(RCU_R2 != 0){Action(RCU_R2);}
          else{
              _servo_6_grip_delta = -POS_DELTA_RCU; 
          }
        }
        
        //R3 Button
        else if(packet[5] & RCU_mask_R3){
          if(RCU_R3 != 0){Action(RCU_R3);}
          else{
            _servo_1_yaw_pos = 242;
            SetPositionI_JOG(SERVO_1_YAW_ID, 70, _servo_1_yaw_pos);  
          }
        }
        
        //LeftJoystick_Rightside
         if((packet[1]>155&&packet[2]>155&&packet[1]>packet[2])|(packet[1]>155&&packet[2]<95&&(packet[1]-155)>(95-packet[2]))|(packet[1]>155&&packet[2]>=95&&packet[2]<=155)){         
           if(RCU_LJR != 0) Action(RCU_LJR); 
           else _servo_1_yaw_delta = -SERVO_DELTA_DEFAULT;  
        }
        
        //LeftJoystick_Leftside
        else if((packet[1]<95&&packet[2]>155&&(95-packet[1])>(packet[2]-155))|(packet[1]<95&&packet[2]<155&&(95-packet[1])>(95-packet[2]))|(packet[1]<95&&packet[2]>=95&&packet[2]<=155)){          
           if(RCU_LJL != 0) Action(RCU_LJL); 
           else _servo_1_yaw_delta = SERVO_DELTA_DEFAULT;  
        }
         
        
        //LeftJoystick_Upside
        if((packet[1]>155&&packet[2]>155&&packet[1]<packet[2])|(packet[1]<95&&packet[2]>155&&(95-packet[1])<(packet[2]-155))|(packet[2]>155&&packet[1]>=95&&packet[1]<=155)){          
            if(RCU_LJU != 0) Action(RCU_LJU); 
            else _servo_2_pitch_delta = SERVO_DELTA_DEFAULT_2; 
        }
        
        //LeftJoystick_Downside
        else if((packet[1]>155&&packet[2]<95&&(packet[1]-155)<(95-packet[2]))|(packet[1]<95&&packet[2]<95&&(95-packet[1])<(155-packet[2]))|(packet[2]<95&&packet[1]>=95&&packet[1]<=155)){          
            if(RCU_LJD != 0) Action(RCU_LJD); 
            else _servo_2_pitch_delta = -SERVO_DELTA_DEFAULT_2;  
        }
        
        
        //RightJoystick_Rightside
         if((packet[3]>155&packet[4]>155&&packet[3]>packet[4])|(packet[3]>155&&packet[4]<95&&(packet[3]-155)>(95-packet[4]))|(packet[3]>155&&packet[4]>=95&&packet[4]<=155)){           
            if(RCU_RJR != 0) Action(RCU_RJR); 
            else _servo_3_pitch_delta = SERVO_DELTA_DEFAULT_2;
        }
        
        //RightJoystick_Leftside
        else if((packet[3]<95&&packet[4]>155&&(95-packet[3])>(packet[4]-155))|(packet[3]<95&&packet[4]<95&&(95-packet[3])>(95-packet[4]))|(packet[3]<95&&packet[4]>=95&&packet[4]<=155)){           
            if(RCU_RJL != 0) Action(RCU_RJL);
            else _servo_3_pitch_delta = -SERVO_DELTA_DEFAULT_2;
        }
        
        
        //RightJoystick_Upside
        if((packet[3]>155&&packet[4]>155&&packet[3]<packet[4])|(packet[3]<95&&packet[4]>155&&(95-packet[3])<(packet[4]-155))|(packet[4]>155&&packet[3]>=95&&packet[3]<=155)){          
            if(RCU_RJU != 0) Action(RCU_RJU);
            else _servo_4_pitch_delta = -SERVO_DELTA_DEFAULT_2;  
        }
        
        //RightJoystick_Downside
        else if((packet[3]>155&&packet[4]<95&&(packet[3]-155)<(95-packet[4]))|(packet[3]<155&&packet[4]<95&&(95-packet[3])<(95-packet[4]))|(packet[4]<95&&packet[3]>=95&&packet[3]<=155)){          
            if(RCU_RJD != 0) Action(RCU_RJD);
            else _servo_4_pitch_delta = SERVO_DELTA_DEFAULT_2;  
        }     
      }
      //==== App Command ==== 
      else if(packet[1]==255 && packet[2]==1){
        if(packet[3] == 102) A1_16_TorqueOff(254);       
        else if(packet[3] == 253) BT_FW();
         else if(packet[3] == 1) {
          _servo_2_pitch_pos += POS_DELTA;
          if(_servo_2_pitch_pos >= SERVO_2_PITCH_MAX) {_servo_2_pitch_pos = SERVO_2_PITCH_MAX;}
          SetPositionI_JOG(SERVO_2_PITCH_ID, SETTING_TIME, _servo_2_pitch_pos); 
        }
        else if(packet[3] == 2) {
          _servo_2_pitch_pos -= POS_DELTA;
          if(_servo_2_pitch_pos <= SERVO_2_PITCH_MIN) {_servo_2_pitch_pos = SERVO_2_PITCH_MIN;}
          SetPositionI_JOG(SERVO_2_PITCH_ID, SETTING_TIME, _servo_2_pitch_pos); 
        }
        else if(packet[3] == 3) {
          _servo_5_row_pos += POS_DELTA;
          if(_servo_5_row_pos >= SERVO_5_ROW_MAX) {_servo_5_row_pos = SERVO_5_ROW_MAX;}
          SetPositionI_JOG(SERVO_5_ROW_ID, SETTING_TIME, _servo_5_row_pos); 
        }
        else if(packet[3] == 4) {
          _servo_5_row_pos -= POS_DELTA;
          if(_servo_5_row_pos <= SERVO_5_ROW_MIN) {_servo_5_row_pos = SERVO_5_ROW_MIN;}
          SetPositionI_JOG(SERVO_5_ROW_ID, SETTING_TIME, _servo_5_row_pos); 
        }
        else if(packet[3] == 5) {
          _servo_1_yaw_pos += POS_DELTA;
          if(_servo_1_yaw_pos >= SERVO_1_YAW_MAX) {_servo_1_yaw_pos = SERVO_1_YAW_MAX;}
          SetPositionI_JOG(SERVO_1_YAW_ID, SETTING_TIME, _servo_1_yaw_pos); 
        }
        else if(packet[3] == 6) {
          _servo_1_yaw_pos -= POS_DELTA;
          if(_servo_1_yaw_pos <= SERVO_1_YAW_MIN) {_servo_1_yaw_pos = SERVO_1_YAW_MIN;}
          SetPositionI_JOG(SERVO_1_YAW_ID, SETTING_TIME, _servo_1_yaw_pos); 
        }
        else if(packet[3] == 7) { //APP page1 button A
          _servo_3_pitch_pos -= POS_DELTA;
          if(_servo_3_pitch_pos <= SERVO_3_PITCH_MIN) {_servo_3_pitch_pos = SERVO_3_PITCH_MIN;}
          SetPositionI_JOG(SERVO_3_PITCH_ID, SETTING_TIME, _servo_3_pitch_pos); 
        }
        else if(packet[3] == 9) { //APP page1 button D
          _servo_3_pitch_pos += POS_DELTA;
          if(_servo_3_pitch_pos >= SERVO_3_PITCH_MAX) {_servo_3_pitch_pos = SERVO_3_PITCH_MAX;}
          SetPositionI_JOG(SERVO_3_PITCH_ID, SETTING_TIME, _servo_3_pitch_pos); 
        }
        else if(packet[3] == 8) { //APP page1 button B
          _servo_4_pitch_pos -= POS_DELTA;
          if(_servo_4_pitch_pos <= SERVO_4_PITCH_MIN) {_servo_4_pitch_pos = SERVO_4_PITCH_MIN;}
          SetPositionI_JOG(SERVO_4_PITCH_ID, SETTING_TIME, _servo_4_pitch_pos); 
        }
         else if(packet[3] == 10) { //APP page1 button E
          _servo_4_pitch_pos += POS_DELTA;
          if(_servo_4_pitch_pos >= SERVO_4_PITCH_MAX) {_servo_4_pitch_pos = SERVO_4_PITCH_MAX;}
          SetPositionI_JOG(SERVO_4_PITCH_ID, SETTING_TIME, _servo_4_pitch_pos); 
        }
        else if(packet[3] == 21) { //APP page1 button C
          _servo_6_grip_pos -= POS_DELTA;
          if(_servo_6_grip_pos <= SERVO_6_GRIPPER_MIN) {_servo_6_grip_pos = SERVO_6_GRIPPER_MIN;}
          SetPositionI_JOG(SERVO_6_GRIPPER_ID, SETTING_TIME_GRIPPER, _servo_6_grip_pos); 
        }
         else if(packet[3] == 20) { //APP page1 button F
           _servo_6_grip_pos += POS_DELTA;
          if(_servo_6_grip_pos >= SERVO_6_GRIPPER_MAX) {_servo_6_grip_pos = SERVO_6_GRIPPER_MAX;}
          SetPositionI_JOG(SERVO_6_GRIPPER_ID, SETTING_TIME_GRIPPER, _servo_6_grip_pos); 
        }
        else if(packet[3] == 22) { //APP mode1 page2 button A
            _servo_2_pitch_pos = SERVO_2_PITCH_MAX;
            SetPositionI_JOG(SERVO_2_PITCH_ID, 100, _servo_2_pitch_pos); 
        }
        else if(packet[3] == 27) { //APP mode1 page2 button B
            _servo_3_pitch_pos = 550;
            SetPositionI_JOG(SERVO_3_PITCH_ID, 100, _servo_3_pitch_pos); 
        }
        else if(packet[3] == 28) { //APP mode1 page2 button C
            _servo_4_pitch_pos = SERVO_4_PITCH_MAX;
            SetPositionI_JOG(SERVO_4_PITCH_ID, 100, _servo_4_pitch_pos); 
        }
         else if(packet[3] == 17) { //APP mode1 page2 button D
             XYZrobot.readPose();
             Action(52);
             _servo_1_yaw_pos = SERVO_1_YAW_CEN; _servo_1_yaw_delta = 0;
              _servo_2_pitch_pos = SERVO_2_PITCH_CEN; _servo_2_pitch_delta = 0;
              _servo_3_pitch_pos = SERVO_3_PITCH_CEN; _servo_3_pitch_delta = 0;
              _servo_4_pitch_pos = SERVO_4_PITCH_CEN; _servo_4_pitch_delta = 0;
              _servo_5_row_pos = SERVO_5_ROW_CEN; _servo_5_row_delta = 0; _servo_row_moving = false;
              _servo_6_grip_pos = SERVO_6_GRIPPER_MIN;_servo_6_grip_delta = 0;
        }
         else if(packet[3] == 18) { //APP mode1 page2 button E
              XYZrobot.readPose();
              Action(53);
              _servo_1_yaw_pos = SERVO_1_YAW_CEN; _servo_1_yaw_delta = 0;
              _servo_2_pitch_pos = SERVO_2_PITCH_CEN; _servo_2_pitch_delta = 0;
              _servo_3_pitch_pos = SERVO_3_PITCH_CEN; _servo_3_pitch_delta = 0;
              _servo_4_pitch_pos = SERVO_4_PITCH_CEN; _servo_4_pitch_delta = 0;
              _servo_5_row_pos = SERVO_5_ROW_CEN; _servo_5_row_delta = 0; _servo_row_moving = false;
              _servo_6_grip_pos = SERVO_6_GRIPPER_MIN;_servo_6_grip_delta = 0;
        }
         else if(packet[3] == 19) { //APP mode1 page2 button F
              XYZrobot.readPose();
              Action(54);
              _servo_1_yaw_pos = SERVO_1_YAW_CEN; _servo_1_yaw_delta = 0;
              _servo_2_pitch_pos = SERVO_2_PITCH_CEN; _servo_2_pitch_delta = 0;
              _servo_3_pitch_pos = SERVO_3_PITCH_CEN; _servo_3_pitch_delta = 0;
              _servo_4_pitch_pos = SERVO_4_PITCH_CEN; _servo_4_pitch_delta = 0;
              _servo_5_row_pos = SERVO_5_ROW_CEN; _servo_5_row_delta = 0; _servo_row_moving = false;
              _servo_6_grip_pos = SERVO_6_GRIPPER_MIN;_servo_6_grip_delta = 0;
        }
        else Action(packet[3]);   
      }
       BT_update = false;     
     }
    }
   else{
         // Button task 
         BUTTON_Task();
    }
  }
  	if(Timer_Task(LOOP_DELTA_TIME)){
		static uint8_t _servo_1_start = 1;
		static uint8_t _servo_2_start = 1;
		static uint8_t _servo_3_start = 1;
                static uint8_t _servo_4_start = 1;
                static uint8_t _servo_5_start = 1;
                static uint8_t _servo_6_start = 1;
		if(_servo_1_yaw_delta == 0) _servo_1_start = 1;// _servo_1_yaw_pos = ReadPosition(SERVO_1_YAW_ID);
		else{
			if(_servo_1_start){
				_servo_1_start = 0;
				_servo_1_yaw_pos = ReadPosition(SERVO_1_YAW_ID);
			}
			_servo_1_yaw_pos += _servo_1_yaw_delta;
			if((_servo_1_yaw_pos >= SERVO_1_YAW_MAX) && (_servo_1_yaw_delta > 0)){
				_servo_1_yaw_pos = SERVO_1_YAW_MAX;
				_servo_1_yaw_delta = 0;
			}
			else if((_servo_1_yaw_pos <= SERVO_1_YAW_MIN) && (_servo_1_yaw_delta < 0)){
				_servo_1_yaw_pos = SERVO_1_YAW_MIN;
				_servo_1_yaw_delta = 0;
			}
			SetPositionI_JOG(SERVO_1_YAW_ID, SETTING_TIME, _servo_1_yaw_pos);
		}
		if(_servo_2_pitch_delta == 0) _servo_2_start = 1;// _servo_2_pitch_pos = ReadPosition(SERVO_2_PITCH_ID);
		else{
			if(_servo_2_start){
				_servo_2_start = 0;
				_servo_2_pitch_pos = ReadPosition(SERVO_2_PITCH_ID);
			}
			_servo_2_pitch_pos += _servo_2_pitch_delta;
			if((_servo_2_pitch_pos >= SERVO_2_PITCH_MAX) && (_servo_2_pitch_delta > 0)){
				_servo_2_pitch_pos = SERVO_2_PITCH_MAX;
				_servo_2_pitch_delta = 0;
			}
			else if((_servo_2_pitch_pos <= SERVO_2_PITCH_MIN) && (_servo_2_pitch_delta < 0)){
				_servo_2_pitch_pos = SERVO_2_PITCH_MIN;
				_servo_2_pitch_delta = 0;
			}
			SetPositionI_JOG(SERVO_2_PITCH_ID, SETTING_TIME, _servo_2_pitch_pos);
		}
		if(_servo_3_pitch_delta == 0) _servo_3_start = 1;// _servo_3_pitch_pos = ReadPosition(SERVO_3_PITCH_ID);
		else{
			if(_servo_3_start){
				_servo_3_start = 0;
				_servo_3_pitch_pos = ReadPosition(SERVO_3_PITCH_ID);
			}
			_servo_3_pitch_pos += _servo_3_pitch_delta;
			if((_servo_3_pitch_pos >= SERVO_3_PITCH_MAX) && (_servo_3_pitch_delta > 0)){
				_servo_3_pitch_pos = SERVO_3_PITCH_MAX;
				_servo_3_pitch_delta = 0;
			}
			else if((_servo_3_pitch_pos <= SERVO_3_PITCH_MIN) && (_servo_3_pitch_delta < 0)){
				_servo_3_pitch_pos = SERVO_3_PITCH_MIN;
				_servo_3_pitch_delta = 0;
			}
			SetPositionI_JOG(SERVO_3_PITCH_ID, SETTING_TIME, _servo_3_pitch_pos);
		}
                if(_servo_4_pitch_delta == 0) _servo_4_start = 1;// _servo_4_pitch_pos = ReadPosition(SERVO_4_PITCH_ID);
		else{
			if(_servo_4_start){
				_servo_4_start = 0;
				_servo_4_pitch_pos = ReadPosition(SERVO_4_PITCH_ID);
			}
			_servo_4_pitch_pos += _servo_4_pitch_delta;
			if((_servo_4_pitch_pos >= SERVO_4_PITCH_MAX) && (_servo_4_pitch_delta > 0)){
				_servo_4_pitch_pos = SERVO_4_PITCH_MAX;
				_servo_4_pitch_delta = 0;
			}
			else if((_servo_4_pitch_pos <= SERVO_3_PITCH_MIN) && (_servo_4_pitch_delta < 0)){
				_servo_4_pitch_pos = SERVO_3_PITCH_MIN;
				_servo_4_pitch_delta = 0;
			}
			SetPositionI_JOG(SERVO_4_PITCH_ID, SETTING_TIME, _servo_4_pitch_pos);
		}
                if(_servo_5_row_delta == 0) _servo_5_start = 1;// _servo_5_row_pos = ReadPosition(SERVO_5_ROW_ID);
		else{
			if(_servo_5_start){
				_servo_5_start = 0;
				_servo_5_row_pos = ReadPosition(SERVO_5_ROW_ID);
			}
			_servo_5_row_pos += _servo_5_row_delta;
			if((_servo_5_row_pos >= SERVO_5_ROW_MAX) && (_servo_5_row_delta > 0)){
				_servo_5_row_pos = SERVO_5_ROW_MAX;
				_servo_5_row_delta = 0;
			}
			else if((_servo_5_row_pos <= SERVO_5_ROW_MIN) && (_servo_5_row_delta < 0)){
				_servo_5_row_pos = SERVO_5_ROW_MIN;
				_servo_5_row_delta = 0;
			}
			SetPositionI_JOG(SERVO_5_ROW_ID, SETTING_TIME, _servo_5_row_pos);
		}
                if(_servo_6_grip_delta == 0) _servo_6_start = 1;// _servo_6_grip_pos = ReadPosition(SERVO_6_GRIPPER_ID);
		else{
			if(_servo_6_start){
				_servo_6_start = 0;
				_servo_6_grip_pos = ReadPosition(SERVO_6_GRIPPER_ID);
			}
			_servo_6_grip_pos += _servo_6_grip_delta;
			if((_servo_6_grip_pos >= SERVO_6_GRIPPER_MAX) && (_servo_6_grip_delta > 0)){
				_servo_6_grip_pos = SERVO_6_GRIPPER_MAX;
				_servo_6_grip_delta = 0;
			}
			else if((_servo_6_grip_pos <= SERVO_6_GRIPPER_MIN) && (_servo_6_grip_delta < 0)){
				_servo_6_grip_pos = SERVO_6_GRIPPER_MIN;
				_servo_6_grip_delta = 0;
			}
			SetPositionI_JOG(SERVO_6_GRIPPER_ID, SETTING_TIME, _servo_6_grip_pos);
		}
	}
}
//=========================== Function ================================
//=====power setup=====//
void Power_Detection_setup(void){
    pinMode(PWRDET_PIN, INPUT);
    analogReference(EXTERNAL); 
}

void Power_Detection_task(void){
  double PWR_Voltage = analogRead(PWRDET_PIN)/1024.0*5.108;
  PWR_Voltage = PWR_Voltage*16.49/6.49;
  if (PWR_Voltage < Power_Voltage_Alarm){
     tone(BUZZER_PIN,1000);
  }  
}
//== Setup function ==
//Configure A1-16 servo motor
void AIM_Task_Setup(){
  XYZrobot.setup(115200, MOTOR_NUMBER);
}

//Configure BT board
void BT_Task_Setup(void){
  Serial2.begin(9600);
}

//Configure speaker Board
void Speaker_Task_Setup(void){
  Serial3.begin(115200);
}

//Configure onboard buzzer pin
void Buzzer_Setup(void){
  pinMode(BUZZER_PIN, OUTPUT);
}

//Configure onboard button pin
void Button_Setup(void){
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);
  pinMode(BUTTON4_PIN, INPUT);
}

//Configure Analog input pin
void Analog_Input_Setup(void){
  pinMode(PWRDET_PIN, INPUT);
  analogReference(EXTERNAL); 
}

//Motion Editor Task
void Motion_Editor_Packet_Task(void){
  static int pBuffer[128] = {0};
  static unsigned char pIndex = 0, pLength = 0xFF, pCMD = 0x00;
  static unsigned char motor_ID = 0;
  static int position_ID = 0;
  static int seq_pPoseCnt = 0xFF, PoseCnt = 0;
  static int SeqCnt = 0;
  static int SeqProcessCnt = 0;
  static int _temp;

  _temp = Serial.read();
  if(_temp == packet_header) goto _packet_start;
  else {return;}
_packet_start:
  pBuffer[header_address] = _temp; pIndex = 1; _reset_timer4(timeout_limit); packet_timeout_status = false;
  while((_temp = Serial.read()) == -1){
    if(packet_timeout_status) {Packet_Error_Feedback(0x00); return;}
  }
  pBuffer[length_address] = _temp; pLength = _temp; pIndex++; _reset_timer4(timeout_limit); packet_timeout_status = false;
  while((_temp = Serial.read()) == -1){
    if(packet_timeout_status) {Packet_Error_Feedback(0x00); return;}
  }
  pBuffer[CMD_address] = _temp; pCMD = _temp; pIndex++; _reset_timer4(timeout_limit); packet_timeout_status = false;
  while(1){
    while((_temp = Serial.read()) == -1){
      if(packet_timeout_status) {Packet_Error_Feedback(0x00); return;}
    }
    pBuffer[pIndex] = _temp; pIndex++; _reset_timer4(timeout_limit); packet_timeout_status = false;
    if(pIndex == pLength) {cb_USB(); _reset_timer4(timeout_limit); goto _packet_finish;}
  }
_packet_finish:
  _servo_1_yaw_delta = 0;
  _servo_2_pitch_delta = 0;
  _servo_3_pitch_delta = 0;
  _servo_4_pitch_delta = 0;
  _servo_5_row_delta = 0;
  _servo_6_grip_delta = 0;
  if(pBuffer[pIndex-1] == packet_tail){
    if(pCMD == CMD_init_motor){//initial motion editor setting
      seq_trigger = false; SeqPos = 0;
      XYZrobot.poseSize = 6;//pBuffer[motor_num_address];
      XYZrobot.readPose();
      Packet_Init(pBuffer[motor_num_address]);
    }
    else if(pCMD == CMD_set_motor){//set motor position
      seq_trigger = false; SeqPos = 0;
      motor_ID = pBuffer[motor_ID_address];
      position_ID = (pBuffer[motor_pos_msb] << 8) + pBuffer[motor_pos_lsb];
      SetPositionI_JOG(motor_ID, 0x00, position_ID);
      Packet_Set(motor_ID, position_ID);
    }
    else if(pCMD == CMD_capture_motor){//get motor position
      seq_trigger = false; SeqPos = 0;
      motor_ID = pBuffer[motor_ID_address];
      Packet_Capture(motor_ID);
    }
    else if(pCMD == CMD_relax_motor){//relax motor
      seq_trigger = false; SeqPos = 0;
      motor_ID = pBuffer[motor_ID_address];
      A1_16_TorqueOff(motor_ID);
      Packet_Relax(motor_ID);
    }
    else if(pCMD == CMD_SN_read){                  // serial number read
      seq_trigger = false; SeqPos = 0;
      Packet_SN();
    }
    else if(pCMD == CMD_SEQ_load_PoseCnt){//load total pose number
      seq_trigger = false; SeqPos = 0;
      seq_pPoseCnt = pBuffer[seq_pose_cnt_address];
      if(seq_pPoseCnt > max_pose_index) Packet_Error_Feedback(0x00);
      else {Packet_Error_Feedback(0x01); SeqProcessCnt = SEQ_Process_load_PoseCnt; seq_loop_trigger = false;}
    }
    else if(pCMD == CMD_SEQ_loop_load_PoseCnt){//load loop sequence pose number
      seq_trigger = false; SeqPos = 0;
      seq_pPoseCnt = pBuffer[seq_pose_cnt_address];
      if(seq_pPoseCnt > max_pose_index) Packet_Error_Feedback(0x00);
      else {Packet_Error_Feedback(0x01); SeqProcessCnt = SEQ_Process_load_PoseCnt; seq_loop_trigger = true;}
    }
    else if(pCMD == CMD_SEQ_load_Pose){//load pose in sequence
      static int PoseID = 0, _i = 0;
      seq_trigger = false; SeqPos = 0;
      if(SeqProcessCnt == SEQ_Process_load_PoseCnt){
        PoseID = pBuffer[seq_pose_ID_address];
        for(_i = 0; _i < XYZrobot.poseSize; _i++){
          poses[PoseCnt][_i] = (pBuffer[seq_pose_start_address + 2*_i] << 8) + pBuffer[seq_pose_start_address + 1 + 2*_i];
          pose_index[PoseCnt] = PoseID;
        }
        PoseCnt++;
        if(PoseCnt == seq_pPoseCnt){Packet_Error_Feedback(0x02); PoseCnt = 0; SeqProcessCnt = SEQ_Process_load_Pose;}
        else Packet_Error_Feedback(0x01);
      }
      else Packet_Error_Feedback(0x00);
    }
    else if(pCMD == CMD_SEQ_load_SEQCnt){//load total sequence number
      seq_trigger = false; SeqPos = 0;
      if(SeqProcessCnt == SEQ_Process_load_Pose){
        seq_pSeqCnt = pBuffer[seq_seq_cnt_address];
        if(seq_pSeqCnt > max_seq_index) Packet_Error_Feedback(0x00);
        else {Packet_Error_Feedback(0x01); SeqProcessCnt = SEQ_Process_load_SEQCnt;}
      }
      else Packet_Error_Feedback(0x00);
    }
    else if(pCMD == CMD_SEQ_load_SEQ){//load sequence
      static int sPoseID = 0, sTime = 0, _i;
      seq_trigger = false; SeqPos = 0;
      if(SeqProcessCnt == SEQ_Process_load_SEQCnt){
        sPoseID = pBuffer[seq_pose_name_address];
        sTime = (pBuffer[seq_pose_time_MSB_address] << 8) + pBuffer[seq_pose_time_LSB_address];
        for(_i = 0;_i < max_pose_index;_i++){
          if(pose_index[_i] == sPoseID){
            sequence[SeqCnt].pose = _i; sequence[SeqCnt].time = sTime;
            SeqCnt++; break;
          }
        }
        if(SeqCnt == seq_pSeqCnt){
          Packet_Error_Feedback(0x02);
          SeqCnt = 0; seq_trigger = true;
          XYZrobot.readPose();
          SeqProcessCnt = SEQ_Process_load_SEQ;
        }
        else Packet_Error_Feedback(0x01);
      }
      else Packet_Error_Feedback(0x00);
    }
    else if(pCMD == CMD_SEQ_halt){//halt sequence
      seq_trigger = false;
      //halt sequence
    }
    else if(pCMD == CMD_SEQ_relax){//relax servo
      seq_trigger = false;
      A1_16_TorqueOff(A1_16_Broadcast_ID);
    }
    else if(pCMD == CMD_version_read){
      seq_trigger = false; SeqPos = 0;
      Packet_Version_Read();
    }
  }
  else{Packet_Error_Feedback(0x00); pLength = 0xFF;}
}
void cb_USB(void){
  while(Serial.read() != -1);
  //clear USB serial buffer
}
void Motion_Editor_Seq_Play(void){
  static int _i = 0;
  static int pose_index = 0;
  pose_index = sequence[SeqPos].pose;
  for(_i = 0; _i < XYZrobot.poseSize; _i++) XYZrobot.setNextPose(_i+1, poses[pose_index][_i]);
  XYZrobot.interpolateSetup(sequence[SeqPos].time);
  while(XYZrobot.interpolating) XYZrobot.interpolateStep();
  SeqPos++;
  if(SeqPos == seq_pSeqCnt){
    SeqPos = 0;
    if(seq_loop_trigger) ;
    else{seq_trigger = false; seq_pSeqCnt = 0xFF;}
  }
}
void Packet_Init(unsigned char motor_num){
  Serial.write(packet_header);
  Serial.write(0x05);
  Serial.write(CMD_init_motor);
  Serial.write(motor_num);
  Serial.write(packet_tail);
}
void Packet_Set(unsigned char motor_ID, int pos_set){
  Serial.write(packet_header);
  Serial.write(0x07);
  Serial.write(CMD_set_motor);
  Serial.write(motor_ID);
  Serial.write(((pos_set & 0xFF00) >> 8));
  Serial.write((pos_set & 0x00FF));
  Serial.write(packet_tail);
}
void Packet_Capture(unsigned char motor_ID){
  static int position_buffer[19] = {0};      // position buffer
  static int _i = 0;
  for(_i = 1;_i < 19;_i++) position_buffer[_i] = ReadPosition(_i);
  Serial.write(packet_header);
  Serial.write(0x29);
  Serial.write(CMD_capture_motor);
  Serial.write(motor_ID);
  for(_i = 1;_i < 19;_i++){
    Serial.write(((position_buffer[_i] & 0xFF00) >> 8));
    Serial.write((position_buffer[_i] & 0x00FF));
  }
  Serial.write(packet_tail);
}
void Packet_Relax(unsigned char motor_ID){
  Serial.write(packet_header);
  Serial.write(0x05);
  Serial.write(CMD_relax_motor);
  Serial.write(motor_ID);
  Serial.write(packet_tail);
}
void Packet_Version_Read(void){
  Serial.write(packet_header);
  Serial.write(0x0A);
  Serial.write(CMD_version_read);
  Serial.write(model_Bolide);                 //Model
  Serial.write(type_Others);                  //Type
  Serial.write(application_test);             //Application
  Serial.write(main_version_number);          //Main Version
  Serial.write(secondary_version_number);     //Secondary Version
  Serial.write(revision_number);              //Revision
  Serial.write(packet_tail);
}
void Packet_Error_Feedback(unsigned char CMD_reaction){
  Serial.write(packet_header);
  Serial.write(0x04);
  Serial.write(CMD_reaction);
  Serial.write(packet_tail);
}
void Packet_SN(void){
	static int _i = 0;
	Serial.write(packet_header);
	Serial.write(0x12);
	Serial.write(CMD_SN_read);
	for(_i = 10;_i < 24;_i++) Serial.write(EEPROM.read(_i));
	Serial.write(packet_tail);
}
//Action Task
void Action(uint8_t N){
  XYZrobot.readPose();
  if(N == 1) {if(ActionNo_1 != None) XYZrobot.playSeq(ActionNo_1);}
  else if(N == 2) {if(ActionNo_2 != None) XYZrobot.playSeq(ActionNo_2);}
  else if(N == 3) {if(ActionNo_3 != None) XYZrobot.playSeq(ActionNo_3);}
  else if(N == 4) {if(ActionNo_4 != None) XYZrobot.playSeq(ActionNo_4);}
  else if(N == 5) {if(ActionNo_5 != None) XYZrobot.playSeq(ActionNo_5);}
  else if(N == 6) {if(ActionNo_6 != None) XYZrobot.playSeq(ActionNo_6);}
  else if(N == 7) {if(ActionNo_7 != None) XYZrobot.playSeq(ActionNo_7);}
  else if(N == 8) {if(ActionNo_8 != None) XYZrobot.playSeq(ActionNo_8);}
  else if(N == 9) {if(ActionNo_9 != None) XYZrobot.playSeq(ActionNo_9);}
  else if(N == 10) {if(ActionNo_10 != None) XYZrobot.playSeq(ActionNo_10);}
  else if(N == 11) {if(ActionNo_11 != None) XYZrobot.playSeq(ActionNo_11);}
  else if(N == 12) {if(ActionNo_12 != None) XYZrobot.playSeq(ActionNo_12);}
  else if(N == 13) {if(ActionNo_13 != None) XYZrobot.playSeq(ActionNo_13);}
  else if(N == 14) {if(ActionNo_14 != None) XYZrobot.playSeq(ActionNo_14);}
  else if(N == 15) {if(ActionNo_15 != None) XYZrobot.playSeq(ActionNo_15);}
  else if(N == 16) {if(ActionNo_16 != None) XYZrobot.playSeq(ActionNo_16);}
  else if(N == 17) {if(ActionNo_17 != None) XYZrobot.playSeq(ActionNo_17);}
  else if(N == 18) {if(ActionNo_18 != None) XYZrobot.playSeq(ActionNo_18);}
  else if(N == 19) {if(ActionNo_19 != None) XYZrobot.playSeq(ActionNo_19);}
  else if(N == 20) {if(ActionNo_20 != None) XYZrobot.playSeq(ActionNo_20);}
  else if(N == 21) {if(ActionNo_21 != None) XYZrobot.playSeq(ActionNo_21);}
  else if(N == 22) {if(ActionNo_22 != None) XYZrobot.playSeq(ActionNo_22);}
  else if(N == 23) {if(ActionNo_23 != None) XYZrobot.playSeq(ActionNo_23);}
  else if(N == 24) {if(ActionNo_24 != None) XYZrobot.playSeq(ActionNo_24);}
  else if(N == 25) {if(ActionNo_25 != None) XYZrobot.playSeq(ActionNo_25);}
  else if(N == 26) {if(ActionNo_26 != None) XYZrobot.playSeq(ActionNo_26);}
  else if(N == 27) {if(ActionNo_27 != None) XYZrobot.playSeq(ActionNo_27);}
  else if(N == 28) {if(ActionNo_28 != None) XYZrobot.playSeq(ActionNo_28);}
  else if(N == 29) {if(ActionNo_29 != None) XYZrobot.playSeq(ActionNo_29);}
  else if(N == 30) {if(ActionNo_30 != None) XYZrobot.playSeq(ActionNo_30);}
  else if(N == 31) {if(ActionNo_31 != None) XYZrobot.playSeq(ActionNo_31);}
  else if(N == 32) {if(ActionNo_32 != None) XYZrobot.playSeq(ActionNo_32);}  
  else if(N == 33) {if(ActionNo_33 != None) XYZrobot.playSeq(ActionNo_33);}
  else if(N == 34) {if(ActionNo_34 != None) XYZrobot.playSeq(ActionNo_34);}
  else if(N == 35) {if(ActionNo_35 != None) XYZrobot.playSeq(ActionNo_35);}
  else if(N == 36) {if(ActionNo_36 != None) XYZrobot.playSeq(ActionNo_36);}
  else if(N == 37) {if(ActionNo_37 != None) XYZrobot.playSeq(ActionNo_37);}
  else if(N == 38) {if(ActionNo_38 != None) XYZrobot.playSeq(ActionNo_38);}
  else if(N == 39) {if(ActionNo_39 != None) XYZrobot.playSeq(ActionNo_39);}
  else if(N == 40) {if(ActionNo_40 != None) XYZrobot.playSeq(ActionNo_40);}
  else if(N == 41) {if(ActionNo_41 != None) XYZrobot.playSeq(ActionNo_41);}
  else if(N == 42) {if(ActionNo_42 != None) XYZrobot.playSeq(ActionNo_42);}
  else if(N == 43) {if(ActionNo_43 != None) XYZrobot.playSeq(ActionNo_43);}
  else if(N == 44) {if(ActionNo_44 != None) XYZrobot.playSeq(ActionNo_44);}
  else if(N == 45) {if(ActionNo_45 != None) XYZrobot.playSeq(ActionNo_45);}
  else if(N == 46) {if(ActionNo_46 != None) XYZrobot.playSeq(ActionNo_46);}
  else if(N == 47) {if(ActionNo_47 != None) XYZrobot.playSeq(ActionNo_47);}
  else if(N == 48) {if(ActionNo_48 != None) XYZrobot.playSeq(ActionNo_48);}
  else if(N == 49) {if(ActionNo_49 != None) XYZrobot.playSeq(ActionNo_49);}
  else if(N == 50) {if(ActionNo_50 != None) XYZrobot.playSeq(ActionNo_50);}
  else if(N == 51) {if(ActionNo_51 != None) XYZrobot.playSeq(ActionNo_51);}
  else if(N == 52) {if(ActionNo_52 != None) XYZrobot.playSeq(ActionNo_52);}
  else if(N == 53) {if(ActionNo_53 != None) XYZrobot.playSeq(ActionNo_53);}
  else if(N == 54) {if(ActionNo_54 != None) XYZrobot.playSeq(ActionNo_54);}
  else {XYZrobot.playSeq(DefaultInitial);}
  
  while((XYZrobot.playing) && !(BT_Packet_Task())){
    XYZrobot.play();
  }
  if(torque_release){
    A1_16_TorqueOff(A1_16_Broadcast_ID);
    torque_release = false;
  }
}

//BT Reading Task
boolean BT_Packet_Task(void){
  //return torque_relase button status
  static int temp_packet[7] = {0};
  static char _i = 0;
  if(Serial2.available() >= 7){
    if((temp_packet[0] = Serial2.read()) == 0) ; else {find_header_BT(); return false;}
    if((temp_packet[1] = Serial2.read()) == 0) {find_header_BT(); return false;}
    if((temp_packet[2] = Serial2.read()) == 0) {find_header_BT(); return false;}
    if((temp_packet[3] = Serial2.read()) == 0) {find_header_BT(); return false;}
    if((temp_packet[4] = Serial2.read()) == 0) {find_header_BT(); return false;}
    if((temp_packet[5] = Serial2.read()) == 0) {find_header_BT(); return false;}
    if((temp_packet[6] = Serial2.read()) == 0) {find_header_BT(); return false;}
    if(temp_packet[1] != 255 && temp_packet[2] != 1){
      Serial2.write((temp_packet[6]&0x00F0)>>4);
    } 
    
    for(_i = 0;_i < 7 ;_i++) packet[_i] = temp_packet[_i];
    BT_update = true;
    cb_BT();
    if((packet[1]!= 255 && packet[2]!=1)&&((packet[5]&0x0010)>>3)) {
      torque_release = true;
      return true;
    }
    else if(packet[1]==255 && packet[2]==1 && packet[3]==102) {
      torque_release = true;
      return true;
    }
    else{
      torque_release = false;
      return false;
    }
  }
  return false;
}

// BT FW Feedback 
void BT_FW(){
  Serial2.write(0xFF);               // packet head
  delay(50);
  Serial2.write(model_Bolide);       //Model
  delay(50);
  Serial2.write(type_Others);        //Type
  delay(50);
  Serial2.write(application_test);   //Application
  delay(50);
  Serial2.write(0x01);               //Main Version
  delay(50);
  Serial2.write(0x02);               //Secondary Version
  delay(50);
  Serial2.write(0x00);               //Revison
  delay(50);
}

// Clean BT Buffer
void cb_BT(void){  
  while((Serial2.read()) != -1);
}
void find_header_BT(void){
  static int cb = 0x00;
  while(Serial2.available() > 0){
    if(Serial2.peek() == 0) return;
    cb = Serial2.read();
  }
}

// Speaker function
void MusicPlaying_wav_play(char song_name[]){
  Serial3.write(0);
  Serial3.print("P");
  Serial3.write(song_name); //set the filename of song : 0000 ~ 9999    
}
void MusicPlaying_wav_stop(){
  Serial3.write(0);
  Serial3.print("S0000");
}
void MusicPlaying_wav_volume(int volume){
  Serial3.write(0);
  Serial3.write('V');  
  Serial3.write(volume);// volume : 0x01 ~ 0x7F 
  Serial3.print("000");
}

// Buzzer function : play start music
void Start_Music(void){
  int _i = 0x00;
  //int duration[8]={200,200,200,200,200,200,400,};
  for(_i = 0 ; _i < 7; _i++){
    tone(BUZZER_PIN, pgm_read_word_near(&start_music_frq[_i]));
    delay(300);
    noTone(BUZZER_PIN);    
  }
}

// Button function
void BUTTON_Task(void){
  static unsigned char button_timer = 0x00;
  static int key = 0x00, last_key = 0x00;
  key = !digitalRead(BUTTON1_PIN) + ((!digitalRead(BUTTON2_PIN))<<1) + ((!digitalRead(BUTTON3_PIN))<<2) + ((!digitalRead(BUTTON4_PIN))<<3);
  if(key != last_key) button_timer++;
  else button_timer = 0;
  if(button_timer > 20){
    button_timer = 0;
    last_key = key;
    if(key != 0){
      XYZrobot.readPose();
      if(last_key == key_mask_button1){
            if(RB_1 != 0)  Action(RB_1);
            else Action(52);
      }
      else if(last_key == key_mask_button2) {
            if(RB_2 != 0)  Action(RB_2);
            else Action(53);
      }
      else if(last_key == key_mask_button3) {
            if(RB_3 != 0)  Action(RB_3);
            else Action(54);
      }
      else if(last_key == key_mask_button4) {
            if(RB_4 != 0)  Action(RB_4);
            else Action(51);
      }
    }
  }
}

// Power Detection function
void Power_Detection_Task(void){
  static double PWR_Voltage;
  PWR_Voltage = analogRead(PWRDET_PIN)*0.0124;
  if(PWR_Voltage < Power_Voltage_Alarm) tone(BUZZER_PIN,1000);  
}
void Initial_Pose_Setup(void){
  XYZrobot.readPose();
  XYZrobot.playSeq(DefaultInitial);
  while(XYZrobot.playing) XYZrobot.play();
}
boolean Timer_Task(unsigned long _time_zone){
  static unsigned long _last_time;
  if(_time_zone == 0){
    _last_time = millis();
    return false;
  }
  else{
    if((millis() - _last_time) > _time_zone){
      _last_time = millis();
      return true;
    }
    else return false;
  }
}
void Timer_Task_Setup(void){
  // Set Timer4 as a normal timer for communcation timeout
  TCCR4A = 0x00;
  TCCR4B |= _BV(CS42);
  TCCR4B &= ~_BV(CS41);
  TCCR4B |= _BV(CS40);
  _enable_timer4();
}
/*==== Interrupt Functions ====*/
ISR(TIMER4_OVF_vect){
  Power_Detection_Task();
  packet_timeout_status = true;
  _reset_timer4(timeout_limit);
}

