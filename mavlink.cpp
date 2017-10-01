/****************************************************
* This is a MEX-FILE for matlab. 
* function name: mavlink
    *this module performs two main tasks:
    * 1- it receives mavlink packets from MATLAB in a buffer and decodes it
        * and retrieve useful information e.g. attitude angles
    *2- return a buffer to matlab to be sent, that contains mavlink messages
        * that are chosen by user input

 ** This Module does not perfrom any serial configuration or communication**
*
*
*
* -- For Educational Use Only
*
* Last updated by : Mohamed Abdelkader, Sept 9th 2015
 * several bugs were fixed
 * removing all realted serial functions
 * adding help function  {needs update}
 * add 'offboard' mode to control Pixhawk
 * add offboard command option (e.g. set position/attitude/velocity) {TBD}

* Credits:
* the following are depricated and not used
*   - previous version (MAVLink_GUI) by Chad
*   -Serial communication part based on SerialIO.cpp, last updated by
*    Lorgio Teodovich, november 2009, available at MATLAB Central (NOT USED)
*
*****************************************************/



/*******************
*Includes
*******************/
//#include <windows.h>

//#undef EXTERN_C

//#include <process.h>
#include "mex.h"
//#include "string.h"
#include "matrix.h"

//#define WIN32_LEAN_AND_MEAN

#define MAVLINK_CRC_EXTRA 1
//#define NATIVE_BIG_ENDIAN
#include "include/mavlink/mavlink_types.h"
#include "include/mavlink/common/mavlink.h"
//#include "include/mavlink/pixhawk/mavlink.h"

#include "time.h"



//using namespace std;
// Can't include Message_Decoder here because haven't made structs yet
// #include "Message_Decoder.h" 
// after structs initialized

//From mxcreatestructarray.c - This means array/struct sizes can change without having to change all code
#define NUMBER_OF_VEHICLES (sizeof(Vehicles)/sizeof(struct Vehicle))
#define NUMBER_OF_FIELDS (sizeof(Vehicle_field_names)/sizeof(*Vehicle_field_names))
#define NUMBER_OF_MESSAGES (sizeof(Messages)/sizeof(struct Message))
#define NUMBER_OF_FIELDS (sizeof(Vehicle_field_names)/sizeof(*Vehicle_field_names))
#define B(x) S_to_binary_(#x)


// convert to binary
static inline unsigned long long S_to_binary_(const char *s)
{
    unsigned long long i = 0;
    while (*s) {
        i <<= 1;
        i += *s++ - '0';
    }
    return i;
}

//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     B(0000110111111000)
//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     B(0000110111000111)
//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION B(0000110000111111)
//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        B(0000111000111111)
//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    B(0000100111111111)
//#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     B0000010111111111)

/*******************
*Global Declarations
*******************/

/* pointer to Bytes read from serial port in ReadAllBytes function
* Must be passed to keep track of number of bytes read.
 */
unsigned char *InputBuffer;

//uint8_t *OutputBuffer;

// define input buffer
//#define Inp_Buff_sz 1024
//unsigned char InputBuffer[Inp_Buff_sz] = {0};

//output buffer
// used in  buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &msg);
#define Out_Buff_sz 300
uint8_t OutputBuffer[Out_Buff_sz]={0};

uint16_t msg_len, buf_len, chksum;
int BytesToRead=0;


/* This is to set size of variables in initialization.
* Maximum number of vehicles to handle.
 **/
const int Max_Num_Vehicles = 5;

// This will keep track of how many vehicles currently exist
int Current_Number_Vehicles = 0;

const int Max_Route_Length = 16;

// UPDATE these if changing fields!
const char *Vehicle_field_names[] = {"updated","ID","Component_ID","type","AP_type","base_mode","custom_mode","roll", "pitch", "yaw", "lat","lon","alt","hdg","action","nav_bearing","target_bearing","target_distance","nav_roll","nav_pitch","nav_yaw","voltage_battery","sys_status","localNEDx","localNEDy","localNEDz","abs_pressure","diff_pressure","att_tstamp","gps_tstamp","highers_tstamp","localned_tstamp", "scaled_abs_press", "scaled_diff_press", "scaled_press_tms","xacc","yacc","zacc","xgyro","ygyro","zgyro","rollspeed","pitchspeed","yawspeed","vx","vy","vz","q0_t","q1_t","q2_t","q3_t","roll_rate_t","pitch_rate_t","yaw_rate_t","thrust_t","att_t_ms"};

//From mxcreatestructarray.c
struct Vehicle
{
  bool updated;
  uint8_t ID;
  uint8_t Component_ID;
  uint8_t type;
  uint8_t AP_type;
  uint8_t base_mode;
  uint32_t custom_mode;
  float roll;
  float pitch;
  float yaw;
  int32_t lat;///*< Latitude (WGS84), in degrees * 1E7*/
  int32_t lon;/*< Longitude (WGS84), in degrees * 1E7*/
  int32_t alt;/*< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up).*/
  uint16_t hdg;
  uint8_t action;
  int16_t nav_bearing;
  int16_t target_bearing;
  uint16_t target_distance;
  float nav_roll;
  float nav_pitch;
  float nav_yaw;
  uint16_t voltage_battery;
  uint8_t sys_status;
  float localNEDx;
  float localNEDy;
  float localNEDz;
  float abs_pressure;
  float diff_pressure;
  uint32_t att_tstamp;
  uint64_t gps_tstamp;
  uint64_t highers_tstamp;
  uint32_t localned_tstamp;
  float scaled_abs_press;
  float scaled_diff_press;
  uint32_t scaled_press_tms;
  float xacc;
  float yacc;
  float zacc;
  float xgyro;
  float ygyro;
  float zgyro;
  float rollspeed;// rad/s -- NEW*****
  float pitchspeed;
  float yawspeed;
  float vx;//in NED m/s
  float vy;
  float vz;
  float q0_t;//target attitude, quaternion
  float q1_t;
  float q2_t;
  float q3_t;
  float roll_rate_t; //target body roll rate rad/s
  float pitch_rate_t;
  float yaw_rate_t;
  float thrust_t; //collective thrust [0-1]
  uint32_t att_target_time_boot_ms;
};

struct Ground_Station {        // probably only need one, still a struct?
    uint8_t ID;                 //  System ID, MP used 255
    uint8_t Component_ID;       //  Component ID, 190 means Mission Planner 
    uint8_t type;               //  GCS is type 6, FW is type 1
    uint8_t Autopilot_type;     //  8 is type invalid
    uint8_t mode;               //  1 is custom, 4 is auto
    uint8_t system_state;       //  0 is unknown/uninitialized
    uint32_t custom_mode;       //  I don't know what this does, in heartbeat
    
};

const struct Vehicle Default_Vehicle = {
    0,  // Updated defaults to 0
    0,  // Vehicle ID, nonexistant vehicles are zeroed
    0,  // Component ID, 0 is all
    1,  // Vehicle type is 1 = FW
    3,  // Vehicle autopilot is 3 = APM
    1, //base mode
    0, // custom mode
    -1000,  // Roll
    -1000,  // Pitch
    -1000,  // Yaw
    0,  // Lat
    0,  // Lon
    0,  // Alt
    0, //hdg
    0,  // action
    0,  // nav_bearing
    0,  // target_bearing
    0,  // target_distance
    0,  // nav_roll
    0,  // nav_pitch
    0,  // target_yaw
    0,  // voltage_battery
    255,//MAV_STATE_STANDBY, //sys_status
    -123456, //localNEDx
    -123456, //localNEDy
    -123456, //localNEDz
    -123456, // absolute pressure
    -123456, // differential pressure
    0, // att_tstamp;
    0, // gps_tstamp;
    0, //highers_tstamp;
    0, // localned_tstamp;
    -123456, //scaled_abs_press
    -123456, //scaled_diff_press
    0,   //scaled_press_tms
    -123456.0,// xacc m/s^2
    -123456.0,// yacc m/s^2
    -123456.0,// zacc m/s^2
    -123456.0,// xgyro rad/s
    -123456.0,// ygyro rad/s
    -123456.0,// zgyro rad/s
    -123456.0, //roll speed rad/s
    -123456.0, // pitch speed
    -123456.0, // yaw speed
    -123456.0, // vx NED
    -123456.0, //vy
    -123456.0, //vz
    1,// q0, quaternion
    0, //q1
    0,//q2
    0,//q3
    -123456.0,// roll target rate
    -123456.0, //pitch target rate
    -123456.0,// yaw target rate, rad/s
    -123456.0,// target thrust
    0// time stamp for target attitude mesg
};
   
const struct Ground_Station GCS = {
    
    200,    // This is my ID, I am the GCS
    191,    // 190 is MissionPlanner - seems somewhat appropriate?            
    6,      // GCS type is 6.  FW type is 1
    8,      // 8 is for invalid autopilot, like a GCS
            //with no autopilot. APM is autopilot type 3
    1,      // Mode 1 is custom, Mode 4 is auto, 
            //Don't know if GCS needs mode...
    0,      // Changed to 0 which is unknown/uninitialized.  Snooped MP 
            // heartbeats are 0.
    0       // Custom Mode.  Not used?
};
   
struct Vehicle Vehicles[Max_Num_Vehicles];
    
// Vehicle This_Vehicle = Default_Vehicle; // So I can deal with one at a time

#include "Message_Decoder.h" // This is switch/case for handling incoming messages
// needs to be after declaring Vehicles
//=========================================================================
// FUNCTION DEFINITIONS
//=========================================================================

void help()
{
    printf("            MAVLink Coding/Decoding via MATLAB            \n");
    printf("-----------------------------------------------------------\n");
    printf("The following is the syntax of some of the possible use of the 'mavlink' function,\n");
    printf("\n");
    printf("1- Send Ping message:\n");
    printf("    [buffer,buf_length]=mavlink('ping')\n");
    printf("==============================================================\n");
    
    printf("2- Request Data stream:\n");
    printf("    [buffer,buf_length]=mavlink('RequestDataStream', Sys_ID,target_comp_id, stream_ID, message_rate,start_stop)\n");
    printf("Sys_ID: the id of target system, stream_ID: which message is requested, rate: rate of sending Hz, start_stop: 1 on, 0 off\n");
    printf("consult http://qgroundcontrol.org/mavlink/start for more info \n");
    printf("also consult https://pixhawk.ethz.ch/mavlink \n");
    printf("==============================================================\n");
    
    printf("3- set mode: auto, manual, ..etc \n");
    printf("    [buffer,buf_length]=mavlink('SetMode', sys_id, base_mode, custom_mode)\n");
    printf("==============================================================\n");
    
    printf("4- send heart beat:\n");
    printf("    [buffer,buf_length]=mavlink('heartbeat')\n");
    printf("==============================================================\n");
    
    printf("5- Request the list of all parameters: \n");
    printf("    [buffer,buf_length]=mavlink('readallparams',sys_id)\n");
    printf("sys_id: target system's id\n");
    printf("==============================================================\n");
    
    printf("6- Read incoming mavlink messages:\n");
    printf("    vehicle=mavlink('readmessage',input_buffer,bytes_to_read)\n");
    printf("    input_buffer: buffer read from the serial port (uint8)\n");
    printf("==============================================================\n");
        
    printf("7- Arming/Disarming MAV:\n");
    printf("    [buffer,buf_length]=mavlink('Arming',sys_id,flag)\n");
    printf("sys_id: id of the target MAV,   flag: 1(arm), 0 (disarm)\n");
    printf("==============================================================\n");
    
    printf("8- Toggling Offboard mode\n");
    printf("    [buffer,buf_length]=mavlink('offboard',sys_id,comp_id,flag)\n");
    printf("sys_id: id of the target MAV, comp_id: target component id   flag: 1(start), 0 (stop)\n");
    printf("==============================================================\n");
    
    printf("9- Send vision information: position(m)/attitude(rad)\n");
    printf("    [buffer,buf_length]=mavlink('sendVisionEst',target_system,target_comp, x, y, z, roll, pitch, yaw)\n");
    printf("sys_id: id of the target MAV, comp_id: target component id, xyz in m, attitude in rad\n");
    printf("==============================================================\n");
    
    printf("10- Send motion capture system data: quaternion (w,x,y,z) in NED, xyz(meters in NED)\n");
    printf("    [buffer,buf_length]=mavlink('mocap',target_system,target_comp, q.w, q.x, q.y, q.z, x, y, z)\n");
    printf("target_system: id of the target MAV, target_comp: target component id,\n");
    printf("NOTE: make sure you input the data in NED frame!!)\n");
    printf("==============================================================\n");
    
    printf("11- Send setpoints: NED position XYZ(m), yaw (rad), vx vy vz(m/s)\n");
    printf("    [buffer,buf_length]=mavlink('setPoints',target_system,target_comp, x, y, z, yaw, vx, vy, vz, MASK)\n");
    printf("target_system: id of the target MAV, target_comp: target component id,\n");
    printf("MASk: to choose which setpoint to set: (1) xyz only, (2) xyz and yaw, (3) vx vy vz and yaw, default is (2)\n");
    printf("==============================================================\n \n");
    
    printf("12- Activate Manual mode\n");
    printf("    [buffer,buf_length]=mavlink('setManual',sys_id,comp_id)\n");
    printf("==============================================================\n \n");
    
    printf("13- Set a parameter\n");
    printf("    [buffer,buf_length]=mavlink('setParameter',target_sys,target_comp,param_name,param_value)\n");
    printf("==============================================================\n \n");
    
    printf("14- Set actuators commands: duty\n");
    printf("    [buffer,buf_length]=mavlink('setActuators',sys_id, comp_id, act1,..., act8);\n");
    printf("act1: is the the duty cycle (normalized to -1->1) for actuator 1 \n");
    printf("==============================================================\n \n");
    //mavlink('setActuators',sys_id, comp_id, act1,..., act8);
    
    printf("buffer: can be sent using serial communication modules, such as in MATLAB (fwrite)\n");
    
    
}
void ReturnBuffer(const unsigned char *buff_in,int len, mxArray *out[]){
    
    mwSize dims[2]={1,len};
    mwSize dims2[2]={1,1};
    unsigned char *start_of_pr;
    size_t bytes_to_copy;
    if (len>0)
    {
        out[0]=mxCreateNumericArray(2, dims, mxUINT8_CLASS, mxREAL);
        start_of_pr = (unsigned char *)mxGetData(out[0]);
        bytes_to_copy = len * mxGetElementSize(out[0]);
        memcpy(start_of_pr,buff_in,bytes_to_copy);

        out[1]=mxCreateDoubleScalar((float) len);
    }
    else// no data to write, return length=0 and buffer[0]=0;
    {
        const unsigned char buff_in[]={0};
        out[1]=mxCreateDoubleScalar((float) len);
        out[0]=mxCreateNumericArray(2, dims2, mxUINT8_CLASS, mxREAL);
        start_of_pr = (unsigned char *)mxGetData(out[0]);
        bytes_to_copy = 1 * mxGetElementSize(out[0]);
        memcpy(start_of_pr,buff_in,bytes_to_copy);
    }
}


/**************************************************************************
 * the MATLAB interface function
 * 
 *************************************************************************/
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
    
    //printf("In Mex\n");
    //mexEvalString("drawnow;");
    
    bool Successful = false;
    char *commandName;
    int commandNum = 0;
    int Length0, i;
    mxArray *InArray;
    double *xValues;
    char *xCharData;
    uint8_t Comp_ID = 0;  // This is all components.  Should check and see if I need to specify on MATLAB side or can be all
    
    
    
    

    
    //Convert the user inputs
    if(nrhs >= 1){
        commandName = (char *) mxArrayToString(prhs[0]);
        if( (strcmp(commandName,"help")== 0)&&(nrhs == 1)){
            help();
        }
        
           
        // Send Ping message
        else if( (strcmp(commandName,"ping")== 0)){
            if(nrhs != 1){
                printf("Invalid ping syntax\n");
            }
            else{
                
                mavlink_message_t tx_msg;
                time_t ltime;
                uint64_t time_usec = time(&ltime);
                
                mavlink_msg_ping_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        time_usec, 1, 0, 0);
                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }    
        }
        
        else if( (strcmp(commandName,"RequestDataStream")== 0)){
            if(nrhs != 6){
                printf("Invalid Data Stream Syntax syntax\n");
            }
            else{
                
                // Stream_ID:
                // 0: all
                // 1: 27,29 - Raw IMU, Scaled Pressure
                // 2: 1,42, 24, 62 - SYS_STATUS, MISSION_CURRENT, GPW_RAW_INT, NAV_CONTROLLER_OUTPUT
                // 3: 36, 35 - RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
                // 4: 34 - RC_CHANNELS_SCALED
                // 5: none
                // 6: 33 - GLOBAL_POSITION_INT - Use This One!!!!!!!!!!!
                // 7: none
                // 8: none
                // 9: none
                // 10: 30 (attitude)
                // 11: 74 (VFR Hud)
                // 12: none
                
                // Message Rate: times per second to transmit data
                // start_stop: 1 for on, 0 for off.
                
                uint8_t Sys_ID = (int)mxGetScalar(prhs[1]);
                uint8_t comp_id = (int)mxGetScalar(prhs[2]); 
                uint8_t stream_ID = (int)mxGetScalar(prhs[3]); 
                uint8_t message_rate = (int)mxGetScalar(prhs[4]);     
                uint8_t start_stop = (int)mxGetScalar(prhs[5]);     
                
                mavlink_message_t tx_msg;
                mavlink_msg_request_data_stream_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, comp_id, stream_ID, message_rate, start_stop); // 0 for component is all
                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }    
        }
        
        
        //test attitude sending
        else if( (strcmp(commandName,"attitude")== 0)){
            if(nrhs != 1){
                printf("Invalid attitude syntax\n");
            }
            else{
                
                mavlink_message_t tx_msg;
                
                mavlink_msg_attitude_pack(GCS.ID, GCS.Component_ID, &tx_msg,
						       (uint32_t) 123, 1.1, 1.2, 1.3, 0.0, 0.0, 0.0);
                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }    
        }
        
        
        
        else if( (strcmp(commandName,"SetMode")== 0)){ // Set autopilot mode (Auto, manual, etc.)
            if(nrhs != 4){
                printf("Invalid Set Mode syntax\n");
            }
            else{
                
                uint8_t Sys_ID = (int)mxGetScalar(prhs[1]); 
                uint8_t base_mode = (int)mxGetScalar(prhs[2]); // Mode, set using bit flags, 
                                                               // 4 (00000100) is auto
                uint8_t custom_mode = (int)mxGetScalar(prhs[3]); // Mode, set using bit flags, 
                                                               // 4 (00000100) is auto
                        
                mavlink_message_t tx_msg;
                mavlink_msg_set_mode_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, base_mode, custom_mode);
                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }    
        }
        
        else if( (strcmp(commandName,"MissionCount")== 0)){ // MP does this before sending new points
            if(nrhs != 3){
                printf("Invalid Mission Count Syntax syntax\n");
            }
            else{
                
                uint8_t Sys_ID = (int)mxGetScalar(prhs[1]); 
                uint16_t count = (int)mxGetScalar(prhs[1]); // Sequence
                        
                mavlink_message_t tx_msg;
                mavlink_msg_mission_count_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, Comp_ID, count);
                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }    
        }
        
        
        else if( (strcmp(commandName,"MissionItem")== 0)){
            if(nrhs != 14){
                printf("Invalid Mission Item Syntax syntax\n");
            }
            else{
                
                uint8_t Sys_ID = (int)mxGetScalar(prhs[1]); 
                uint16_t seq = (int)mxGetScalar(prhs[2]); // Sequence
                uint8_t frame = (int)mxGetScalar(prhs[3]); // 0 = Global, 1 = local NED, 2 = Mission (no frame) 3 = Global/relative alt, 4 = local ENU
                uint16_t command = (int)mxGetScalar(prhs[4]); // command number, see MAV_CMD
                uint8_t current = (int)mxGetScalar(prhs[5]); // is this current mission? 1 = true
                uint8_t autocontinue = (int)mxGetScalar(prhs[6]); // 1 = true
                float param1 = (float)mxGetScalar(prhs[7]); // radius of mission item, hor far away is 'reached'
                float param2 = (float)mxGetScalar(prhs[8]); // how long to stay inside radius until proceeding (RW only?)
                float param3 = (float)mxGetScalar(prhs[9]); // for Loiter - radius to orbit.  positive = CW
                float param4 = (float)mxGetScalar(prhs[10]); // Yaw orientation (RW only?)
                float x = (float)mxGetScalar(prhs[11]);
                float y = (float)mxGetScalar(prhs[12]);
                float z = (float)mxGetScalar(prhs[13]);
                        
                mavlink_message_t tx_msg;
                mavlink_msg_mission_item_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, Comp_ID, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);
                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }    
        }
        
        
            // Send mission request list message asks for list of missions
            // specific requests for mission information need to include which one.
        else if( (strcmp(commandName,"missionrequestlist")== 0)){
            if(nrhs != 2){
                printf("Invalid mission request list syntax\n");
            }
            else{
                
                uint8_t Sys_ID = (int)mxGetScalar(prhs[1]); 
                
                mavlink_message_t tx_msg;
                
                mavlink_msg_mission_request_list_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, Comp_ID);
                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }    
        }
        
        // Should start by identifying how many things to write, then waiting for response
        else if( (strcmp(commandName,"missionwritelist")== 0)){
            if(nrhs != 4){
                printf("Invalid mission write list syntax\n");
            }
            else{
                
                mavlink_message_t tx_msg;
                
                uint8_t Sys_ID = (int)mxGetScalar(prhs[1]); 
                uint16_t start_index = (int)mxGetScalar(prhs[2]); 
                uint16_t end_index = (int)mxGetScalar(prhs[3]);    
                
                mavlink_msg_mission_write_partial_list_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, Comp_ID, start_index, end_index);
                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }    
        }
        
        
        // Send mission request asks for information on specific mission
        else if( (strcmp(commandName,"missionrequest")== 0)){
            if(nrhs != 3){ // must include which mission to look for
                printf("Invalid mission request list syntax\n");
            }
            else{
                
                uint8_t Sys_ID = (int)mxGetScalar(prhs[1]); 
                uint16_t sequence = (int)mxGetScalar(prhs[2]);
                mavlink_message_t tx_msg;
                
                mavlink_msg_mission_request_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                       Sys_ID, Comp_ID, sequence);
                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }    
        }
        
        // This currently uses the _command_long_ message to send a command
        // Any command is possible, I built for NAV_WAYPOINT, which is 16
        else if( (strcmp(commandName,"missioncmd")== 0)){
            if(nrhs != 10){ 
                printf("Invalid mission write syntax\n");
            }
            //MAV_CMD_NAV_WAYPOINT 
            // Param #1 - Hold time in decimal seconds for Rotary (ignor for FW)
            // Param #2 - Acceptance radius in Meters (how close before you are 'there')
            // Param #3 - how far away to pass (radius in Loiter) use 0 to pass over, + for CW, - for CCW
            // Param #4 - Yaw Angle - rotary (ignored, or zero, or null?)
            // Param 5, 6, 7 - Lat, Long, Alt
            
            // NOTE: if use mission point instead, can be given in local coords
            
            else{
                
                uint8_t Sys_ID = (int)mxGetScalar(prhs[1]); 
                int command = (int)mxGetScalar(prhs[2]); // prhs counts from 0 up.  number 1 is the second
                float p1 = (float)mxGetScalar(prhs[3]);     
                float p2 = (float)mxGetScalar(prhs[4]);
                float p3 = (float)mxGetScalar(prhs[5]);
                float p4 = (float)mxGetScalar(prhs[6]);
                float p5 = (float)mxGetScalar(prhs[7]);
                float p6 = (float)mxGetScalar(prhs[8]);
                float p7 = (float)mxGetScalar(prhs[9]); // Reading a prhs past 
                                                        //what is input will compile, and then crash matlab
                
                uint8_t confirmation = 0; // some commands transmit multiple times for confirmation (like kill command)
                mavlink_message_t tx_msg;
                
                mavlink_msg_command_long_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, Comp_ID, command, confirmation, p1, p2, p3, p4, p5, p6, 0);
                
                                    printf("msg packed \n");
                                    //mexEvalString("drawnow;");
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }    
        }
        
        //Control gimbal via Mavlink
        else if( (strcmp(commandName,"gimbalCommand")== 0)){
            if(nrhs != 6){ 
                printf("Invalid gimbal control syntax\n");
            }
            /* 
             * Inputs
             *  ('gimbalCommand', 'target_system', 'target_comp', pitch, roll, yaw)
             **/
            //MAV_CMD_DO_MOUNT_CONTROL 
            // Param #1 - pitch or lat in degrees, depending on mount mode
            // Param #2 - roll or lon in degrees depending on mount mode
            // Param #3 - yaw or alt (in meters) depending on mount mode
            // Param #4 - reserved
            // Param #5 - reserved
            // Param #6 - reserved
            // Param #7 - MAV_MOUNT_MODE enum value
            /* MAV_MOUNT_MODE:
             *0	MAV_MOUNT_MODE_RETRACT	Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization

                1	MAV_MOUNT_MODE_NEUTRAL	Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.

                2	MAV_MOUNT_MODE_MAVLINK_TARGETING	Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization

                3	MAV_MOUNT_MODE_RC_TARGETING	Load neutral position and start RC Roll,Pitch,Yaw control with stabilization

                4	MAV_MOUNT_MODE_GPS_POINT	Load neutral position and start to point to Lat,Lon,Alt
             */
            
            else{
                int command = MAV_CMD_DO_MOUNT_CONTROL;
                
                uint8_t Sys_ID = (uint8_t)mxGetScalar(prhs[1]); 
                uint8_t Comp_ID= (uint8_t)mxGetScalar(prhs[2]);
                
                float p1 = (float)mxGetScalar(prhs[3]); // roll     
                float p2 = (float)mxGetScalar(prhs[4]); // pitch
                float p3 = (float)mxGetScalar(prhs[5]); // yaw
                float p4 = 0;
                float p5 = 0;
                float p6 = 0;
                float p7 = MAV_MOUNT_MODE_MAVLINK_TARGETING; // Reading a prhs past 
                                                        //what is input will compile, and then crash matlab
                
                uint8_t confirmation = 0; // some commands transmit multiple times for confirmation (like kill command)
                mavlink_message_t tx_msg;
                
                mavlink_msg_command_long_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, Comp_ID, command, confirmation, p1, p2, p3, p4, p5, p6, p7);
                
                                    //printf("msg packed \n");
                                    //mexEvalString("drawnow;");
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }    
        }// End: gimbalCommand
        
        
        
        // Send heartbeat message
        else if( (strcmp(commandName,"heartbeat")== 0)){
            if(nrhs != 1){
                printf("Invalid heartbeat syntax\n");
            }
            else{
                mavlink_message_t tx_msg;
                //mavlink_heartbeat_t hb_msg;
                
                mavlink_msg_heartbeat_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        GCS.type, GCS.Autopilot_type, GCS.mode, GCS.custom_mode, GCS.system_state);
                //mavlink_msg_heartbeat_encode(GCS.ID, GCS.Component_ID, &tx_msg, &hb_msg);
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
                
            }    
        }
        
        else if( (strcmp(commandName,"clearmissions")== 0)){
            if(nrhs != 2){
                printf("Invalid clear mission syntax\n");
            }
            else{
                mavlink_message_t tx_msg;
                
                uint8_t Sys_ID = (int)mxGetScalar(prhs[1]); 
                
                mavlink_msg_mission_clear_all_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, Comp_ID);
                                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
                
            }    
        }
        
        
                // Send change operator control message
        else if( (strcmp(commandName,"changecontrol")== 0)){
            if(nrhs != 2){
                printf("Invalid change control syntax\n");
            }
            else{
                
                uint8_t Sys_ID = (int)mxGetScalar(prhs[1]); 
                
                mavlink_message_t tx_msg;
                uint8_t control_request = 0; // what is control request?
                uint8_t version; // probably defined somewhere
                const char *passkey = "passkey";
                mavlink_msg_change_operator_control_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, control_request, version, passkey);
                                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
                
            }    
        }
        
        // Arm/Disarm MAV
        // mavlink('Arming', sys_id,arm=0:off/1:on)
        else if ((strcmp(commandName,"Arming")== 0)){
            if(nrhs != 3){
                printf("Invalid Arming syntax\n");
            }
            else{
                uint8_t Sys_ID = (uint8_t)mxGetScalar(prhs[1]);
                float arm=(float)mxGetScalar(prhs[2]);
                
                // Comp_ID; // all target components
                uint8_t confirmation = 0; // some commands transmit multiple times for confirmation (like kill command)
                mavlink_message_t tx_msg;
                uint16_t command= MAV_CMD_COMPONENT_ARM_DISARM; 
                 
                mavlink_msg_command_long_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, Comp_ID, MAV_CMD_COMPONENT_ARM_DISARM, confirmation, arm, 0, 0, 0, 0, 0, 0);
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);

            }
        }
        
        // offboard mode
        // first toggle the mode ON 'once', them start sending command
        // Prepare command for off-board mode
        // mavlink('offboard',sys_id,com_id,flag)
        else if ((strcmp(commandName,"offboard")== 0)){
            if(nrhs != 4){
                printf("Invalid offboard syntax\n");
            }
            else{
                uint8_t Sys_ID = (uint8_t)mxGetScalar(prhs[1]);
                uint8_t autopilot_id = (uint8_t)mxGetScalar(prhs[2]);
                float flag=(float)mxGetScalar(prhs[3]);
                
                // Comp_ID; // all target components
                mavlink_message_t tx_msg;
                mavlink_command_long_t com;
                
                com.target_system    = Sys_ID;
                com.target_component = autopilot_id;
                com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
                com.confirmation     = true;
                com.param1           = flag; // flag >0.5 => start, <0.5 => stop

                // Encode
                mavlink_msg_command_long_encode(GCS.ID, GCS.Component_ID, &tx_msg, &com);
                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);

            }
        } // end offboard command
        
        
        // set MANUAL mode

        else if ((strcmp(commandName,"setManual")== 0)){
            if(nrhs != 3){
                printf("Invalid setManual syntax\n");
            }
            else{
                uint8_t Sys_ID = (uint8_t)mxGetScalar(prhs[1]);
                uint8_t autopilot_id = (uint8_t)mxGetScalar(prhs[2]);
                
                // Comp_ID; // all target components
                mavlink_message_t tx_msg;
                mavlink_command_long_t com;
                
                com.target_system    = Sys_ID;
                com.target_component = autopilot_id;
                com.command          = MAV_CMD_DO_SET_MODE;
                com.confirmation     = true;
                com.param1           = float(2);
                com.param2           = float(65536);

                // Encode
                mavlink_msg_command_long_encode(GCS.ID, GCS.Component_ID, &tx_msg, &com);
                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);

            }
        } // end set Manual command
        
        // seind Vision estimated data: position (meters), attitude (rad)
        else if ((strcmp(commandName,"sendVisionEst")== 0)){
            if(nrhs != 9){
                printf("Invalid sendVisionEst syntax\n");
            }
            else{
                uint8_t Sys_ID = (uint8_t)mxGetScalar(prhs[1]);
                uint8_t component_id = (uint8_t)mxGetScalar(prhs[2]);
                float x=(float)mxGetScalar(prhs[3]);
                float y=(float)mxGetScalar(prhs[4]);
                float z=(float)mxGetScalar(prhs[5]);
                float roll=(float)mxGetScalar(prhs[6]);
                float pitch=(float)mxGetScalar(prhs[7]);
                float yaw=(float)mxGetScalar(prhs[8]);
                                
                mavlink_message_t tx_msg;
                time_t ltime;
                uint64_t time_usec =time(&ltime);
                
                // Encode
                mavlink_msg_vision_position_estimate_pack(Sys_ID, component_id, &tx_msg,
						       time_usec, x, y, z, roll, pitch, yaw);              
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);

            }
        } // end sendVisionEst command    
        
        // seind motion capture system's estimated data: quaternion (w,x,y,z), XYZ9)meters
        else if ((strcmp(commandName,"mocap")== 0)){
            if(nrhs !=10){
                printf("Invalid mocap syntax\n");
            }
            else{
                mavlink_message_t tx_msg;
                mavlink_att_pos_mocap_t mocap;
                time_t ltime;
                uint64_t t_usec =time(&ltime);
                
                mocap.time_usec=t_usec;
                uint8_t Sys_ID = (uint8_t)mxGetScalar(prhs[1]);
                uint8_t component_id = (uint8_t)mxGetScalar(prhs[2]);
                mocap.q[0]=(float)mxGetScalar(prhs[3]);//q.w
                mocap.q[1]=(float)mxGetScalar(prhs[4]);//q.x
                mocap.q[2]=(float)mxGetScalar(prhs[5]);//q.y
                mocap.q[3]=(float)mxGetScalar(prhs[6]);//q.z
                mocap.x=(float)mxGetScalar(prhs[7]);//x
                mocap.y=(float)mxGetScalar(prhs[8]);//y
                mocap.z=(float)mxGetScalar(prhs[9]);//z
               
                // Encode
                mavlink_msg_att_pos_mocap_encode( Sys_ID, component_id, &tx_msg, &mocap);
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);

            }
        } // end mocap command
        
        // send position set points: NED xyz(m), yaw(rad), vx vy vz(m/s)
        else if ((strcmp(commandName,"setPoints")== 0)){
            if(nrhs != 11){
                printf("Invalid setPoints syntax\n");
            }
            else{
                mavlink_set_position_target_local_ned_t sp;
                mavlink_message_t tx_msg;
                
                sp.target_system    = (uint8_t)mxGetScalar(prhs[1]);
                sp.target_component = (uint8_t)mxGetScalar(prhs[2]);
                
                sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
                
                int mask = (int)mxGetScalar(prhs[10]);
                switch (mask)
                {
                    //set poistion only, xyz
                    case 1:
                        sp.type_mask = (uint16_t)MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
                        sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
                        
                        sp.x   = (float)mxGetScalar(prhs[3]);
                        sp.y   = (float)mxGetScalar(prhs[4]);
                        sp.z   = (float)mxGetScalar(prhs[5]);
                        break;
                    //position and yaw angle    
                    case 2:
                        sp.type_mask = (uint16_t)MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
                        sp.type_mask &= (uint16_t)MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;
                        sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
                        
                        sp.x   = (float)mxGetScalar(prhs[3]);
                        sp.y   = (float)mxGetScalar(prhs[4]);
                        sp.z   = (float)mxGetScalar(prhs[5]);
                        sp.yaw  = (float)mxGetScalar(prhs[6]);
                        break;
                    //yaw, velocity
                    case 3:
                        sp.type_mask = (uint16_t)MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;
                        sp.type_mask &= (uint16_t)MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;
                        sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
                        
                        sp.yaw  = (float)mxGetScalar(prhs[6]);
                        sp.vx   = (float)mxGetScalar(prhs[7]);
                        sp.vy   = (float)mxGetScalar(prhs[8]);
                        sp.vz   = (float)mxGetScalar(prhs[9]);
                        break;
                    // position NED and yaw angle
                    default :
                        sp.type_mask = (uint16_t)MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
                        sp.type_mask &= (uint16_t)MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;
                        sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
                        
                        sp.x   = (float)mxGetScalar(prhs[3]);
                        sp.y   = (float)mxGetScalar(prhs[4]);
                        sp.z   = (float)mxGetScalar(prhs[5]);
                        sp.yaw  = (float)mxGetScalar(prhs[6]);
                }

                time_t ltime;
                sp.time_boot_ms = (uint32_t)(time(&ltime)/1000);
                // Encode
                mavlink_msg_set_position_target_local_ned_encode(GCS.ID, GCS.Component_ID, &tx_msg, &sp);             
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);

            }
        } // end setPoint command
        
        // send actuator commands
        // mavlink('setActuators',sys_id, comp_id,actuator_group,  act1,..., act8);
        else if ((strcmp(commandName,"setActuators")== 0)){
            if(nrhs != 12){
                printf("Invalid setPoints syntax\n");
            }
            else {
            mavlink_set_actuator_control_target_t setActuators;
            mavlink_message_t tx_msg;
            
            time_t ltime;
            setActuators.time_usec = (uint64_t)(time(&ltime));
            
            
            setActuators.target_system=(uint8_t)mxGetScalar(prhs[1]);
            setActuators.target_component=(uint8_t)mxGetScalar(prhs[2]);
            setActuators.group_mlx=(uint8_t)mxGetScalar(prhs[3]);
            
            //get actuators controls values (duties: -1->1, 0: neutral)
            // 0->1 for motors with unidirction
            setActuators.controls[0]=(float)mxGetScalar(prhs[4]);
            setActuators.controls[1]=(float)mxGetScalar(prhs[5]);
            setActuators.controls[2]=(float)mxGetScalar(prhs[6]);
            setActuators.controls[3]=(float)mxGetScalar(prhs[7]);
            setActuators.controls[4]=(float)mxGetScalar(prhs[8]);
            setActuators.controls[5]=(float)mxGetScalar(prhs[9]);
            setActuators.controls[6]=(float)mxGetScalar(prhs[10]);
            setActuators.controls[7]=(float)mxGetScalar(prhs[11]);
            
            
            mavlink_msg_set_actuator_control_target_encode(GCS.ID, GCS.Component_ID, &tx_msg, &setActuators);
            buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
            ReturnBuffer(OutputBuffer,buf_len,plhs);
            }
        } //End: setActuators
        
        else if( (strcmp(commandName,"readallparams")== 0)){
            if(nrhs != 2){
                printf("Invalid Read all Parameter syntax\n");
            }
            else{
                mavlink_message_t tx_msg;
                
                uint8_t Sys_ID = (int)mxGetScalar(prhs[1]); 
                
                mavlink_msg_param_request_list_pack(GCS.ID, GCS.Component_ID, &tx_msg, 
                        Sys_ID, Comp_ID);
                                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
                
            }
        }
        
        // set a parameter
        else if(strcmp(commandName,"setParameter")== 0){
            if(nrhs!=5){
                printf("Invalid setParameter command syntax\n");
            }
            else{
                mavlink_message_t tx_msg;
                mavlink_param_set_t setparam;
                
                setparam.target_system= (uint8_t)mxGetScalar(prhs[1]);
                setparam.target_component =(uint8_t)mxGetScalar(prhs[2]);
                mxGetString(prhs[3], setparam.param_id, 16);
                setparam.param_value = (float)mxGetScalar(prhs[4]);
                setparam.param_type=MAV_PARAM_TYPE_REAL32;
                
                mavlink_msg_param_set_encode(GCS.ID, GCS.Component_ID, &tx_msg, &setparam);
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }
        }
        
        // set_attitude_target (quaternion)
        else if(strcmp(commandName,"setAttitudeTarget_q")== 0){
            if(nrhs!=7){
                printf("Invalid setAttitudeTarget_q command syntax\n");
            }
            else{
                mavlink_message_t tx_msg;
                mavlink_set_attitude_target_t setatt;
                
                setatt.target_system=(uint8_t)mxGetScalar(prhs[1]);
                setatt.target_component=(uint8_t)mxGetScalar(prhs[2]);
                
                setatt.q[0]=(float)mxGetScalar(prhs[3]);
                setatt.q[1]=(float)mxGetScalar(prhs[4]);
                setatt.q[2]=(float)mxGetScalar(prhs[5]);
                setatt.q[3]=(float)mxGetScalar(prhs[6]);
                
                setatt.type_mask=(1 << 6) | (7 << 0);
                
                time_t ltime;
                setatt.time_boot_ms = (uint32_t)(time(&ltime)/1000);
                
                mavlink_msg_set_attitude_target_encode(GCS.ID, GCS.Component_ID, &tx_msg, &setatt);
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }
        }// end of set_attitude_target (quaternion)
        
        // set_attitude_target (angular rates)
        else if(strcmp(commandName,"setAttitudeTarget_rates")== 0){
            if(nrhs!=6){
                printf("Invalid setAttitudeTarget_rates command syntax\n");
            }
            else{
                mavlink_message_t tx_msg;
                mavlink_set_attitude_target_t setatt;
                
                setatt.target_system=(uint8_t)mxGetScalar(prhs[1]);
                setatt.target_component=(uint8_t)mxGetScalar(prhs[2]);
                
                setatt.body_roll_rate=(float)mxGetScalar(prhs[3]);
                setatt.body_pitch_rate=(float)mxGetScalar(prhs[4]);
                setatt.body_yaw_rate=(float)mxGetScalar(prhs[5]);
                
                setatt.type_mask=(1 << 7) | (1 << 6);
                
                time_t ltime;
                setatt.time_boot_ms = (uint32_t)(time(&ltime)/1000);
                
                mavlink_msg_set_attitude_target_encode(GCS.ID, GCS.Component_ID, &tx_msg, &setatt);
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }
        }// end of set_attitude_target (angular rates)
        
        // set_attitude_target (thrust)
        else if(strcmp(commandName,"setAttitudeTarget_thrust")== 0){
            if(nrhs!=4){
                printf("Invalid setAttitudeTarget_thrust command syntax\n");
            }
            else{
                mavlink_message_t tx_msg;
                mavlink_set_attitude_target_t setatt;
                
                setatt.target_system=(uint8_t)mxGetScalar(prhs[1]);
                setatt.target_component=(uint8_t)mxGetScalar(prhs[2]);
                
                setatt.thrust=(float)mxGetScalar(prhs[3]);
                
                setatt.type_mask=(1 << 7) | (7 << 0);
                
                time_t ltime;
                setatt.time_boot_ms = (uint32_t)(time(&ltime)/1000);
                
                mavlink_msg_set_attitude_target_encode(GCS.ID, GCS.Component_ID, &tx_msg, &setatt);
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }
        }// end of set_attitude_target (thrust)
        
        // set_channel_override
        else if(strcmp(commandName,"setChannelOverride")== 0){
            if(nrhs!=11){
                printf("Invalid setChannelOverride command syntax\n");
            }
            else{
                mavlink_message_t tx_msg;
                mavlink_rc_channels_override_t setch;
                
                setch.target_system=(uint8_t)mxGetScalar(prhs[1]);
                setch.target_component=(uint8_t)mxGetScalar(prhs[2]);
                
                if ( (uint16_t)mxGetScalar(prhs[3]) == -1)
                    setch.chan1_raw=UINT16_MAX;
                else
                    setch.chan1_raw=(uint16_t)mxGetScalar(prhs[3]);
                
                if ( (uint16_t)mxGetScalar(prhs[4]) == -1)
                    setch.chan2_raw=UINT16_MAX;
                else
                    setch.chan2_raw=(uint16_t)mxGetScalar(prhs[4]);
                
                if ( (uint16_t)mxGetScalar(prhs[5]) == -1)
                    setch.chan3_raw=UINT16_MAX;
                else
                    setch.chan3_raw=(uint16_t)mxGetScalar(prhs[5]);
                
                if ( (uint16_t)mxGetScalar(prhs[6]) == -1)
                    setch.chan4_raw=UINT16_MAX;
                else
                    setch.chan4_raw=(uint16_t)mxGetScalar(prhs[6]);
                
                if ( (uint16_t)mxGetScalar(prhs[7]) == -1)
                    setch.chan5_raw=UINT16_MAX;
                else
                    setch.chan5_raw=(uint16_t)mxGetScalar(prhs[7]);
                
                if ( (uint16_t)mxGetScalar(prhs[8]) == -1)
                    setch.chan6_raw=UINT16_MAX;
                else
                    setch.chan6_raw=(uint16_t)mxGetScalar(prhs[8]);
                
                if ( (uint16_t)mxGetScalar(prhs[9]) == -1)
                    setch.chan7_raw=UINT16_MAX;
                else
                    setch.chan7_raw=(uint16_t)mxGetScalar(prhs[9]);
                
                if ( (uint16_t)mxGetScalar(prhs[10]) == -1)
                    setch.chan8_raw=UINT16_MAX;
                else
                    setch.chan8_raw=(uint16_t)mxGetScalar(prhs[10]);
                
                
                
                mavlink_msg_rc_channels_override_encode(GCS.ID, GCS.Component_ID, &tx_msg, &setch);
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
            }
        }// end of set_channel_override
  
        
        /*
        else if( (strcmp(commandName,"localNED")== 0)){
            if(nrhs != 7){
                printf("Invalid Read all Parameter syntax\n");
            }
            else{
                mavlink_message_t tx_msg;
                
                float x = (float)mxGetScalar(prhs[1]);
                float y = (float)mxGetScalar(prhs[2]);
                float z = (float)mxGetScalar(prhs[3]);
                float vx = (float)mxGetScalar(prhs[4]);
                float vy = (float)mxGetScalar(prhs[5]);
                float vz = (float)mxGetScalar(prhs[6]);
                
                mavlink_msg_local_position_ned_pack(GCS.ID, 0, &tx_msg,
						       123,  x,  y,  z,  vx,  vy,  vz);
                                
                buf_len = mavlink_msg_to_send_buffer(OutputBuffer, &tx_msg);
                
				ReturnBuffer(OutputBuffer,buf_len,plhs);
                
            }
        }
        */
        
   
        // read mavlink message
        else if(strcmp(commandName,"readmessage")== 0){
            if (nrhs != 4){
                printf("Invalid syntax\n");
            }

            else{
                //InputBuffer=mxGetChars(prhs[1]);
                InputBuffer = (unsigned char *)mxGetData(prhs[1]);
                BytesToRead= (int)mxGetScalar(prhs[2]);
                
                int NumOfVehicles= (int)mxGetScalar(prhs[3]);
                
                if (NumOfVehicles <1 || NumOfVehicles > Max_Num_Vehicles)
                {
                    printf("Number of Vehicles requested is greater than the maximum. \n");
                    return;
                }
                // These might go into the readmessage part, otherwise, the plhs part always causes an output
                //mwSize dims[2] = {1, Max_Num_Vehicles};
                mwSize dims[2] = {1, NumOfVehicles};
                //for (i = 0; i<Max_Num_Vehicles; i++){
                for (i = 0; i<NumOfVehicles; i++){
                    Vehicles[i] = Default_Vehicle;
                }

                /* Create a 1-by-n array of structs. */ 
                plhs[0] = mxCreateStructArray(2, dims, NUMBER_OF_FIELDS, Vehicle_field_names);
                
//debug    printf("In read\n");

                mavlink_status_t status;
                mavlink_message_t rx_msg;
                int chan = 0;
                bool msgReceived = false;
                bool Updated = false;
                int Attitude_Count = 0;
                int VFR_Count = 0;
                
                
                
                 for(i=0;i<BytesToRead;i++){
                     
                     if (mavlink_parse_char(chan, InputBuffer[i], &rx_msg, &status)){
                        msgReceived = true;
//debug                         printf(" message received with id: %d\n",  rx_msg.msgid);
// debug                         printf("Received message with ID %d, sequence: %d from component %d of system %d \n",
// debug                                rx_msg.msgid, rx_msg.seq, rx_msg.compid, rx_msg.sysid);

                        /**************************************************
                            * Identify which Vehicle's array to populate
                         **************************************************/
                        if (rx_msg.sysid < NumOfVehicles+1 )
                        {
                            Vehicles[rx_msg.sysid-1].ID = rx_msg.sysid;
    //debug                            printf("I received a message from a vehicle with ID = %d \n", rx_msg.sysid);

                            // IntrepretMessage is in header now
                            InterpretMessage(rx_msg, "Vehicles",rx_msg.sysid-1); // InterpretMessage is in Message_Decoder Header


                            //InterpretMessage(rx_msg, "printf",rx_msg.sysid-1)  ; // Just print information.
                        }
                        else if (rx_msg.sysid > NumOfVehicles+1 & rx_msg.sysid < 100)
                        {
                            printf("Received Vehicle ID that is greater than Maximum number of allowed vehicles. \n");
                            //return;
                        }
                       //break; // if you don't want to go through all bytes, once received a one vaid mavlink parse char!
                    } // end if mavlink_parse
                 
                 } // end  going thru all read bytes
               
               
       // After the for loop has parsed all read bytes, send one return back
                if (msgReceived) {
                    msgReceived = false; // Reset


/**************************************************************************
*This section is based mxcreatestructarray.c
* Each field needs its own pointer.  They can be re-used as the for
* loop cycles as long as they are created inside the for loop each time
* Re-using fields, or not re-creating them, will crash MATLAB.
*
*************************************************************************/

                    //for (i=0; i<NUMBER_OF_VEHICLES; i++) {
                    for (i=0; i<NumOfVehicles; i++) {
                    

                        mxArray *ptr_updated;
                        mxArray *ptr_ID;
                        mxArray *ptr_Component_ID;
                        mxArray *ptr_type;
                        mxArray *ptr_AP_type;
                        mxArray *ptr_base_mode;
                        mxArray *ptr_custom_mode;
                        mxArray *ptr_roll;
                        mxArray *ptr_pitch;
                        mxArray *ptr_yaw;
                        mxArray *ptr_lat;
                        mxArray *ptr_lon;
                        mxArray *ptr_alt;
                        mxArray *ptr_hdg;
                        mxArray *ptr_action;
                        mxArray *ptr_nav_bearing;
                        mxArray *ptr_target_bearing;
                        mxArray *ptr_target_distance;
                        mxArray *ptr_nav_roll;
                        mxArray *ptr_nav_pitch;
                        mxArray *ptr_nav_yaw;
                        mxArray *ptr_voltage_battery;
                        mxArray *ptr_sys_status;
                        mxArray *ptr_localNEDx;
                        mxArray *ptr_localNEDy;
                        mxArray *ptr_localNEDz;
                        mxArray *ptr_abs_pressure;
                        mxArray *ptr_diff_pressure;
                        mxArray *ptr_att_tstamp;
                        mxArray *ptr_gps_tstamp;
                        mxArray *ptr_highers_tstamp;
                        mxArray *ptr_localned_tstamp;
                        mxArray *ptr_scaled_abs_press;
                        mxArray *ptr_scaled_diff_press;
                        mxArray *ptr_scaled_press_tms;
                        mxArray *ptr_xacc;
                        mxArray *ptr_yacc;
                        mxArray *ptr_zacc;
                        mxArray *ptr_xgyro;
                        mxArray *ptr_ygyro;
                        mxArray *ptr_zgyro;
                        mxArray *ptr_rollspeed;
                        mxArray *ptr_pitchspeed;
                        mxArray *ptr_yawspeed;
                        mxArray *ptr_vx;
                        mxArray *ptr_vy;
                        mxArray *ptr_vz;
                        mxArray *ptr_q0_t;
                        mxArray *ptr_q1_t;
                        mxArray *ptr_q2_t;
                        mxArray *ptr_q3_t;
                        mxArray *ptr_roll_rate_t;
                        mxArray *ptr_pitch_rate_t;
                        mxArray *ptr_yaw_rate_t;
                        mxArray *ptr_thrust_t;
                        mxArray *ptr_att_ms_t;

                        ptr_updated = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_ID = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_Component_ID = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_type = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_AP_type = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_base_mode = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_custom_mode = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_roll = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_pitch = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_yaw = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_lat = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_lon = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_alt = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_hdg = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_action = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_nav_bearing = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_target_bearing = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_target_distance = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_nav_roll = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_nav_pitch = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_nav_yaw = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_voltage_battery = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_sys_status = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_localNEDx = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_localNEDy = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_localNEDz = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_abs_pressure = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_diff_pressure = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_att_tstamp = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_gps_tstamp = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_highers_tstamp = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_localned_tstamp = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_scaled_abs_press = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_scaled_diff_press = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_scaled_press_tms= mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_xacc= mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_yacc= mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_zacc= mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_xgyro= mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_ygyro= mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_zgyro= mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_rollspeed = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_pitchspeed = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_yawspeed = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_vx = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_vy = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_vz = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_q0_t = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_q1_t = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_q2_t = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_q3_t = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_roll_rate_t = mxCreateDoubleMatrix(1,1,mxREAL); 
                        ptr_pitch_rate_t = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_yaw_rate_t = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_thrust_t = mxCreateDoubleMatrix(1,1,mxREAL);
                        ptr_att_ms_t = mxCreateDoubleMatrix(1,1,mxREAL);
                                
                        /*Could also use mxSetField and "field_name" instead of number
                         *ByNumber is supposed to be faster.  This is the 'dumb' method, because
                         *the program depends on the order not changing.  Example program
                         *mccreatestructarray.c used name_field = mxGetFieldNumber(plhs[0],"name")
                         *to pull the number for each field.  I've opted not to, but it means
                         * if the order changes, it won't work. 
                         */
                        *mxGetPr(ptr_updated) = Vehicles[i].updated;
                        mxSetFieldByNumber(plhs[0],i,0,ptr_updated);

                        *mxGetPr(ptr_ID) = Vehicles[i].ID;
                        mxSetFieldByNumber(plhs[0],i,1,ptr_ID);

                        *mxGetPr(ptr_Component_ID) = Vehicles[i].Component_ID;
                        mxSetFieldByNumber(plhs[0],i,2,ptr_Component_ID);

                        *mxGetPr(ptr_type) = Vehicles[i].type;
                        mxSetFieldByNumber(plhs[0],i,3,ptr_type);

                        *mxGetPr(ptr_AP_type) = Vehicles[i].AP_type;
                        mxSetFieldByNumber(plhs[0],i,4,ptr_AP_type);

                        *mxGetPr(ptr_base_mode) = Vehicles[i].base_mode;
                        mxSetFieldByNumber(plhs[0],i,5,ptr_base_mode);

                        *mxGetPr(ptr_custom_mode) = Vehicles[i].custom_mode;
                        mxSetFieldByNumber(plhs[0],i,6,ptr_custom_mode);

                        *mxGetPr(ptr_roll) = Vehicles[i].roll;
                        mxSetFieldByNumber(plhs[0],i,7,ptr_roll);

                        *mxGetPr(ptr_pitch) = Vehicles[i].pitch;
                        mxSetFieldByNumber(plhs[0],i,8,ptr_pitch);

                        *mxGetPr(ptr_yaw) = Vehicles[i].yaw;
                        mxSetFieldByNumber(plhs[0],i,9,ptr_yaw);

                        *mxGetPr(ptr_lat) = Vehicles[i].lat;
                        mxSetFieldByNumber(plhs[0],i,10,ptr_lat);

                        *mxGetPr(ptr_lon) = Vehicles[i].lon;
                        mxSetFieldByNumber(plhs[0],i,11,ptr_lon);

                        *mxGetPr(ptr_alt) = Vehicles[i].alt;
                        mxSetFieldByNumber(plhs[0],i,12,ptr_alt);

                        *mxGetPr(ptr_hdg) = Vehicles[i].hdg;
                        mxSetFieldByNumber(plhs[0],i,13,ptr_hdg);

                        *mxGetPr(ptr_action) = Vehicles[i].action;
                        mxSetFieldByNumber(plhs[0],i,14,ptr_action);

                        *mxGetPr(ptr_nav_bearing) = Vehicles[i].nav_bearing;
                        mxSetFieldByNumber(plhs[0],i,15,ptr_nav_bearing);

                        *mxGetPr(ptr_target_bearing) = Vehicles[i].target_bearing;
                        mxSetFieldByNumber(plhs[0],i,16,ptr_target_bearing);

                        *mxGetPr(ptr_target_distance) = Vehicles[i].target_distance;
                        mxSetFieldByNumber(plhs[0],i,17,ptr_target_distance);

                        *mxGetPr(ptr_nav_roll) = Vehicles[i].nav_roll;
                        mxSetFieldByNumber(plhs[0],i,18,ptr_nav_roll);

                        *mxGetPr(ptr_nav_pitch) = Vehicles[i].nav_pitch;
                        mxSetFieldByNumber(plhs[0],i,19,ptr_nav_pitch);

                        *mxGetPr(ptr_nav_yaw) = Vehicles[i].nav_yaw;
                        mxSetFieldByNumber(plhs[0],i,20,ptr_nav_yaw);

                        *mxGetPr(ptr_voltage_battery) = Vehicles[i].voltage_battery;
                        mxSetFieldByNumber(plhs[0],i,21,ptr_voltage_battery);

                        *mxGetPr(ptr_sys_status) = Vehicles[i].sys_status;
                        mxSetFieldByNumber(plhs[0],i,22,ptr_sys_status);
                        
                        *mxGetPr(ptr_localNEDx) = Vehicles[i].localNEDx;
                        mxSetFieldByNumber(plhs[0],i,23,ptr_localNEDx);
                        
                        *mxGetPr(ptr_localNEDy) = Vehicles[i].localNEDy;
                        mxSetFieldByNumber(plhs[0],i,24,ptr_localNEDy);
                        
                        *mxGetPr(ptr_localNEDz) = Vehicles[i].localNEDz;
                        mxSetFieldByNumber(plhs[0],i,25,ptr_localNEDz);
                        
                        *mxGetPr(ptr_abs_pressure) = Vehicles[i].abs_pressure;
                        mxSetFieldByNumber(plhs[0],i,26,ptr_abs_pressure);
                        
                        *mxGetPr(ptr_diff_pressure) = Vehicles[i].diff_pressure;
                        mxSetFieldByNumber(plhs[0],i,27,ptr_diff_pressure);
                        
                        *mxGetPr(ptr_att_tstamp) = Vehicles[i].att_tstamp;
                        mxSetFieldByNumber(plhs[0],i,28,ptr_att_tstamp);
                        
                        *mxGetPr(ptr_gps_tstamp) = Vehicles[i].gps_tstamp;
                        mxSetFieldByNumber(plhs[0],i,29,ptr_gps_tstamp);
                        
                        *mxGetPr(ptr_highers_tstamp) = Vehicles[i].highers_tstamp;
                        mxSetFieldByNumber(plhs[0],i,30,ptr_highers_tstamp);
                        
                        *mxGetPr(ptr_localned_tstamp) = Vehicles[i].localned_tstamp;
                        mxSetFieldByNumber(plhs[0],i,31,ptr_localned_tstamp);
                        
                        *mxGetPr(ptr_scaled_abs_press) = Vehicles[i].scaled_abs_press;
                        mxSetFieldByNumber(plhs[0],i,32,ptr_scaled_abs_press);
                        
                        *mxGetPr(ptr_scaled_diff_press) = Vehicles[i].scaled_diff_press;
                        mxSetFieldByNumber(plhs[0],i,33,ptr_scaled_diff_press);
                        
                        *mxGetPr(ptr_scaled_press_tms) = Vehicles[i].scaled_press_tms;
                        mxSetFieldByNumber(plhs[0],i,34,ptr_scaled_press_tms);
                        
                        *mxGetPr(ptr_xacc) = Vehicles[i].xacc;
                        mxSetFieldByNumber(plhs[0],i,35,ptr_xacc);
                        
                        *mxGetPr(ptr_yacc) = Vehicles[i].yacc;
                        mxSetFieldByNumber(plhs[0],i,36,ptr_yacc);
                        
                        *mxGetPr(ptr_zacc) = Vehicles[i].zacc;
                        mxSetFieldByNumber(plhs[0],i,37,ptr_zacc);
                        
                        *mxGetPr(ptr_xgyro) = Vehicles[i].xgyro;
                        mxSetFieldByNumber(plhs[0],i,38,ptr_xgyro);
                        
                        *mxGetPr(ptr_ygyro) = Vehicles[i].ygyro;
                        mxSetFieldByNumber(plhs[0],i,39,ptr_ygyro);
                        
                        *mxGetPr(ptr_zgyro) = Vehicles[i].zgyro;
                        mxSetFieldByNumber(plhs[0],i,40,ptr_zgyro);
                        
                        *mxGetPr(ptr_rollspeed) = Vehicles[i].rollspeed;
                        mxSetFieldByNumber(plhs[0],i,41,ptr_rollspeed);
                        
                        *mxGetPr(ptr_pitchspeed) = Vehicles[i].pitchspeed;
                        mxSetFieldByNumber(plhs[0],i,42,ptr_pitchspeed);
                        
                        *mxGetPr(ptr_yawspeed) = Vehicles[i].yawspeed;
                        mxSetFieldByNumber(plhs[0],i,43,ptr_yawspeed);
                        
                        *mxGetPr(ptr_vx) = Vehicles[i].vx;
                        mxSetFieldByNumber(plhs[0],i,44,ptr_vx);
                        
                        *mxGetPr(ptr_vy) = Vehicles[i].vy;
                        mxSetFieldByNumber(plhs[0],i,45,ptr_vy);
                        
                        *mxGetPr(ptr_vz) = Vehicles[i].vz;
                        mxSetFieldByNumber(plhs[0],i,46,ptr_vz);
                        
                        *mxGetPr(ptr_q0_t) = Vehicles[i].q0_t;
                        mxSetFieldByNumber(plhs[0],i,47,ptr_q0_t);
                        
                        *mxGetPr(ptr_q1_t) = Vehicles[i].q1_t;
                        mxSetFieldByNumber(plhs[0],i,48,ptr_q1_t);
                        
                        *mxGetPr(ptr_q2_t) = Vehicles[i].q2_t;
                        mxSetFieldByNumber(plhs[0],i,49,ptr_q2_t);
                        
                        *mxGetPr(ptr_q3_t) = Vehicles[i].q3_t;
                        mxSetFieldByNumber(plhs[0],i,50,ptr_q3_t);
                        
                        
                        *mxGetPr(ptr_roll_rate_t) = Vehicles[i].roll_rate_t;
                        mxSetFieldByNumber(plhs[0],i,51,ptr_roll_rate_t);
                        
                        *mxGetPr(ptr_pitch_rate_t) = Vehicles[i].pitch_rate_t;
                        mxSetFieldByNumber(plhs[0],i,52,ptr_pitch_rate_t);
                        
                        *mxGetPr(ptr_yaw_rate_t) = Vehicles[i].yaw_rate_t;
                        mxSetFieldByNumber(plhs[0],i,53,ptr_yaw_rate_t);

                        
                        *mxGetPr(ptr_thrust_t) = Vehicles[i].thrust_t;
                        mxSetFieldByNumber(plhs[0],i,54,ptr_thrust_t);
                        
                        *mxGetPr(ptr_att_ms_t) = Vehicles[i].att_target_time_boot_ms;
                        mxSetFieldByNumber(plhs[0],i,55,ptr_att_ms_t);

                    }


                } // end if msgReceived
               
//debug               printf(" \n " );
            }
        }
        else{
            printf("invalid syntax\n");
        }
        
    } // end if(nrhs >=1)
} // end mexFunction

  