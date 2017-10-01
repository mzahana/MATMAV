

/* This function takes received messages as inputs and turns them into 
 * information. Made to multifunction, printf read data for running through
 * logs or update global Vehicles array for use in program
 *
 *  rx_msg is the received message to be decoded
 *  output_type is the string "Vehicles" or "printf" and will send the decoded data to either
 *      the Vehicles array of structs or print it to the screen
 *  ID_index is the index in Vehicles array of the vehicle that sent the message
 *      
 *
 *  **************IMPLEMENT ID_index********************************
*/
void InterpretMessage(mavlink_message_t rx_msg,char *output_type, int ID_index){
    
//debug    printf("This is for Vehicle: %d\n",ID_index);
//debug    printf("=====================================\n");
            if((strcmp(output_type,"printf")== 0)||(strcmp(output_type,"Vehicles")==0)){
             //This is good
            }else {
            // Handle error here
            }
    
           // get the component id
            Vehicles[ID_index].Component_ID=rx_msg.compid;
    switch (rx_msg.msgid) 
    {
        
        case MAVLINK_MSG_ID_HEARTBEAT :
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&rx_msg, &heartbeat);
            
            if(strcmp(output_type,"printf")== 0){
                // Do I care about printing heartbeats?
                printf("Heartbeat received, base mode = %u, custom mode = %u, Autopilot= %u\n",
                        heartbeat.base_mode, heartbeat.custom_mode, heartbeat.autopilot);
            }else if(strcmp(output_type,"Vehicles")== 0){
             Vehicles[ID_index].updated=1;
             Vehicles[ID_index].base_mode = heartbeat.base_mode;
             Vehicles[ID_index].custom_mode = heartbeat.custom_mode;
             Vehicles[ID_index].AP_type = heartbeat.autopilot;
             Vehicles[ID_index].sys_status = heartbeat.system_status;
            }
            
            
            break;
            
                    
        case MAVLINK_MSG_ID_SET_MODE :
            // SHOULDN"T USE THIS FOR INCOMING!  This changes the mode
            mavlink_set_mode_t set_mode;
            mavlink_msg_set_mode_decode(&rx_msg, &set_mode);
            
            if(strcmp(output_type,"printf")== 0){
                //printf("Set mode received, base mode = %u, custom mode = %u\n",
                //        set_mode.base_mode, set_mode.custom_mode);
            }else if(strcmp(output_type,"Vehicles")== 0){
             
            }
            
            
            break;
        case MAVLINK_MSG_ID_SYS_STATUS :
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&rx_msg, &sys_status);
            
            if(strcmp(output_type,"printf")== 0){
                printf("System Status received, Sys ID = %u, voltage = %u\n",
                        rx_msg.sysid, sys_status.voltage_battery);
            }else if(strcmp(output_type,"Vehicles")== 0){
                
             Vehicles[ID_index].voltage_battery = sys_status.voltage_battery;
            }
            
            
            break;
        case MAVLINK_MSG_ID_HIGHRES_IMU :
            mavlink_highres_imu_t highers_imu;
            mavlink_msg_highres_imu_decode(&rx_msg, &highers_imu);
            
            if(strcmp(output_type,"printf")== 0){
                //printf("Received highers imu.");
            }else if(strcmp(output_type,"Vehicles")== 0){
             
             Vehicles[ID_index].xacc = highers_imu.xacc;
             Vehicles[ID_index].yacc = highers_imu.yacc;
             Vehicles[ID_index].zacc = highers_imu.zacc;
             Vehicles[ID_index].xgyro = highers_imu.xgyro;
             Vehicles[ID_index].ygyro = highers_imu.ygyro;
             Vehicles[ID_index].zgyro = highers_imu.zgyro;
             Vehicles[ID_index].abs_pressure = highers_imu.abs_pressure;
             Vehicles[ID_index].diff_pressure = highers_imu.diff_pressure;
             Vehicles[ID_index].highers_tstamp=highers_imu.time_usec;
            }
            break;
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
            mavlink_scaled_pressure_t scaled_pres;
            mavlink_msg_scaled_pressure_decode(&rx_msg, &scaled_pres);
            
            if(strcmp(output_type,"printf")== 0){
                //printf("Received Scaled presuure values.");
            }else if(strcmp(output_type,"Vehicles")== 0){
                
             Vehicles[ID_index].scaled_abs_press = scaled_pres.press_abs;
             Vehicles[ID_index].scaled_diff_press = scaled_pres.press_diff;
             Vehicles[ID_index].scaled_press_tms=scaled_pres.time_boot_ms;
            }
            break;
            
                    
        case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT :
            // Note: This output is in degrees.  set_roll_pitch... input is in radians.
            mavlink_nav_controller_output_t nav_controller;
            mavlink_msg_nav_controller_output_decode(&rx_msg, &nav_controller);
            
            if(strcmp(output_type,"printf")== 0){
               // printf("Nav Output received, roll = %f, pitch = %f, bearing = %d \n target bearing = %d, distance = %u \n",
              //          nav_controller.nav_roll, nav_controller.nav_pitch, nav_controller.nav_bearing, nav_controller.target_bearing, nav_controller.wp_dist);
            }else if(strcmp(output_type,"Vehicles")== 0){

             Vehicles[ID_index].nav_bearing = nav_controller.nav_bearing;
             Vehicles[ID_index].target_bearing = nav_controller.target_bearing;
             Vehicles[ID_index].target_distance = nav_controller.wp_dist;
            }
            
            
            break;
            
                                
        case MAVLINK_MSG_ID_MISSION_CURRENT :
            mavlink_mission_current_t mission_current;
            mavlink_msg_mission_current_decode(&rx_msg, &mission_current);
            
            if(strcmp(output_type,"printf")== 0){
               // printf("Current Mission = %u \n",
                 //       mission_current.seq);
            }else if(strcmp(output_type,"Vehicles")== 0){
             
            }
            
            
            break;
            
                                
        case MAVLINK_MSG_ID_GPS_RAW_INT :
            mavlink_gps_raw_int_t gps_raw;
            mavlink_msg_gps_raw_int_decode(&rx_msg, &gps_raw);
            
            if(strcmp(output_type,"printf")== 0){
                // Do I care?
            }else if(strcmp(output_type,"Vehicles")== 0){
               // Do I care? Yes
                
                Vehicles[ID_index].lat = gps_raw.lat;
                Vehicles[ID_index].lon = gps_raw.lon;
                Vehicles[ID_index].alt = gps_raw.alt;
                //Vehicles[ID_index].hdg = gps_raw.hdg;
                Vehicles[ID_index].gps_tstamp = gps_raw.time_usec;
                
            }
            
            
            break;
            
        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED :
            mavlink_mission_item_reached_t item_reached;
            mavlink_msg_mission_item_reached_decode(&rx_msg, &item_reached);
            
            if(strcmp(output_type,"printf")== 0){
                //printf("Mission Item Reached");
            }else if(strcmp(output_type,"Vehicles")== 0){
//             Vehicles[ID_index].action = 2;  // I'm not using this, and APM does not transmit for guided mode
//             Vehicles[ID_index].updated = 1;
            }
            
            
            break;
            
        case MAVLINK_MSG_ID_PING :
            mavlink_ping_t ping;
            mavlink_msg_ping_decode(&rx_msg, &ping);
            if(strcmp(output_type,"printf")== 0){
                printf("\nPing reply received, time = %d \n", ping.time_usec);
            }else if(strcmp(output_type,"Vehicles")== 0){
            // put data in Vehicles array if desired
            }
            
            break;
            
            
        case MAVLINK_MSG_ID_MISSION_ACK :
            mavlink_mission_ack_t msn_ack;
            mavlink_msg_mission_ack_decode(&rx_msg, &msn_ack);
            if(strcmp(output_type,"printf")== 0){
                //printf("\nMission Acknowledgement received, type = %d \n", msn_ack.type);
            }else if(strcmp(output_type,"Vehicles")== 0){
            // put data in Vehicles array if desired
            }
            
            break;
            
            
        case MAVLINK_MSG_ID_COMMAND_ACK :
            mavlink_command_ack_t cmd_ack;
            mavlink_msg_command_ack_decode(&rx_msg, &cmd_ack);
            if(strcmp(output_type,"printf")== 0){
                //printf("\nCommand Acknowledgement received, command = %d, result = %d \n", cmd_ack.command, cmd_ack.result);
            }
            
            break;
            
            
        case MAVLINK_MSG_ID_STATUSTEXT :
            mavlink_statustext_t statustext;
            mavlink_msg_statustext_decode(&rx_msg, &statustext);
            if(strcmp(output_type,"printf")== 0){
                printf("\nStatus Text received, text = %s \n", statustext.text);
            }else if(strcmp(output_type,"Vehicles")== 0){
            // put data in Vehicles array if desired
            }
            
            break;
            
            
        case MAVLINK_MSG_ID_MISSION_ITEM :
            mavlink_mission_item_t msn_item;
            mavlink_msg_mission_item_decode(&rx_msg, &msn_item);
            if(strcmp(output_type,"printf")== 0){
                //printf("\n Mission Item received, param 1 = %f param 2 = %f param 3 = %f param 4 = %f \n x=%f, y=%f, z=%f \n",
               //         msn_item.param1,msn_item.param2,msn_item.param3,msn_item.param4, msn_item.x,msn_item.y,msn_item.z);
              //  printf(" sequence = %d, command = %d, frame = %d, current = %d, autocontinue = %d \n",
              //          msn_item.seq,msn_item.command,msn_item.frame,msn_item.current,msn_item.autocontinue);
            }else if(strcmp(output_type,"Vehicles")== 0){
            // put data in Vehicles array if desired
            }
            
            break;
            
            // This is fused position
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT :
            mavlink_global_position_int_t position;
            mavlink_msg_global_position_int_decode(&rx_msg, &position);
            if(strcmp(output_type,"printf")== 0){
                printf("\n Position received, Lat=%d, Long=%d \n MSL = %d, AGL = %d \n",
                        position.lat, position.lon, position.alt, position.relative_alt);
            }else if(strcmp(output_type,"Vehicles")== 0){
                /*
                Vehicles[ID_index].lat = position.lat;
                Vehicles[ID_index].lon = position.lon;
                Vehicles[ID_index].alt = position.alt;
                Vehicles[ID_index].hdg = position.hdg;
                */
            }
            
            break;
            
            
        case MAVLINK_MSG_ID_MISSION_COUNT :
            mavlink_mission_count_t mission_count;
            mavlink_msg_mission_count_decode(&rx_msg, &mission_count);
            if(strcmp(output_type,"printf")== 0){
                //printf("\n Mission Count received, count = %d \n", mission_count.count);
            }else if(strcmp(output_type,"Vehicles")== 0){
            // put data in Vehicles array if desired
            }
            
            break;
            
        case MAVLINK_MSG_ID_ATTITUDE :
//                                     classIDflags = mxCalloc(nfields, sizeof(mxClassID));
            
            
// Not doing attitude count, need to keep track of recency of updates somewhow
//                                     attitudeReceived = TRUE;
//                                     Attitude_Count++;
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&rx_msg, &attitude);
            
            
            if(strcmp(output_type,"printf")== 0){
             //   printf("\n ");
                printf("found attitude message\n");
                printf("roll: %f\n",attitude.roll);
                printf("pitch: %f\n",attitude.pitch);
                printf("yaw: %f\n", attitude.yaw);
            }else if(strcmp(output_type,"Vehicles")== 0){
            // put data in Vehicles array if desired
            
                Vehicles[ID_index].roll = attitude.roll;
                Vehicles[ID_index].pitch = attitude.pitch;
                Vehicles[ID_index].yaw = attitude.yaw;
                Vehicles[ID_index].rollspeed=attitude.rollspeed;
                Vehicles[ID_index].pitchspeed=attitude.pitchspeed;
                Vehicles[ID_index].yawspeed=attitude.yawspeed;
                Vehicles[ID_index].att_tstamp = attitude.time_boot_ms;

            
            }
//                                     if (tens_count >= 10){
// Not sure what this is doing      memcpy(messageInfo, Info, sizeof(mavlink_message_info_t)*256);
                        
            break;
            
        case MAVLINK_MSG_ID_VFR_HUD :
            
            mavlink_vfr_hud_t vfr_hud;
            mavlink_msg_vfr_hud_decode(&rx_msg, &vfr_hud);
            if(strcmp(output_type,"printf")== 0){
             //   printf("\n ");
            }else if(strcmp(output_type,"Vehicles")== 0){
            // put data in Vehicles array if desired
                
            }
            
//                                     }
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                    
            mavlink_local_position_ned_t localNED;
            mavlink_msg_local_position_ned_decode(&rx_msg, &localNED);
            if(strcmp(output_type,"printf")== 0){
             //   printf("\n ");
            }else if(strcmp(output_type,"Vehicles")== 0){
            // put data in Vehicles array if desired
            Vehicles[ID_index].localNEDx = localNED.x; 
            Vehicles[ID_index].localNEDy = localNED.y;
            Vehicles[ID_index].localNEDz = localNED.z;
            Vehicles[ID_index].vx = localNED.vx;
            Vehicles[ID_index].vy = localNED.vy;
            Vehicles[ID_index].vz = localNED.vz;
            Vehicles[ID_index].localned_tstamp = localNED.time_boot_ms;
            }

            break;
        case MAVLINK_MSG_ID_ATTITUDE_TARGET:
            mavlink_attitude_target_t att_t;
            mavlink_msg_attitude_target_decode(&rx_msg, &att_t);
            if(strcmp(output_type,"printf")== 0){
             //   printf("\n ");
            }else if(strcmp(output_type,"Vehicles")== 0){
            // put data in Vehicles array if desired
            Vehicles[ID_index].q0_t = att_t.q[0];
            Vehicles[ID_index].q1_t = att_t.q[1];
            Vehicles[ID_index].q2_t = att_t.q[2];
            Vehicles[ID_index].q3_t = att_t.q[3];
            Vehicles[ID_index].roll_rate_t = att_t.body_roll_rate;
            Vehicles[ID_index].pitch_rate_t = att_t.body_pitch_rate;
            Vehicles[ID_index].yaw_rate_t = att_t.body_yaw_rate;
            Vehicles[ID_index].thrust_t = att_t.thrust;
            Vehicles[ID_index].att_target_time_boot_ms = att_t.time_boot_ms;
            }
            break;
            
        default :
            /*
            if(strcmp(output_type,"printf")== 0){
                printf("Nrr?"); // I don't know what this message means
            }
             */
            break;
             
    }
    
//debug    printf("==========End of interpret===========\n");
}


