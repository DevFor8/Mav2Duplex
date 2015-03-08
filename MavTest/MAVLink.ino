/*
	 Mav2DupEx Version 0.1
	 2014, by DevFor8.com, info@devfor8.com
	 
	 part of code is based on ArduCAM OSD
*/

#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;
static uint8_t crlf_count = 0;

static int packet_drops = 0;
static int parse_error = 0;

boolean getBit(byte Reg, byte whichBit) {
    boolean State;
    State = Reg & (1 << whichBit);
    return State;
}  

void request_mavlink_rates()
{
    const int  maxStreams = 6;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1, 
        MAV_DATA_STREAM_EXTRA2};
    const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02};
    for (int i=0; i < maxStreams; i++) {
        mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
            apm_mav_system, apm_mav_component,
            MAVStreams[i], MAVRates[i], 1);
    }
}

void read_mavlink(int maxframes){
    mavlink_message_t msg; 
    mavlink_status_t status;
    int current_frames = 0;

    //grabing data 
    while(Serial.available() > 0) { 
        uint8_t c = Serial.read();
        //Serial.print(c);        Serial.print(" ");

        //trying to grab msg  
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            mavlink_active = 1;
            current_frames++; //this is frame
            //handle msg
            switch(msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                {
                  Serial.println("beat");
                    mavbeat = 1;
                    apm_mav_system    = msg.sysid;
                    apm_mav_component = msg.compid;
                    apm_mav_type      = mavlink_msg_heartbeat_get_type(&msg);            
                 //   osd_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
                    osd_mode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&msg);
                    //Mode (arducoper armed/disarmed)
                    base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
                    if(getBit(base_mode,7)) motor_armed = 1;
                    else motor_armed = 0;

                    lastMAVBeat = millis();
                    if(waitingMAVBeats == 1){
                        enable_mav_request = 1;
                    }
                }
                break;
            case MAVLINK_MSG_ID_STATUSTEXT:
                {
                  Serial.println("stext");
                  char text[MAVLINK_MSG_ID_STATUSTEXT_LEN] ;
                  mavlink_msg_statustext_get_text(&msg,&text[0]);
                  strncpy((char*)LastMessage,text,LCDMaxPos);
                  break;
                }
            case MAVLINK_MSG_ID_SYS_STATUS:
                {
                  Serial.println("status");
                    osd_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
                    osd_curr_A = (mavlink_msg_sys_status_get_current_battery(&msg) / 100.0f); //Battery current, in 10*milliamperes (1 = 10 milliampere)         10
                    osd_battery_remaining_A = mavlink_msg_sys_status_get_battery_remaining(&msg); //Remaining battery energy: (0%: 0, 100%: 100)
                }
                break;

            case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                  Serial.println("gps:");
                    osd_lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
                    osd_lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
                    osd_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
                    osd_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
                  //serial.println(osd_fix_type);

                }
                break; 
            case MAVLINK_MSG_ID_VFR_HUD:
                {
                  Serial.println("hud:");
                    //osd_airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
                    //osd_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
                    osd_heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
                  //Serial.println(osd_heading);  
                    //osd_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
                    //if(osd_throttle > 100 && osd_throttle < 150) osd_throttle = 100;//Temporary fix for ArduPlane 2.28
                    //if(osd_throttle < 0 || osd_throttle > 150) osd_throttle = 0;//Temporary fix for ArduPlane 2.28
                    osd_baro_alt = mavlink_msg_vfr_hud_get_alt(&msg);
                    //osd_climb = mavlink_msg_vfr_hud_get_climb(&msg);
                }
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                {
                  Serial.println("atti");
                    osd_pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
                    osd_roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
                    osd_yaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
                  //Serial.print(osd_pitch);  Serial.print("  / ");Serial.print(osd_roll);  Serial.print("  / ");Serial.println(osd_yaw); 
                }
                break;

            default:
                {
                    
                    Serial.print("unk:");Serial.println(msg.msgid);
                }
                break;
            }
        delayMicroseconds(200); //wait just between frames, not eaach character
      }
        
        //next one
        if (current_frames>maxframes)
          break; //we need time for Jeti
    }
    // Update global packet drops counter
    packet_drops += status.packet_rx_drop_count;
    parse_error += status.parse_error;

}
