#include <MAVLink.h>
#include <SPI.h>  
#include "RF24.h"

RF24 radio (4, 5);
byte addresses[][6] = {"0"};

uint16_t rc[16];

uint32_t prevMillis = 0;

typedef struct __telem_t {
  uint8_t type;
  float ground_speed;
  float pitch;
  float yaw;
  float roll;
  int32_t lat;
  int32_t lon;
  int32_t alt;
  // int32_t relative_alt;
} Telem;

Telem tel;

void setup()
{
  setCpuFrequencyMhz(240);
  delay(1000);

  Serial.begin(256000);
  Serial2.begin(256000);

  radio.begin();  
  radio.setPayloadSize(255);
  radio.setChannel(115); 
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, addresses[0]);
  radio.startListening();
}


uint32_t failsafeTimeout = 0;
uint32_t sda = 0;
void loop() {
  receive_radio();

  // for (int i=0; i < 16; i++) {
  //     Serial.print(rc[i]);
  //     Serial.print(" ");
  //   }
  //   Serial.println("");

  if (failsafeTimeout > 1000) {
    rc[0] = 0;
    rc[1] = 0;
    rc[2] = 0;
    rc[3] = 0;
    rc[4] = 0;
    rc[5] = 0;
    rc[6] = 0;
    rc[7] = 0;
    rc[8] = 0;
    rc[9] = 0;
    rc[10] = 0;
    rc[11] = 0;
    rc[12] = 0;
    rc[13] = 0;
    rc[14] = 0;
    rc[15] = 0;
    sendCommand();
  }
  

  receive_telemetry_from_ardupilot();
  if (millis() - prevMillis > 2000) {
    sendHeartbeat();
    request_data_from_ardupilot();
    prevMillis = millis();
  }
  
}

void receive_radio() {
  if ( radio.available()) {
    radio.read( (uint8_t *)rc, 32 ); 
    sendCommand();
  } else {
    failsafeTimeout = millis() - sda;
    sda = millis();
  }
}

void send_telemetry_to_ground() {
  radio.stopListening();
  radio.openWritingPipe(addresses[0]);
  radio.write(&tel, sizeof(tel));
  radio.openReadingPipe(1, addresses[0]);
  radio.startListening();
}

void receive_telemetry_from_ardupilot() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while(Serial2.available()>0) {
    uint8_t c = Serial2.read();

    receive_radio();

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);
          }
          break;

        case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
          {
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
          }
          break;

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          {
            mavlink_global_position_int_t global_position;
            mavlink_msg_global_position_int_decode(&msg, &global_position);
            tel.lat = global_position.lat;
            tel.lon = global_position.lon;
            tel.alt = global_position.alt;
            // tel.relative_alt = global_position.relative_alt;
            send_telemetry_to_ground();
          }
          break;

        case MAVLINK_MSG_ID_VFR_HUD:
          {
            mavlink_vfr_hud_t vfr_hud;
            mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
            tel.ground_speed = vfr_hud.groundspeed;
            send_telemetry_to_ground();
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            tel.pitch = attitude.pitch;
            tel.yaw = attitude.yaw;
            tel.roll = attitude.roll;
            send_telemetry_to_ground();
          }
          break;

       default:
          break;
      }
    }
  }
}

void request_data_from_ardupilot()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x06,0x06};
    
  for (int i=0; i < maxStreams; i++) {
    /*
     * mavlink_msg_request_data_stream_pack(system_id, component_id, 
     *    &msg, 
     *    target_system, target_component, 
     *    MAV_DATA_STREAM_POSITION, 10000000, 1);
     *    
     * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
     *    mavlink_message_t* msg,
     *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
     *    uint16_t req_message_rate, uint8_t start_stop)
     * 
     */
    mavlink_msg_request_data_stream_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, 1, 1, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial2.write(buf, len);
  }
}

void sendHeartbeat()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len); 
}

void sendCommand()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_rc_channels_override_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, 1, 1, rc[0],rc[1],rc[2],rc[3],rc[4],rc[5],rc[6],rc[7],rc[8],rc[9],rc[10],rc[11],rc[12],rc[13],rc[14],rc[15],0,0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}



// void sendGps()
// {
//   mavlink_message_t msg;
//   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//   // mavlink_msg_gps_raw_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 99999999999, 3, 50, 50, 50000, UINT16_MAX, UINT16_MAX, 0, UINT16_MAX, 20, 50, 0, 0, 0, 0, 36000);
//   mavlink_msg_command_long_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, 1, MAV_COMP_ID_AUTOPILOT1, 400, 0, 0, 21196, 0, 0, 0, 0, 0);
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

//   Serial2.write(buf, len);
// }


