#include <mavlink.h>        // Mavlink interface
#include <SoftwareSerial.h>
#define RX_PIN 0
#define TX_PIN 1
SoftwareSerial SoftSerial(RX_PIN, TX_PIN); //Telemetrie TX->Teensy RX=0, Telemetrie RX->Teensy TX=1
int sysid = 1;                   ///< ID 20 for this airplane
int compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

// Define the system type, in this case a quad copter
uint8_t system_type = MAV_TYPE_QUADROTOR;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
uint64_t startTime = millis();
boolean printFull = true;
void setup() {
  Serial.begin(57600);
  while (!Serial) {
    ;
  }
  Serial.println("Serial Connected\t(1/2)");
  SoftSerial.begin(57600);
  while(SoftSerial.available() > 0){
    ;
  }
  Serial.println("SoftSerial Connected\t(2/2)");
}

void loop() {

  /* The default UART header for your MCU */
  int sysid = 1;                   ///< ID 20 for this airplane
  int compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

  // Define the system type, in this case a quad copter
  uint8_t system_type = MAV_TYPE_QUADROTOR;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.

  SoftSerial.write(buf, len);

  delay(1000);
  comm_receive();

  

}


void comm_receive() {

  mavlink_message_t msg;
  mavlink_status_t status;

  // COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)

  while (SoftSerial.available() > 0 )
  {
    uint8_t c = SoftSerial.read();
    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message
      if (true) {
        Serial.println("\n\n======Message======");
        Serial.print("Magic (0xFE,254): ");
        Serial.println(msg.magic);
        Serial.print("Message Count: ");
        Serial.println(status.msg_received);
        Serial.print("Length of Payload: ");
        Serial.println(msg.len);
        Serial.print("Seq: ");
        Serial.println(msg.seq);
        Serial.print("SysID: ");
        Serial.println(msg.sysid);
        Serial.print("CompID: ");
        Serial.println(msg.compid);
        Serial.print("Msg ID: ");
        Serial.println(msg.msgid);
        Serial.println("------Message------\n\n");
      }
      switch (msg.msgid)
      {

        case MAVLINK_MSG_ID_HEARTBEAT:
          {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            if (true) {
              Serial.println("\n\n++++++HeartBeat++++++");
              Serial.print("Type: ");
              Serial.println(hb.type);
              Serial.print("Autopilot: ");
              Serial.println(hb.autopilot);
              Serial.print("Base Mode: ");
              Serial.println(hb.base_mode);
              Serial.print("System Status: ");
              Serial.println(hb.system_status);
              Serial.print("Mavlink Version: ");
              Serial.println(hb.mavlink_version);
              Serial.println("------HeartBeat------\n\n");
            }
            // E.g. read GCS heartbeat and go into
            // comm lost mode if timer times out
          }
          break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
          // EXECUTE ACTION
          //moveU();
          break;
        case MAVLINK_MSG_ID_ATTITUDE:
          mavlink_attitude_t at;
          mavlink_msg_attitude_decode(&msg, &at);
          if (true) {
            Serial.println("\n\n++++++Attitude++++++");
            Serial.print("Roll: ");
            Serial.println(at.roll);
            Serial.print("RollSpeed: ");
            Serial.println(at.rollspeed);
            Serial.print("Yaw: ");
            Serial.println(at.yaw);
            Serial.print("Yaw Speed: ");
            Serial.println(at.yawspeed);
            Serial.print("Pitch: : ");
            Serial.println(at.pitch);
            Serial.print("Pitch Speed: : ");
            Serial.println(at.pitchspeed);
            Serial.println("------Attitutde------\n\n");
          }
          break;
        case MAVLINK_MSG_ID_STATUSTEXT :
          mavlink_statustext_t  st;
          mavlink_msg_statustext_decode (&msg, &st);
          Serial.println("Status Text: ");
          for (int i = 0; i < 50; i ++)
            Serial.print(st.text[i]);
          Serial.println("");
          break;
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
          mavlink_request_data_stream_t req;
          mavlink_msg_request_data_stream_decode(&msg, &req);
          Serial.println("\n------DATA------");
          Serial.print("Reqest Data Stream: ");
          Serial.println(req.target_system);
          Serial.println("------DATA------\n\n");
          moveU();
          break;
        default:
          Serial.println("no message: ID=");
          Serial.println(msg.msgid);
          break;
      }
    }
    // And get the next one
  }
}
void moveDrone(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed) {
  Serial.println("Moving");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t system_mode = MAV_MODE_GUIDED_ARMED; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_ACTIVE; ///< System ready for flight
  mavlink_msg_attitude_pack(sysid, compid, &msg, millis()-startTime, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
  //4th param is system time since start
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SoftSerial.write(buf, len);
  mavlink_attitude_t at;
  mavlink_msg_attitude_decode(&msg, &at);
  Serial.println("\n\n++++++SENDING ATT++++++");
  Serial.print("Roll: ");
  Serial.println(at.roll);
  Serial.print("RollSpeed: ");
  Serial.println(at.rollspeed);
  Serial.print("Yaw: ");
  Serial.println(at.yaw);
  Serial.print("Yaw Speed: ");
  Serial.println(at.yawspeed);
  Serial.print("Pitch: : ");
  Serial.println(at.pitch);
  Serial.print("Pitch Speed: : ");
  Serial.println(at.pitchspeed);
  Serial.println("------SENDING ATT------\n\n");
}
void moveDrone2(float roll, float pitch, float yaw, float thrust) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t system_mode = MAV_MODE_GUIDED_ARMED; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_ACTIVE;
  mavlink_msg_set_roll_pitch_yaw_thrust_pack(sysid, compid, &msg, 200, 200, roll,  pitch,  yaw,  thrust );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SoftSerial.write(buf, len);

}

//TODO: Fill in
void moveF() {
  moveDrone(0, 0, 0, 0, 0, 0);
}
void moveB() {
  moveDrone(0, 0, 0, 0, 0, 0);
}
void moveL() {
  moveDrone(0, 0, 0, 0, 0, 0);
}
void moveR() {
  moveDrone(0, 0, 0, 0, 0, 0);
}
void moveU() {
  moveDrone2(3, 2, 8, 1);
}
void moveD() {
  moveDrone(0, 0, 0, 0, 0, 0);
}
