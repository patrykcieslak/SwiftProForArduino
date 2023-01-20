#include "uarm_swift.h"

#define PCK_RX_BUFFER_SIZE  64
#define PIf  3.141592654f

bool connected;
volatile bool report_pos;
volatile uint8_t update_cnt; // Allows for low frequency with 8-bi timer
SerialPacket tx_pck;
SerialPacket rx_pck;
uint8_t pck_rx_buffer[PCK_RX_BUFFER_SIZE];
uint8_t pck_rx_index;
float drive_vel_setpoint[4];
int16_t drive_pos[4];

static void update(void)
{
  sei();
  ++update_cnt;
  if(update_cnt % 5 == 0) // 10Hz
  {
    uarm_swift_update();
    report_pos = true;
    update_cnt = 0;
  }
}

int main(void)
{
  // Initialize system upon power-up.
  serial_init();  // Setup serial baud rate and interrupts
  uarm_swift_init();
  sei(); // Enable interrupts

  // Start update
  report_pos = false;
  update_cnt = 0;
  drive_vel_setpoint[0] = 0.f;
  drive_vel_setpoint[1] = 0.f;
  drive_vel_setpoint[2] = 0.f;
  drive_vel_setpoint[3] = 0.f; 
  drive_pos[0] = 0;
  drive_pos[1] = 0;
  drive_pos[2] = 0;
  drive_pos[3] = 0;
  time0_set(100.f, update);
  time0_start();

  // Main loop (communication)
  serial_reset_read_buffer(); // Clear serial read buffer
  pck_rx_index = 0;
  connected = false;

  while(1)
  {
    if(report_pos && connected)
    {
      report_pos = false;
      tx_pck.cmd = CMD_POSITION;
      tx_pck.dataLen = 8;
      drive_pos[0] = (int16_t)roundf(base_drive_state.position/PIf * INT16_MAX);
      drive_pos[1] = (int16_t)roundf(arml_drive_state.position/PIf * INT16_MAX);
      drive_pos[2] = (int16_t)roundf(armr_drive_state.position/PIf * INT16_MAX);
      drive_pos[3] = (int16_t)roundf(ee_drive_state.position/PIf * INT16_MAX);
      tx_pck.data[0] = (uint8_t)(drive_pos[0] >> 8);
      tx_pck.data[1] = (uint8_t)(drive_pos[0] & 0xFF);
      tx_pck.data[2] = (uint8_t)(drive_pos[1] >> 8);
      tx_pck.data[3] = (uint8_t)(drive_pos[1] & 0xFF);
      tx_pck.data[4] = (uint8_t)(drive_pos[2] >> 8);
      tx_pck.data[5] = (uint8_t)(drive_pos[2] & 0xFF);
      tx_pck.data[6] = (uint8_t)(drive_pos[3] >> 8);
      tx_pck.data[7] = (uint8_t)(drive_pos[3] & 0xFF);
      writePacket(&tx_pck);
    }
  
    // Process communication
    uint8_t rx_count = serial_get_rx_buffer_count();
    for(uint8_t i=0; i<rx_count; ++i)
    {
      pck_rx_buffer[pck_rx_index] = serial_read();
      if(pck_rx_buffer[pck_rx_index] == 0x00)
      {
        if(decodePacket(pck_rx_buffer, pck_rx_index+1, &rx_pck))
        {
          if(!connected && rx_pck.cmd == CMD_CONNECT)
          {
            //beep_tone(100, 1000);
            connected = true;
            tx_pck.cmd = CMD_CONNECT;
            tx_pck.dataLen = 0;
            writePacket(&tx_pck);
          }
          else if(connected)
          {
            switch(rx_pck.cmd)
            {
              case CMD_CONNECT:
              {
                tx_pck.cmd = CMD_CONNECT;
                tx_pck.dataLen = 0;
              }
                break;

              case CMD_ARM_DRIVES:
              {
                drives_arm();
                tx_pck.cmd = CMD_ARM_DRIVES;
                tx_pck.dataLen = 0;
              }
                break;

              case CMD_DISARM_DRIVES:
              {
                drives_disarm();
                tx_pck.cmd = CMD_DISARM_DRIVES;
                tx_pck.dataLen = 0;
              }
                break;

              case CMD_STOP:
              {
                drives_stop();
                tx_pck.cmd = CMD_STOP;
                tx_pck.dataLen = 0;
              }
                break;
  
              case CMD_SET_VELOCITY:
              {
                if(rx_pck.dataLen == 8)
                {
                  drive_vel_setpoint[0] = (float)( (int16_t)((uint16_t)rx_pck.data[0] << 8 | rx_pck.data[1]) )/(float)INT16_MAX * PIf;
                  drive_vel_setpoint[1] = (float)( (int16_t)((uint16_t)rx_pck.data[2] << 8 | rx_pck.data[3]) )/(float)INT16_MAX * PIf;
                  drive_vel_setpoint[2] = (float)( (int16_t)((uint16_t)rx_pck.data[4] << 8 | rx_pck.data[5]) )/(float)INT16_MAX * PIf;
                  drive_vel_setpoint[3] = (float)( (int16_t)((uint16_t)rx_pck.data[6] << 8 | rx_pck.data[7]) )/(float)INT16_MAX * PIf;
                  drives_set_velocity(drive_vel_setpoint);
                  tx_pck.cmd = CMD_SET_VELOCITY;
                  tx_pck.dataLen = 0;
                }
              }
                break;

              case CMD_PUMP_ON:
              {
                pump_on();
                tx_pck.cmd = CMD_PUMP_ON;
                tx_pck.dataLen = 0;
              }
                break;

              case CMD_PUMP_OFF:
              {
                pump_off();
                tx_pck.cmd = CMD_PUMP_OFF;
                tx_pck.dataLen = 0;
              }
                break;

              default:
              {
                tx_pck.cmd = CMD_ERROR;
                tx_pck.dataLen = 2;
                tx_pck.data[0] = (uint8_t)rx_pck.cmd;
                tx_pck.data[1] = (uint8_t)ERR_UNSUPPORTED_CMD;
              }
                break;
            }
            writePacket(&tx_pck);
          }
        }
        else // Error
        {


        }
        pck_rx_index = 0;
      }
      else
      {      
        ++pck_rx_index;
        if(pck_rx_index >= PCK_RX_BUFFER_SIZE)
          pck_rx_index = 0;
      }
    }
  }

  return 0; /* Never reached */
}