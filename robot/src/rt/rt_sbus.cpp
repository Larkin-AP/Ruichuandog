/*!
 * @file rt_sbus.cpp
 * @brief Communication with RC controller receiver
 */

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <string>

#ifdef linux
#define termios asmtermios

#include <asm/termios.h>

#undef termios
#endif

#include <termios.h>

//#include "rt/rt_interface_lcm.h"
#include "rt/rt_sbus.h"
#include "rt/rt_serial.h"

// 对遥控器通道进行矫正时候打开此宏,将数值填写到下面到
//#define Show_RT9S_Celebration
//
#define Left_Stick_LRight_Max 1745//1700
#define Left_Stick_LRight_Min 372//320
#define Left_Stick_LRight_Zero 1000

#define Left_Stick_FBack_Max 1566//1700
#define Left_Stick_FBack_Min 244//389
#define Left_Stick_FBack_Zero 1000

#define Right_Stick_FBack_Max 1720
#define Right_Stick_FBack_Min 411
#define Right_Stick_FBack_Zero 1000

#define Right_Stick_LRight_Max 1636
#define Right_Stick_LRight_Min 294
#define Right_Stick_LRight_Zero 1013



pthread_mutex_t sbus_data_m;

uint16_t channels[18];
uint16_t channel_data[18];

/**@brief Name of SBUS serial port in simulator*/
#define K_SBUS_PORT_SIM "/dev/ttyUSB0"
/**@brief Name of SBUS serial port on the mini cheetah*/
#define K_SBUS_PORT_MC "/dev/ttyS4"

/*!
 * Unpack sbus message into channels
 */
void unpack_sbus_data(uint8_t sbus_data[], uint16_t *channels_) {
  if ((sbus_data[0] == 0xF) && (sbus_data[24] == 0x0)) {
    channels_[0] = ((sbus_data[1]) | ((sbus_data[2] & 0x7) << 8));
    channels_[1] = (sbus_data[2] >> 3) | ((sbus_data[3] & 0x3F) << 5);
    channels_[2] = ((sbus_data[3] & 0xC0) >> 6) | (sbus_data[4] << 2) |
                   ((sbus_data[5] & 0x1) << 10);
    channels_[3] = ((sbus_data[5] & 0xFE) >> 1) | ((sbus_data[6] & 0xF) << 7);
    channels_[4] = ((sbus_data[6] & 0xF0) >> 4) | ((sbus_data[7] & 0x7F) << 4);
    channels_[5] = ((sbus_data[7] & 0x80) >> 7) | (sbus_data[8] << 1) |
                   ((sbus_data[9] & 0x3) << 9);
    channels_[6] = ((sbus_data[9] & 0xFC) >> 2) | ((sbus_data[10] & 0x1F) << 6);
    channels_[7] = ((sbus_data[10] & 0xE0) >> 5) | (sbus_data[11] << 3);

    channels_[8] = ((sbus_data[12]) | ((sbus_data[13] & 0x7) << 8));
    channels_[9] = (sbus_data[13] >> 3) | ((sbus_data[14] & 0x3F) << 5);
    channels_[10] = ((sbus_data[14] & 0xC0) >> 6) | (sbus_data[15] << 2) |
                    ((sbus_data[16] & 0x1) << 10);
    channels_[11] =
        ((sbus_data[16] & 0xFE) >> 1) | ((sbus_data[17] & 0xF) << 7);
    channels_[12] =
        ((sbus_data[17] & 0xF0) >> 4) | ((sbus_data[18] & 0x7F) << 4);
    channels_[13] = ((sbus_data[18] & 0x80) >> 7) | (sbus_data[19] << 1) |
                    ((sbus_data[20] & 0x3) << 9);
    channels_[14] =
        ((sbus_data[20] & 0xFC) >> 2) | ((sbus_data[21] & 0x1F) << 6);
    channels_[15] = ((sbus_data[21] & 0xE0) >> 5) | (sbus_data[22] << 3);

    channels_[16] = (sbus_data[23] & 0x80) >> 7;
    channels_[17] = (sbus_data[23] & 0x40) >> 6;

    pthread_mutex_lock(&sbus_data_m);

    for (int i = 0; i < 18; i++) {
       // printf("[%d] %d ", i, channels_[i]);
      channel_data[i] = channels_[i];
    }
    //printf("\n\n");

#ifdef Show_RT9S_Celebration
static int show_rt9s_times=0;
show_rt9s_times++;
if(show_rt9s_times%100)
    {
        printf("Left_Stick_LRight_value =\t %d\n",channel_data[0]);
        printf("Left_Stick_FBack_value = \t%d\n",channel_data[1]);
        printf("Right_Stick_FBack_value = \t%d\n",channel_data[2]);
        printf("Right_Stick_LRight_value = \t%d\n",channel_data[3]);
        printf("------------------------------------------\n");
    }
#endif

    pthread_mutex_unlock(&sbus_data_m);

//     for(int i = 0; i < 18; i++) {
//       printf("[%2d] %04d ", i, channel_data[i]);
//     }
//     printf("\n");

//     for(int i = 0; i < 24; i++) {
//       printf("[%2d] %04d ", i, sbus_data[i]);
//     }
//     printf("\n\n");

  } else {
    // printf("Bad Packet\n");
  }
}

/*!
 * Read data from serial port
 */
int read_sbus_data(int port, uint8_t *sbus_data) {
  uint8_t packet_full = 0;
  uint8_t read_byte[1] = {0};
  int timeout_counter = 0;
//  printf("read_sbus_data\n");
  // int n = read(fd1, read_buff, sizeof(read_buff));
  while ((!packet_full) && (timeout_counter < 50)) {
    timeout_counter++;
    // Read a byte //
    while(read(port, read_byte, sizeof(read_byte)) != 1) {
//        printf("Not read a byte data\n");
    }

    // Shift the buffer //
    for (int i = 0; i < 24; i++) {
      sbus_data[i] = sbus_data[i + 1];
    }
    sbus_data[24] = read_byte[0];

    // Check for the correct start and stop bytes ///
    if ((sbus_data[0] == 15) && (sbus_data[24] == 0)) {
      // unpack_sbus_data(sbus_data_buff, channels);
      packet_full = 1;
    }
   // printf("read sbus once\n");
  }
  return packet_full;
}

/*!
 * Get sbus channel
 */
int read_sbus_channel(int channel) {
  pthread_mutex_lock(&sbus_data_m);
  int value = channel_data[channel];
  pthread_mutex_unlock(&sbus_data_m);
  return value;
}

/*!
 * Receive serial and find packets
 */
int receive_sbus(int port) {
  uint16_t read_buff[25] = {0};
  int x = read_sbus_data(port, (uint8_t *)read_buff);
  if (x) {
    unpack_sbus_data((uint8_t *)read_buff, channels);
  } else {
    printf("SBUS tried read 50 bytes without seeing a packet\n");
  }
  return x;
}

/*!
 * Initialize SBUS serial port
 */
int init_sbus(int is_simulator) {
  // char *port1;
  std::string port1;
  if (is_simulator) {
    port1 = K_SBUS_PORT_SIM;
  } else {
    port1 = K_SBUS_PORT_MC;
  }

  if (pthread_mutex_init(&sbus_data_m, NULL) != 0) {
    printf("Failed to initialize sbus data mutex.\n");
  }

  int fd1 = open(port1.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd1 < 0) {
    printf("Error opening %s: %s\n", port1.c_str(), strerror(errno));
  } else {
    init_serial_for_sbus(fd1, 100000);
#ifdef linux
    //set_interface_attribs_custom_baud(fd1, 100000, 0, 0);
#endif
  }
  printf("SBUS init OK!\n");
  return fd1;
}
// scale to -1 ~ 1
static float scale_joystick(uint16_t in) {
  return (in - 172) * 2.f / (1811.f - 172.f) - 1.f;
}

static TaranisSwitchState map_switch(uint16_t in) {
  switch(in) {
    case 1811:
      return TaranisSwitchState::SWITCH_DOWN;
    case 992:
      return TaranisSwitchState::SWITCH_MIDDLE;
    case 172:
      return TaranisSwitchState::SWITCH_UP;
    default:
      printf("[SBUS] switch returned bad value %d\n", in);
      return TaranisSwitchState::SWITCH_UP;
  }
}


void update_taranis_x7(Taranis_X7_data* data) {
  pthread_mutex_lock(&sbus_data_m);
  data->left_stick[0] = scale_joystick(channel_data[3]);
  data->left_stick[1] = scale_joystick(channel_data[0]);
  data->right_stick[0] = scale_joystick(channel_data[1]);
  data->right_stick[1] = scale_joystick(channel_data[2]);
  data->left_lower_left_switch = map_switch(channel_data[4]);
  data->left_lower_right_switch = map_switch(channel_data[5]);
  data->left_upper_switch = map_switch(channel_data[6]);
  data->right_lower_left_switch = map_switch(channel_data[7]);
  data->right_lower_right_switch = map_switch(channel_data[8]);
  data->right_upper_switch = map_switch(channel_data[9]);
  data->knobs[0] = scale_joystick(channel_data[10]);
  data->knobs[1] = scale_joystick(channel_data[11]);

  pthread_mutex_unlock(&sbus_data_m);
}

#ifdef RC_AT9s
static AT9s_SwitchStateBool map_switch_bool(uint16_t in) {
    switch(in) {
        case 305:
        case 306:
        case 307:
            return AT9s_SwitchStateBool::AT9S_BOOL_UP;
        case 1694:
            return AT9s_SwitchStateBool::AT9S_BOOL_DOWN;
        default:
            //printf("[SBUS] switch returned bad value %d\n", in);
            return AT9s_SwitchStateBool::AT9S_BOOL_UP;
    }
}
static AT9s_SwitchStateTri map_switch_tri(uint16_t in) {
    switch(in) {
        case 305:
        case 306:
        case 307:
            return AT9s_SwitchStateTri::AT9S_TRI_UP;
        case 1000:
            return AT9s_SwitchStateTri::AT9S_TRI_MIDDLE;
        case 1694:
            return AT9s_SwitchStateTri::AT9S_TRI_DOWN;
        default:
            printf("[SBUS] switch returned bad value %d\n", in);
            return AT9s_SwitchStateTri::AT9S_TRI_UP;
    }
}

void update_taranis_at9s(AT9s_data* data)
{
    pthread_mutex_lock(&sbus_data_m);

    if(channel_data[0]<(Left_Stick_LRight_Zero-2))
        data->left_stick_y=-1.0+(channel_data[0]-Left_Stick_LRight_Min)*1.0/(Left_Stick_LRight_Zero-Left_Stick_LRight_Min);
    else if(channel_data[0]>(Left_Stick_LRight_Zero+2))
        data->left_stick_y=1.0-(Left_Stick_LRight_Max-channel_data[0])*1.0/(Left_Stick_LRight_Max-Left_Stick_LRight_Zero);
    else
        data->left_stick_y=0;

    //data->left_stick_y=((channel_data[0]-Left_Stick_LRight_Min)*2.f/(Left_Stick_LRight_Max-Left_Stick_LRight_Min)-1.f);
    if(channel_data[1]>(Left_Stick_FBack_Zero+2))
        data->left_stick_x=-1.0-(channel_data[1]-Left_Stick_FBack_Max)*1.0/(Left_Stick_FBack_Max-Left_Stick_FBack_Zero);
    else if(channel_data[1]<(Left_Stick_FBack_Zero-2))
        data->left_stick_x=1.0-(channel_data[1]-Left_Stick_FBack_Min)*1.0/(Left_Stick_FBack_Zero-Left_Stick_FBack_Min);
    else
        data->left_stick_x=0;
//    data->left_stick_x=-((channel_data[1]-Left_Stick_FBack_Min)*2.f/(Left_Stick_FBack_Max-Left_Stick_FBack_Min)-1.f);
    if(channel_data[2]>(Right_Stick_FBack_Zero+5))
        data->right_stick_x=-1.0-(channel_data[2]-Right_Stick_FBack_Max)*1.0/(Right_Stick_FBack_Max-Right_Stick_FBack_Zero);
    else if(channel_data[2]<(Right_Stick_FBack_Zero-5))
        data->right_stick_x=1.0-(channel_data[2]-Right_Stick_FBack_Min)*1.0/(Right_Stick_FBack_Zero-Right_Stick_FBack_Min);
    else
        data->right_stick_x=0.0;
    //data->right_stick_x=-((channel_data[2]+204-Right_Stick_FBack_Min)*2.f/(Right_Stick_FBack_Max-Right_Stick_FBack_Min)-1.f);

    if(channel_data[3]<(Right_Stick_LRight_Zero-5))
        data->right_stick_y=-1.0+(channel_data[3]-Right_Stick_LRight_Min)*1.0/(Right_Stick_LRight_Zero-Right_Stick_LRight_Min);//
    else if(channel_data[3]>(Right_Stick_LRight_Zero+5))
        data->right_stick_y=1.0-(Right_Stick_LRight_Max-channel_data[3])*1.0/(Right_Stick_LRight_Max-Right_Stick_LRight_Zero);
    else
        data->right_stick_y = 0 ;
   // data->right_stick_y=(channel_data[3]-Right_Stick_LRight_Min)*2.f/(Right_Stick_LRight_Max-Right_Stick_LRight_Min)-1.f-0.07f;

    //data->SWF=map_switch_bool(channel_data[5]); //第六通道为swf
    data->SWG=map_switch_tri(channel_data[5]); // 第六通道为swg
    data->SWE=map_switch_tri(channel_data[4]);
    data->SWA=map_switch_bool(channel_data[6]);
//    data->SWB=map_switch_bool(channel_data[7]);
    data->varB=(channel_data[7]-1000)/700.0; // 数据300 - 1700
//    printf("varBtest: %.2f\n",data->varB);
    data->SWD=map_switch_bool(channel_data[9]);
    data->SWC=map_switch_tri(channel_data[8]);
    pthread_mutex_unlock(&sbus_data_m);

/*   static int count_tims(0);
   count_tims++;
   if(count_tims%10){
       printf("right x: %.2f\t%.2f\t%.2f\t%.2f\t%d\t%d \t%d\t%d\t%d\t%d\t%d\n",
              data->right_stick_x,
              data->right_stick_y,
              data->left_stick_x,
              data->left_stick_y,
              data->SWE,
              channel_data[4],
              channel_data[5],channel_data[6],channel_data[7],channel_data[8],channel_data[9]);
   }*/
}

#endif
