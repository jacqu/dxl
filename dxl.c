/*
 * DXL.C : merging all of the Dynamixel lib code into one file
 * 
 * Author: Ryu Woon Jung (Leon)
 * 
 * JG, 6/7/16
 */

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include "dxl.h"

/* Private defines */

#define True                						1
#define False               						0

#define LATENCY_TIMER   								1000  // msec (USB latency timer)

///////////////// for Protocol 1.0 Packet /////////////////
#define TXPACKET_MAX_LEN    						(250)
#define RXPACKET_MAX_LEN    						(250)

#define PKT_HEADER0             				0
#define PKT_HEADER1             				1
#define PKT_ID                  				2
#define PKT_LENGTH              				3
#define PKT_INSTRUCTION         				4
#define PKT_ERROR               				4
#define PKT_PARAMETER0          				5

///////////////// Protocol 1.0 Error bit /////////////////
#define ERRBIT_VOLTAGE          				1       // Supplied voltage is out of the range (operating volatage set in the control table)
#define ERRBIT_ANGLE            				2       // Goal position is written out of the range (from CW angle limit to CCW angle limit)
#define ERRBIT_OVERHEAT         				4       // Temperature is out of the range (operating temperature set in the control table)
#define ERRBIT_RANGE            				8       // Command(setting value) is out of the range for use.
#define ERRBIT_CHECKSUM         				16      // Instruction packet checksum is incorrect.
#define ERRBIT_OVERLOAD         				32      // The current load cannot be controlled by the set torque.
#define ERRBIT_INSTRUCTION      				64      // Undefined instruction or delivering the action command without the reg_write command.

///////////////// for Protocol 2.0 Packet /////////////////
#define TXPACKET_MAX_LEN_2    					(4*1024)
#define RXPACKET_MAX_LEN_2    					(4*1024)

#define PKT_HEADER0_2             			0
#define PKT_HEADER1_2             			1
#define PKT_HEADER2_2             			2
#define PKT_RESERVED_2            			3
#define PKT_ID_2                  			4
#define PKT_LENGTH_L_2            			5
#define PKT_LENGTH_H_2            			6
#define PKT_INSTRUCTION_2         			7
#define PKT_ERROR_2               			8
#define PKT_PARAMETER0_2          			8

///////////////// Protocol 2.0 Error bit /////////////////
#define ERRNUM_RESULT_FAIL_2      			1       // Failed to process the instruction packet.
#define ERRNUM_INSTRUCTION_2      			2       // Instruction error
#define ERRNUM_CRC_2              			3       // CRC check error
#define ERRNUM_DATA_RANGE_2       			4       // Data range error
#define ERRNUM_DATA_LENGTH_2      			5       // Data length error
#define ERRNUM_DATA_LIMIT_2       			6       // Data limit error
#define ERRNUM_ACCESS_2           			7       // Access error

#define ERRBIT_ALERT_2            			128     // When the device has a problem, this bit is set to 1. Check "Device Status Check" value.

/* Private macros */

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

/* Private structure definitions */

typedef struct
{
  int     socket_fd;
  int     baudrate;
  char    port_name[30];

  double  packet_start_time;
  double  packet_timeout;
  double  tx_time_per_byte;
} PortData;

typedef struct
{
  uint8_t     *data_write;
  uint8_t     *data_read;
  uint8_t     *tx_packet;
  uint8_t     *rx_packet;
  uint8_t     error;
  int         communication_result;
  uint8_t     *broadcast_ping_id_list;
}	PacketData;

typedef struct
{
  uint8_t     id;
  uint16_t    start_address;
  uint16_t    data_length;
  uint8_t     *data;
} DataListR;

typedef struct
{
  int         port_num;
  int         protocol_version;

  int         data_list_length;

  uint8_t     last_result;
  uint8_t     is_param_changed;

  DataListR   *data_list;
} GroupDataR;

typedef struct
{
  uint8_t     id;
  uint16_t    data_end;
  uint16_t    start_address;
  uint16_t    data_length;
  uint8_t     *data;
} DataListW;

typedef struct
{
  int         port_num;
  int         protocol_version;

  int         data_list_length;

  uint8_t     is_param_changed;

  uint16_t    param_length;

  DataListW   *data_list;
} GroupDataW;

typedef struct
{
  uint8_t     id;
  uint8_t     *data;
} DataListSR;

typedef struct
{
  int         port_num;
  int         protocol_version;

  int         data_list_length;

  uint8_t     last_result;
  uint8_t     is_param_changed;

  uint16_t    start_address;
  uint16_t    data_length;

  DataListSR  *data_list;
} GroupDataSR;

typedef struct
{
  uint8_t     id;
  uint16_t    data_end;
  uint8_t     *data;
}DataListSW;

typedef struct
{
  int         port_num;
  int         protocol_version;

  int         data_list_length;

  uint8_t     is_param_changed;

  uint16_t    start_address;
  uint16_t    data_length;

  DataListSW  *data_list;
}GroupDataSW;

/* Global variables */

static const int 												DEFAULT_BAUDRATE = 1000000;

int     																g_used_port_num;
uint8_t    															*g_is_using;

int 																		g_used_group_num_r = 0;
int 																		g_used_group_num_w = 0;
int 																		g_used_group_num_sr = 0;
int 																		g_used_group_num_sw = 0;

PortData 																*portData;
PacketData 															*packetData;
GroupDataR 															*groupDataR;
GroupDataW															*groupDataW;
GroupDataSR 														*groupDataSR;
GroupDataSW															*groupDataSW;

/*
 * 
 * port_handler_linux.c
 * 
 * Low-level access functions to Linux serial port.
 * 
 */

int portHandlerLinux(const char *port_name)
{
  int port_num;

  if (portData == NULL)
  {
    port_num = 0;
    g_used_port_num = 1;
    portData = (PortData *)calloc(1, sizeof(PortData));
    g_is_using = (uint8_t*)calloc(1, sizeof(uint8_t));
  }
  else
  {
    for (port_num = 0; port_num < g_used_port_num; port_num++)
    {
      if (!strcmp(portData[port_num].port_name, port_name))
        break;
    }

    if (port_num == g_used_port_num)
    {
      for (port_num = 0; port_num < g_used_port_num; port_num++)
      {
        if (portData[port_num].socket_fd != -1)
          break;
      }

      if (port_num == g_used_port_num)
      {
        g_used_port_num++;
        portData = (PortData*)realloc(portData, g_used_port_num * sizeof(PortData));
        g_is_using = (uint8_t*)realloc(g_is_using, g_used_port_num * sizeof(uint8_t));
      }
    }
    else
    {
      printf("[PortHandler setup] The port number %d has same device name... reinitialize port number %d!!\n", port_num, port_num);
    }
  }

  portData[port_num].socket_fd = -1;
  portData[port_num].baudrate = DEFAULT_BAUDRATE;
  portData[port_num].packet_start_time = 0.0;
  portData[port_num].packet_timeout = 0.0;
  portData[port_num].tx_time_per_byte = 0.0;

  g_is_using[port_num] = False;

  setPortNameLinux(port_num, port_name);

  return port_num;
}

uint8_t openPortLinux(int port_num)
{
  return setBaudRateLinux(port_num, portData[port_num].baudrate);
}

void closePortLinux(int port_num)
{
  if (portData[port_num].socket_fd != -1)
  {
    close(portData[port_num].socket_fd);
    portData[port_num].socket_fd = -1;
  }
}

void clearPortLinux(int port_num)
{
  tcflush(portData[port_num].socket_fd, TCIOFLUSH);
}

void setPortNameLinux(int port_num, const char *port_name)
{
  strcpy(portData[port_num].port_name, port_name);
}

char *getPortNameLinux(int port_num)
{
  return portData[port_num].port_name;
}

uint8_t setBaudRateLinux(int port_num, const int baudrate)
{
  int baud = getCFlagBaud(baudrate);

  closePortLinux(port_num);

  if (baud <= 0)   // custom baudrate
  {
    setupPortLinux(port_num, B38400);
    portData[port_num].baudrate = baudrate;
    return setCustomBaudrateLinux(port_num, baudrate);
  }
  else
  {
    portData[port_num].baudrate = baudrate;
    return setupPortLinux(port_num, baud);
  }
}

int getBaudRateLinux(int port_num)
{
  return portData[port_num].baudrate;
}

int getBytesAvailableLinux(int port_num)
{
  int bytes_available;
  ioctl(portData[port_num].socket_fd, FIONREAD, &bytes_available);
  return bytes_available;
}

int readPortLinux(int port_num, uint8_t *packet, int length)
{
  return read(portData[port_num].socket_fd, packet, length);
}

int writePortLinux(int port_num, uint8_t *packet, int length)
{
  return write(portData[port_num].socket_fd, packet, length);
}

void setPacketTimeoutLinux(int port_num, uint16_t packet_length)
{
  portData[port_num].packet_start_time = getCurrentTimeLinux();
  portData[port_num].packet_timeout = (portData[port_num].tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void setPacketTimeoutMSecLinux(int port_num, double msec)
{
  portData[port_num].packet_start_time = getCurrentTimeLinux();
  portData[port_num].packet_timeout = msec;
}

uint8_t isPacketTimeoutLinux(int port_num)
{
  if (getTimeSinceStartLinux(port_num) > portData[port_num].packet_timeout)
  {
    portData[port_num].packet_timeout = 0;
    return True;
  }
  return False;
}

double getCurrentTimeLinux()
{
  struct timespec tv;
  clock_gettime(CLOCK_REALTIME, &tv);
  return ((double)tv.tv_sec*1000.0 + (double)tv.tv_nsec*0.001*0.001);
}

double getTimeSinceStartLinux(int port_num)
{
  double time_since_start;

  time_since_start = getCurrentTimeLinux() - portData[port_num].packet_start_time;
  if (time_since_start < 0.0)
    portData[port_num].packet_start_time = getCurrentTimeLinux();

  return time_since_start;
}

uint8_t setupPortLinux(int port_num, int cflag_baud)
{
  struct termios newtio;

  portData[port_num].socket_fd = open(portData[port_num].port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (portData[port_num].socket_fd < 0)
  {
    printf("[PortHandlerLinux::SetupPort] Error opening serial port!\n");
    return False;
  }

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  newtio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  // clean the buffer and activate the settings for the port
  tcflush(portData[port_num].socket_fd, TCIFLUSH);
  tcsetattr(portData[port_num].socket_fd, TCSANOW, &newtio);

  portData[port_num].tx_time_per_byte = (1000.0 / (double)portData[port_num].baudrate) * 10.0;
  return True;
}

uint8_t setCustomBaudrateLinux(int port_num, int speed)
{
  // try to set a custom divisor
  struct serial_struct ss;
  if (ioctl(portData[port_num].socket_fd, TIOCGSERIAL, &ss) != 0)
  {
    printf("[PortHandlerLinux::SetCustomBaudrate] TIOCGSERIAL failed!\n");
    return False;
  }

  ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
  ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;
  int closest_speed = ss.baud_base / ss.custom_divisor;

  if (closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100)
  {
    printf("[PortHandlerLinux::setCustomBaudrate] Cannot set speed to %d, closest is %d \n", speed, closest_speed);
    return False;
  }

  if (ioctl(portData[port_num].socket_fd, TIOCSSERIAL, &ss) < 0)
  {
    printf("[PortHandlerLinux::setCustomBaudrate] TIOCSSERIAL failed!\n");
    return False;
  }

  portData[port_num].tx_time_per_byte = (1000.0 / (double)speed) * 10.0;
  return True;
}

int getCFlagBaud(int baudrate)
{
  switch (baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}

/*
 * 
 * port_handler.c
 * 
 * Wrappers to low-level access functions to serial port.
 * 
 */

int     portHandler         (const char *port_name) { return portHandlerLinux(port_name); };

uint8_t openPort            (int port_num) { return openPortLinux(port_num); };
void    closePort           (int port_num) { closePortLinux(port_num); };
void    clearPort           (int port_num) { clearPortLinux(port_num); };

void    setPortName         (int port_num, const char *port_name) { setPortNameLinux(port_num, port_name); };
char   *getPortName         (int port_num) { return getPortNameLinux(port_num); };

uint8_t setBaudRate         (int port_num, const int baudrate) { return setBaudRateLinux(port_num, baudrate); };
int     getBaudRate         (int port_num) { return getBaudRateLinux(port_num); }

int     getBytesAvailable   (int port_num) { return getBytesAvailableLinux(port_num); }

int     readPort            (int port_num, uint8_t *packet, int length) { return readPortLinux(port_num, packet, length); };
int     writePort           (int port_num, uint8_t *packet, int length) { return writePortLinux(port_num, packet, length); };

void    setPacketTimeout    (int port_num, uint16_t packet_length) { setPacketTimeoutLinux(port_num, packet_length); };
void    setPacketTimeoutMSec(int port_num, double msec) { setPacketTimeoutMSecLinux(port_num, msec); };
uint8_t isPacketTimeout     (int port_num) { return isPacketTimeoutLinux(port_num); };

/*
 * 
 * packet_handler.c
 * 
 * Packet management functions.
 * 
 */

void packetHandler()
{
  int port_num;

  if (packetData == NULL)
    packetData = (PacketData*)malloc(1 * sizeof(PacketData));

  packetData = (PacketData*)realloc(packetData, g_used_port_num * sizeof(PacketData));

  for (port_num = 0; port_num < g_used_port_num; port_num++)
  {
    packetData[port_num].data_write = (uint8_t *)calloc(1, sizeof(uint8_t));
    packetData[port_num].data_read = (uint8_t *)calloc(1, sizeof(uint8_t));
    packetData[port_num].tx_packet = (uint8_t *)calloc(1, sizeof(uint8_t));
    packetData[port_num].rx_packet = (uint8_t *)calloc(1, sizeof(uint8_t));
    packetData[port_num].error = 0;
    packetData[port_num].communication_result = 0;
  }
}

void printTxRxResult(int protocol_version, int result)
{
  if (protocol_version == 1)
  {
    printTxRxResult1(result);
  }
  else
  {
    printTxRxResult2(result);
  }
};

void printRxPacketError(int protocol_version, uint8_t error)
{
  if (protocol_version == 1)
  {
    printRxPacketError1(error);
  }
  else
  {
    printRxPacketError2(error);
  }
};

int getLastTxRxResult(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    return getLastTxRxResult1(port_num);
  }
  else
  {
    return getLastTxRxResult2(port_num);
  }
}

uint8_t getLastRxPacketError(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    return getLastRxPacketError1(port_num);
  }
  else
  {
    return getLastRxPacketError2(port_num);
  }
}

void setDataWrite(int port_num, int protocol_version, uint16_t data_length, uint16_t data_pos, uint32_t data)
{
  if (protocol_version == 1)
  {
    setDataWrite1(port_num, data_length, data_pos, data);
  }
  else
  {
    setDataWrite2(port_num, data_length, data_pos, data);
  }
}

uint32_t getDataRead(int port_num, int protocol_version, uint16_t data_length, uint16_t data_pos)
{
  if (protocol_version == 1)
  {
    return getDataRead1(port_num, data_length, data_pos);
  }
  else
  {
    return getDataRead2(port_num, data_length, data_pos);
  }
}

void txPacket(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    txPacket1(port_num);
  }
  else
  {
    txPacket2(port_num);
  }
};

void rxPacket(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    rxPacket1(port_num);
  }
  else
  {
    rxPacket2(port_num);
  }
};

void txRxPacket(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    txRxPacket1(port_num);
  }
  else
  {
    txRxPacket2(port_num);
  }
};

void ping(int port_num, int protocol_version, uint8_t id)
{
  if (protocol_version == 1)
  {
    ping1(port_num, id);
  }
  else
  {
    ping2(port_num, id);
  }
};

uint16_t pingGetModelNum(int port_num, int protocol_version, uint8_t id)
{
  if (protocol_version == 1)
  {
    return pingGetModelNum1(port_num, id);
  }
  else
  {
    return pingGetModelNum2(port_num, id);
  }
};

// broadcastPing
void broadcastPing(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    broadcastPing1(port_num);
  }
  else
  {
    broadcastPing2(port_num);
  }
};

uint8_t getBroadcastPingResult(int port_num, int protocol_version, int id)
{
  if (protocol_version == 1)
  {
    return getBroadcastPingResult1(port_num, id);
  }
  else
  {
    return getBroadcastPingResult2(port_num, id);
  }
}

void reboot(int port_num, int protocol_version, uint8_t id)
{
  if (protocol_version == 1)
  {
    reboot1(port_num, id);
  }
  else
  {
    reboot2(port_num, id);
  }
};

void factoryReset(int port_num, int protocol_version, uint8_t id, uint8_t option)
{
  if (protocol_version == 1)
  {
    factoryReset1(port_num, id, option);
  }
  else
  {
    factoryReset2(port_num, id, option);
  }
};

void readTx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    readTx1(port_num, id, address, length);
  }
  else
  {
    readTx2(port_num, id, address, length);
  }
};

void readRx(int port_num, int protocol_version, uint16_t length)
{
  if (protocol_version == 1)
  {
    readRx1(port_num, length);
  }
  else
  {
    readRx2(port_num, length);
  }
};

void readTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    readTxRx1(port_num, id, address, length);
  }
  else
  {
    readTxRx2(port_num, id, address, length);
  }
};

void read1ByteTx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    read1ByteTx1(port_num, id, address);
  }
  else
  {
    read1ByteTx2(port_num, id, address);
  }
};

uint8_t read1ByteRx(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    return read1ByteRx1(port_num);
  }
  else
  {
    return read1ByteRx2(port_num);
  }
};

uint8_t read1ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    return read1ByteTxRx1(port_num, id, address);
  }
  else
  {
    return read1ByteTxRx2(port_num, id, address);
  }
};

void read2ByteTx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    read2ByteTx1(port_num, id, address);
  }
  else
  {
    read2ByteTx2(port_num, id, address);
  }
};

uint16_t read2ByteRx(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    return read2ByteRx1(port_num);
  }
  else
  {
    return read2ByteRx2(port_num);
  }
};
uint16_t read2ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    return read2ByteTxRx1(port_num, id, address);
  }
  else
  {
    return read2ByteTxRx2(port_num, id, address);
  }
};

void read4ByteTx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    read4ByteTx1(port_num, id, address);
  }
  else
  {
    read4ByteTx2(port_num, id, address);
  }
};

uint32_t read4ByteRx(int port_num, int protocol_version)
{
  if (protocol_version == 1)
  {
    return read4ByteRx1(port_num);
  }
  else
  {
    return read4ByteRx2(port_num);
  }
};

uint32_t read4ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address)
{
  if (protocol_version == 1)
  {
    return read4ByteTxRx1(port_num, id, address);
  }
  else
  {
    return read4ByteTxRx2(port_num, id, address);
  }
};

void writeTxOnly(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    writeTxOnly1(port_num, id, address, length);
  }
  else
  {
    writeTxOnly2(port_num, id, address, length);
  }
};

void writeTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    writeTxRx1(port_num, id, address, length);
  }
  else
  {
    writeTxRx2(port_num, id, address, length);
  }
};

void write1ByteTxOnly(int port_num, int protocol_version, uint8_t id, uint16_t address, uint8_t data)
{
  if (protocol_version == 1)
  {
    write1ByteTxOnly1(port_num, id, address, data);
  }
  else
  {
    write1ByteTxOnly2(port_num, id, address, data);
  }
};

void write1ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint8_t data)
{
  if (protocol_version == 1)
  {
    write1ByteTxRx1(port_num, id, address, data);
  }
  else
  {
    write1ByteTxRx2(port_num, id, address, data);
  }
};

void write2ByteTxOnly(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t data)
{
  if (protocol_version == 1)
  {
    write2ByteTxOnly1(port_num, id, address, data);
  }
  else
  {
    write2ByteTxOnly2(port_num, id, address, data);
  }
};

void write2ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t data)
{
  if (protocol_version == 1)
  {
    write2ByteTxRx1(port_num, id, address, data);
  }
  else
  {
    write2ByteTxRx2(port_num, id, address, data);
  }
};

void write4ByteTxOnly(int port_num, int protocol_version, uint8_t id, uint16_t address, uint32_t data)
{
  if (protocol_version == 1)
  {
    write4ByteTxOnly1(port_num, id, address, data);
  }
  else
  {
    write4ByteTxOnly2(port_num, id, address, data);
  }
};

void write4ByteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint32_t data)
{
  if (protocol_version == 1)
  {
    write4ByteTxRx1(port_num, id, address, data);
  }
  else
  {
    write4ByteTxRx2(port_num, id, address, data);
  }
};

void regWriteTxOnly(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    regWriteTxOnly1(port_num, id, address, length);
  }
  else
  {
    regWriteTxOnly2(port_num, id, address, length);
  }
};

void regWriteTxRx(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length)
{
  if (protocol_version == 1)
  {
    regWriteTxRx1(port_num, id, address, length);
  }
  else
  {
    regWriteTxRx2(port_num, id, address, length);
  }
};

void syncReadTx(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  if (protocol_version == 1)
  {
    syncReadTx1(port_num, start_address, data_length, param_length);
  }
  else
  {
    syncReadTx2(port_num, start_address, data_length, param_length);
  }
};
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

void syncWriteTxOnly(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  if (protocol_version == 1)
  {
    syncWriteTxOnly1(port_num, start_address, data_length, param_length);
  }
  else
  {
    syncWriteTxOnly2(port_num, start_address, data_length, param_length);
  }
};

void bulkReadTx(int port_num, int protocol_version, uint16_t param_length)
{
  if (protocol_version == 1)
  {
    bulkReadTx1(port_num, param_length);
  }
  else
  {
    bulkReadTx2(port_num, param_length);
  }
};
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

void bulkWriteTxOnly(int port_num, int protocol_version, uint16_t param_length)
{
  if (protocol_version == 1)
  {
    bulkWriteTxOnly1(port_num, param_length);
  }
  else
  {
    bulkWriteTxOnly2(port_num, param_length);
  }
};

/*
 * 
 * packet_handler.c
 * 
 * Protocol 1 packet management functions.
 * 
 */

void printTxRxResult1(int result)
{
  switch (result)
  {
    case COMM_SUCCESS:
      printf("[TxRxResult] Communication success.\n");
      break;

    case COMM_PORT_BUSY:
      printf("[TxRxResult] Port is in use!\n");
      break;

    case COMM_TX_FAIL:
      printf("[TxRxResult] Failed transmit instruction packet!\n");
      break;

    case COMM_RX_FAIL:
      printf("[TxRxResult] Failed get status packet from device!\n");
      break;

    case COMM_TX_ERROR:
      printf("[TxRxResult] Incorrect instruction packet!\n");
      break;

    case COMM_RX_WAITING:
      printf("[TxRxResult] Now recieving status packet!\n");
      break;

    case COMM_RX_TIMEOUT:
      printf("[TxRxResult] There is no status packet!\n");
      break;

    case COMM_RX_CORRUPT:
      printf("[TxRxResult] Incorrect status packet!\n");
      break;

    case COMM_NOT_AVAILABLE:
      printf("[TxRxResult] Protocol does not support This function!\n");
      break;

    default:
      break;
  }
}

void printRxPacketError1(uint8_t error)
{
  if (error & ERRBIT_VOLTAGE)
    printf("[RxPacketError] Input voltage error!\n");

  if (error & ERRBIT_ANGLE)
    printf("[RxPacketError] Angle limit error!\n");

  if (error & ERRBIT_OVERHEAT)
    printf("[RxPacketError] Overheat error!\n");

  if (error & ERRBIT_RANGE)
    printf("[RxPacketError] Out of range error!\n");

  if (error & ERRBIT_CHECKSUM)
    printf("[RxPacketError] Checksum error!\n");

  if (error & ERRBIT_OVERLOAD)
    printf("[RxPacketError] Overload error!\n");

  if (error & ERRBIT_INSTRUCTION)
    printf("[RxPacketError] Instruction code error!\n");
}

int getLastTxRxResult1(int port_num)
{
  return packetData[port_num].communication_result;
}
uint8_t getLastRxPacketError1(int port_num)
{
  return packetData[port_num].error;
}

void setDataWrite1(int port_num, uint16_t data_length, uint16_t data_pos, uint32_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, (data_pos + data_length) * sizeof(uint8_t));

  switch (data_length)
  {
    case 1:
      packetData[port_num].data_write[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      break;

    case 2:
      packetData[port_num].data_write[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      packetData[port_num].data_write[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      break;

    default:
      printf("[Set Data for Write] failed");
      break;
  }
}

uint32_t getDataRead1(int port_num, uint16_t data_length, uint16_t data_pos)
{
  switch (data_length)
  {
  case 1:
    return packetData[port_num].data_read[data_pos + 0];

  case 2:
    return DXL_MAKEWORD(packetData[port_num].data_read[data_pos + 0], packetData[port_num].data_read[data_pos + 1]);

  default:
    printf("[Set Data Read] failed... ");
    return 0;
  }
}

void txPacket1(int port_num)
{
  int idx;

  uint8_t checksum = 0;
  uint8_t total_packet_length = packetData[port_num].tx_packet[PKT_LENGTH] + 4; // 4: HEADER0 HEADER1 ID LENGTH
  uint8_t written_packet_length = 0;

  if (g_is_using[port_num])
  {
    packetData[port_num].communication_result = COMM_PORT_BUSY;
    return ;
  }
  g_is_using[port_num] = True;

  // check max packet length
  if (total_packet_length > TXPACKET_MAX_LEN)
  {
    g_is_using[port_num] = False;
    packetData[port_num].communication_result = COMM_TX_ERROR;
    return;
  }

  // make packet header
  packetData[port_num].tx_packet[PKT_HEADER0] = 0xFF;
  packetData[port_num].tx_packet[PKT_HEADER1] = 0xFF;

  // add a checksum to the packet
  for (idx = 2; idx < total_packet_length - 1; idx++)   // except header, checksum
  {
    checksum += packetData[port_num].tx_packet[idx];
  }
  packetData[port_num].tx_packet[total_packet_length - 1] = ~checksum;

  // tx packet
  clearPort(port_num);
  written_packet_length = writePort(port_num, packetData[port_num].tx_packet, total_packet_length);
  if (total_packet_length != written_packet_length)
  {
    g_is_using[port_num] = False;
    packetData[port_num].communication_result = COMM_TX_FAIL;
    return;
  }

  packetData[port_num].communication_result = COMM_SUCCESS;
}

void rxPacket1(int port_num)
{
  uint8_t idx, s;
  int i;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  uint8_t checksum = 0;
  uint8_t rx_length = 0;
  uint8_t wait_length = 6;    // minimum length ( HEADER0 HEADER1 ID LENGTH ERROR CHKSUM )

  while (True)
  {
    rx_length += readPort(port_num, &packetData[port_num].rx_packet[rx_length], wait_length - rx_length);
    if (rx_length >= wait_length)
    {
      idx = 0;

      // find packet header
      for (idx = 0; idx < (rx_length - 1); idx++)
      {
        if (packetData[port_num].rx_packet[idx] == 0xFF && packetData[port_num].rx_packet[idx + 1] == 0xFF)
          break;
      }

      if (idx == 0)   // found at the beginning of the packet
      {
        if (packetData[port_num].rx_packet[PKT_ID] > 0xFD ||                   // unavailable ID
            packetData[port_num].rx_packet[PKT_LENGTH] > RXPACKET_MAX_LEN ||   // unavailable Length
            packetData[port_num].rx_packet[PKT_ERROR] >= 0x64)                 // unavailable Error
        {
          // remove the first byte in the packet
          for (s = 0; s < rx_length - 1; s++)
          {
            packetData[port_num].rx_packet[s] = packetData[port_num].rx_packet[1 + s];
          }

          rx_length -= 1;
          continue;
        }

        // re-calculate the exact length of the rx packet
        wait_length = packetData[port_num].rx_packet[PKT_LENGTH] + PKT_LENGTH + 1;
        if (rx_length < wait_length)
        {
          // check timeout
          if (isPacketTimeout(port_num) == True)
          {
            if (rx_length == 0)
              packetData[port_num].communication_result = COMM_RX_TIMEOUT;
            else
              packetData[port_num].communication_result = COMM_RX_CORRUPT;
            break;
          }
          else
          {
            continue;
          }
        }

        // calculate checksum
        for (i = 2; i < wait_length - 1; i++)   // except header, checksum
        {
          checksum += packetData[port_num].rx_packet[i];
        }
        checksum = ~checksum;

        // verify checksum
        if (packetData[port_num].rx_packet[wait_length - 1] == checksum)
        {
          packetData[port_num].communication_result = COMM_SUCCESS;
        }
        else
        {
          packetData[port_num].communication_result = COMM_RX_CORRUPT;
        }
        break;
      }
      else
      {
        // remove unnecessary packets
        for (s = 0; s < rx_length - idx; s++)
        {
          packetData[port_num].rx_packet[s] = packetData[port_num].rx_packet[idx + s];
        }
        rx_length -= idx;
      }
    }
    else
    {
      // check timeout
      if (isPacketTimeout(port_num) == True)
      {
        if (rx_length == 0)
        {
          packetData[port_num].communication_result = COMM_RX_TIMEOUT;
        }
        else
        {
          packetData[port_num].communication_result = COMM_RX_CORRUPT;
        }
        break;
      }
    }
  }
  g_is_using[port_num] = False;
}

// NOT for BulkRead instruction
void txRxPacket1(int port_num)
{
  packetData[port_num].communication_result = COMM_TX_FAIL;

  // tx packet
  txPacket1(port_num);

  if (packetData[port_num].communication_result != COMM_SUCCESS)
    return;

  // (ID == Broadcast ID && NOT BulkRead) == no need to wait for status packet
  // (Instruction == Action) == no need to wait for status packet
  if ((packetData[port_num].tx_packet[PKT_ID] == BROADCAST_ID && packetData[port_num].tx_packet[PKT_INSTRUCTION] != INST_BULK_READ) ||
    (packetData[port_num].tx_packet[PKT_INSTRUCTION] == INST_ACTION))
  {
    g_is_using[port_num] = False;
    return;
  }

  // set packet timeout
  if (packetData[port_num].tx_packet[PKT_INSTRUCTION] == INST_READ)
  {
    setPacketTimeout(port_num, (uint16_t)(packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] + 6));
  }
  else
  {
    setPacketTimeout(port_num, (uint16_t)6);
  }

  // rx packet
  rxPacket1(port_num);
  // check txpacket ID == rxpacket ID
  if (packetData[port_num].tx_packet[PKT_ID] != packetData[port_num].rx_packet[PKT_ID])
    rxPacket1(port_num);

  if (packetData[port_num].communication_result == COMM_SUCCESS && packetData[port_num].tx_packet[PKT_ID] != BROADCAST_ID)
  {
    if (packetData[port_num].error != 0)
      packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR];
  }
}

void ping1(int port_num, uint8_t id)
{
  pingGetModelNum1(port_num, id);
}

uint16_t pingGetModelNum1(int port_num, uint8_t id)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 2 * sizeof(uint8_t));
  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 6);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 6);

  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return 0;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = 2;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_PING;

  txRxPacket1(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    readTxRx1(port_num, id, 0, 2);  // Address 0 : Model Number
    if (packetData[port_num].communication_result == COMM_SUCCESS)
      return DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]);
  }

  return 0;
}

void broadcastPing1(int port_num)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}

uint8_t getBroadcastPingResult1(int port_num, int id)
{
  return False;
}

void action1(int port_num, uint8_t id)
{
  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 6);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = 2;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_ACTION;

  txRxPacket1(port_num);
}

void reboot1(int port_num, uint8_t id)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}

void factoryReset1(int port_num, uint8_t id, uint8_t option)
{
  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 6);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 6);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = 2;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_FACTORY_RESET;

  txRxPacket1(port_num);
}

void readTx1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 8);

  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = 4;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = (uint8_t)address;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = (uint8_t)length;

  txPacket1(port_num);

  // set packet timeout
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    setPacketTimeout(port_num, (uint16_t)(length + 6));
}

void readRx1(int port_num, uint16_t length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, RXPACKET_MAX_LEN);

  rxPacket1(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    if (packetData[port_num].error != 0)
      packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR];
    for (s = 0; s < length; s++)
    {
      packetData[port_num].data_read[s] = packetData[port_num].rx_packet[PKT_PARAMETER0 + s];
    }
  }
}

void readTxRx1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint8_t s;
  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 8);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, RXPACKET_MAX_LEN);

  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = 4;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = (uint8_t)address;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = (uint8_t)length;

  txRxPacket1(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    if (packetData[port_num].error != 0)
      packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR];
    for (s = 0; s < length; s++)
    {
      packetData[port_num].data_read[s] = packetData[port_num].rx_packet[PKT_PARAMETER0 + s];
    }
  }
}

void read1ByteTx1(int port_num, uint8_t id, uint16_t address)
{
  readTx1(port_num, id, address, 1);
}
uint8_t read1ByteRx1(int port_num)
{
	packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 1 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  readRx1(port_num, 1);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return packetData[port_num].data_read[0];
  return 0;
}
uint8_t read1ByteTxRx1(int port_num, uint8_t id, uint16_t address)
{
	packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 1 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  readTxRx1(port_num, id, address, 1);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return packetData[port_num].data_read[0];
  return 0;
}

void read2ByteTx1(int port_num, uint8_t id, uint16_t address)
{
  readTx1(port_num, id, address, 2);
}
uint16_t read2ByteRx1(int port_num)
{
	packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 2 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  readRx1(port_num, 2);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]);
  return 0;
}
uint16_t read2ByteTxRx1(int port_num, uint8_t id, uint16_t address)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 2 * sizeof(uint8_t));
	packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  readTxRx1(port_num, id, address, 2);

  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]);

  return 0;
}

void read4ByteTx1(int port_num, uint8_t id, uint16_t address)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}
uint32_t read4ByteRx1(int port_num)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
  return 0;
}
uint32_t read4ByteTxRx1(int port_num, uint8_t id, uint16_t address)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
  return 0;
}

void writeTxOnly1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 7);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = length + 3;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0] = (uint8_t)address;

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 1 + s] = packetData[port_num].data_write[s];
  }

  txPacket1(port_num);
  g_is_using[port_num] = False;

}

void writeTxRx1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 7);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 6);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = length + 3;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0] = (uint8_t)address;

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 1 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket1(port_num);
}

void write1ByteTxOnly1(int port_num, uint8_t id, uint16_t address, uint8_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 1 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = data;
  writeTxOnly1(port_num, id, address, 1);
}
void write1ByteTxRx1(int port_num, uint8_t id, uint16_t address, uint8_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 1 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = data;
  writeTxRx1(port_num, id, address, 1);
}

void write2ByteTxOnly1(int port_num, uint8_t id, uint16_t address, uint16_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 2 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = DXL_LOBYTE(data);
  packetData[port_num].data_write[1] = DXL_HIBYTE(data);
  writeTxOnly1(port_num, id, address, 2);
}
void write2ByteTxRx1(int port_num, uint8_t id, uint16_t address, uint16_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 2 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = DXL_LOBYTE(data);
  packetData[port_num].data_write[1] = DXL_HIBYTE(data);
  writeTxRx1(port_num, id, address, 2);
}

void write4ByteTxOnly1(int port_num, uint8_t id, uint16_t address, uint32_t data)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}
void write4ByteTxRx1(int port_num, uint8_t id, uint16_t address, uint32_t data)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}

void regWriteTxOnly1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 6);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = length + 3;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_REG_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0] = (uint8_t)address;

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 1 + s] = packetData[port_num].data_write[s];
  }

   txPacket1(port_num);
  g_is_using[port_num] = False;
}

void regWriteTxRx1(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 6);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 6);

  packetData[port_num].tx_packet[PKT_ID] = id;
  packetData[port_num].tx_packet[PKT_LENGTH] = length + 3;
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_REG_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0] = (uint8_t)address;

  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, length * sizeof(uint8_t));

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 1 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket1(port_num);
}

void syncReadTx1(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}

void syncWriteTxOnly1(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 8); // 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN ... CHKSUM

  packetData[port_num].tx_packet[PKT_ID] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH] = param_length + 4; // 4: INST START_ADDR DATA_LEN ... CHKSUM
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_SYNC_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = start_address;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 1] = data_length;

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 2 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket1(port_num);
}

void bulkReadTx1(int port_num, uint16_t param_length)
{
  uint8_t s;

  int i;
  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 7);  // 7: HEADER0 HEADER1 ID LEN INST 0x00 ... CHKSUM

  packetData[port_num].tx_packet[PKT_ID] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH] = param_length + 3; // 3: INST 0x00 ... CHKSUM
  packetData[port_num].tx_packet[PKT_INSTRUCTION] = INST_BULK_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0 + 0] = 0x00;

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0 + 1 + s] = packetData[port_num].data_write[s];
  }

  txPacket1(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    int wait_length = 0;
    for (i = 0; i < param_length; i += 3)
    {
      wait_length += packetData[port_num].data_write[i] + 7;
    }

    setPacketTimeout(port_num, (uint16_t)wait_length);
  }
}

void bulkWriteTxOnly1(int port_num, uint16_t param_length)
{
  packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
}

/*
 * 
 * packet_handler.c
 * 
 * Protocol 2 packet management functions.
 * 
 */

void printTxRxResult2(int result)
{
  switch (result)
  {
    case COMM_SUCCESS:
      printf("[TxRxResult] Communication success.\n");
      break;

    case COMM_PORT_BUSY:
      printf("[TxRxResult] Port is in use!\n");
      break;

    case COMM_TX_FAIL:
      printf("[TxRxResult] Failed transmit instruction packet!\n");
      break;

    case COMM_RX_FAIL:
      printf("[TxRxResult] Failed get status packet from device!\n");
      break;

    case COMM_TX_ERROR:
      printf("[TxRxResult] Incorrect instruction packet!\n");
      break;

    case COMM_RX_WAITING:
      printf("[TxRxResult] Now recieving status packet!\n");
      break;

    case COMM_RX_TIMEOUT:
      printf("[TxRxResult] There is no status packet!\n");
      break;

    case COMM_RX_CORRUPT:
      printf("[TxRxResult] Incorrect status packet!\n");
      break;

    case COMM_NOT_AVAILABLE:
      printf("[TxRxResult] Protocol does not support This function!\n");
      break;

    default:
      break;
  }
}

void printRxPacketError2(uint8_t error)
{
  if (error & ERRBIT_ALERT_2)
    printf("[RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!\n");

  int not_alert_error = error & ~ERRBIT_ALERT_2;

  switch (not_alert_error)
  {
    case 0:
      break;

    case ERRNUM_RESULT_FAIL_2:
      printf("[RxPacketError] Failed to process the instruction packet!\n");
      break;

    case ERRNUM_INSTRUCTION_2:
      printf("[RxPacketError] Undefined instruction or incorrect instruction!\n");
      break;

    case ERRNUM_CRC_2:
      printf("[RxPacketError] CRC doesn't match!\n");
      break;

    case ERRNUM_DATA_RANGE_2:
      printf("[RxPacketError] The data value is out of range!\n");
      break;

    case ERRNUM_DATA_LENGTH_2:
      printf("[RxPacketError] The data length does not match as expected!\n");
      break;

    case ERRNUM_DATA_LIMIT_2:
      printf("[RxPacketError] The data value exceeds the limit value!\n");
      break;

    case ERRNUM_ACCESS_2:
      printf("[RxPacketError] Writing or Reading is not available to target address!\n");
      break;

    default:
      printf("[RxPacketError] Unknown error code!\n");
      break;
  }
}

int getLastTxRxResult2(int port_num)
{
  return packetData[port_num].communication_result;
}
uint8_t getLastRxPacketError2(int port_num)
{
  return packetData[port_num].error;
}

void setDataWrite2(int port_num, uint16_t data_length, uint16_t data_pos, uint32_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, (data_pos + data_length) * sizeof(uint8_t));

  switch (data_length)
  {
    case 1:
      packetData[port_num].data_write[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      break;

    case 2:
      packetData[port_num].data_write[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      packetData[port_num].data_write[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      break;

    case 4:
      packetData[port_num].data_write[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      packetData[port_num].data_write[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      packetData[port_num].data_write[data_pos + 2] = DXL_LOBYTE(DXL_HIWORD(data));
      packetData[port_num].data_write[data_pos + 3] = DXL_HIBYTE(DXL_HIWORD(data));
      break;

    default:
      printf("[Set Data Write] failed... ");
      break;
  }
}
uint32_t getDataRead2(int port_num, uint16_t data_length, uint16_t data_pos)
{
  switch (data_length)
  {
  case 1:
    return packetData[port_num].data_read[data_pos + 0];

  case 2:
    return DXL_MAKEWORD(packetData[port_num].data_read[data_pos + 0], packetData[port_num].data_read[data_pos + 1]);

  case 4:
    return DXL_MAKEDWORD(DXL_MAKEWORD(packetData[port_num].data_read[data_pos + 0], packetData[port_num].data_read[data_pos + 1])
      , DXL_MAKEWORD(packetData[port_num].data_read[data_pos + 2], packetData[port_num].data_read[data_pos + 3]));

  default:
    printf("[Set Data Read] failed... ");
    return 0;
  }
}

unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  uint16_t i, j;
  uint16_t crc_table[256] = { 0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202 };

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

void addStuffing(uint8_t *packet)
{
  uint8_t s;

  int i = 0, index = 0;
  int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L_2], packet[PKT_LENGTH_H_2]);
  int packet_length_out = packet_length_in;
  uint8_t temp[TXPACKET_MAX_LEN_2] = { 0 };

  for (s = PKT_HEADER0_2; s <= PKT_LENGTH_H_2; s++)
  {
    temp[s] = packet[s]; // FF FF FD XX ID LEN_L LEN_H
  }

  index = PKT_INSTRUCTION_2;
  for (i = 0; i < packet_length_in - 2; i++)  // except CRC
  {
    temp[index++] = packet[i + PKT_INSTRUCTION_2];
    if (packet[i + PKT_INSTRUCTION_2] == 0xFD && packet[i + PKT_INSTRUCTION_2 - 1] == 0xFF && packet[i + PKT_INSTRUCTION_2 - 2] == 0xFF)
    {   // FF FF FD
      temp[index++] = 0xFD;
      packet_length_out++;
    }
  }
  temp[index++] = packet[PKT_INSTRUCTION_2 + packet_length_in - 2];
  temp[index++] = packet[PKT_INSTRUCTION_2 + packet_length_in - 1];

  if (packet_length_in != packet_length_out)
    packet = (uint8_t *)realloc(packet, index * sizeof(uint8_t));

  for (s = 0; s < index; s++)
  {
    packet[s] = temp[s];
  }

  packet[PKT_LENGTH_L_2] = DXL_LOBYTE(packet_length_out);
  packet[PKT_LENGTH_H_2] = DXL_HIBYTE(packet_length_out);
}

void removeStuffing(uint8_t *packet)
{
  int i = 0, index = 0;
  int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L_2], packet[PKT_LENGTH_H_2]);
  int packet_length_out = packet_length_in;

  index = PKT_INSTRUCTION_2;
  for (i = 0; i < packet_length_in - 2; i++)  // except CRC
  {
    if (packet[i + PKT_INSTRUCTION_2] == 0xFD && packet[i + PKT_INSTRUCTION_2 + 1] == 0xFD && packet[i + PKT_INSTRUCTION_2 - 1] == 0xFF && packet[i + PKT_INSTRUCTION_2 - 2] == 0xFF)
    {   // FF FF FD FD
      packet_length_out--;
      i++;
    }
    packet[index++] = packet[i + PKT_INSTRUCTION_2];
  }
  packet[index++] = packet[PKT_INSTRUCTION_2 + packet_length_in - 2];
  packet[index++] = packet[PKT_INSTRUCTION_2 + packet_length_in - 1];

  packet[PKT_LENGTH_L_2] = DXL_LOBYTE(packet_length_out);
  packet[PKT_LENGTH_H_2] = DXL_HIBYTE(packet_length_out);
}

void txPacket2(int port_num)
{
  uint16_t total_packet_length = 0;
  uint16_t written_packet_length = 0;

  if (g_is_using[port_num])
  {
    packetData[port_num].communication_result = COMM_PORT_BUSY;
    return;
  }
  g_is_using[port_num] = True;

  // byte stuffing for header
  addStuffing(packetData[port_num].tx_packet);

  // check max packet length
  total_packet_length = DXL_MAKEWORD(packetData[port_num].tx_packet[PKT_LENGTH_L_2], packetData[port_num].tx_packet[PKT_LENGTH_H_2]) + 7;
  // 7: HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H
  if (total_packet_length > TXPACKET_MAX_LEN_2)
  {
    g_is_using[port_num] = False;
    packetData[port_num].communication_result = COMM_TX_ERROR;
    return;
  }

  // make packet header
  packetData[port_num].tx_packet[PKT_HEADER0_2] = 0xFF;
  packetData[port_num].tx_packet[PKT_HEADER1_2] = 0xFF;
  packetData[port_num].tx_packet[PKT_HEADER2_2] = 0xFD;
  packetData[port_num].tx_packet[PKT_RESERVED_2] = 0x00;

  // add CRC16
  uint16_t crc = updateCRC(0, packetData[port_num].tx_packet, total_packet_length - 2);    // 2: CRC16
  packetData[port_num].tx_packet[total_packet_length - 2] = DXL_LOBYTE(crc);
  packetData[port_num].tx_packet[total_packet_length - 1] = DXL_HIBYTE(crc);

  // tx packet
  clearPort(port_num);
  written_packet_length = writePort(port_num, packetData[port_num].tx_packet, total_packet_length);
  if (total_packet_length != written_packet_length)
  {
    g_is_using[port_num] = False;
    packetData[port_num].communication_result = COMM_TX_FAIL;
    return;
  }

  packetData[port_num].communication_result = COMM_SUCCESS;
}

void rxPacket2(int port_num)
{
  uint8_t s;
  uint16_t idx;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  uint16_t rx_length = 0;
  uint16_t wait_length = 11;
  // minimum length ( HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H )

  while (True)
  {
    rx_length += readPort(port_num, &packetData[port_num].rx_packet[rx_length], wait_length - rx_length);
    if (rx_length >= wait_length)
    {
      idx = 0;

      // find packet header
      for (idx = 0; idx < (rx_length - 3); idx++)
      {
        if ((packetData[port_num].rx_packet[idx] == 0xFF) && (packetData[port_num].rx_packet[idx + 1] == 0xFF) && (packetData[port_num].rx_packet[idx + 2] == 0xFD) && (packetData[port_num].rx_packet[idx + 3] != 0xFD))
          break;
      }

      if (idx == 0)   // found at the beginning of the packet
      {
        if (packetData[port_num].rx_packet[PKT_RESERVED_2] != 0x00 ||
          packetData[port_num].rx_packet[PKT_ID_2] > 0xFC ||
          DXL_MAKEWORD(packetData[port_num].rx_packet[PKT_LENGTH_L_2], packetData[port_num].rx_packet[PKT_LENGTH_H_2]) > RXPACKET_MAX_LEN_2 ||
          packetData[port_num].rx_packet[PKT_INSTRUCTION_2] != 0x55)
        {
          // remove the first byte in the packet
          for (s = 0; s < rx_length - 1; s++)
          {
            packetData[port_num].rx_packet[s] = packetData[port_num].rx_packet[1 + s];
          }

          rx_length -= 1;
          continue;
        }

        // re-calculate the exact length of the rx packet
        if (wait_length != DXL_MAKEWORD(packetData[port_num].rx_packet[PKT_LENGTH_L_2], packetData[port_num].rx_packet[PKT_LENGTH_H_2]) + PKT_LENGTH_H_2 + 1)
        {
          wait_length = DXL_MAKEWORD(packetData[port_num].rx_packet[PKT_LENGTH_L_2], packetData[port_num].rx_packet[PKT_LENGTH_H_2]) + PKT_LENGTH_H_2 + 1;
          continue;
        }

        if (rx_length < wait_length)
        {
          // check timeout
          if (isPacketTimeout(port_num) == True)
          {
            if (rx_length == 0)
            {
              packetData[port_num].communication_result = COMM_RX_TIMEOUT;
            }
            else
            {
              packetData[port_num].communication_result = COMM_RX_CORRUPT;
            }
            break;
          }
          else
          {
            continue;
          }
        }

        // verify CRC16
        uint16_t crc = DXL_MAKEWORD(packetData[port_num].rx_packet[wait_length - 2], packetData[port_num].rx_packet[wait_length - 1]);
        if (updateCRC(0, packetData[port_num].rx_packet, wait_length - 2) == crc)
        {
          packetData[port_num].communication_result = COMM_SUCCESS;
        }
        else
        {
          packetData[port_num].communication_result = COMM_RX_CORRUPT;
        }
        break;
      }
      else
      {
        // remove unnecessary packets
        for (s = 0; s < rx_length - idx; s++)
        {
          packetData[port_num].rx_packet[s] = packetData[port_num].rx_packet[idx + s];
        }

        rx_length -= idx;
      }
    }
    else
    {
      // check timeout
      if (isPacketTimeout(port_num) == True)
      {
        if (rx_length == 0)
        {
          packetData[port_num].communication_result = COMM_RX_TIMEOUT;
        }
        else
        {
          packetData[port_num].communication_result = COMM_RX_CORRUPT;
        }
        break;
      }
    }
  }
  g_is_using[port_num] = False;

  if (packetData[port_num].communication_result == COMM_SUCCESS)
    removeStuffing(packetData[port_num].rx_packet);
}

// NOT for BulkRead / SyncRead instruction
void txRxPacket2(int port_num)
{
  packetData[port_num].communication_result = COMM_TX_FAIL;

  // tx packet
  txPacket2(port_num);
  if (packetData[port_num].communication_result != COMM_SUCCESS)
    return;

  // (ID == Broadcast ID && NOT BulkRead) == no need to wait for status packet
  // (Instruction == Action) == no need to wait for status packet
  if ((packetData[port_num].tx_packet[PKT_ID_2] == BROADCAST_ID && packetData[port_num].tx_packet[PKT_INSTRUCTION_2] != INST_BULK_READ) ||
    (packetData[port_num].tx_packet[PKT_ID_2] == BROADCAST_ID && packetData[port_num].tx_packet[PKT_INSTRUCTION_2] != INST_SYNC_READ) ||
    (packetData[port_num].tx_packet[PKT_INSTRUCTION_2] == INST_ACTION))
  {
    g_is_using[port_num] = False;
    return;
  }

  // set packet timeout
  if (packetData[port_num].tx_packet[PKT_INSTRUCTION_2] == INST_READ)
  {
    setPacketTimeout(port_num, (uint16_t)(DXL_MAKEWORD(packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 2], packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 3]) + 11));
  }
  else
  {
    setPacketTimeout(port_num, (uint16_t)11);   // HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H
  }

  // rx packet
  rxPacket2(port_num);
  // check txpacket ID == rxpacket ID
  if (packetData[port_num].tx_packet[PKT_ID_2] != packetData[port_num].rx_packet[PKT_ID_2])
    rxPacket2(port_num);

  if (packetData[port_num].communication_result == COMM_SUCCESS && packetData[port_num].tx_packet[PKT_ID_2] != BROADCAST_ID)
  {
    packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR_2];
  }
}

void ping2(int port_num, uint8_t id)
{
  pingGetModelNum2(port_num, id);
}

uint16_t pingGetModelNum2(int port_num, uint8_t id)
{
  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 10);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 14);

  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return 0;
  }

  packetData[port_num].tx_packet[PKT_ID_2] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = 3;
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_PING;

  txRxPacket2(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEWORD(packetData[port_num].rx_packet[PKT_PARAMETER0_2 + 1], packetData[port_num].rx_packet[PKT_PARAMETER0_2 + 2]);
  return 0;
}

void broadcastPing2(int port_num)
{
  uint8_t s;
  int id;
  uint16_t idx;

  packetData[port_num].broadcast_ping_id_list = (uint8_t *)calloc(255, sizeof(uint8_t));

  const int STATUS_LENGTH     = 14;
  int result = COMM_TX_FAIL;

  for (id = 0; id < 255; id++)
  {
    packetData[port_num].broadcast_ping_id_list[id] = 255;
  }

  uint16_t rx_length = 0;
  uint16_t wait_length = STATUS_LENGTH * MAX_ID;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 10 * sizeof(uint8_t));
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, STATUS_LENGTH * MAX_ID * sizeof(uint8_t));

  packetData[port_num].tx_packet[PKT_ID_2] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = 3;
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_PING;

  txPacket2(port_num);
  if (packetData[port_num].communication_result != COMM_SUCCESS)
  {
    g_is_using[port_num] = False;
    return;
  }

  // set rx timeout
  setPacketTimeout(port_num, (uint16_t)(wait_length * 30));

  while (1)
  {
    rx_length += readPort(port_num, &packetData[port_num].rx_packet[rx_length], wait_length - rx_length);
    if (isPacketTimeout(port_num) == True)// || rx_length >= wait_length)
      break;
  }

  g_is_using[port_num] = False;

  if (rx_length == 0)
  {
    packetData[port_num].communication_result = COMM_RX_TIMEOUT;
    return;
  }

  while (1)
  {
    if (rx_length < STATUS_LENGTH)
    {
      packetData[port_num].communication_result = COMM_RX_CORRUPT;
    }

    idx = 0;

    // find packet header
    for (idx = 0; idx < (rx_length - 2); idx++)
    {
      if (packetData[port_num].rx_packet[idx] == 0xFF && packetData[port_num].rx_packet[idx + 1] == 0xFF && packetData[port_num].rx_packet[idx + 2] == 0xFD)
        break;
    }

    if (idx == 0)   // found at the beginning of the packet
    {
      // verify CRC16
      uint16_t crc = DXL_MAKEWORD(packetData[port_num].rx_packet[STATUS_LENGTH - 2], packetData[port_num].rx_packet[STATUS_LENGTH - 1]);

      if (updateCRC(0, packetData[port_num].rx_packet, STATUS_LENGTH - 2) == crc)
      {
        packetData[port_num].communication_result = COMM_SUCCESS;

        packetData[port_num].broadcast_ping_id_list[packetData[port_num].rx_packet[PKT_ID_2]] = packetData[port_num].rx_packet[PKT_ID_2];

        for (s = 0; s < rx_length - STATUS_LENGTH; s++)
        {
          packetData[port_num].rx_packet[s] = packetData[port_num].rx_packet[STATUS_LENGTH + s];
        }
        rx_length -= STATUS_LENGTH;

        if (rx_length == 0)
          return;
      }
      else
      {
        result = COMM_RX_CORRUPT;

        // remove header (0xFF 0xFF 0xFD)
        for (s = 0; s < rx_length - 3; s++)
        {
          packetData[port_num].rx_packet[s] = packetData[port_num].rx_packet[3 + s];
        }
        rx_length -= 3;
      }
    }
    else
    {
      // remove unnecessary packets
      for (s = 0; s < rx_length - idx; s++)
      {
        packetData[port_num].rx_packet[s] = packetData[port_num].rx_packet[idx + s];
      }
      rx_length -= idx;
    }
  }

  packetData[port_num].communication_result = result;
  return;
}

uint8_t getBroadcastPingResult2(int port_num, int id)
{
  if (packetData[port_num].broadcast_ping_id_list[id] == id)
  {
    return True;
  }
  else
  {
    return False;
  }
}

void action2(int port_num, uint8_t id)
{
  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 10);

  packetData[port_num].tx_packet[PKT_ID_2] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = 3;
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_ACTION;

  txRxPacket2(port_num);
}

void reboot2(int port_num, uint8_t id)
{
  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 10);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 11);

  packetData[port_num].tx_packet[PKT_ID_2] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = 3;
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_REBOOT;

  txRxPacket2(port_num);
}

void factoryReset2(int port_num, uint8_t id, uint8_t option)
{
  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 11);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 11);

  packetData[port_num].tx_packet[PKT_ID_2] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = 4;
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_FACTORY_RESET;
  packetData[port_num].tx_packet[PKT_PARAMETER0_2] = option;

  txRxPacket2(port_num);
}

void readTx2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)malloc(14);

  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[port_num].tx_packet[PKT_ID_2] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = 7;
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 1] = (uint8_t)DXL_HIBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 2] = (uint8_t)DXL_LOBYTE(length);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 3] = (uint8_t)DXL_HIBYTE(length);

  txPacket2(port_num);

  // set packet timeout
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    setPacketTimeout(port_num, (uint16_t)(length + 11));
}

void readRx2(int port_num, uint16_t length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, RXPACKET_MAX_LEN_2);  //(length + 11 + (length/3));  // (length/3): consider stuffing

  rxPacket2(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    if (packetData[port_num].error != 0)
      packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR_2];
    for (s = 0; s < length; s++)
    {
      packetData[port_num].data_read[s] = packetData[port_num].rx_packet[PKT_PARAMETER0_2 + 1 + s];
    }
  }
}

void readTxRx2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, 14);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, RXPACKET_MAX_LEN_2);  //(length + 11 + (length/3));  // (length/3): consider stuffing

  if (id >= BROADCAST_ID)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[port_num].tx_packet[PKT_ID_2] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = 7;
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = 0;
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 1] = (uint8_t)DXL_HIBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 2] = (uint8_t)DXL_LOBYTE(length);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 3] = (uint8_t)DXL_HIBYTE(length);

  txRxPacket2(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    if (packetData[port_num].error != 0)
      packetData[port_num].error = (uint8_t)packetData[port_num].rx_packet[PKT_ERROR_2];
    for (s = 0; s < length; s++)
    {
      packetData[port_num].data_read[s] = packetData[port_num].rx_packet[PKT_PARAMETER0_2 + 1 + s];
    }
  }
}

void read1ByteTx2(int port_num, uint8_t id, uint16_t address)
{
  readTx2(port_num, id, address, 1);
}
uint8_t read1ByteRx2(int port_num)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 1 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  readRx2(port_num, 1);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return packetData[port_num].data_read[0];
  return 0;
}
uint8_t read1ByteTxRx2(int port_num, uint8_t id, uint16_t address)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 1 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  readTxRx2(port_num, id, address, 1);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return packetData[port_num].data_read[0];
  return 0;
}

void read2ByteTx2(int port_num, uint8_t id, uint16_t address)
{
  readTx2(port_num, id, address, 2);
}
uint16_t read2ByteRx2(int port_num)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 2 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  readRx2(port_num, 2);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]);
  return 0;
}
uint16_t read2ByteTxRx2(int port_num, uint8_t id, uint16_t address)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 2 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  readTxRx2(port_num, id, address, 2);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]);
  return 0;
}

void read4ByteTx2(int port_num, uint8_t id, uint16_t address)
{
  readTx2(port_num, id, address, 4);
}
uint32_t read4ByteRx2(int port_num)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 4 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  packetData[port_num].data_read[2] = 0;
  packetData[port_num].data_read[3] = 0;
  readRx2(port_num, 4);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEDWORD(DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]), DXL_MAKEWORD(packetData[port_num].data_read[2], packetData[port_num].data_read[3]));
  return 0;
}
uint32_t read4ByteTxRx2(int port_num, uint8_t id, uint16_t address)
{
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, 4 * sizeof(uint8_t));
  packetData[port_num].data_read[0] = 0;
  packetData[port_num].data_read[1] = 0;
  packetData[port_num].data_read[2] = 0;
  packetData[port_num].data_read[3] = 0;
  readTxRx2(port_num, id, address, 4);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
    return DXL_MAKEDWORD(DXL_MAKEWORD(packetData[port_num].data_read[0], packetData[port_num].data_read[1]), DXL_MAKEWORD(packetData[port_num].data_read[2], packetData[port_num].data_read[3]));
  return 0;
}


void writeTxOnly2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 12);

  packetData[port_num].tx_packet[PKT_ID_2] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = DXL_LOBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = DXL_HIBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 1] = (uint8_t)DXL_HIBYTE(address);

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 2 + s] = packetData[port_num].data_write[s];
  }

  txPacket2(port_num);
  g_is_using[port_num] = False;
}

void writeTxRx2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 12);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 11);

  packetData[port_num].tx_packet[PKT_ID_2] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = DXL_LOBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = DXL_HIBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 1] = (uint8_t)DXL_HIBYTE(address);

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 2 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket2(port_num);
}

void write1ByteTxOnly2(int port_num, uint8_t id, uint16_t address, uint8_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 1 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = data;
  writeTxOnly2(port_num, id, address, 1);
}
void write1ByteTxRx2(int port_num, uint8_t id, uint16_t address, uint8_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 1 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = data;
  writeTxRx2(port_num, id, address, 1);
}

void write2ByteTxOnly2(int port_num, uint8_t id, uint16_t address, uint16_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 2 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = DXL_LOBYTE(data);
  packetData[port_num].data_write[1] = DXL_HIBYTE(data);
  writeTxOnly2(port_num, id, address, 2);
}
void write2ByteTxRx2(int port_num, uint8_t id, uint16_t address, uint16_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 2 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = DXL_LOBYTE(data);
  packetData[port_num].data_write[1] = DXL_HIBYTE(data);
  writeTxRx2(port_num, id, address, 2);
}

void write4ByteTxOnly2(int port_num, uint8_t id, uint16_t address, uint32_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 4 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = DXL_LOBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[1] = DXL_HIBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[2] = DXL_LOBYTE(DXL_HIWORD(data));
  packetData[port_num].data_write[3] = DXL_HIBYTE(DXL_HIWORD(data));
  writeTxOnly2(port_num, id, address, 4);
}
void write4ByteTxRx2(int port_num, uint8_t id, uint16_t address, uint32_t data)
{
  packetData[port_num].data_write = (uint8_t *)realloc(packetData[port_num].data_write, 4 * sizeof(uint8_t));
  packetData[port_num].data_write[0] = DXL_LOBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[1] = DXL_HIBYTE(DXL_LOWORD(data));
  packetData[port_num].data_write[2] = DXL_LOBYTE(DXL_HIWORD(data));
  packetData[port_num].data_write[3] = DXL_HIBYTE(DXL_HIWORD(data));
  writeTxRx2(port_num, id, address, 4);
}

void regWriteTxOnly2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 12);

  packetData[port_num].tx_packet[PKT_ID_2] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = DXL_LOBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = DXL_HIBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_REG_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 1] = (uint8_t)DXL_HIBYTE(address);

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 2 + s] = packetData[port_num].data_write[s];
  }

  txPacket2(port_num);
  g_is_using[port_num] = False;
}

void regWriteTxRx2(int port_num, uint8_t id, uint16_t address, uint16_t length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, length + 12);
  packetData[port_num].rx_packet = (uint8_t *)realloc(packetData[port_num].rx_packet, 11);

  packetData[port_num].tx_packet[PKT_ID_2] = id;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = DXL_LOBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = DXL_HIBYTE(length + 5);
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_REG_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 0] = (uint8_t)DXL_LOBYTE(address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 1] = (uint8_t)DXL_HIBYTE(address);

  for (s = 0; s < length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 2 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket2(port_num);
}

void syncReadTx2(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 14);
  // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H

  packetData[port_num].tx_packet[PKT_ID_2] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = DXL_LOBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = DXL_HIBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_SYNC_READ;
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 0] = DXL_LOBYTE(start_address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 1] = DXL_HIBYTE(start_address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 2] = DXL_LOBYTE(data_length);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 3] = DXL_HIBYTE(data_length);

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 4 + s] = packetData[port_num].data_write[s];
  }

  txPacket2(port_num);

  if (packetData[port_num].communication_result == COMM_SUCCESS)
    setPacketTimeout(port_num, (uint16_t)((11 + data_length) * param_length));
}

void syncWriteTxOnly2(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 14);
  // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H

  packetData[port_num].tx_packet[PKT_ID_2] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = DXL_LOBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = DXL_HIBYTE(param_length + 7); // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_SYNC_WRITE;
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 0] = DXL_LOBYTE(start_address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 1] = DXL_HIBYTE(start_address);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 2] = DXL_LOBYTE(data_length);
  packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 3] = DXL_HIBYTE(data_length);

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0_2 + 4 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket2(port_num);
}

void bulkReadTx2(int port_num, uint16_t param_length)
{
  uint8_t s;
  int i;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 10);
  // 10: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST CRC16_L CRC16_H

  packetData[port_num].tx_packet[PKT_ID_2] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = DXL_LOBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = DXL_HIBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_BULK_READ;

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0_2 + s] = packetData[port_num].data_write[s];
  }

  txPacket2(port_num);
  if (packetData[port_num].communication_result == COMM_SUCCESS)
  {
    int wait_length = 0;
    for (i = 0; i < param_length; i += 5)
    {
      wait_length += DXL_MAKEWORD(packetData[port_num].data_write[i + 3], packetData[port_num].data_write[i + 4]) + 10;
    }
    setPacketTimeout(port_num, (uint16_t)wait_length);
  }
}

void bulkWriteTxOnly2(int port_num, uint16_t param_length)
{
  uint8_t s;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  packetData[port_num].tx_packet = (uint8_t *)realloc(packetData[port_num].tx_packet, param_length + 10);
  // 10: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST CRC16_L CRC16_H

  packetData[port_num].tx_packet[PKT_ID_2] = BROADCAST_ID;
  packetData[port_num].tx_packet[PKT_LENGTH_L_2] = DXL_LOBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_LENGTH_H_2] = DXL_HIBYTE(param_length + 3); // 3: INST CRC16_L CRC16_H
  packetData[port_num].tx_packet[PKT_INSTRUCTION_2] = INST_BULK_WRITE;

  for (s = 0; s < param_length; s++)
  {
    packetData[port_num].tx_packet[PKT_PARAMETER0_2 + s] = packetData[port_num].data_write[s];
  }

  txRxPacket2(port_num);
}

/*
 * 
 * group_bulk_read.c
 * 
 * Bulk read handling functions.
 * 
 */
 
 static int sizeR(int group_num)
{
  int data_num;
  int real_size = 0;

  for (data_num = 0; data_num < groupDataR[group_num].data_list_length; data_num++)
  {
    if (groupDataR[group_num].data_list[data_num].id != NOT_USED_ID)
      real_size++;
  }
  return real_size;
}

static int findR(int group_num, int id)
{
  int data_num;

  for (data_num = 0; data_num < groupDataR[group_num].data_list_length; data_num++)
  {
    if (groupDataR[group_num].data_list[data_num].id == id)
      break;
  }

  return data_num;
}

int groupBulkRead(int port_num, int protocol_version)
{
  int group_num = 0;

  if (g_used_group_num_r != 0)
  {
    for (group_num = 0; group_num < g_used_group_num_r; group_num++)
    {
      if (groupDataR[group_num].is_param_changed != True)
        break;
    }
  }

  if (group_num == g_used_group_num_r)
  {
    g_used_group_num_r++;
    groupDataR = (GroupDataR *)realloc(groupDataR, g_used_group_num_r * sizeof(GroupDataR));
  }

  groupDataR[group_num].port_num = port_num;
  groupDataR[group_num].protocol_version = protocol_version;
  groupDataR[group_num].data_list_length = 0;
  groupDataR[group_num].last_result = False;
  groupDataR[group_num].is_param_changed = False;
  groupDataR[group_num].data_list = 0;

  groupBulkReadClearParam(group_num);

  return group_num;
}

void groupBulkReadMakeParam(int group_num)
{
  int data_num, idx;
  int port_num = groupDataR[group_num].port_num;

  if (sizeR(group_num) == 0)
    return;

  if (groupDataR[group_num].protocol_version == 1)
  {
    packetData[port_num].data_write = (uint8_t*)realloc(packetData[port_num].data_write, sizeR(group_num) * sizeof(uint8_t) * 3); // ID(1) + ADDR(1) + LENGTH(1)
  }
  else    // 2.0
  {
    packetData[port_num].data_write = (uint8_t*)realloc(packetData[port_num].data_write, sizeR(group_num) * sizeof(uint8_t) * 5); // ID(1) + ADDR(2) + LENGTH(2)
  }

  idx = 0;
  for (data_num = 0; data_num < groupDataR[group_num].data_list_length; data_num++)
  {
    if (groupDataR[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

    if (groupDataR[group_num].protocol_version == 1)
    {
      packetData[port_num].data_write[idx++] = (uint8_t)groupDataR[group_num].data_list[data_num].data_length;       // LEN
      packetData[port_num].data_write[idx++] = groupDataR[group_num].data_list[data_num].id;                         // ID
      packetData[port_num].data_write[idx++] = (uint8_t)groupDataR[group_num].data_list[data_num].start_address;     // ADDR
    }
    else    // 2.0
    {
      packetData[port_num].data_write[idx++] = groupDataR[group_num].data_list[data_num].id;                         // ID
      packetData[port_num].data_write[idx++] = DXL_LOBYTE(groupDataR[group_num].data_list[data_num].start_address);  // ADDR_L
      packetData[port_num].data_write[idx++] = DXL_HIBYTE(groupDataR[group_num].data_list[data_num].start_address);  // ADDR_H
      packetData[port_num].data_write[idx++] = DXL_LOBYTE(groupDataR[group_num].data_list[data_num].data_length);    // LEN_L
      packetData[port_num].data_write[idx++] = DXL_HIBYTE(groupDataR[group_num].data_list[data_num].data_length);    // LEN_H
    }
  }
}

uint8_t groupBulkReadAddParam(int group_num, uint8_t id, uint16_t start_address, uint16_t data_length)
{
  int data_num = 0;

  if (id == NOT_USED_ID)
    return False;

  if (groupDataR[group_num].data_list_length != 0)
    data_num = findR(group_num, id);

  if (groupDataR[group_num].data_list_length == data_num)
  {
    groupDataR[group_num].data_list_length++;
    groupDataR[group_num].data_list = (DataListR *)realloc(groupDataR[group_num].data_list, groupDataR[group_num].data_list_length * sizeof(DataListR));

    groupDataR[group_num].data_list[data_num].id = id;
    groupDataR[group_num].data_list[data_num].data_length = data_length;
    groupDataR[group_num].data_list[data_num].start_address = start_address;
    groupDataR[group_num].data_list[data_num].data = (uint8_t *)calloc(groupDataR[group_num].data_list[data_num].data_length, sizeof(uint8_t));
  }

  groupDataR[group_num].is_param_changed = True;
  return True;
}

void groupBulkReadRemoveParam(int group_num, uint8_t id)
{
  int data_num = findR(group_num, id);

  if (groupDataR[group_num].data_list[data_num].id == NOT_USED_ID)  // NOT exist
    return;

  groupDataR[group_num].data_list[data_num].data = 0;

  groupDataR[group_num].data_list[data_num].id = NOT_USED_ID;
  groupDataR[group_num].data_list[data_num].data_length = 0;
  groupDataR[group_num].data_list[data_num].start_address = 0;

  groupDataR[group_num].is_param_changed = True;
}

void groupBulkReadClearParam(int group_num)
{
  int port_num = groupDataR[group_num].port_num;

  if (sizeR(group_num) == 0)
    return;

  groupDataR[group_num].data_list = 0;

  packetData[port_num].data_write = 0;

  groupDataR[group_num].data_list_length = 0;
  
  groupDataR[group_num].is_param_changed = False;
}

void groupBulkReadTxPacket(int group_num)
{
  int port_num = groupDataR[group_num].port_num;

  if (sizeR(group_num) == 0)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  if (groupDataR[group_num].is_param_changed == True)
    groupBulkReadMakeParam(group_num);

  if (groupDataR[group_num].protocol_version == 1)
  {
    bulkReadTx(groupDataR[group_num].port_num, groupDataR[group_num].protocol_version, sizeR(group_num) * 3);
  }
  else
  {
    bulkReadTx(groupDataR[group_num].port_num, groupDataR[group_num].protocol_version, sizeR(group_num) * 5);
  }
}

void groupBulkReadRxPacket(int group_num)
{
  int data_num, c;
  int port_num = groupDataR[group_num].port_num;

  packetData[port_num].communication_result = COMM_RX_FAIL;

  groupDataR[group_num].last_result = False;

  if (sizeR(group_num) == 0)
  {
    packetData[groupDataR[group_num].port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  for (data_num = 0; data_num < groupDataR[group_num].data_list_length; data_num++)
  {
    if (groupDataR[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

    packetData[port_num].data_read
      = (uint8_t *)realloc(packetData[port_num].data_read, groupDataR[group_num].data_list[data_num].data_length * sizeof(uint8_t));

    readRx(groupDataR[group_num].port_num, groupDataR[group_num].protocol_version, groupDataR[group_num].data_list[data_num].data_length);
    if (packetData[groupDataR[group_num].port_num].communication_result != COMM_SUCCESS)
      return;

    for (c = 0; c < groupDataR[group_num].data_list[data_num].data_length; c++)
    {
      groupDataR[group_num].data_list[data_num].data[c] = packetData[port_num].data_read[c];
    }
  }

  if (packetData[port_num].communication_result == COMM_SUCCESS)
    groupDataR[group_num].last_result = True;
}

void groupBulkReadTxRxPacket(int group_num)
{
  int port_num = groupDataR[group_num].port_num;

  packetData[port_num].communication_result = COMM_TX_FAIL;

  groupBulkReadTxPacket(group_num);
  if (packetData[port_num].communication_result != COMM_SUCCESS)
    return;

  groupBulkReadRxPacket(group_num);
}

uint8_t groupBulkReadIsAvailable(int group_num, uint8_t id, uint16_t address, uint16_t data_length)
{
  int data_num = findR(group_num, id);
  uint16_t start_addr;

  if (groupDataR[group_num].last_result == False || groupDataR[group_num].data_list[data_num].id == NOT_USED_ID)
    return False;

  start_addr = groupDataR[group_num].data_list[data_num].start_address;

  if (address < start_addr || start_addr + groupDataR[group_num].data_list[data_num].data_length - data_length < address)
    return False;

  return True;
}

uint32_t groupBulkReadGetData(int group_num, uint8_t id, uint16_t address, uint16_t data_length)
{
  int data_num = findR(group_num, id);

  if (groupBulkReadIsAvailable(group_num, id, address, data_length) == False)
    return 0;

  uint16_t start_addr = groupDataR[group_num].data_list[data_num].start_address;

  switch (data_length)
  {
    case 1:
      return groupDataR[group_num].data_list[data_num].data[address - start_addr];

    case 2:
      return DXL_MAKEWORD(groupDataR[group_num].data_list[data_num].data[address - start_addr], groupDataR[group_num].data_list[data_num].data[address - start_addr + 1]);

    case 4:
      return DXL_MAKEDWORD(DXL_MAKEWORD(groupDataR[group_num].data_list[data_num].data[address - start_addr + 0], groupDataR[group_num].data_list[data_num].data[address - start_addr + 1]),
        DXL_MAKEWORD(groupDataR[group_num].data_list[data_num].data[address - start_addr + 2], groupDataR[group_num].data_list[data_num].data[address - start_addr + 3]));

    default:
      return 0;
  }
}

/*
 * 
 * group_bulk_write.c
 * 
 * Bulk write handling functions.
 * 
 */

static int sizeW(int group_num)
{
  int data_num;
  int real_size = 0;

  for (data_num = 0; data_num < groupDataW[group_num].data_list_length; data_num++)
  {
    if (groupDataW[group_num].data_list[data_num].id != NOT_USED_ID)
      real_size++;
  }
  return real_size;
}

static int findW(int group_num, int id)
{
  int data_num;

  for (data_num = 0; data_num < groupDataW[group_num].data_list_length; data_num++)
  {
    if (groupDataW[group_num].data_list[data_num].id == id)
      break;
  }

  return data_num;
}

int groupBulkWrite(int port_num, int protocol_version)
{
  int group_num = 0;

  if (g_used_group_num_w != 0)
  {
    for (group_num = 0; group_num < g_used_group_num_w; group_num++)
    {
      if (groupDataW[group_num].is_param_changed != True)
        break;
    }
  }

  if (group_num == g_used_group_num_w)
  {
    g_used_group_num_w++;
    groupDataW = (GroupDataW *)realloc(groupDataW, g_used_group_num_w * sizeof(GroupDataW));
  }

  groupDataW[group_num].port_num = port_num;
  groupDataW[group_num].protocol_version = protocol_version;
  groupDataW[group_num].data_list_length = 0;
  groupDataW[group_num].is_param_changed = False;
  groupDataW[group_num].param_length = 0;
  groupDataW[group_num].data_list = 0;

  groupBulkWriteClearParam(group_num);

  return group_num;
}

void groupBulkWriteMakeParam(int group_num)
{
  int data_num, idx, c;
  int port_num = groupDataW[group_num].port_num;

  if (groupDataW[group_num].protocol_version == 1)
    return;

  if (sizeW(group_num) == 0)
    return;

  groupDataW[group_num].param_length = 0;

  idx = 0;
  for (data_num = 0; data_num < groupDataW[group_num].data_list_length; data_num++)
  {
    if (groupDataW[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

    groupDataW[group_num].param_length += 1 + 2 + 2 + groupDataW[group_num].data_list[data_num].data_length;

    packetData[port_num].data_write = (uint8_t*)realloc(packetData[port_num].data_write, groupDataW[group_num].param_length * sizeof(uint8_t));

    packetData[port_num].data_write[idx++] = groupDataW[group_num].data_list[data_num].id;
    packetData[port_num].data_write[idx++] = DXL_LOBYTE(groupDataW[group_num].data_list[data_num].start_address);
    packetData[port_num].data_write[idx++] = DXL_HIBYTE(groupDataW[group_num].data_list[data_num].start_address);
    packetData[port_num].data_write[idx++] = DXL_LOBYTE(groupDataW[group_num].data_list[data_num].data_length);
    packetData[port_num].data_write[idx++] = DXL_HIBYTE(groupDataW[group_num].data_list[data_num].data_length);

    for (c = 0; c < groupDataW[group_num].data_list[data_num].data_length; c++)
    {
      packetData[port_num].data_write[idx++] = groupDataW[group_num].data_list[data_num].data[c];
    }
  }
}

uint8_t groupBulkWriteAddParam(int group_num, uint8_t id, uint16_t start_address, uint16_t data_length, uint32_t data, uint16_t input_length)
{
  int data_num = 0;

  if (groupDataW[group_num].protocol_version == 1)
    return False;

  if (id == NOT_USED_ID)
    return False;

  if (groupDataW[group_num].data_list_length != 0)
    data_num = findW(group_num, id);

  if (groupDataW[group_num].data_list_length == data_num)
  {
    groupDataW[group_num].data_list_length++;
    groupDataW[group_num].data_list = (DataListW *)realloc(groupDataW[group_num].data_list, groupDataW[group_num].data_list_length * sizeof(DataListW));

    groupDataW[group_num].data_list[data_num].id = id;
    groupDataW[group_num].data_list[data_num].data_length = data_length;
    groupDataW[group_num].data_list[data_num].start_address = start_address;
    groupDataW[group_num].data_list[data_num].data = (uint8_t *)calloc(groupDataW[group_num].data_list[data_num].data_length, sizeof(uint8_t));
    groupDataW[group_num].data_list[data_num].data_end = 0;
  }
  else
  {
    if (groupDataW[group_num].data_list[data_num].data_end + input_length > groupDataW[group_num].data_list[data_num].data_length)
      return False;
  }

  switch (input_length)
  {
    case 1:
      groupDataW[group_num].data_list[data_num].data[groupDataW[group_num].data_list[data_num].data_end + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      break;

    case 2:
      groupDataW[group_num].data_list[data_num].data[groupDataW[group_num].data_list[data_num].data_end + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupDataW[group_num].data_list[data_num].data[groupDataW[group_num].data_list[data_num].data_end + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      break;

    case 4:
      groupDataW[group_num].data_list[data_num].data[groupDataW[group_num].data_list[data_num].data_end + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupDataW[group_num].data_list[data_num].data[groupDataW[group_num].data_list[data_num].data_end + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      groupDataW[group_num].data_list[data_num].data[groupDataW[group_num].data_list[data_num].data_end + 2] = DXL_LOBYTE(DXL_HIWORD(data));
      groupDataW[group_num].data_list[data_num].data[groupDataW[group_num].data_list[data_num].data_end + 3] = DXL_HIBYTE(DXL_HIWORD(data));
      break;

    default:
      return False;
  }
  groupDataW[group_num].data_list[data_num].data_end = input_length;

  groupDataW[group_num].is_param_changed = True;
  return True;
}
void groupBulkWriteRemoveParam(int group_num, uint8_t id)
{
  int data_num = findW(group_num, id);

  if (groupDataW[group_num].protocol_version == 1)
    return;

  if (data_num == groupDataW[group_num].data_list_length)
    return;

  if (groupDataW[group_num].data_list[data_num].id == NOT_USED_ID)  // NOT exist
    return;

  groupDataW[group_num].data_list[data_num].data_end = 0;

  groupDataW[group_num].data_list[data_num].data = 0;

  groupDataW[group_num].data_list[data_num].data_length = 0;
  groupDataW[group_num].data_list[data_num].start_address = 0;
  groupDataW[group_num].data_list[data_num].id = NOT_USED_ID;

  groupDataW[group_num].is_param_changed = True;
}

uint8_t groupBulkWriteChangeParam(int group_num, uint8_t id, uint16_t start_address, uint16_t data_length, uint32_t data, uint16_t input_length, uint16_t data_pos)
{
  int data_num = findW(group_num, id);

  if (groupDataW[group_num].protocol_version == 1)
    return False;

  if (id == NOT_USED_ID)
    return False;

  if (data_num == groupDataW[group_num].data_list_length)
    return False;

  if (data_pos + input_length > groupDataW[group_num].data_list[data_num].data_length)
    return False;

  groupDataW[group_num].data_list[data_num].data_length = data_length;
  groupDataW[group_num].data_list[data_num].start_address = start_address;

  switch (input_length)
  {
    case 1:
      groupDataW[group_num].data_list[data_num].data[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      break;

    case 2:
      groupDataW[group_num].data_list[data_num].data[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupDataW[group_num].data_list[data_num].data[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      break;

    case 4:
      groupDataW[group_num].data_list[data_num].data[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupDataW[group_num].data_list[data_num].data[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      groupDataW[group_num].data_list[data_num].data[data_pos + 2] = DXL_LOBYTE(DXL_HIWORD(data));
      groupDataW[group_num].data_list[data_num].data[data_pos + 3] = DXL_HIBYTE(DXL_HIWORD(data));
      break;

    default:
      return False;
  }

  groupDataW[group_num].is_param_changed = True;
  return True;
}
void groupBulkWriteClearParam(int group_num)
{
  int port_num = groupDataW[group_num].port_num;

  if (groupDataW[group_num].protocol_version == 1)
    return;

  if (sizeW(group_num) == 0)
    return;

  groupDataW[group_num].data_list = 0;

  packetData[port_num].data_write = 0;

  groupDataW[group_num].data_list_length = 0;
  
  groupDataW[group_num].is_param_changed = False;
}
void groupBulkWriteTxPacket(int group_num)
{
  if (groupDataW[group_num].protocol_version == 1)
  {
    packetData[groupDataW[group_num].port_num].communication_result =  COMM_NOT_AVAILABLE;
    return;
  }

  if (sizeW(group_num) == 0)
  {
    packetData[groupDataW[group_num].port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  if (groupDataW[group_num].is_param_changed == True)
    groupBulkWriteMakeParam(group_num);

  bulkWriteTxOnly(groupDataW[group_num].port_num, groupDataW[group_num].protocol_version, groupDataW[group_num].param_length);
}

/*
 * 
 * group_sync_read.c
 * 
 * Sync read handling functions.
 * 
 */

static int sizeSR(int group_num)
{
  int data_num;
  int real_size = 0;

  for (data_num = 0; data_num < groupDataSR[group_num].data_list_length; data_num++)
  {
    if (groupDataSR[group_num].data_list[data_num].id != NOT_USED_ID)
      real_size++;
  }
  return real_size;
};

static int findSR(int group_num, int id)
{
  int data_num;

  for (data_num = 0; data_num < groupDataSR[group_num].data_list_length; data_num++)
  {
    if (groupDataSR[group_num].data_list[data_num].id == id)
      break;
  }

  return data_num;
}

int groupSyncRead(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length)
{
  int group_num = 0;

  if (g_used_group_num_sr != 0)
  {
    for (group_num = 0; group_num < g_used_group_num_sr; group_num++)
    {
      if (groupDataSR[group_num].is_param_changed != True)
        break;
    }
  }

  if (group_num == g_used_group_num_sr)
  {
    g_used_group_num_sr++;
    groupDataSR = (GroupDataSR *)realloc(groupDataSR, g_used_group_num_sr * sizeof(GroupDataSR));
  }

  groupDataSR[group_num].port_num = port_num;
  groupDataSR[group_num].protocol_version = protocol_version;
  groupDataSR[group_num].data_list_length = 0;
  groupDataSR[group_num].last_result = False;
  groupDataSR[group_num].is_param_changed = False;
  groupDataSR[group_num].start_address = start_address;
  groupDataSR[group_num].data_length = data_length;
  groupDataSR[group_num].data_list = 0;

  groupSyncReadClearParam(group_num);

  return group_num;
}

void groupSyncReadMakeParam(int group_num)
{
  int data_num, idx;
  int port_num = groupDataSR[group_num].port_num;

  if (groupDataSR[group_num].protocol_version == 1)
    return;

  if (sizeSR(group_num) == 0)
    return;

  packetData[port_num].data_write = (uint8_t*)realloc(packetData[port_num].data_write, sizeSR(group_num) * (1) * sizeof(uint8_t)); // ID(1)

  idx = 0;
  for (data_num = 0; data_num < groupDataSR[group_num].data_list_length; data_num++)
  {
    if (groupDataSR[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

    packetData[port_num].data_write[idx++] = groupDataSR[group_num].data_list[data_num].id;
  }
}

uint8_t groupSyncReadAddParam(int group_num, uint8_t id)
{
  int data_num = 0;

  if (groupDataSR[group_num].protocol_version == 1)
    return False;

  if (id == NOT_USED_ID)
    return False;

  if (groupDataSR[group_num].data_list_length != 0)
    data_num = findSR(group_num, id);

  if (groupDataSR[group_num].data_list_length == data_num)
  {
    groupDataSR[group_num].data_list_length++;
    groupDataSR[group_num].data_list = (DataListSR *)realloc(groupDataSR[group_num].data_list, groupDataSR[group_num].data_list_length * sizeof(DataListSR));

    groupDataSR[group_num].data_list[data_num].id = id;
    groupDataSR[group_num].data_list[data_num].data = (uint8_t *)calloc(groupDataSR[group_num].data_length, sizeof(uint8_t));
  }

  groupDataSR[group_num].is_param_changed = True;
  return True;
}
void groupSyncReadRemoveParam(int group_num, uint8_t id)
{
  int data_num = findSR(group_num, id);

  if (groupDataSR[group_num].protocol_version == 1)
    return;

  if (groupDataSR[group_num].data_list[data_num].id == NOT_USED_ID)  // NOT exist
    return;

  groupDataSR[group_num].data_list[data_num].data = 0;

  groupDataSR[group_num].data_list[data_num].id = NOT_USED_ID;

  groupDataSR[group_num].is_param_changed = True;
}
void groupSyncReadClearParam(int group_num)
{
  int port_num = groupDataSR[group_num].port_num;

  if (groupDataSR[group_num].protocol_version == 1)
    return;

  if (sizeSR(group_num) == 0)
    return;

  groupDataSR[group_num].data_list = 0;

  packetData[port_num].data_write = 0;

  groupDataSR[group_num].data_list_length = 0;
  
  groupDataSR[group_num].is_param_changed = False;
}

void groupSyncReadTxPacket(int group_num)
{
  int port_num = groupDataSR[group_num].port_num;

  if (groupDataSR[group_num].protocol_version == 1)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  if (sizeSR(group_num) == 0)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  if (groupDataSR[group_num].is_param_changed == True)
    groupSyncReadMakeParam(group_num);

  syncReadTx(groupDataSR[group_num].port_num
    , groupDataSR[group_num].protocol_version
    , groupDataSR[group_num].start_address
    , groupDataSR[group_num].data_length
    , (sizeSR(group_num) * 1));
}

void groupSyncReadRxPacket(int group_num)
{
  int data_num, c;
  int port_num = groupDataSR[group_num].port_num;

  groupDataSR[group_num].last_result = False;

  if (groupDataSR[group_num].protocol_version == 1)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[groupDataSR[group_num].port_num].communication_result = COMM_RX_FAIL;

  if (sizeSR(group_num) == 0)
  {
    packetData[groupDataSR[group_num].port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  for (data_num = 0; data_num < groupDataSR[group_num].data_list_length; data_num++)
  {
    if (groupDataSR[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

      packetData[port_num].data_read
        = (uint8_t *)realloc(packetData[port_num].data_read, groupDataSR[group_num].data_length * sizeof(uint8_t));

      readRx(groupDataSR[group_num].port_num, groupDataSR[group_num].protocol_version, groupDataSR[group_num].data_length);
      if (packetData[port_num].communication_result != COMM_SUCCESS)
        return;

      for (c = 0; c < groupDataSR[group_num].data_length; c++)
        groupDataSR[group_num].data_list[data_num].data[c] = packetData[port_num].data_read[c];
  }

  if (packetData[port_num].communication_result == COMM_SUCCESS)
    groupDataSR[group_num].last_result = True;
}

void groupSyncReadTxRxPacket(int group_num)
{
  int port_num = groupDataSR[group_num].port_num;

  if (groupDataSR[group_num].protocol_version == 1)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[port_num].communication_result = COMM_TX_FAIL;

  groupSyncReadTxPacket(group_num);
  if (packetData[port_num].communication_result != COMM_SUCCESS)
    return;

  groupSyncReadRxPacket(group_num);
}

uint8_t groupSyncReadIsAvailable(int group_num, uint8_t id, uint16_t address, uint16_t data_length)
{
  int data_num = findSR(group_num, id);

  if (groupDataSR[group_num].protocol_version == 1 || groupDataSR[group_num].last_result == False || groupDataSR[group_num].data_list[data_num].id == NOT_USED_ID)
    return False;

  if (address < groupDataSR[group_num].start_address || groupDataSR[group_num].start_address + groupDataSR[group_num].data_length - data_length < address) {
    return False;
  }
  return True;
}

uint32_t groupSyncReadGetData(int group_num, uint8_t id, uint16_t address, uint16_t data_length)
{
  int data_num = findSR(group_num, id);

  if (groupSyncReadIsAvailable(group_num, id, address, data_length) == False)
    return 0;

  switch (data_length)
  {
    case 1:
      return groupDataSR[group_num].data_list[data_num].data[address - groupDataSR[group_num].start_address];

    case 2:
      return DXL_MAKEWORD(groupDataSR[group_num].data_list[data_num].data[address - groupDataSR[group_num].start_address], groupDataSR[group_num].data_list[data_num].data[address - groupDataSR[group_num].start_address + 1]);

    case 4:
      return DXL_MAKEDWORD(
        DXL_MAKEWORD(
        groupDataSR[group_num].data_list[data_num].data[address - groupDataSR[group_num].start_address + 0]
        , groupDataSR[group_num].data_list[data_num].data[address - groupDataSR[group_num].start_address + 1])
        , DXL_MAKEWORD(
          groupDataSR[group_num].data_list[data_num].data[address - groupDataSR[group_num].start_address + 2]
          , groupDataSR[group_num].data_list[data_num].data[address - groupDataSR[group_num].start_address + 3])
      );

    default:
      return 0;
  }
}

/*
 * 
 * group_sync_write.c
 * 
 * Sync write handling functions.
 * 
 */

int sizeSW(int group_num)
{
  int data_num;
  int real_size = 0;

  for (data_num = 0; data_num < groupDataSW[group_num].data_list_length; data_num++)
  {
    if (groupDataSW[group_num].data_list[data_num].id != NOT_USED_ID)
      real_size++;
  }
  return real_size;
}

int findSW(int group_num, int id)
{
  int data_num;

  for (data_num = 0; data_num < groupDataSW[group_num].data_list_length; data_num++)
  {
    if (groupDataSW[group_num].data_list[data_num].id == id)
      break;
  }

  return data_num;
}

int groupSyncWrite(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length)
{
  int group_num = 0;

  if (g_used_group_num_sw != 0)
  {
    for (group_num = 0; group_num < g_used_group_num_sw; group_num++)
    {
      if (groupDataSW[group_num].is_param_changed != True)
        break;
    }
  }

  if (group_num == g_used_group_num_sw)
  {
    g_used_group_num_sw++;
    groupDataSW = (GroupDataSW *)realloc(groupDataSW, g_used_group_num_sw * sizeof(GroupDataSW));
  }

  groupDataSW[group_num].port_num = port_num;
  groupDataSW[group_num].protocol_version = protocol_version;
  groupDataSW[group_num].data_list_length = 0;
  groupDataSW[group_num].is_param_changed = False;
  groupDataSW[group_num].start_address = start_address;
  groupDataSW[group_num].data_length = data_length;
  groupDataSW[group_num].data_list = 0;

  groupSyncWriteClearParam(group_num);

  return group_num;
}

void groupSyncWriteMakeParam(int group_num)
{
  int data_num, c, idx;
  int port_num = groupDataSW[group_num].port_num;

  if (sizeSW(group_num) == 0)
    return;

  packetData[port_num].data_write = (uint8_t*)realloc(packetData[port_num].data_write, sizeSW(group_num) * (1 + groupDataSW[group_num].data_length) * sizeof(uint8_t)); // ID(1) + DATA(data_length)

  idx = 0;
  for (data_num = 0; data_num < groupDataSW[group_num].data_list_length; data_num++)
  {
    if (groupDataSW[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

    packetData[port_num].data_write[idx++] = groupDataSW[group_num].data_list[data_num].id;
    for (c = 0; c < groupDataSW[group_num].data_length; c++)
    {
      packetData[port_num].data_write[idx++] = groupDataSW[group_num].data_list[data_num].data[c];
    }
  }
}

uint8_t groupSyncWriteAddParam(int group_num, uint8_t id, uint32_t data, uint16_t input_length)
{
  int data_num = 0;

  if (id == NOT_USED_ID)
    return False;

  if (groupDataSW[group_num].data_list_length != 0)
    data_num = findSW(group_num, id);

  if (groupDataSW[group_num].data_list_length == data_num)
  {
    groupDataSW[group_num].data_list_length++;
    groupDataSW[group_num].data_list = (DataListSW *)realloc(groupDataSW[group_num].data_list, groupDataSW[group_num].data_list_length * sizeof(DataListSW));

    groupDataSW[group_num].data_list[data_num].id = id;
    groupDataSW[group_num].data_list[data_num].data = (uint8_t *)calloc(groupDataSW[group_num].data_length, sizeof(uint8_t));
    groupDataSW[group_num].data_list[data_num].data_end = 0;
  }
  else
  {
    if (groupDataSW[group_num].data_list[data_num].data_end + input_length > groupDataSW[group_num].data_length)
      return False;
  }

  switch (input_length)
  {
    case 1:
      groupDataSW[group_num].data_list[data_num].data[groupDataSW[group_num].data_list[data_num].data_end + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      break;

    case 2:
      groupDataSW[group_num].data_list[data_num].data[groupDataSW[group_num].data_list[data_num].data_end + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupDataSW[group_num].data_list[data_num].data[groupDataSW[group_num].data_list[data_num].data_end + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      break;

    case 4:
      groupDataSW[group_num].data_list[data_num].data[groupDataSW[group_num].data_list[data_num].data_end + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupDataSW[group_num].data_list[data_num].data[groupDataSW[group_num].data_list[data_num].data_end + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      groupDataSW[group_num].data_list[data_num].data[groupDataSW[group_num].data_list[data_num].data_end + 2] = DXL_LOBYTE(DXL_HIWORD(data));
      groupDataSW[group_num].data_list[data_num].data[groupDataSW[group_num].data_list[data_num].data_end + 3] = DXL_HIBYTE(DXL_HIWORD(data));
      break;

    default:
      return False;
  }
  groupDataSW[group_num].data_list[data_num].data_end = input_length;

  groupDataSW[group_num].is_param_changed = True;
  return True;
}

void groupSyncWriteRemoveParam(int group_num, uint8_t id)
{
  int data_num = findSW(group_num, id);

  if (data_num == groupDataSW[group_num].data_list_length)
    return;

  if (groupDataSW[group_num].data_list[data_num].id == NOT_USED_ID)  // NOT exist
    return;

  groupDataSW[group_num].data_list[data_num].data_end = 0;

  groupDataSW[group_num].data_list[data_num].data = 0;

  groupDataSW[group_num].data_list[data_num].id = NOT_USED_ID;

  groupDataSW[group_num].is_param_changed = True;
}

uint8_t groupSyncWriteChangeParam(int group_num, uint8_t id, uint32_t data, uint16_t input_length, uint16_t data_pos)
{
  if (id == NOT_USED_ID)  // NOT exist
    return False;

  int data_num = findSW(group_num, id);

  if (data_num == groupDataSW[group_num].data_list_length)
    return False;

  if (data_pos + input_length > groupDataSW[group_num].data_length)
    return False;

  switch (input_length)
  {
    case 1:
      groupDataSW[group_num].data_list[data_num].data[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      break;

    case 2:
      groupDataSW[group_num].data_list[data_num].data[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupDataSW[group_num].data_list[data_num].data[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      break;

    case 4:
      groupDataSW[group_num].data_list[data_num].data[data_pos + 0] = DXL_LOBYTE(DXL_LOWORD(data));
      groupDataSW[group_num].data_list[data_num].data[data_pos + 1] = DXL_HIBYTE(DXL_LOWORD(data));
      groupDataSW[group_num].data_list[data_num].data[data_pos + 2] = DXL_LOBYTE(DXL_HIWORD(data));
      groupDataSW[group_num].data_list[data_num].data[data_pos + 3] = DXL_HIBYTE(DXL_HIWORD(data));
      break;

    default:
      return False;
  }

  groupDataSW[group_num].is_param_changed = True;
  return True;
}

void groupSyncWriteClearParam(int group_num)
{
  int port_num = groupDataSW[group_num].port_num;

  if (sizeSW(group_num) == 0)
    return;

  groupDataSW[group_num].data_list = 0;

  packetData[port_num].data_write = 0;

  groupDataSW[group_num].data_list_length = 0;
  
  groupDataSW[group_num].is_param_changed = False;
}

void groupSyncWriteTxPacket(int group_num)
{
  int port_num = groupDataSW[group_num].port_num;

  if (sizeSW(group_num) == 0)
  {
  	packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
  	return;
  }

  if (groupDataSW[group_num].is_param_changed == True)
    groupSyncWriteMakeParam(group_num);

	syncWriteTxOnly(
    groupDataSW[group_num].port_num
    , groupDataSW[group_num].protocol_version
    , groupDataSW[group_num].start_address
    , groupDataSW[group_num].data_length
    , sizeSW(group_num) * (1 + groupDataSW[group_num].data_length));
}

/*
 * 
 * getch and kbhit
 * 
 * Helper functions for interactions in console.
 * 
 */
 
int getch()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

/*
 * 
 * main
 * 
 * Protocol 2 ping example.
 * 
 * 1. Adapt PROTOCOL_VERSION, DXL_ID, BAUDRATE and DEVICENAME to your setup.
 * 2. Compile with : gcc -Wall dxl.c -o dxl -lrt
 * 3. Power your Dynamixel and connect the USB2Dynamixel dongle.
 * 4. Start the program with ./dxl
 * 5. Verify that the returned device number corresponds to your Dynamixel.
 * 
 */
 
// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"
// Number of times ping cmd should be issued                                                           
#define NB_LOOP													100

int main()
{
  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(DEVICENAME);

  // Initialize PacketHandler Structs
  packetHandler();

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_model_number;                      // Dynamixel model number
  
  struct timespec before, after;
  unsigned long long ping_delay;
  int i;

  // Open port
  if (openPort(port_num))
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }
	
	for ( i = 0; i < NB_LOOP; i++ )	{
		// Try to ping the Dynamixel
		// Get Dynamixel model number
		clock_gettime( CLOCK_MONOTONIC, &before );
		dxl_model_number = pingGetModelNum(port_num, PROTOCOL_VERSION, DXL_ID);
		clock_gettime( CLOCK_MONOTONIC, &after );
		ping_delay = 	( after.tv_sec - before.tv_sec ) * 1000000 +
						( after.tv_nsec - before.tv_nsec ) / 1000;
						
		if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
		{
			printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
		}
		else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
		{
			printRxPacketError(PROTOCOL_VERSION, dxl_error);
		}

		printf("[ID:%03d] ping Succeeded. Dynamixel model number : %d\n", DXL_ID, dxl_model_number);
		printf("Ping delay: %llu us\n", ping_delay );
	}

  // Close port
  closePort(port_num);

  return 0;
}
