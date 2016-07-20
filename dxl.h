/* External defines */

// Special IDs
#define NOT_USED_ID         						0xFF		// 255
#define BROADCAST_ID        						0xFE    // 254
#define MAX_ID              						0xFC    // 252

// Instruction for DXL Protocol
#define INST_PING           						1
#define INST_READ           						2
#define INST_WRITE          						3
#define INST_REG_WRITE      						4
#define INST_ACTION         						5
#define INST_FACTORY_RESET  						6
#define INST_SYNC_WRITE     						131     // 0x83
#define INST_BULK_READ      						146     // 0x92
// --- Only for 2.0 --- //
#define INST_REBOOT         						8
#define INST_STATUS         						85      // 0x55
#define INST_SYNC_READ      						130     // 0x82
#define INST_BULK_WRITE     						147     // 0x93

// Communication Result
#define COMM_SUCCESS        						0       // tx or rx packet communication success
#define COMM_PORT_BUSY      						-1000   // Port is busy (in use)
#define COMM_TX_FAIL        						-1001   // Failed transmit instruction packet
#define COMM_RX_FAIL        						-1002   // Failed get status packet
#define COMM_TX_ERROR       						-2000   // Incorrect instruction packet
#define COMM_RX_WAITING     						-3000   // Now recieving status packet
#define COMM_RX_TIMEOUT     						-3001   // There is no status packet
#define COMM_RX_CORRUPT     						-3002   // Incorrect status packet
#define COMM_NOT_AVAILABLE  						-9000

/*
 * 
 * port_handler_linux.h
 * 
 * Low-level access functions to Linux serial port.
 * 
 */

int 		portHandlerLinux        	(const char *port_name);

uint8_t setupPortLinux          	(int port_num, const int cflag_baud);
uint8_t setCustomBaudrateLinux  	(int port_num, int speed);
int     getCFlagBaud            	(const int baudrate);

double  getCurrentTimeLinux     	();
double  getTimeSinceStartLinux  	(int port_num);

uint8_t openPortLinux           	(int port_num);
void    closePortLinux          	(int port_num);
void    clearPortLinux          	(int port_num);

void    setPortNameLinux        	(int port_num, const char *port_name);
char   *getPortNameLinux        	(int port_num);

uint8_t setBaudRateLinux        	(int port_num, const int baudrate);
int     getBaudRateLinux        	(int port_num);

int     getBytesAvailableLinux  	(int port_num);

int     readPortLinux           	(int port_num, uint8_t *packet, int length);
int     writePortLinux          	(int port_num, uint8_t *packet, int length);

void    setPacketTimeoutLinux     (int port_num, uint16_t packet_length);
void    setPacketTimeoutMSecLinux (int port_num, double msec);
uint8_t isPacketTimeoutLinux      (int port_num);

/*
 * 
 * port_handler.h
 * 
 * Low-level access functions to Linux serial port.
 * 
 */

int     portHandler             	(const char *port_name);

uint8_t openPort                	(int port_num);
void    closePort               	(int port_num);
void    clearPort               	(int port_num);

void    setPortName             	(int port_num, const char* port_name);
char   *getPortName             	(int port_num);

uint8_t setBaudRate             	(int port_num, const int baudrate);
int     getBaudRate             	(int port_num);

int     getBytesAvailable       	(int port_num);

int     readPort                	(int port_num, uint8_t *packet, int length);
int     writePort               	(int port_num, uint8_t *packet, int length);

void    setPacketTimeout        	(int port_num, uint16_t packet_length);
void    setPacketTimeoutMSec    	(int port_num, double msec);
uint8_t isPacketTimeout         	(int port_num);

/*
 * 
 * packet_handler.h
 * 
 * Packet handling funtions.
 * 
 */

void        packetHandler       	();

void        printTxRxResult     	(int protocol_version, int result);
void        printRxPacketError  	(int protocol_version, uint8_t error);

int         getLastTxRxResult   	(int port_num, int protocol_version);
uint8_t     getLastRxPacketError  (int port_num, int protocol_version);

void        setDataWrite        	(int port_num, int protocol_version, uint16_t data_length, uint16_t data_pos, uint32_t data);
uint32_t    getDataRead         	(int port_num, int protocol_version, uint16_t data_length, uint16_t data_pos);

void        txPacket            	(int port_num, int protocol_version);

void        rxPacket            	(int port_num, int protocol_version);

void        txRxPacket          	(int port_num, int protocol_version);

void        ping                	(int port_num, int protocol_version, uint8_t id);

uint16_t    pingGetModelNum     	(int port_num, int protocol_version, uint8_t id);

// broadcastPing
void        broadcastPing       	(int port_num, int protocol_version);
uint8_t     getBroadcastPingResult  (int port_num, int protocol_version, int id);

void        reboot              	(int port_num, int protocol_version, uint8_t id);

void        factoryReset        	(int port_num, int protocol_version, uint8_t id, uint8_t option);

void        readTx              	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);
void        readRx              	(int port_num, int protocol_version, uint16_t length);
void        readTxRx            	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);

void        read1ByteTx         	(int port_num, int protocol_version, uint8_t id, uint16_t address);
uint8_t     read1ByteRx         	(int port_num, int protocol_version);
uint8_t     read1ByteTxRx       	(int port_num, int protocol_version, uint8_t id, uint16_t address);

void        read2ByteTx         	(int port_num, int protocol_version, uint8_t id, uint16_t address);
uint16_t    read2ByteRx         	(int port_num, int protocol_version);
uint16_t    read2ByteTxRx       	(int port_num, int protocol_version, uint8_t id, uint16_t address);

void        read4ByteTx         	(int port_num, int protocol_version, uint8_t id, uint16_t address);
uint32_t    read4ByteRx         	(int port_num, int protocol_version);
uint32_t    read4ByteTxRx       	(int port_num, int protocol_version, uint8_t id, uint16_t address);

void    writeTxOnly             	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);
void    writeTxRx               	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);

void    write1ByteTxOnly        	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint8_t data);
void    write1ByteTxRx          	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint8_t data);

void    write2ByteTxOnly        	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t data);
void    write2ByteTxRx          	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t data);

void    write4ByteTxOnly        	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint32_t data);
void    write4ByteTxRx          	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint32_t data);

void    regWriteTxOnly          	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);
void    regWriteTxRx            	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);

void    syncReadTx              	(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length, uint16_t param_length);
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

void    syncWriteTxOnly         	(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length, uint16_t param_length);

void    bulkReadTx              	(int port_num, int protocol_version, uint16_t param_length);
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

void    bulkWriteTxOnly         	(int port_num, int protocol_version, uint16_t param_length);

/*
 * 
 * protocol1_packet_handler.h
 * 
 * Protocol 1 packet handling funtions.
 * 
 */

void        printTxRxResult1    	(int result);
void        printRxPacketError1 	(uint8_t error);

int         getLastTxRxResult1  	(int port_num);
uint8_t     getLastRxPacketError1 (int port_num);

void        setDataWrite1       	(int port_num, uint16_t data_length, uint16_t data_pos, uint32_t data);
uint32_t    getDataRead1        	(int port_num, uint16_t data_length, uint16_t data_pos);

void        txPacket1           	(int port_num);
void        rxPacket1           	(int port_num);
void        txRxPacket1         	(int port_num);

void        ping1               	(int port_num, uint8_t id);
uint16_t    pingGetModelNum1    	(int port_num, uint8_t id);

// broadcastPing
void        broadcastPing1      	(int port_num);
uint8_t     getBroadcastPingResult1 (int port_num, int id);

void        action1             	(int port_num, uint8_t id);
void        reboot1             	(int port_num, uint8_t id);
void        factoryReset1       	(int port_num, uint8_t id, uint8_t option);

void        readTx1             	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        readRx1             	(int port_num, uint16_t length);
void        readTxRx1           	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        read1ByteTx1        	(int port_num, uint8_t id, uint16_t address);
uint8_t     read1ByteRx1        	(int port_num);
uint8_t     read1ByteTxRx1      	(int port_num, uint8_t id, uint16_t address);

void        read2ByteTx1        	(int port_num, uint8_t id, uint16_t address);
uint16_t    read2ByteRx1        	(int port_num);
uint16_t    read2ByteTxRx1      	(int port_num, uint8_t id, uint16_t address);

void        read4ByteTx1        	(int port_num, uint8_t id, uint16_t address);
uint32_t    read4ByteRx1        	(int port_num);
uint32_t    read4ByteTxRx1      	(int port_num, uint8_t id, uint16_t address);

void        writeTxOnly1        	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        writeTxRx1          	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        write1ByteTxOnly1   	(int port_num, uint8_t id, uint16_t address, uint8_t data);
void        write1ByteTxRx1     	(int port_num, uint8_t id, uint16_t address, uint8_t data);

void        write2ByteTxOnly1   	(int port_num, uint8_t id, uint16_t address, uint16_t data);
void        write2ByteTxRx1     	(int port_num, uint8_t id, uint16_t address, uint16_t data);

void        write4ByteTxOnly1   	(int port_num, uint8_t id, uint16_t address, uint32_t data);
void        write4ByteTxRx1     	(int port_num, uint8_t id, uint16_t address, uint32_t data);

void        regWriteTxOnly1     	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        regWriteTxRx1       	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        syncReadTx1         	(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

// param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
void        syncWriteTxOnly1    	(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);

// param : LEN1 ID1 ADDR1 LEN2 ID2 ADDR2 ...
void        bulkReadTx1         	(int port_num, uint16_t param_length);
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

// param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
void        bulkWriteTxOnly1    	(int port_num, uint16_t param_length);

/*
 * 
 * protocol2_packet_handler.h
 * 
 * Protocol 2 packet handling funtions.
 * 
 */

uint16_t    updateCRC           	(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
void        addStuffing         	(uint8_t *packet);
void        removeStuffing      	(uint8_t *packet);

void        printTxRxResult2    	(int result);
void        printRxPacketError2   (uint8_t error);

int         getLastTxRxResult2  	(int port_num);
uint8_t     getLastRxPacketError2 (int port_num);

void        setDataWrite2       	(int port_num, uint16_t data_length, uint16_t data_pos, uint32_t data);
uint32_t    getDataRead2        	(int port_num, uint16_t data_length, uint16_t data_pos);

void        txPacket2           	(int port_num);
void        rxPacket2           	(int port_num);
void        txRxPacket2         	(int port_num);

void        ping2               	(int port_num, uint8_t id);
uint16_t    pingGetModelNum2    	(int port_num, uint8_t id);

// BroadcastPing
void        broadcastPing2      	(int port_num);
uint8_t     getBroadcastPingResult2 (int port_num, int id);

void        action2             	(int port_num, uint8_t id);
void        reboot2             	(int port_num, uint8_t id);
void        factoryReset2       	(int port_num, uint8_t id, uint8_t option);

void        readTx2             	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        readRx2             	(int port_num, uint16_t length);
void        readTxRx2           	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        read1ByteTx2        	(int port_num, uint8_t id, uint16_t address);
uint8_t     read1ByteRx2        	(int port_num);
uint8_t     read1ByteTxRx2      	(int port_num, uint8_t id, uint16_t address);

void        read2ByteTx2        	(int port_num, uint8_t id, uint16_t address);
uint16_t    read2ByteRx2        	(int port_num);
uint16_t    read2ByteTxRx2      	(int port_num, uint8_t id, uint16_t address);

void        read4ByteTx2        	(int port_num, uint8_t id, uint16_t address);
uint32_t    read4ByteRx2        	(int port_num);
uint32_t    read4ByteTxRx2      	(int port_num, uint8_t id, uint16_t address);

void        writeTxOnly2        	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        writeTxRx2          	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        write1ByteTxOnly2   	(int port_num, uint8_t id, uint16_t address, uint8_t data);
void        write1ByteTxRx2     	(int port_num, uint8_t id, uint16_t address, uint8_t data);

void        write2ByteTxOnly2   	(int port_num, uint8_t id, uint16_t address, uint16_t data);
void        write2ByteTxRx2     	(int port_num, uint8_t id, uint16_t address, uint16_t data);

void        write4ByteTxOnly2   	(int port_num, uint8_t id, uint16_t address, uint32_t data);
void        write4ByteTxRx2     	(int port_num, uint8_t id, uint16_t address, uint32_t data);

void        regWriteTxOnly2     	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        regWriteTxRx2       	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        syncReadTx2         	(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

// param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
void        syncWriteTxOnly2   		(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);

// param : ID1 ADDR_L1 ADDR_H1 LEN_L1 LEN_H1 ID2 ADDR_L2 ADDR_H2 LEN_L2 LEN_H2 ...
void        bulkReadTx2        		(int port_num, uint16_t param_length);
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

// param : ID1 START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H DATA0 DATA1 ... DATAn ID2 START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H DATA0 DATA1 ... DATAn
void        bulkWriteTxOnly2   		(int port_num, uint16_t param_length);

/*
 * 
 * group_bulk_read.h
 * 
 * Bulk read funtions.
 * 
 */
 
int         groupBulkRead               (int port_num, int protocol_version);

uint8_t     groupBulkReadAddParam       (int group_num, uint8_t id, uint16_t start_address, uint16_t data_length);
void        groupBulkReadRemoveParam    (int group_num, uint8_t id);
void        groupBulkReadClearParam     (int group_num);

void        groupBulkReadTxPacket       (int group_num);
void        groupBulkReadRxPacket       (int group_num);
void        groupBulkReadTxRxPacket     (int group_num);

uint8_t     groupBulkReadIsAvailable    (int group_num, uint8_t id, uint16_t address, uint16_t data_length);
uint32_t    groupBulkReadGetData        (int group_num, uint8_t id, uint16_t address, uint16_t data_length);

/*
 * 
 * group_bulk_write.h
 * 
 * Bulk write funtions.
 * 
 */
 
int     		groupBulkWrite              (int port_num, int protocol_version);

uint8_t 		groupBulkWriteAddParam      (int group_num, uint8_t id, uint16_t start_address, uint16_t data_length, uint32_t data, uint16_t input_length);
void    		groupBulkWriteRemoveParam   (int group_num, uint8_t id);
uint8_t 		groupBulkWriteChangeParam   (int group_num, uint8_t id, uint16_t start_address, uint16_t data_length, uint32_t data, uint16_t input_length, uint16_t data_pos);
void    		groupBulkWriteClearParam    (int group_num);

void    		groupBulkWriteTxPacket      (int group_num);

/*
 * 
 * group_sync_read.h
 * 
 * Sync read funtions.
 * 
 */

int         groupSyncRead               (int port_num, int protocol_version, uint16_t start_address, uint16_t data_length);

uint8_t     groupSyncReadAddParam       (int group_num, uint8_t id);
void        groupSyncReadRemoveParam    (int group_num, uint8_t id);
void        groupSyncReadClearParam     (int group_num);

void        groupSyncReadTxPacket       (int group_num);
void        groupSyncReadRxPacket       (int group_num);
void        groupSyncReadTxRxPacket     (int group_num);

uint8_t     groupSyncReadIsAvailable    (int group_num, uint8_t id, uint16_t address, uint16_t data_length);
uint32_t    groupSyncReadGetData        (int group_num, uint8_t id, uint16_t address, uint16_t data_length);

/*
 * 
 * group_sync_write.h
 * 
 * Sync write funtions.
 * 
 */
 
int     		groupSyncWrite              (int port_num, int protocol_version, uint16_t start_address, uint16_t data_length);

uint8_t 		groupSyncWriteAddParam      (int group_num, uint8_t id, uint32_t data, uint16_t data_length);
void    		groupSyncWriteRemoveParam   (int group_num, uint8_t id);
uint8_t 		groupSyncWriteChangeParam   (int group_num, uint8_t id, uint32_t data, uint16_t data_length, uint16_t data_pos);
void    		groupSyncWriteClearParam    (int group_num);

void    		groupSyncWriteTxPacket      (int group_num);
