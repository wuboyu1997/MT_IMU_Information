#ifndef _DRVERR_H
#define _DRVERR_H


// enum of possible errors
typedef enum
{
	NO_ERR,					// No Error
	POLL_TIMEOUT,			// Polling toggle bit failed
	VERIFY_WRITE,			// Verifying write to flash failed
	INVALID_SECTOR,			// Invalid Sector
	INVALID_BLOCK,			// Invalid Block
	UNKNOWN_COMMAND,		// Unknown Command
	PROCESS_COMMAND_ERR,	// Processing command
	NOT_READ_ERROR,			// Could not read memory from target
	DRV_NOTAT_BREAK,		// The drive was not at AFP_BreakReady
	BUFFER_IS_NULL,			// Could not allocate storage for the buffer
	NO_ACCESS_SECTOR,		// Cannot access the sector( could be locked or something is stored there that should not be touched )
	NUM_ERROR_CODES,
	UART_RX_FULL,			//Lost one byte when the RX circular buffer is full.
	UART_TX_FULL,			//When TX circular buffer is full, push one byte will be lost
	UART_RX_BUF_EMPTY,      //Try to pop one byte from empty RX circular buffer	
	OUT_OF_FLASH_NUM,       //Out of flash number (0-2)
	FLASH_GET_CODE_FAIL,    //Check codes of flash devices failed
	FLASH_COMPARE_ERROR, 	//Compare read/write flash error
	UNDEFINED_URX_STAT,		//Undefined uart rx buf status
	UNDEFINED_INS,			//Undefined instruction
	FAIL_ON_MALLOC,			//Failed on malloc()
	CHECK_SUM_ERR,			//Check sum error, VR101 protocol
	TOO_MUCH_DATA, 			//Too much data to print in a data rate period
	OUT_OF_16BIT_RANGE,		//Out of 16bit range 
	MATRIX_NO_INVERSE,		//Matrix no inverse
	WRITE_BACK_FLASH,        //Try to write to a Flash address that have been written
	SDRAM_TEST_ERR,			//SDRAM self test error
	COMP_SELFCHECK_FAIL,     //Compass self check fail
	UNFINISHED_HMC5883SEQ,    //Unfinished HMC5883 continous read sequence while starting a new one
	SENSOR_SATURATION,       //Sensor saturation
	GYRO_BIT_ERR,			 //Gyro Built-in-test error
	CALIB_MAG2D_LARGE_INCLIN, //too large pitch or roll angle during 2D magnetometer calib
	ACCEL_INITIAL_CHECK_ERR,  //Accel initial check error
	GYRO_INITIAL_CHECK_ERR ,   //gyroscope initial check error
	QUEUE_EMPTY,			  // CirQueue empty error
	QUEUE_FULL,				  // CirQueue full error
	OUT_OF_8BIT_RANGE         //for 203 temperature conversion
	
}ERROR_CODE;


typedef struct 
{
	unsigned int Sec;
	unsigned int OptOnemS;
	ERROR_CODE ErrMsg;
} SingleErrMsg;

#define LOG_ERR_NUMBER 16

typedef struct
{
	unsigned int ptr;
	SingleErrMsg Error[LOG_ERR_NUMBER];
} _ERRORLOG;

#endif

