
#ifndef DDE_CONFIG_H
#define	DDE_CONFIG_H

// DDE
#define DATA_EE_BANKS                     4 // 6
#define DATA_EE_SIZE                      128 // 255
#define DATA_EE_TOTAL_SIZE                (DATA_EE_BANKS * DATA_EE_SIZE)
#define NUM_DATA_EE_PAGES                 2
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE    1024
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW     128
#define DEE_STRUCTURE_FLASH_START_ADDRESS 0x4000  //

// If the device has ECC
#define __HAS_ECC	                      1
#define ERASE_WRITE_CYCLE_MAX             10000
#define NUMBER_OF_ROWS_IN_PAGE            (_FLASH_PAGE / _FLASH_ROW)
#define PAGE_AVAILABLE                    1
#define PAGE_CURRENT                      0
#define PAGE_EXPIRED                      0
#define PAGE_NOT_AVAILABLE                0
#define PAGE_NOT_CURRENT                  1
#define PAGE_NOT_EXPIRED                  1
#define STATUS_AVAILABLE                  18
#define STATUS_CURRENT                    19
#define STATUS_EXPIRED                    20
#define ERASE_STATE                       0xFFFFFF


// DEE map for Device Vamp1K
#define EEA_CONFIG              000 // Config: 94 eeprom's byte to store config
#define EEA_MEAS_COUNTER        100 // Status: meas_counter  
#define EEA_RESET_COUNTER       101 // Status: reset counter / RCON map...
#define EEA_SKEYL               102 // Status: skey LSW
#define EEA_SKEYH               103 // Status: skey MSW
#define EEA_SST26_HA        110
#define EEA_SST26_LA        111


#endif	// DDE_CONFIG

