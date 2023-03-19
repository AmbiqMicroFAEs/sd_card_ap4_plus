//=============================================================================
// SD CARD driver example
// Mariusz Lacina, Ambiq, 2023
//=============================================================================
#ifndef SDIO_H
#define SDIO_H
#include <stdint.h>
#include <stdbool.h>
#include "am_hal_global.h"
#include "am_hal_card.h"
#include "am_hal_card_host.h"
#include "sdhc.h"

//=============================================================================
//
//  SDIO SPEED
//
//=============================================================================
#define SDIO_DEFAULT_SPEED  12500000
#define SDIO_HIGH_SPEED     25000000

//=============================================================================
//
// SDHC card structure
//
//=============================================================================
typedef struct
{
  am_hal_card_type_e  eType;
  am_hal_card_state_e eState;
  uint32_t bCidValid:1;
  uint32_t bCsdValid:1;
  uint32_t bExtCsdValid:1;
  uint32_t ui32OCR;
  uint16_t ui16RCA;
  uint32_t ui32CID[4];
  uint32_t ui32CSD[4];
  uint32_t ui32ExtCSD[128];
  uint8_t  ui8SpecVer;
  uint8_t  ui8ExtCSDRev;
  uint8_t  ui8PowerOffNotification;
  bool     bHighCapcity;
  uint32_t ui32CacheSize;
  uint32_t ui32SleepNotificationTimeout;
  uint32_t ui32PowerOffNotificationLongTimeout;
  uint32_t ui32GenericCmd6Timeout;
  uint32_t ui32MaxBlks;
  uint32_t ui32Capacity;
  uint8_t  ui8SecureErase;
  uint8_t  ui8DeviceType;
  uint32_t ui32BlkNum;
  bool     bUseBlkEmulation;
  uint32_t ui32NativeBlkSize;
  uint32_t ui32BlkSize;
  uint16_t ui16CmdClass;
  am_hal_card_pwr_ctrl_func pCardPwrCtrlFunc;
  am_hal_card_pwr_ctrl_policy_e eCardPwrCtrlPolicy;
}__attribute__((packed))  sd_card_t;


//=============================================================================
//
// SD CARD HOST & CARD SRTUCTURE
//
//=============================================================================
typedef struct{
  sdhc_host_cfg_t      host;
  sd_card_t            card;
}sd_card_host_t;

//=============================================================================
//
// SD Host Register Types
//
//=============================================================================
typedef struct
{
    bool bValid;
    uint32_t regHOSTCTRL1;
    uint32_t regCLOCKCTRL;
    uint32_t regINTENABLE;
    uint32_t regINTSIG;
    uint32_t regAUTO;
} sd_card_sdhc_register_state_t;


//=============================================================================
//
// SD CARD COMMANDS
//
//=============================================================================
#define  CMD0_go_idle           0
#define  CMD2_all_send_cid      2
#define  CMD3_send_rel_addr     3
#define  CMD4_set_dsr           4
#define  CMD5_io_send_op_cond   5
#define  CMD6_switch_func       6
#define  CMD7_sel_desel_card    7
#define  CMD8_send_if_cond      8
#define  CMD9_send_csd          9
#define  CMD10_send_cid         10
#define  CMD11_voltage_switch   11
#define  CMD12_stop_transm      12
#define  CMD13_send_status      13
#define  CMD15_go_inactive      15
#define  CMD16_set_blocklen         16
#define  CMD17_read_single_block    17
#define  CMD18_read_multiple_block  18
#define  CMD19_send_tuning_blobk    19
#define  CMD23_set_block_count      23
#define  CMD24_write_block          24
#define  CMD25_write_mult_block     25
#define  CMD27_program_csd          27
#define  CMD28_set_write_prot       28
#define  CMD29_clr_write_prot       29
#define  CMD30_send_write_prot      30
#define  CMD32_erase_wr_blk_start   32
#define  CMD33_erase_wr_blk_end     33
#define  CMD38_erase                38
#define  CMD42_lock_unlock          42
#define  CMD52_io_rw_direct         52
#define  CMD53_io_rw_extended       53
#define  CMD55_app_cmd              55
#define  CMD56_gen_cmd              56
#define  ACMD6_set_bus_width        6
#define  ACMD13_sd_status           13
#define  ACMD22_send_num_wr_blocks  22
#define  ACMD23_set_wr_blk_erase_cnt  23
#define  ACMD41_sd_app_op_cond      41
#define  ACMD42_set_clr_card_det    42
#define  ACMD51_send_scr            51    

//=============================================================================
//
// SD CARD STATES  , field "current_state" of Card Status (R1 response)
//
//=============================================================================
#define   SD_STATE_IDLE       0
#define   SD_STATE_READY      1
#define   SD_STATE_IDENT      2
#define   SD_STATE_STBY       3
#define   SD_STATE_TRAN       4
#define   SD_STATE_DATA       5
#define   SD_STATE_RCV        6
#define   SD_STATE_PRG        7
#define   SD_STATE_DIS        8
#define   SD_STATE_IOMODE     15            

  
//=============================================================================
//
// SD CARD STATUS REGISTER
//
//=============================================================================
typedef struct{
  uint32_t  reserved            :3; //reserved
  uint32_t  ake_seq_error       :1; //0 = Not used with SDIO operation
  uint32_t  reserved2           :1; //reserved
  uint32_t  app_cmd             :1; //0 = CMD55 not used in SDIO operation
  uint32_t  reserved3           :2; //reserved
  uint32_t  ready_for_data      :1; //0 = not used with SDIO operation
  uint32_t  current_state       :4; //15 = I/O only
  uint32_t  erase_reset         :1; //0 = not used with SDIO operation
  uint32_t  card_ecc_disabled   :1; //0 = not used with SDIO operation
  uint32_t  wp_erase_skip       :1; //0 = not used with SDIO operation
  uint32_t  cid_csd_overwrite   :1; //0 = not used with SDIO operation
  uint32_t  overrun             :1; //0 = not used with SDIO operation
  uint32_t  underrun            :1; //0 = not used with SDIO operation
  uint32_t  error               :1; //1 = a general or unknown error occured
  uint32_t  cc_error            :1; //0 = not used with SDIO operation
  uint32_t  card_ecc_failed     :1; //0 = not used with SDIO operation
  uint32_t  illegal_command     :1; //1 = previous command not legal for the card state
  uint32_t  com_crc_error       :1; //1 = the CRC check of the prvious command failed
  uint32_t  lock_unlock_failed  :1; //0 = not used with SDIO operation
  uint32_t  card_is_locked      :1; //0 = not used with SDIO operation
  uint32_t  wp_violation        :1; //0 = not used with SDIO operation
  uint32_t  erase_param         :1; //0 = not used with SDIO operation
  uint32_t  erase_seq_error     :1; //0 = not used with SDIO operation
  uint32_t  block_len_error     :1; //0 = not used with SDIO operation
  uint32_t  address_error       :1; //0 = not used with SDIO operation
  uint32_t  out_of_range        :1; //1 = the command's argument was out of the allowed range for this card
}__attribute__((packed)) SD_card_status_reg_t;


//=============================================================================
//
// SD CARD OCR REGISTER / ACMD41 ARGUMENT
//
//=============================================================================
typedef struct{
  uint32_t  reserved  :15;
  uint32_t  v27_v28   :1;
  uint32_t  v28_v29   :1;
  uint32_t  v29_v30   :1;
  uint32_t  v30_v31   :1;
  uint32_t  v31_v32   :1;
  uint32_t  v32_v33   :1;
  uint32_t  v33_v34   :1;
  uint32_t  v34_v35   :1;
  uint32_t  v35_v36   :1;   //b23

  uint32_t  S18R      :1;   //b24 -Switching to 1.8V request, 0-use current voltage, 1-switch to 1.8V signal voltage

  uint32_t  reserved2 :3;

  uint32_t  XPC       :1;   //SDXC power control, 0-Power saving, 1-maximum performance
  uint32_t  FB        :1;

  uint32_t  HCS       :1;   //Host(HCS)/Card (CCS) Capacity support: 0-SDSC, 1-SDHC or SDXC Supported
  uint32_t  busy      :1;   //Card Power Up Status (0-card busy, 1-init completed)
}__attribute__((packed))SD_ocr_reg_t;


//=============================================================================
//
// SD CARD CID REGISTER
//
//=============================================================================
typedef struct{
  uint32_t  unused        :1;
  uint32_t  crc           :7;   //CRC7 checksum

  uint32_t  mdt           :12;  //manufacturing date
  uint32_t  reserved      :4;   

  uint32_t  psn           :32;  //product serial number

  uint32_t  prv           :8;   //product revision
  uint32_t  pnm_l         :24;  //product name

  uint32_t  pnm_h         :16;  //product name
  uint32_t  oid           :16;   //OEM/Application ID

  uint32_t  mid           :8;    //manufacturer ID    
}__attribute__((packed))SD_card_cid_t;

//=============================================================================
//
// SD CARD CSD RESPONSE OFFSETS
//
//=============================================================================
//#define   CSD_OFFSET_VERSION                (126%32)  //30
//#define   CSD_OFFSET_TAAC                   (112%32)  //16
//#define   CSD_OFFSET_NSAC                   (104%32)  //8
//#define   CSD_OFFSET_TRAN_SPEED             (96%32)   //0
//#define   CSD_OFFSET_CCC                    (84%32)   //20
//#define   CSD_OFFSET_READ_BL_LEN            (80%32)   //16
//#define   CSD_OFFSET_READ_BL_PARTIAL        (79%32)   //15
//#define   CSD_OFFSET_WRITE_BLK_MISALIGN     (78%32)
//#define   CSD_OFFSET_READ_BLK_MISALIGN      (77%32)
//#define   CSD_OFFSET_DSR_IMP                (76%32)

//=============================================================================
//
// SD CARD CSD STRUCTURE
//
//=============================================================================
typedef struct{
  uint8_t   csd_structure;              //CSD structure
  uint8_t   taac;                       //data read access-time-1
  uint8_t   nsac;                       //data read access-time-2 in CLK cycles (NSAC*100)
  uint8_t   tran_speed;                 //max. data transfer rate
  uint16_t  ccc;                        //card command classes
  uint8_t   read_bl_len;                //max. read data block length
  uint8_t   read_bl_partial;            //partial blocks for read allowed
  uint8_t   write_blk_misalign;         //write block misalignment
  uint8_t   read_blk_misalign;          //read block misalignment
  uint8_t   dsr_imp;                    //DSR implemented
  uint32_t  c_size;                     //device size
  uint8_t   vdd_r_curr_min;             //max. read current @VDD min
  uint8_t   vdd_r_curr_max;             //max. read current @VDD max
  uint8_t   vdd_w_curr_min;             //max. write current @VDD min
  uint8_t   vdd_w_curr_max;             //max. write current @VDD max
  uint8_t   c_size_mult;                //device size multiplier
  uint8_t   erase_blk_en;               //erase single block enable
  uint8_t   sector_size;                //erase sector size
  uint8_t   wp_grp_size;                //write protect group size
  uint8_t   wrp_grp_enable;             //write protect group enable
  uint8_t   r2w_factor;                 //write speed factor
  uint8_t   write_bl_len;               //max. write data block length
  uint8_t   write_bl_partial;           //partial blocks for write allowed
  uint8_t   file_format_grp;            //File format group
  uint8_t   copy;                       //copy flag (OTP)
  uint8_t   perm_write_protect;         //permanent write protection
  uint8_t   tmp_write_protect;          //temporary write protection
  uint8_t   file_format;                //file_format
}__attribute__((packed))SD_card_csd_t;


//=============================================================================
//
// SD CARD SCR RESPONSE OFFSETS, 2 X 32b
//
//=============================================================================
#define   SCR_OFFSET_STRUCTURE                (60%32) //4 BIT WIDTH 
#define   SCR_OFFSET_SD_SPEC                  (56%32) //4 BIT WIDTH
#define   SCR_OFFSET_DATA_STAT_AFTER_ERASE    (55%32) //1 BIT WIDTH
#define   SCR_OFFSET_SD_SECURITY              (52%32) //3 BIT WIDTH
#define   SCR_OFFSET_SD_BUS_WIDTH             (48%32) //4 BIT WIDTH
#define   SCR_OFFSET_MANUFACTURER_USE         0       //32 BIT WIDTH (LOWER)

typedef struct{
  uint8_t   scr_structure;
  uint8_t   sd_spec;
  uint8_t   data_stat_after_erase;
  uint8_t   sd_security;
  uint8_t   sd_bus_width;
  uint32_t  manufacturer_usage;
}__attribute__((packed))SD_card_scr_t;


//=============================================================================
//
// SD CARD CMD6 ARGUMENT STRUCTURE
//
//=============================================================================
typedef struct{
  uint32_t  func_grp_1_access_mode  :4;
  uint32_t  func_grp_2_cmd_sys      :4;
  uint32_t  func_grp_3              :4;
  uint32_t  func_grp_4              :4;
  uint32_t  func_grp_5              :4;
  uint32_t  func_grp_6              :4;
  uint32_t  reserved                :7;
  uint32_t  mode                    :1;
}__attribute__((packed))CMD6_arg_t;


//=============================================================================
//
// SD CARD R1 RESPONSE
//
//=============================================================================
typedef struct{
  SD_card_status_reg_t  card_status;
}__attribute__((packed)) SD_R1_response_t;  


//=============================================================================
//
// SD CARD R2 CID/CSD RESPONSE
//
//=============================================================================
typedef struct{
  uint32_t   cid_csd[4];
}__attribute__((packed)) SD_R2_response_t;  


//=============================================================================
//
// SD CARD R3 RESPONSE
//
//=============================================================================
typedef struct{
  uint32_t  ocr;
}__attribute__((packed)) SD_R3_response_t;  


//=============================================================================
//
// SD CARD R6 RCA RESPONSE
//
//=============================================================================
typedef struct{
  uint32_t  reserved            :3; //reserved
  uint32_t  ake_seq_error       :1; //0 = Not used with SDIO operation
  uint32_t  reserved2           :1; //reserved
  uint32_t  app_cmd             :1; //0 = CMD55 not used in SDIO operation
  uint32_t  reserved3           :2; //reserved
  uint32_t  ready_for_data      :1; //0 = not used with SDIO operation
  uint32_t  current_state       :4; //15 = I/O only  
  uint32_t  error               :1; //1 = a general or unknown error occured
  uint32_t  illegal_command     :1; //1 = previous command not legal for the card state
  uint32_t  com_crc_error       :1; //1 = the CRC check of the prvious command failed
  uint32_t  rca                 :16; //Relative Card Address
}__attribute__((packed)) SD_R6_rca_response_t;  


//=============================================================================
//
// SD CARD R7 RESPONSE
//
//=============================================================================
typedef struct{
  uint32_t  ech0_back         :8;
  uint32_t  voltage_accepted  :4;
  uint32_t  reserved          :20;
  uint32_t  command_index     :6;
}__attribute__((packed)) SD_R7_response_t;  


//=============================================================================
//
// SD CARD R7 RESPONSE
//
//=============================================================================
uint32_t sd_card_init(sd_card_host_t *pCard);
void sd_card_host_init(sd_card_host_t *pCard, bool reinit);
uint32_t sd_card_read_block(sd_card_host_t *pCard, uint32_t address, uint32_t size, uint8_t* data);
uint32_t sd_card_write_block(sd_card_host_t *pCard, uint32_t address, uint32_t size, uint8_t* data);



#endif

