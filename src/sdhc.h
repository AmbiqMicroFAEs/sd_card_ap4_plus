//=============================================================================
// SD CARD driver example
// Mariusz Lacina, Ambiq, 2023
//=============================================================================
#ifndef SDHC_H
#define SDHC_H
#include <stdint.h>
#include <stdbool.h>

#include "am_mcu_apollo.h"

//=============================================================================
//
// SD CARD: sdhc_execute_cmd
// Wait the command done by checking command completion interrupt status
//
//=============================================================================
#define AM_HAL_WAIT_CMD_DONE_TIMEOUT 2
#define DYNAMIC_SWITCH_SDCLK_FEATURE

//=============================================================================
//
// SDHC: Interrupt configuration structure
//  0 - INT DISABLED
//  1 - INT ENABLED
//
//=============================================================================
typedef struct{
uint32_t   SDIO_INTSIG_CMDCMPEN         :1;
uint32_t   SDIO_INTSIG_XFERCMPEN        :1;
uint32_t   SDIO_INTSIG_BLOCKGAPEN       :1;
uint32_t   SDIO_INTSIG_DMAINTEN         :1;
uint32_t   SDIO_INTSIG_BUFFERWREN       :1;
uint32_t   SDIO_INTSIG_BUFFERRDEN       :1;
uint32_t   SDIO_INTSIG_CARDINSERTEN     :1;
uint32_t   SDIO_INTSIG_CARDREMOVALEN    :1;
uint32_t   SDIO_INTSIG_CARDINTEN        :1;
uint32_t   SDIO_INTSIG_INTAEN           :1;
uint32_t   SDIO_INTSIG_INTBEN           :1;
uint32_t   SDIO_INTSIG_INTCEN           :1;
uint32_t   SDIO_INTSIG_RETUNEEVENTEN    :1;
uint32_t   SDIO_INTSIG_BOOTACKEN        :1;
uint32_t   SDIO_INTSIG_BOOTTERM         :1;
uint32_t   SDIO_INTSIG_FIXED0           :1;
uint32_t   SDIO_INTSIG_CMDTOERREN       :1;
uint32_t   SDIO_INTSIG_CMDCRCERREN      :1;
uint32_t   SDIO_INTSIG_CMDENDBITERREN   :1;
uint32_t   SDIO_INTSIG_CMDIDXERREN      :1;
uint32_t   SDIO_INTSIG_DATATOERROREN    :1;
uint32_t   SDIO_INTSIG_DATACRCERREN     :1;
uint32_t   SDIO_INTSIG_DATAENDERREN     :1;
uint32_t   SDIO_INTSIG_CURRLMTERREN     :1;
uint32_t   SDIO_INTSIG_AUTOCMD12ERREN   :1;
uint32_t   SDIO_INTSIG_ADMAERREN        :1;
uint32_t   SDIO_INTSIG_TUNINGERREN      :1;
uint32_t   SDIO_ITSIG_RSVD              :1;
uint32_t   SDIO_INTSIG_TGTRESPEN        :1;
uint32_t   SDIO_INTSIG_VNDERREN         :3;
}__attribute__((packed)) sdio_intsig_t;


//=============================================================================
//
//  SD Host software reset types
//
//=============================================================================
typedef enum
{
    AM_HAL_SDHC_SW_RESET_DATA_LINE = 0U, /**< Reset the data circuit only. */
    AM_HAL_SDHC_SW_RESET_CMD_LINE  = 1U, /**< Reset the command circuit only. */
    AM_HAL_SDHC_SW_RESET_ALL       = 2U  /**< Reset the whole SD Host controller. */
}sdhc_sw_reset_e;

//=============================================================================
//
//  SDHC host configuration
//
//=============================================================================
typedef struct {
    void *pHandle;
    uint32_t ui32Module;
    bool bCardInSlot;
    bool bInited;
    am_hal_host_xfer_mode_e eXferMode;
    am_hal_host_bus_width_e eBusWidth;
    am_hal_host_bus_voltage_e eBusVoltage;
    am_hal_host_uhs_mode_e eUHSMode;
    uint32_t ui32MaxADMA2BlkNums;
    uint32_t ui32Clock;
    uint8_t  ui8Version;
    uint32_t ui32MaxClock;
    uint32_t ui32MinClock;
    uint32_t ui32OCRAvail;
    am_hal_card_cmd_t AsyncCmd;
    am_hal_card_cmd_data_t AsyncCmdData;
    am_hal_host_event_cb_t pfunEvtCallback;
}sdhc_host_cfg_t;


//=============================================================================
//
// SDHC: SDHC register states for power save restore
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
}sdhc_register_state_t;


//=============================================================================
//
// SDHC: SDHC State structure
//
//=============================================================================
typedef struct
{
    //
    //! Handle validation prefix.
    //
    am_hal_handle_prefix_t prefix;

    //
    //! Physical module number.
    //
    uint32_t ui32Module;
    uint32_t ui32HostSDMABufSize;
    uint8_t  ui8BaseClockFreq;

    //
    //! Link to the card host
    //
    sdhc_host_cfg_t *pHost;

    //
    //! Save the error count
    //
    bool bCmdErr;
    bool bDataErr;
    uint32_t ui32DataErrCnt;
    uint32_t ui32CmdErrCnt;

    //
    //! Store the data transfer infomation
    //
    uint32_t *pui32Buf;
    uint32_t ui32DataLen;
    uint32_t ui32BlkCnt;
    uint32_t ui32BlkNum;
    uint32_t ui32BlkSize;
    uint32_t ui32BlksPerSDMA;
    am_hal_data_dir_e eDataDir;
    bool bAsyncCmdIsDone;

    //
    //! Power Save-Restore register state
    //
    sdhc_register_state_t registerState;
}sdhc_state_t;



//=============================================================================
//
//  SDHC ADMA CONFIGURATION
//
//=============================================================================
typedef uint32_t dma_addr_t;

typedef struct
{
    uint8_t  ui8Attr;
    uint8_t  ui8Reserved;
    uint16_t ui16Len;
    uint32_t ui32AddrLow;
}sdhc_adma_desc_t;

//
// Maximum block number for one ADMA2 is limited to 127*SDHC_ADMA_TABLE_NO_ENTRIES.
// enlarging the entry number if want to transfer more blocks
// in one ADMA2 transaction.
//
//
// ADMA2 functions - maximum 127*16 = 2032 Blocks can be used by upper layer block-oriented APIs
// redefining  SDHC_ADMA_TABLE_NO_ENTRIES' to get the much larger data transfer.
//

#define SDHC_ADMA_MAX_LEN             65535
#define SDHC_ADMA_MAX_BLKS_PER_ENTRY  127 //  (SDHC_ADMA_MAX_LEN/512)
#define SDHC_ADMA_TABLE_NO_ENTRIES    16  // 127*16*512 = 1016KB

//
// Decriptor table defines
//
#define AM_HAL_ADMA_DESC_ATTR_VALID     (0x1 << 0)
#define AM_HAL_ADMA_DESC_ATTR_END       (0x1 << 1)
#define AM_HAL_ADMA_DESC_ATTR_INT       (0x1 << 2)
#define AM_HAL_ADMA_DESC_ATTR_ACT0      (0x1 << 3)
#define AM_HAL_ADMA_DESC_ATTR_ACT1      (0x1 << 4)
#define AM_HAL_ADMA_DESC_ATTR_ACT2      (0x1 << 5)

#define AM_HAL_ADMA_DESC_TRANSFER_DATA  AM_HAL_ADMA_DESC_ATTR_ACT2
#define AM_HAL_ADMA_DESC_LINK_DESC      (AM_HAL_ADMA_DESC_ATTR_ACT1 | AM_HAL_ADMA_DESC_ATTR_ACT2)



//=============================================================================
//
//  SDHC FUNCTIONS
//
//=============================================================================
uint32_t sdhc_host_config(sdhc_host_cfg_t *pHost);
uint32_t sdhc_host_setup(sdhc_host_cfg_t *pHost);
uint32_t sdhc_host_init(sdhc_host_cfg_t *pHost);
uint32_t sdhc_host_deinit(sdhc_host_cfg_t *pHost);
uint32_t sdhc_set_bus_clock(sdhc_host_cfg_t *pHost, uint32_t ui32Clock, bool enable);
uint32_t sdhc_execute_cmd(void *pHandle, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData);
uint32_t sdhc_interrupt_service(void *pHandle, uint32_t ui32IntStatus);
uint32_t sdhc_enable(void *pHandle);



#endif


