//=============================================================================
// SD CARD driver example
// Mariusz Lacina, Ambiq, 2023
//=============================================================================
#include <stdbool.h>
#include <stdint.h>

#include "am_mcu_apollo.h"

#include "am_util_delay.h"
#include "am_util_stdio.h"
#include "am_util_debug.h"

#include "sdhc.h"


//=============================================================================
//
// SDHC: macros
//
//=============================================================================
#define AM_HAL_MAGIC_SDHC 0x8313B0
#define AM_HAL_SDHC_CHK_HANDLE(h)                                                                                      \
    ((h) && ((am_hal_handle_prefix_t *)(h))->s.bInit && (((am_hal_handle_prefix_t *)(h))->s.magic == AM_HAL_MAGIC_SDHC))


//=============================================================================
//
// SDHC: DEBUG PRINTF
//
//=============================================================================
#define SDHC_DEBUG_PRINTF_ENABLED        //Enable for debug commands
#define SDHC_PRINTF_UTIL(...)                                             \
    am_util_stdio_printf(__VA_ARGS__);
#define SDHC_PRINTF(fmt, ...) SDHC_PRINTF_UTIL("[SDHC] line %04d - "fmt, __LINE__, ##__VA_ARGS__)

//=============================================================================
//
// SDHC: FUNCTIONS
//
//=============================================================================
static void     sdhc_pio_xfer_data(sdhc_state_t *pSDHCState);
static void     sdhc_sdma_xfer_data(sdhc_state_t *pSDHCState);
static inline   am_hal_card_data_err_e sdhc_check_data_error_type(uint32_t ui32IntStatus);
static inline   am_hal_card_cmd_err_e sdhc_check_cmd_error_type(uint32_t ui32IntStatus);
uint32_t        sdhc_interrupt_service(void *pHandle, uint32_t ui32IntStatus);
void            sdhc_set_txrx_delay(uint8_t ui8TxRxDelays[2]);;
static uint32_t sdhc_software_reset(SDIO_Type *pSDHC, sdhc_sw_reset_e eSoftwareReset);
uint32_t        sdhc_enable(void *pHandle);
static inline uint32_t sdhc_check_cmd_inhibit(SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData);
static uint32_t sdhc_prepare_cmd(SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData);
static void     sdhc_prepare_sdhci_adma_desc(uint32_t ui32Idx, dma_addr_t ui32DmaAddr, uint16_t ui16Len, bool bEnd);
static void     sdhc_prepare_adma_table(am_hal_card_cmd_data_t *pCmdData);
static uint32_t sdhc_prepare_xfer(sdhc_state_t *pSDHCState, SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData);
static uint32_t sdhc_send_cmd(sdhc_state_t *pSDHCState, SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData);
static uint32_t inline sdhc_wait_cmd_done(sdhc_state_t *pSDHCState, SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd);
static void     sdhc_get_cmd_response(SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd);
static void     sdhc_sdma_xfer_data(sdhc_state_t *pSDHCState);
static void     sdhc_pio_xfer_data(sdhc_state_t *pSDHCState);
static uint32_t sdhc_xfer_data(sdhc_state_t *pSDHCState, am_hal_card_cmd_data_t *pCmdData);
uint32_t        sdhc_card_busy(void *pHandle, uint32_t ui32TimeoutMS);
void            sdhc_set_bus_width(sdhc_host_cfg_t *pHost, am_hal_host_bus_width_e width);
void            sdhc_set_bus_voltage(sdhc_host_cfg_t *pHost, am_hal_host_bus_voltage_e eBusVoltage);
uint32_t        sdhc_set_bus_clock(sdhc_host_cfg_t *pHost, uint32_t ui32Clock, bool enable);
void            sdhc_interrupt_cfg(sdhc_host_cfg_t *pHost, sdio_intsig_t int_cfg);
uint32_t        sdhc_host_config(sdhc_host_cfg_t *pHost);
uint32_t        sdhc_host_init(sdhc_host_cfg_t *pHost);
uint32_t        sdhc_host_deinit(sdhc_host_cfg_t *pHost);
uint32_t        sdhc_execute_cmd(void *pHandle, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData);



//=============================================================================
//
// SDHC: ADMA TABLE
//
//=============================================================================
static sdhc_adma_desc_t adma_desc_table[SDHC_ADMA_TABLE_NO_ENTRIES];


//=============================================================================
//
// SDHC: structore for storing intstat bits used in polling loops 
//
//=============================================================================
volatile uint32_t sdhc_intstat = 0;


//=============================================================================
//
// SDHC: am_hal_sdhc_check_data_error_type
//
//=============================================================================
static inline am_hal_card_data_err_e sdhc_check_data_error_type(uint32_t ui32IntStatus)
{
    if (ui32IntStatus & SDIO_INTSTAT_ADMAERROR_Msk)
    {
        return AM_HAL_DATA_ERR_ADMAERROR;
    }
    else if (ui32IntStatus & SDIO_INTSTAT_DATACRCERROR_Msk)
    {
        return AM_HAL_DATA_ERR_DATACRCERROR;
    }
    else if (ui32IntStatus & SDIO_INTSTAT_DATATIMEOUTERROR_Msk)
    {
        return AM_HAL_DATA_ERR_DATATIMEOUTERROR;
    }
    else if (ui32IntStatus & SDIO_INTSTAT_DATAENDBITERROR_Msk)
    {
        return AM_HAL_DATA_ERR_DATAENDBITERROR;
    }
    else
    {
        return AM_HAL_DATA_ERR_TIMEOUT;
    }
}


//=============================================================================
//
// SDHC: interrupt service routine
//
//=============================================================================
uint32_t sdhc_interrupt_service(void *pHandle, uint32_t ui32IntStatus)
{
sdhc_state_t *pSDHCState = (sdhc_state_t *)pHandle;
sdhc_host_cfg_t *pHost = pSDHCState->pHost;
SDIO_Type *pSDHC = NULL;
am_hal_host_evt_t evt;

    sdhc_intstat |= ui32IntStatus;

    return AM_HAL_STATUS_SUCCESS;


//    ToDo, asynchronous trasnfers:


//    pSDHC = SDHCn(pSDHCState->ui32Module);

//    //
//    // Asynchronous PIO write
//    //
//    if ( ui32IntStatus & SDIO_INTSTAT_BUFFERREADREADY_Msk ||
//        ui32IntStatus & SDIO_INTSTAT_BUFFERWRITEREADY_Msk )
//    {
//        sdhc_pio_xfer_data(pSDHCState);

//#ifdef SDHC_DEBUG_PRINTF_ENABLED
//        // SDHC_PRINTF("%d\n", pSDHCState->ui32BlkCnt);
//#endif
//    }

//    //
//    // Asynchronous SDMA interrupt
//    //
//    if ( ui32IntStatus & SDIO_INTSTAT_DMAINTERRUPT_Msk )
//    {
//        // Invalidate DAXI to make sure CPU sees the new data when loaded
//        am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, 0);

//        sdhc_sdma_xfer_data(pSDHCState);

//#ifdef SDHC_DEBUG_PRINTF_ENABLED
//        //SDHC_PRINTF("ISR - DMA Xfer BlkCnt %d\n", pSDHCState->ui32BlkCnt);
//#endif
//        evt.eType = AM_HAL_EVT_SDMA_DONE;
//        evt.pCtx = pSDHCState->pHost;
//        evt.ui32BlkCnt = pSDHCState->ui32BlksPerSDMA;
//        if ( pSDHCState->pHost->pfunEvtCallback )
//        {
//            pSDHCState->pHost->pfunEvtCallback(&evt);
//        }
//    }

//    //
//    // Asynchronous PIO read, write, SDMA and ADMA xfer completion
//    //
//    if ( ui32IntStatus & SDIO_INTSTAT_TRANSFERCOMPLETE_Msk )
//    {
//        // Invalidate DAXI to make sure CPU sees the new data when loaded
//        am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, 0);

//        evt.eType = AM_HAL_EVT_XFER_COMPLETE;
//        evt.pCtx = pSDHCState->pHost;
//        evt.ui32BlkCnt = pSDHCState->ui32BlkCnt;
//        if ( pSDHCState->pHost->pfunEvtCallback )
//        {
//            pSDHCState->pHost->pfunEvtCallback(&evt);
//        }

//#ifdef SDHC_DEBUG_PRINTF_ENABLED
//        SDHC_PRINTF("ISR - Xfer Completion BlkCnt %d\n", pSDHCState->ui32BlkNum);
//#endif
//        pSDHCState->bAsyncCmdIsDone = true;

//#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE
//        //
//        // Disable the SDCLK after the xfer is done
//        //
//        pSDHC->CLOCKCTRL_b.SDCLKEN = 0x0;

//#ifdef SDHC_DEBUG_PRINTF_ENABLED
//        SDHC_PRINTF("Disable the SDCLK\n");
//#endif
//#endif

//    }

//    if (ui32IntStatus & (SDIO_INTSTAT_ADMAERROR_Msk         |
//                         SDIO_INTSTAT_DATACRCERROR_Msk      |
//                         SDIO_INTSTAT_DATATIMEOUTERROR_Msk  |
//                         SDIO_INTSTAT_DATAENDBITERROR_Msk))
//    {
//        evt.eType = AM_HAL_EVT_DAT_ERR;
//        evt.pCtx = pSDHCState->pHost;
//        evt.ui32BlkCnt = pSDHCState->ui32BlkCnt;
//        pHost->AsyncCmdData.eDataError = sdhc_check_data_error_type(ui32IntStatus);
//        pSDHCState->bDataErr = true;
//        pSDHCState->ui32DataErrCnt++;
//        if ( pSDHCState->pHost->pfunEvtCallback )
//        {
//            pSDHCState->pHost->pfunEvtCallback(&evt);
//        }

//#ifdef SDHC_DEBUG_PRINTF_ENABLED
//        SDHC_PRINTF("Xfer ERR INT 0x%x\n", ui32IntStatus);
//#endif
//        pSDHCState->bAsyncCmdIsDone = true;

//#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE
//        //
//        // Disable the SDCLK after the xfer is done
//        //
//        pSDHC->CLOCKCTRL_b.SDCLKEN = 0x0;

//#ifdef SDHC_DEBUG_PRINTF_ENABLED
//        SDHC_PRINTF("Disable the SDCLK\n");
//#endif
//#endif

//    }

//    //
//    // Return the status.
//    //
//    return AM_HAL_STATUS_SUCCESS;
}


//=============================================================================
//
// SDHC: HOST SET TXRX DELAY
//
//=============================================================================
void sdhc_set_txrx_delay(uint8_t ui8TxRxDelays[2])
{
    // Adjust TX CLK delay
    MCUCTRL->SDIOCTRL_b.SDIOITAPCHGWIN = 1;

    MCUCTRL->SDIOCTRL_b.SDIOOTAPDLYSEL = ui8TxRxDelays[0];
    MCUCTRL->SDIOCTRL_b.SDIOOTAPDLYENA = 1;

    // Adjust RX CLK delay
    MCUCTRL->SDIOCTRL_b.SDIOITAPDLYSEL = ui8TxRxDelays[1];
    MCUCTRL->SDIOCTRL_b.SDIOITAPDLYENA = 1;

    MCUCTRL->SDIOCTRL_b.SDIOITAPCHGWIN = 0;
}



//=============================================================================
//
// SDHC: am_hal_sdhc_software_reset
//
//=============================================================================
static uint32_t sdhc_software_reset(SDIO_Type *pSDHC, sdhc_sw_reset_e eSoftwareReset)
{
uint32_t ui32Mask;
uint32_t ui32Timeout;

    if ( !pSDHC->PRESENT_b.CARDINSERTED )
    {
        return AM_HAL_STATUS_FAIL;
    }

    ui32Mask = 0x1 << (SDIO_CLOCKCTRL_SWRSTDAT_Pos - eSoftwareReset);

    pSDHC->CLOCKCTRL |= ui32Mask;

    ui32Timeout = 150;
    do
    {
        if ( ui32Timeout == 0 )
        {
            return AM_HAL_STATUS_FAIL;
        }
        ui32Timeout--;
        am_util_delay_ms(1);
    } while (pSDHC->CLOCKCTRL & ui32Mask);

    return AM_HAL_STATUS_SUCCESS;
}


//=============================================================================
//
// SDHC: Enable function
//
//=============================================================================
uint32_t sdhc_enable(void *pHandle)
{
sdhc_state_t *pSDHCState = (sdhc_state_t *)pHandle;
SDIO_Type *pSDHC;
uint32_t ui32Status;

#ifndef AM_HAL_DISABLE_API_VALIDATION
    //
    // Check the handle.
    //
    if ( !AM_HAL_SDHC_CHK_HANDLE(pHandle) )
    {
        return AM_HAL_STATUS_INVALID_HANDLE;
    }

    if ( pSDHCState->prefix.s.bEnable )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
#endif // AM_HAL_DISABLE_API_VALIDATION

    pSDHC = SDHCn(pSDHCState->ui32Module);

    //
    // Enable the clock to SDIO peripheral
    //
    MCUCTRL->SDIOCTRL |= (MCUCTRL_SDIOCTRL_SDIOSYSCLKEN_Msk | MCUCTRL_SDIOCTRL_SDIOXINCLKEN_Msk);

    // Wait some time util clock stable
    am_util_delay_ms(10);

    //
    // Note SW_RESET_ALL is *only* used here before initializing other registers
    //
    if ( (ui32Status = sdhc_software_reset(pSDHC, AM_HAL_SDHC_SW_RESET_ALL)) != AM_HAL_STATUS_SUCCESS )
    {

#ifdef SDHC_DEBUG_PRINTF_ENABLED
        SDHC_PRINTF("Software Reset ALL failed\n");
#endif
        return ui32Status;
    }
    //
    // Enable the SDIO CD (GPIO75) & WP (GPIO74) pins
    // remapping to FPGA GP36 and GP35
    //
    GPIO->SDIFCDWP_b.SDIFCD = 75;
    GPIO->SDIFCDWP_b.SDIFWP = 74;

    //
    // Enable all interrupts
    //
    SDHCn(pSDHCState->ui32Module)->INTENABLE = (uint32_t)-1;

    //
    // Disable all interrupts SIGNAL
    //
    SDHCn(pSDHCState->ui32Module)->INTSIG = 0x0;

    //
    // Enable SDIO IRQ only after host is initialized successfully
    //
    NVIC_SetPriority(SDIO_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(SDIO_IRQn);

    //
    // Reset CMD and Data error count
    //
    pSDHCState->bCmdErr = false;
    pSDHCState->bDataErr = false;
    pSDHCState->ui32DataErrCnt = 0;
    pSDHCState->ui32CmdErrCnt = 0;

    pSDHCState->prefix.s.bEnable = true;

    //
    // Return the status.
    //
    return AM_HAL_STATUS_SUCCESS;
}


//=============================================================================
//
// SDHC: am_hal_sdhc_check_cmd_inhibit
//
//=============================================================================
static inline uint32_t sdhc_check_cmd_inhibit(SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
uint32_t ui32CmdInhibitMask;

    //
    // Check if CMD and DAT Lines are busy
    //
    ui32CmdInhibitMask = SDIO_PRESENT_CMDINHCMD_Msk | SDIO_PRESENT_CMDINHDAT_Msk;

    if ( !pCmdData || pCmdData->bNotUseDataLine )
    {
        ui32CmdInhibitMask &= ~SDIO_PRESENT_CMDINHDAT_Msk;
    }

    if ( am_hal_delay_us_status_check(100, (uint32_t)&pSDHC->PRESENT, ui32CmdInhibitMask,
        ui32CmdInhibitMask, false) == AM_HAL_STATUS_TIMEOUT )
    {
#ifdef SDHC_DEBUG_PRINTF_ENABLED
        SDHC_PRINTF("%s : CMD and DAT line is busy\n", __FUNCTION__);
#endif

        pCmd->eError = AM_HAL_CMD_ERR_TIMEOUT;
        return AM_HAL_STATUS_TIMEOUT;
    }

    return AM_HAL_STATUS_SUCCESS;
}


//=============================================================================
//
// SDHC: am_hal_sdhc_prepare_cmd
//
//=============================================================================
static uint32_t sdhc_prepare_cmd(SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
uint32_t ui32CmdReg = 0x0;

    //
    // Response Type Select
    //
    if ( !(pCmd->ui32RespType & MMC_RSP_PRESENT) )
    {
        ui32CmdReg = SDIO_TRANSFER_RESPTYPESEL_NORESPONSE;
    }
    else if ( pCmd->ui32RespType & MMC_RSP_136 )
    {
        ui32CmdReg |= SDIO_TRANSFER_RESPTYPESEL_LEN136 << SDIO_TRANSFER_RESPTYPESEL_Pos;
    }
    else if ( pCmd->ui32RespType & MMC_RSP_BUSY )
    {
        ui32CmdReg |= SDIO_TRANSFER_RESPTYPESEL_LEN48CHKBUSY << SDIO_TRANSFER_RESPTYPESEL_Pos;
    }
    else
    {
        ui32CmdReg |= SDIO_TRANSFER_RESPTYPESEL_LEN48 << SDIO_TRANSFER_RESPTYPESEL_Pos;
    }

    //
    // Data Present Select
    //
    if ( pCmdData != NULL )
    {
        ui32CmdReg |= SDIO_TRANSFER_DATAPRSNTSEL_Msk;
    }

    //
    // Command CRC Check Enable
    //
    if ( pCmd->ui32RespType & MMC_RSP_CRC )
    {
        ui32CmdReg |= SDIO_TRANSFER_CMDCRCCHKEN_Msk;
    }

    //
    // Command Index Check Enable
    //
    if ( pCmd->ui32RespType & MMC_RSP_OPCODE )
    {
        ui32CmdReg |= SDIO_TRANSFER_CMDIDXCHKEN_Msk;
    }

    ui32CmdReg |= (pCmd->ui8Idx << SDIO_TRANSFER_CMDIDX_Pos);

    return ui32CmdReg;
}


//=============================================================================
//
// SDHC: am_hal_sdhc_prepare_sdhci_adma_desc
//
//=============================================================================
static void sdhc_prepare_sdhci_adma_desc(uint32_t ui32Idx, dma_addr_t ui32DmaAddr, uint16_t ui16Len, bool bEnd)
{
sdhc_adma_desc_t *pDesc;
uint8_t ui8Attr;

    pDesc = &adma_desc_table[ui32Idx];

    ui8Attr = AM_HAL_ADMA_DESC_ATTR_VALID | AM_HAL_ADMA_DESC_TRANSFER_DATA;
    if ( bEnd )
    {
        ui8Attr |= AM_HAL_ADMA_DESC_ATTR_END;
    }

    pDesc->ui8Attr = ui8Attr;
    pDesc->ui16Len = ui16Len;
    pDesc->ui8Reserved = 0;
    pDesc->ui32AddrLow = ui32DmaAddr;
}


//=============================================================================
//
// SDHC: am_hal_sdhc_prepare_adma_table
//
//=============================================================================
static void sdhc_prepare_adma_table(am_hal_card_cmd_data_t *pCmdData)
{
bool bEnd;
uint32_t i;
int32_t i32BlkCnt;
uint32_t ui32XferBytes;
dma_addr_t ui32DmaAddr;

    i = 0;
    i32BlkCnt = pCmdData->ui32BlkCnt;
    ui32DmaAddr = (dma_addr_t)(pCmdData->pui8Buf);
    while (i32BlkCnt > 0)
    {
        ui32XferBytes = (i32BlkCnt > SDHC_ADMA_MAX_BLKS_PER_ENTRY) ?
            SDHC_ADMA_MAX_BLKS_PER_ENTRY*pCmdData->ui32BlkSize : i32BlkCnt*pCmdData->ui32BlkSize;
        bEnd = (i32BlkCnt > SDHC_ADMA_MAX_BLKS_PER_ENTRY) ? false : true;
        sdhc_prepare_sdhci_adma_desc(i, ui32DmaAddr, ui32XferBytes, bEnd);
        i++;
        ui32DmaAddr += ui32XferBytes;
        i32BlkCnt -= SDHC_ADMA_MAX_BLKS_PER_ENTRY;
    }
}


//=============================================================================
//
// SDHC: sdhc_prepare_xfer
//  Prepare transfer mode register and set the block size, block count, 
//  SDMA and host control registers (for DMA)
//
//=============================================================================
static uint32_t sdhc_prepare_xfer(sdhc_state_t *pSDHCState, SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
uint32_t ui32ModeReg = 0;
uint32_t ui32BlkReg  = 0;

    if ( pCmdData != NULL )
    {
        pSDHCState->pui32Buf = (uint32_t *)(pCmdData->pui8Buf);
        pSDHCState->ui32DataLen = pCmdData->ui32BlkCnt * pCmdData->ui32BlkSize;
        pSDHCState->eDataDir = pCmdData->dir;
        pSDHCState->ui32BlkSize = pCmdData->ui32BlkSize;
        pSDHCState->ui32BlkCnt = pCmdData->ui32BlkCnt;
        pSDHCState->ui32BlkNum = pCmdData->ui32BlkCnt;

        //
        // Transfer direction - 0x1 Read
        //
        if ( pCmdData->dir == AM_HAL_DATA_DIR_READ )
        {
            ui32ModeReg |= SDIO_TRANSFER_DXFERDIRSEL_Msk;
        }

        //
        // Transfer DMA setting
        //
        if ( pCmdData->eXferMode == AM_HAL_HOST_XFER_DEFAULT )
        {
            pCmdData->eXferMode = pSDHCState->pHost->eXferMode;
        }

        switch (pCmdData->eXferMode)
        {
            case AM_HAL_HOST_XFER_SDMA:
                ui32ModeReg |= SDIO_TRANSFER_DMAEN_Msk;
                pSDHC->HOSTCTRL1_b.DMASELECT = SDIO_HOSTCTRL1_DMASELECT_SDMA;
                pSDHC->SDMA = (dma_addr_t)(pCmdData->pui8Buf);
                pSDHCState->ui32BlksPerSDMA = pSDHCState->ui32HostSDMABufSize / pCmdData->ui32BlkSize;
                break;
            case AM_HAL_HOST_XFER_ADMA:
                ui32ModeReg |= SDIO_TRANSFER_DMAEN_Msk;
                pSDHC->HOSTCTRL1_b.DMASELECT = SDIO_HOSTCTRL1_DMASELECT_ADMA232;
                sdhc_prepare_adma_table(pCmdData);
                pSDHC->ADMALOWD = (dma_addr_t)(&adma_desc_table[0]);
                break;
            default:
                break;
        }

        //
        // Auto Command setting
        //
        if ( pCmd->bAutoCMD12 )
        {
            ui32ModeReg |= SDIO_TRANSFER_ACMDEN_CMD12ENABLE << SDIO_TRANSFER_ACMDEN_Pos;
        }
        else if ( pCmd->bAutoCMD23 )
        {
            ui32ModeReg |= SDIO_TRANSFER_ACMDEN_CMD23ENABLE << SDIO_TRANSFER_ACMDEN_Pos;

#ifdef SDHC_DEBUG_PRINTF_ENABLED
            if (pCmdData->eXferMode == AM_HAL_HOST_XFER_SDMA)
            {
                SDHC_PRINTF("SDMA can't be used if enabling CMD23\n");
            }
#endif

            pSDHC->SDMA = pCmdData->ui32BlkCnt;
        }

        //
        // Set the block count and size
        //
        ui32BlkReg |= pCmdData->ui32BlkSize << SDIO_BLOCK_TRANSFERBLOCKSIZE_Pos;
        if ( pCmdData->ui32BlkCnt > 1 )
        {
            ui32ModeReg |= SDIO_TRANSFER_BLKSEL_Msk | SDIO_TRANSFER_BLKCNTEN_Msk;
            ui32BlkReg |= pCmdData->ui32BlkCnt << SDIO_BLOCK_BLKCNT_Pos;
        }

        //pSDHC->BLOCK |= ui32BlkReg;

        pSDHC->BLOCK = ( pSDHC->BLOCK & ~(SDIO_BLOCK_TRANSFERBLOCKSIZE_Msk << SDIO_BLOCK_TRANSFERBLOCKSIZE_Pos)) | ui32BlkReg; //Andrea (Wisycom)

        //
        // Set the data timeout
        //
        pSDHC->CLOCKCTRL |= (0xe << SDIO_CLOCKCTRL_TIMEOUTCNT_Pos);
    }

    return ui32ModeReg;
}

//=============================================================================
//
// SDHC: am_hal_sdhc_software_reset
//  Sending the command by writing the argument and command registers
//
//=============================================================================
static uint32_t sdhc_send_cmd(sdhc_state_t *pSDHCState, SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
uint32_t ui32CmdReg;

    if ( sdhc_check_cmd_inhibit(pSDHC, pCmd, pCmdData) == AM_HAL_STATUS_TIMEOUT )
    {
        pCmd->eError = AM_HAL_CMD_ERR_INHIBIT;
        pSDHCState->bCmdErr = true;
        pSDHCState->ui32CmdErrCnt++;
        return AM_HAL_STATUS_TIMEOUT;
    }

    //
    // Prepare the command register value
    //
    ui32CmdReg = sdhc_prepare_cmd(pSDHC, pCmd, pCmdData);

    //
    // Prepare the transfer mode register value
    //
    ui32CmdReg |= sdhc_prepare_xfer(pSDHCState, pSDHC, pCmd, pCmdData);

    //
    // Eliminate the DAXI side effects
    //
    am_hal_sysctrl_bus_write_flush();

    //
    // Write the argument and command register
    //
    pSDHC->ARGUMENT1 = pCmd->ui32Arg;
    pSDHC->TRANSFER  = ui32CmdReg;

    return AM_HAL_STATUS_SUCCESS;

}


//=============================================================================
//
// SDHC: am_hal_sdhc_check_cmd_error_type
//
//=============================================================================
static inline am_hal_card_cmd_err_e sdhc_check_cmd_error_type(uint32_t ui32IntStatus)
{
    if (ui32IntStatus & SDIO_INTSTAT_COMMANDINDEXERROR_Msk)
    {
        return AM_HAL_CMD_ERR_INDEX;
    }
    else if (ui32IntStatus & SDIO_INTSTAT_COMMANDENDBITERROR_Msk)
    {
        return AM_HAL_CMD_ERR_ENDBIT;
    }
    else if (ui32IntStatus & SDIO_INTSTAT_COMMANDCRCERROR_Msk)
    {
        return AM_HAL_CMD_ERR_CRC;
    }
    else if (ui32IntStatus & SDIO_INTSTAT_COMMANDTIMEOUTERROR_Msk)
    {
        return AM_HAL_CMD_ERR_NO_RESPONSE;
    }
    else
    {
        return AM_HAL_CMD_ERR_TIMEOUT;
    }
}



//=============================================================================
//
// SDHC: am_hal_sdhc_wait_cmd_done
// Wait the command done by checking command completion interrupt status
//
//  Up to 200 msec timeout delay for some old cards
//=============================================================================
static uint32_t inline sdhc_wait_cmd_done(sdhc_state_t *pSDHCState, SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd)
{
uint32_t ui32Status;
uint32_t ui32BitMask = SDIO_INTSTAT_COMMANDCOMPLETE_Msk;  

    ui32Status = am_hal_delay_us_status_check(
        AM_HAL_WAIT_CMD_DONE_TIMEOUT*1000, (uint32_t)&sdhc_intstat,
        ui32BitMask,
        ui32BitMask,
        true);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
        pCmd->eError = sdhc_check_cmd_error_type(sdhc_intstat);
        pSDHCState->bCmdErr = true;
        pSDHCState->ui32CmdErrCnt++;
#ifdef SDHC_DEBUG_PRINTF_ENABLED
        SDHC_PRINTF("wait CMD completion INT timeout, Command idx: %d, Error Type: %d\n", pCmd->ui8Idx, pCmd->eError);
#endif

        return AM_HAL_STATUS_TIMEOUT;
    }

return AM_HAL_STATUS_SUCCESS;
}


//=============================================================================
//
// SDHC: am_hal_sdhc_get_cmd_response
//
//=============================================================================
static void sdhc_get_cmd_response(SDIO_Type *pSDHC, am_hal_card_cmd_t *pCmd)
{
uint32_t ui32RegResp[4];

    pCmd->eError = AM_HAL_CMD_ERR_NONE;
    if ( pCmd->ui32RespType == MMC_RSP_NONE )
    {
        return;
    }

    if ( pCmd->ui32RespType & MMC_RSP_136 )
    {
        ui32RegResp[0] = pSDHC->RESPONSE0;
        ui32RegResp[1] = pSDHC->RESPONSE1;
        ui32RegResp[2] = pSDHC->RESPONSE2;
        ui32RegResp[3] = pSDHC->RESPONSE3;
        pCmd->ui32Resp[0] = (ui32RegResp[3] << 8) | (ui32RegResp[2] >> 24);
        pCmd->ui32Resp[1] = (ui32RegResp[2] << 8) | (ui32RegResp[1] >> 24);
        pCmd->ui32Resp[2] = (ui32RegResp[1] << 8) | (ui32RegResp[0] >> 24);
        pCmd->ui32Resp[3] = (ui32RegResp[0] << 8);
    }
    else
    {
        pCmd->ui32Resp[0] = pSDHC->RESPONSE0;
    }
}


//=============================================================================
//
// SDHC: sdhc_sdma_xfer_data
//  Do the SDMA block transfer
//
//=============================================================================
static void sdhc_sdma_xfer_data(sdhc_state_t *pSDHCState)
{
    SDIO_Type *pSDHC = SDHCn(pSDHCState->ui32Module);

    //
    // Load the next DMA address
    //
    pSDHCState->ui32BlkCnt -= pSDHCState->ui32BlksPerSDMA;
    pSDHCState->pui32Buf += pSDHCState->ui32HostSDMABufSize / sizeof(uint32_t);
    pSDHC->SDMA = (dma_addr_t)(pSDHCState->pui32Buf);
}


//=============================================================================
//
// SDHC: am_hal_sdhc_pio_xfer_data
//  Do the PIO block transfer
//
//=============================================================================
static void sdhc_pio_xfer_data(sdhc_state_t *pSDHCState)
{
uint32_t ui32PreBufReadyMask;

    ui32PreBufReadyMask =  pSDHCState->eDataDir == AM_HAL_DATA_DIR_READ ?
    SDIO_PRESENT_BUFRDEN_Msk : SDIO_PRESENT_BUFWREN_Msk;

    SDIO_Type *pSDHC = SDHCn(pSDHCState->ui32Module);

    while (pSDHC->PRESENT & ui32PreBufReadyMask)
    {
        if ( pSDHCState->eDataDir == AM_HAL_DATA_DIR_READ )
        {
            for (int i = 0; i < pSDHCState->ui32BlkSize; i += 4)
            {
                *pSDHCState->pui32Buf++ = pSDHC->BUFFER;
            }
        }
        else
        {
            for (int i = 0; i < pSDHCState->ui32BlkSize; i += 4)
            {
                 pSDHC->BUFFER = *pSDHCState->pui32Buf++;
            }
        }
        pSDHCState->ui32BlkCnt--;
    }
}

//=============================================================================
//
// SDHC: am_hal_sdhc_xfer_data
//  Transfer the block data to the card
//  sdhc_intstat
//=============================================================================
static uint32_t sdhc_xfer_data(sdhc_state_t *pSDHCState,
                                      am_hal_card_cmd_data_t *pCmdData)
{
bool bXferDone;
uint32_t ui32BufReadyMask;

    SDIO_Type *pSDHC = SDHCn(pSDHCState->ui32Module);  

    // Xfer timeout value depends on the card performance. 32000 is close to 200 msec.
    uint32_t ui32Timeout = 32000*pCmdData->ui32BlkCnt;
#ifdef SDHC_DEBUG_PRINTF_ENABLED
    sdhc_host_cfg_t *pHost = pSDHCState->pHost;
    SDHC_PRINTF("Xfer Timeout is %d\n", ui32Timeout);
    SDHC_PRINTF("Xfer BLK Cnt is %d\n", pSDHCState->ui32BlkCnt);
    SDHC_PRINTF("Xfer DataLen is %d\n", pSDHCState->ui32DataLen);
    SDHC_PRINTF("Xfer Mode  is %d\n", pHost->eXferMode);
    SDHC_PRINTF("Xfer speed is %d\n", pHost->ui32Clock);
    SDHC_PRINTF("Xfer Width is %d\n", pHost->eBusWidth);
#endif

    ui32BufReadyMask = pSDHCState->eDataDir == AM_HAL_DATA_DIR_READ ?
        SDIO_INTSTAT_BUFFERREADREADY_Msk : SDIO_INTSTAT_BUFFERWRITEREADY_Msk;

    bXferDone = false;
    while ( !(sdhc_intstat & SDIO_INTSTAT_TRANSFERCOMPLETE_Msk) && (ui32Timeout > 0) )
    {
        if ( sdhc_intstat & SDIO_INTSTAT_TRANSFERCOMPLETE_Msk )
        {
            // Invalidate DAXI to make sure CPU sees the new data when loaded
            am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, 0);

            //
            // Transfer completed
            //
            pSDHCState->ui32BlkCnt = 0;
            bXferDone = true;

#ifdef SDHC_DEBUG_PRINTF_ENABLED
            SDHC_PRINTF("Xfer Completed\n");
#endif

        }
        else if ( sdhc_intstat & SDIO_INTSTAT_DMAINTERRUPT_Msk )
        {
            // Invalidate DAXI to make sure CPU sees the new data when loaded
            am_hal_daxi_control(AM_HAL_DAXI_CONTROL_INVALIDATE, 0);

            //
            // Transfer SDMA data
            //
           

#ifdef SDHC_DEBUG_PRINTF_ENABLED
            SDHC_PRINTF("Xfer SDMA DMA INTR\n");
#endif

            sdhc_sdma_xfer_data(pSDHCState);
        }
        else if ( sdhc_intstat & ui32BufReadyMask )
        {

            //
            // Transfer PIO data if PIO buffer is ready
            //        

#ifdef SDHC_DEBUG_PRINTF_ENABLED
            SDHC_PRINTF("Xfer PIO\n");
#endif

            sdhc_pio_xfer_data(pSDHCState);
        }
        else if (sdhc_intstat & (SDIO_INTSTAT_ADMAERROR_Msk    |
                         SDIO_INTSTAT_DATACRCERROR_Msk          |
                         SDIO_INTSTAT_DATATIMEOUTERROR_Msk      |
                         SDIO_INTSTAT_DATAENDBITERROR_Msk))
        {

#ifdef SDHC_DEBUG_PRINTF_ENABLED
            SDHC_PRINTF("Xfer Data Error INTSTAT = 0x%x\n", sdhc_intstat);
#endif

            pCmdData->eDataError = sdhc_check_data_error_type(sdhc_intstat);
            pSDHCState->bDataErr = true;
            pSDHCState->ui32DataErrCnt++;
            ui32Timeout = 1;
        }

        if ( ui32Timeout-- > 0 )
        {
            am_util_delay_us(5);
        }
    }

#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE

    //
    // Disable the SDCLK after the xfer is done
    //
    pSDHC->CLOCKCTRL_b.SDCLKEN = 0x0;
#ifdef SDHC_DEBUG_PRINTF_ENABLED
    SDHC_PRINTF("Disable the SDCLK\n");
#endif

#endif

    if ( !bXferDone && ui32Timeout == 0 )
    {
        return (pSDHCState->ui32BlkNum - pSDHCState->ui32BlkCnt) << 16 | AM_HAL_STATUS_TIMEOUT;
    }
    else
    {
        return (pSDHCState->ui32BlkNum - pSDHCState->ui32BlkCnt) << 16 | AM_HAL_STATUS_SUCCESS;
    }
}



//=============================================================================
//
// SD CARD HOST CARD BUSY
//
//=============================================================================
uint32_t sdhc_card_busy(void *pHandle, uint32_t ui32TimeoutMS)
{
sdhc_state_t *pSDHCState = (sdhc_state_t *)pHandle;
uint32_t ui32Dat0BusyMask;
SDIO_Type *pSDHC;

    pSDHC = SDHCn(pSDHCState->ui32Module);

    ui32Dat0BusyMask = 0x1 << SDIO_PRESENT_DAT30LINE_Pos;

    uint32_t ui32Status;
    ui32Status = am_hal_delay_us_status_check(
        ui32TimeoutMS*1000, (uint32_t)&pSDHC->PRESENT,
        ui32Dat0BusyMask,
        ui32Dat0BusyMask,
        true);

    if ( AM_HAL_STATUS_SUCCESS != ui32Status )
    {
#ifdef SDHC_DEBUG_PRINTF_ENABLED
        SDHC_PRINTF("%s : wait DAT0 busy timeout\n", __FUNCTION__);
#endif
    }

#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE
    //
    // Disable the SDCLK
    //
    pSDHC->CLOCKCTRL_b.SDCLKEN = 0x0;

#ifdef SDHC_DEBUG_PRINTF_ENABLED
    SDHC_PRINTF("Disable the SDCLK\n\r");
#endif

#endif

    return ui32Status;
}

//=============================================================================
//
// SDIO: HOST SET BUS WIDTH 
//
//=============================================================================
void sdhc_set_bus_width(sdhc_host_cfg_t *pHost, am_hal_host_bus_width_e width)
{
  SDHCn(pHost->ui32Module)->HOSTCTRL1_b.XFERWIDTH = 0x0;
  SDHCn(pHost->ui32Module)->HOSTCTRL1_b.DATATRANSFERWIDTH = (width == AM_HAL_HOST_BUS_WIDTH_4) ?  1 : 0;
  pHost->eBusWidth = width;
}

//=============================================================================
//
// SDIO: HOST SET BUS VOLTAGE
//
//=============================================================================
void sdhc_set_bus_voltage(sdhc_host_cfg_t *pHost, am_hal_host_bus_voltage_e eBusVoltage)
{
    switch (eBusVoltage)
    {
        case AM_HAL_HOST_BUS_VOLTAGE_1_8:
            SDHCn(pHost->ui32Module)->HOSTCTRL1 |= SDIO_HOSTCTRL1_VOLTSELECT_1_8V << SDIO_HOSTCTRL1_VOLTSELECT_Pos;
            break;
        case AM_HAL_HOST_BUS_VOLTAGE_3_0:
            SDHCn(pHost->ui32Module)->HOSTCTRL1 |= SDIO_HOSTCTRL1_VOLTSELECT_3_0V << SDIO_HOSTCTRL1_VOLTSELECT_Pos;
            break;
        case AM_HAL_HOST_BUS_VOLTAGE_3_3:
            SDHCn(pHost->ui32Module)->HOSTCTRL1 |= SDIO_HOSTCTRL1_VOLTSELECT_3_3V << SDIO_HOSTCTRL1_VOLTSELECT_Pos;
            break;
    }
    SDHCn(pHost->ui32Module)->HOSTCTRL1 |= SDIO_HOSTCTRL1_SDBUSPOWER_Msk;

    pHost->eBusVoltage = eBusVoltage;
}

//=============================================================================
//
// SDIO: HOST SET BUS CLOCK
//
//=============================================================================
uint32_t sdhc_set_bus_clock(sdhc_host_cfg_t *pHost, uint32_t ui32Clock, bool enable)
{
uint32_t ui32Divider;
SDIO_Type *pSDHC;

    pSDHC = SDHCn(pHost->ui32Module);

    //
    // Find the nearest the clock divider
    //
    for (ui32Divider = 1; ui32Divider <= 256; ui32Divider *= 2)
    {
        if ( (pHost->ui32MaxClock / ui32Divider) <= ui32Clock )
        {
            break;
        }
    }
    ui32Clock = pHost->ui32MaxClock / ui32Divider;

    ui32Divider >>= 1;

    //
    // Should stop the clock before changing the clock setting
    //
    pSDHC->CLOCKCTRL &= ~(SDIO_CLOCKCTRL_CLKEN_Msk | SDIO_CLOCKCTRL_SDCLKEN_Msk);

    //
    // Set the divider
    //
    pSDHC->CLOCKCTRL_b.FREQSEL = ui32Divider;

    //
    // Now enable the internal clock
    //
    pSDHC->CLOCKCTRL_b.CLKEN = 0x1;

    //
    // Wait util the internal clock to be stablized
    //
    uint32_t ui32Timeout = 10;
    while (pSDHC->CLOCKCTRL_b.CLKSTABLE)
    {
        if ( ui32Timeout == 0 )
        {

#ifdef SDHC_DEBUG_PRINTF_ENABLED
        SDHC_PRINTF("\nInternal clock can not be stablized\n\r");
#endif

        return AM_HAL_STATUS_FAIL;
        }
        ui32Timeout--;
        am_util_delay_ms(1);
    }

    //
    // Now enable the SDCLK
    //
    if(enable)pSDHC->CLOCKCTRL_b.SDCLKEN = 0x1;

    pHost->ui32Clock = ui32Clock;

return AM_HAL_STATUS_SUCCESS;
}


//=============================================================================
//
// SDIO: HOST INTERRUPTS (INTSIG) -> void am_sdio_isr(void)
//
//=============================================================================
void sdhc_interrupt_cfg(sdhc_host_cfg_t *pHost, sdio_intsig_t int_cfg)
{
SDIO_Type *pSDHC;

    *(uint32_t*)&sdhc_intstat = 0; //clear instat pooling bits

    pSDHC = SDHCn(pHost->ui32Module);

    pSDHC->INTSIG = *(uint32_t*)&int_cfg;
}



//=============================================================================
//
// SDHC: HOST CONFIGURATION
//
//=============================================================================
uint32_t sdhc_host_config(sdhc_host_cfg_t *pHost)
{
SDIO_Type *pSDHC;
sdhc_state_t *pSDHCState = (sdhc_state_t *)pHost->pHandle;
sdio_intsig_t int_cfg={0};

    pHost->ui32Module = 0;      //only one instance on Apollo4

    pSDHCState->pHost = pHost;  //pHost;
    pSDHC = SDHCn(pHost->ui32Module);
    pSDHCState->ui8BaseClockFreq = pSDHC->CAPABILITIES0_b.SDCLKFREQ;


    pHost->eBusVoltage = AM_HAL_HOST_BUS_VOLTAGE_1_8;
    pHost->eBusWidth = AM_HAL_HOST_BUS_WIDTH_1;
    pHost->eXferMode = AM_HAL_HOST_XFER_ADMA;
    pHost->eUHSMode = AM_HAL_HOST_UHS_NONE;

    pHost->ui8Version = pSDHC->SLOTSTAT_b.SPECVER;
    pHost->ui32MaxClock = pSDHCState->ui8BaseClockFreq * 1000000;  //96 MHz;
    pHost->ui32MinClock = pHost->ui32MaxClock / 256;    //375kHz
    pHost->ui32Clock = 0;

    pHost->ui32MaxADMA2BlkNums = SDHC_ADMA_TABLE_NO_ENTRIES*SDHC_ADMA_MAX_BLKS_PER_ENTRY;

    //
    // Default 4K SDMA boundary size
    //
    pSDHCState->ui32HostSDMABufSize = 4096*(0x1 << pSDHC->BLOCK_b.HOSTSDMABUFSZ);
    pSDHCState->bAsyncCmdIsDone = true;
   
    sdhc_set_bus_width(pHost, pHost->eBusWidth);

    sdhc_set_bus_voltage(pHost,pHost->eBusVoltage);


return sdhc_set_bus_clock(pHost, 375000, true);
}



//=============================================================================
//
// SDHC: HOST INIT
//
//=============================================================================
uint32_t sdhc_host_init(sdhc_host_cfg_t *pHost)
{
    //
    // Must initialize the SDHC state firstly
    //
    if (am_hal_sdhc_initialize(pHost->ui32Module, &pHost->pHandle) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Must power the SDHC peripheral firstly
    //
    if (am_hal_sdhc_power_control(pHost->pHandle, AM_HAL_SYSCTRL_WAKE, false) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Enable the clock to SDHC peripheral
    //
    if (sdhc_enable(pHost->pHandle) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    //
    // Host initial configuration
    //
    sdhc_host_config(pHost);

return AM_HAL_STATUS_SUCCESS;
}

//=============================================================================
//
// SDHC HOST DE-INIT
//
//=============================================================================
uint32_t sdhc_host_deinit(sdhc_host_cfg_t *pHost)
{
    if (am_hal_sdhc_disable(pHost->pHandle) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    if (am_hal_sdhc_power_control(pHost->pHandle, AM_HAL_SYSCTRL_NORMALSLEEP, false) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

    if (am_hal_sdhc_deinitialize(pHost->pHandle) != AM_HAL_STATUS_SUCCESS)
    {
        return AM_HAL_STATUS_FAIL;
    }

return AM_HAL_STATUS_SUCCESS;
}



//=============================================================================
//
// SDHC: sdhc_execute_cmd
//
//=============================================================================
uint32_t sdhc_execute_cmd(void *pHandle, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
sdhc_state_t *pSDHCState = (sdhc_state_t *)pHandle;
SDIO_Type *pSDHC = NULL;
uint32_t ui32Status  = AM_HAL_STATUS_SUCCESS;
sdio_intsig_t int_cfg={0};

    pSDHC = SDHCn(pSDHCState->ui32Module);

    if ( !pSDHCState->bAsyncCmdIsDone )
    {
#ifdef SDHC_DEBUG_PRINTF_ENABLED
        SDHC_PRINTF("Return because the previous async command is still ongoing\n");
#endif
        return AM_HAL_STATUS_IN_USE;
    }

    //
    // Reset the CMD and DATA internal circuit for safe
    //
    if (pSDHCState->bCmdErr)
    {
        sdhc_software_reset(pSDHC, AM_HAL_SDHC_SW_RESET_CMD_LINE);
        pSDHCState->bCmdErr = false;
#ifdef SDHC_DEBUG_PRINTF_ENABLED
        SDHC_PRINTF("Software reset the Cmd Error\n");
#endif
    }

    if (pSDHCState->bDataErr)
    {
        sdhc_software_reset(pSDHC, AM_HAL_SDHC_SW_RESET_DATA_LINE);
        pSDHCState->bDataErr = false;
#ifdef SDHC_DEBUG_PRINTF_ENABLED
        SDHC_PRINTF("Software reset the Data Error\n");
#endif
    }

    //
    // Clear all interrupts's status
    //
    pSDHC->INTSTAT = ((uint32_t)-1);
 
    //
    // Enable interrupts
    // 
    int_cfg.SDIO_INTSIG_CMDCMPEN = 1;
    int_cfg.SDIO_INTSIG_ADMAERREN = 1;
    int_cfg.SDIO_INTSIG_AUTOCMD12ERREN = 1;
    int_cfg.SDIO_INTSIG_BUFFERRDEN = 1;
    int_cfg.SDIO_INTSIG_BUFFERWREN = 1;
    int_cfg.SDIO_INTSIG_CMDCRCERREN = 1;
    int_cfg.SDIO_INTSIG_CMDENDBITERREN = 1;
    int_cfg.SDIO_INTSIG_CMDIDXERREN = 1;
    int_cfg.SDIO_INTSIG_CMDTOERREN = 1;
    int_cfg.SDIO_INTSIG_CURRLMTERREN = 1;
    int_cfg.SDIO_INTSIG_DATACRCERREN = 1;
    int_cfg.SDIO_INTSIG_DATAENDERREN = 1;
    int_cfg.SDIO_INTSIG_DATATOERROREN = 1;
    int_cfg.SDIO_INTSIG_TUNINGERREN = 1;
    int_cfg.SDIO_INTSIG_XFERCMPEN = 1;
    int_cfg.SDIO_INTSIG_DMAINTEN = 1;
    
    pSDHC->INTSIG |= *(uint32_t*)&int_cfg;
    sdhc_intstat = 0;


#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE
    //
    // Enable the SDCLK
    //
#ifdef SDHC_DEBUG_PRINTF_ENABLED
    SDHC_PRINTF("Enable the SDCLK\n");
#endif
    pSDHC->CLOCKCTRL_b.SDCLKEN = 0x1;
    
#endif

    if ( (ui32Status = sdhc_send_cmd(pSDHCState, pSDHC, pCmd, pCmdData)) != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    if ( (ui32Status = sdhc_wait_cmd_done(pSDHCState, pSDHC, pCmd)) != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    sdhc_get_cmd_response(pSDHC, pCmd);

    if ( pCmd->bASync )
    {
        AM_CRITICAL_BEGIN
        pSDHC->INTSIG = ((uint32_t)-1);
        pSDHCState->bAsyncCmdIsDone = false;
        AM_CRITICAL_END
    }

    if ( pCmdData == NULL || pCmd->bASync )
    {
#ifdef DYNAMIC_SWITCH_SDCLK_FEATURE
        if (pCmdData == NULL && !pCmd->bCheckBusyCmd)
        {
            pSDHC->CLOCKCTRL_b.SDCLKEN = 0x0;
#ifdef SDHC_DEBUG_PRINTF_ENABLED
            SDHC_PRINTF("Disable the SDCLK\n");
#endif
        }
#endif
        return AM_HAL_STATUS_SUCCESS;
    }

    return sdhc_xfer_data(pSDHCState, pCmdData);
}


