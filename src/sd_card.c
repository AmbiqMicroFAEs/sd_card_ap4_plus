//=============================================================================
// SD CARD driver example
// Mariusz Lacina, Ambiq, 2023
//=============================================================================
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "sd_card.h"
#include "sdhc.h"

//=============================================================================
//
// SD CARD: DEBUG PRINTF
//
//=============================================================================
#define SD_CARD_DEBUG_PRINTF_ENABLED        //Enable for local debug printouts
#define SD_CARD_PRINTF_UTIL(...)                                             \
    am_util_stdio_printf(__VA_ARGS__);
#define SDIO_PRINTF(fmt, ...) SD_CARD_PRINTF_UTIL("[SD_CARD] line %04d - "fmt, __LINE__, ##__VA_ARGS__)


//=============================================================================
//
// SD CARD FUNCTIONS
//
//=============================================================================
uint32_t  sd_card_host_power_control(sdhc_host_cfg_t *pHost, bool bOnOff);
uint32_t  sd_card_execute_cmd(sdhc_host_cfg_t *pHost, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData);
uint32_t  sd_card_set_uhs_mode(sd_card_host_t *pCard, am_hal_host_uhs_mode_e eUHSMode);
bool      sd_card_get_cd_inserted(sd_card_host_t *pCard);
uint32_t  sd_card_host_setup(sd_card_host_t *pCard);
uint32_t  sd_card_get_status(sd_card_host_t *pCard, uint32_t *pui32Status);
uint32_t  sd_card_sdhc_set_uhs_mode(sd_card_host_t *pCard, am_hal_host_uhs_mode_e eUHSMode);
uint32_t  sd_card_execute_cmd(sdhc_host_cfg_t *pHost, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData);
uint32_t  sd_card_read_card_status(sd_card_host_t *pCard, uint32_t *pui32Status);
void      sd_card_host_init(sd_card_host_t *pCard, bool reinit);
uint32_t  sd_card_cmd0_go_idle(sd_card_host_t *pCard);
uint32_t  sd_card_cmd5_io_send_op_cond(sd_card_host_t *pCard);
uint32_t  sd_card_cmd55_app_cmd(sd_card_host_t *pCard);
uint32_t  sd_card_acmd41_sd_app_op_cond(sd_card_host_t *pCard);
uint32_t  sd_card_cmd8_send_if_cond(sd_card_host_t *pCard);
uint32_t  sd_card_cmd13_send_status(sd_card_host_t *pCard);
uint32_t  sd_card_cmd2_all_send_cid(sd_card_host_t *pCard);
uint32_t  sd_card_cmd3_send_rel_addr(sd_card_host_t *pCard);
uint32_t  sd_card_cmd9_send_csd(sd_card_host_t *pCard);
uint32_t  sd_card_cmd7_select_card(sd_card_host_t *pCard);
uint32_t  sd_card_cmd7_deselect_card(sd_card_host_t *pCard);
void      sd_card_scr_get(uint8_t *data);
uint32_t  sd_card_acmd51_send_scr(sd_card_host_t *pCard);
void      sd_card_csd_decode(sd_card_host_t *pCard);
uint32_t  sd_card_acmd6_set_bus_width(sd_card_host_t *pCard, uint8_t width);
uint32_t  sd_card_cmd6_switch_func(sd_card_host_t *pCard, CMD6_arg_t cmd_arg);
uint32_t  sd_card_cmd16_set_blocklen(sd_card_host_t *pCard, uint32_t block_len);
uint32_t  sd_card_cmd17_read_single_block(sd_card_host_t *pCard, uint32_t size, uint32_t address, uint8_t *buf);
uint32_t  sd_card_cmd24_write_single_block(sd_card_host_t *pCard, uint32_t size, uint32_t address, uint8_t *buf);
uint32_t  sd_card_init(sd_card_host_t *pCard);
uint32_t  sd_card_write_block(sd_card_host_t *pCard, uint32_t address, uint32_t size, uint8_t* data);
uint32_t  sd_card_read_block(sd_card_host_t *pCard, uint32_t address, uint32_t size, uint8_t* data);

void sd_card_event_cb(am_hal_host_evt_t *pEvt);
void am_sdio_isr(void);


//=============================================================================
//
// SD CARD CONFIG STRUCTURE
//
//=============================================================================
sd_card_host_t sd_card_host = {
  
    {
    .pHandle    = NULL,
    .bInited    = false,
    .eBusVoltage = AM_HAL_HOST_BUS_VOLTAGE_1_8,
    .ui32Module = 0x0,
    .pfunEvtCallback = sd_card_event_cb,
    },
    {0}
};

SD_card_status_reg_t sd_card_status={0};
SD_card_csd_t sd_card_csd={0};
SD_card_scr_t sd_card_scr={0};
bool CMD8_error;
volatile bool bAsyncWriteIsDone = false;
volatile bool bAsyncReadIsDone  = false;


//=============================================================================
//
// Card event callback
//
//=============================================================================
void sd_card_event_cb(am_hal_host_evt_t *pEvt)
{
    am_hal_card_host_t *pHost = (am_hal_card_host_t *)pEvt->pCtx;

    if (AM_HAL_EVT_XFER_COMPLETE == pEvt->eType &&
        pHost->AsyncCmdData.dir == AM_HAL_DATA_DIR_READ)
    {
        bAsyncReadIsDone = true;
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
        SD_CARD_PRINTF_UTIL("Last Read Xfered block %d\n", pEvt->ui32BlkCnt);
#endif
    }

    if (AM_HAL_EVT_XFER_COMPLETE == pEvt->eType &&
        pHost->AsyncCmdData.dir == AM_HAL_DATA_DIR_WRITE)
    {
        bAsyncWriteIsDone = true;
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
        SD_CARD_PRINTF_UTIL("Last Write Xfered block %d\n", pEvt->ui32BlkCnt);
#endif
    }

    if (AM_HAL_EVT_SDMA_DONE == pEvt->eType &&
        pHost->AsyncCmdData.dir == AM_HAL_DATA_DIR_READ)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
        SD_CARD_PRINTF_UTIL("SDMA Read Xfered block %d\n", pEvt->ui32BlkCnt);
#endif
    }

    if (AM_HAL_EVT_SDMA_DONE == pEvt->eType &&
        pHost->AsyncCmdData.dir == AM_HAL_DATA_DIR_WRITE)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
        SD_CARD_PRINTF_UTIL("SDMA Write Xfered block %d\n", pEvt->ui32BlkCnt);
#endif
    }

    if (AM_HAL_EVT_DAT_ERR == pEvt->eType)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
        SD_CARD_PRINTF_UTIL("Data error type %d\n", pHost->AsyncCmdData.eDataError);
#endif
    }

    if (AM_HAL_EVT_CARD_PRESENT == pEvt->eType)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
        SD_CARD_PRINTF_UTIL("A card is inserted\n");
#endif
    }
}

//=============================================================================
//
// SDIO ISR
//
//=============================================================================
void am_sdio_isr(void)
{
uint32_t ui32IntStatus;

    //get int status
    ui32IntStatus = SDHCn(sd_card_host.host.ui32Module)->INTSTAT;

     //clear int status
    SDHCn(sd_card_host.host.ui32Module)->INTSTAT = ui32IntStatus;

    //SDHC int servic
    sdhc_interrupt_service(sd_card_host.host.pHandle, ui32IntStatus);
}


//=============================================================================
//
// SDIO: HOST GET CD
//
//=============================================================================
bool sd_card_get_cd_inserted(sd_card_host_t *pCard)
{
return SDHCn(pCard->host.ui32Module)->PRESENT_b.CARDINSERTED;
}


//=============================================================================
//
// SDIO: HOST SET UHS MODE
//
//=============================================================================
uint32_t sd_card_sdhc_set_uhs_mode(sd_card_host_t *pCard, am_hal_host_uhs_mode_e eUHSMode)
{
SDIO_Type *pSDHC;

    pSDHC = SDHCn(pCard->host.ui32Module);
 
    if (pCard->host.eUHSMode == eUHSMode )
    {
        return AM_HAL_STATUS_SUCCESS;
    }

    pSDHC->AUTO_b.UHSMODESEL = eUHSMode - AM_HAL_HOST_UHS_NONE;
    pCard->host.eUHSMode = eUHSMode;

    return AM_HAL_STATUS_SUCCESS;
}


//=============================================================================
//
// SDIO: HOST EXECUTE COMMAND
//
//=============================================================================
uint32_t sd_card_execute_cmd(sdhc_host_cfg_t *pHost, am_hal_card_cmd_t *pCmd, am_hal_card_cmd_data_t *pCmdData)
{
    uint32_t ui32Status;
    uint8_t ui8Retries = 3;

    while (ui8Retries--)
    {
        ui32Status = sdhc_execute_cmd(pHost->pHandle, pCmd, pCmdData);
        if ((ui32Status & 0xFFFF) == AM_HAL_STATUS_SUCCESS)
        {
            return ui32Status;
        }
    }
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
    SD_CARD_PRINTF_UTIL("\nFailed to send the %d command after retry three times\n\r", pCmd->ui8Idx);
#endif
    return ui32Status;
}


//=============================================================================
//
// SDIO: STATUS
//
//=============================================================================
uint32_t sd_card_read_card_status(sd_card_host_t *pCard, uint32_t *pui32Status)
{
    uint32_t ui32Status;
    am_hal_card_cmd_t cmd;


    if ( pCard->card.eState < AM_HAL_CARD_STATE_STDY )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.ui8Idx = CMD13_send_status;
    cmd.ui32Arg = pCard->card.ui16RCA << 16;
    cmd.ui32RespType = MMC_RSP_R1;

    if ( (ui32Status = sd_card_execute_cmd(pCard->host.pHandle, &cmd, NULL)) != AM_HAL_STATUS_SUCCESS )
    {
        return ui32Status;
    }

    if ( cmd.eError == AM_HAL_CMD_ERR_NONE )
    {
        *pui32Status = cmd.ui32Resp[0];
    }

    return ui32Status;
}


//=============================================================================
//
// SDIO: HOST INITIALIZE
//
//=============================================================================
void sd_card_host_init(sd_card_host_t *pCard, bool reinit)
{
    if (pCard->host.bInited && !reinit)
    {
        return;
    }

    if (pCard->host.bInited)
    {
        sdhc_host_deinit(&pCard->host);    
    }

    if (sdhc_host_init(&pCard->host) != AM_HAL_STATUS_SUCCESS)
    {
        pCard->host.pHandle = NULL;
        return;
    }

    pCard->host.bInited = true;
}


//=============================================================================
//
// SDIO: CMD0
//
//=============================================================================
uint32_t sd_card_cmd0_go_idle(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.ui8Idx   = CMD0_go_idle;
    cmd.ui32Arg  = 0x0;
    cmd.ui32RespType = MMC_RSP_NONE;

    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    return cmd.eError;
}

//=============================================================================
//
// SDIO: CMD5
//
//=============================================================================
uint32_t sd_card_cmd5_io_send_op_cond(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.ui8Idx   = CMD5_io_send_op_cond;
    cmd.ui32Arg  = 0x0;
    cmd.ui32RespType = MMC_RSP_R1;

    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    return cmd.eError;
}


//=============================================================================
//
// SDIO: CMD55 - APP CMD
//
//=============================================================================
uint32_t sd_card_cmd55_app_cmd(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.ui8Idx   = CMD55_app_cmd;
    cmd.ui32Arg  = 0x0;
    cmd.ui32RespType = MMC_RSP_R1;
  
    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];
    }

return cmd.eError;
}


//=============================================================================
//
// SDIO: ACMD41 - NEGOTIATE OPERATING CONDITIONS, INITIALIZE
//
//=============================================================================
uint32_t sd_card_acmd41_sd_app_op_cond(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;
SD_ocr_reg_t sd_ocr_host={0}, sd_ocr_card={0};
uint8_t attempts=255;

    do{

    //Send the application command
    memset(&cmd, 0, sizeof(cmd));
    cmd.ui8Idx   = CMD55_app_cmd;
    cmd.ui32Arg  = 0x0;
    cmd.ui32RespType = MMC_RSP_R1;
  
    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    if(cmd.eError != AM_HAL_CMD_ERR_NONE)
    {
    return cmd.eError;
    }

    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];
       
    //Send the ACMD41
    sd_ocr_host.v31_v32 = 0;
    sd_ocr_host.v32_v33 = 1;  //host operating voltage
    sd_ocr_host.v33_v34 = 0;
      
    sd_ocr_host.HCS = 1;      //High Capacity Support
    if(CMD8_error)
    sd_ocr_host.HCS = 0;  

    memset(&cmd, 0, sizeof(cmd));
    cmd.ui8Idx   = 41;
    cmd.ui32Arg  = *(uint32_t*)&sd_ocr_host;
    cmd.ui32RespType = MMC_RSP_R3;

    sd_card_execute_cmd(&pCard->host,&cmd, NULL);
    if(cmd.eError != AM_HAL_CMD_ERR_NONE)
    {
    return cmd.eError;
    }

    *(uint32_t*)&sd_ocr_card = cmd.ui32Resp[0];
    
    if(sd_ocr_card.busy == 1)break;

    am_util_delay_ms(10);  //card is busy, wait and repeat

    attempts--;
    }while(attempts>0);

  if(attempts == 0)return AM_HAL_CMD_ERR_NO_RESPONSE;  


  pCard->card.bHighCapcity = sd_ocr_card.HCS; //high capacity card
  pCard->card.ui32OCR = cmd.ui32Resp[0];    // Card OCR register
   
  if((sd_ocr_card.v31_v32 == 1)||
     (sd_ocr_card.v32_v33 == 1)||
     (sd_ocr_card.v33_v34 == 1))
      return cmd.eError;

return AM_HAL_CMD_ERR_NO_RESPONSE;
}



//=============================================================================
//
// SDIO: CMD8 - IF COND
//
//=============================================================================
uint32_t sd_card_cmd8_send_if_cond(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;
uint32_t  check_pattern = 0xaa;


    memset(&cmd, 0, sizeof(cmd));
    cmd.ui8Idx   = CMD8_send_if_cond;

    cmd.ui32Arg  = (uint32_t)(1<<8)+check_pattern; //VHS

    cmd.ui32RespType = MMC_RSP_R7;
  
    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];
    }

return cmd.eError;
}



//=============================================================================
//
// SDIO: CMD13 - SEND STATUS
//
//=============================================================================
uint32_t sd_card_cmd13_send_status(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.ui8Idx   = CMD13_send_status;
    cmd.ui32Arg  = 0;
    cmd.ui32RespType = MMC_RSP_R1;
  
    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];
    }

return cmd.eError;
}

//=============================================================================
//
// SDIO: CMD2 - Get CID
//
//=============================================================================
uint32_t sd_card_cmd2_all_send_cid(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;
SD_card_cid_t *card_cid;
uint32_t pattern_check = 0xaa;

    memset(&cmd, 0x0, sizeof(cmd));

    cmd.ui8Idx   = CMD2_all_send_cid;
    cmd.ui32Arg  = 0;
    cmd.ui32RespType = MMC_RSP_R2;

    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    pCard->card.bCidValid = 1;
    pCard->card.ui32CID[0] = cmd.ui32Resp[0];
    pCard->card.ui32CID[1] = cmd.ui32Resp[1];
    pCard->card.ui32CID[2] = cmd.ui32Resp[2];
    pCard->card.ui32CID[3] = cmd.ui32Resp[3];
    card_cid = (SD_card_cid_t*)&pCard->card.ui32CID[0]; //Verification
    }
 
    return cmd.eError;
}

//=============================================================================
//
// SDIO: CMD3 - Get RCA
//
//=============================================================================
uint32_t sd_card_cmd3_send_rel_addr(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;
SD_card_cid_t *card_cid;
SD_R6_rca_response_t *response;

    memset(&cmd, 0x0, sizeof(cmd));

    cmd.ui8Idx   = CMD3_send_rel_addr;
    cmd.ui32Arg  = 0;
    cmd.ui32RespType = MMC_RSP_R6;

    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    response = (SD_R6_rca_response_t*)&cmd.ui32Resp;
    pCard->card.ui16RCA = response->rca;
    }
 
    return cmd.eError;
}

//=============================================================================
//
// SDIO: CMD9 - Get CSD
//
//=============================================================================
uint32_t  sd_card_cmd9_send_csd(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;
SD_card_cid_t *card_cid;


    memset(&cmd, 0x0, sizeof(cmd));

    cmd.ui8Idx   = CMD9_send_csd;
    cmd.ui32Arg  = ((uint32_t)pCard->card.ui16RCA<<16);
    cmd.ui32RespType = MMC_RSP_R2;

    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    pCard->card.bCsdValid = 1;
    pCard->card.ui32CSD[0] = cmd.ui32Resp[0];
    pCard->card.ui32CSD[1] = cmd.ui32Resp[1];
    pCard->card.ui32CSD[2] = cmd.ui32Resp[2];
    pCard->card.ui32CSD[3] = cmd.ui32Resp[3];
    }
 
return cmd.eError;
}


//=============================================================================
//
// SDIO: CMD7 - Select Card 
//
//=============================================================================
uint32_t sd_card_cmd7_select_card(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;
SD_card_cid_t *card_cid;

    memset(&cmd, 0x0, sizeof(cmd));

    cmd.ui8Idx   = CMD7_sel_desel_card;
    cmd.ui32Arg  = ((uint32_t)pCard->card.ui16RCA<<16);
    cmd.ui32RespType = MMC_RSP_R1b;

    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];
    }
 
return cmd.eError; 
}

//=============================================================================
//
// SDIO: CMD7 - Select Card 
//
//=============================================================================
uint32_t sd_card_cmd7_deselect_card(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;
SD_card_cid_t *card_cid;

    memset(&cmd, 0x0, sizeof(cmd));

    cmd.ui8Idx   = CMD7_sel_desel_card;
    cmd.ui32Arg  = 0;
    cmd.ui32RespType = MMC_RSP_NONE;

    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

return cmd.eError; 
}

//=============================================================================
//
// SDIO: decode the card SCR to SCR structure "SD_card_scr_t sd_card_scr"
//
//=============================================================================
void sd_card_scr_get(uint8_t *data)
{
uint32_t  buf;

    buf = ((uint32_t)data[0]<<24); //upper word
    buf+= ((uint32_t)data[1]<<16);
    buf+= ((uint32_t)data[2]<<8);
    buf+= (uint32_t)data[3];

    sd_card_scr.scr_structure = (uint8_t)((buf>>SCR_OFFSET_STRUCTURE)&0b1111); 
    sd_card_scr.sd_spec = (uint8_t)((buf>>SCR_OFFSET_SD_SPEC)&0b1111);
    sd_card_scr.data_stat_after_erase = (uint8_t)((buf>>SCR_OFFSET_DATA_STAT_AFTER_ERASE)&0b1);
    sd_card_scr.sd_security = (uint8_t)((buf>>SCR_OFFSET_SD_SECURITY)&0b111);
    sd_card_scr.sd_bus_width = (uint8_t)((buf>>SCR_OFFSET_SD_BUS_WIDTH)&0b1111);

    buf = ((uint32_t)data[4]<<24); //lower word
    buf+= ((uint32_t)data[5]<<16);
    buf+= ((uint32_t)data[6]<<8);
    buf+= (uint32_t)data[7];

    sd_card_scr.manufacturer_usage = buf;
}

//=============================================================================
//
// SDIO: ACMD51 - Read SCR
//
//=============================================================================
uint32_t sd_card_acmd51_send_scr(sd_card_host_t *pCard)
{
am_hal_card_cmd_t cmd;
am_hal_card_cmd_data_t cmd_data;
SD_card_cid_t *card_cid;
uint8_t buf[16];
 
    //Send the application command
    memset(&cmd, 0, sizeof(cmd));
 
    cmd.ui8Idx   = CMD55_app_cmd;
    cmd.ui32Arg  = ((uint32_t)pCard->card.ui16RCA<<16);
    cmd.ui32RespType = MMC_RSP_R1;
 

    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    if(cmd.eError != AM_HAL_CMD_ERR_NONE)
    {
    return cmd.eError;
    }

    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];


    //Send the command
    memset(&cmd, 0, sizeof(cmd));
    memset(&cmd_data, 0, sizeof(cmd_data));
    memset(buf,0, sizeof(buf));    

    cmd_data.bNotUseDataLine = 0;
    cmd_data.dir = AM_HAL_DATA_DIR_READ;
    cmd_data.eXferMode = AM_HAL_HOST_XFER_DEFAULT; //AM_HAL_HOST_XFER_ADMA; //AM_HAL_HOST_XFER_DEFAULT;
    cmd_data.pui8Buf = buf;
    cmd_data.ui32BlkCnt = 1;
    cmd_data.ui32BlkSize = 16;

    cmd.ui8Idx   = 51;
    cmd.ui32Arg  = ((uint32_t)pCard->card.ui16RCA<<16);
    cmd.ui32RespType = MMC_RSP_R1;

    sd_card_execute_cmd(&pCard->host,&cmd, &cmd_data);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];
    sd_card_scr_get(buf);
    }

return cmd.eError;
}


//=============================================================================
//
// SDIO: decode the card CSD to CSD structure "SD_card_csd_t sd_card_csd"
//
//=============================================================================
void sd_card_csd_decode(sd_card_host_t *pCard)
{
uint32_t buf[4];
uint8_t csd_version;

  buf[0] = pCard->card.ui32CSD[0];
  buf[1] = pCard->card.ui32CSD[1];    
  buf[2] = pCard->card.ui32CSD[2];
  buf[3] = pCard->card.ui32CSD[3];

  sd_card_csd.csd_structure = (uint8_t)(buf[0]>>30);
  csd_version = sd_card_csd.csd_structure;
  sd_card_csd.taac = (uint8_t)((buf[0]&0x00FF0000)>>16);
  sd_card_csd.nsac = (uint8_t)((buf[0]&0x0000FF00)>>8);
  sd_card_csd.tran_speed = (uint8_t)((buf[0]&0x000000FF));

  sd_card_csd.ccc = (uint16_t)((buf[1]&0xFFF00000)>>20);
  sd_card_csd.read_bl_len = (uint8_t)((buf[1]&0x000F0000)>>16);
  sd_card_csd.read_bl_partial = (uint8_t)((buf[1]&0x00008000)>>15);
  sd_card_csd.write_blk_misalign = (uint8_t)((buf[1]&0x00004000)>>14);
  sd_card_csd.read_blk_misalign = (uint8_t)((buf[1]&0x00002000)>>13);
  sd_card_csd.dsr_imp = (uint8_t)((buf[1]&0x00001000)>>12);
  
  if(sd_card_csd.csd_structure == 0) //CSD V1.0
  {
  
  sd_card_csd.c_size = (uint16_t)((buf[1]&0x3FF)<<2);
  sd_card_csd.c_size += (uint16_t)(buf[2]>>30);
  sd_card_csd.vdd_r_curr_min = (uint8_t)((buf[2]&0x38000000)>>27);
  sd_card_csd.vdd_r_curr_max = (uint8_t)((buf[2]&0x07000000)>>24);
  sd_card_csd.vdd_w_curr_min = (uint8_t)((buf[2]&0x00E00000)>>21);    
  sd_card_csd.vdd_w_curr_max = (uint8_t)((buf[2]&0x001C0000)>>18); 
  sd_card_csd.c_size_mult = (uint8_t)((buf[2]&0x00038000)>>15); 
 
  
  }else                              //CSD V2.0
  {           
  sd_card_csd.c_size = (uint16_t)((buf[1]&0x3F)<<16);
  sd_card_csd.c_size += (uint16_t)((buf[2]&0xFFFF0000)>>16);
  sd_card_csd.vdd_r_curr_min = 0;
  sd_card_csd.vdd_r_curr_max = 0;
  sd_card_csd.vdd_w_curr_min = 0;
  sd_card_csd.vdd_w_curr_max = 0;
  sd_card_csd.c_size_mult = 0;
  }

  sd_card_csd.erase_blk_en = (uint8_t)((buf[2]&0x00004000)>>14); 
  sd_card_csd.sector_size = (uint8_t)((buf[2]&0x00002F80)>>7);
  sd_card_csd.wp_grp_size = (uint8_t)(buf[2]&0x0000007F);

  sd_card_csd.wrp_grp_enable = (uint8_t)((buf[3]>>31)&0x01);
  sd_card_csd.r2w_factor = (uint8_t)((buf[3]>>26)&0x07);
  sd_card_csd.write_bl_len = (uint8_t)((buf[3]>>22)&0x0F);
  sd_card_csd.write_bl_partial = (uint8_t)((buf[3]>>21)&0x01);
  sd_card_csd.file_format_grp = (uint8_t)((buf[3]>>15)&0x01);
  sd_card_csd.copy = (uint8_t)((buf[3]>>14)&0x01);
  sd_card_csd.perm_write_protect = (uint8_t)((buf[3]>>13)&0x01);
  sd_card_csd.tmp_write_protect = (uint8_t)((buf[3]>>12)&0x01);
  sd_card_csd.file_format = (uint8_t)((buf[3]>>10)&0x03);
}



//=============================================================================
//
// SDIO: ACMD6 - Set BUS Width
//
//=============================================================================
uint32_t sd_card_acmd6_set_bus_width(sd_card_host_t *pCard, uint8_t width)
{
am_hal_card_cmd_t cmd;
SD_card_cid_t *card_cid;
uint8_t  bus_width;
 
    switch(width)
    {
    case AM_HAL_HOST_BUS_WIDTH_1:
    bus_width = 0;
    if((sd_card_scr.sd_bus_width&0b0001) == 0)return AM_HAL_CMD_ERR_NO_RESPONSE;
    break;

    case AM_HAL_HOST_BUS_WIDTH_4:
    bus_width = 2;
    if((sd_card_scr.sd_bus_width&0b0100) == 0)return AM_HAL_CMD_ERR_NO_RESPONSE;
    break;

    default:
    return AM_HAL_CMD_ERR_NO_RESPONSE;
    }

 
    //Send the application command
    memset(&cmd, 0, sizeof(cmd));
 
    cmd.ui8Idx   = CMD55_app_cmd;
    cmd.ui32Arg  = ((uint32_t)pCard->card.ui16RCA<<16);
    cmd.ui32RespType = MMC_RSP_R1;

    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    if(cmd.eError != AM_HAL_CMD_ERR_NONE)
    {
    return cmd.eError;
    }

    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];

    //Send the command
    memset(&cmd, 0, sizeof(cmd));
    cmd.ui8Idx   = ACMD6_set_bus_width;
    cmd.ui32Arg  = bus_width;
    cmd.ui32RespType = MMC_RSP_R1;

    sd_card_execute_cmd(&pCard->host,&cmd, NULL);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];
    }

return cmd.eError;
}



//=============================================================================
//
// SDIO: CMD6 - Switch function
// 
// Inputs: 
//    CMD6_arg_t cmd_arg, see SD Layer Spec. Table 4-9
// 
//=============================================================================
uint32_t  sd_card_cmd6_switch_func(sd_card_host_t *pCard, CMD6_arg_t cmd_arg)
{
am_hal_card_cmd_t cmd;
am_hal_card_cmd_data_t cmd_data;
SD_card_cid_t *card_cid;
uint8_t buf[64];
  
    memset(&cmd, 0, sizeof(cmd));
    memset(&cmd_data, 0, sizeof(cmd_data));
    memset(&buf, 0, sizeof(buf));

    cmd_data.bNotUseDataLine = 0;
    cmd_data.dir = AM_HAL_DATA_DIR_READ;
    cmd_data.eXferMode = AM_HAL_HOST_XFER_DEFAULT; //AM_HAL_HOST_XFER_ADMA; //AM_HAL_HOST_XFER_DEFAULT;
    cmd_data.pui8Buf = buf;
    cmd_data.ui32BlkCnt = 1;
    cmd_data.ui32BlkSize = 64;

    cmd.ui8Idx   = CMD6_switch_func;
    cmd.ui32Arg  = *(uint32_t*)&cmd_arg;
    
    cmd.ui32RespType = MMC_RSP_R1;

    sd_card_execute_cmd(&pCard->host,&cmd, &cmd_data);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];
    
    //to do: decode the received sd_card_scr_decode(buf);
    }

return cmd.eError;
}

//=============================================================================
//
// SDIO: CMD16 - Set block length
//
//=============================================================================
uint32_t  sd_card_cmd16_set_blocklen(sd_card_host_t *pCard, uint32_t block_len)
{
am_hal_card_cmd_t cmd;
      
    memset(&cmd, 0, sizeof(cmd));
    cmd.ui8Idx   = CMD16_set_blocklen;
    cmd.ui32Arg  = block_len;
    cmd.ui32RespType = MMC_RSP_R1;

    sd_card_execute_cmd(&pCard->host,&cmd,NULL);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];
    }
return cmd.eError;
}


//=============================================================================
//
// SDIO: CMD17 - Read Single Block
//
//=============================================================================
uint32_t sd_card_cmd17_read_single_block(sd_card_host_t *pCard, uint32_t size, uint32_t address, uint8_t *buf)
{
am_hal_card_cmd_t cmd;
am_hal_card_cmd_data_t cmd_data;

    memset(&cmd, 0, sizeof(cmd));
    memset(&cmd_data, 0, sizeof(cmd_data));

    cmd.ui8Idx   = CMD17_read_single_block;
    cmd.ui32Arg  = address;
    cmd.ui32RespType = MMC_RSP_R1;
    cmd.bCheckBusyCmd = true;

    cmd_data.bNotUseDataLine = 0;
    cmd_data.dir = AM_HAL_DATA_DIR_READ;
    cmd_data.eXferMode = AM_HAL_HOST_XFER_DEFAULT;
    cmd_data.pui8Buf = buf;
    cmd_data.ui32BlkCnt = 1;
    cmd_data.ui32BlkSize = size;
    
    sd_card_execute_cmd(&pCard->host,&cmd,&cmd_data);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];
    }

return cmd.eError;
}


//=============================================================================
//
// SDIO: CMD24 - Write Single Block
//
//=============================================================================
uint32_t sd_card_cmd24_write_single_block(sd_card_host_t *pCard, uint32_t size, uint32_t address, uint8_t *buf)
{
am_hal_card_cmd_t cmd;
am_hal_card_cmd_data_t cmd_data;

    memset(&cmd, 0, sizeof(cmd));
    memset(&cmd_data, 0, sizeof(cmd_data));

    cmd.ui8Idx   = CMD24_write_block;
    cmd.ui32Arg  = address;
    cmd.ui32RespType = MMC_RSP_R1; //MMC_RSP_R1;//MMC_RSP_R1; //MMC_RSP_NONE
   
    cmd_data.bNotUseDataLine = 0;
    cmd_data.dir = AM_HAL_DATA_DIR_WRITE;
    cmd_data.eXferMode = AM_HAL_HOST_XFER_DEFAULT;
    cmd_data.pui8Buf = buf;
    cmd_data.ui32BlkCnt = 1;
    cmd_data.ui32BlkSize = size;
    
    sd_card_execute_cmd(&pCard->host,&cmd,&cmd_data);

    if(cmd.eError == AM_HAL_CMD_ERR_NONE)
    {
    *(uint32_t*)&sd_card_status = cmd.ui32Resp[0];
    }

return cmd.eError;
}

//=============================================================================
//
// SDIO: INIT
//
//=============================================================================
uint32_t sd_card_init(sd_card_host_t *pCard)
{
uint32_t status;
CMD6_arg_t cmd6_arg;

    
    //
    // Software reset CMD0
    //
    if(sd_card_cmd0_go_idle(pCard) != AM_HAL_CMD_ERR_NONE)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
        SD_CARD_PRINTF_UTIL("\nCMD0 Failed\n\r");
#endif
        return AM_HAL_STATUS_FAIL;
    }

    am_util_delay_ms(50);


    //
    // CMD8 - mandatory but some cards will not answer
    //
    CMD8_error = false;
    if(sd_card_cmd8_send_if_cond(pCard)!=AM_HAL_CMD_ERR_NONE)
    CMD8_error = true;
    
    //
    // ACMD41 - mandatory
    //
    if(sd_card_acmd41_sd_app_op_cond(pCard) != AM_HAL_CMD_ERR_NONE)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
    SD_CARD_PRINTF_UTIL("\nACMD41 Failed\n\r");
#endif
    return AM_HAL_STATUS_FAIL;
    }
    
    //
    // Get card CID
    //
    if(sd_card_cmd2_all_send_cid(pCard) != AM_HAL_CMD_ERR_NONE)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
    SD_CARD_PRINTF_UTIL("\nRead Card CID Failed!\n\r");
#endif
    return AM_HAL_STATUS_FAIL;
    }

    //
    // Get RCA (Relative Card Address )
    //
    if(sd_card_cmd3_send_rel_addr(pCard) != AM_HAL_CMD_ERR_NONE)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
    SD_CARD_PRINTF_UTIL("\nRead Card RCA Failed!\n\r");
#endif
    return AM_HAL_STATUS_FAIL;
    }

    //
    // Get CSD
    //
    if(sd_card_cmd9_send_csd(pCard) != AM_HAL_CMD_ERR_NONE)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
    SD_CARD_PRINTF_UTIL("\nRead Card CSD Failed!\n\r");
#endif
    return AM_HAL_STATUS_FAIL;
    }
    sd_card_csd_decode(pCard);

    //
    // Select card 
    //
    if(sd_card_cmd7_select_card(pCard) != AM_HAL_CMD_ERR_NONE)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
    SD_CARD_PRINTF_UTIL("\nSelect Card Failed!\n\r");
#endif
    return AM_HAL_STATUS_FAIL;
    }
    
    //
    // Get SCR register
    //
    if(sd_card_acmd51_send_scr(pCard) != AM_HAL_CMD_ERR_NONE)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
    SD_CARD_PRINTF_UTIL("\nCard SCR Read Failed!\n\r");
#endif
    return AM_HAL_STATUS_FAIL;
    }

    //sdhc_set_bus_clock(&pCard->host,1500000,false); //switch to high speed, doesn't work on the handmade development board due to unmatched wire connections

    //
    // Set bus width to 4 data lines
    //
    if(sd_card_acmd6_set_bus_width(pCard, AM_HAL_HOST_BUS_WIDTH_4) != AM_HAL_CMD_ERR_NONE)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
    SD_CARD_PRINTF_UTIL("\nSet BUS Width Failed!\n\r");
#endif
    return AM_HAL_STATUS_FAIL;
    }
    am_hal_sdhc_set_bus_width(pCard->host.pHandle,AM_HAL_HOST_BUS_WIDTH_4);

    
    if(sd_card_scr.sd_spec >= 1) //Sd Ver 1.1 and higher
    {
      //
      // CMD6 - check function
      //
      cmd6_arg.reserved = 0;
      cmd6_arg.mode = 0;
      cmd6_arg.func_grp_6 = 0xf;
      cmd6_arg.func_grp_5 = 0xf;
      cmd6_arg.func_grp_4 = 0xf;
      cmd6_arg.func_grp_3 = 0xf;
      cmd6_arg.func_grp_2_cmd_sys = 0xf;
      cmd6_arg.func_grp_1_access_mode = 0x1;
    
      if(sd_card_cmd6_switch_func(pCard, cmd6_arg)!=AM_HAL_CMD_ERR_NONE)
      {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
      SD_CARD_PRINTF_UTIL("\nCMD6 Check Func Failed!\n\r");
#endif
      return AM_HAL_STATUS_FAIL;
      }
    
      //
      // Select High Speed Mode
      //
      cmd6_arg.mode = 1;
      if(sd_card_cmd6_switch_func(pCard, cmd6_arg)!=AM_HAL_CMD_ERR_NONE)
      {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
      SD_CARD_PRINTF_UTIL("\nCMD6 Check Func Failed!\n\r");
#endif
      return AM_HAL_STATUS_FAIL;
      }
    }

    //
    // CMD16 Set Block Length
    //
    if(sd_card_cmd16_set_blocklen(pCard,512)!=AM_HAL_CMD_ERR_NONE)
    {
#ifdef SD_CARD_DEBUG_PRINTF_ENABLED
    SD_CARD_PRINTF_UTIL("\nCMD16 Set Bus Width Failed!\n\r");
#endif
    return AM_HAL_STATUS_FAIL;
    }

return AM_HAL_STATUS_SUCCESS;
}


//=============================================================================
//
// SDIO: WRITE SINGLE BLOCK OF DATA
//
//=============================================================================
uint32_t sd_card_write_block(sd_card_host_t *pCard, uint32_t address, uint32_t size, uint8_t* data)
{
return sd_card_cmd24_write_single_block(pCard,size,address, data); 
}

//=============================================================================
//
// SDIO: READ SINGLE BLOCK OF DATA
//
//=============================================================================
uint32_t sd_card_read_block(sd_card_host_t *pCard, uint32_t address, uint32_t size, uint8_t* data)
{
return sd_card_cmd17_read_single_block(pCard,size,address, data);   
}





