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
#include "sd_card_gpio.h"


//=============================================================================
//
// Memory config
//
//=============================================================================
void memory_config(void)
{
			
		am_hal_pwrctrl_mcu_memory_config_t McuMemCfg =
    {
      .eCacheCfg    = AM_HAL_PWRCTRL_CACHE_ALL,
      .bRetainCache = false,
      .eDTCMCfg     = AM_HAL_PWRCTRL_DTCM_384K,
      .eRetainDTCM  = AM_HAL_PWRCTRL_DTCM_384K,
      .bEnableNVM0  = true,
      .bRetainNVM0  = false
    };

    am_hal_pwrctrl_sram_memcfg_t SRAMMemCfg =
    {
      .eSRAMCfg         = AM_HAL_PWRCTRL_SRAM_ALL,
      .eActiveWithMCU   = AM_HAL_PWRCTRL_SRAM_NONE,
      .eActiveWithDSP   = AM_HAL_PWRCTRL_SRAM_NONE,
      .eSRAMRetain      = AM_HAL_PWRCTRL_SRAM_ALL
    };
		
		am_hal_pwrctrl_dsp_memory_config_t ExtSRAMMemCfg =
    {
    .bEnableICache      = false,  //Should always be "false"
    .bRetainCache       = false,  //Should always be "false"
    .bEnableRAM         = false,  //Controls Extended RAM power when MCU awake
    .bActiveRAM         = false,  //Should be "false"
    .bRetainRAM         = false 	//true configures Extended RAM to be retained in deep sleep
    };
		
		
    am_hal_pwrctrl_mcu_memory_config(&McuMemCfg);
    am_hal_pwrctrl_sram_config(&SRAMMemCfg);
		am_hal_pwrctrl_dsp_memory_config(AM_HAL_DSP0, &ExtSRAMMemCfg);
		am_hal_pwrctrl_dsp_memory_config(AM_HAL_DSP1, &ExtSRAMMemCfg);	
}

extern sd_card_host_t sd_card_host;






//=============================================================================
//
// Main function.
//
//=============================================================================
int main(void)
{
uint8_t ui8TxRxDelays[2];
uint32_t status;
uint8_t buf[512], ver_buf[512]; 
uint32_t  address, size;
bool ver_error;

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();
	
		//
		// Configure the memory 
		//
		memory_config();
		
    //
		// Configure GPIO pins 
		//
    sdio_gpio_power_up();

    //
    // Global interrupt enable
    //
    am_hal_interrupt_master_enable();

    //
    // Enable printing to the console.
    //
    am_bsp_debug_printf_enable();


    do{
    //
    // Init the card host
    //
    sd_card_host_init(&sd_card_host,true);

    am_util_delay_ms(10);  

    if (sd_card_host.host.pHandle == NULL)
    {
    am_util_stdio_printf("\nSD Card Host NOT Initialised!\n");
    break;
    }
    am_util_stdio_printf("\nSD Card Host Initialised !\n");
  

    status = sd_card_init(&sd_card_host);
    if(status != AM_HAL_STATUS_SUCCESS)
    {
    am_util_stdio_printf("\nSD Card NOT Initialised!\n\n");
    break;
    }
    am_util_stdio_printf("\nCard configured !\n\n\r");
 

    }while(0);


//=============================================================================
//
// SD CARD READ TESTS - 4 DATA LINE MODE
//
//uint32_t sd_card_read_block(sd_card_host_t *pCard, uint32_t address, uint32_t size, uint8_t* data);
//uint32_t sd_card_write_block(sd_card_host_t *pCard, uint32_t address, uint32_t size, uint8_t* data);
//=============================================================================
    if(status == AM_HAL_STATUS_SUCCESS)
    {

    address = 0x1000;
    size = 512;
    memset(buf,0,sizeof(buf));
    memset(ver_buf,0,sizeof(ver_buf));

    for(int i=0;i<512;i++)buf[i] = i;

   
    am_util_stdio_printf("Writing %uB, address:0x%x \n\r",size,address);
    status = sd_card_write_block(&sd_card_host, address, size, buf);
    
    am_util_stdio_printf("reading %uB, address:0x%x \n\r",size,address);
    status = sd_card_read_block(&sd_card_host, address, size, ver_buf);

    ver_error = false;
    for(int i=0;i<size;i++)
    {
    if(buf[i]!=ver_buf[i])ver_error=true;
    i=size;
    }

    if(ver_error == false)
    am_util_stdio_printf("\n\rVerification succesfull!\n\n\r");    
    else
    am_util_stdio_printf("\n\rVerification error!\n\n\r");  


    address += 512;

    
    am_util_stdio_printf("Writing %uB, address:0x%x \n\r",size,address);
    status = sd_card_write_block(&sd_card_host, address, size, buf);
    
    am_util_stdio_printf("reading %uB, address:0x%x \n\r",size,address);
    status = sd_card_read_block(&sd_card_host, address, size, ver_buf);

    ver_error = false;
    for(int i=0;i<size;i++)
    {
    if(buf[i]!=ver_buf[i])ver_error=true;
    i=size;
    }

    if(ver_error == false)
    am_util_stdio_printf("\n\rVerification succesfull!\n\n\r");    
    else
    am_util_stdio_printf("\n\rVerification error!\n\n\r");  



    am_util_stdio_printf("\nCard test finished !!! \n\r");
   

    }

    sdio_gpio_power_down();

		//
    // Loop forever.
    //
    while (1)
    {
    
        //
        // Go to Deep Sleep.
        //
        //am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

		
    }
} // main()
