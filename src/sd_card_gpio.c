//=============================================================================
// SD CARD driver example
// Mariusz Lacina, Ambiq, 2023
//=============================================================================

#include <stdbool.h>
#include <stdint.h>
#include "am_mcu_apollo.h"
#include "sd_card_gpio.h"
#include "am_util.h"

//=============================================================================
//
// SDIO PORTS
//
//=============================================================================
#define SDIO_D0_PIN   84
#define SDIO_D1_PIN   85
#define SDIO_D2_PIN   86
#define SDIO_D3_PIN   87
#define SDIO_CLK_PIN  88
#define SDIO_CMD_PIN  83
#define SDIO_PWR      56

//=============================================================================
//
// SDIO_PWR CONFIG
//
//=============================================================================
const am_hal_gpio_pincfg_t g_GPIO_56_SDIO_PWR =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_56_GPIO,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_1_5K, 
    .GP.cfg_b.uNCE                 = 0,
};

//=============================================================================
//
// SDIO_D0_PIN CONFIG
//
//=============================================================================
const am_hal_gpio_pincfg_t g_GPIO_84_SDIO_D0 =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_84_SDIF_DAT0, 
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_1P0X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE, 
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//=============================================================================
//
// SDIO_D1_PIN CONFIG
//
//=============================================================================
const am_hal_gpio_pincfg_t g_GPIO_85_SDIO_D1 =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_85_SDIF_DAT1,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_1P0X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//=============================================================================
//
// SDIO_D2_PIN CONFIG
//
//=============================================================================
const am_hal_gpio_pincfg_t g_GPIO_86_SDIO_D2 =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_86_SDIF_DAT2,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_1P0X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//=============================================================================
//
// SDIO_D3_PIN CONFIG
//
//=============================================================================
const am_hal_gpio_pincfg_t g_GPIO_87_SDIO_D3 =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_87_SDIF_DAT3,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_1P0X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//=============================================================================
//
// SDIO_CLK_PIN CONFIG
//
//=============================================================================
const am_hal_gpio_pincfg_t g_GPIO_88_SDIO_CLK =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_88_SDIF_CLKOUT,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_1P0X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//=============================================================================
//
// SDIO_CMD_PIN CONFIG
//
//=============================================================================
const am_hal_gpio_pincfg_t g_GPIO_83_SDIO_CMD =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_83_SDIF_CMD,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_1P0X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//=============================================================================
//
// GPIO OFF
//
//=============================================================================
const am_hal_gpio_pincfg_t g_GPIO_OFF =
{
    .GP.cfg_b.uFuncSel             = 3, //GPIO
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_NONE,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_1P0X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_NONE, 
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};

//=============================================================================
//
// SDIO gpio POWER DOWN
//
//=============================================================================
void sdio_gpio_power_down(void)
{
    am_hal_gpio_pinconfig(SDIO_D0_PIN,g_GPIO_OFF);
    am_hal_gpio_pinconfig(SDIO_D1_PIN,g_GPIO_OFF);
    am_hal_gpio_pinconfig(SDIO_D2_PIN,g_GPIO_OFF);
    am_hal_gpio_pinconfig(SDIO_D3_PIN,g_GPIO_OFF); 
    am_hal_gpio_pinconfig(SDIO_CLK_PIN,g_GPIO_OFF);
    am_hal_gpio_pinconfig(SDIO_CMD_PIN,g_GPIO_OFF);

    am_hal_gpio_output_clear(SDIO_PWR);     //switch off the card power 

    am_util_delay_ms(300);  
}


//=============================================================================
//
// SDIO gpio POWER UP
//
//=============================================================================
void sdio_gpio_power_up(void)
{
    am_hal_gpio_pinconfig(SDIO_PWR,g_GPIO_56_SDIO_PWR); 
    am_hal_gpio_output_clear(SDIO_PWR);     //switch off the card power

    am_hal_gpio_pinconfig(SDIO_D0_PIN,g_GPIO_84_SDIO_D0);
    am_hal_gpio_pinconfig(SDIO_D1_PIN,g_GPIO_85_SDIO_D1);
    am_hal_gpio_pinconfig(SDIO_D2_PIN,g_GPIO_86_SDIO_D2);
    am_hal_gpio_pinconfig(SDIO_D3_PIN,g_GPIO_87_SDIO_D3); 
    am_hal_gpio_pinconfig(SDIO_CLK_PIN,g_GPIO_88_SDIO_CLK);
    am_hal_gpio_pinconfig(SDIO_CMD_PIN,g_GPIO_83_SDIO_CMD);

    am_hal_gpio_output_set(SDIO_PWR);

    am_util_delay_ms(300);  
  
}


