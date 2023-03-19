"#sd_card_ap4_plus" 

The example demonstrates initialization, write and read of SD CARD connected to Apollo4 Plus SDHC controller. The feature is still not supported
by the regular SDK4.3 (March/2023). Please check the latest SDK support before use. Following the card initialization, the example writes and verifies block of data. Results are printed out over the UART console. Cards tested: GOOD RAM 2GB, SANDISK microSDXC Ultra C10 64GB
The following features are implemented and tested:
1. Card initialization and data exchange: CMD0, CMD8, ACMD41, CMD2, CMD3, CMD9, CMD7, ACMD51, ACMD6, CMD16, CMD24, CMD17
2. Blocking DMA mode communication only.
3. The high speed clock was not tested due to unmatched long wires used on the handmade protoytpe board (see below).

Prototype board:
![board](https://user-images.githubusercontent.com/69169627/226170090-01f10c68-1e93-410a-b8b8-1613e9e38673.png)
The board includes: I/O logic level 1.8V/3.3V translators (2 x TXB0104 - an application dedicated device should be selected here e.g. NVT4857UK), 3.3V LDO + power switch controlled by SDIO_PWR port

Interconnections:

#define SDIO_D0_PIN   84
#define SDIO_D1_PIN   85
#define SDIO_D2_PIN   86
#define SDIO_D3_PIN   87
#define SDIO_CLK_PIN  88
#define SDIO_CMD_PIN  83
#define SDIO_PWR      56

Card initialization scope diagram:

![sd_init](https://user-images.githubusercontent.com/69169627/226170341-06f83fde-5c2f-45b2-8b62-35dec15a0acb.png)

Data block write & read diagram:

![block_write_read](https://user-images.githubusercontent.com/69169627/226170364-9f9e4255-563e-4e21-bdab-5ddf28e78c21.png)

UART console output:

![terminal_output](https://user-images.githubusercontent.com/69169627/226170381-a4cdc23c-0d24-4639-b71e-735146159ad1.jpg)

Project folders structure:

![proj_structure](https://user-images.githubusercontent.com/69169627/226171105-efa22bdd-5e02-4f98-bbfc-59cc5d4416c3.png)

The project can be compiled using the free Keil MDK Community Edition: https://www2.keil.com/mdk5/editions/community
