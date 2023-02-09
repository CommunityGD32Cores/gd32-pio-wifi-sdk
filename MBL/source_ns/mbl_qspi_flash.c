/*!
    \file    mbl_qspi_flash.c
    \brief   Non-secure MBL qspi flash file for GD32W51x WiFi SDK

    \version 2021-10-30, V1.0.0, firmware for GD32W51x
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32w51x.h"
#include "mbl_qspi_flash.h"
#include "platform_def.h"

/*!
    \brief      configure qspi flash gpio
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void qspi_flash_gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);

    gpio_af_set(GPIOA, GPIO_AF_3, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    gpio_af_set(GPIOB, GPIO_AF_3, GPIO_PIN_3 | GPIO_PIN_4);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3 | GPIO_PIN_4);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_3 | GPIO_PIN_4);
}

/*!
    \brief      enable qspi flash write
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void qspi_flash_write_enable(void)
{
    qspi_command_struct sCommand;
    qspi_autopolling_struct sConfig;

    /* Enable write operations ------------------------------------------ */
    sCommand.instruction_mode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.instruction        = WRITE_ENABLE_CMD;
    sCommand.addr_mode          = QSPI_ADDR_NONE;
    sCommand.addr_size          = QSPI_ADDR_24_BITS;
    sCommand.addr               = 0;
    sCommand.altebytes_mode     = QSPI_ALTE_BYTES_NONE;
    sCommand.altebytes_size     = QSPI_ALTE_BYTES_8_BITS;
    sCommand.altebytes          = 0;
    sCommand.dummycycles        = 0;
    sCommand.data_mode          = QSPI_DATA_NONE;
    sCommand.data_length        = 0;
    sCommand.sioo_mode          = QSPI_SIOO_INST_EVERY_CMD;

    qspi_command(&sCommand);

    /* Configure automatic polling mode to wait for write enabling ---- */
    sConfig.match               = 0x02;
    sConfig.mask                = 0x02;
    sConfig.match_mode          = QSPI_MATCH_MODE_AND;
    sConfig.statusbytes_size    = 1;
    sConfig.interval            = 0x10;
    sConfig.auto_stop           = QSPI_AUTO_STOP_ENABLE;

    sCommand.instruction        = READ_STATUS_REG1_CMD;
    sCommand.data_mode          = QSPI_DATA_1_LINE;

    qspi_autopolling(&sCommand, &sConfig);
}

/*!
    \brief      configure automatic polling mode to wait for memory ready
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void qspi_flash_autopolling_ready(void)
{
    qspi_command_struct sCommand;
    qspi_autopolling_struct sConfig;

    /* Configure automatic polling mode to wait for memory ready ------ */
    sCommand.instruction_mode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.instruction        = READ_STATUS_REG1_CMD;
    sCommand.addr_mode          = QSPI_ADDR_NONE;
    sCommand.addr_size          = QSPI_ADDR_24_BITS;
    sCommand.addr               = 0;
    sCommand.altebytes_mode     = QSPI_ALTE_BYTES_NONE;
    sCommand.altebytes_size     = QSPI_ALTE_BYTES_8_BITS;
    sCommand.altebytes          = 0;
    sCommand.dummycycles        = 0;
    sCommand.data_mode          = QSPI_DATA_1_LINE;
    sCommand.data_length        = 0;
    sCommand.sioo_mode          = QSPI_SIOO_INST_EVERY_CMD;

    sConfig.match               = 0x00;
    sConfig.mask                = 0x01;
    sConfig.match_mode          = QSPI_MATCH_MODE_AND;
    sConfig.statusbytes_size    = 1;
    sConfig.interval            = 0x10;
    sConfig.auto_stop           = QSPI_AUTO_STOP_ENABLE;

    qspi_autopolling(&sCommand, &sConfig);
}

static void qspi_tcfg_fmc_set(void)
{
    qspi_command_struct sCommand;

#if ( QSPI_FLASH_MODE == QSPI_FLASH_4_LINES )
    sCommand.instruction_mode   = (BITS(8,9) & ((uint32_t)(1) << 8));                            // 1 line
    sCommand.instruction        = (BITS(0,7) & (uint32_t)(QUAD_IO_FAST_READ_CMD));   // 0xeb
    sCommand.addr_mode          = (BITS(10,11) & ((uint32_t)(3) << 10));                         // 4 lines
    sCommand.addr_size          = (BITS(12,13) & ((uint32_t)(2) << 12));                          // 24 bits address
    sCommand.addr               = 0;
    sCommand.altebytes_mode     = (BITS(14,15) & ((uint32_t)(3) << 14));                         // 4 lines
    sCommand.altebytes_size     = 0;                            // 8 bits
    sCommand.altebytes          = 0;
    sCommand.dummycycles        = (BITS(18,22) & ((uint32_t)(4) << 18));                           // 4 dummy cycles
    sCommand.data_mode          = (BITS(24,25) & ((uint32_t)(3) << 24));    // 4 lines
    sCommand.data_length        = 0;
    sCommand.sioo_mode          = 0;
#elif ( QSPI_FLASH_MODE == QSPI_FLASH_2_LINES )
    sCommand.instruction_mode   = (BITS(8,9) & ((uint32_t)(1) << 8))(0x1);                            // 1 line
    sCommand.instruction        = (BITS(0,7) & (uint32_t)(DUAL_IO_FAST_READ_CMD));   // 0xBB
    sCommand.addr_mode          = (BITS(10,11) & ((uint32_t)(2) << 10));                         // 2 lines
    sCommand.addr_size          = (BITS(12,13) & ((uint32_t)(2) << 12));                          // 24 bits address
    sCommand.addr               = 0;
    sCommand.altebytes_mode     = (BITS(14,15) & ((uint32_t)(2) << 14));                         // 2 lines
    sCommand.altebytes_size     = 0;                            // 8 bits
    sCommand.altebytes          = 0;
    /*
        according to GD25Q16C datasheet, there are 4 alternate bits and 2 dummy cycles for 0xBB cmd,
        but GD32W51x QSPI mode has alternate bits's limit to 8bits, so set sCommand.altebytes_size
        to 8, so alternate data[3:0]'s transfer costs 2 cycles on two lines transfer, so set
        sCommand.dummycycles to 0(which not means there is no dummy cycles)
    */
    sCommand.dummycycles        = 0;                             // 0 dummy cycles
    sCommand.data_mode          = (BITS(24,25) & ((uint32_t)(2) << 24));                         // 2 lines
    sCommand.data_length        = 0;
    sCommand.sioo_mode          = 0;
#elif ( QSPI_FLASH_MODE == QSPI_FLASH_1_LINE )
    sCommand.instruction_mode   = (BITS(8,9) & ((uint32_t)(1) << 8))(0x1);                            // 1 line
    sCommand.instruction        = (BITS(0,7) & (uint32_t)(READ_DATA_BYTE_CMD));      // 0x03
    sCommand.addr_mode          = (BITS(10,11) & ((uint32_t)(1) << 10));                         // 1 line
    sCommand.addr_size          = (BITS(12,13) & ((uint32_t)(2) << 12));                          // 24 bits address
    sCommand.addr               = 0;
    sCommand.altebytes_mode     = 0;                           // no alternate byte
    sCommand.altebytes_size     = 0;                            // 8 bits
    sCommand.altebytes          = 0;
    sCommand.dummycycles        = 0;                             // 0 dummy cycles
    sCommand.data_mode          = (BITS(24,25) & ((uint32_t)(1) << 24));                         // 1 line
    sCommand.data_length        = 0;
    sCommand.sioo_mode          = 0;
#endif

    while((QSPI_CTL & QSPI_FLAG_BUSY) != 0U){
    }
    QSPI_TCFGF = sCommand.instruction_mode | sCommand.instruction | sCommand.addr_mode | \
           sCommand.addr_size | sCommand.addr | sCommand.altebytes_mode | sCommand.altebytes | \
           sCommand.dummycycles | sCommand.data_mode | sCommand.data_length | sCommand.sioo_mode;
}

/*!
    \brief      configure qspi flash to quad mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void qspi_flash_quad_enable_set(void)
{
    uint32_t id = 0;
    uint8_t mode_s = 0;
    uint16_t mode = 0;
    qspi_command_struct sCommand;

    qspi_flash_read_id(&id);

    qspi_flash_write_enable();

    sCommand.instruction_mode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.instruction        = 0;
    sCommand.addr_mode          = QSPI_ADDR_NONE;
    sCommand.addr_size          = QSPI_ADDR_24_BITS;
    sCommand.addr               = 0;
    sCommand.altebytes_mode     = QSPI_ALTE_BYTES_NONE;
    sCommand.altebytes_size     = QSPI_ALTE_BYTES_8_BITS;
    sCommand.altebytes          = 0;
    sCommand.dummycycles        = 0;
    sCommand.data_mode          = QSPI_DATA_1_LINE;
    sCommand.data_length        = 1;
    sCommand.sioo_mode          = QSPI_SIOO_INST_EVERY_CMD;

    sCommand.instruction        = READ_STATUS_REG1_CMD;
    qspi_command(&sCommand);
    qspi_receive(&mode_s);

    mode |= (uint16_t)mode_s;

    sCommand.instruction        = READ_STATUS_REG_CMD;
    qspi_command(&sCommand);
    qspi_receive(&mode_s);

    mode |= (uint16_t)mode_s << 8;

    if (mode & 0x0200) {
        /* quad mode, do nothing */
    } else {
        switch ((id & 0x00ff0000) >> 16) {
        case 0x16:                                       //GD32Q32
        case 0x17:                                       //GD32Q64
        case 0x18:                                       //GD32Q128
            mode = mode >> 8;
            mode |= 0x02;
            sCommand.instruction = WRITE_STATUS_REG_CMD;//write flash status[s15-s8]
            qspi_command(&sCommand);
            qspi_transmit((uint8_t *)&mode);
            break;
        case 0x14:                                       //GD32Q80
        case 0x15:                                       //GD32Q16
        case 0x19:                                       //GD32Q256
            /* To fully support GD32Q256, user has to set bit8(ADS) of Status Registers
                to enable 4-bytes Address Mode, which enabled by cmd 0xB7(the Enter 4-Byte
                Address Mode command enables accessing the address length of 32-bit for
                the memory area of higher density (larger than 128Mb), The device default
                is in 24-bit address mode); and then change all qspi related operations's
                address size from QSPI_ADDR_24_BITS to QSPI_ADDR_32_BITS. */
        default:
            mode |= (0x0200);
            sCommand.instruction = WRITE_STATUS_REG1_CMD;//write flash status[s15-s0]
            sCommand.data_length = 2;
            qspi_command(&sCommand);
            qspi_transmit((uint8_t *)&mode);
            break;
        }

        qspi_flash_autopolling_ready();
    }
}

/*!
    \brief      configure qspi flash
    \param[in]  clock_prescaler: prescaler of qspi clock
    \param[out] none
    \retval     none
*/
void qspi_flash_config(uint32_t clock_prescaler)
{
    qspi_init_struct Init;

    // rcu_periph_clock_enable(RCU_GTZC);
    // tzgpc_tzspc_peripheral_attributes_config(TZGPC_PERIPH_QSPI_FLASHREG, TZGPC_TZSPC_PERIPH_SEC);

    qspi_flash_gpio_config();

    // qspi_deinit();
    rcu_periph_clock_enable(RCU_QSPI);

    Init.prescaler              = clock_prescaler;  /* QSPI clock = AHBCLK/(ClockPrescaler+1) */
    Init.fifo_threshold         = 4;
    Init.sample_shift           = QSPI_SAMPLE_SHIFTING_HALFCYCLE;  // QSPI_SAMPLE_SHIFTING_NONE;
    Init.flash_size             = 23;  /* 2^(FlashSize+1) ***** number of address bits = FlashSize + 1*/
    Init.cs_high_time           = QSPI_CS_HIGH_TIME_1_CYCLE;
    Init.clock_mode             = QSPI_CLOCK_MODE_0;
    qspi_init(&Init);

    qspi_flash_quad_enable_set();
    qspi_tcfg_fmc_set();
}

/*!
    \brief      erase qspi flash sector
    \param[in]  address: qspi flash address
    \param[out] none
    \retval     0
*/
int32_t qspi_flash_sector_erase(uint32_t address)
{
    qspi_command_struct sCommand;

    qspi_flash_write_enable();
    sCommand.instruction_mode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.instruction        = SECTOR_ERASE_CMD;
    sCommand.addr_mode          = QSPI_ADDR_1_LINE;
    sCommand.addr_size          = QSPI_ADDR_24_BITS;
    sCommand.addr               = address;
    sCommand.altebytes_mode     = QSPI_ALTE_BYTES_NONE;
    sCommand.altebytes_size     = QSPI_ALTE_BYTES_8_BITS;
    sCommand.altebytes          = 0;
    sCommand.dummycycles        = 0;
    sCommand.data_mode          = QSPI_DATA_NONE;
    sCommand.data_length        = 0;
    sCommand.sioo_mode          = QSPI_SIOO_INST_EVERY_CMD;
    qspi_command(&sCommand);
    qspi_flash_autopolling_ready();

    return 0;
}

/*!
    \brief      erase qspi flash full chip
    \param[in]  none
    \param[out] none
    \retval     0
*/
int32_t qspi_flash_chip_erase(void)
{
    qspi_command_struct sCommand;

    qspi_flash_write_enable();
    sCommand.instruction_mode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.instruction        = CHIP_ERASE_CMD;
    sCommand.addr_mode          = QSPI_ADDR_NONE;
    sCommand.addr_size          = QSPI_ADDR_8_BITS;
    sCommand.addr               = 0;
    sCommand.altebytes_mode     = QSPI_ALTE_BYTES_NONE;
    sCommand.altebytes_size     = QSPI_ALTE_BYTES_8_BITS;
    sCommand.altebytes          = 0;
    sCommand.dummycycles        = 0;
    sCommand.data_mode          = QSPI_DATA_NONE;
    sCommand.data_length        = 0;
    sCommand.sioo_mode          = QSPI_SIOO_INST_EVERY_CMD;
    qspi_command(&sCommand);
    qspi_flash_autopolling_ready();

    return 0;
}

/*!
    \brief      read qspi flash device id
    \param[in]  None
    \param[out] None
    \retval     0
*/
int32_t qspi_flash_read_id(void *id)
{
    qspi_command_struct sCommand = {0};

    sCommand.instruction_mode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.instruction        = CHIP_READ_ID_CMD;
    sCommand.dummycycles        = 0;
    sCommand.data_mode          = QSPI_DATA_1_LINE;
    sCommand.data_length        = 3;
    sCommand.sioo_mode          = QSPI_SIOO_INST_EVERY_CMD;
    qspi_command(&sCommand);
    qspi_receive(id);

    return 0;
}

/*!
    \brief      read qspi flash
    \param[in]  adress: flash's internal address to read from
    \param[out] data: pointer to the buffer that receives the data read from the flash
    \param[in]  size: size of data read from flash
    \retval     0
*/
int32_t qspi_flash_read(uint32_t address, void *data, uint32_t size)
{
    qspi_command_struct sCommand;

#if ( QSPI_FLASH_MODE == QSPI_FLASH_4_LINES )
    sCommand.instruction_mode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.instruction        = QUAD_IO_FAST_READ_CMD;
    sCommand.addr_mode          = QSPI_ADDR_4_LINES;
    sCommand.addr_size          = QSPI_ADDR_24_BITS;
    sCommand.addr               = address;
    sCommand.altebytes_mode     = QSPI_ALTE_BYTES_4_LINES;
    sCommand.altebytes_size     = QSPI_ALTE_BYTES_8_BITS;
    sCommand.altebytes          = 0;
    sCommand.dummycycles        = 4;
    sCommand.data_mode          = QSPI_DATA_4_LINES;
    sCommand.data_length        = size;
    sCommand.sioo_mode          = QSPI_SIOO_INST_EVERY_CMD;
#elif ( QSPI_FLASH_MODE == QSPI_FLASH_2_LINES )
    sCommand.instruction_mode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.instruction        = DUAL_IO_FAST_READ_CMD;
    sCommand.addr_mode          = QSPI_ADDR_2_LINES;
    sCommand.addr_size          = QSPI_ADDR_24_BITS;
    sCommand.addr               = address;
    sCommand.altebytes_mode     = QSPI_ALTE_BYTES_2_LINES;
    sCommand.altebytes_size     = QSPI_ALTE_BYTES_8_BITS;
    sCommand.altebytes          = 0;
    /*
        according to GD25Q16C datasheet, there are 4 alternate bits and 2 dummy cycles for 0xBB cmd,
        but GD32W51x QSPI mode has alternate bits's limit to 8bits, so set sCommand.altebytes_size
        to 8, so alternate data[3:0]'s transfer costs 2 cycles on two lines transfer, so set
        sCommand.dummycycles to 0(which not means there is no dummy cycles)
    */
    sCommand.dummycycles        = 0;
    sCommand.data_mode          = QSPI_DATA_2_LINES;
    sCommand.data_length        = size;
    sCommand.sioo_mode          = QSPI_SIOO_INST_EVERY_CMD;
#elif ( QSPI_FLASH_MODE == QSPI_FLASH_1_LINE )
    sCommand.instruction_mode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.instruction        = READ_DATA_BYTE_CMD;
    sCommand.addr_mode          = QSPI_ADDR_1_LINE;
    sCommand.addr_size          = QSPI_ADDR_24_BITS;
    sCommand.addr               = address;
    sCommand.altebytes_mode     = QSPI_ALTE_BYTES_NONE;
    sCommand.altebytes_size     = QSPI_ALTE_BYTES_8_BITS;
    sCommand.altebytes          = 0;
    sCommand.dummycycles        = 0;
    sCommand.data_mode          = QSPI_DATA_1_LINE;
    sCommand.data_length        = size;
    sCommand.sioo_mode          = QSPI_SIOO_INST_EVERY_CMD;
#endif

    qspi_command(&sCommand);
    qspi_receive(data);

    return 0;
}

/*!
    \brief      program qspi flash page
    \param[in]  adress: flash's internal address to write to
    \param[in]  data: pointer to the buffer that receives the data read from the flash
    \param[in]  size: size of data read from flash
    \param[out] none
    \retval     0
*/
int32_t qspi_flash_page_program(uint32_t address, const uint8_t *data, uint32_t size)
{
    qspi_command_struct sCommand;

    /* Writing Sequence 4 Line------------------------------------------ */
    qspi_flash_write_enable();

#if ( QSPI_FLASH_MODE == QSPI_FLASH_4_LINES )
    sCommand.instruction_mode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.instruction        = QUAD_IN_FAST_PROG_CMD;
    sCommand.addr_mode          = QSPI_ADDR_1_LINE;
    sCommand.addr_size          = QSPI_ADDR_24_BITS;
    sCommand.addr               = address;
    sCommand.altebytes_mode     = QSPI_ALTE_BYTES_NONE;
    sCommand.altebytes_size     = QSPI_ALTE_BYTES_8_BITS;
    sCommand.altebytes          = 0;
    sCommand.dummycycles        = 0;
    sCommand.data_mode          = QSPI_DATA_4_LINES;
    sCommand.data_length        = size;
    sCommand.sioo_mode          = QSPI_SIOO_INST_EVERY_CMD;
#elif ( QSPI_FLASH_MODE == QSPI_FLASH_2_LINES ) || ( QSPI_FLASH_MODE == QSPI_FLASH_1_LINE )
    sCommand.instruction_mode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.instruction        = PAGE_PROG_CMD;
    sCommand.addr_mode          = QSPI_ADDR_1_LINE;
    sCommand.addr_size          = QSPI_ADDR_24_BITS;
    sCommand.addr               = address;
    sCommand.altebytes_mode     = QSPI_ALTE_BYTES_NONE;
    sCommand.altebytes_size     = QSPI_ALTE_BYTES_8_BITS;
    sCommand.altebytes          = 0;
    sCommand.dummycycles        = 0;
    sCommand.data_mode          = QSPI_DATA_1_LINE;
    sCommand.data_length        = size;
    sCommand.sioo_mode          = QSPI_SIOO_INST_EVERY_CMD;
#endif
    qspi_command(&sCommand);
    qspi_transmit((uint8_t *)data);
    qspi_flash_autopolling_ready();

    return 0;
}
