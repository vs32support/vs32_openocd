/******************************************************************************
 *   This program is distributed in the hope that it will be useful,          *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                     *
 *   See LICENSE for more info.                                               *
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../../../../src/flash/nor/spi.h"

/* Register offsets */

#define VS32SPI_REG_CR1             0x00
#define VS32SPI_REG_CR2             0x01
#define VS32SPI_REG_EN              0x02
#define VS32SPI_REG_SR              0x03
#define VS32SPI_REG_TX              0x04
#define VS32SPI_REG_RX              0x05
#define VS32SPI_REG_PSCR            0x09

// Definite Values
#define VS32SPI_SPI_ENABLE          0x01
#define VS32SPI_SPI_DISABLE         0x00

#define VS32SPI_SPI_BSY             0x800
#define VS32SPI_STATUS_RXNE         0x001
#define VS32SPI_STATUS_TX_RDY       0x00A

/* Timeouts we use, in number of status checks. */
#define TIMEOUT                       1000

/* #define DEBUG to make the return error codes provide enough information to
 * reconstruct the stack from where the error occurred. This is not enabled
 * usually to reduce the program size. */
// #ifdef DEBUG
#define ERROR_STACK(x)              (x)
#define ERROR_VS32SPI_IDLE_WAIT   0x00010
#define ERROR_VS32SPI_TX          0x00100
#define ERROR_VS32SPI_RX          0x01000
#define ERROR_VS32SPI_WIP         0x50000
// #else
// #define ERROR_STACK(x)              0
// #define ERROR_VS32SPI_IDLE_WAIT   1
// #define ERROR_VS32SPI_TX          1
// #define ERROR_VS32SPI_RX          1
// #define ERROR_VS32SPI_WIP         1
// #endif

#define ERROR_OK                    0

static void vs32spi_enable_txn(volatile uint32_t *ctrl_base, uint8_t enable);
static int  vs32spi_spi_idle(volatile uint32_t *ctrl_base);
static int  vs32spi_tx_data(volatile uint32_t *ctrl_base, uint32_t in);
static int  vs32spi_wip(volatile uint32_t *ctrl_base);
static void vs32spi_spi_init(volatile uint32_t *ctrl_base, uint16_t prescalar,
                        uint8_t mode, bool duplex, bool is_rx_1st, bool mstr,
                        uint8_t csid);
static void vs32spi_set_params(volatile uint32_t *ctrl_base, uint8_t tx_bits,
                        uint8_t rx_bits);
static uint32_t vs32spi_block_read(volatile uint32_t *ctrl_base,
                        uint32_t offset, const unsigned int sz,
                        uint32_t* dest, uint32_t flash_info);

/* Can set bits 3:0 in result. */
/* flash_info contains:
 *   bits 7:0 -- pprog_cmd
 *   bit 8    -- 0 means send 3 bytes after pprog_cmd, 1 means send 4 bytes
 *               after pprog_cmd
 *   bits 31:16 -- prescalar
 */
//dest <= offset[0 ... count-1];
int flash_vs32spi_rd(volatile uint32_t *ctrl_base, unsigned offset,
        uint32_t count, uintptr_t dest, uint32_t flash_info )
{
/************************************************
 *  1. Checkout whether SPI is busy ?
 *  2. Checkout If flash is free to be read.
 *  3. Set CRC variables
 *  4. From Offset read count bytes and discard the result.
 *  5. Read CRC finally and return
 ***********************************************/
    int result;
/*STEP -1*/
    result = vs32spi_spi_idle(ctrl_base);
    if (result != ERROR_OK)
        return result | ERROR_STACK(0x1);

    vs32spi_spi_init(ctrl_base, ((flash_info & 0xFFFF0000) >> 16), 0, false,
                                  false, true, 0);
/*STEP -2*/
    result = vs32spi_wip(ctrl_base);
    if (result != ERROR_OK) {
        result |= ERROR_STACK(0x2);
        goto err;
    }
/*STEP -3/4/5*/
    result = vs32spi_block_read(ctrl_base, offset, count, (uint32_t*)dest, flash_info);
err:
    return result;
}

static void vs32spi_write_reg(volatile uint32_t *ctrl_base, unsigned address,
                                uint32_t value)
{
    ctrl_base[address] = value;
}
static void vs32spi_read_reg(volatile uint32_t *ctrl_base, unsigned address,
                                uint32_t* value)
{
    *value = ctrl_base[address];
}
static void vs32spi_enable_txn(volatile uint32_t *ctrl_base, uint8_t enable)
{
    vs32spi_write_reg(ctrl_base, VS32SPI_REG_EN, enable);
}

/* Can set bits 7:4 in result. */
static int vs32spi_spi_idle(volatile uint32_t *ctrl_base)
{
    unsigned timeout = TIMEOUT;
    while (timeout--) {
        uint32_t status;
        vs32spi_read_reg(ctrl_base, VS32SPI_REG_SR, &status);
        if ((status & VS32SPI_SPI_BSY) == 0)
            return ERROR_OK;
    }
    return ERROR_VS32SPI_IDLE_WAIT;
}

static void vs32spi_spi_init(volatile uint32_t *ctrl_base, uint16_t prescalar,
                                uint8_t mode, bool duplex, bool is_rx_1st,
                                bool mstr, uint8_t csid)
{
    vs32spi_write_reg(ctrl_base, VS32SPI_REG_PSCR, prescalar);
    uint32_t ctrl_reg;

    // Clear the Internal States, Set CPOL CPHA NON_DUPLEX & Half duplex first
    vs32spi_read_reg(ctrl_base, VS32SPI_REG_CR1, &ctrl_reg);
    vs32spi_write_reg(ctrl_base, VS32SPI_REG_CR1, ((ctrl_reg & 0x0000BFBE) | (duplex << 14 |
                         1 << 10 | is_rx_1st << 6 | mstr << 2 | (mode & 3))));

    vs32spi_read_reg(ctrl_base, VS32SPI_REG_CR2, &ctrl_reg);
    vs32spi_write_reg(ctrl_base, VS32SPI_REG_CR2, ((ctrl_reg & 0xF8FFFFFF) | (csid << 24)));
}

static void vs32spi_set_params(volatile uint32_t *ctrl_base, uint8_t tx_bits,
                                 uint8_t rx_bits)
{
    uint32_t fmt;
    vs32spi_read_reg(ctrl_base, VS32SPI_REG_CR1, &fmt);
    fmt = ((fmt & 0x0000FFBF) | (rx_bits << 24 | tx_bits << 16 | 1 << 15 | 1 << 10 ));
    vs32spi_write_reg(ctrl_base, VS32SPI_REG_CR1, fmt);
}

/* Can set bits 11:8 in result. */
static int vs32spi_tx_data(volatile uint32_t *ctrl_base, uint32_t in)
{
    unsigned timeout = TIMEOUT;
    while (timeout--) {
        uint32_t txstatus;
        vs32spi_read_reg(ctrl_base, VS32SPI_REG_SR, &txstatus);
        if ((txstatus & VS32SPI_STATUS_TX_RDY) == VS32SPI_STATUS_TX_RDY){
            vs32spi_write_reg(ctrl_base, VS32SPI_REG_TX, in);
            return ERROR_OK;
        }
    }
    return ERROR_VS32SPI_TX;
}

/* Can set bits 19:16 and 15:12 in result. */
static int vs32spi_wip(volatile uint32_t *ctrl_base)
{
    unsigned timeout = TIMEOUT;
    do{
        uint32_t result;
        vs32spi_set_params(ctrl_base, 8, 8);
        vs32spi_tx_data(ctrl_base, SPIFLASH_READ_STATUS << 24);
        vs32spi_enable_txn(ctrl_base, VS32SPI_SPI_ENABLE);
        unsigned timein = TIMEOUT;
        while (timein--) {
            vs32spi_read_reg(ctrl_base, VS32SPI_REG_SR, &result);
            if (((result & VS32SPI_STATUS_RXNE) != 0)) {
                vs32spi_read_reg(ctrl_base, VS32SPI_REG_RX, &result);
                goto check_wip;
            }
        }
        return (ERROR_VS32SPI_RX | ERROR_STACK(0x40000));
check_wip:
        // Catch Program Error from Status Register if at all happens
        if (((result >> 24) & 0x41) == 0x41) {
            return result | ERROR_STACK(0x80000);
        }
        if (((result >> 24) & SPIFLASH_BSY_BIT) == 0) {
            return ERROR_OK;
        }
    }while (timeout--);

    return ERROR_VS32SPI_WIP;
}

static uint32_t vs32spi_block_read(volatile uint32_t *ctrl_base,
                        uint32_t offset, const unsigned int limit,
                        uint32_t *dest, uint32_t flash_info)
{
    uint32_t cmd_addr = (((flash_info & 0xff) << 24) | ((offset >> 8) & 0xffffff));
    uint32_t address = ((offset & 0xff) << 24) ;
    // command (8) + addr (32) + dummy(CR2NV[3:0] = 8)
    // Receive unlimited data continously
    vs32spi_set_params(ctrl_base, 48 , 255);
    vs32spi_tx_data(ctrl_base, cmd_addr);
    vs32spi_tx_data(ctrl_base, address);
    vs32spi_enable_txn(ctrl_base, VS32SPI_SPI_ENABLE);
    register volatile uint32_t *loc = (uint32_t *)dest;
    register unsigned int indx = 0;
    register unsigned int sz = limit;
    while(indx < sz){
        while(!(ctrl_base[VS32SPI_REG_SR] & VS32SPI_STATUS_RXNE));
        *(loc++) = ctrl_base[VS32SPI_REG_RX];
        indx += 4;
    }
    ctrl_base[VS32SPI_REG_EN] = VS32SPI_SPI_DISABLE;
    vs32spi_set_params(ctrl_base, 0 , 0); // Clear SPI Internals.
    return ERROR_OK;
}
