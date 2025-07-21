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

// CONSTANTS
#define RX_FIFO_LIMIT                   20
#define TX_FIFO_LIMIT                   20

// Definite Values
#define VS32SPI_SPI_ENABLE          0x01

#define VS32SPI_SPI_BSY             0x800
#define VS32SPI_STATUS_RXNE         0x001
#define VS32SPI_STATUS_TX_RDY       0x00A

/* Timeouts we use, in number of status checks. */
#define TIMEOUT                       1000

/* #define DEBUG to make the return error codes provide enough information to
 * reconstruct the stack from where the error occurred. This is not enabled
 * usually to reduce the program size. */
#ifdef DEBUG
#define ERROR_STACK(x)              (x)
#define ERROR_VS32SPI_IDLE_WAIT   0x00010
#define ERROR_VS32SPI_TX          0x00100
#define ERROR_VS32SPI_RX          0x01000
#define ERROR_VS32SPI_WIP         0x50000
#else
#define ERROR_STACK(x)              0
#define ERROR_VS32SPI_IDLE_WAIT   1
#define ERROR_VS32SPI_TX          1
#define ERROR_VS32SPI_RX          1
#define ERROR_VS32SPI_WIP         1
#endif

#define ERROR_OK                    0

static void vs32spi_enable_txn(volatile uint32_t *ctrl_base);
static int  vs32spi_spi_idle(volatile uint32_t *ctrl_base);
static int  vs32spi_tx_data(volatile uint32_t *ctrl_base, uint32_t in);
static int  vs32spi_wip(volatile uint32_t *ctrl_base);
static void vs32spi_spi_init(volatile uint32_t *ctrl_base, uint16_t prescalar,
                        uint8_t mode, bool duplex, bool is_rx_1st, bool mstr,
                        uint8_t csid);
static void vs32spi_set_params(volatile uint32_t *ctrl_base, uint8_t tx_bits,
                        uint8_t rx_bits);
static int  vs32spi_write_buffer(volatile uint32_t *ctrl_base,
                        const uint8_t *buffer, unsigned offset, unsigned len,
                        uint32_t flash_info);

/* Can set bits 3:0 in result. */
/* flash_info contains:
 *   bits 7:0 -- pprog_cmd
 *   bit 8    -- 0 means send 3 bytes after pprog_cmd, 1 means send 4 bytes
 *               after pprog_cmd
 *   bits 31:16 -- prescalar
 */
int flash_vs32spi(volatile uint32_t *ctrl_base, uint32_t page_size,
        const uint8_t *buffer, unsigned offset, uint32_t count,
        uint32_t flash_info)
{
/************************************************
 *  1. Checkout whether SPI is busy ?
 *  2. Checkout If flash is free to be written.
 *  3. Get the Page offset and page size.
 *  4. Initiate writes to TXR register & Enable SPI
 *  5. Wait until the SPI gets disabled
 *     (Happens when written data is transferred)
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
/*STEP -3*/
    /* Assume page_size is always power of two */
    uint32_t page_offset = offset & (page_size - 1);

    /* Workaround: every odd (1,3,5) page in flash is unwritten, As work-around,
    every odd page write is iterated twice which is tracked with otrack */
    uint8_t otrack = 0;
/*STEP -4*/
    /* central part, aligned words */
    while (count > 0) {
        uint32_t cur_count;
        /* clip block at page boundary */
        if (page_offset + count > page_size)
            cur_count = page_size - page_offset;
        else
            cur_count = count;

        result = vs32spi_write_buffer(ctrl_base, buffer, offset, cur_count, flash_info);
        if (result != ERROR_OK) {
            result |= ERROR_STACK(0x3);
            goto err;
        }else
            ++otrack;

        page_offset = 0;
        buffer += ((otrack%2) == 0) ? 0 : cur_count;
        offset += ((otrack%2) == 0) ? 0 : cur_count;
        count  -= ((otrack%2) == 0) ? 0 : cur_count;
    }
/*STEP -5*/
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
static void vs32spi_enable_txn(volatile uint32_t *ctrl_base)
{
    vs32spi_write_reg(ctrl_base, VS32SPI_REG_EN, VS32SPI_SPI_ENABLE);
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
    vs32spi_write_reg(ctrl_base, VS32SPI_REG_CR1, ((ctrl_reg & 0x0000BFBC) | (duplex << 14 |
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
    uint32_t result = vs32spi_spi_idle(ctrl_base);
    if (result != ERROR_OK)
        return result | ERROR_STACK(0x10000);

    unsigned timeout = TIMEOUT;
    do{
        vs32spi_set_params(ctrl_base, 8, 8);
        vs32spi_tx_data(ctrl_base, SPIFLASH_READ_STATUS << 24);
        vs32spi_enable_txn(ctrl_base);
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

/* Can set bits 23:20 in result. */
static int vs32spi_write_buffer(volatile uint32_t *ctrl_base,
                        const uint8_t *buffer, unsigned offset, unsigned len,
                        uint32_t flash_info)
{
    unsigned timeout = TIMEOUT;
    unsigned trim_length = 0;
    vs32spi_set_params(ctrl_base, 8, 0);
/******************************************************************************
 *  1. Set WEL in Flash SR & set bits that are going to be xfered
 *  2. Load the initial address (4 Byte wide)
 *  3. Adjust the buffered data for 1 + 3 format.
 *  4. Enable the SPI for transfer
 *  5. Loop to load the data.
 *  6. Ensure to stop the SPI transfer appropriately
 *  7. exit once the SPI becomes IDLE.
 ******************************************************************************
 */
/* STEP -1 */
    int result = vs32spi_tx_data(ctrl_base, SPIFLASH_WRITE_ENABLE << 24);
    if (result != ERROR_OK)
        return result | ERROR_STACK(0x100000);
    vs32spi_enable_txn(ctrl_base);

    result = vs32spi_spi_idle(ctrl_base);
    if (result != ERROR_OK)
        return result | ERROR_STACK(0x200000);

    uint8_t bits_2_transfer = 40;
    if(len < 10 ){
        bits_2_transfer += (len * 8);
    } else {
        bits_2_transfer = 255;
    }

    vs32spi_set_params(ctrl_base, bits_2_transfer, 0);

/*STEP -2 */
    uint32_t cmd = (((flash_info & 0xff) << 24) | ((offset >> 8) & 0xffffff));

    result = vs32spi_tx_data(ctrl_base, cmd);
    if (result != ERROR_OK)
        return result | ERROR_STACK(0x300000);

/*STEP -3 */
    cmd = ((offset & 0xff) << 24) ;

    uint8_t init_switch = (len < 4) ? len : 3 ;
    switch(init_switch){
        case 3: cmd |= buffer[2]      ;
                __attribute__((fallthrough));
        case 2: cmd |= buffer[1] << 8 ;
                __attribute__((fallthrough));
        case 1: cmd |= buffer[0] << 16;
    }

    result = vs32spi_tx_data(ctrl_base, cmd);
    if (result != ERROR_OK)
        return result | ERROR_STACK(0x400000);

/*STEP -4*/
    /* Enable the SPI for possible transfer since unlimited data
       is going to be transferred to the SPI */
    if(bits_2_transfer == 255){
        vs32spi_enable_txn(ctrl_base);
    }

/*STEP -5*/
    /* WARNINGS:
        1. Below snippet will read beyond the specified buffer, and this was
    intentional to avoid speed drops that could happen with byte-wise writes.

        2. Although the buffer comes at byte wide accesibility, to reduce the
    load latency, the byte is RECASTED to loading word. This decision is taken
    purely on the performance grounds. It should be noted that while loading
    words, LITTLE ENDIAN ASSUMPTION IS CONSIDERED.

    While loop has been optimized to limit the load & stores that can happen
    while filling the Tx Register. Currently,4 aligned loads are being used.
    This loop atleast needs 600ns @50MHz in typical case (excluding the if case)

    Usage of register keyword is INTENTIONAL which allows compiler to take avbl.
    pool of temp. registers.
    */
    if(len > 3){
        register unsigned ii = 3;
        register uint32_t curr_data = 0;
        register uint32_t next_data = 0;
        const unsigned length = len;
        register uint32_t* location = (uint32_t*)(buffer);
        while (ii < length){
            curr_data = (*(location));
            next_data =  *(++location);
            next_data = (((next_data & 0xFF) << 16) |
                         ((next_data & 0xFF0000) >> 16)) |
                          (next_data & 0xFF00)  ;
            if((ii + 4) > length){
                trim_length = 3 - (length % 4);
                switch(trim_length){
                    case 3: next_data = 0; break;
                    case 2: next_data &= 0xFF0000; break;
                    case 1: next_data &= 0xFFFF00; break;
                }
            }
            while((ctrl_base[VS32SPI_REG_SR] & 0xA) != 0xA);
            ctrl_base[VS32SPI_REG_TX] = ((curr_data & 0xFF000000) | next_data);
            ii += 4;
        }
    }

    vs32spi_enable_txn(ctrl_base);

    while (timeout--) {
        uint32_t status;
        vs32spi_read_reg(ctrl_base, VS32SPI_REG_SR, &status);
/*STEP -7*/
        if ((status & VS32SPI_SPI_BSY) == 0)
            goto end_wip;
/*STEP -6*/
        if((((status & 0x00FF0000) >> 16) == (trim_length)) &&
           (bits_2_transfer == 255)){
            vs32spi_write_reg(ctrl_base, VS32SPI_REG_EN, 0);
        }
    }
    return result | ERROR_STACK(0x800000);

end_wip:
#if 0
    result = vs32spi_wip(ctrl_base);
    if (result != ERROR_OK)
        return result | ERROR_STACK(0x900000);
#endif
    return ERROR_OK;
}
