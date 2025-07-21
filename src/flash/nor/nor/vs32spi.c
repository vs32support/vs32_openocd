/*****************************************************************************
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>         *
 *   Modified by vervesemi from the original stmsmi.c *
 *                                                                           *
 *   This program is free software; you can redistribute it and/or modify    *
 *   it under the terms of the GNU General Public License as published by    *
 *   the Free Software Foundation; either version 2 of the License, or       *
 *   (at your option) any later version.                                     *
 *                                                                           *
 *   This program is distributed in the hope that it will be useful,         *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *   GNU General Public License for more details.                            *
 *                                                                           *
 *   You should have received a copy of the GNU General Public License       *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.   *
 *****************************************************************************/

/* The SPI bus controller provided by Incore Semiconductors Pvt. Ltd. is
 * a General purpose SPI that can also be used to interface SPI Flash Memories.
 */

/* ATTENTION:
 * This Controller assumes
 * 1) there's a FIFO of 16 bytes storage capacity.
 * 2) SPI-0 with Chip Select 0 is used in Controller (Master) mode
 * 3) MSB first transfer with Half duplex communication
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include "target/riscv/riscv.h"
#include "vs32spi.h"

/*
 * Add the pseudo keyword 'fallthrough' so case statement blocks
 * must end with any of these keywords:
 *   break;
 *   fallthrough;
 *   goto <label>;
 *   return [expression];
 *
 *  gcc: https://gcc.gnu.org/onlinedocs/gcc/Statement-Attributes.html#Statement-Attributes
 */
#if __has_attribute(__fallthrough__)
# define fallthrough                    __attribute__((__fallthrough__))
#else
# define fallthrough                    do {} while (0)  /* fallthrough */
#endif

struct vs32spi_flash_bank {
    bool probed;
    target_addr_t ctrl_base;
    const struct flash_device *dev;
};

struct vs32spi_target {
    char *name;
    uint32_t tap_idcode;
    uint32_t ctrl_base;
};

static const struct vs32spi_target target_devices[] = {
    /* name,   tap_idcode, ctrl_base */
    { "VS32 MCU Flasher", 0x13631093, 0x00020100 },
    { NULL, 0, 0 }
};

FLASH_BANK_COMMAND_HANDLER(vs32spi_flash_bank_command)
{
    struct vs32spi_flash_bank *vs32spi_info;

    LOG_DEBUG("%s", __func__);

    if (CMD_ARGC < 6)
        return ERROR_COMMAND_SYNTAX_ERROR;

    vs32spi_info = malloc(sizeof(struct vs32spi_flash_bank));
    if (vs32spi_info == NULL) {
        LOG_ERROR("not enough memory");
        return ERROR_FAIL;
    }

    bank->driver_priv = vs32spi_info;
    vs32spi_info->probed = false;
    vs32spi_info->ctrl_base = 0;
    if (CMD_ARGC >= 7) {
        COMMAND_PARSE_ADDRESS(CMD_ARGV[6], vs32spi_info->ctrl_base);
        LOG_DEBUG("ASSUMING VS32SPI Controller is at ctrl_base = " TARGET_ADDR_FMT,
            vs32spi_info->ctrl_base);
    }

    return ERROR_OK;
}

static int vs32spi_read_reg(struct flash_bank *bank, uint32_t *value, target_addr_t address)
{
    struct target *target = bank->target;
    struct vs32spi_flash_bank *vs32spi_info = bank->driver_priv;

    int result = target_read_u32(target, vs32spi_info->ctrl_base + address, value);
    if (result != ERROR_OK) {
        LOG_ERROR("vs32spi_read_reg() %d error at " TARGET_ADDR_FMT,
                result, vs32spi_info->ctrl_base + address);
        return result;
    }
    return ERROR_OK;
}

static int vs32spi_write_reg(struct flash_bank *bank, target_addr_t address, uint32_t value)
{
    struct target *target = bank->target;
    struct vs32spi_flash_bank *vs32spi_info = bank->driver_priv;

    int result = target_write_u32(target, vs32spi_info->ctrl_base + address, value);
    if (result != ERROR_OK) {
        LOG_ERROR("vs32spi_write_reg() error writing 0x%" PRIx32 " to " TARGET_ADDR_FMT,
                value, vs32spi_info->ctrl_base + address);
        return result;
    }
    return ERROR_OK;
}

static int vs32spi_enable_spi(struct flash_bank *bank)
{
    return vs32spi_write_reg(bank, VS32SPI_REG_EN, VS32SPI_SPI_ENABLE);
}

static int vs32spi_select_csid(struct flash_bank *bank, uint8_t id)
{
    uint32_t cr2;
    if (vs32spi_read_reg(bank, &cr2, VS32SPI_REG_CR2) != ERROR_OK)
        return ERROR_FAIL;
    return vs32spi_write_reg(bank, VS32SPI_REG_CR2, ((cr2 & 0xF8FFFFFF) | (id << 24)));
}

static int vs32spi_spi_init(struct flash_bank *bank,uint16_t prescale,
                              uint8_t mode, bool duplex, bool is_rx_1st,
                              bool is_mstr )
{
    uint32_t cr1;
    vs32spi_write_reg(bank, VS32SPI_REG_PSCR, prescale);
    if (vs32spi_read_reg(bank, &cr1, VS32SPI_REG_CR1) != ERROR_OK)
        return ERROR_FAIL;

    return vs32spi_write_reg(bank, VS32SPI_REG_CR1, ((cr1 & 0x0000BFB8) |
                                   (duplex << 14 | 1 << 10 | is_rx_1st << 6
                                   | is_mstr << 2 | mode)));
}

static int vs32spi_set_params(struct flash_bank *bank, uint8_t tx_bits,
                                uint8_t rx_bits, bool lsb_first)
{
    uint32_t cr1;
    if (vs32spi_read_reg(bank, &cr1, VS32SPI_REG_CR1) != ERROR_OK)
        return ERROR_FAIL;

    return vs32spi_write_reg(bank, VS32SPI_REG_CR1,
            ((cr1 & 0x0000FFBF) | (rx_bits << 24 | tx_bits << 16 | 1 << 10 | lsb_first << 6)));
}

#if DEBUG
void vs32spi_indicate_loading(void)
{
  static int loc = 0;
  char cursor_list[8] = {'|', '|', '/', '/', '~', '~', '\\', '\\'};
  printf("%c\b", cursor_list[loc]);
  loc = (loc + 1) % 8;
  fflush(stdout);
}
#endif

static int vs32spi_spi_wait(struct flash_bank *bank, int timeout)
{
    int64_t endtime = timeval_ms() + timeout;
    uint32_t status;

    while (1) {
        if (vs32spi_read_reg(bank, &status, VS32SPI_REG_SR) != ERROR_OK)
            return ERROR_FAIL;
        if ((status & VS32SPI_SPI_BSY) == 0){
            break;
        }
        if (timeval_ms() > endtime) {
            LOG_ERROR("SPI is still busy.");
            return ERROR_TARGET_TIMEOUT;
        }
    }
    return ERROR_OK;
}

static int vs32spi_tx(struct flash_bank *bank, uint32_t in)
{
    int64_t endtime = timeval_ms() + VS32SPI_CMD_TIMEOUT;
    uint32_t tx_status;
    while (1) {
        if (vs32spi_read_reg(bank, &tx_status, VS32SPI_REG_SR) != ERROR_OK)
            return ERROR_FAIL;
        if ((VS32SPI_STATUS_TXFIFO(tx_status) <= VS32_TX_FIFO_LIMIT) &&
                ((tx_status & VS32SPI_STATUS_TXE) != 0)){
            break;
        }
        if (timeval_ms() > endtime) {
            LOG_ERROR("Enqueue failed in txfifo (value=0x%" PRIx32 ").", (tx_status >> 16));
            return ERROR_TARGET_TIMEOUT;
        }
    }
    LOG_DEBUG("%s: status=0x%08" PRIx32 " data=0x%08" PRIx32, __func__, tx_status, in);
    return vs32spi_write_reg(bank, VS32SPI_REG_TX, in);
}

static int vs32spi_rx(struct flash_bank *bank, uint32_t *out)
{
    int64_t endtime = timeval_ms() + VS32SPI_CMD_TIMEOUT;
    uint32_t rx_status;
    uint32_t value;

    while (1) {
        if (vs32spi_read_reg(bank, &rx_status, VS32SPI_REG_SR) != ERROR_OK)
            return ERROR_FAIL;

        if ((rx_status & VS32SPI_STATUS_RXNE)){
            vs32spi_read_reg(bank, &value, VS32SPI_REG_RX);
            break;
        }
        if (timeval_ms() > endtime) {
            LOG_ERROR("rxfifo hasn't enough to receive (value=0x%" PRIx32 ").", value  >> 24);
            return ERROR_TARGET_TIMEOUT;
        }
    }

    if (out)
        *out = value;
    LOG_DEBUG("%s: status=0x%08" PRIx32 " data=0x%08" PRIx32, __func__, rx_status, value);
    return ERROR_OK;
}

static int vs32spi_read_flash_status(struct flash_bank *bank, uint32_t *status)
{

    if (vs32spi_select_csid(bank, 0) != ERROR_OK)
        return ERROR_FAIL;
    if (vs32spi_set_params(bank, 8, 8, false) != ERROR_OK)
        return ERROR_FAIL;
    if (vs32spi_tx(bank, SPIFLASH_READ_STATUS << 24) != ERROR_OK)
        return ERROR_FAIL;
    if (vs32spi_enable_spi(bank) != ERROR_OK)
        return ERROR_FAIL;
    if (vs32spi_rx(bank, status) != ERROR_OK)
        return ERROR_FAIL;
    else
        *status = *status >> 24;
    if (vs32spi_spi_wait(bank, VS32SPI_CMD_TIMEOUT) != ERROR_OK)
        return ERROR_FAIL;
    return ERROR_OK;
}

static int vs32spi_flash_wip_ready(struct flash_bank *bank, int timeout)
{
    int64_t endtime = timeval_ms() + timeout;
    uint32_t status;
    int retval;

    do {
        retval = vs32spi_read_flash_status(bank, &status);
        if(retval != ERROR_OK)
            return retval;
        if((status & SPIFLASH_BSY_BIT) == 0)
            return ERROR_OK;
        alive_sleep(1);
    } while (timeval_ms() < endtime);

    LOG_ERROR("Timeout with Flash WIP");
    return ERROR_TARGET_TIMEOUT;
}

static int vs32spi_flash_wr_enable(struct flash_bank *bank){
    uint32_t status;
    int retval;

    if (vs32spi_select_csid(bank, 0) != ERROR_OK)
        return ERROR_FAIL;
    if (vs32spi_set_params(bank, 8, 0, false) != ERROR_OK)
        return ERROR_FAIL;
    if (vs32spi_tx(bank, SPIFLASH_WRITE_ENABLE << 24) != ERROR_OK)
        return ERROR_FAIL;
    if (vs32spi_enable_spi(bank) != ERROR_OK)
        return ERROR_FAIL;

    /* Wait for transmit to finish */
    retval = vs32spi_spi_wait(bank, VS32SPI_CMD_TIMEOUT);
    if(retval != ERROR_OK)
        return retval;

    /* read flash status register */
    retval = vs32spi_read_flash_status(bank, &status);
    if(retval != ERROR_OK)
        return retval;

    /* Check write enabled */
    if((status & SPIFLASH_WE_BIT) == 0){
        LOG_ERROR("Cannot enable write to flash. Status=0x%08" PRIx32, status);
        return ERROR_FAIL;
    }

    return ERROR_OK;
}


static int vs32spi_erase_sector(struct flash_bank *bank, int sector)
{
    // struct vs32spi_flash_bank *vs32spi_info = bank->driver_priv;
    int retval;

    retval = vs32spi_flash_wr_enable(bank);
    if (retval != ERROR_OK)
        return retval;

    sector = bank->sectors[sector].offset;
    // vs32spi_info->dev->erase_cmd
    uint32_t cmd = ((VS32SPI_SECTOR_ERASE_4B << 24) | ((sector >> 8) & 0x00FFFFFF));

    retval = vs32spi_set_params(bank, 40, 0, false);
    if (retval != ERROR_OK)
        return retval;
    retval = vs32spi_tx(bank, cmd);
    if (retval != ERROR_OK)
        return retval;
    retval = vs32spi_tx(bank, ((sector & 0xFF) << 24));
    if (retval != ERROR_OK)
        return retval;

    vs32spi_enable_spi(bank);
    retval = vs32spi_spi_wait(bank, VS32SPI_CMD_TIMEOUT);
    if (retval != ERROR_OK)
        return retval;

    retval = vs32spi_flash_wip_ready(bank, VS32SPI_MAX_TIMEOUT);
    if (retval != ERROR_OK)
        return retval;

    return ERROR_OK;
}

static int vs32spi_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
    struct target *target = bank->target;
    struct vs32spi_flash_bank *vs32spi_info = bank->driver_priv;
    int retval = ERROR_OK;

    LOG_DEBUG("%s: from sector %u to sector %u", __func__, first, last);

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if ((last < first) || (last >= bank->num_sectors)) {
        LOG_ERROR("Flash sector invalid");
        return ERROR_FLASH_SECTOR_INVALID;
    }

    if (!(vs32spi_info->probed)) {
        LOG_ERROR("Flash bank not probed");
        return ERROR_FLASH_BANK_NOT_PROBED;
    }

    for (unsigned int sector = first; sector <= last; sector++) {
        if (bank->sectors[sector].is_protected) {
            LOG_ERROR("Flash sector %u protected", sector);
            return ERROR_FAIL;
        }
    }

    if (vs32spi_info->dev->erase_cmd == 0x00)
        return ERROR_FLASH_OPER_UNSUPPORTED;

    /* Initialize the SPI appropriately */
    retval = vs32spi_spi_wait(bank, VS32SPI_MAX_TIMEOUT);
    if (retval != ERROR_OK)
        return retval;

    vs32spi_spi_init(bank, VS32_STD_PRESCALAR, 0, false, false, true);

    /* poll WIP */
    retval = vs32spi_flash_wip_ready(bank, VS32SPI_PROBE_TIMEOUT);
    if (retval != ERROR_OK)
        goto done;

    for (unsigned int sector = first; sector <= last; sector++) {
        retval = vs32spi_erase_sector(bank, sector);
        if (retval != ERROR_OK)
            goto done;
        keep_alive();
    }

done:
    return retval;
}

static int vs32spi_protect(struct flash_bank *bank, int set,
        unsigned int first, unsigned int last)
{
    for (unsigned int sector = first; sector <= last; sector++)
        bank->sectors[sector].is_protected = set;
    return ERROR_OK;
}
static int vs32spi_verify(struct flash_bank *bank, const uint8_t *buffer,
    uint32_t offset, uint32_t count)
{
    struct target *target = bank->target;
    struct vs32spi_flash_bank *vs32spi_info = bank->driver_priv;
    uint32_t address = bank->base + offset;
    uint32_t cur_count;
    const uint16_t prescalar = VS32_FXD_PRESCALAR/2;
    int retval = ERROR_OK;

    LOG_DEBUG("Verifying buffer of %i bytes from 0x%8.8x",
        (int)count, (unsigned)address);

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if ((offset + count) > bank->size){
        LOG_ERROR("Read beyond end of flash");
        return ERROR_FLASH_DST_OUT_OF_BANK;
    }

    if (!(vs32spi_info->probed)) {
        LOG_ERROR("Flash bank not probed");
        return ERROR_FLASH_BANK_NOT_PROBED;
    }

    /* Check sector protection */
    for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
        /* Start offset in or before this sector? */
        /* End offset in or behind this sector? */
        if ((offset <
                    (bank->sectors[sector].offset + bank->sectors[sector].size))
                && ((offset + count - 1) >= bank->sectors[sector].offset)
                && bank->sectors[sector].is_protected) {
            LOG_ERROR("Flash sector %u protected", sector);
            return ERROR_FAIL;
        }
    }

    static const uint8_t vs32spi_rd32_bin[] = {
        #include "../../../contrib/loaders/flash/vs32spi/riscv32_rd_vs32spi.inc"
    };

    static const uint8_t vs32spi_rd64_bin[] = {
        #include "../../../contrib/loaders/flash/vs32spi/riscv64_rd_vs32spi.inc"
    };

    int xlen = riscv_xlen(target);
    struct working_area *algorithm_wa = NULL;
    struct working_area *data_wa = NULL;
    const uint8_t *bin;
    size_t bin_size;
    if (xlen == 32) {
        bin = vs32spi_rd32_bin;
        bin_size = sizeof(vs32spi_rd32_bin);
    } else {
        bin = vs32spi_rd64_bin;
        bin_size = sizeof(vs32spi_rd64_bin);
    }

    unsigned data_wa_size = 0;
    if (target_alloc_working_area_try(target, bin_size, &algorithm_wa) == ERROR_OK) {
        retval = target_write_buffer(target, algorithm_wa->address, bin_size, bin);
        if (retval == ERROR_OK) {
            data_wa_size = MIN(target->working_area_size - algorithm_wa->size, count);
            while (1) {
                if (target_alloc_working_area_try(target, data_wa_size, &data_wa) == ERROR_OK) {
                    LOG_DEBUG("Allocated %u - bytes Working Area for Verifying", data_wa_size);
                    break;
                }
                data_wa_size = data_wa_size * 3 / 4;
            }

            struct reg_param reg_params[5];
            init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
            init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
            init_reg_param(&reg_params[2], "a2", xlen, PARAM_OUT);
            init_reg_param(&reg_params[3], "a3", xlen, PARAM_OUT);
            init_reg_param(&reg_params[4], "a4", xlen, PARAM_OUT);
            while (count > 0){
                cur_count = MIN(count, data_wa_size);
                buf_set_u64(reg_params[0].value, 0, xlen, vs32spi_info->ctrl_base);
                buf_set_u64(reg_params[1].value, 0, xlen, offset);
                buf_set_u64(reg_params[2].value, 0, xlen, count);
                buf_set_u64(reg_params[3].value, 0, xlen, data_wa->address);
                buf_set_u64(reg_params[4].value, 0, xlen, prescalar << 16 | VS32SPI_FLASH_FASTREAD_4B
                  /*  vs32spi_info->dev->pprog_cmd */ | (bank->size > 0x1000000 ? 0x100 : 0));

                retval = vs32spi_flash_wip_ready(bank, VS32SPI_PROBE_TIMEOUT);
                if (retval != ERROR_OK)
                    goto verif_err;

                LOG_DEBUG("Reading %d bytes at 0x%" PRIx32 ,cur_count, offset );

                retval = target_run_algorithm(target, 0, NULL, ARRAY_SIZE(reg_params), reg_params,
                                          algorithm_wa->address, 0, cur_count << 2, NULL);
                if (retval != ERROR_OK) {
                    LOG_ERROR("Failed to execute algorithm at " TARGET_ADDR_FMT ": %d",
                            algorithm_wa->address, retval);
                    goto verif_err;
                }

                int algorithm_result = buf_get_u64(reg_params[0].value, 0, xlen);
                if (algorithm_result != 0) {
                    LOG_ERROR("Algorithm returned error 0x%x", algorithm_result);
                    retval = ERROR_FAIL;
                    goto verif_err;
                }
                uint8_t *lbuffer = (uint8_t *)malloc(cur_count);
                if(lbuffer == NULL){
                    LOG_ERROR("No enough heap memory to verify");
                    goto verif_err;
                }
                retval = target_read_buffer(target, data_wa->address, cur_count, lbuffer);
                if (retval != ERROR_OK) {
                    LOG_DEBUG("Failed to Read %d bytes from " TARGET_ADDR_FMT ": %d",
                        cur_count, data_wa->address, retval);
                    goto verif_err;
                }

                if(cur_count%4 != 0){
                    uint32_t n_count = cur_count -(cur_count%4);
                    buf_bswap32(lbuffer, lbuffer, cur_count);
                    buf_bswap16(lbuffer+n_count, lbuffer+n_count, (cur_count%4));
                }else
                    buf_bswap32(lbuffer, lbuffer, cur_count);

                retval = memcmp(lbuffer, buffer, cur_count);
                

                if(retval != ERROR_OK){
                    LOG_DEBUG("received value buffer \n %02x %02x %02x %02x %02x %02x %02x ...\n \
                     %02x %02x %02x %02x %02x %02x %02x ...", buffer[0], buffer[1], buffer[2], buffer[3],\
                     buffer[4], buffer[5], buffer[6], lbuffer[0], lbuffer[1], lbuffer[2], \
                     lbuffer[3], lbuffer[4], lbuffer[5], lbuffer[6]);
                    free(lbuffer);
                    goto verif_err;
                } else {
                    free(lbuffer);
                }

                buffer += cur_count;
                offset += cur_count;
                count  -= cur_count;
            }

            destroy_reg_param(&reg_params[0]);
            destroy_reg_param(&reg_params[1]);
            destroy_reg_param(&reg_params[2]);
            destroy_reg_param(&reg_params[3]);
            destroy_reg_param(&reg_params[4]);
            target_free_working_area(target, data_wa);
            target_free_working_area(target, algorithm_wa);
        }
        else{
            LOG_ERROR("Failed to write code to " TARGET_ADDR_FMT ": %d",
                    algorithm_wa->address, retval);
            goto verif_err;
        }
    } else{
        LOG_ERROR("Not enough working area, can't do SPI verify");
        return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }

verif_err:
    if(algorithm_wa){
        target_free_working_area(target, algorithm_wa);
        algorithm_wa = NULL;
    }
    if(data_wa){
        target_free_working_area(target, data_wa);
        data_wa = NULL;
    }
    return retval;
}

static int vs32spi_slow_reads(struct flash_bank *bank, uint8_t *buffer,
                                uint32_t address, uint32_t count)
{
    int retval;
    uint32_t cmd;
    register uint8_t ii;

    // If greater than VS32_RX_BYTE_CAPPING bytes to be read,
    // Limit one SPI Read Transaction to VS32_RX_BYTE_CAPPING bytes of data.
    uint8_t bits_2_receive = (count < VS32_RX_BYTE_CAPPING ) ?
                             (count << 3) : VS32_RX_BIT_XFER_CAP;
    retval = vs32spi_set_params(bank, 40, bits_2_receive, false);
    if (retval != ERROR_OK)
        return retval;
    // vs32spi_info->dev->read_cmd
    cmd = VS32SPI_FLASH_READ_4B << 24 | ((address >> 8) & 0x00FFFFFF) ;
    retval = vs32spi_tx(bank, cmd);
    if (retval != ERROR_OK)
        return retval;

    cmd = ((address & 0xFF) << 24);
    retval = vs32spi_tx(bank, cmd);
    if (retval != ERROR_OK)
        return retval;

    retval = vs32spi_enable_spi(bank);
    if (retval != ERROR_OK)
        return retval;

    for(ii = 0; ii < count; (ii = ii + 4)){
        retval = vs32spi_rx(bank, &cmd);
        if (retval != ERROR_OK)
            return retval;

        h_u32_to_be((buffer + ii), cmd);
    }

    LOG_DEBUG("%s read value =0x%08" PRIx32 " from buffer of %i byte at 0x%8.8x", __func__,
        cmd , (int)count, (unsigned)address);

    retval = vs32spi_spi_wait(bank, VS32SPI_MAX_TIMEOUT);
    if (retval != ERROR_OK)
        return retval;

    return ERROR_OK;
}

static int vs32spi_read(struct flash_bank *bank, uint8_t *buffer,
        uint32_t offset, uint32_t count)
{
    struct target *target = bank->target;
    struct vs32spi_flash_bank *vs32spi_info = bank->driver_priv;
    uint32_t address = bank->base + offset;
    uint32_t cur_count;
    const uint16_t prescalar = 6;
    int retval = ERROR_OK;

    LOG_INFO("reading buffer of %i bytes from 0x%8.8x",
        (int)count, (unsigned)address);

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if ((offset + count) > bank->size){
        LOG_ERROR("Read beyond end of flash");
        return ERROR_FLASH_DST_OUT_OF_BANK;
    }

    if (!(vs32spi_info->probed)) {
        LOG_ERROR("Flash bank not probed");
        return ERROR_FLASH_BANK_NOT_PROBED;
    }

    retval = vs32spi_spi_wait(bank, VS32SPI_MAX_TIMEOUT);
    if (retval != ERROR_OK)
        return retval;

    vs32spi_spi_init(bank, VS32_STD_PRESCALAR, 0, false, false, true);

    retval = vs32spi_flash_wip_ready(bank, VS32SPI_MAX_TIMEOUT);
    if (retval != ERROR_OK)
        return retval;

    static const uint8_t vs32spi_rd32_bin[] = {
        #include "../../../contrib/loaders/flash/vs32spi/riscv32_rd_vs32spi.inc"
    };

    static const uint8_t vs32spi_rd64_bin[] = {
        #include "../../../contrib/loaders/flash/vs32spi/riscv64_rd_vs32spi.inc"
    };

    int xlen = riscv_xlen(target);
    struct working_area *algorithm_wa = NULL;
    struct working_area *data_wa = NULL;
    const uint8_t *bin;
    size_t bin_size;
    if (xlen == 32) {
        bin = vs32spi_rd32_bin;
        bin_size = sizeof(vs32spi_rd32_bin);
    } else {
        bin = vs32spi_rd64_bin;
        bin_size = sizeof(vs32spi_rd64_bin);
    }

    unsigned data_wa_size = 0;
    if (target_alloc_working_area(target, bin_size, &algorithm_wa) == ERROR_OK) {
        retval = target_write_buffer(target, algorithm_wa->address, bin_size, bin);
        if (retval != ERROR_OK) {
            LOG_ERROR("Failed to write code to " TARGET_ADDR_FMT ": %d",
                    algorithm_wa->address, retval);
            target_free_working_area(target, algorithm_wa);
            algorithm_wa = NULL;
        }
        else {
            data_wa_size = MIN(target->working_area_size - algorithm_wa->size, count);
            while (1) {
                if (data_wa_size < 128 || VS32SPI_TGT_MEM_NOT_ALOWD) {
                    LOG_DEBUG("Not Using Target Work Area for SPI Reads");
                    target_free_working_area(target, algorithm_wa);
                    algorithm_wa = NULL;
                    break;
                }
                if (target_alloc_working_area_try(target, data_wa_size, &data_wa) ==
                        ERROR_OK) {
                    data_wa_size -= (data_wa_size%4); // multiple of 4
                    LOG_DEBUG("Allocated %u - bytes Working Area for reading", data_wa_size);
                    break;
                }
                data_wa_size = data_wa_size * 3 / 4;
            }
        }
    }
    else{
        LOG_WARNING("Not enough working area, Switching to Slow Reads ");
        algorithm_wa = NULL;
    }

    if(algorithm_wa){
        struct reg_param reg_params[5];
        init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
        init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
        init_reg_param(&reg_params[2], "a2", xlen, PARAM_OUT);
        init_reg_param(&reg_params[3], "a3", xlen, PARAM_OUT);
        init_reg_param(&reg_params[4], "a4", xlen, PARAM_OUT);
        while (count > 0) {
            cur_count = MIN(count, data_wa_size);
            buf_set_u64(reg_params[0].value, 0, xlen, vs32spi_info->ctrl_base);
            buf_set_u64(reg_params[1].value, 0, xlen, offset);
            buf_set_u64(reg_params[2].value, 0, xlen, count);
            buf_set_u64(reg_params[3].value, 0, xlen, data_wa->address);
            buf_set_u64(reg_params[4].value, 0, xlen, prescalar << 16 | VS32SPI_FLASH_FASTREAD_4B
                  /*  vs32spi_info->dev->pprog_cmd */ | (bank->size > 0x1000000 ? 0x100 : 0));

            retval = vs32spi_flash_wip_ready(bank, VS32SPI_PROBE_TIMEOUT);
            if (retval != ERROR_OK)
                goto rd_err;

            LOG_DEBUG("Reading %d bytes at 0x%" PRIx32 ,cur_count, offset );

            retval = target_run_algorithm(target, 0, NULL, ARRAY_SIZE(reg_params), reg_params,
                                          algorithm_wa->address, 0, cur_count << 2, NULL);
            if (retval != ERROR_OK) {
                LOG_ERROR("Failed to execute algorithm at " TARGET_ADDR_FMT ": %d",
                        algorithm_wa->address, retval);
                goto rd_err;
            }

            int algorithm_result = buf_get_u64(reg_params[0].value, 0, xlen);
            if (algorithm_result != 0) {
                LOG_ERROR("Algorithm returned error 0x%x", algorithm_result);
                retval = ERROR_FAIL;
                goto rd_err;
            }

            retval = target_read_buffer(target, data_wa->address, cur_count, buffer);
            if (retval != ERROR_OK) {
                LOG_DEBUG("Failed to Read %d bytes from " TARGET_ADDR_FMT ": %d",
                        cur_count, data_wa->address, retval);
                goto rd_err;
            }

            if(cur_count%4 != 0){
                uint32_t n_count = cur_count -(cur_count%4);
                buf_bswap32(buffer, buffer, cur_count);
                buf_bswap16(buffer+n_count, buffer+n_count, (cur_count%4));
            }else
                buf_bswap32(buffer, buffer, cur_count);

            buffer += cur_count;
            offset += cur_count;
            count  -= cur_count;
        }
        destroy_reg_param(&reg_params[0]);
        destroy_reg_param(&reg_params[1]);
        destroy_reg_param(&reg_params[2]);
        destroy_reg_param(&reg_params[3]);
        destroy_reg_param(&reg_params[4]);
        target_free_working_area(target, data_wa);
        target_free_working_area(target, algorithm_wa);
    }
    else{
#if DEBUG
        printf("Reading from Flash: ");
        printf("\033[?25l");  // hide the cursor
#endif
        unsigned ii = 0;
        do{
            uint32_t length = ((count - ii) > (VS32_RX_BYTE_CAPPING - 1)) ?
                                                     VS32_RX_BYTE_CAPPING :
                                     ((count - ii) % VS32_RX_BYTE_CAPPING);
            retval = vs32spi_slow_reads(bank, (buffer + ii), (address + ii),
                                           length);
            if (retval != ERROR_OK)
                return retval;
            ii += length;
#ifdef DEBUG
            vs32spi_indicate_loading();
#endif
            keep_alive();
        } while(ii < count);

#if DEBUG
        printf("\033[?25h"); // restore the cursor
        printf("\n");
#endif
    }

rd_err:
    if (algorithm_wa) {
        target_free_working_area(target, data_wa);
        target_free_working_area(target, algorithm_wa);
    }

    return retval;
}

static int vs32spi_slow_writes(struct flash_bank *bank, const uint8_t *buffer,
            uint32_t address, unsigned hw_len)
{
    uint8_t bits_2_transfer, init_switch ;
    uint32_t cmd;
    unsigned ii;
    int retval;

    retval = vs32spi_flash_wr_enable(bank);
    if (retval != ERROR_OK)
        return retval;

    // If greater than VS32_TX_BYTE_CAPPING bytes to be written,
    // Limit one SPI Write Transaction to VS32_TX_BYTE_CAPPING bytes of data.
    bits_2_transfer = (hw_len < VS32_TX_BYTE_CAPPING ) ?
                         ((hw_len << 3) + 40) : VS32_TX_BIT_XFER_CAP;
    retval = vs32spi_set_params(bank, bits_2_transfer, 0, false);
    if (retval != ERROR_OK)
        return retval;
    /*vs32spi_info->dev->pprog_cmd */
    cmd = VS32SPI_FLASH_WRITE_4B << 24 | ((address >> 8) & 0x00FFFFFF) ;
    retval = vs32spi_tx(bank, cmd);
    if (retval != ERROR_OK)
        return retval;

    cmd = ((address & 0xFF) << 24);
    ii = 0;
    init_switch = (hw_len < 4) ? hw_len : 3;
    switch(init_switch){
        case 3: cmd |= buffer[ii + 2] ;
                fallthrough;
        case 2: cmd |= buffer[ii + 1] << 8;
                fallthrough;
        case 1: cmd |= buffer[ii] << 16;
    }

    retval = vs32spi_tx(bank, cmd );
    if (retval != ERROR_OK)
        return retval;
    else
        cmd = 0;
    if(hw_len > 3){
        ii = 3;
        do{
            init_switch = (ii + 4 < hw_len) ? 3 : hw_len % 4;
            switch(init_switch){
                case 3: cmd |= buffer[ii + 3]       ;
                        fallthrough;
                case 2: cmd |= buffer[ii + 2] << 8  ;
                        fallthrough;
                case 1: cmd |= buffer[ii + 1] << 16 ;
                        fallthrough;
                case 0: cmd |= buffer[ii]     << 24 ;
            }
            if (vs32spi_tx(bank, cmd ) != ERROR_OK)
                return ERROR_FAIL;
            else{
                ii += init_switch + 1;
                cmd = 0;
                // if(ii > 11){
                //     retval = vs32spi_enable_spi(bank);
                //     if (retval != ERROR_OK)
                //         return retval;
                // }
            }
        }while (ii < hw_len);
    }
    LOG_DEBUG("%s: address=0x%08" PRIx32 " len=0x%08" PRIx32 ,
                       __func__, address, hw_len );

    retval = vs32spi_enable_spi(bank);
    if (retval != ERROR_OK)
        return retval;

    retval = vs32spi_spi_wait(bank, VS32SPI_MAX_TIMEOUT);
    if (retval != ERROR_OK)
        return retval;

    retval = vs32spi_flash_wip_ready(bank, VS32SPI_PROBE_TIMEOUT);
    if (retval != ERROR_OK)
        return retval;

    return ERROR_OK;
}

static int vs32spi_write_buffer(struct flash_bank *bank,
        const uint8_t *buffer, uint32_t address, uint32_t len)
{
    // struct vs32spi_flash_bank *vs32spi_info = bank->driver_priv;
    int retval;

    /* TODO!!! assert that len < page size */

    /* Although SPI has unlimited transfer mode, we'd choose to send
    VS32_TX_BYTE_CAPPING bytes of data at a time */
    unsigned ii = 0;
    do{
        uint32_t length = ((len - ii) > (VS32_TX_BYTE_CAPPING - 1)) ?
                                               VS32_TX_BYTE_CAPPING :
                                 ((len - ii) % VS32_TX_BYTE_CAPPING);
        retval = vs32spi_slow_writes(bank, (buffer + ii), (address + ii),
                                       length );
#ifdef DEBUG
        vs32spi_indicate_loading();
#endif
        if (retval != ERROR_OK)
            return retval;
        ii += length;
        keep_alive();
    } while(ii < len);

    return ERROR_OK;
}

static const uint8_t riscv32_bin[] = {
#include "../../../contrib/loaders/flash/vs32spi/riscv32_wr_vs32spi.inc"
};

static const uint8_t riscv64_bin[] = {
#include "../../../contrib/loaders/flash/vs32spi/riscv64_wr_vs32spi.inc"
};


/* Dev Notes:
 *   For Large writes the 'allocated working area' is used  in which:
 *   1.  The 'data_wa_size' is trimmed to be a multiple of 'page_size'. This
 *       ensures, that the continuous writes to a page isn't broken.
 *   2.  Writes are always maximized in loop.
 *       a)  writes 'data_wa_size' bytes in one go
 *       b)  if data to be written is less than 'data_wa_size', the writes are
 *           divided into two portions
 *            i) write the remaining data as mutliple of 'page_size'
 *           ii) residue from above is appended with zeros upto 'page_size'
 */
static int vs32spi_write(struct flash_bank *bank, const uint8_t *buffer,
        uint32_t offset, uint32_t count)
{
    struct target *target = bank->target;
    struct vs32spi_flash_bank *vs32spi_info = bank->driver_priv;
    uint32_t cur_count, page_size;
    const uint16_t prescalar = VS32_FXD_PRESCALAR;
    int retval = ERROR_OK;


    LOG_INFO("Writing buffer of %i bytes at 0x%08" PRIx32,
        (int)count, (unsigned)offset);

    if (target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if (offset + count > vs32spi_info->dev->size_in_bytes) {
        LOG_WARNING("Write past end of flash. Extra data discarded.");
        count = vs32spi_info->dev->size_in_bytes - offset;
    }

    /* Check sector protection */
    for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
        /* Start offset in or before this sector? */
        /* End offset in or behind this sector? */
        if ((offset <
                    (bank->sectors[sector].offset + bank->sectors[sector].size))
                && ((offset + count - 1) >= bank->sectors[sector].offset)
                && bank->sectors[sector].is_protected) {
            LOG_ERROR("Flash sector %u protected", sector);
            return ERROR_FAIL;
        }
    }

    int xlen = riscv_xlen(target);
    struct working_area *algorithm_wa = NULL;
    struct working_area *data_wa = NULL;
    const uint8_t *bin;
    size_t bin_size;
    if (xlen == 32) {
        bin = riscv32_bin;
        bin_size = sizeof(riscv32_bin);
    } else {
        bin = riscv64_bin;
        bin_size = sizeof(riscv64_bin);
    }

    /* If no valid page_size, use reasonable default. */
    page_size = vs32spi_info->dev->pagesize ? vs32spi_info->dev->pagesize :
                                                SPIFLASH_DEF_PAGESIZE;
    unsigned data_wa_size = 0;
    if (target_alloc_working_area(target, bin_size, &algorithm_wa) == ERROR_OK) {
        retval = target_write_buffer(target, algorithm_wa->address, bin_size, bin);
        if (retval != ERROR_OK) {
            LOG_ERROR("Failed to write code to " TARGET_ADDR_FMT ": %d",
                    algorithm_wa->address, retval);
            target_free_working_area(target, algorithm_wa);
            algorithm_wa = NULL;

        } else {
            vs32spi_spi_init(bank, prescalar, 0, false, false, true);
            data_wa_size = MIN(target->working_area_size - algorithm_wa->size, count);
            while (1) {
                if (data_wa_size < 128 || VS32SPI_TGT_MEM_NOT_ALOWD) {
                    LOG_DEBUG("Not Using Target Work Area for SPI Writes");
                    target_free_working_area(target, algorithm_wa);
                    algorithm_wa = NULL;
                    break;
                }
                if (target_alloc_working_area_try(target, data_wa_size, &data_wa) ==
                        ERROR_OK) {
                    data_wa_size -= (data_wa_size%page_size) ; //dev-note: 1
                    LOG_DEBUG("Allocated %u - bytes Working Area", data_wa_size);
                    break;
                }

                data_wa_size = data_wa_size * 3 / 4;
            }
        }
    } else {
        LOG_WARNING("Couldn't allocate %zd-byte working area.", bin_size);
        algorithm_wa = NULL;
    }

    if (algorithm_wa) {
        struct reg_param reg_params[6];
        init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
        init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
        init_reg_param(&reg_params[2], "a2", xlen, PARAM_OUT);
        init_reg_param(&reg_params[3], "a3", xlen, PARAM_OUT);
        init_reg_param(&reg_params[4], "a4", xlen, PARAM_OUT);
        init_reg_param(&reg_params[5], "a5", xlen, PARAM_OUT);

        while (count > 0) {
            cur_count = MIN(count, data_wa_size); // dev-note 2.a
            uint8_t *data_buffer = NULL;
            unsigned append_val = 0;
            if(cur_count == count){               // dev-note 2.b
                if(cur_count < page_size){        // dev-note 2.b.ii
                    append_val  = (page_size - cur_count);
                    data_buffer = (uint8_t *)calloc(page_size, sizeof(uint8_t));
                    if(data_buffer == NULL)
                        goto wr_err;
                    else{
                        memcpy(data_buffer, buffer, cur_count);
                        if(data_buffer == NULL)
                            goto wr_err;
                    }
                }
                else{                             // dev-note 2.b.i
                    cur_count -= (cur_count % page_size);
                }
            }
            buf_set_u64(reg_params[0].value, 0, xlen, vs32spi_info->ctrl_base);
            buf_set_u64(reg_params[1].value, 0, xlen, page_size);
            buf_set_u64(reg_params[2].value, 0, xlen, data_wa->address);
            buf_set_u64(reg_params[3].value, 0, xlen, offset);
            buf_set_u64(reg_params[4].value, 0, xlen, (cur_count +append_val));
            buf_set_u64(reg_params[5].value, 0, xlen, prescalar << 16 | VS32SPI_FLASH_WRITE_4B
                  /*  vs32spi_info->dev->pprog_cmd */ | (bank->size > 0x1000000 ? 0x100 : 0));

            retval = target_write_buffer(target, data_wa->address, (cur_count +append_val), \
                                         ( (append_val != 0) ? data_buffer :  buffer));

            if((append_val != 0)){
                free((uint8_t *)data_buffer);
            }

            if (retval != ERROR_OK) {
                LOG_DEBUG("Failed to write %d bytes to " TARGET_ADDR_FMT ": %d",
                        cur_count, data_wa->address, retval);
                goto wr_err;
            }

            retval = vs32spi_flash_wip_ready(bank, VS32SPI_PROBE_TIMEOUT);
            if (retval != ERROR_OK)
                goto wr_err;

            LOG_DEBUG("Writing %d bytes at 0x%" PRIx32 ,cur_count, offset );

            LOG_DEBUG("write(ctrl_base=0x%" TARGET_PRIxADDR ", page_size=0x%x, "
                      "address=0x%" TARGET_PRIxADDR ", offset=0x%" PRIx32
                      ", count=0x%" PRIx32 "), buffer=%02x %02x %02x %02x %02x %02x ..." PRIx32 "\n",
                    vs32spi_info->ctrl_base, page_size, data_wa->address, offset, cur_count,
                    buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
            retval = target_run_algorithm(target, 0, NULL, ARRAY_SIZE(reg_params), reg_params,
                                          algorithm_wa->address, 0, cur_count << 2, NULL);
            if (retval != ERROR_OK) {
                LOG_ERROR("Failed to execute algorithm at " TARGET_ADDR_FMT ": %d",
                        algorithm_wa->address, retval);
                goto wr_err;
            }

            int algorithm_result = buf_get_u64(reg_params[0].value, 0, xlen);
            if (algorithm_result != 0) {
                LOG_ERROR("Algorithm returned error 0x%x", algorithm_result);
                retval = ERROR_FAIL;
                goto wr_err;
            }

            buffer += cur_count;
            offset += cur_count;
            count  -= cur_count;

        }

        target_free_working_area(target, data_wa);
        target_free_working_area(target, algorithm_wa);

    }
    else {
        vs32spi_spi_wait(bank, VS32SPI_CMD_TIMEOUT);
        if (retval != ERROR_OK)
            return retval;

        // Switch to operations as fast as possible.
        vs32spi_spi_init(bank, VS32_STD_PRESCALAR, 0, false, false, true);

        /* poll WIP */
        retval = vs32spi_flash_wip_ready(bank, VS32SPI_PROBE_TIMEOUT);
        if (retval != ERROR_OK)
            goto wr_err;
#if DEBUG
        printf("Writing to Flash: ");
        printf("\033[?25l");  // hide the cursor
#endif
        uint32_t page_offset = offset % page_size;
        /* central part, aligned words */
        while (count > 0) {
            /* clip block at page boundary */
            if (page_offset + count > page_size)
                cur_count = page_size - page_offset;
            else
                cur_count = count;

            retval = vs32spi_write_buffer(bank, buffer, offset, cur_count);
            if (retval != ERROR_OK)
                goto wr_err;

            page_offset = 0;
            buffer += cur_count;
            offset += cur_count;
            count  -= cur_count;
        }
#if DEBUG
        printf("\033[?25h"); // restore the cursor
        printf("\n"); // restore the cursor
#endif
    }

    return ERROR_OK;

wr_err:
    if (algorithm_wa) {
        target_free_working_area(target, data_wa);
        target_free_working_area(target, algorithm_wa);
    }

    return retval;
}

/* Return ID of flash device */
static int vs32spi_read_flash_id(struct flash_bank *bank, uint32_t *id)
{
    int retval;

    if (bank->target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }
    /* poll WIP */
    retval = vs32spi_flash_wip_ready(bank, VS32SPI_PROBE_TIMEOUT);
    if (retval != ERROR_OK)
        return retval;

    retval = vs32spi_set_params(bank, 8, 24, false);
    if (retval != ERROR_OK)
        return retval;
    /* Send SPI command "read ID" */
    retval = vs32spi_tx(bank, (SPIFLASH_READ_ID << 24));
    retval = vs32spi_enable_spi(bank);
    if (retval != ERROR_OK)
        return retval;

    *id = 0;
    uint8_t read_id[4];
    /* read ID from Receive Register */
    if (vs32spi_rx(bank, id) != ERROR_OK)
        return ERROR_FAIL;

    h_u32_to_be(read_id, *id);
    *id = *((uint32_t*)read_id);

    retval = vs32spi_spi_wait(bank, VS32SPI_MAX_TIMEOUT);
    if (retval != ERROR_OK)
        return retval;

    return ERROR_OK;
}

static int vs32spi_probe(struct flash_bank *bank)
{
    struct vs32spi_flash_bank *vs32spi_info = bank->driver_priv;
    struct flash_sector *sectors;
    uint32_t sectorsize;
    uint32_t id = 0;
    const struct vs32spi_target *target_device;
    int retval;

    if (bank->target->state != TARGET_HALTED) {
        LOG_ERROR("Target not halted");
        return ERROR_TARGET_NOT_HALTED;
    }

    if (vs32spi_info->probed)
        free(bank->sectors);
    bank->sectors = NULL;
    vs32spi_info->probed = false;

    if (vs32spi_info->ctrl_base == 0) {
        for (target_device = target_devices ; target_device->name ; ++target_device)
            if (target_device->tap_idcode == bank->target->tap->idcode)
                break;

        if (!target_device->name) {
            LOG_ERROR("Device ID 0x%" PRIx32 " is not known as VS32SPI capable",
                    bank->target->tap->idcode);
            return ERROR_FAIL;
        }

        vs32spi_info->ctrl_base = target_device->ctrl_base;

        LOG_DEBUG("Valid VS32SPI on device %s at address " TARGET_ADDR_FMT,
                target_device->name, bank->base);

    } else {
      LOG_DEBUG("VS32SPI Controller specified at address " TARGET_ADDR_FMT
              " controls at " TARGET_ADDR_FMT, vs32spi_info->ctrl_base,
              bank->base);
    }

    /* Initialize SPI to MODE 0, HALF DUPLEX TX 1st, with Prescalar of 4 */
    vs32spi_spi_init(bank, VS32_STD_PRESCALAR, 0, false, false, true);

    /* read and decode flash ID; returns in SW mode */
    retval = vs32spi_read_flash_id(bank, &id);
    if (retval != ERROR_OK)
        return retval;

    vs32spi_info->dev = NULL;
    for (const struct flash_device *p = flash_devices; p->name ; p++)
        if (p->device_id == id) {
            vs32spi_info->dev = p;
            break;
        }

    if (!vs32spi_info->dev) {
        LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
        return ERROR_FAIL;
    }

    LOG_INFO("%s found flash device \'%s\' (ID 0x%08" PRIx32 ")", target_device->name,
            vs32spi_info->dev->name, vs32spi_info->dev->device_id);

    /* Set correct size value */
    bank->size = vs32spi_info->dev->size_in_bytes;

    if (bank->size <= (1UL << 16))
        LOG_WARNING("device needs 2-byte addresses - not implemented");

    /* if no sectors, treat whole bank as single sector */
    sectorsize = vs32spi_info->dev->sectorsize ?
        vs32spi_info->dev->sectorsize : vs32spi_info->dev->size_in_bytes;

    /* create and fill sectors array */
    bank->num_sectors = vs32spi_info->dev->size_in_bytes / sectorsize;
    sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
    if (sectors == NULL) {
        LOG_ERROR("not enough memory");
        return ERROR_FAIL;
    }

    for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
        sectors[sector].offset = sector * sectorsize;
        sectors[sector].size   = sectorsize;
        sectors[sector].is_erased    = -1;
        sectors[sector].is_protected = 0;
    }

    bank->sectors = sectors;
    vs32spi_info->probed = true;
    return ERROR_OK;
}

static int vs32spi_auto_probe(struct flash_bank *bank)
{
    struct vs32spi_flash_bank *vs32spi_info = bank->driver_priv;
    if (vs32spi_info->probed)
        return ERROR_OK;
    return vs32spi_probe(bank);
}

static int vs32spi_protect_check(struct flash_bank *bank)
{
    /* Nothing to do. Protection is only handled in SW. */
    return ERROR_OK;
}

//static int  get_vs32spi_info(struct flash_bank *bank, char *buf, int buf_size)
//{
//    struct vs32spi_flash_bank *vs32spi_info = bank->driver_priv;
//
//    if (!(vs32spi_info->probed)) {
//        snprintf(buf, buf_size,
//                "\nVS32SPI flash bank not probed yet\n");
//        return ERROR_OK;
//    }
//
//    snprintf(buf, buf_size, "\nVS32SPI flash information:\n"
//            "  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
//            vs32spi_info->dev->name, vs32spi_info->dev->device_id);
//
//    return ERROR_OK;
//}

const struct flash_driver vs32spi_flash = {
    .name = "vs32spi",
    .usage = "CMD_ARGV[2] is used to specify the valid   \
        location to write the boot image or flash data \n\
        \t CMD_ARGV[6] is used to specify the flash      \
        associated SPI controller\'s Address. If not     \
        specified, the address should be availabe in     \
        \'target_devices\' structure",
    .flash_bank_command = vs32spi_flash_bank_command,
    .erase   = vs32spi_erase,
    .protect = vs32spi_protect,
    .write   = vs32spi_write,
    .read    = vs32spi_read,
    .probe   = vs32spi_probe,
    .verify  = vs32spi_verify,
    .auto_probe    = vs32spi_auto_probe,
    .erase_check   = default_flash_blank_check,
    .protect_check = vs32spi_protect_check,
    //.info = get_vs32spi_info,
    .free_driver_priv = default_flash_free_driver_priv
};
