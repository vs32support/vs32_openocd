
#ifndef _vs32spi_h
#define _vs32spi_h

/* Register offsets */
#define VS32SPI_REG_CR1             0x00
#define VS32SPI_REG_CR2             0x04
#define VS32SPI_REG_EN              0x08
#define VS32SPI_REG_SR              0x0C
#define VS32SPI_REG_TX              0x10
#define VS32SPI_REG_RX              0x14
#define VS32SPI_REG_PSCR            0x24

/* Polynomial Registers */
#define VS32SPI_REG_RX_CRC          0x18
#define VS32SPI_REG_TX_CRC          0x1C
#define VS32SPI_REG_CRCPR           0x28

/* Delay Registers */
#define VS32SPI_REG_DELAYS          0x20

// CONSTANTS
#define VS32_TX_FIFO_LIMIT           20
#define VS32_TX_BYTE_CAPPING         16
#define VS32_RX_BYTE_CAPPING         24
#define VS32_TX_BIT_XFER_CAP        168
#define VS32_RX_BIT_XFER_CAP        192

// Definite Values
#define VS32SPI_SPI_ENABLE          0x01
#define VS32SPI_SPI_DISABLE         0x00

#define VS32SPI_SPI_BSY             0x800
#define VS32SPI_STATUS_RXNE         0x001
#define VS32SPI_STATUS_TXE          0x002

#define VS32SPI_STATUS_RXFIFO(x)    (((x) & 0xFF000000) >> 24)
#define VS32SPI_STATUS_TXFIFO(x)    (((x) & 0x00FF0000) >> 16)

/* Timeout in ms */
#define VS32SPI_CMD_TIMEOUT         (1000)
#define VS32SPI_PROBE_TIMEOUT       (100)
#define VS32SPI_MAX_TIMEOUT         (5000)
#define VS32_STD_PRESCALAR             2 // Switch to 4 for safer Transactions
#define VS32_FXD_PRESCALAR            16 // Increase value if write fails.

/* SPI FLASH 4B Commands */
#define VS32SPI_SECTOR_ERASE_4B     0xDC
#define VS32SPI_FLASH_WRITE_4B      0x12
#define VS32SPI_FLASH_READ_4B       0x13
#define VS32SPI_FLASH_FASTREAD_4B   0x0C

#define DEBUG                            1
#define VS32SPI_TGT_MEM_NOT_ALOWD      0

#endif /* _vs32spi_h */
