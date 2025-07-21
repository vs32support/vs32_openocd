BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= riscv64-unknown-elf-

RISCV_CC=$(CROSS_COMPILE)gcc
RISCV_OBJCOPY=$(CROSS_COMPILE)objcopy
RISCV_OBJDUMP=$(CROSS_COMPILE)objdump

CFLAGS = -nostdlib -nostartfiles -Wall -Werror -Os -fPIC -Wunused-result -g
RISCV32_CFLAGS = -march=rv32e -mabi=ilp32e $(CFLAGS)
RISCV64_CFLAGS = -march=rv64i -mabi=lp64 $(CFLAGS)

all: riscv32_wr_vs32spi.inc riscv64_wr_vs32spi.inc riscv32_rd_vs32spi.inc riscv64_rd_vs32spi.inc

.PHONY: clean

# .c -> .o
riscv32_%.o:  riscv_%.c
	$(RISCV_CC) -c $(RISCV32_CFLAGS) $^ -o $@

riscv64_%.o:  riscv_%.c
	$(RISCV_CC) -c $(RISCV64_CFLAGS) $< -o $@

# .S -> .o
riscv32_%.o:  riscv_%.S
	$(RISCV_CC) -c $(RISCV32_CFLAGS) $^ -o $@

riscv64_%.o:  riscv_%.S
	$(RISCV_CC) -c $(RISCV64_CFLAGS) $^ -o $@

# .o -> .elf
riscv32_rd_%.elf:	riscv32_rd_%.o riscv32_rd_wrapr.o
	$(RISCV_CC) -T riscv.lds $(RISCV32_CFLAGS) $^ -o $@

riscv64_rd_%.elf:	riscv64_rd_%.o riscv64_rd_wrapr.o
	$(RISCV_CC) -T riscv.lds $(RISCV64_CFLAGS) $^ -o $@

# .o -> .elf
riscv32_wr_%.elf:	riscv32_wr_%.o riscv32_wr_wrapr.o
	$(RISCV_CC) -T riscv.lds $(RISCV32_CFLAGS) $^ -o $@

riscv64_wr_%.elf:	riscv64_wr_%.o riscv64_wr_wrapr.o
	$(RISCV_CC) -T riscv.lds $(RISCV64_CFLAGS) $^ -o $@

# .elf -> .bin
%.bin: %.elf
	$(RISCV_OBJCOPY) -Obinary $< $@

# utility
%.lst: %.elf
	$(RISCV_OBJDUMP) -S $< > $@

# .bin -> .inc
%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.o *.lst *.bin *.inc
