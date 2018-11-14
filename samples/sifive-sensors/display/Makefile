
BUILD_DIR=build
ELF_OUT=$(BUILD_DIR)/zephyr/zephyr.elf
HEX_OUT=$(BUILD_DIR)/zephyr/zephyr.hex

ZEPHYR_BOARD=sifive-sensors

QEMU=qemu-system-riscv32
QEMU_MACHINE=sifive_e

OBJCOPY=riscv64-sifive-elf-objcopy

GDB=riscv64-sifive-elf-gdb
GDB_PORT=31337

all: clean elf hex

hex: $(HEX_OUT)
$(HEX_OUT): $(ELF_OUT)
	$(OBJCOPY) -O ihex $(ELF_OUT) $(HEX_OUT)

elf: $(ELF_OUT)
$(ELF_OUT): $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake -DBOARD=$(ZEPHYR_BOARD) ..
	cd $(BUILD_DIR) && make -j$(nproc)

$(BUILD_DIR):
	rm -rf $(BUILD_DIR) && mkdir $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)

