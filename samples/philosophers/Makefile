
BUILD_DIR=build
ELF_OUT=$(BUILD_DIR)/zephyr/zephyr.elf

ZEPHYR_BOARD=hifive1

QEMU=/scratch/nathanielg/riscv-qemu/install/bin/qemu-system-riscv32
QEMU_MACHINE=sifive_e

GDB=riscv64-sifive-elf-gdb
GDB_PORT=31337

all: clean elf run

elf: $(ELF_OUT)
$(ELF_OUT): $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake -DBOARD=$(ZEPHYR_BOARD) ..
	cd $(BUILD_DIR) && make -j$(nproc)

$(BUILD_DIR):
	rm -rf $(BUILD_DIR) && mkdir $(BUILD_DIR)

run: $(ELF_OUT)
	$(QEMU) -machine $(QEMU_MACHINE) -kernel $(ELF_OUT) -nographic

debug: $(ELF_OUT)
	$(QEMU) -machine $(QEMU_MACHINE) -kernel $(ELF_OUT) -nographic -gdb tcp::$(GDB_PORT) -S &
	sleep 2
	$(GDB) $(ELF_OUT) -ex "target remote localhost:$(GDB_PORT)"
	pkill qemu

clean:
	rm -rf $(BUILD_DIR)

