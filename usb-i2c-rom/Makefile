include ../.env

toolchain=$(ARM_NONE_EABI_TOOLCHAIN)
CC="$(toolchain)/arm-none-eabi-gcc"
LD="$(toolchain)/arm-none-eabi-ld"  
GDB="$(toolchain)/arm-none-eabi-gdb"
CFLAGS=-mcpu=cortex-m3 -mthumb -g -ffreestanding

rompath=usb-i2c-rom.elf

ifeq ($(ROM_DEBUG),1)
	CFLAGS += -O3 -DDEBUG=1
else
	CFLAGS += -O3
endif

LDFLAGS=-o $(rompath) -T stm32.ld
sources = $(wildcard src/*.c) $(wildcard src/usb/*.c)
objs = $(patsubst %.c, obj/%.o, $(sources))

build: compile link
run: build flash

compile: $(objs)
$(objs): obj/%.o : %.c ../.env
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@ 

link:
	$(LD) $(LDFLAGS) $(objs)

flash:
	openocd -f interface/stlink-v2.cfg \
			-f target/stm32f1x.cfg \
			-c "program $(rompath) verify" \
			-c "arm semihosting enable" \
			-c "reset" \

debug:
	$(GDB) --ex "file $(rompath)" --ex "target extended-remote localhost:3333"

clean: $(objs)
	rm $(objs)