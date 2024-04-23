CELL_MK_DIR = $(CELL_SDK)/samples/mk
include $(CELL_MK_DIR)/sdk.makedef.mk

CELL_BUILD_TOOLS = GCC

PPU_AR = ppu-lv2-ar
PPU_RANLIB = ppu-lv2-ranlib
PPU_STRIP = ppu-lv2-strip
PPU_CC = ppu-lv2-gcc
PPU_CXX = ppu-lv2-g++
PPU_LD = ppu-lv2-ld
PPU_CCLD = ppu-lv2-gcc
PPU_CXXLD = ppu-lv2-g++
PPU_PRX_CCLD = ppu-lv2-gcc
PPU_PRX_CXXLD = ppu-lv2-g++
PPU_PRX_EXPORTPICKUP = ppu-lve-prx-exportpickup
PPU_PRX_STRIP = ppu-lv2-prx-strip

PPU_PRX_FLAGS = -mprx -mno-sn-ld -Os -ffunction-sections -fdata-sections -fno-builtin-printf -nodefaultlibs -std=gnu99 -Wno-shadow -Wno-unused-parameter
PPU_CFLAGS = $(PPU_PRX_FLAGS)
PPU_CXXFLAGS += $(PPU_PRX_FLAGS)
PPU_LDFLAGS += $(PPU_PRX_FLAGS)
PPU_PRX_LDFLAGS += -mprx -ffunction-sections -fdata-sections -Wl,--strip-unused-data

PPU_PRX_STRIPFLAGS += --strip-debug --strip-section-header

PPU_INCDIRS += -I$(CELL_TARGET_PATH)/ppu/include/cell/net
PPU_SRCS = libc.c main.c
PPU_PRX_LDLIBS 	= -lusbd_stub -lio_stub -lfs_stub #-ldbg_libio_stub
PPU_PRX_TARGET = xpad.prx

CLEANFILES = $(PRX_DIR)/$(PPU_SPRX_TARGET)

all:
	$(MAKE) $(PPU_OBJS_DEPENDS)
	$(PPU_PRX_STRIP) $(PPU_PRX_STRIPFLAGS) $(PPU_PRX_TARGET)
	scetool -0 SELF -1 TRUE -s FALSE -2 04 -3 1070000052000001 -4 01000002 -5 APP -6 0003004000000000 -A 0001000000000000 -e xpad.prx xpad.sprx

include $(CELL_MK_DIR)/sdk.target.mk
