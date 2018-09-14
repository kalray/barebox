CPPFLAGS += -fno-strict-aliasing

board-$(CONFIG_GENERIC) := generic

KALLSYMS += --symbol-prefix=_

ifeq ($(CROSS_COMPILE),)
CROSS_COMPILE    := k1-elf-
endif

DEFAULT_CFLAGS := -nostdlib -fno-builtin -fstrict-align -g
DEFAULT_CFLAGS += -DTEXT_BASE=$(CONFIG_TEXT_BASE)

LIBGCC_PATH = $(dir $(shell $(CC) $(CFLAGS) --print-libgcc-file-name))

CFLAGS += $(DEFAULT_CFLAGS)
AFLAGS += $(DEFAULT_CFLAGS)

LDFLAGS += -m elf64k1

archprepare: maketools

PHONY += maketools

ifneq ($(board-y),)
BOARD  := arch/k1c/boards/$(board-y)/
else
BOARD  :=
endif

#~ common-y += $(BOARD)
common-y += arch/k1c/lib/
common-y += arch/k1c/cpu/

lds-y += arch/k1c/cpu/barebox.lds

cmd_barebox__ ?= $(LD) $(LDFLAGS) $(LDFLAGS_barebox) -o $@ \
      -T $(barebox-lds)                         \
      --start-group $(barebox-common) --end-group  	          \
      -L$(LIBGCC_PATH) -lgcc \
      $(filter-out $(barebox-lds) $(barebox-common) FORCE ,$^)
