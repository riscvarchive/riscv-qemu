#=======================================================================
# Makefile modified from riscv-tests/isa
#-----------------------------------------------------------------------
-include ../../../config-host.mak

XLEN ?= 64

SIM = ../../../riscv64-softmmu/qemu-system-riscv$(XLEN)
SIM_FLAGS = -M sifive  -nographic -kernel

CROSS   = riscv$(XLEN)-unknown-elf-
CC      = $(CROSS)gcc
AS      = $(CROSS)as
LD      = $(CC)
OBJDUMP = $(CROSS)objdump

RSRC_PATH = $(SRC_PATH)/tests/tcg/riscv

CFLAGS  = -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles
OBJDUMP_FLAGS = --disassemble-all --disassemble-zeroes --section=.text \
		--section=.text.startup --section=.text.init --section=.data


include $(RSRC_PATH)/rv64ui/Makefrag
include $(RSRC_PATH)/rv64uc/Makefrag
include $(RSRC_PATH)/rv64um/Makefrag
include $(RSRC_PATH)/rv64ua/Makefrag
include $(RSRC_PATH)/rv64uf/Makefrag
include $(RSRC_PATH)/rv64ud/Makefrag
include $(RSRC_PATH)/rv64si/Makefrag
include $(RSRC_PATH)/rv64mi/Makefrag
include $(RSRC_PATH)/rv32ui/Makefrag
include $(RSRC_PATH)/rv32uc/Makefrag
include $(RSRC_PATH)/rv32um/Makefrag
include $(RSRC_PATH)/rv32ua/Makefrag
include $(RSRC_PATH)/rv32uf/Makefrag
include $(RSRC_PATH)/rv32si/Makefrag
include $(RSRC_PATH)/rv32mi/Makefrag

#--------------------------------------------------------------------
# Build rules
#--------------------------------------------------------------------

#RISCV_PREFIX ?= riscv$(XLEN)-unknown-elf-
#RISCV_GCC ?= $(RISCV_PREFIX)gcc
#RISCV_GCC_OPTS ?= -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles
#RISCV_OBJDUMP ?= $(RISCV_PREFIX)objdump --disassemble-all --disassemble-zeroes --section=.text --section=.text.startup --section=.text.init --section=.data
#RISCV_SIM ?= spike

#vpath %.S $(src_dir)

vpath %.S $(RSRC_PATH)
#------------------------------------------------------------
# Build assembly tests

%.dump: %
	$(OBJDUMP) $(OBJDUMP_FLAGS) $< > $@

define compile_template

$$($(1)_p_tests): $(1)-p-%: $(1)/%.S
	$$(CC) $(2) $$(CFLAGS) -I$(RSRC_PATH)/env/p -I$(RSRC_PATH)/macros/scalar -T$(RSRC_PATH)/env/p/link.ld $$< -o $$@
$(1)_tests += $$($(1)_p_tests)

$(1)_tests_dump = $$(addsuffix .dump, $$($(1)_tests))

$(1): $$($(1)_tests_dump)

.PHONY: $(1)

tests += $$($(1)_tests)

endef


ifeq ($(XLEN),64)
$(eval $(call compile_template,rv64ui))
$(eval $(call compile_template,rv64um))
$(eval $(call compile_template,rv64ua))
$(eval $(call compile_template,rv64uf))
$(eval $(call compile_template,rv64ud))
else
$(eval $(call compile_template,rv32ui,-m32))
$(eval $(call compile_template,rv32um,-m32))
$(eval $(call compile_template,rv32ua,-m32))
$(eval $(call compile_template,rv32uf,-m32))
endif

tests_dump = $(addsuffix .dump, $(tests))


#------------------------------------------------------------
# Default

all: $(tests_dump)

check: $(tests)
	@pass=0; \
	failed=0; \
	for case in $(tests); do \
        echo $(SIM) $(SIM_FLAGS) ./$$case;\
        $(SIM) $(SIM_FLAGS) ./$$case; \
	ret=$$?; \
	if [ $$ret -ne 0 ]; then \
	echo $$case failed at $$ret; \
	failed=$$(($$failed+1)); \
	else \
	echo $$case pass; \
	pass=$$(($$pass+1)); \
	fi; \
	done; \
	echo pass/failed/all : $$pass/$$failed/$$(($$pass+$$failed))

#------------------------------------------------------------
# Clean up

clean:
	rm -rf $(tests) $(tests_dump)
