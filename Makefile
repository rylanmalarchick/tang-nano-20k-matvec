# Tang Nano 20K (Gowin GW2AR-18) open-source toolchain Makefile
# Flow: Yosys (synth) -> nextpnr-himbaechel (P&R) -> gowin_pack (bitstream) -> openFPGALoader (flash)

DEVICE     = GW2AR-LV18QN88C8/I7
FAMILY     = GW2A-18C
CST        = constraints/tangnano20k.cst

# Default project to build
PROJECT   ?= blink
TOP       ?= $(PROJECT)

# Source files: project .v files + shared library modules
SRC        = $(wildcard $(PROJECT)/*.v) $(wildcard lib/*.v)

# Output files
BUILD_DIR  = build/$(PROJECT)
JSON       = $(BUILD_DIR)/$(TOP).json
PNR        = $(BUILD_DIR)/$(TOP)_pnr.json
PACK       = $(BUILD_DIR)/$(TOP).fs

.PHONY: all synth pnr pack flash flash-persistent sim clean

all: pack

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Synthesis
synth: $(JSON)
$(JSON): $(SRC) | $(BUILD_DIR)
	yosys -p "read_verilog $(SRC); synth_gowin -top $(TOP) -json $@"

# Place & Route
pnr: $(PNR)
$(PNR): $(JSON) $(CST)
	nextpnr-himbaechel --device $(DEVICE) --vopt family=$(FAMILY) \
		--vopt cst=$(CST) --json $(JSON) --write $@

# Bitstream packing
pack: $(PACK)
$(PACK): $(PNR)
	gowin_pack -d $(FAMILY) -o $@ $(PNR)

# Flash to board via JTAG
flash: $(PACK)
	openFPGALoader -b tangnano20k $(PACK)

# Flash to onboard flash (persistent across power cycles)
flash-persistent: $(PACK)
	openFPGALoader -b tangnano20k -f $(PACK)

# Verilator simulation
# Usage: make sim PROJECT=fixed_point SIM_TOP=complex_mul
# SIM_SRC: override to list specific .v files (default: all project + lib .v files)
SIM_TOP   ?= $(TOP)
SIM_SRC   ?= $(wildcard $(PROJECT)/*.v) $(wildcard lib/*.v)
SIM_TB    ?= $(PROJECT)/$(SIM_TOP)_tb.cpp
SIM_DIR    = build/sim_$(SIM_TOP)

sim: $(SIM_TB)
	mkdir -p $(SIM_DIR)
	verilator --cc $(foreach f,$(SIM_SRC),$(abspath $(f))) \
		--exe $(abspath $(SIM_TB)) \
		--top-module $(SIM_TOP) --Mdir $(SIM_DIR) --build
	$(SIM_DIR)/V$(SIM_TOP)

clean:
	rm -rf build/
