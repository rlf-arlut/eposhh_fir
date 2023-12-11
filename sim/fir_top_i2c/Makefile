# SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
# SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
#
# SPDX-License-Identifier: BSD-3-Clause

# Makefile

# defaults
SIM ?= icarus
TOPLEVEL_LANG ?= verilog

WPWD=$(shell pwd)
DUTD=$(WPWD)/../../rtl

VERILOG_SOURCES = $(DUTD)/fir_top_i2c.v $(DUTD)/i2c_target.v $(DUTD)/i2c_target_wbc.v
VERILOG_SOURCES += $(DUTD)/adder_tree.v $(DUTD)/mult_reg.v $(DUTD)/crc16.v $(DUTD)/snapshot_tx.v

# TOPLEVEL is the name of the toplevel module in your Verilog or VHDL file
TOPLEVEL = fir_top_i2c

# MODULE is the basename of the Python test file
MODULE = fir_top_i2c_cocotb

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim

PLUSARGS = '-lxt2'
