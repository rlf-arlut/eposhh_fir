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

VERILOG_SOURCES = $(DUTD)/adder_tree.v

# TOPLEVEL is the name of the toplevel module in your Verilog or VHDL file
TOPLEVEL = adder_tree

# MODULE is the basename of the Python test file
MODULE = adder_tree_cocotb 

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim

PLUSARGS = '-lxt2'
