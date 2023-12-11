# SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
# SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
#
# SPDX-License-Identifier: BSD-3-Clause

import logging
import cocotb
from cocotb.triggers import Timer
from cocotb.triggers import FallingEdge, RisingEdge
from cocotb.clock import Clock
from numpy import ceil, log2


class TB:
    def __init__(self, dut):
        self.dut = dut

        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)


CLK_PERIOD_NS = 10

SKIP_ALL = False


@cocotb.test(skip=(SKIP_ALL))  # denotes a test
async def add_nums(dut):
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clock, CLK_PERIOD_NS, units="ns").start())

    #    tb.dut.clock_ena.value = 1

    data = cocotb.binary.BinaryValue(n_bits=tb.dut.N.value * tb.dut.DATA_WIDTH.value, bigEndian=False)

    sum = 0
    await Timer(CLK_PERIOD_NS * 10, units="ns")
    tb.log.info("Set data inputs to 0")
    tb.dut.data.value = 0
    await Timer(CLK_PERIOD_NS * 10, units="ns")
    tb.log.info("Verify sum is 0")
    assert tb.dut.result.value == 0
    await Timer(CLK_PERIOD_NS / 2, units="ns")

    tb.log.info("Set data inputs to non-zero values")
    for i in range(tb.dut.N.value):
        data = data | ((i + 1) << (tb.dut.DATA_WIDTH.value * i))
        sum = sum + (i + 1)

    tb.dut.data.value = data
    await Timer(CLK_PERIOD_NS / 2, units="ns")
    await Timer(CLK_PERIOD_NS * (ceil(log2(tb.dut.N.value)) - 1), units="ns")
    tb.log.info("Verify sum is still 0 one clock before pipelined sum pops out")
    assert tb.dut.result.value == 0
    await Timer(CLK_PERIOD_NS * 1, units="ns")
    tb.log.info("Verify pipelined sum pops out on expected clock")
    assert tb.dut.result.value == sum
    await Timer(CLK_PERIOD_NS * 50, units="ns")

    print(sum)
