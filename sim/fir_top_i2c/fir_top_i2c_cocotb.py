# SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
# SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
#
# SPDX-License-Identifier: BSD-3-Clause

import logging
import math
import random
import cocotb
from cocotb.triggers import Timer
from cocotb.triggers import FallingEdge, RisingEdge
from cocotb.clock import Clock
from cocotbext.i2c import I2cMaster
from eposhh_fir import *

# crc16 function
# THIS IS GENERATED CODE.
#
# This code is Public Domain.
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
# RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
# NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
# USE OR PERFORMANCE OF THIS SOFTWARE.

# CRC polynomial coefficients: x^16 + x^15 + x^2 + 1
#                              0xA001 (hex)
# CRC width:                   16 bits
# CRC shift direction:         right

# fmt: off
def crc16(crcIn, data):
    class bitwrapper:
        def __init__(self, value):
            self.value = value
        def __getitem__(self, index):
            return ((self.value >> index) & 1)
        def __setitem__(self, index, value):
            if value:
                self.value |= 1 << index
            else:
                self.value &= ~(1 << index)
    crcIn = bitwrapper(crcIn)
    data = bitwrapper(data)
    ret = bitwrapper(0)
    ret[0] = (crcIn[0] ^ crcIn[1] ^ crcIn[2] ^ crcIn[3] ^ crcIn[4] ^ crcIn[5] ^ crcIn[6] ^ crcIn[7] ^ crcIn[8] ^ data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7])
    ret[1] = (crcIn[9])
    ret[2] = (crcIn[10])
    ret[3] = (crcIn[11])
    ret[4] = (crcIn[12])
    ret[5] = (crcIn[13])
    ret[6] = (crcIn[0] ^ crcIn[14] ^ data[0])
    ret[7] = (crcIn[0] ^ crcIn[1] ^ crcIn[15] ^ data[0] ^ data[1])
    ret[8] = (crcIn[1] ^ crcIn[2] ^ data[1] ^ data[2])
    ret[9] = (crcIn[2] ^ crcIn[3] ^ data[2] ^ data[3])
    ret[10] = (crcIn[3] ^ crcIn[4] ^ data[3] ^ data[4])
    ret[11] = (crcIn[4] ^ crcIn[5] ^ data[4] ^ data[5])
    ret[12] = (crcIn[5] ^ crcIn[6] ^ data[5] ^ data[6])
    ret[13] = (crcIn[6] ^ crcIn[7] ^ data[6] ^ data[7])
    ret[14] = (crcIn[0] ^ crcIn[1] ^ crcIn[2] ^ crcIn[3] ^ crcIn[4] ^ crcIn[5] ^ crcIn[6] ^ data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6])
    ret[15] = (crcIn[0] ^ crcIn[1] ^ crcIn[2] ^ crcIn[3] ^ crcIn[4] ^ crcIn[5] ^ crcIn[6] ^ crcIn[7] ^ data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7])
    return ret.value
# end crc16 function
# fmt: on
class DataValidMonitor:
    """
    Reusable Monitor of one-way control flow (data/valid) streaming data interface

    Args
        clk: clock signal
        valid: control signal noting a transaction occured
        datas: named handles to be sampled when transaction occurs
    """

    def __init__(self, clk, pdws, data, valid):
        self.log = logging.getLogger(f"cocotb.ssmon")
        self.log.setLevel(logging.INFO)
        self._clk = clk
        self._pdws = pdws
        self._data = data
        self._valid = valid
        self._coro = None
        self.log.info("Started snapshot monitor")

    def start(self) -> None:
        """Start monitor"""
        if self._coro is not None:
            raise RuntimeError("Monitor already started")
        self._coro = cocotb.start_soon(self._run())

    def stop(self) -> None:
        """Stop monitor"""
        if self._coro is None:
            raise RuntimeError("Monitor never started")
        self._coro.kill()
        self._coro = None

    async def _run(self) -> None:
        self.log.info("Started snapshot monitor running")
        state = "IDLE"
        self.log.info("Started snapshot monitor running")
        self.log.info("State is " + state)
        data_word = 0
        count = 0

        while True:
            await RisingEdge(self._clk)
            if state == "IDLE":
                self.log.info("PDW Capture IDLE")
                if self._valid.value.binstr != "1":
                    await RisingEdge(self._valid)
                    await RisingEdge(self._clk)
                if self._valid.value == 1:
                    self.log.info("PDW Capture IDLE Moving to CAPTURE")
                    state = "CAPTURE"
                    pdw = [0] * 13
                    total_bits = 0
                    crc16word = 0
                    self.log.debug("PDW Capture CAPTURE on bit " + str(total_bits) + " " + str(self._data.value))
                    data_word = data_word | self._data.value
                    bit = 1
                    word_count = 0
                    total_bits = total_bits + 1
            elif state == "CAPTURE":
                if self._valid.value == 1:
                    self.log.debug("PDW Capture CAPTURE on bit " + str(total_bits) + " " + str(self._data.value))
                    data_word = data_word | (self._data.value << bit)

                    self.log.debug("data_word is " + str(data_word))
                    if bit == 15:
                        pdw[word_count] = data_word

                        if total_bits < 192:
                            crc16word = crc16(crc16word, (data_word >> 8) & 0xFF)
                            self.log.debug("crc16 is " + str(crc16word))
                            self.log.debug((data_word >> 8) & 0xFF)
                            self.log.debug(crc16word)
                        self.log.debug(pdw)
                        data_word = 0
                        bit = 0
                        word_count = word_count + 1
                    else:
                        if bit == 7:
                            if total_bits < 192:
                                crc16word = crc16(crc16word, data_word & 0xFF)
                                self.log.debug("crc16 is " + str(crc16word))
                                self.log.debug((data_word) & 0xFF)
                        bit = bit + 1

                    total_bits = total_bits + 1
                else:
                    assert total_bits == 208
                    assert crc16word == pdw[12]
                    self.log.info("Moving to IDLE")
                    state = "IDLE"

class TB:
    pdws = []

    def __init__(self, dut):
        self.dut = dut

        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)

        # fmt: off
        self.i2c_master = I2cMaster(
            sda     = dut.i2c_sda_o,
            sda_o   = dut.i2c_sda_i,
            scl     = dut.i2c_scl_o,
            scl_o   = dut.i2c_scl_i,
            speed   = 800e3,
        )
        self.ssmon = DataValidMonitor(
            clk = self.dut.clk,
            data = self.dut.pdw_data,
            valid = self.dut.pdw_frame,
            pdws = self.pdws
        )
        # fmt: on
        self.efc = EposhhFirCocotb(self.i2c_master)

CLK_PERIOD_NS = 10

SKIP_ALL = False
SKIP_TEST_BIG_TEST = False
SKIP_TEST_BASIC_SAMPLES = False
SKIP_I2C_WRITE_THRESHOLDS = False
SKIP_SIMPLE_I2C_WRITE = False
SKIP_EFC_I2C_CONTROL = False
SKIP_EFC_I2C_COEFFS = False
SKIP_EFC_I2C_SSDELAY = False
SKIP_EFC_I2C_THRESHOLDS = False
SKIP_POSITIVE_LO_THRESHOLD = False
SKIP_POSITIVE_HI_THRESHOLD = False
SKIP_NEGATIVE_LO_THRESHOLD = False
SKIP_NEGATIVE_HI_THRESHOLD = False
SKIP_SINGLE_SNAPSHOT_POSITIVE = False
SKIP_SINGLE_SNAPSHOT_NEGATIVE = False
SKIP_SINGLE_SNAPSHOT_1_DELAY = False
SKIP_SINGLE_SNAPSHOT_DELAY = False
SKIP_SINGLE_SNAPSHOT_MAX_DELAY = False
SKIP_SNAPSHOT_SAME_VAL_NO_REPLACE_MORE_POS = False
SKIP_SNAPSHOT_SAME_VAL_NO_REPLACE_MORE_NEG = False
SKIP_SNAPSHOT_SAME_VAL_NO_REPLACE_LARGE_MAG = False
SKIP_SINGLE_SNAPSHOT_REPLACEMENT_MORE_POS = False
SKIP_SINGLE_SNAPSHOT_REPLACEMENT_MORE_NEG = False
SKIP_SINGLE_SNAPSHOT_REPLACEMENT_LARGE_MAG = False
SKIP_DOUBLE_SNAPSHOT_MORE_POS = False
SKIP_DOUBLE_SNAPSHOT_MORE_NEG = False
SKIP_DOUBLE_SNAPSHOT_LARGE_MAG = False

@cocotb.test(skip=(SKIP_ALL & SKIP_TEST_BIG_TEST))
async def test_big_test(dut):
    """Big Test

    This test will configure the FIR filter, stream in samples, and trigger
    all replacement mechanisms.
    """
    tb = TB(dut)
    tb.dut.sample_i.value = 0
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(10, "us")
    await tb.efc.set_threshold_hi(0x7FFFFFFFF)
    await tb.efc.set_threshold_lo(0x800000000)

    tb.ssmon.start()

    # make all coefficients unity for simplicity
    coefs = [1 for i in range(tb.efc.HW_NUM_TAPS)]

    await tb.efc.write_fir_coefficients(coefs)
    await tb.efc.set_threshold_hi(0x7250)
    await tb.efc.set_threshold_lo(-5)
    await tb.efc.set_ss_delay(10)
    await tb.efc.set_snapshot_replacement("more_pos")

    samples = 1
    pdw_frame_low = 0
    for i in range(100):
        tb.dut.sample_i.value = samples
        await RisingEdge(tb.dut.clk)
        samples = samples + 1

    tb.dut.sample_i.value = 0
    while pdw_frame_low < 10:
        for i in range(100):
            await RisingEdge(tb.dut.clk)
        if tb.dut.pdw_frame.value == 0:
            pdw_frame_low = pdw_frame_low + 1
        else:
            pdw_frame_low = 0
    print(tb.pdws)

@cocotb.test(skip=(SKIP_ALL & SKIP_TEST_BASIC_SAMPLES))  # denotes a test
async def test_basic_samples(dut):
    """Basic samples input test

    This test streams in 10 signed samples and directly checks the values inside
    the module after 10 clocks.
    """
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(10, "us")
    await tb.efc.set_threshold_hi(0x7FFFFFFFF)
    await tb.efc.set_threshold_lo(0x800000000)

    tb.dut.sample_i.value = 0x55
    await RisingEdge(tb.dut.clk)
    tb.dut.sample_i.value = 0xAA
    await RisingEdge(tb.dut.clk)
    tb.dut.sample_i.value = 0x55
    await RisingEdge(tb.dut.clk)

    samples = 5

    for i in range(10):
        tb.dut.sample_i.value = samples
        await RisingEdge(tb.dut.clk)
        tb.dut.sample_i.value = 0
        await FallingEdge(tb.dut.clk)
        samples = samples - 1

    await RisingEdge(tb.dut.clk)
    await RisingEdge(tb.dut.clk)
    samples = 5
    for i in range(9, -1, -1):
        value = tb.dut._id("samples[" + str(i) + "]", extended=False).value
        print("samples[" + str(i) + "] is " + str(value.signed_integer))
        print("samples[" + str(i) + "] is " + str(value.binstr))
        assert value.signed_integer == (samples << 8)
        samples = samples - 1

    # Handy reminder how to access various value formats in cocotb
    #    value = tb.dut._id("samples[0]",extended=False).value
    #    print(value.binstr)
    #    print(value.integer)
    #    print(value.signed_integer)
    #    print(value.buff)
    #    print(value.n_bits)
    await Timer(10, "us")

@cocotb.test(skip=(SKIP_ALL & SKIP_I2C_WRITE_THRESHOLDS))  # denotes a test
async def i2c_write_thresholds(dut):
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())

    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")
    await tb.efc.set_threshold_hi(0x7FFFFFFFF)
    await tb.efc.set_threshold_lo(0x800000000)

    await tb.efc.set_ss_delay(0)
    await tb.efc.check_ss_delay(0)
    await tb.efc.set_ss_delay(5)
    await tb.efc.check_ss_delay(5)
    thresholdlo0 = 0xB0D1
    thresholdlo1 = 0xDEAD
    thresholdlo2 = 0x01FF

    thresholdhi0 = 0xBD01
    thresholdhi1 = 0xDAED
    thresholdhi2 = 0x0177

    threshold = [thresholdlo0, thresholdlo1, thresholdlo2, thresholdhi0, thresholdhi1, thresholdhi2]

    data = 0x0000
    for i in range(6):
        addr = (i * 2) + 0x20
        await tb.i2c_master.write(tb.dut.I2C_ADDR.value, addr.to_bytes(2, "big") + data.to_bytes(2, "little"))
        await tb.i2c_master.send_stop()

    data = 0xFFFF
    for i in range(6):
        addr = (i * 2) + 0x20
        await tb.i2c_master.write(tb.dut.I2C_ADDR.value, addr.to_bytes(2, "big") + data.to_bytes(2, "little"))
        await tb.i2c_master.send_stop()

    data = 0x0000
    for i in range(6):
        addr = (i * 2) + 0x20
        await tb.i2c_master.write(tb.dut.I2C_ADDR.value, addr.to_bytes(2, "big") + data.to_bytes(2, "little"))
        await tb.i2c_master.send_stop()

    for i in range(6):
        addr = (i * 2) + 0x20
        await tb.i2c_master.write(tb.dut.I2C_ADDR.value, addr.to_bytes(2, "big") + threshold[i].to_bytes(2, "little"))
        await tb.i2c_master.send_stop()

    await Timer(5, "us")

    for i in range(6):
        addr = (i * 2) + 0x20
        await tb.i2c_master.write(tb.dut.I2C_ADDR.value, addr.to_bytes(2, "big"))
        data = await tb.i2c_master.read(tb.dut.I2C_ADDR.value, 2)
        await tb.i2c_master.send_stop()
        tb.log.info("Read data: %s", data.hex())
        assert threshold[i].to_bytes(2, "little") == data

    await Timer(5, "us")

@cocotb.test(skip=(SKIP_ALL & SKIP_SIMPLE_I2C_WRITE))  # denotes a test
async def simple_i2c_write(dut):

    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())

    i2c_addr = tb.dut.I2C_ADDR.value

    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")
    await tb.efc.set_threshold_hi(0x7FFFFFFFF)
    await tb.efc.set_threshold_lo(0x800000000)

    test_addr = b"\x00\x00"
    test_data = b"\x56\x78"

    await tb.i2c_master.write(tb.dut.I2C_ADDR.value, test_addr + test_data)
    await tb.i2c_master.send_stop()

    await Timer(1, "us")

    await tb.i2c_master.write(tb.dut.I2C_ADDR.value, test_addr)
    data = await tb.i2c_master.read(tb.dut.I2C_ADDR.value, 2)
    await tb.i2c_master.send_stop()

    tb.log.info("Read data: %s", data.hex())
    assert test_data == data

    await tb.efc.bus_write(0x0, 0x1234)
    data = await tb.efc.bus_read(0x0)
    test_data = b"\x34\x12"
    print(data)
    print(data.hex())
    assert 0x1234 == int.from_bytes(data, "little")

    await tb.efc.bus_read_compare(0x0, 0x1234)

    await tb.efc.set_snapshot_replacement("large_mag")

    await tb.efc.set_threshold_hi(0x712345678)
    await tb.efc.set_threshold_lo(-1 * 2 ** (tb.efc.HW_FIR_OUT_WDITH - 1))
    await Timer(1, "us")

    val = 257

    for i in range(0, 20, 2):
        await tb.i2c_master.write(tb.dut.I2C_ADDR.value, i.to_bytes(2, "big") + val.to_bytes(2, "little"))
        await tb.i2c_master.send_stop()
        val = val + 1

    val = 257
    for i in range(0, 20, 2):
        await tb.i2c_master.write(tb.dut.I2C_ADDR.value, i.to_bytes(2, "big"))
        data = await tb.i2c_master.read(tb.dut.I2C_ADDR.value, 2)
        await tb.i2c_master.send_stop()
        tb.log.info("Read data: %s", data.hex())
        assert val.to_bytes(2, "little") == data
        val = val + 1

    # thresh0 = b"\xD1\xB0"
    # thresh1 = b"\xAD\xDE"
    # thresh2 = b"\xFF\x01"
    thresh0 = 0xB0D1
    thresh1 = 0xDEAD
    thresh2 = 0x01FF

    addr = 0x20
    await tb.i2c_master.write(tb.dut.I2C_ADDR.value, addr.to_bytes(2, "big") + thresh0.to_bytes(2, "little"))
    await tb.i2c_master.send_stop()
    addr = 0x22
    await tb.i2c_master.write(tb.dut.I2C_ADDR.value, addr.to_bytes(2, "big") + thresh1.to_bytes(2, "little"))
    await tb.i2c_master.send_stop()
    addr = 0x24
    await tb.i2c_master.write(tb.dut.I2C_ADDR.value, addr.to_bytes(2, "big") + thresh2.to_bytes(2, "little"))
    await tb.i2c_master.send_stop()

    await Timer(10, "us")

@cocotb.test(skip=(SKIP_ALL & SKIP_EFC_I2C_CONTROL))  # denotes a test
async def efc_i2c_control(dut):
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    i2c_addr = tb.dut.I2C_ADDR.value
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")
    replacement_type_list = ["more_neg", "more_pos", "large_mag"]
    expected_bits_set = [0b1001, 0b0101, 0b0011]
    for i in range(3):
        await tb.efc.set_snapshot_replacement(replacement_type_list[i])
        assert tb.dut.control.value.integer == expected_bits_set[i]

@cocotb.test(skip=(SKIP_ALL & SKIP_EFC_I2C_COEFFS))  # denotes a test
async def efc_i2c_coeffs(dut):
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    i2c_addr = tb.dut.I2C_ADDR.value
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")
    min_coeff = -1 * (2**(tb.efc.HW_COEF_WIDTH - 1))
    max_coeff = 2**(tb.efc.HW_COEF_WIDTH - 1) - 1
    coeffs = [min_coeff for i in range(tb.efc.HW_NUM_TAPS)]
    await tb.efc.write_fir_coefficients([0xFFFF & coeff for coeff in coeffs])
    await tb.efc.check_fir_coefficients([0xFFFF & coeff for coeff in coeffs])
    for i in range(tb.efc.HW_NUM_TAPS):
        assert tb.dut._id("coeff[" + str(i) + "]", extended=False).value.signed_integer == coeffs[i]
    coeffs = [max_coeff for i in range(tb.efc.HW_NUM_TAPS)]
    await tb.efc.write_fir_coefficients([0xFFFF & coeff for coeff in coeffs])
    await tb.efc.check_fir_coefficients([0xFFFF & coeff for coeff in coeffs])
    for i in range(tb.efc.HW_NUM_TAPS):
        assert tb.dut._id("coeff[" + str(i) + "]", extended=False).value.signed_integer == coeffs[i]
    coeffs = [random.randint(min_coeff, max_coeff) for i in range(tb.efc.HW_NUM_TAPS)]
    await tb.efc.write_fir_coefficients([0xFFFF & coeff for coeff in coeffs])
    await tb.efc.check_fir_coefficients([0xFFFF & coeff for coeff in coeffs])
    for i in range(tb.efc.HW_NUM_TAPS):
        assert tb.dut._id("coeff[" + str(i) + "]", extended=False).value.signed_integer == coeffs[i]

@cocotb.test(skip=(SKIP_ALL & SKIP_EFC_I2C_SSDELAY))  # denotes a test
async def efc_i2c_ssdelay(dut):
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    i2c_addr = tb.dut.I2C_ADDR.value
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")
    ss_delay_list = [0, 7, 15]
    for ss_delay in ss_delay_list:
        await tb.efc.set_ss_delay(ss_delay)
        await tb.efc.check_ss_delay(ss_delay)
        assert tb.dut.ss_delay.value == ss_delay

@cocotb.test(skip=(SKIP_ALL & SKIP_EFC_I2C_THRESHOLDS))  # denotes a test
async def efc_i2c_thresholds(dut):
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    i2c_addr = tb.dut.I2C_ADDR.value
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")
    min_thresh = -1 * (2**(36 - 1))
    max_thresh = 2**(36 - 1) - 1
    # threshold_lo
    await tb.efc.set_threshold_lo(min_thresh)
    await RisingEdge(tb.dut.clk)
    assert tb.dut.threshold_lo.value.signed_integer == min_thresh
    await tb.efc.set_threshold_lo(0)
    await RisingEdge(tb.dut.clk)
    assert tb.dut.threshold_lo.value.signed_integer == 0
    await tb.efc.set_threshold_lo(max_thresh)
    await RisingEdge(tb.dut.clk)
    assert tb.dut.threshold_lo.value.signed_integer == max_thresh
    #threshold_hi
    await tb.efc.set_threshold_hi(min_thresh)
    await RisingEdge(tb.dut.clk)
    assert tb.dut.threshold_hi.value.signed_integer == min_thresh
    await tb.efc.set_threshold_hi(0)
    await RisingEdge(tb.dut.clk)
    assert tb.dut.threshold_hi.value.signed_integer == 0
    await tb.efc.set_threshold_hi(max_thresh)
    await RisingEdge(tb.dut.clk)
    assert tb.dut.threshold_hi.value.signed_integer == max_thresh

async def helper_i2c_write_thresholds(tb, threshold_lo_array, threshold_hi_array):
    # Set threshold values using I2C.
    # Thresholds are 36 bit values, split into 2x 16 bit values, then a 4 bit value
    #threshold = [*threshold_lo_array, *threshold_hi_array]
    threshold = [*threshold_hi_array, *threshold_lo_array]
    for i in range(6):
        addr = (i * 2) + 0x20
        await tb.i2c_master.write(tb.dut.I2C_ADDR.value, addr.to_bytes(2, "big") + threshold[i].to_bytes(2, "little"))
        await tb.i2c_master.send_stop()

async def helper_generate_fir_out(tb, samples, coeffs):
    """
    Assumes 16-bit samples
    """
    for i in range(tb.efc.HW_NUM_TAPS):
        tb.dut._id("coeff[" + str(i) + "]", extended=False).value = coeffs[i]

    for i in range(tb.efc.HW_NUM_TAPS):
        tb.dut.sample_i.value = 0xFF & (samples[i] >> 8)
        await RisingEdge(tb.dut.clk)
        tb.dut.sample_i.value = 0xFF & samples[i]
        await FallingEdge(tb.dut.clk)

    # clock cycle delays for shifting in sample values
    for i in range(2):
        await RisingEdge(tb.dut.clk)

    # 1 clock for multiply, clog2(num_taps) clocks for adder tree
    for i in range(1+math.ceil(math.log2(tb.efc.HW_NUM_TAPS))):
        await RisingEdge(tb.dut.clk)

@cocotb.test(skip=(SKIP_ALL & SKIP_POSITIVE_LO_THRESHOLD))  # denotes a test
async def positive_lo_threshold(dut):
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    i2c_addr = tb.dut.I2C_ADDR.value
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")

    threshold_lo = [100, 0, 0]
    threshold_hi = [10000, 0, 0]
    await helper_i2c_write_thresholds(tb, threshold_lo, threshold_hi)

    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples_99 = [*[10 for i in range(tb.efc.HW_NUM_TAPS-1)], 9]
    samples_100 = [10 for i in range(tb.efc.HW_NUM_TAPS)]
    samples_101 = [*[10 for i in range(tb.efc.HW_NUM_TAPS-1)], 11]

    await helper_generate_fir_out(tb, samples_99, coeffs)
    assert tb.dut.fir_out.value.signed_integer == 99
    assert tb.dut.fir_out_lte_thresh_lo.value.integer == 1

    await helper_generate_fir_out(tb, samples_100, coeffs)
    assert tb.dut.fir_out.value.signed_integer == 100
    assert tb.dut.fir_out_lte_thresh_lo.value.integer == 1

    await helper_generate_fir_out(tb, samples_101, coeffs)
    assert tb.dut.fir_out.value.signed_integer == 101
    assert tb.dut.fir_out_lte_thresh_lo.value.integer == 0
    await Timer(10, "us")

@cocotb.test(skip=(SKIP_ALL & SKIP_POSITIVE_HI_THRESHOLD))  # denotes a test
async def positive_hi_threshold(dut):
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    i2c_addr = tb.dut.I2C_ADDR.value
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")

    threshold_lo = [10, 0, 0]
    threshold_hi = [100, 0, 0]
    await helper_i2c_write_thresholds(tb, threshold_lo, threshold_hi)

    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples_99 = [*[10 for i in range(tb.efc.HW_NUM_TAPS-1)], 9]
    samples_100 = [10 for i in range(tb.efc.HW_NUM_TAPS)]
    samples_101 = [*[10 for i in range(tb.efc.HW_NUM_TAPS-1)], 11]

    await helper_generate_fir_out(tb, samples_99, coeffs)
    assert tb.dut.fir_out.value.signed_integer == 99
    assert tb.dut.fir_out_gte_thresh_hi.value.integer == 0

    await helper_generate_fir_out(tb, samples_100, coeffs)
    assert tb.dut.fir_out.value.signed_integer == 100
    assert tb.dut.fir_out_gte_thresh_hi.value.integer == 1

    await helper_generate_fir_out(tb, samples_101, coeffs)
    assert tb.dut.fir_out.value.signed_integer == 101
    assert tb.dut.fir_out_gte_thresh_hi.value.integer == 1
    await Timer(10, "us")

@cocotb.test(skip=(SKIP_ALL & SKIP_NEGATIVE_LO_THRESHOLD))  # denotes a test
async def negative_lo_threshold(dut):
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    i2c_addr = tb.dut.I2C_ADDR.value
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")

    threshold_lo = [0xFF9C, 0XFFFF, 0x000F]
    threshold_hi = [10, 0, 0]
    await helper_i2c_write_thresholds(tb, threshold_lo, threshold_hi)

    coeffs = [-1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples_99 = [*[10 for i in range(tb.efc.HW_NUM_TAPS-1)], 9]
    samples_100 = [10 for i in range(tb.efc.HW_NUM_TAPS)]
    samples_101 = [*[10 for i in range(tb.efc.HW_NUM_TAPS-1)], 11]

    await helper_generate_fir_out(tb, samples_99, coeffs)
    assert tb.dut.fir_out.value.signed_integer == -99
    assert tb.dut.fir_out_lte_thresh_lo.value.integer == 0

    await helper_generate_fir_out(tb, samples_100, coeffs)
    assert tb.dut.fir_out.value.signed_integer == -100
    assert tb.dut.fir_out_lte_thresh_lo.value.integer == 1

    await helper_generate_fir_out(tb, samples_101, coeffs)
    assert tb.dut.fir_out.value.signed_integer == -101
    assert tb.dut.fir_out_lte_thresh_lo.value.integer == 1
    await Timer(10, "us")

@cocotb.test(skip=(SKIP_ALL & SKIP_NEGATIVE_HI_THRESHOLD))  # denotes a test
async def negative_hi_threshold(dut):
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    i2c_addr = tb.dut.I2C_ADDR.value
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")

    threshold_lo = [0xFC18, 0xFFFF, 0x000F]
    threshold_hi = [0xFF9C, 0XFFFF, 0x000F]
    await helper_i2c_write_thresholds(tb, threshold_lo, threshold_hi)

    coeffs = [-1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples_99 = [*[10 for i in range(tb.efc.HW_NUM_TAPS-1)], 9]
    samples_100 = [10 for i in range(tb.efc.HW_NUM_TAPS)]
    samples_101 = [*[10 for i in range(tb.efc.HW_NUM_TAPS-1)], 11]

    await helper_generate_fir_out(tb, samples_99, coeffs)
    assert tb.dut.fir_out.value.signed_integer == -99
    assert tb.dut.fir_out_gte_thresh_hi.value.integer == 1

    await helper_generate_fir_out(tb, samples_100, coeffs)
    assert tb.dut.fir_out.value.signed_integer == -100
    assert tb.dut.fir_out_gte_thresh_hi.value.integer == 1

    await helper_generate_fir_out(tb, samples_101, coeffs)
    assert tb.dut.fir_out.value.signed_integer == -101
    assert tb.dut.fir_out_gte_thresh_hi.value.integer == 0
    await Timer(10, "us")

def calc_pdw_crc16(pdw):
    l = len(pdw)
    crc16_reg = cocotb.binary.BinaryValue(0, n_bits=16, bigEndian=False)
    if l % 8 != 0:
        print(f"l = {l}")
        raise RuntimeError("Expects number of bits in pdw to be divisible by 8")
    for i in range(int(l / 8)):
        crc16_reg = crc16(crc16_reg, pdw[(i + 1) * 8 - 1:i * 8])
    return crc16_reg

def fir_check(coeffs, samples):
    num_taps = len(coeffs)
    if len(coeffs) != len(samples):
        raise "fir_check expects len(coeffs) = len(samples)"
    fir_out = 0
    for i in range(num_taps):
        fir_out = fir_out + (coeffs[i])*(samples[i])
    return fir_out

async def test_single_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_idx, additional_ss_idxs=[]):
    tb.dut.sample_i.value = 0x00
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    i2c_addr = tb.dut.I2C_ADDR.value
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")

    await tb.efc.set_threshold_lo(thresh_lo)
    await tb.efc.set_threshold_hi(thresh_hi)
    await tb.efc.set_ss_delay(ss_delay)
    await tb.efc.set_snapshot_replacement(replacement_type)

    await tb.efc.write_fir_coefficients([0xFFFF & coeff for coeff in coeffs])

    clks_before_valid_output = tb.efc.HW_NUM_TAPS + 1 + math.ceil(math.log2(tb.efc.HW_NUM_TAPS))
    for i in range(len(samples)):
        tb.dut.sample_i.value = 0xFF & (samples[i] >> 8)
        await RisingEdge(tb.dut.clk)
        tb.dut.sample_i.value = 0xFF & samples[i]
        await FallingEdge(tb.dut.clk)
        if i >= clks_before_valid_output:
            if i - clks_before_valid_output + tb.efc.HW_NUM_TAPS < len(samples):
                fir_out = tb.dut._id("fir_out",extended=False).value
                expected_samples = samples[(i - clks_before_valid_output):(i - clks_before_valid_output + tb.efc.HW_NUM_TAPS)]
                expected_fir_out = fir_check(coeffs, expected_samples)
                assert fir_out.signed_integer == expected_fir_out
            if replacement_type == "more_pos":
                assert tb.dut.fir_out_lte_thresh_lo.value.integer == 0
            elif replacement_type == "more_neg":
                assert tb.dut.fir_out_gte_thresh_hi.value.integer == 0
            if (i - clks_before_valid_output) == expected_ss_idx:
                #expected_samples = samples[(i - clks_before_valid_output):(i - clks_before_valid_output + tb.efc.HW_NUM_TAPS)]
                expected_pdw = cocotb.binary.BinaryValue(0, n_bits=(tb.dut.SS_BUFF_SZ.value + 16), bigEndian=False)
                expected_pdw[tb.dut.ABS_TIME_WIDTH.value:0] = tb.dut.abs_time_cntr.value.integer
                for j in range(tb.efc.HW_NUM_TAPS):
                    expected_pdw[tb.dut.ABS_TIME_WIDTH.value + (j + 1) * tb.efc.HW_SAMPLE_WIDTH - 1:tb.dut.ABS_TIME_WIDTH.value + j * tb.efc.HW_SAMPLE_WIDTH] = 0xFFFF & expected_samples[j]
                expected_pdw[tb.dut.SS_BUFF_SZ.value + 16 - 1: tb.dut.SS_BUFF_SZ.value] = calc_pdw_crc16(expected_pdw[tb.dut.SS_BUFF_SZ.value - 1:0])
                if replacement_type == "more_pos":
                    assert tb.dut.fir_out_gte_thresh_hi.value.integer == 1
                elif replacement_type == "more_neg":
                    assert tb.dut.fir_out_lte_thresh_lo.value.integer == 1
            else:
                if replacement_type == "more_pos":
                    if (i - clks_before_valid_output) in additional_ss_idxs:
                        assert tb.dut.fir_out_gte_thresh_hi.value.integer == 1
                    else:
                        assert tb.dut.fir_out_gte_thresh_hi.value.integer == 0
                elif replacement_type == "more_neg":
                    if (i - clks_before_valid_output) in additional_ss_idxs:
                        assert tb.dut.fir_out_lte_thresh_lo.value.integer == 1
                    else:
                        assert tb.dut.fir_out_lte_thresh_lo.value.integer == 0
                #elif replacement_type == "large_mag":
                    #print(f"ss_buff = {tb.dut.ss_buff.value}")
            if (i - clks_before_valid_output) > (expected_ss_idx + 2 + ss_delay) and (i - clks_before_valid_output) <= (expected_ss_idx + tb.dut.SS_BUFF_SZ.value + 16 + 2 + ss_delay):
                assert tb.dut.pdw_frame.value.integer == 1
                assert tb.dut.pdw_data.value == expected_pdw[i - clks_before_valid_output - (expected_ss_idx + 3 + ss_delay)].value
            else:
                assert tb.dut.pdw_frame.value == 0

    await Timer(10, "us")

@cocotb.test(skip=(SKIP_ALL & SKIP_SINGLE_SNAPSHOT_POSITIVE))  # denotes a test
async def single_snapshot_positive(dut):
    tb = TB(dut)
    thresh_lo = -1
    thresh_hi = 10 * tb.efc.HW_NUM_TAPS
    ss_delay = 0
    replacement_type = "more_pos"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    # ramp from 0 to all 10s for num_taps then back to 0
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[10 for i in range(tb.efc.HW_NUM_TAPS)], \
               *[i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(tb.efc.HW_NUM_TAPS * tb.efc.HW_SAMPLE_WIDTH + tb.dut.ABS_TIME_WIDTH.value + 16)]]
    expected_ss_idx = 3 * tb.efc.HW_NUM_TAPS

    await test_single_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_idx)

@cocotb.test(skip=(SKIP_ALL & SKIP_SINGLE_SNAPSHOT_NEGATIVE))  # denotes a test
async def single_snapshot_negative(dut):
    tb = TB(dut)
    thresh_lo = -10 * tb.efc.HW_NUM_TAPS
    thresh_hi = 1
    ss_delay = 0
    replacement_type = "more_neg"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    # ramp from 0 to all -10s for num_taps then back to 0
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[-10 for i in range(tb.efc.HW_NUM_TAPS)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(tb.efc.HW_NUM_TAPS * tb.efc.HW_SAMPLE_WIDTH + tb.dut.ABS_TIME_WIDTH.value + 16)]]
    expected_ss_idx = 3 * tb.efc.HW_NUM_TAPS
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]

    await test_single_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_idx)

@cocotb.test(skip=(SKIP_ALL & SKIP_SINGLE_SNAPSHOT_1_DELAY))  # denotes a test
async def single_snapshot_1_delay(dut):
    tb = TB(dut)
    thresh_lo = -10 * tb.efc.HW_NUM_TAPS
    thresh_hi = 1
    ss_delay = 1
    replacement_type = "more_neg"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    # ramp from 0 to all -10s for num_taps then back to 0
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[-10 for i in range(tb.efc.HW_NUM_TAPS)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(tb.efc.HW_NUM_TAPS * tb.efc.HW_SAMPLE_WIDTH + tb.dut.ABS_TIME_WIDTH.value + 16)]]
    expected_ss_idx = 3 * tb.efc.HW_NUM_TAPS
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]

    await test_single_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_idx)

@cocotb.test(skip=(SKIP_ALL & SKIP_SINGLE_SNAPSHOT_DELAY))  # denotes a test
async def single_snapshot_delay(dut):
    tb = TB(dut)
    thresh_lo = -10 * tb.efc.HW_NUM_TAPS
    thresh_hi = 1
    ss_delay = 7
    replacement_type = "more_neg"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    # ramp from 0 to all -10s for num_taps then back to 0
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[-10 for i in range(tb.efc.HW_NUM_TAPS)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(tb.efc.HW_NUM_TAPS * tb.efc.HW_SAMPLE_WIDTH + tb.dut.ABS_TIME_WIDTH.value + 16)]]
    expected_ss_idx = 3 * tb.efc.HW_NUM_TAPS
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]

    await test_single_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_idx)

@cocotb.test(skip=(SKIP_ALL & SKIP_SINGLE_SNAPSHOT_MAX_DELAY))  # denotes a test
async def single_snapshot_max_delay(dut):
    tb = TB(dut)
    thresh_lo = -10 * tb.efc.HW_NUM_TAPS
    thresh_hi = 1
    max_delay = 2**(tb.dut.RF_SS_DELAY_WIDTH.value) - 1
    ss_delay = max_delay
    replacement_type = "more_neg"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    # ramp from 0 to all -10s for num_taps then back to 0
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[-10 for i in range(tb.efc.HW_NUM_TAPS)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(tb.efc.HW_NUM_TAPS * tb.efc.HW_SAMPLE_WIDTH + tb.dut.ABS_TIME_WIDTH.value + 16)]]
    expected_ss_idx = 3 * tb.efc.HW_NUM_TAPS
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]

    await test_single_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_idx)

@cocotb.test(skip=(SKIP_ALL & SKIP_SNAPSHOT_SAME_VAL_NO_REPLACE_MORE_POS))  # denotes a test
async def snapshot_same_val_no_replace_more_pos(dut):
    tb = TB(dut)
    thresh_lo = -1
    thresh_hi = 10 * tb.efc.HW_NUM_TAPS
    ss_delay = 8
    replacement_type = "more_pos"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    extra_ss_cycles = 5
    # ramp from 0 to all 10s for num_taps then back to 0
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[10 for i in range(tb.efc.HW_NUM_TAPS + extra_ss_cycles)], \
               *[i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(tb.efc.HW_NUM_TAPS * tb.efc.HW_SAMPLE_WIDTH + tb.dut.ABS_TIME_WIDTH.value + 16 + ss_delay)]]
    expected_ss_idx = 3 * tb.efc.HW_NUM_TAPS
    additional_ss_idxs = [expected_ss_idx + i + 1 for i in range(extra_ss_cycles)]

    await test_single_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_idx, additional_ss_idxs)

@cocotb.test(skip=(SKIP_ALL & SKIP_SNAPSHOT_SAME_VAL_NO_REPLACE_MORE_NEG))  # denotes a test
async def snapshot_same_val_no_replace_more_neg(dut):
    tb = TB(dut)
    thresh_lo = -10 * tb.efc.HW_NUM_TAPS
    thresh_hi = 1
    ss_delay = 9
    replacement_type = "more_neg"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    extra_ss_cycles = 4
    # ramp from 0 to all 10s for num_taps then back to 0
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[-10 for i in range(tb.efc.HW_NUM_TAPS + extra_ss_cycles)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(tb.efc.HW_NUM_TAPS * tb.efc.HW_SAMPLE_WIDTH + tb.dut.ABS_TIME_WIDTH.value + 16 + ss_delay)]]
    expected_ss_idx = 3 * tb.efc.HW_NUM_TAPS
    additional_ss_idxs = [expected_ss_idx + i + 1 for i in range(extra_ss_cycles)]

    await test_single_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_idx, additional_ss_idxs)

@cocotb.test(skip=(SKIP_ALL & SKIP_SNAPSHOT_SAME_VAL_NO_REPLACE_LARGE_MAG))  # denotes a test
async def snapshot_same_val_no_replace_large_mag(dut):
    tb = TB(dut)
    thresh_lo = -10 * tb.efc.HW_NUM_TAPS
    thresh_hi = 10 * tb.efc.HW_NUM_TAPS
    ss_delay = 14
    replacement_type = "large_mag"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    # ramp from 0 to all 10s for num_taps then back to 0
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[10 for i in range(tb.efc.HW_NUM_TAPS)], \
               -60, -60, -50, 50, -30, 30, 20, -10, 10,
               *[i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(tb.efc.HW_NUM_TAPS * tb.efc.HW_SAMPLE_WIDTH + tb.dut.ABS_TIME_WIDTH.value + 16 + ss_delay)]]
    expected_ss_idx = 3 * tb.efc.HW_NUM_TAPS
    additional_ss_idxs = [expected_ss_idx + i for i in [3, 5, 13]]

    await test_single_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_idx, additional_ss_idxs)

async def test_snapshot_replacement(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_replacement_idxs,
                                expected_ss_non_replacement_idxs=[]):
    tb.dut.sample_i.value = 0x00
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    i2c_addr = tb.dut.I2C_ADDR.value
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")

    await tb.efc.set_threshold_lo(thresh_lo)
    await tb.efc.set_threshold_hi(thresh_hi)
    await tb.efc.set_ss_delay(ss_delay)
    await tb.efc.set_snapshot_replacement(replacement_type)

    await tb.efc.write_fir_coefficients([0xFFFF & coeff for coeff in coeffs])

    clks_before_valid_output = tb.efc.HW_NUM_TAPS + 1 + math.ceil(math.log2(tb.efc.HW_NUM_TAPS))
    expected_pdw = cocotb.binary.BinaryValue(0, n_bits=(tb.dut.SS_BUFF_SZ.value + 16), bigEndian=False)

    for i in range(len(samples)):
        tb.dut.sample_i.value = 0xFF & (samples[i] >> 8)
        await RisingEdge(tb.dut.clk)
        tb.dut.sample_i.value = 0xFF & samples[i]
        await FallingEdge(tb.dut.clk)
        if i >= clks_before_valid_output:
            print("-----")
            print(f"i - clks_before_valid_output = {i - clks_before_valid_output}, fir_out = {tb.dut.fir_out.value.signed_integer}")
            if i - clks_before_valid_output + tb.efc.HW_NUM_TAPS < len(samples):
                fir_out = tb.dut._id("fir_out",extended=False).value
                expected_samples = samples[(i - clks_before_valid_output):(i - clks_before_valid_output + tb.efc.HW_NUM_TAPS)]
                expected_fir_out = fir_check(coeffs, expected_samples)
                assert fir_out.signed_integer == expected_fir_out
            if replacement_type == "more_pos":
                assert tb.dut.fir_out_lte_thresh_lo.value.integer == 0
            elif replacement_type == "more_neg":
                assert tb.dut.fir_out_gte_thresh_hi.value.integer == 0
            if (i - clks_before_valid_output) in expected_ss_replacement_idxs:
                print("REPLACEMENT----------------------------")
                print(f"fir_out = {tb.dut.fir_out.value.signed_integer}")
                expected_samples = samples[(i - clks_before_valid_output):(i - clks_before_valid_output + tb.efc.HW_NUM_TAPS)]
                expected_pdw = cocotb.binary.BinaryValue(0, n_bits=(tb.dut.SS_BUFF_SZ.value + 16), bigEndian=False)
                expected_pdw[tb.dut.ABS_TIME_WIDTH.value:0] = tb.dut.abs_time_cntr.value.integer
                for j in range(tb.efc.HW_NUM_TAPS):
                    expected_pdw[tb.dut.ABS_TIME_WIDTH.value + (j + 1) * tb.efc.HW_SAMPLE_WIDTH - 1:tb.dut.ABS_TIME_WIDTH.value + j * tb.efc.HW_SAMPLE_WIDTH] = 0xFFFF & expected_samples[tb.efc.HW_NUM_TAPS - j - 1]
                expected_pdw[tb.dut.SS_BUFF_SZ.value + 16 - 1: tb.dut.SS_BUFF_SZ.value] = calc_pdw_crc16(expected_pdw[tb.dut.SS_BUFF_SZ.value - 1:0])
                if replacement_type == "more_pos":
                    assert tb.dut.fir_out_gte_thresh_hi.value.integer == 1
                elif replacement_type == "more_neg":
                    assert tb.dut.fir_out_lte_thresh_lo.value.integer == 1
                elif replacement_type == "large_mag":
                    assert (tb.dut.fir_out_gte_thresh_hi.value.integer == 1) or (tb.dut.fir_out_lte_thresh_lo.value.integer == 1)
            else:
                if replacement_type == "more_pos":
                    if (i - clks_before_valid_output) in expected_ss_non_replacement_idxs:
                        assert tb.dut.fir_out_gte_thresh_hi.value.integer == 1
                    else:
                        assert tb.dut.fir_out_gte_thresh_hi.value.integer == 0
                elif replacement_type == "more_neg":
                    if (i - clks_before_valid_output) in expected_ss_non_replacement_idxs:
                        assert tb.dut.fir_out_lte_thresh_lo.value.integer == 1
                    else:
                        assert tb.dut.fir_out_lte_thresh_lo.value.integer == 0
            if (i - clks_before_valid_output) > (expected_ss_replacement_idxs[0] + 2 + ss_delay) and (i - clks_before_valid_output) <= (expected_ss_replacement_idxs[0] + tb.dut.SS_BUFF_SZ.value + 16 + 2 + ss_delay):
                assert tb.dut.pdw_frame.value.integer == 1
                assert tb.dut.pdw_data.value == expected_pdw[i - clks_before_valid_output - (expected_ss_replacement_idxs[0] + 3 + ss_delay)].value
            else:
                assert tb.dut.pdw_frame.value == 0

    await Timer(10, "us")

@cocotb.test(skip=(SKIP_ALL & SKIP_SINGLE_SNAPSHOT_REPLACEMENT_MORE_POS))  # denotes a test
async def single_snapshot_replacement_more_pos(dut):
    tb = TB(dut)
    thresh_lo = -1
    thresh_hi = 10 * tb.efc.HW_NUM_TAPS
    ss_delay = 8
    replacement_type = "more_pos"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    # ramp from 0 to all 10s for num_taps then back to 0
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[10 for i in range(tb.efc.HW_NUM_TAPS)], \
               9, 9, 13,
               *[i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(tb.efc.HW_NUM_TAPS * tb.efc.HW_SAMPLE_WIDTH + tb.dut.ABS_TIME_WIDTH.value + 16 + ss_delay)]]
    expected_ss_replacement_idxs = [3 * tb.efc.HW_NUM_TAPS, 3 * tb.efc.HW_NUM_TAPS + 3]
    expected_ss_non_replacement_idxs = [3 * tb.efc.HW_NUM_TAPS + 4]

    await test_snapshot_replacement(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_replacement_idxs,
                                expected_ss_non_replacement_idxs)

@cocotb.test(skip=(SKIP_ALL & SKIP_SINGLE_SNAPSHOT_REPLACEMENT_MORE_NEG))  # denotes a test
async def single_snapshot_replacement_more_neg(dut):
    tb = TB(dut)
    thresh_lo = -10 * tb.efc.HW_NUM_TAPS
    thresh_hi = 1
    ss_delay = 7
    replacement_type = "more_neg"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    # ramp from 0 to all 10s for num_taps then back to 0
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[-10 for i in range(tb.efc.HW_NUM_TAPS)], \
               -9, -9, -13,
               *[-i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(tb.efc.HW_NUM_TAPS * tb.efc.HW_SAMPLE_WIDTH + tb.dut.ABS_TIME_WIDTH.value + 16 + ss_delay)]]
    expected_ss_replacement_idxs = [3 * tb.efc.HW_NUM_TAPS, 3 * tb.efc.HW_NUM_TAPS + 3]
    expected_ss_non_replacement_idxs = [3 * tb.efc.HW_NUM_TAPS + 4]

    await test_snapshot_replacement(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_replacement_idxs,
                                expected_ss_non_replacement_idxs)

@cocotb.test(skip=(SKIP_ALL & SKIP_SINGLE_SNAPSHOT_REPLACEMENT_LARGE_MAG))  # denotes a test
async def single_snapshot_replacement_large_mag(dut):
    tb = TB(dut)
    thresh_lo = -13
    thresh_hi = 13
    ss_delay = 14
    replacement_type = "large_mag"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[1 for i in range(tb.efc.HW_NUM_TAPS)], \
               4, -8, -10, -5, 3,
               *[1 for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(tb.efc.HW_NUM_TAPS * tb.efc.HW_SAMPLE_WIDTH + tb.dut.ABS_TIME_WIDTH.value + 16 + ss_delay)]]
    expected_ss_replacement_idxs = [2 * tb.efc.HW_NUM_TAPS + 1, 2 * tb.efc.HW_NUM_TAPS + 11]
    expected_ss_non_replacement_idxs = [2 * tb.efc.HW_NUM_TAPS + 4]

    await test_snapshot_replacement(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_replacement_idxs,
                                expected_ss_non_replacement_idxs)

async def test_multi_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_tx_idxs,
                                expected_tx_start_idxs):
    if len(expected_ss_tx_idxs) != len(expected_tx_start_idxs):
        raise Error("len(expected_ss_tx_idxs) must equal len(expected_tx_start_idxs)")
    tb.dut.sample_i.value = 0x00
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    i2c_addr = tb.dut.I2C_ADDR.value
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(100, "us")

    await tb.efc.set_threshold_lo(thresh_lo)
    await tb.efc.set_threshold_hi(thresh_hi)
    await tb.efc.set_ss_delay(ss_delay)
    await tb.efc.set_snapshot_replacement(replacement_type)

    await tb.efc.write_fir_coefficients([0xFFFF & coeff for coeff in coeffs])

    clks_before_valid_output = tb.efc.HW_NUM_TAPS + 1 + math.ceil(math.log2(tb.efc.HW_NUM_TAPS))
    expected_pdw = [cocotb.binary.BinaryValue(0, n_bits=(tb.dut.SS_BUFF_SZ.value + 16), bigEndian=False) for i in range(len(expected_ss_tx_idxs))]
    k = 0
    for i in range(len(samples)):
        tb.dut.sample_i.value = 0xFF & (samples[i] >> 8)
        await RisingEdge(tb.dut.clk)
        tb.dut.sample_i.value = 0xFF & samples[i]
        await FallingEdge(tb.dut.clk)
        if i >= clks_before_valid_output:
            print("-----")
            #print(f"i - clks_before_valid_output = {i - clks_before_valid_output}, fir_out = {tb.dut.fir_out.value.signed_integer}")
            if i - clks_before_valid_output + tb.efc.HW_NUM_TAPS < len(samples):
                fir_out = tb.dut._id("fir_out",extended=False).value
                expected_samples = samples[(i - clks_before_valid_output):(i - clks_before_valid_output + tb.efc.HW_NUM_TAPS)]
                expected_fir_out = fir_check(coeffs, expected_samples)
                assert fir_out.signed_integer == expected_fir_out
            if (i - clks_before_valid_output) in expected_ss_tx_idxs:
                ss_k = 0
                for kk in range(len(expected_ss_tx_idxs)):
                    if expected_ss_tx_idxs[kk] == i - clks_before_valid_output:
                        ss_k = kk
                print("REPLACEMENT----------------------------")
                print(f"fir_out = {tb.dut.fir_out.value.signed_integer}")
                expected_samples = samples[(i - clks_before_valid_output):(i - clks_before_valid_output + tb.efc.HW_NUM_TAPS)]
                expected_pdw[ss_k] = cocotb.binary.BinaryValue(0, n_bits=(tb.dut.SS_BUFF_SZ.value + 16), bigEndian=False)
                expected_pdw[ss_k][tb.dut.ABS_TIME_WIDTH.value:0] = tb.dut.abs_time_cntr.value.integer
                for j in range(tb.efc.HW_NUM_TAPS):
                    expected_pdw[ss_k][tb.dut.ABS_TIME_WIDTH.value + (j + 1) * tb.efc.HW_SAMPLE_WIDTH - 1:tb.dut.ABS_TIME_WIDTH.value + j * tb.efc.HW_SAMPLE_WIDTH] = 0xFFFF & expected_samples[tb.efc.HW_NUM_TAPS - j - 1]
                expected_pdw[ss_k][tb.dut.SS_BUFF_SZ.value + 16 - 1: tb.dut.SS_BUFF_SZ.value] = calc_pdw_crc16(expected_pdw[ss_k][tb.dut.SS_BUFF_SZ.value - 1:0])
                print(f"expected_pdw = {expected_pdw}")
                if replacement_type == "more_pos":
                    assert tb.dut.fir_out_gte_thresh_hi.value.integer == 1
                elif replacement_type == "more_neg":
                    assert tb.dut.fir_out_lte_thresh_lo.value.integer == 1
                elif replacement_type == "large_mag":
                    assert (tb.dut.fir_out_gte_thresh_hi.value.integer == 1) or (tb.dut.fir_out_lte_thresh_lo.value.integer == 1)
            if (i - clks_before_valid_output) > (expected_tx_start_idxs[k]) and (i - clks_before_valid_output) <= (expected_tx_start_idxs[k] + tb.dut.SS_BUFF_SZ.value + 16):
                print(f"i - clks = {i - clks_before_valid_output}, expected_pdw[idx] = {expected_pdw[k][i - clks_before_valid_output - (expected_tx_start_idxs[k] + 1)].value}, pdw_data = {tb.dut.pdw_data.value}")
                assert tb.dut.pdw_frame.value.integer == 1
                assert tb.dut.pdw_data.value == expected_pdw[k][i - clks_before_valid_output - (expected_tx_start_idxs[k] + 1)].value
            else:
                assert tb.dut.pdw_frame.value == 0
            if (i - clks_before_valid_output) == (expected_tx_start_idxs[k] + tb.dut.SS_BUFF_SZ.value + 16):
                print("Transmission finished.")
                if k == (len(expected_ss_tx_idxs) - 1):
                    break
                else:
                    k = k + 1
            
    await Timer(10, "us")
    
@cocotb.test(skip=(SKIP_ALL & SKIP_DOUBLE_SNAPSHOT_MORE_POS))  # denotes a test
async def double_snapshot_more_pos(dut):
    tb = TB(dut)
    thresh_lo = -1
    thresh_hi = 10 * tb.efc.HW_NUM_TAPS
    ss_delay = 3
    replacement_type = "more_pos"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[10 for i in range(tb.efc.HW_NUM_TAPS)], \
               9, 10, 10, 10, 11, 11,
               *[i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(3 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(2 * (tb.dut.SS_BUFF_SZ.value + 16) + ss_delay)]]
    expected_ss_tx_idxs = [3 * tb.efc.HW_NUM_TAPS, 3 * tb.efc.HW_NUM_TAPS + 6]
    expected_tx_start_idxs = [3 * tb.efc.HW_NUM_TAPS + 2 + ss_delay, 3 * tb.efc.HW_NUM_TAPS + 2 + ss_delay + tb.dut.SS_BUFF_SZ.value + 16 + 2]

    await test_multi_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_tx_idxs,
                                expected_tx_start_idxs)

@cocotb.test(skip=(SKIP_ALL & SKIP_DOUBLE_SNAPSHOT_MORE_NEG))  # denotes a test
async def double_snapshot_more_neg(dut):
    tb = TB(dut)
    thresh_lo = -10 * tb.efc.HW_NUM_TAPS
    thresh_hi = 1
    ss_delay = 3
    replacement_type = "more_neg"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[-i for i in range(tb.efc.HW_NUM_TAPS)], \
               *[-10 for i in range(tb.efc.HW_NUM_TAPS)], \
               -9, -10, -10, -10, -11, -11,
               *[-i for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(3 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(2 * (tb.dut.SS_BUFF_SZ.value + 16) + ss_delay)]]
    expected_ss_tx_idxs = [3 * tb.efc.HW_NUM_TAPS, 3 * tb.efc.HW_NUM_TAPS + 6]
    expected_tx_start_idxs = [3 * tb.efc.HW_NUM_TAPS + 2 + ss_delay, 3 * tb.efc.HW_NUM_TAPS + 2 + ss_delay + tb.dut.SS_BUFF_SZ.value + 16 + 2]

    await test_multi_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_tx_idxs,
                                expected_tx_start_idxs)

@cocotb.test(skip=(SKIP_ALL & SKIP_DOUBLE_SNAPSHOT_LARGE_MAG))  # denotes a test
async def double_snapshot_large_mag(dut):
    tb = TB(dut)
    thresh_lo = -10
    thresh_hi = 10
    ss_delay = 2
    replacement_type = "large_mag"
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [*[0 for i in range(2 * tb.efc.HW_NUM_TAPS)], \
               *[1 for i in range(tb.efc.HW_NUM_TAPS)], \
               4, -8, -10, -5, 3,
               *[1 for i in range(tb.efc.HW_NUM_TAPS - 1, -1, -1)], \
               *[0 for i in range(3 * tb.efc.HW_NUM_TAPS)], \
               *[0 for i in range(2 * (tb.dut.SS_BUFF_SZ.value + 16) + ss_delay)]]
    expected_ss_tx_idxs = [21, 31]
    # idx 20 is first fir_out over threshold
    expected_tx_start_idxs = [20 + 2 + ss_delay, 20 + 2 + ss_delay + tb.dut.SS_BUFF_SZ.value + 16 + 2]

    await test_multi_snapshot(tb, coeffs, samples, ss_delay, replacement_type,
                                thresh_lo, thresh_hi, expected_ss_tx_idxs,
                                expected_tx_start_idxs)
