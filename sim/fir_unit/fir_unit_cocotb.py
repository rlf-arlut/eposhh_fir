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
        # fmt: on
        self.efc = EposhhFirCocotb(self.i2c_master)

CLK_PERIOD_NS = 10

SKIP_ALL = False

# COEF = 1
SKIP_SAMPLE_0_COEF_1          = False
SKIP_SAMPLE_0x11_COEF_1       = False
SKIP_SAMPLE_NEG_1_COEF_1      = False
SKIP_SAMPLE_RAND_POS_COEF_1   = False
SKIP_SAMPLE_RAND_NEG_COEF_1   = False
SKIP_SAMPLE_RAND_MIX_COEF_1   = False
# SAMPLE = 1
SKIP_SAMPLE_1_COEF_0          = False
SKIP_SAMPLE_1_COEF_0x11       = False
SKIP_SAMPLE_1_COEF_NEG_1      = False
SKIP_SAMPLE_1_COEF_RAND_POS   = False
SKIP_SAMPLE_1_COEF_RAND_NEG   = False
SKIP_SAMPLE_1_COEF_RAND_MIX   = False
# other
SKIP_SAMPLE_NEG_1_COEF_NEG_1  = False
SKIP_MAX_POS                  = False
SKIP_MAX_NEG                  = False
SKIP_SAMPLE_ORDER             = False
SKIP_BOTH_RAND_MIX            = False
SKIP_BOTH_RAND_MIX_VECTORS    = False


def fir_check(coeffs, samples):
    num_taps = len(coeffs)
    if len(coeffs) != len(samples):
        raise "fir_check expects len(coeffs) = len(samples)"
    fir_out = 0
    for i in range(num_taps):
        fir_out = fir_out + (coeffs[i])*(samples[i])
    return fir_out

async def helper_test_fir_output_value(tb, samples, coeffs):
    """
    Calculate expected FIR output
    
    Assumes 16-bit samples
    """
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(10, "us")
    
    for i in range(tb.efc.HW_NUM_TAPS):
        tb.dut._id("coeff[" + str(i) + "]", extended=False).value = coeffs[i]
    
    for i in range(tb.efc.HW_NUM_TAPS):
        tb.dut.sample_i.value = 0xFF & (samples[i] >> 8)
        await RisingEdge(tb.dut.clk)
        tb.dut.sample_i.value = 0xFF & samples[i]
        await FallingEdge(tb.dut.clk)
        print(f'i = {i}, fir_out = {tb.dut._id("fir_out",extended=False).value}') # to help find extra clk cycles
        
    print("samples shifted in") # to help find extra clk cycles
    # clock cycle delays for shifting in sample values
    for i in range(2):
        await RisingEdge(tb.dut.clk)
        print(f'i = {i+tb.efc.HW_NUM_TAPS}, fir_out = {tb.dut._id("fir_out",extended=False).value}') # to help find extra clk cycles
    
    for i in range(tb.efc.HW_NUM_TAPS):
        value = tb.dut._id("samples[" + str(i) + "]", extended=False).value
        print("samples[" + str(i) + "] is " + str(value.signed_integer))
        print("samples[" + str(i) + "] is " + str(value.binstr))
    
    # 1 clock for multiply, clog2(num_taps) clocks for adder tree
    for i in range(1+math.ceil(math.log2(tb.efc.HW_NUM_TAPS))):
        await RisingEdge(tb.dut.clk)
        print(f'i = {i+tb.efc.HW_NUM_TAPS+2}, fir_out = {tb.dut._id("fir_out",extended=False).value}') # to help find extra clk cycles
        
    fir_out = tb.dut._id("fir_out",extended=False).value
    expected_fir_out = fir_check(coeffs, samples)
    print(f'fir_out is {fir_out.signed_integer}')
    print(f'fir_out is {fir_out.binstr}')

    assert fir_out.signed_integer == expected_fir_out
    
    # Handy reminder how to access various value formats in cocotb
    #    value = tb.dut._id("samples[0]",extended=False).value
    #    print(value.binstr)
    #    print(value.integer)
    #    print(value.signed_integer)
    #    print(value.buff)
    #    print(value.n_bits)
    await Timer(10, "us")

# COEF = 1
@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_0_COEF_1))  # denotes a test
async def test_fir_out_sample_0_coef_1(dut):
    """
    FIR output test, constant samples of zero, unity coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [0x0000 for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_0x11_COEF_1))  # denotes a test
async def test_fir_out_sample_0x11_coef_1(dut):
    """
    FIR output test, small constant samples, unity coefficients

    This test streams in 10 samples, directly sets coefficients to 1,
    and checks FIR output value.
    """
    tb = TB(dut)
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [0x0011 for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_NEG_1_COEF_1))  # denotes a test
async def test_fir_out_sample_neg_1_coef_1(dut):
    """
    FIR output test, constant samples of -1, unity coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [-1 for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_RAND_POS_COEF_1))  # denotes a test
async def test_fir_out_sample_rand_pos_coef_1(dut):
    """
    FIR output test, random positive samples, unity coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    max_sample = 2**(tb.efc.HW_SAMPLE_WIDTH - 1) - 1
    samples = [random.randint(0, max_sample) for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_RAND_NEG_COEF_1))  # denotes a test
async def test_fir_out_sample_rand_neg_coef_1(dut):
    """
    FIR output test, random negative samples, unity coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    min_sample = -1*(2**(tb.efc.HW_SAMPLE_WIDTH - 1))
    samples = [random.randint(min_sample, 0) for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_RAND_MIX_COEF_1))  # denotes a test
async def test_fir_out_sample_rand_mix_coef_1(dut):
    """
    FIR output test, random positive and negative samples, unity coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    coeffs = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    max_sample = 2**(tb.efc.HW_SAMPLE_WIDTH - 1) - 1
    min_sample = -1*(2**(tb.efc.HW_SAMPLE_WIDTH - 1))
    samples = [random.randint(min_sample, max_sample) for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

# SAMPLE = 1
@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_1_COEF_0))  # denotes a test
async def test_fir_out_sample_1_coef_0(dut):
    """
    FIR output test, constant samples of 1, 0 coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    coeffs = [0 for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_1_COEF_0x11))  # denotes a test
async def test_fir_out_sample_1_coef_0x11(dut):
    """
    FIR output test, constant samples of 1, 0x11 coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    coeffs = [0x0011 for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_1_COEF_NEG_1))  # denotes a test
async def test_fir_out_sample_1_coef_neg_1(dut):
    """
    FIR output test, constant samples of 1, -1 coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    coeffs = [-1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_1_COEF_RAND_POS))  # denotes a test
async def test_fir_out_sample_1_coef_rand_pos(dut):
    """
    FIR output test, constant samples of 1, random positive coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    max_coef = 2**(tb.efc.HW_COEF_WIDTH - 1) - 1
    coeffs = [random.randint(0, max_coef) for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_1_COEF_RAND_NEG))  # denotes a test
async def test_fir_out_sample_1_coef_rand_neg(dut):
    """
    FIR output test, constant samples of 1, random negative coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    min_coef = -1*(2**(tb.efc.HW_COEF_WIDTH - 1))
    coeffs = [random.randint(min_coef, 0) for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_1_COEF_RAND_MIX))  # denotes a test
async def test_fir_out_sample_1_coef_rand_mix(dut):
    """
    FIR output test, constant samples of 1, random positive and negative coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    max_coef = 2**(tb.efc.HW_COEF_WIDTH - 1) - 1
    min_coef = -1*(2**(tb.efc.HW_COEF_WIDTH - 1))
    coeffs = [random.randint(min_coef, max_coef) for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [1 for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

# other
@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_NEG_1_COEF_NEG_1))  # denotes a test
async def test_fir_out_sample_neg_1_coef_neg_1(dut):
    """
    FIR output test, -1 samples, -1 coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    coeffs = [-1 for i in range(tb.efc.HW_NUM_TAPS)]
    samples = [-1 for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_MAX_POS))  # denotes a test
async def test_fir_out_max_pos(dut):
    """
    FIR output test, maximum positive samples and coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    max_coef = 2**(tb.efc.HW_COEF_WIDTH - 1) - 1
    coeffs = [max_coef for i in range(tb.efc.HW_NUM_TAPS)]
    max_sample = 2**(tb.efc.HW_SAMPLE_WIDTH - 1) - 1
    samples = [max_sample for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_MAX_NEG))  # denotes a test
async def test_fir_out_max_neg(dut):
    """
    FIR output test, maximum negative samples and coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    min_coef = -1*(2**(tb.efc.HW_COEF_WIDTH - 1))
    coeffs = [min_coef for i in range(tb.efc.HW_NUM_TAPS)]
    min_sample = -1*(2**(tb.efc.HW_SAMPLE_WIDTH - 1))
    samples = [min_sample for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)

@cocotb.test(skip=(SKIP_ALL | SKIP_SAMPLE_ORDER))  # denotes a test
async def test_fir_out_sample_order(dut):
    """
    FIR output test, check order of coeffs same as order of samples

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    for i in range(tb.efc.HW_NUM_TAPS):
        coeffs = [i for i in range(tb.efc.HW_NUM_TAPS)]
        samples = [i for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)


@cocotb.test(skip=(SKIP_ALL | SKIP_BOTH_RAND_MIX))  # denotes a test
async def test_fir_out_both_rand_mix(dut):
    """
    FIR output test, random positive and negative samples and coefficients

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    max_coef = 2**(tb.efc.HW_COEF_WIDTH - 1) - 1
    min_coef = -1*(2**(tb.efc.HW_COEF_WIDTH - 1))
    coeffs = [random.randint(min_coef, max_coef) for i in range(tb.efc.HW_NUM_TAPS)]
    max_sample = 2**(tb.efc.HW_SAMPLE_WIDTH - 1) - 1
    min_sample = -1*(2**(tb.efc.HW_SAMPLE_WIDTH - 1))
    samples = [random.randint(min_sample, max_sample) for i in range(tb.efc.HW_NUM_TAPS)]
    await helper_test_fir_output_value(tb, samples, coeffs)


@cocotb.test(skip=(SKIP_ALL | SKIP_BOTH_RAND_MIX_VECTORS))  # denotes a test
async def test_fir_out_both_rand_mix_vectors(dut):
    """
    FIR output test, random positive and negative samples and coefficients shifting though

    This test streams in 10 samples, directly sets coefficients,
    and checks FIR output value.
    """
    tb = TB(dut)
    await cocotb.start(Clock(tb.dut.clk, CLK_PERIOD_NS, units="ns").start())
    tb.dut.rst.value = 1
    await Timer(10, "us")
    tb.dut.rst.value = 0
    await Timer(10, "us")
    vector_multiple_of_num_taps = 500
    num_inputs = vector_multiple_of_num_taps * tb.efc.HW_NUM_TAPS
    max_coef = 2**(tb.efc.HW_COEF_WIDTH - 1) - 1
    min_coef = -1*(2**(tb.efc.HW_COEF_WIDTH - 1))
    coeffs = [random.randint(min_coef, max_coef) for i in range(tb.efc.HW_NUM_TAPS)]
    
    max_sample = 2**(tb.efc.HW_SAMPLE_WIDTH - 1) - 1
    min_sample = -1*(2**(tb.efc.HW_SAMPLE_WIDTH - 1))
    samples = [random.randint(min_sample, max_sample) for i in range(num_inputs)]

    # load in coefs over I2C
    await tb.efc.write_fir_coefficients([0xFFFF & coeff for coeff in coeffs])

    # num_taps clocks to load in samples
    # 1 clock for multiply, clog2(num_taps) clocks for adder tree
    clks_before_valid_output = tb.efc.HW_NUM_TAPS + 1 + math.ceil(math.log2(tb.efc.HW_NUM_TAPS))
    for i in range(num_inputs):
        tb.dut.sample_i.value = 0xFF & (samples[i] >> 8)
        await RisingEdge(tb.dut.clk)
        tb.dut.sample_i.value = 0xFF & samples[i]
        await FallingEdge(tb.dut.clk)
        print(f'i = {i}, fir_out = {tb.dut._id("fir_out",extended=False).value}') # to help find extra clk cycles
        if i >= clks_before_valid_output:
            fir_out = tb.dut._id("fir_out",extended=False).value
            expected_samples = samples[(i - clks_before_valid_output):(i - clks_before_valid_output + tb.efc.HW_NUM_TAPS)]
            expected_fir_out = fir_check(coeffs, expected_samples)
            assert fir_out.signed_integer == expected_fir_out
    await Timer(10, "us")
