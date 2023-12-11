// SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
// SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
//
// SPDX-License-Identifier: BSD-3-Clause

`default_nettype none
module fir_top_i2c_tmr #(
    parameter [6:0] I2C_ADDR = 7'h2E,
    parameter SAMPLE_WIDTH = 16,
    parameter COEF_WIDTH = 16,
    parameter NUM_TAPS = 10
) (
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif
    input  wire       clk,
    input  wire       rst,
    // ------ I2C C2 ----------------------------
    input  wire       i2c_scl_i,
    output wire       i2c_scl_o,
    output wire       i2c_scl_t,
    input  wire       i2c_sda_i,
    output wire       i2c_sda_o,
    output wire       i2c_sda_t,
    // ------ SPI For Samples Input -------------
    input  wire [7:0] sample_i,
    // ------ Pulse Descriptor Word Output ------
    output reg        pdw_data,
    output reg        pdw_frame
);

    // ------------------------------------------
    // ---- Declarations ------------------------
    // ------------------------------------------
    localparam [31:0] ZEROS = 0;

    // ------ Wishbone --------------------------
    localparam WB_DATA_WIDTH = 16;
    localparam WB_ADDR_WIDTH = 16;
    localparam WB_SELECT_WIDTH = 2;

    // verilog_format: off // verible insists on left alignment
    reg  [WB_DATA_WIDTH-1:0] wb_dat_i;
    reg  [WB_DATA_WIDTH-1:0] wb_dat_i_mux;
    wire [WB_DATA_WIDTH-1:0] wb_dat_o;
    wire [WB_ADDR_WIDTH-1:0] wb_adr_o;
    wire [              1:0] wb_sel;
    wire                     wb_we;
    wire                     wb_stb;
    wire                     wb_cyc;
    reg                      wb_ack;
    wire                     busy;
    wire                     bus_addressed;
    wire                     bus_active;
    // verilog_format: on


    // ------ Address Map -----------------------
    localparam RMH = 7;
    localparam RML = 5;
    localparam RAH = 4;
    localparam RAL = 1;
    localparam RM_COEF = 0;
    localparam RM_THRESH = 1;
    localparam RM_CTRL = 2;

    // ------ Registers -------------------------
    localparam RA_CTRL = 0;
    localparam RB_CTRL_EN = 0;
    localparam RA_SS_DELAY = 1;
    localparam RF_SS_DELAY = 0;
    localparam RF_SS_DELAY_WIDTH = $clog2(NUM_TAPS);
    localparam RB_REPLACE_LARGE_MAG = 1;
    localparam RB_REPLACE_MORE_POS = 2;
    localparam RB_REPLACE_MORE_NEG = 3;
    localparam RB_REPLACE_NEVER = 11;

    reg        [           3:0] control;
    reg        [RF_SS_DELAY_WIDTH-1:0] ss_delay;
    reg signed [          15:0] thresholds          [0:5];
    reg signed [          15:0] thresholds_readback;
    reg        [COEF_WIDTH-1:0] coeff               [9:0];
    reg        [COEF_WIDTH-1:0] coeff_readback;

    // ---- FIR Filter --------------------------
    localparam PRODUCT_WIDTH = COEF_WIDTH + SAMPLE_WIDTH;
    reg signed  [                   PRODUCT_WIDTH-1:0] products     [NUM_TAPS-1:0];
    reg         [        (PRODUCT_WIDTH*NUM_TAPS)-1:0] products_1d;
    wire signed [(PRODUCT_WIDTH+$clog2(NUM_TAPS))-1:0] fir_out;
    wire signed [(PRODUCT_WIDTH+$clog2(NUM_TAPS))-1:0] abs_fir_out;
    reg signed  [(PRODUCT_WIDTH+$clog2(NUM_TAPS))-1:0] ss_fir_out;
    wire signed [                                35:0] threshold_hi;
    wire signed [                                35:0] threshold_lo;

    // ---- Sample Stream and Snapshotting (ss)---
    localparam ABS_TIME_WIDTH = 32;
    // 1 for multiplier reg, clog2 for number of adder_tree stages
    localparam SAMPLE_BUFFER_SZ = NUM_TAPS + 1 + $clog2(NUM_TAPS);
    // 2 for abs time counter, 1 for CRC16
    localparam SS_BUFF_SZ = (SAMPLE_WIDTH * NUM_TAPS) + ABS_TIME_WIDTH;


    reg  [  SAMPLE_WIDTH-1:0] samples               [SAMPLE_BUFFER_SZ-1:0];
    reg  [  SAMPLE_WIDTH/2-1:0] sample_in_pos;
    reg  [  SAMPLE_WIDTH/2-1:0] sample_in_neg;
    // Snapshot buffer would not adjust properly if SAMPLE_WIDTH is not 16-bits
    reg  [    SS_BUFF_SZ-1:0] ss_buff;
    reg  [ABS_TIME_WIDTH-1:0] abs_time_cntr;
    wire                      fir_out_gte_thresh_hi;
    wire                      fir_out_lte_thresh_lo;
    wire                      fir_out_gt_ss_fir_out;
    wire                      fir_out_lt_ss_fir_out;
    wire                      ss_load_en_hi;
    wire                      ss_load_en_lo;
    wire                      ss_load_en_mag;
    wire                      ss_load_en;
    reg                       ss_buff_valid;
    reg  [RF_SS_DELAY_WIDTH-1:0] ss_delay_cntr;

    // ---- Snapshot TX--------------------------
    wire ss_tx_idle;
    wire ss_tx_start;

    // ------------------------------------------
    // ---- Sample Stream and Snapshoting -------
    // ------------------------------------------
    //-------------------------------------------
    // Reg Map
    //
    // -16-bit address, byte addressable
    // -Must only use aligned addresses
    //
    // 0x0000 - 0x000A - Coefficents
    // 0x0100 - 0x01FF - Control and Status

    integer                      i;
    always @(posedge clk) begin
        sample_in_pos <= sample_i;

        samples[0] <= {sample_in_pos, sample_in_neg};

        for (i = SAMPLE_BUFFER_SZ - 1; i > 0; i = i - 1) samples[i] <= samples[i-1];
    end
    always @(negedge clk) begin
        sample_in_neg <= sample_i;
    end
    // ------------------------------------------


    // ------------------------------------------
    // ---- Wishbone and Registers --------------
    // ------------------------------------------
    always @(posedge clk) wb_ack <= wb_stb & ~wb_ack;

    integer tap;
    integer th;
    always @(posedge clk) begin
        if (rst == 1'b1) begin
            control[RB_CTRL_EN] <= 0;
            abs_time_cntr <= 0;

        end else begin
            abs_time_cntr <= abs_time_cntr + 1;

            if ((wb_adr_o[WB_ADDR_WIDTH-1:RMH+1] == 0)
                && (wb_adr_o[RMH:RML] == RM_CTRL)
                && (wb_adr_o[RAH:RAL] == RA_CTRL)
                && wb_we && wb_stb) begin
                control <= wb_dat_o[3:0];
            end
        end
    end
    always @(posedge clk) begin
        if ((wb_adr_o[WB_ADDR_WIDTH-1:RMH+1] == 0)
            && (wb_adr_o[RMH:RML] == RM_CTRL)
            && (wb_adr_o[RAH:RAL] == RA_SS_DELAY)
            && wb_we && wb_stb) begin
            ss_delay <= wb_dat_o[RF_SS_DELAY+:RF_SS_DELAY_WIDTH];
        end

        if ((wb_adr_o[WB_ADDR_WIDTH-1:RMH+1] == 0)
            && (wb_adr_o[RMH:RML] == RM_THRESH)
            && wb_we && wb_stb) begin
                case(wb_adr_o[RAH:RAL])
                    4'b0000: thresholds[3'd0] <= wb_dat_o;
                    4'b0001: thresholds[3'd1] <= wb_dat_o;
                    4'b0010: thresholds[3'd2] <= wb_dat_o;
                    4'b0011: thresholds[3'd3] <= wb_dat_o;
                    4'b0100: thresholds[3'd4] <= wb_dat_o;
                    4'b0101: thresholds[3'd5] <= wb_dat_o;
                    default: thresholds[3'd0] <= wb_dat_o;
                endcase
        end
        if ((wb_adr_o[WB_ADDR_WIDTH-1:RMH+1] == 0)
            && (wb_adr_o[RMH:RML] == RM_COEF)
            && wb_we && wb_stb) begin
            case(wb_adr_o[RAH:RAL])
                4'b0000: coeff[4'd0] <= wb_dat_o;
                4'b0001: coeff[4'd1] <= wb_dat_o;
                4'b0010: coeff[4'd2] <= wb_dat_o;
                4'b0011: coeff[4'd3] <= wb_dat_o;
                4'b0100: coeff[4'd4] <= wb_dat_o;
                4'b0101: coeff[4'd5] <= wb_dat_o;
                4'b0110: coeff[4'd6] <= wb_dat_o;
                4'b0111: coeff[4'd7] <= wb_dat_o;
                4'b1000: coeff[4'd8] <= wb_dat_o;
                4'b1001: coeff[4'd9] <= wb_dat_o;
                default: coeff[4'd0] <= wb_dat_o;
            endcase
         end
    end
    always @(posedge clk) begin
        wb_dat_i <= wb_dat_i_mux;
    end

    integer tap1;
    integer th1;
    always @* begin

        for (tap1 = 0; tap1 < NUM_TAPS; tap1 = tap1 + 1) begin
            if (wb_adr_o[RAH:RAL] == tap1[3:0]) begin
                coeff_readback = coeff[tap1];
            end
        end

        for (th1 = 0; th1 < 6; th1 = th1 + 1) begin
            if (wb_adr_o[RAH:RAL] == th1[3:0]) begin
                thresholds_readback = thresholds[th1];
            end
        end

        case (wb_adr_o[RMH:RML])
            3'b000:  wb_dat_i_mux = coeff_readback;
            3'b001:  wb_dat_i_mux = thresholds_readback;
            3'b010:  wb_dat_i_mux = (wb_adr_o[RAH:RAL] == RA_CTRL) ? {12'b0, control[3:0]} : {ZEROS[0+:WB_DATA_WIDTH-RF_SS_DELAY_WIDTH] , ss_delay};
            default: wb_dat_i_mux = coeff_readback;
        endcase
    end


    // ------------------------------------------
    // ---- FIR Filter --------------------------
    // ------------------------------------------
    generate
        genvar gentap;
        for (gentap = 0; gentap < NUM_TAPS; gentap = gentap + 1) begin : gen_fir

            mult_reg #(
                .IN0_WIDTH(SAMPLE_WIDTH),
                .IN1_WIDTH(COEF_WIDTH)
            ) mult_reg (
                .clk_i    (clk),
                .in0_i    (samples[NUM_TAPS-gentap-1]),
                .in1_i    (coeff[gentap]),
                .product_o(products[gentap])
            );
        end
    endgenerate


    // Combine products into single wire to pass to adder_tree module
    integer b;
    always @* begin
        for (b = 0; b < 10; b = b + 1) begin
            products_1d[(PRODUCT_WIDTH*b)+:PRODUCT_WIDTH] = products[b];
        end
    end

    /*
    adder_tree #(
        .N         (NUM_TAPS),
        .DATA_WIDTH(COEF_WIDTH + SAMPLE_WIDTH)
    ) adder_tree (
        .clock (clk),
        .data  (products_1d),
        .result(fir_out)
    );
    */
    adder_tree10 #(
        .DATA_WIDTH(PRODUCT_WIDTH)
    ) adder_tree10 (
        .clk(clk),
        .in0(products[0]),
        .in1(products[1]),
        .in2(products[2]),
        .in3(products[3]),
        .in4(products[4]),
        .in5(products[5]),
        .in6(products[6]),
        .in7(products[7]),
        .in8(products[8]),
        .in9(products[9]),
        .sum(fir_out)
    );


    // ------------------------------------------
    // ---- Snapshot & Replacement --------------
    // ------------------------------------------
    assign threshold_hi = {thresholds[2][3:0], thresholds[1], thresholds[0]};
    assign threshold_lo = {thresholds[5][3:0], thresholds[4], thresholds[3]};
    assign fir_out_gte_thresh_hi = (fir_out >= threshold_hi);
    assign fir_out_lte_thresh_lo = (fir_out <= threshold_lo);
    assign fir_out_gt_ss_fir_out = (fir_out > ss_fir_out);
    assign fir_out_lt_ss_fir_out = (fir_out < ss_fir_out);
    assign abs_fir_out = fir_out[(PRODUCT_WIDTH+$clog2(NUM_TAPS))-1] ? -fir_out : fir_out;
    assign ss_load_en_hi = fir_out_gte_thresh_hi & (~ss_buff_valid | (fir_out_gt_ss_fir_out & control[RB_REPLACE_MORE_POS]));
    assign ss_load_en_lo = fir_out_lte_thresh_lo & (~ss_buff_valid | (fir_out_lt_ss_fir_out & control[RB_REPLACE_MORE_NEG]));
    assign ss_load_en_mag = (fir_out_gte_thresh_hi | fir_out_lte_thresh_lo) & (abs_fir_out > ss_fir_out) & control[RB_REPLACE_LARGE_MAG];
    assign ss_load_en = ss_load_en_hi | ss_load_en_lo | ss_load_en_mag;

    integer o;
    always @(posedge clk) begin
        if (rst == 1'b1) begin
            ss_fir_out    <= 0;

        end else begin
            if (ss_load_en && (~ss_buff_valid || (ss_tx_idle && (ss_delay_cntr == 0)))) begin
                ss_delay_cntr <= ss_delay;
            end else if (ss_delay_cntr != 0) begin
                ss_delay_cntr <= ss_delay_cntr - 1;
            end

            if (ss_load_en) begin
                if (control[RB_REPLACE_LARGE_MAG]) ss_fir_out <= abs_fir_out;
                else ss_fir_out <= fir_out;

                ss_buff[31:0] <= abs_time_cntr;

                for (o = 0; o < NUM_TAPS; o = o + 1) begin
                    ss_buff[(ABS_TIME_WIDTH+o*SAMPLE_WIDTH)+:SAMPLE_WIDTH] <= samples[o+1+$clog2(NUM_TAPS)];
                end
            end
        end
    end

    always @(posedge clk) begin
        if (rst == 1'b1) begin
            ss_buff_valid <= 1'b0;
        end else begin
            if (control[RB_CTRL_EN]) begin
                ss_buff_valid <= ss_load_en | (ss_buff_valid & (~ss_tx_idle | (ss_delay_cntr != 0)));
            end
        end
    end

    // ------------------------------------------
    // ---- Snapshot Transmission ---------------
    // ------------------------------------------

    assign ss_tx_start = ss_buff_valid & (ss_delay_cntr == 0);

    snapshot_txTMR #(
        .SAMPLE_WIDTH(SAMPLE_WIDTH),
        .COEF_WIDTH(COEF_WIDTH),
        .NUM_TAPS(NUM_TAPS),
        .ABS_TIME_WIDTH(ABS_TIME_WIDTH),
        .RF_SS_DELAY_WIDTH(RF_SS_DELAY_WIDTH),
        .SS_BUFF_SZ(SS_BUFF_SZ)
    ) snapshot_tx_0 (
        .clk(clk),
        .rst(rst),
        .ss_tx_start(ss_tx_start),
        .ss_buff(ss_buff),
        .idle(ss_tx_idle),
        .pdw_data(pdw_data),
        .pdw_frame(pdw_frame)
    );

    i2c_target_wbmTMR #(
        .WB_DATA_WIDTH  (WB_DATA_WIDTH),
        .WB_ADDR_WIDTH  (WB_ADDR_WIDTH),
        .WB_SELECT_WIDTH(WB_SELECT_WIDTH)
    ) i2c_target_wbmTMR_0 (
        .clk           (clk),
        .rst           (rst),
        .i2c_scl_i     (i2c_scl_i),
        .i2c_scl_o     (i2c_scl_o),
        .i2c_scl_t     (i2c_scl_t),
        .i2c_sda_i     (i2c_sda_i),
        .i2c_sda_o     (i2c_sda_o),
        .i2c_sda_t     (i2c_sda_t),
        .wb_adr_o      (wb_adr_o),
        .wb_dat_i      (wb_dat_i),
        .wb_dat_o      (wb_dat_o),
        .wb_we_o       (wb_we),
        .wb_sel_o      (wb_sel),
        .wb_stb_o      (wb_stb),
        .wb_ack_i      (wb_ack),
        //.wb_err_i(wb_err_i),
        .wb_err_i      (1'b0),
        .wb_cyc_o      (wb_cyc),
        .busy          (busy),
        .bus_addressed (bus_addressed),
        .bus_active    (bus_active),
        .enable        (1'b1),
        .device_address(I2C_ADDR)
    );

`ifdef COCOTB_SIM
    integer j, k, l, m;
    initial begin
        $dumpfile("fir_top_i2c_tmr.lxt2");
        $dumpvars;
        for (m = 0; m < NUM_TAPS; m = m + 1) $dumpvars(0, products[m]);
        for (j = 0; j < SAMPLE_BUFFER_SZ; j = j + 1) $dumpvars(0, samples[j]);
        for (k = 0; k < NUM_TAPS; k = k + 1) $dumpvars(0, coeff[k]);
        for (l = 0; l < 6; l = l + 1) $dumpvars(0, thresholds[l]);
    end
`endif  // COCOTB_SIM

endmodule
`default_nettype wire
