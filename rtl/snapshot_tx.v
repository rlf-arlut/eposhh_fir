// SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
// SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
//
// SPDX-License-Identifier: BSD-3-Clause

`default_nettype none
module snapshot_tx (clk, rst, ss_tx_start, ss_buff, idle, pdw_data, pdw_frame);
    parameter SAMPLE_WIDTH = 16;
    parameter COEF_WIDTH = 16;
    parameter NUM_TAPS = 10;
    parameter ABS_TIME_WIDTH = 32;
    parameter RF_SS_DELAY_WIDTH = $clog2(NUM_TAPS);
    parameter SS_BUFF_SZ = (SAMPLE_WIDTH * NUM_TAPS) + ABS_TIME_WIDTH;
    input clk;
    input rst;
    input ss_tx_start;
    input [    SS_BUFF_SZ-1:0] ss_buff;
    output      idle;
    output reg        pdw_data;
    output reg        pdw_frame;

    // ---- Snapshot TX--------------------------
    // SS TX FSM
    localparam SS_TX_IDLE = 0;
    localparam SS_TX_TXING = 1;
    localparam SS_TX_CRC = 2;
    localparam SS_TX_FIN = 3;

    reg [1:0] ss_tx_cs, ss_tx_ns;

    reg  [    SS_BUFF_SZ-1:0] ss_txbuff;
    localparam SS_TX_LEN = (SAMPLE_WIDTH * NUM_TAPS) + ABS_TIME_WIDTH + 16;
    localparam SS_TX_LEN_LOG2 = $clog2(SS_TX_LEN);
    reg     [SS_TX_LEN_LOG2-1:0] ss_tx_bit_cnt;
    reg     [              15:0] crc16_reg;
    wire    [              15:0] crc16_out;
    wire [7:0] ss_tx_crc16_in = ss_txbuff[7:0];

    assign idle = (ss_tx_cs == SS_TX_IDLE);


    always @* begin
        case (ss_tx_cs)
            SS_TX_IDLE: begin
                if (ss_tx_start) begin
                    ss_tx_ns = SS_TX_TXING;
                end else begin
                    ss_tx_ns = SS_TX_IDLE;
                end
            end
            SS_TX_TXING: begin
                if (ss_tx_bit_cnt < SS_TX_LEN - 16 - 1) begin
                    ss_tx_ns = SS_TX_TXING;
                end else begin
                    ss_tx_ns = SS_TX_CRC;
                end
            end
            SS_TX_CRC: begin
                if (ss_tx_bit_cnt == SS_TX_LEN - 1) begin
                    ss_tx_ns = SS_TX_FIN;
                end else begin
                    ss_tx_ns = SS_TX_CRC;
                end
            end
            SS_TX_FIN: begin
                ss_tx_ns = SS_TX_IDLE;
            end
        endcase

    end

    always @(posedge clk) begin
        if (rst == 1'b1) begin
            ss_tx_cs        <= SS_TX_IDLE;

        end else begin
            ss_tx_cs <= ss_tx_ns;

            case (ss_tx_cs)
                SS_TX_IDLE: begin
                    ss_tx_bit_cnt   <= 0;
                    crc16_reg           <= 0;

                end
                SS_TX_TXING: begin
                    ss_tx_bit_cnt <= ss_tx_bit_cnt + 1;

                    // send data through CRC16
                    if ((ss_tx_bit_cnt[3:0] == 0) || (ss_tx_bit_cnt[3:0] == 8)) begin
                        crc16_reg <= crc16_out;
                    end

                end
                SS_TX_CRC: begin
                    crc16_reg         <= {1'b0, crc16_reg[15:1]};
                    ss_tx_bit_cnt <= ss_tx_bit_cnt + 1;
                end
                SS_TX_FIN: begin
                end
            endcase
        end
    end
    always @(posedge clk) begin
        if (rst == 1'b1) begin
            pdw_frame       <= 0;

        end else begin

            case (ss_tx_cs)
                SS_TX_IDLE: begin
                    ss_txbuff       <= ss_buff;
                end
                SS_TX_TXING: begin
                    ss_txbuff     <= {1'b0, ss_txbuff[SS_BUFF_SZ-1:1]};
                    pdw_frame     <= 1'b1;
                    pdw_data      <= ss_txbuff[0];
                end
                SS_TX_CRC: begin
                    pdw_frame     <= 1'b1;
                    pdw_data      <= crc16_reg[0];
                end
                SS_TX_FIN: begin
                    pdw_frame       <= 1'b0;
                end
            endcase
        end
    end
    crc16 crc16_ (
        .crcIn (crc16_reg),
        .data  (ss_tx_crc16_in),
        .crcOut(crc16_out)
    );
endmodule
`default_nettype wire
