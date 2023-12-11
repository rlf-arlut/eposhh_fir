// SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
// SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
//
// SPDX-License-Identifier: BSD-3-Clause

`default_nettype none
module mult_reg #(
    parameter IN0_WIDTH = 16,
    parameter IN1_WIDTH = 16
) (
    input wire clk_i,
    input wire signed [IN0_WIDTH-1:0] in0_i,
    input wire signed [IN0_WIDTH-1:0] in1_i,
    output reg signed [(IN0_WIDTH+IN1_WIDTH)-1:0] product_o
);
    always @(posedge clk_i) product_o <= in0_i * in1_i;
endmodule
`default_nettype wire
