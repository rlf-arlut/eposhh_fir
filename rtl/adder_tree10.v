// SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
// SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
//
// SPDX-License-Identifier: BSD-3-Clause

`default_nettype none
module adder_tree10 #(
    parameter DATA_WIDTH = 18
) (
    input clk,
    input wire signed [DATA_WIDTH-1:0] in0,
    input wire signed [DATA_WIDTH-1:0] in1,
    input wire signed [DATA_WIDTH-1:0] in2,
    input wire signed [DATA_WIDTH-1:0] in3,
    input wire signed [DATA_WIDTH-1:0] in4,
    input wire signed [DATA_WIDTH-1:0] in5,
    input wire signed [DATA_WIDTH-1:0] in6,
    input wire signed [DATA_WIDTH-1:0] in7,
    input wire signed [DATA_WIDTH-1:0] in8,
    input wire signed [DATA_WIDTH-1:0] in9,
    output reg signed [DATA_WIDTH+3:0] sum
);

    reg signed [DATA_WIDTH:0] isum00;
    reg signed [DATA_WIDTH:0] isum01;
    reg signed [DATA_WIDTH:0] isum02;
    reg signed [DATA_WIDTH:0] isum03;
    reg signed [DATA_WIDTH:0] isum04;

    reg signed [DATA_WIDTH+1:0] isum10;
    reg signed [DATA_WIDTH+1:0] isum11;
    reg signed [DATA_WIDTH+1:0] isum12;
    
    reg signed [DATA_WIDTH+2:0] isum20;
    reg signed [DATA_WIDTH+2:0] isum21;

    always @(posedge clk) begin
        isum00 <= in0 + in1;
        isum01 <= in2 + in3;
        isum02 <= in4 + in5;
        isum03 <= in6 + in7;
        isum04 <= in8 + in9;

        isum10 <= isum00 + isum01;
        isum11 <= isum02 + isum03;
        isum12 <= {isum04[DATA_WIDTH], isum04};

        isum20 <= isum10 + isum11;
        isum21 <= {isum12[DATA_WIDTH+1],isum12};

        sum <= isum20 + isum21;
    end
endmodule
`default_nettype wire
