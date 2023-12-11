// SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
// SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
//
// SPDX-License-Identifier: BSD-3-Clause

module majorityVoter #(
  parameter WIDTH = 1
)( 
  input wire  [WIDTH-1:0] inA,
  input wire  [WIDTH-1:0] inB,
  input wire  [WIDTH-1:0] inC,
  output wire [WIDTH-1:0] out,
  output reg              tmrErr
);
  assign out = (inA&inB) | (inA&inC) | (inB&inC);
  always @(inA or inB or inC) begin
    if (inA!=inB || inA!=inC || inB!=inC)
      tmrErr = 1;
    else
      tmrErr = 0;
  end
endmodule
