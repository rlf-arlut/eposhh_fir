// SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
// SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
//
// SPDX-License-Identifier: BSD-3-Clause

module fanout #(
  parameter WIDTH = 1
)(
  input wire  [WIDTH-1:0] in,
  output wire [WIDTH-1:0] outA,
  output wire [WIDTH-1:0] outB,
  output wire [WIDTH-1:0] outC
);
  assign outA = in;
  assign outB = in;
  assign outC = in;
endmodule
