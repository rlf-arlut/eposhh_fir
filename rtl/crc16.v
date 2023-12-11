// SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
// SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
//
// SPDX-License-Identifier: BSD-3-Clause

// vim: ts=4 sw=4 noexpandtab

// THIS IS GENERATED CODE.
//
// This code is Public Domain.
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
// SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
// RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
// NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
// USE OR PERFORMANCE OF THIS SOFTWARE.

`ifndef CRC16_V_
`define CRC16_V_

// CRC polynomial coefficients: x^16 + x^15 + x^2 + 1
//                              0xA001 (hex)
// CRC width:                   16 bits
// CRC shift direction:         right

module crc16 (
    input [15:0] crcIn,
    input [7:0] data,
    output [15:0] crcOut
);
    assign crcOut[0] = (crcIn[0] ^ crcIn[1] ^ crcIn[2] ^ crcIn[3] ^ crcIn[4] ^ crcIn[5] ^ crcIn[6] ^ crcIn[7] ^ crcIn[8] ^ data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7]);
    assign crcOut[1] = (crcIn[9]);
    assign crcOut[2] = (crcIn[10]);
    assign crcOut[3] = (crcIn[11]);
    assign crcOut[4] = (crcIn[12]);
    assign crcOut[5] = (crcIn[13]);
    assign crcOut[6] = (crcIn[0] ^ crcIn[14] ^ data[0]);
    assign crcOut[7] = (crcIn[0] ^ crcIn[1] ^ crcIn[15] ^ data[0] ^ data[1]);
    assign crcOut[8] = (crcIn[1] ^ crcIn[2] ^ data[1] ^ data[2]);
    assign crcOut[9] = (crcIn[2] ^ crcIn[3] ^ data[2] ^ data[3]);
    assign crcOut[10] = (crcIn[3] ^ crcIn[4] ^ data[3] ^ data[4]);
    assign crcOut[11] = (crcIn[4] ^ crcIn[5] ^ data[4] ^ data[5]);
    assign crcOut[12] = (crcIn[5] ^ crcIn[6] ^ data[5] ^ data[6]);
    assign crcOut[13] = (crcIn[6] ^ crcIn[7] ^ data[6] ^ data[7]);
    assign crcOut[14] = (crcIn[0] ^ crcIn[1] ^ crcIn[2] ^ crcIn[3] ^ crcIn[4] ^ crcIn[5] ^ crcIn[6] ^ data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6]);
    assign crcOut[15] = (crcIn[0] ^ crcIn[1] ^ crcIn[2] ^ crcIn[3] ^ crcIn[4] ^ crcIn[5] ^ crcIn[6] ^ crcIn[7] ^ data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7]);
endmodule

`endif // CRC16_V_
