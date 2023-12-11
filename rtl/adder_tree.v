// SPDX-License-Identifier: LGPL
// Code originally copied from https://opencores.org/projects/adder_tree
module adder_tree #(
    parameter N = 3,
    parameter DATA_WIDTH = 16,
    parameter RESULT_WIDTH = ((N - 1) < 2 ** $clog2(N)) ? DATA_WIDTH + $clog2(N) : DATA_WIDTH + $clog2(N) + 1
) (
    input clock,
    input [(DATA_WIDTH*N)-1:0] data,
    output signed [RESULT_WIDTH-1:0] result
);
    generate
        if (N == 2) begin : gen_only2
            add #(
                .DATAA_WIDTH (DATA_WIDTH),
                .DATAB_WIDTH (DATA_WIDTH),
                .RESULT_WIDTH(RESULT_WIDTH)
            ) add_inst (
                .clock    (clock),
                .dataa    (data[0+:DATA_WIDTH]),
                .datab    (data[DATA_WIDTH+:DATA_WIDTH]),
                .result   (result)
            );
        end else begin : gen_recurse
            localparam RES_WIDTH = (RESULT_WIDTH > DATA_WIDTH + 1) ? DATA_WIDTH + 1 : RESULT_WIDTH;
            localparam RESULTS = (N % 2 == 0) ? N / 2 : N / 2 + 1;

            wire signed [(RES_WIDTH*RESULTS)-1:0] res;

            add_pairs #(
                .N           (N),
                .DATA_WIDTH  (DATA_WIDTH),
                .RESULT_WIDTH(RES_WIDTH)
            ) add_pairs_inst (
                .clock    (clock),
                .data     (data),
                .result   (res)
            );

            adder_tree #(
                .N         (RESULTS),
                .DATA_WIDTH(RES_WIDTH)
            ) adder_tree_inst (
                .clock    (clock),
                .data     (res),
                .result   (result)
            );
        end
    endgenerate
    /*
`ifdef COCOTB_SIM
    initial begin
        $dumpfile("adder_tree.lxt2");
        $dumpvars;
    end
`endif  // COCOTB_SIM
*/

endmodule

//////////////////////
module add_pairs #(
    parameter N = 32,
    parameter DATA_WIDTH = 18,
    parameter RESULT_WIDTH = DATA_WIDTH + 1,
    parameter RESULTS = (N % 2 == 0) ? N / 2 : N / 2 + 1
) (
    input clock,
    input [(DATA_WIDTH*N)-1:0] data,
    output [(RESULT_WIDTH*RESULTS)-1:0] result
);
    genvar i;

    generate
        for (i = 0; i < N/2; i = i +1) begin : gen_adder
            add #(
                .DATAA_WIDTH (DATA_WIDTH),
                .DATAB_WIDTH (DATA_WIDTH),
                .RESULT_WIDTH(RESULT_WIDTH)
            ) add_inst (
                .clock,
                //.dataa (data[(2*i*DATA_WIDTH)+DATA_WIDTH-1:(2*i*DATA_WIDTH)]),
                //.datab (data[(2*i*DATA_WIDTH)+(2*DATA_WIDTH)-1:(2*i*DATA_WIDTH)+DATA_WIDTH]),
                //.result(result[(RESULT_WIDTH*i)+RESULT_WIDTH-1:(RESULT_WIDTH*i)])
                .dataa (data[(2*i*DATA_WIDTH)+:DATA_WIDTH]),
                .datab (data[(2*i*DATA_WIDTH)+DATA_WIDTH+:DATA_WIDTH]),
                .result(result[(RESULT_WIDTH*i)+:RESULT_WIDTH])
            );
        end

        if (RESULTS == N / 2 + 1) begin: gen_single
            reg [RESULT_WIDTH-1:0] res;

            always @(posedge clock) res <= $signed(data[(DATA_WIDTH*N)-1:(DATA_WIDTH*N)-DATA_WIDTH]);

            assign result[(RESULT_WIDTH*RESULTS)-1:(RESULT_WIDTH*RESULTS)-RESULT_WIDTH] = res;
        end
    endgenerate
endmodule

//////////////////////
module add #(
    parameter DATAA_WIDTH = 16,
    parameter DATAB_WIDTH = 17,
    parameter RESULT_WIDTH = (DATAA_WIDTH > DATAB_WIDTH) ? DATAA_WIDTH + 1 : DATAB_WIDTH + 1
) (
    input clock,
    input signed [DATAA_WIDTH-1:0] dataa,
    input signed [DATAB_WIDTH-1:0] datab,
    output reg signed [RESULT_WIDTH-1:0] result
);
    always @(posedge clock) begin
            result <= dataa + datab;
    end
endmodule
