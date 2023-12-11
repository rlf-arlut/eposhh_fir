// SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
// SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
//
// SPDX-License-Identifier: BSD-3-Clause

/****************************************************************************************************
 *                          ! THIS FILE WAS AUTO-GENERATED BY TMRG TOOL !                           *
 *                                   ! DO NOT EDIT IT MANUALLY !                                    *
 *                                                                                                  *
 * file    : ./snapshot_txTMR.v                                                                     *
 *                                                                                                  *
 * user    : rfriesen                                                                               *
 * host    : sgl-lap054                                                                             *
 * date    : 08/09/2023 15:12:47                                                                    *
 *                                                                                                  *
 * workdir : /home/rfriesen/git/eposhh_fir5tap/rtl                                                  *
 * cmd     : /home/rfriesen/git/tmrg/bin/tmrg --log=fir_top_i2c-tmr.log --generate-report --stats -v -c *
 *           fir_top_i2c_tmr.cnf fir_top_i2c_tmr.v mult_reg.v snapshot_tx.v i2c_target_wbc.v        *
 *           i2c_target.v adder_tree10.v crc16.v                                                    *
 * tmrg rev: 6574161d3b316703ff7c8948d3a894c6005f77d4                                               *
 *                                                                                                  *
 * src file: snapshot_tx.v                                                                          *
 *           Git SHA           : 0ff2f60e67f53627c21a57a2c49e20cd48629ae9                           *
 *           Modification time : 2023-09-05 14:18:20.983464                                         *
 *           File Size         : 3846                                                               *
 *           MD5 hash          : ed20f25cf0fc2c92ed18a771a6e1ca79                                   *
 *                                                                                                  *
 ****************************************************************************************************/

`default_nettype  none
module snapshot_txTMR(
  clk,
  rst,
  ss_tx_start,
  ss_buff,
  idle,
  pdw_data,
  pdw_frame
);
parameter SAMPLE_WIDTH =16;
parameter COEF_WIDTH =16;
parameter NUM_TAPS =10;
parameter ABS_TIME_WIDTH =32;
parameter RF_SS_DELAY_WIDTH =$clog2(NUM_TAPS);
parameter SS_BUFF_SZ =(SAMPLE_WIDTH*NUM_TAPS)+ABS_TIME_WIDTH;
localparam SS_TX_IDLE =0;
localparam SS_TX_TXING =1;
localparam SS_TX_CRC =2;
localparam SS_TX_FIN =3;
localparam SS_TX_LEN =(SAMPLE_WIDTH*NUM_TAPS)+ABS_TIME_WIDTH+16;
localparam SS_TX_LEN_LOG2 =$clog2(SS_TX_LEN);
wire [SS_BUFF_SZ-1:0] ss_txbuffC;
wire [SS_BUFF_SZ-1:0] ss_txbuffB;
wire [SS_BUFF_SZ-1:0] ss_txbuffA;
wire ss_tx_startC;
wire ss_tx_startB;
wire ss_tx_startA;
wire rstC;
wire rstB;
wire rstA;
wire clkC;
wire clkB;
wire clkA;
wire ss_tx_csTmrError;
wire [1:0] ss_tx_cs;
wire crc16_regTmrError;
wire [15:0] crc16_reg;
input clk;
input rst;
input ss_tx_start;
input [SS_BUFF_SZ-1:0] ss_buff;
output idle;
output reg    pdw_data;
output reg    pdw_frame;
reg  [1:0] ss_tx_csA ;
reg  [1:0] ss_tx_nsA ;
reg  [1:0] ss_tx_csB ;
reg  [1:0] ss_tx_nsB ;
reg  [1:0] ss_tx_csC ;
reg  [1:0] ss_tx_nsC ;
reg  [SS_BUFF_SZ-1:0] ss_txbuff ;
reg  [SS_TX_LEN_LOG2-1:0] ss_tx_bit_cntA ;
reg  [SS_TX_LEN_LOG2-1:0] ss_tx_bit_cntB ;
reg  [SS_TX_LEN_LOG2-1:0] ss_tx_bit_cntC ;
reg  [15:0] crc16_regA ;
reg  [15:0] crc16_regB ;
reg  [15:0] crc16_regC ;
wire [15:0] crc16_outA;
wire [15:0] crc16_outB;
wire [15:0] crc16_outC;
wire [7:0] ss_tx_crc16_inA =  ss_txbuffA[7:0] ;
wire [7:0] ss_tx_crc16_inB =  ss_txbuffB[7:0] ;
wire [7:0] ss_tx_crc16_inC =  ss_txbuffC[7:0] ;
assign idle =  (ss_tx_cs==SS_TX_IDLE);

always @*
  begin
    case (ss_tx_csA)
      SS_TX_IDLE : 
        begin
          if (ss_tx_startA)
            begin
              ss_tx_nsA =  SS_TX_TXING;
            end
          else
            begin
              ss_tx_nsA =  SS_TX_IDLE;
            end
        end
      SS_TX_TXING : 
        begin
          if (ss_tx_bit_cntA<SS_TX_LEN-16-1)
            begin
              ss_tx_nsA =  SS_TX_TXING;
            end
          else
            begin
              ss_tx_nsA =  SS_TX_CRC;
            end
        end
      SS_TX_CRC : 
        begin
          if (ss_tx_bit_cntA==SS_TX_LEN-1)
            begin
              ss_tx_nsA =  SS_TX_FIN;
            end
          else
            begin
              ss_tx_nsA =  SS_TX_CRC;
            end
        end
      SS_TX_FIN : 
        begin
          ss_tx_nsA =  SS_TX_IDLE;
        end
    endcase
  end

always @*
  begin
    case (ss_tx_csB)
      SS_TX_IDLE : 
        begin
          if (ss_tx_startB)
            begin
              ss_tx_nsB =  SS_TX_TXING;
            end
          else
            begin
              ss_tx_nsB =  SS_TX_IDLE;
            end
        end
      SS_TX_TXING : 
        begin
          if (ss_tx_bit_cntB<SS_TX_LEN-16-1)
            begin
              ss_tx_nsB =  SS_TX_TXING;
            end
          else
            begin
              ss_tx_nsB =  SS_TX_CRC;
            end
        end
      SS_TX_CRC : 
        begin
          if (ss_tx_bit_cntB==SS_TX_LEN-1)
            begin
              ss_tx_nsB =  SS_TX_FIN;
            end
          else
            begin
              ss_tx_nsB =  SS_TX_CRC;
            end
        end
      SS_TX_FIN : 
        begin
          ss_tx_nsB =  SS_TX_IDLE;
        end
    endcase
  end

always @*
  begin
    case (ss_tx_csC)
      SS_TX_IDLE : 
        begin
          if (ss_tx_startC)
            begin
              ss_tx_nsC =  SS_TX_TXING;
            end
          else
            begin
              ss_tx_nsC =  SS_TX_IDLE;
            end
        end
      SS_TX_TXING : 
        begin
          if (ss_tx_bit_cntC<SS_TX_LEN-16-1)
            begin
              ss_tx_nsC =  SS_TX_TXING;
            end
          else
            begin
              ss_tx_nsC =  SS_TX_CRC;
            end
        end
      SS_TX_CRC : 
        begin
          if (ss_tx_bit_cntC==SS_TX_LEN-1)
            begin
              ss_tx_nsC =  SS_TX_FIN;
            end
          else
            begin
              ss_tx_nsC =  SS_TX_CRC;
            end
        end
      SS_TX_FIN : 
        begin
          ss_tx_nsC =  SS_TX_IDLE;
        end
    endcase
  end

always @( posedge clkA )
  begin
    if (rstA==1'b1)
      begin
        ss_tx_csA <= SS_TX_IDLE;
      end
    else
      begin
        ss_tx_csA <= ss_tx_nsA;
        case (ss_tx_csA)
          SS_TX_IDLE : 
            begin
              ss_tx_bit_cntA <= 0;
              crc16_regA <= 0;
            end
          SS_TX_TXING : 
            begin
              ss_tx_bit_cntA <= ss_tx_bit_cntA+1;
              if ((ss_tx_bit_cntA[3:0] ==0)||(ss_tx_bit_cntA[3:0] ==8))
                begin
                  crc16_regA <= crc16_outA;
                end
            end
          SS_TX_CRC : 
            begin
              crc16_regA <= {1'b0,crc16_regA[15:1] };
              ss_tx_bit_cntA <= ss_tx_bit_cntA+1;
            end
          SS_TX_FIN : 
            begin
            end
        endcase
      end
  end

always @( posedge clkB )
  begin
    if (rstB==1'b1)
      begin
        ss_tx_csB <= SS_TX_IDLE;
      end
    else
      begin
        ss_tx_csB <= ss_tx_nsB;
        case (ss_tx_csB)
          SS_TX_IDLE : 
            begin
              ss_tx_bit_cntB <= 0;
              crc16_regB <= 0;
            end
          SS_TX_TXING : 
            begin
              ss_tx_bit_cntB <= ss_tx_bit_cntB+1;
              if ((ss_tx_bit_cntB[3:0] ==0)||(ss_tx_bit_cntB[3:0] ==8))
                begin
                  crc16_regB <= crc16_outB;
                end
            end
          SS_TX_CRC : 
            begin
              crc16_regB <= {1'b0,crc16_regB[15:1] };
              ss_tx_bit_cntB <= ss_tx_bit_cntB+1;
            end
          SS_TX_FIN : 
            begin
            end
        endcase
      end
  end

always @( posedge clkC )
  begin
    if (rstC==1'b1)
      begin
        ss_tx_csC <= SS_TX_IDLE;
      end
    else
      begin
        ss_tx_csC <= ss_tx_nsC;
        case (ss_tx_csC)
          SS_TX_IDLE : 
            begin
              ss_tx_bit_cntC <= 0;
              crc16_regC <= 0;
            end
          SS_TX_TXING : 
            begin
              ss_tx_bit_cntC <= ss_tx_bit_cntC+1;
              if ((ss_tx_bit_cntC[3:0] ==0)||(ss_tx_bit_cntC[3:0] ==8))
                begin
                  crc16_regC <= crc16_outC;
                end
            end
          SS_TX_CRC : 
            begin
              crc16_regC <= {1'b0,crc16_regC[15:1] };
              ss_tx_bit_cntC <= ss_tx_bit_cntC+1;
            end
          SS_TX_FIN : 
            begin
            end
        endcase
      end
  end

always @( posedge clk )
  begin
    if (rst==1'b1)
      begin
        pdw_frame <= 0;
      end
    else
      begin
        case (ss_tx_cs)
          SS_TX_IDLE : 
            begin
              ss_txbuff <= ss_buff;
            end
          SS_TX_TXING : 
            begin
              ss_txbuff <= {1'b0,ss_txbuff[SS_BUFF_SZ-1:1] };
              pdw_frame <= 1'b1;
              pdw_data <= ss_txbuff[0] ;
            end
          SS_TX_CRC : 
            begin
              pdw_frame <= 1'b1;
              pdw_data <= crc16_reg[0] ;
            end
          SS_TX_FIN : 
            begin
              pdw_frame <= 1'b0;
            end
        endcase
      end
  end

crc16TMR crc16_ (
    .crcInA(crc16_regA),
    .crcInB(crc16_regB),
    .crcInC(crc16_regC),
    .dataA(ss_tx_crc16_inA),
    .dataB(ss_tx_crc16_inB),
    .dataC(ss_tx_crc16_inC),
    .crcOutA(crc16_outA),
    .crcOutB(crc16_outB),
    .crcOutC(crc16_outC)
    );

majorityVoter #(.WIDTH(16)) crc16_regVoter (
    .inA(crc16_regA),
    .inB(crc16_regB),
    .inC(crc16_regC),
    .out(crc16_reg),
    .tmrErr(crc16_regTmrError)
    );

majorityVoter #(.WIDTH(2)) ss_tx_csVoter (
    .inA(ss_tx_csA),
    .inB(ss_tx_csB),
    .inC(ss_tx_csC),
    .out(ss_tx_cs),
    .tmrErr(ss_tx_csTmrError)
    );

fanout clkFanout (
    .in(clk),
    .outA(clkA),
    .outB(clkB),
    .outC(clkC)
    );

fanout rstFanout (
    .in(rst),
    .outA(rstA),
    .outB(rstB),
    .outC(rstC)
    );

fanout ss_tx_startFanout (
    .in(ss_tx_start),
    .outA(ss_tx_startA),
    .outB(ss_tx_startB),
    .outC(ss_tx_startC)
    );

fanout #(.WIDTH(((SS_BUFF_SZ-1)>(0)) ? ((SS_BUFF_SZ-1)-(0)+1) : ((0)-(SS_BUFF_SZ-1)+1))) ss_txbuffFanout (
    .in(ss_txbuff),
    .outA(ss_txbuffA),
    .outB(ss_txbuffB),
    .outC(ss_txbuffC)
    );
endmodule

`default_nettype  wire
