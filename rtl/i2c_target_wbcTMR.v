// SPDX-FileCopyrightText: 2023 Board of Regents, The University of Texas System
// SPDX-FileAttributionText: <text>This software was developed with government support under Grant no. DE-SC0023055 awarded by the Department of Energy. The government has certain rights in the copyright.</text>
//
// SPDX-License-Identifier: BSD-3-Clause

/****************************************************************************************************
 *                          ! THIS FILE WAS AUTO-GENERATED BY TMRG TOOL !                           *
 *                                   ! DO NOT EDIT IT MANUALLY !                                    *
 *                                                                                                  *
 * file    : ./i2c_target_wbcTMR.v                                                                  *
 *                                                                                                  *
 * user    : rfriesen                                                                               *
 * host    : sgl-lap054                                                                             *
 * date    : 11/09/2023 16:08:18                                                                    *
 *                                                                                                  *
 * workdir : /home/rfriesen/git/eposhh_fir5tap/rtl                                                  *
 * cmd     : /home/rfriesen/git/tmrg/bin/tmrg -v -c fir_top_i2c_tmr.cnf --no-common-definitions     *
 *           --generate-report fir_top_i2c_tmr.v adder_tree10.v crc16TMR.v snapshot_txTMR.v         *
 *           i2c_target.v i2c_target_wbc.v snapshot_txTMR.v mult_reg.v voter.v fanout.v             *
 * tmrg rev: 6574161d3b316703ff7c8948d3a894c6005f77d4                                               *
 *                                                                                                  *
 * src file: i2c_target_wbc.v                                                                       *
 *           Git SHA           : f952ba4289b6e09faf0e42a1ff4faddf2935b3fd                           *
 *           Modification time : 2023-09-11 16:04:40.117068                                         *
 *           File Size         : 17980                                                              *
 *           MD5 hash          : cc33969ec84a9a7846032ef2b3343846                                   *
 *                                                                                                  *
 ****************************************************************************************************/

module i2c_target_wbmTMR #(
  parameter  FILTER_LEN  = 4,
  parameter  WB_DATA_WIDTH  = 32,
  parameter  WB_ADDR_WIDTH  = 32,
  parameter  WB_SELECT_WIDTH  = (WB_DATA_WIDTH/8)
)(
  input wire  clk ,
  input wire  rst ,
  input wire  i2c_scl_i ,
  output wire  i2c_scl_o ,
  output wire  i2c_scl_t ,
  input wire  i2c_sda_i ,
  output wire  i2c_sda_o ,
  output wire  i2c_sda_t ,
  output wire [WB_ADDR_WIDTH-1:0] wb_adr_o ,
  input wire [WB_DATA_WIDTH-1:0] wb_dat_i ,
  output wire [WB_DATA_WIDTH-1:0] wb_dat_o ,
  output wire  wb_we_o ,
  output wire [WB_SELECT_WIDTH-1:0] wb_sel_o ,
  output wire  wb_stb_o ,
  input wire  wb_ack_i ,
  input wire  wb_err_i ,
  output wire  wb_cyc_o ,
  output wire  busy ,
  output wire  bus_addressed ,
  output wire  bus_active ,
  input wire  enable ,
  input wire [6:0] device_address 
);
parameter WB_VALID_ADDR_WIDTH =WB_ADDR_WIDTH-$clog2(WB_SELECT_WIDTH);
parameter WB_WORD_WIDTH =WB_SELECT_WIDTH;
parameter WB_WORD_SIZE =WB_DATA_WIDTH/WB_WORD_WIDTH;
parameter WORD_PART_ADDR_WIDTH =$clog2(WB_WORD_SIZE/8);
parameter ADDR_WIDTH_ADJ =WB_ADDR_WIDTH+WORD_PART_ADDR_WIDTH;
parameter ADDR_WORD_WIDTH =(ADDR_WIDTH_ADJ+7)/8;
localparam [2:0] STATE_IDLE =3'd0;
localparam [2:0] STATE_ADDRESS =3'd1;
localparam [2:0] STATE_READ_1 =3'd2;
localparam [2:0] STATE_READ_2 =3'd3;
localparam [2:0] STATE_WRITE_1 =3'd4;
localparam [2:0] STATE_WRITE_2 =3'd5;
wire wb_err_iC;
wire wb_err_iB;
wire wb_err_iA;
wire [WB_DATA_WIDTH-1:0] wb_dat_iC;
wire [WB_DATA_WIDTH-1:0] wb_dat_iB;
wire [WB_DATA_WIDTH-1:0] wb_dat_iA;
wire wb_ack_iC;
wire wb_ack_iB;
wire wb_ack_iA;
wire rstC;
wire rstB;
wire rstA;
wire i2c_sda_iC;
wire i2c_sda_iB;
wire i2c_sda_iA;
wire i2c_scl_iC;
wire i2c_scl_iB;
wire i2c_scl_iA;
wire enableC;
wire enableB;
wire enableA;
wire [6:0] device_addressC;
wire [6:0] device_addressB;
wire [6:0] device_addressA;
wire clkC;
wire clkB;
wire clkA;
wire bus_addressedC;
wire bus_addressedB;
wire bus_addressedA;
wire wb_we_o_regTmrError;
wire wb_we_o_reg;
wire wb_stb_o_regTmrError;
wire wb_stb_o_reg;
wire wb_sel_o_regTmrError;
wire [WB_SELECT_WIDTH-1:0] wb_sel_o_reg;
wire wb_cyc_o_regTmrError;
wire wb_cyc_o_reg;
wire i2c_sda_tTmrError;
wire i2c_sda_tB;
wire i2c_sda_tC;
wire i2c_sda_tA;
wire i2c_sda_oTmrError;
wire i2c_sda_oB;
wire i2c_sda_oC;
wire i2c_sda_oA;
wire i2c_scl_tTmrError;
wire i2c_scl_tB;
wire i2c_scl_tC;
wire i2c_scl_tA;
wire i2c_scl_oTmrError;
wire i2c_scl_oB;
wire i2c_scl_oC;
wire i2c_scl_oA;
wire data_regTmrError;
wire [WB_DATA_WIDTH-1:0] data_reg;
wire busy_regTmrError;
wire busy_reg;
wire bus_addressedTmrError;
wire bus_activeTmrError;
wire bus_activeB;
wire bus_activeC;
wire bus_activeA;
wire addr_regTmrError;
wire [ADDR_WIDTH_ADJ-1:0] addr_reg;
initial
  begin
    if (WB_WORD_WIDTH*WB_WORD_SIZE!=WB_DATA_WIDTH)
      begin
        $error("Error: WB data width not evenly divisble");
        $finish;
      end
    if (2**$clog2(WB_WORD_WIDTH)!=WB_WORD_WIDTH)
      begin
        $error("Error: WB word width must be even power of two");
        $finish;
      end
    if (8*2**$clog2(WB_WORD_SIZE/8)!=WB_WORD_SIZE)
      begin
        $error("Error: WB word size must be a power of two multiple of 8 bits");
        $finish;
      end
  end
reg  [2:0] state_regA ;
reg  [2:0] state_nextA ;
reg  [2:0] state_regB ;
reg  [2:0] state_nextB ;
reg  [2:0] state_regC ;
reg  [2:0] state_nextC ;
reg  [7:0] count_regA ;
reg  [7:0] count_nextA ;
reg  [7:0] count_regB ;
reg  [7:0] count_nextB ;
reg  [7:0] count_regC ;
reg  [7:0] count_nextC ;
reg  last_cycle_regA ;
reg  last_cycle_regB ;
reg  last_cycle_regC ;
reg  [ADDR_WIDTH_ADJ-1:0] addr_regA ;
reg  [ADDR_WIDTH_ADJ-1:0] addr_nextA ;
reg  [ADDR_WIDTH_ADJ-1:0] addr_regB ;
reg  [ADDR_WIDTH_ADJ-1:0] addr_nextB ;
reg  [ADDR_WIDTH_ADJ-1:0] addr_regC ;
reg  [ADDR_WIDTH_ADJ-1:0] addr_nextC ;
reg  [WB_DATA_WIDTH-1:0] data_regA ;
reg  [WB_DATA_WIDTH-1:0] data_nextA ;
reg  [WB_DATA_WIDTH-1:0] data_regB ;
reg  [WB_DATA_WIDTH-1:0] data_nextB ;
reg  [WB_DATA_WIDTH-1:0] data_regC ;
reg  [WB_DATA_WIDTH-1:0] data_nextC ;
reg  wb_we_o_regA ;
reg  wb_we_o_nextA ;
reg  wb_we_o_regB ;
reg  wb_we_o_nextB ;
reg  wb_we_o_regC ;
reg  wb_we_o_nextC ;
reg  [WB_SELECT_WIDTH-1:0] wb_sel_o_regA ;
reg  [WB_SELECT_WIDTH-1:0] wb_sel_o_nextA ;
reg  [WB_SELECT_WIDTH-1:0] wb_sel_o_regB ;
reg  [WB_SELECT_WIDTH-1:0] wb_sel_o_nextB ;
reg  [WB_SELECT_WIDTH-1:0] wb_sel_o_regC ;
reg  [WB_SELECT_WIDTH-1:0] wb_sel_o_nextC ;
reg  wb_stb_o_regA ;
reg  wb_stb_o_nextA ;
reg  wb_stb_o_regB ;
reg  wb_stb_o_nextB ;
reg  wb_stb_o_regC ;
reg  wb_stb_o_nextC ;
reg  wb_cyc_o_regA ;
reg  wb_cyc_o_nextA ;
reg  wb_cyc_o_regB ;
reg  wb_cyc_o_nextB ;
reg  wb_cyc_o_regC ;
reg  wb_cyc_o_nextC ;
reg  busy_regA ;
reg  busy_regB ;
reg  busy_regC ;
reg  [7:0] data_in_regA ;
reg  [7:0] data_in_nextA ;
reg  [7:0] data_in_regB ;
reg  [7:0] data_in_nextB ;
reg  [7:0] data_in_regC ;
reg  [7:0] data_in_nextC ;
reg  data_in_valid_regA ;
reg  data_in_valid_nextA ;
reg  data_in_valid_regB ;
reg  data_in_valid_nextB ;
reg  data_in_valid_regC ;
reg  data_in_valid_nextC ;
wire data_in_readyA;
wire data_in_readyB;
wire data_in_readyC;
wire [7:0] data_outA;
wire [7:0] data_outB;
wire [7:0] data_outC;
wire data_out_validA;
wire data_out_validB;
wire data_out_validC;
wire data_out_lastA;
wire data_out_lastB;
wire data_out_lastC;
reg  data_out_ready_regA ;
reg  data_out_ready_nextA ;
reg  data_out_ready_regB ;
reg  data_out_ready_nextB ;
reg  data_out_ready_regC ;
reg  data_out_ready_nextC ;
assign wb_adr_o =  {addr_reg[ADDR_WIDTH_ADJ-1:ADDR_WIDTH_ADJ-WB_VALID_ADDR_WIDTH] ,{ WB_ADDR_WIDTH - WB_VALID_ADDR_WIDTH {1'b0} } };
assign wb_dat_o =  data_reg;
assign wb_we_o =  wb_we_o_reg;
assign wb_sel_o =  wb_sel_o_reg;
assign wb_stb_o =  wb_stb_o_reg;
assign wb_cyc_o =  wb_cyc_o_reg;
assign busy =  busy_reg;

always @*
  begin
    state_nextA =  STATE_IDLE;
    count_nextA =  count_regA;
    data_in_nextA =  8'd0;
    data_in_valid_nextA =  1'b0;
    data_out_ready_nextA =  1'b0;
    addr_nextA =  addr_regA;
    data_nextA =  data_regA;
    wb_we_o_nextA =  wb_we_o_regA;
    wb_sel_o_nextA =  wb_sel_o_regA;
    wb_stb_o_nextA =  1'b0;
    wb_cyc_o_nextA =  1'b0;
    case (state_regA)
      STATE_IDLE : 
        begin
          wb_we_o_nextA =  1'b0;
          if (data_out_validA)
            begin
              count_nextA =  ADDR_WORD_WIDTH-1;
              state_nextA =  STATE_ADDRESS;
            end
          else
            if (data_in_readyA&~data_in_valid_regA)
              begin
                wb_cyc_o_nextA =  1'b1;
                wb_stb_o_nextA =  1'b1;
                wb_sel_o_nextA =  {WB_SELECT_WIDTH{1'b1}};
                state_nextA =  STATE_READ_1;
              end
        end
      STATE_ADDRESS : 
        begin
          data_out_ready_nextA =  1'b1;
          if (data_out_ready_regA&data_out_validA)
            begin
              addr_nextA[8*count_regA+:8]  =  data_outA;
              count_nextA =  count_regA-1;
              if (count_regA==0)
                begin
                  if (WB_ADDR_WIDTH==WB_VALID_ADDR_WIDTH&&WORD_PART_ADDR_WIDTH==0)
                    begin
                      count_nextA =  0;
                    end
                  else
                    begin
                      count_nextA =  addr_nextA[ADDR_WIDTH_ADJ-WB_VALID_ADDR_WIDTH-1:0] ;
                    end
                  wb_sel_o_nextA =  {WB_SELECT_WIDTH{1'b0}};
                  data_nextA =  {WB_DATA_WIDTH{1'b0}};
                  if (data_out_lastA)
                    begin
                      state_nextA =  STATE_IDLE;
                    end
                  else
                    begin
                      state_nextA =  STATE_WRITE_1;
                    end
                end
              else
                begin
                  if (data_out_lastA)
                    begin
                      state_nextA =  STATE_IDLE;
                    end
                  else
                    begin
                      state_nextA =  STATE_ADDRESS;
                    end
                end
            end
          else
            begin
              state_nextA =  STATE_ADDRESS;
            end
        end
      STATE_READ_1 : 
        begin
          wb_cyc_o_nextA =  1'b1;
          wb_stb_o_nextA =  1'b1;
          if (wb_ack_iA||wb_err_iA)
            begin
              data_nextA =  wb_dat_iA;
              addr_nextA =  addr_regA+(1<<(WB_ADDR_WIDTH-WB_VALID_ADDR_WIDTH+WORD_PART_ADDR_WIDTH));
              wb_cyc_o_nextA =  1'b0;
              wb_stb_o_nextA =  1'b0;
              wb_sel_o_nextA =  {WB_SELECT_WIDTH{1'b0}};
              state_nextA =  STATE_READ_2;
            end
          else
            begin
              state_nextA =  STATE_READ_1;
            end
        end
      STATE_READ_2 : 
        begin
          if (data_out_validA|!bus_addressedA)
            begin
              state_nextA =  STATE_IDLE;
            end
          else
            if (data_in_readyA&~data_in_valid_regA)
              begin
                data_in_nextA =  data_regA[8*count_regA+:8] ;
                data_in_valid_nextA =  1'b1;
                count_nextA =  count_regA+1;
                if (count_regA==(WB_SELECT_WIDTH*WB_WORD_SIZE/8)-1)
                  begin
                    count_nextA =  0;
                    state_nextA =  STATE_IDLE;
                  end
                else
                  begin
                    state_nextA =  STATE_READ_2;
                  end
              end
            else
              begin
                state_nextA =  STATE_READ_2;
              end
        end
      STATE_WRITE_1 : 
        begin
          data_out_ready_nextA =  1'b1;
          if (data_out_ready_regA&data_out_validA)
            begin
              data_nextA[8*count_regA+:8]  =  data_outA;
              count_nextA =  count_regA+1;
              wb_sel_o_nextA[count_regA>>((WB_WORD_SIZE/8)-1)]  =  1'b1;
              if (count_regA==(WB_SELECT_WIDTH*WB_WORD_SIZE/8)-1||data_out_lastA)
                begin
                  count_nextA =  0;
                  wb_we_o_nextA =  1'b1;
                  wb_cyc_o_nextA =  1'b1;
                  wb_stb_o_nextA =  1'b1;
                  state_nextA =  STATE_WRITE_2;
                end
              else
                begin
                  state_nextA =  STATE_WRITE_1;
                end
            end
          else
            begin
              state_nextA =  STATE_WRITE_1;
            end
        end
      STATE_WRITE_2 : 
        begin
          wb_cyc_o_nextA =  1'b1;
          wb_stb_o_nextA =  1'b1;
          if (wb_ack_iA||wb_err_iA)
            begin
              data_nextA =  {WB_DATA_WIDTH{1'b0}};
              addr_nextA =  addr_regA+(1<<(WB_ADDR_WIDTH-WB_VALID_ADDR_WIDTH+WORD_PART_ADDR_WIDTH));
              wb_cyc_o_nextA =  1'b0;
              wb_stb_o_nextA =  1'b0;
              wb_sel_o_nextA =  {WB_SELECT_WIDTH{1'b0}};
              if (last_cycle_regA)
                begin
                  state_nextA =  STATE_IDLE;
                end
              else
                begin
                  state_nextA =  STATE_WRITE_1;
                end
            end
          else
            begin
              state_nextA =  STATE_WRITE_2;
            end
        end
    endcase
  end

always @*
  begin
    state_nextB =  STATE_IDLE;
    count_nextB =  count_regB;
    data_in_nextB =  8'd0;
    data_in_valid_nextB =  1'b0;
    data_out_ready_nextB =  1'b0;
    addr_nextB =  addr_regB;
    data_nextB =  data_regB;
    wb_we_o_nextB =  wb_we_o_regB;
    wb_sel_o_nextB =  wb_sel_o_regB;
    wb_stb_o_nextB =  1'b0;
    wb_cyc_o_nextB =  1'b0;
    case (state_regB)
      STATE_IDLE : 
        begin
          wb_we_o_nextB =  1'b0;
          if (data_out_validB)
            begin
              count_nextB =  ADDR_WORD_WIDTH-1;
              state_nextB =  STATE_ADDRESS;
            end
          else
            if (data_in_readyB&~data_in_valid_regB)
              begin
                wb_cyc_o_nextB =  1'b1;
                wb_stb_o_nextB =  1'b1;
                wb_sel_o_nextB =  {WB_SELECT_WIDTH{1'b1}};
                state_nextB =  STATE_READ_1;
              end
        end
      STATE_ADDRESS : 
        begin
          data_out_ready_nextB =  1'b1;
          if (data_out_ready_regB&data_out_validB)
            begin
              addr_nextB[8*count_regB+:8]  =  data_outB;
              count_nextB =  count_regB-1;
              if (count_regB==0)
                begin
                  if (WB_ADDR_WIDTH==WB_VALID_ADDR_WIDTH&&WORD_PART_ADDR_WIDTH==0)
                    begin
                      count_nextB =  0;
                    end
                  else
                    begin
                      count_nextB =  addr_nextB[ADDR_WIDTH_ADJ-WB_VALID_ADDR_WIDTH-1:0] ;
                    end
                  wb_sel_o_nextB =  {WB_SELECT_WIDTH{1'b0}};
                  data_nextB =  {WB_DATA_WIDTH{1'b0}};
                  if (data_out_lastB)
                    begin
                      state_nextB =  STATE_IDLE;
                    end
                  else
                    begin
                      state_nextB =  STATE_WRITE_1;
                    end
                end
              else
                begin
                  if (data_out_lastB)
                    begin
                      state_nextB =  STATE_IDLE;
                    end
                  else
                    begin
                      state_nextB =  STATE_ADDRESS;
                    end
                end
            end
          else
            begin
              state_nextB =  STATE_ADDRESS;
            end
        end
      STATE_READ_1 : 
        begin
          wb_cyc_o_nextB =  1'b1;
          wb_stb_o_nextB =  1'b1;
          if (wb_ack_iB||wb_err_iB)
            begin
              data_nextB =  wb_dat_iB;
              addr_nextB =  addr_regB+(1<<(WB_ADDR_WIDTH-WB_VALID_ADDR_WIDTH+WORD_PART_ADDR_WIDTH));
              wb_cyc_o_nextB =  1'b0;
              wb_stb_o_nextB =  1'b0;
              wb_sel_o_nextB =  {WB_SELECT_WIDTH{1'b0}};
              state_nextB =  STATE_READ_2;
            end
          else
            begin
              state_nextB =  STATE_READ_1;
            end
        end
      STATE_READ_2 : 
        begin
          if (data_out_validB|!bus_addressedB)
            begin
              state_nextB =  STATE_IDLE;
            end
          else
            if (data_in_readyB&~data_in_valid_regB)
              begin
                data_in_nextB =  data_regB[8*count_regB+:8] ;
                data_in_valid_nextB =  1'b1;
                count_nextB =  count_regB+1;
                if (count_regB==(WB_SELECT_WIDTH*WB_WORD_SIZE/8)-1)
                  begin
                    count_nextB =  0;
                    state_nextB =  STATE_IDLE;
                  end
                else
                  begin
                    state_nextB =  STATE_READ_2;
                  end
              end
            else
              begin
                state_nextB =  STATE_READ_2;
              end
        end
      STATE_WRITE_1 : 
        begin
          data_out_ready_nextB =  1'b1;
          if (data_out_ready_regB&data_out_validB)
            begin
              data_nextB[8*count_regB+:8]  =  data_outB;
              count_nextB =  count_regB+1;
              wb_sel_o_nextB[count_regB>>((WB_WORD_SIZE/8)-1)]  =  1'b1;
              if (count_regB==(WB_SELECT_WIDTH*WB_WORD_SIZE/8)-1||data_out_lastB)
                begin
                  count_nextB =  0;
                  wb_we_o_nextB =  1'b1;
                  wb_cyc_o_nextB =  1'b1;
                  wb_stb_o_nextB =  1'b1;
                  state_nextB =  STATE_WRITE_2;
                end
              else
                begin
                  state_nextB =  STATE_WRITE_1;
                end
            end
          else
            begin
              state_nextB =  STATE_WRITE_1;
            end
        end
      STATE_WRITE_2 : 
        begin
          wb_cyc_o_nextB =  1'b1;
          wb_stb_o_nextB =  1'b1;
          if (wb_ack_iB||wb_err_iB)
            begin
              data_nextB =  {WB_DATA_WIDTH{1'b0}};
              addr_nextB =  addr_regB+(1<<(WB_ADDR_WIDTH-WB_VALID_ADDR_WIDTH+WORD_PART_ADDR_WIDTH));
              wb_cyc_o_nextB =  1'b0;
              wb_stb_o_nextB =  1'b0;
              wb_sel_o_nextB =  {WB_SELECT_WIDTH{1'b0}};
              if (last_cycle_regB)
                begin
                  state_nextB =  STATE_IDLE;
                end
              else
                begin
                  state_nextB =  STATE_WRITE_1;
                end
            end
          else
            begin
              state_nextB =  STATE_WRITE_2;
            end
        end
    endcase
  end

always @*
  begin
    state_nextC =  STATE_IDLE;
    count_nextC =  count_regC;
    data_in_nextC =  8'd0;
    data_in_valid_nextC =  1'b0;
    data_out_ready_nextC =  1'b0;
    addr_nextC =  addr_regC;
    data_nextC =  data_regC;
    wb_we_o_nextC =  wb_we_o_regC;
    wb_sel_o_nextC =  wb_sel_o_regC;
    wb_stb_o_nextC =  1'b0;
    wb_cyc_o_nextC =  1'b0;
    case (state_regC)
      STATE_IDLE : 
        begin
          wb_we_o_nextC =  1'b0;
          if (data_out_validC)
            begin
              count_nextC =  ADDR_WORD_WIDTH-1;
              state_nextC =  STATE_ADDRESS;
            end
          else
            if (data_in_readyC&~data_in_valid_regC)
              begin
                wb_cyc_o_nextC =  1'b1;
                wb_stb_o_nextC =  1'b1;
                wb_sel_o_nextC =  {WB_SELECT_WIDTH{1'b1}};
                state_nextC =  STATE_READ_1;
              end
        end
      STATE_ADDRESS : 
        begin
          data_out_ready_nextC =  1'b1;
          if (data_out_ready_regC&data_out_validC)
            begin
              addr_nextC[8*count_regC+:8]  =  data_outC;
              count_nextC =  count_regC-1;
              if (count_regC==0)
                begin
                  if (WB_ADDR_WIDTH==WB_VALID_ADDR_WIDTH&&WORD_PART_ADDR_WIDTH==0)
                    begin
                      count_nextC =  0;
                    end
                  else
                    begin
                      count_nextC =  addr_nextC[ADDR_WIDTH_ADJ-WB_VALID_ADDR_WIDTH-1:0] ;
                    end
                  wb_sel_o_nextC =  {WB_SELECT_WIDTH{1'b0}};
                  data_nextC =  {WB_DATA_WIDTH{1'b0}};
                  if (data_out_lastC)
                    begin
                      state_nextC =  STATE_IDLE;
                    end
                  else
                    begin
                      state_nextC =  STATE_WRITE_1;
                    end
                end
              else
                begin
                  if (data_out_lastC)
                    begin
                      state_nextC =  STATE_IDLE;
                    end
                  else
                    begin
                      state_nextC =  STATE_ADDRESS;
                    end
                end
            end
          else
            begin
              state_nextC =  STATE_ADDRESS;
            end
        end
      STATE_READ_1 : 
        begin
          wb_cyc_o_nextC =  1'b1;
          wb_stb_o_nextC =  1'b1;
          if (wb_ack_iC||wb_err_iC)
            begin
              data_nextC =  wb_dat_iC;
              addr_nextC =  addr_regC+(1<<(WB_ADDR_WIDTH-WB_VALID_ADDR_WIDTH+WORD_PART_ADDR_WIDTH));
              wb_cyc_o_nextC =  1'b0;
              wb_stb_o_nextC =  1'b0;
              wb_sel_o_nextC =  {WB_SELECT_WIDTH{1'b0}};
              state_nextC =  STATE_READ_2;
            end
          else
            begin
              state_nextC =  STATE_READ_1;
            end
        end
      STATE_READ_2 : 
        begin
          if (data_out_validC|!bus_addressedC)
            begin
              state_nextC =  STATE_IDLE;
            end
          else
            if (data_in_readyC&~data_in_valid_regC)
              begin
                data_in_nextC =  data_regC[8*count_regC+:8] ;
                data_in_valid_nextC =  1'b1;
                count_nextC =  count_regC+1;
                if (count_regC==(WB_SELECT_WIDTH*WB_WORD_SIZE/8)-1)
                  begin
                    count_nextC =  0;
                    state_nextC =  STATE_IDLE;
                  end
                else
                  begin
                    state_nextC =  STATE_READ_2;
                  end
              end
            else
              begin
                state_nextC =  STATE_READ_2;
              end
        end
      STATE_WRITE_1 : 
        begin
          data_out_ready_nextC =  1'b1;
          if (data_out_ready_regC&data_out_validC)
            begin
              data_nextC[8*count_regC+:8]  =  data_outC;
              count_nextC =  count_regC+1;
              wb_sel_o_nextC[count_regC>>((WB_WORD_SIZE/8)-1)]  =  1'b1;
              if (count_regC==(WB_SELECT_WIDTH*WB_WORD_SIZE/8)-1||data_out_lastC)
                begin
                  count_nextC =  0;
                  wb_we_o_nextC =  1'b1;
                  wb_cyc_o_nextC =  1'b1;
                  wb_stb_o_nextC =  1'b1;
                  state_nextC =  STATE_WRITE_2;
                end
              else
                begin
                  state_nextC =  STATE_WRITE_1;
                end
            end
          else
            begin
              state_nextC =  STATE_WRITE_1;
            end
        end
      STATE_WRITE_2 : 
        begin
          wb_cyc_o_nextC =  1'b1;
          wb_stb_o_nextC =  1'b1;
          if (wb_ack_iC||wb_err_iC)
            begin
              data_nextC =  {WB_DATA_WIDTH{1'b0}};
              addr_nextC =  addr_regC+(1<<(WB_ADDR_WIDTH-WB_VALID_ADDR_WIDTH+WORD_PART_ADDR_WIDTH));
              wb_cyc_o_nextC =  1'b0;
              wb_stb_o_nextC =  1'b0;
              wb_sel_o_nextC =  {WB_SELECT_WIDTH{1'b0}};
              if (last_cycle_regC)
                begin
                  state_nextC =  STATE_IDLE;
                end
              else
                begin
                  state_nextC =  STATE_WRITE_1;
                end
            end
          else
            begin
              state_nextC =  STATE_WRITE_2;
            end
        end
    endcase
  end

always @( posedge clkA )
  begin
    state_regA <= state_nextA;
    count_regA <= count_nextA;
    if (data_out_ready_regA&data_out_validA)
      begin
        last_cycle_regA <= data_out_lastA;
      end
    addr_regA <= addr_nextA;
    data_regA <= data_nextA;
    wb_we_o_regA <= wb_we_o_nextA;
    wb_sel_o_regA <= wb_sel_o_nextA;
    wb_stb_o_regA <= wb_stb_o_nextA;
    wb_cyc_o_regA <= wb_cyc_o_nextA;
    busy_regA <= state_nextA!=STATE_IDLE;
    data_in_regA <= data_in_nextA;
    data_in_valid_regA <= data_in_valid_nextA;
    data_out_ready_regA <= data_out_ready_nextA;
    if (rstA)
      begin
        state_regA <= STATE_IDLE;
        data_in_valid_regA <= 1'b0;
        data_out_ready_regA <= 1'b0;
        wb_stb_o_regA <= 1'b0;
        wb_cyc_o_regA <= 1'b0;
        busy_regA <= 1'b0;
        last_cycle_regA <= 1'b0;
        addr_regA <= {ADDR_WIDTH_ADJ{1'b0}};
        wb_we_o_regA <= 1'b0;
      end
  end

always @( posedge clkB )
  begin
    state_regB <= state_nextB;
    count_regB <= count_nextB;
    if (data_out_ready_regB&data_out_validB)
      begin
        last_cycle_regB <= data_out_lastB;
      end
    addr_regB <= addr_nextB;
    data_regB <= data_nextB;
    wb_we_o_regB <= wb_we_o_nextB;
    wb_sel_o_regB <= wb_sel_o_nextB;
    wb_stb_o_regB <= wb_stb_o_nextB;
    wb_cyc_o_regB <= wb_cyc_o_nextB;
    busy_regB <= state_nextB!=STATE_IDLE;
    data_in_regB <= data_in_nextB;
    data_in_valid_regB <= data_in_valid_nextB;
    data_out_ready_regB <= data_out_ready_nextB;
    if (rstB)
      begin
        state_regB <= STATE_IDLE;
        data_in_valid_regB <= 1'b0;
        data_out_ready_regB <= 1'b0;
        wb_stb_o_regB <= 1'b0;
        wb_cyc_o_regB <= 1'b0;
        busy_regB <= 1'b0;
        last_cycle_regB <= 1'b0;
        addr_regB <= {ADDR_WIDTH_ADJ{1'b0}};
        wb_we_o_regB <= 1'b0;
      end
  end

always @( posedge clkC )
  begin
    state_regC <= state_nextC;
    count_regC <= count_nextC;
    if (data_out_ready_regC&data_out_validC)
      begin
        last_cycle_regC <= data_out_lastC;
      end
    addr_regC <= addr_nextC;
    data_regC <= data_nextC;
    wb_we_o_regC <= wb_we_o_nextC;
    wb_sel_o_regC <= wb_sel_o_nextC;
    wb_stb_o_regC <= wb_stb_o_nextC;
    wb_cyc_o_regC <= wb_cyc_o_nextC;
    busy_regC <= state_nextC!=STATE_IDLE;
    data_in_regC <= data_in_nextC;
    data_in_valid_regC <= data_in_valid_nextC;
    data_out_ready_regC <= data_out_ready_nextC;
    if (rstC)
      begin
        state_regC <= STATE_IDLE;
        data_in_valid_regC <= 1'b0;
        data_out_ready_regC <= 1'b0;
        wb_stb_o_regC <= 1'b0;
        wb_cyc_o_regC <= 1'b0;
        busy_regC <= 1'b0;
        last_cycle_regC <= 1'b0;
        addr_regC <= {ADDR_WIDTH_ADJ{1'b0}};
        wb_we_o_regC <= 1'b0;
      end
  end

i2c_targetTMR #(.FILTER_LEN(FILTER_LEN)) i2c_target_inst (
    .clk(clk),
    .rst(rst),
    .release_busA(1'b0),
    .release_busB(1'b0),
    .release_busC(1'b0),
    .s_axis_data_tdataA(data_in_regA),
    .s_axis_data_tdataB(data_in_regB),
    .s_axis_data_tdataC(data_in_regC),
    .s_axis_data_tvalidA(data_in_valid_regA),
    .s_axis_data_tvalidB(data_in_valid_regB),
    .s_axis_data_tvalidC(data_in_valid_regC),
    .s_axis_data_treadyA(data_in_readyA),
    .s_axis_data_treadyB(data_in_readyB),
    .s_axis_data_treadyC(data_in_readyC),
    .s_axis_data_tlastA(1'b0),
    .s_axis_data_tlastB(1'b0),
    .s_axis_data_tlastC(1'b0),
    .m_axis_data_tdataA(data_outA),
    .m_axis_data_tdataB(data_outB),
    .m_axis_data_tdataC(data_outC),
    .m_axis_data_tvalidA(data_out_validA),
    .m_axis_data_tvalidB(data_out_validB),
    .m_axis_data_tvalidC(data_out_validC),
    .m_axis_data_treadyA(data_out_ready_regA),
    .m_axis_data_treadyB(data_out_ready_regB),
    .m_axis_data_treadyC(data_out_ready_regC),
    .m_axis_data_tlastA(data_out_lastA),
    .m_axis_data_tlastB(data_out_lastB),
    .m_axis_data_tlastC(data_out_lastC),
    .scl_iA(i2c_scl_iA),
    .scl_iB(i2c_scl_iB),
    .scl_iC(i2c_scl_iC),
    .scl_oA(i2c_scl_oA),
    .scl_oB(i2c_scl_oB),
    .scl_oC(i2c_scl_oC),
    .scl_tA(i2c_scl_tA),
    .scl_tB(i2c_scl_tB),
    .scl_tC(i2c_scl_tC),
    .sda_iA(i2c_sda_iA),
    .sda_iB(i2c_sda_iB),
    .sda_iC(i2c_sda_iC),
    .sda_oA(i2c_sda_oA),
    .sda_oB(i2c_sda_oB),
    .sda_oC(i2c_sda_oC),
    .sda_tA(i2c_sda_tA),
    .sda_tB(i2c_sda_tB),
    .sda_tC(i2c_sda_tC),
    .busyA(),
    .busyB(),
    .busyC(),
    .bus_addressA(),
    .bus_addressB(),
    .bus_addressC(),
    .bus_addressedA(bus_addressedA),
    .bus_addressedB(bus_addressedB),
    .bus_addressedC(bus_addressedC),
    .bus_activeA(bus_activeA),
    .bus_activeB(bus_activeB),
    .bus_activeC(bus_activeC),
    .enableA(enableA),
    .enableB(enableB),
    .enableC(enableC),
    .device_addressA(device_addressA),
    .device_addressB(device_addressB),
    .device_addressC(device_addressC),
    .device_address_maskA(7'h7f),
    .device_address_maskB(7'h7f),
    .device_address_maskC(7'h7f)
    );

majorityVoter #(.WIDTH(((ADDR_WIDTH_ADJ-1)>(0)) ? ((ADDR_WIDTH_ADJ-1)-(0)+1) : ((0)-(ADDR_WIDTH_ADJ-1)+1))) addr_regVoter (
    .inA(addr_regA),
    .inB(addr_regB),
    .inC(addr_regC),
    .out(addr_reg),
    .tmrErr(addr_regTmrError)
    );

majorityVoter bus_activeVoter (
    .inA(bus_activeA),
    .inB(bus_activeB),
    .inC(bus_activeC),
    .out(bus_active),
    .tmrErr(bus_activeTmrError)
    );

majorityVoter bus_addressedVoter (
    .inA(bus_addressedA),
    .inB(bus_addressedB),
    .inC(bus_addressedC),
    .out(bus_addressed),
    .tmrErr(bus_addressedTmrError)
    );

majorityVoter busy_regVoter (
    .inA(busy_regA),
    .inB(busy_regB),
    .inC(busy_regC),
    .out(busy_reg),
    .tmrErr(busy_regTmrError)
    );

majorityVoter #(.WIDTH(((WB_DATA_WIDTH-1)>(0)) ? ((WB_DATA_WIDTH-1)-(0)+1) : ((0)-(WB_DATA_WIDTH-1)+1))) data_regVoter (
    .inA(data_regA),
    .inB(data_regB),
    .inC(data_regC),
    .out(data_reg),
    .tmrErr(data_regTmrError)
    );

majorityVoter i2c_scl_oVoter (
    .inA(i2c_scl_oA),
    .inB(i2c_scl_oB),
    .inC(i2c_scl_oC),
    .out(i2c_scl_o),
    .tmrErr(i2c_scl_oTmrError)
    );

majorityVoter i2c_scl_tVoter (
    .inA(i2c_scl_tA),
    .inB(i2c_scl_tB),
    .inC(i2c_scl_tC),
    .out(i2c_scl_t),
    .tmrErr(i2c_scl_tTmrError)
    );

majorityVoter i2c_sda_oVoter (
    .inA(i2c_sda_oA),
    .inB(i2c_sda_oB),
    .inC(i2c_sda_oC),
    .out(i2c_sda_o),
    .tmrErr(i2c_sda_oTmrError)
    );

majorityVoter i2c_sda_tVoter (
    .inA(i2c_sda_tA),
    .inB(i2c_sda_tB),
    .inC(i2c_sda_tC),
    .out(i2c_sda_t),
    .tmrErr(i2c_sda_tTmrError)
    );

majorityVoter wb_cyc_o_regVoter (
    .inA(wb_cyc_o_regA),
    .inB(wb_cyc_o_regB),
    .inC(wb_cyc_o_regC),
    .out(wb_cyc_o_reg),
    .tmrErr(wb_cyc_o_regTmrError)
    );

majorityVoter #(.WIDTH(((WB_SELECT_WIDTH-1)>(0)) ? ((WB_SELECT_WIDTH-1)-(0)+1) : ((0)-(WB_SELECT_WIDTH-1)+1))) wb_sel_o_regVoter (
    .inA(wb_sel_o_regA),
    .inB(wb_sel_o_regB),
    .inC(wb_sel_o_regC),
    .out(wb_sel_o_reg),
    .tmrErr(wb_sel_o_regTmrError)
    );

majorityVoter wb_stb_o_regVoter (
    .inA(wb_stb_o_regA),
    .inB(wb_stb_o_regB),
    .inC(wb_stb_o_regC),
    .out(wb_stb_o_reg),
    .tmrErr(wb_stb_o_regTmrError)
    );

majorityVoter wb_we_o_regVoter (
    .inA(wb_we_o_regA),
    .inB(wb_we_o_regB),
    .inC(wb_we_o_regC),
    .out(wb_we_o_reg),
    .tmrErr(wb_we_o_regTmrError)
    );

    /*
fanout bus_addressedFanout (
    .in(bus_addressed),
    .outA(bus_addressedA),
    .outB(bus_addressedB),
    .outC(bus_addressedC)
    );
    */

fanout clkFanout (
    .in(clk),
    .outA(clkA),
    .outB(clkB),
    .outC(clkC)
    );

fanout #(.WIDTH(7)) device_addressFanout (
    .in(device_address),
    .outA(device_addressA),
    .outB(device_addressB),
    .outC(device_addressC)
    );

fanout enableFanout (
    .in(enable),
    .outA(enableA),
    .outB(enableB),
    .outC(enableC)
    );

fanout i2c_scl_iFanout (
    .in(i2c_scl_i),
    .outA(i2c_scl_iA),
    .outB(i2c_scl_iB),
    .outC(i2c_scl_iC)
    );

fanout i2c_sda_iFanout (
    .in(i2c_sda_i),
    .outA(i2c_sda_iA),
    .outB(i2c_sda_iB),
    .outC(i2c_sda_iC)
    );

fanout rstFanout (
    .in(rst),
    .outA(rstA),
    .outB(rstB),
    .outC(rstC)
    );

fanout wb_ack_iFanout (
    .in(wb_ack_i),
    .outA(wb_ack_iA),
    .outB(wb_ack_iB),
    .outC(wb_ack_iC)
    );

fanout #(.WIDTH(((WB_DATA_WIDTH-1)>(0)) ? ((WB_DATA_WIDTH-1)-(0)+1) : ((0)-(WB_DATA_WIDTH-1)+1))) wb_dat_iFanout (
    .in(wb_dat_i),
    .outA(wb_dat_iA),
    .outB(wb_dat_iB),
    .outC(wb_dat_iC)
    );

fanout wb_err_iFanout (
    .in(wb_err_i),
    .outA(wb_err_iA),
    .outB(wb_err_iB),
    .outC(wb_err_iC)
    );
endmodule

