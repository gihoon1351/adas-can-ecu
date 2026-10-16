`timescale 1ns / 1ps

module alu (
    input [19 * 16-1:0] pipe_in_alu,
    input [32 * 16 -1 : 0] mux_in_alu,
    output [32 * 16 -1 : 0] o_alu
);

  //signed extend from pipe input
  //19bit to 32bit
  wire [31:0] extend_0, extend_1, extend_2, extend_3, extend_4, extend_5, extend_6, extend_7;
  wire [31:0] extend_8, extend_9, extend_10, extend_11, extend_12, extend_13, extend_14, extend_15;
  assign extend_0  = {{13{pipe_in_alu[303]}}, pipe_in_alu[303:285]};
  assign extend_1  = {{13{pipe_in_alu[284]}}, pipe_in_alu[284:266]};
  assign extend_2  = {{13{pipe_in_alu[265]}}, pipe_in_alu[265:247]};
  assign extend_3  = {{13{pipe_in_alu[246]}}, pipe_in_alu[246:228]};
  assign extend_4  = {{13{pipe_in_alu[227]}}, pipe_in_alu[227:209]};
  assign extend_5  = {{13{pipe_in_alu[208]}}, pipe_in_alu[208:190]};
  assign extend_6  = {{13{pipe_in_alu[189]}}, pipe_in_alu[189:171]};
  assign extend_7  = {{13{pipe_in_alu[170]}}, pipe_in_alu[170:152]};
  assign extend_8  = {{13{pipe_in_alu[151]}}, pipe_in_alu[151:133]};
  assign extend_9  = {{13{pipe_in_alu[132]}}, pipe_in_alu[132:114]};
  assign extend_10 = {{13{pipe_in_alu[113]}}, pipe_in_alu[113:95]};
  assign extend_11 = {{13{pipe_in_alu[94]}}, pipe_in_alu[94:76]};
  assign extend_12 = {{13{pipe_in_alu[75]}}, pipe_in_alu[75:57]};
  assign extend_13 = {{13{pipe_in_alu[56]}}, pipe_in_alu[56:38]};
  assign extend_14 = {{13{pipe_in_alu[37]}}, pipe_in_alu[37:19]};
  assign extend_15 = {{13{pipe_in_alu[18]}}, pipe_in_alu[18:0]};



  wire [31:0] mux_in_15, mux_in_14, mux_in_13, mux_in_12, mux_in_11, mux_in_10, mux_in_9, mux_in_8;
  wire [31:0] mux_in_0, mux_in_1, mux_in_2, mux_in_3, mux_in_4, mux_in_5, mux_in_6, mux_in_7;
  assign mux_in_15 = mux_in_alu[31:0];
  assign mux_in_14 = mux_in_alu[63:32];
  assign mux_in_13 = mux_in_alu[95:64];
  assign mux_in_12 = mux_in_alu[127:96];
  assign mux_in_11 = mux_in_alu[159:128];
  assign mux_in_10 = mux_in_alu[191:160];
  assign mux_in_9  = mux_in_alu[223:192];
  assign mux_in_8  = mux_in_alu[255:224];
  assign mux_in_7  = mux_in_alu[287:256];
  assign mux_in_6  = mux_in_alu[319:288];
  assign mux_in_5  = mux_in_alu[351:320];
  assign mux_in_4  = mux_in_alu[383:352];
  assign mux_in_3  = mux_in_alu[415:384];
  assign mux_in_2  = mux_in_alu[447:416];
  assign mux_in_1  = mux_in_alu[479:448];
  assign mux_in_0  = mux_in_alu[511:480];


  wire [31:0] sum_0, sum_1, sum_2, sum_3, sum_4, sum_5, sum_6, sum_7, sum_8;
  wire [31:0] sum_15, sum_14, sum_13, sum_12, sum_11, sum_10, sum_9;

  assign sum_15 = $signed(extend_15) + $signed(mux_in_15);
  assign sum_14 = $signed(extend_14) + $signed(mux_in_14);
  assign sum_13 = $signed(extend_13) + $signed(mux_in_13);
  assign sum_12 = $signed(extend_12) + $signed(mux_in_12);
  assign sum_11 = $signed(extend_11) + $signed(mux_in_11);
  assign sum_10 = $signed(extend_10) + $signed(mux_in_10);
  assign sum_9 = $signed(extend_9) + $signed(mux_in_9);
  assign sum_8 = $signed(extend_8) + $signed(mux_in_8);
  assign sum_7 = $signed(extend_7) + $signed(mux_in_7);
  assign sum_6 = $signed(extend_6) + $signed(mux_in_6);
  assign sum_5 = $signed(extend_5) + $signed(mux_in_5);
  assign sum_4 = $signed(extend_4) + $signed(mux_in_4);
  assign sum_3 = $signed(extend_3) + $signed(mux_in_3);
  assign sum_2 = $signed(extend_2) + $signed(mux_in_2);
  assign sum_1 = $signed(extend_1) + $signed(mux_in_1);
  assign sum_0 = $signed(extend_0) + $signed(mux_in_0);


  assign o_alu = {
    sum_0,
    sum_1,
    sum_2,
    sum_3,
    sum_4,
    sum_5,
    sum_6,
    sum_7,
    sum_8,
    sum_9,
    sum_10,
    sum_11,
    sum_12,
    sum_13,
    sum_14,
    sum_15
  };

endmodule
