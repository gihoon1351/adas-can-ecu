`timescale 1ns / 1ps

(* use_dsp = "yes" *)
module multiplier (
    input [63:0] ia_data,
    input [63:0] weight0_data,
    input [63:0] weight1_data,
    input [63:0] weight2_data,
    input [63:0] weight3_data,
    input [63:0] weight4_data,
    input [63:0] weight5_data,
    input [63:0] weight6_data,
    input [63:0] weight7_data,
    input [63:0] weight8_data,
    input [63:0] weight9_data,
    input [63:0] weight10_data,
    input [63:0] weight11_data,
    input [63:0] weight12_data,
    input [63:0] weight13_data,
    input [63:0] weight14_data,
    input [63:0] weight15_data,
    output [16 * 8 * 16 - 1 : 0] mul_result  // 2048bit
);

  wire [15:0] mul_0_0, mul_0_1, mul_0_2, mul_0_3, mul_0_4, mul_0_5, mul_0_6, mul_0_7;
  wire [15:0] mul_1_0, mul_1_1, mul_1_2, mul_1_3, mul_1_4, mul_1_5, mul_1_6, mul_1_7;
  wire [15:0] mul_2_0, mul_2_1, mul_2_2, mul_2_3, mul_2_4, mul_2_5, mul_2_6, mul_2_7;
  wire [15:0] mul_3_0, mul_3_1, mul_3_2, mul_3_3, mul_3_4, mul_3_5, mul_3_6, mul_3_7;
  wire [15:0] mul_4_0, mul_4_1, mul_4_2, mul_4_3, mul_4_4, mul_4_5, mul_4_6, mul_4_7;
  wire [15:0] mul_5_0, mul_5_1, mul_5_2, mul_5_3, mul_5_4, mul_5_5, mul_5_6, mul_5_7;
  wire [15:0] mul_6_0, mul_6_1, mul_6_2, mul_6_3, mul_6_4, mul_6_5, mul_6_6, mul_6_7;
  wire [15:0] mul_7_0, mul_7_1, mul_7_2, mul_7_3, mul_7_4, mul_7_5, mul_7_6, mul_7_7;
  wire [15:0] mul_8_0, mul_8_1, mul_8_2, mul_8_3, mul_8_4, mul_8_5, mul_8_6, mul_8_7;
  wire [15:0] mul_9_0, mul_9_1, mul_9_2, mul_9_3, mul_9_4, mul_9_5, mul_9_6, mul_9_7;
  wire [15:0] mul_10_0, mul_10_1, mul_10_2, mul_10_3, mul_10_4, mul_10_5, mul_10_6, mul_10_7;
  wire [15:0] mul_11_0, mul_11_1, mul_11_2, mul_11_3, mul_11_4, mul_11_5, mul_11_6, mul_11_7;
  wire [15:0] mul_12_0, mul_12_1, mul_12_2, mul_12_3, mul_12_4, mul_12_5, mul_12_6, mul_12_7;
  wire [15:0] mul_13_0, mul_13_1, mul_13_2, mul_13_3, mul_13_4, mul_13_5, mul_13_6, mul_13_7;
  wire [15:0] mul_14_0, mul_14_1, mul_14_2, mul_14_3, mul_14_4, mul_14_5, mul_14_6, mul_14_7;
  wire [15:0] mul_15_0, mul_15_1, mul_15_2, mul_15_3, mul_15_4, mul_15_5, mul_15_6, mul_15_7;

  wire [16 * 8 - 1:0] row_0, row_1, row_2, row_3, row_4, row_5, row_6, row_7, row_8;
  wire [16 * 8 - 1:0] row_9, row_10, row_11, row_12, row_13, row_14, row_15;

  assign mul_result = { //output
    row_0,
    row_1,
    row_2,
    row_3,
    row_4,
    row_5,
    row_6,
    row_7,
    row_8,
    row_9,
    row_10,
    row_11,
    row_12,
    row_13,
    row_14,
    row_15
  };

  // 0행
  assign mul_0_0 = $signed({ia_data[63:56]}) * $signed(weight0_data[63:56]);
  assign mul_0_1 = $signed({ia_data[55:48]}) * $signed(weight0_data[55:48]);
  assign mul_0_2 = $signed({ia_data[47:40]}) * $signed(weight0_data[47:40]);
  assign mul_0_3 = $signed({ia_data[39:32]}) * $signed(weight0_data[39:32]);
  assign mul_0_4 = $signed({ia_data[31:24]}) * $signed(weight0_data[31:24]);
  assign mul_0_5 = $signed({ia_data[23:16]}) * $signed(weight0_data[23:16]);
  assign mul_0_6 = $signed({ia_data[15:8]	}) * $signed(weight0_data[15:8]);
  assign mul_0_7 = $signed({ia_data[7:0]	}) * $signed(weight0_data[7:0]);

  assign row_0 = {mul_0_0, mul_0_1, mul_0_2, mul_0_3, mul_0_4, mul_0_5, mul_0_6, mul_0_7};

  // 1행
  assign mul_1_0 = $signed({ia_data[63:56]}) * $signed(weight1_data[63:56]);
  assign mul_1_1 = $signed({ia_data[55:48]}) * $signed(weight1_data[55:48]);
  assign mul_1_2 = $signed({ia_data[47:40]}) * $signed(weight1_data[47:40]);
  assign mul_1_3 = $signed({ia_data[39:32]}) * $signed(weight1_data[39:32]);
  assign mul_1_4 = $signed({ia_data[31:24]}) * $signed(weight1_data[31:24]);
  assign mul_1_5 = $signed({ia_data[23:16]}) * $signed(weight1_data[23:16]);
  assign mul_1_6 = $signed({ia_data[15:8]	}) * $signed(weight1_data[15:8]);
  assign mul_1_7 = $signed({ia_data[7:0]	}) * $signed(weight1_data[7:0]);

  assign row_1 = {mul_1_0, mul_1_1, mul_1_2, mul_1_3, mul_1_4, mul_1_5, mul_1_6, mul_1_7};

  // 2행
  assign mul_2_0 = $signed({ia_data[63:56]}) * $signed(weight2_data[63:56]);
  assign mul_2_1 = $signed({ia_data[55:48]}) * $signed(weight2_data[55:48]);
  assign mul_2_2 = $signed({ia_data[47:40]}) * $signed(weight2_data[47:40]);
  assign mul_2_3 = $signed({ia_data[39:32]}) * $signed(weight2_data[39:32]);
  assign mul_2_4 = $signed({ia_data[31:24]}) * $signed(weight2_data[31:24]);
  assign mul_2_5 = $signed({ia_data[23:16]}) * $signed(weight2_data[23:16]);
  assign mul_2_6 = $signed({ia_data[15:8]	}) * $signed(weight2_data[15:8]);
  assign mul_2_7 = $signed({ia_data[7:0]	}) * $signed(weight2_data[7:0]);

  assign row_2 = {mul_2_0, mul_2_1, mul_2_2, mul_2_3, mul_2_4, mul_2_5, mul_2_6, mul_2_7};

  // 3행
  assign mul_3_0 = $signed({ia_data[63:56]}) * $signed(weight3_data[63:56]);
  assign mul_3_1 = $signed({ia_data[55:48]}) * $signed(weight3_data[55:48]);
  assign mul_3_2 = $signed({ia_data[47:40]}) * $signed(weight3_data[47:40]);
  assign mul_3_3 = $signed({ia_data[39:32]}) * $signed(weight3_data[39:32]);
  assign mul_3_4 = $signed({ia_data[31:24]}) * $signed(weight3_data[31:24]);
  assign mul_3_5 = $signed({ia_data[23:16]}) * $signed(weight3_data[23:16]);
  assign mul_3_6 = $signed({ia_data[15:8]	}) * $signed(weight3_data[15:8]);
  assign mul_3_7 = $signed({ia_data[7:0]	}) * $signed(weight3_data[7:0]);

  assign row_3 = {mul_3_0, mul_3_1, mul_3_2, mul_3_3, mul_3_4, mul_3_5, mul_3_6, mul_3_7};

  // 4행
  assign mul_4_0 = $signed({ia_data[63:56]}) * $signed(weight4_data[63:56]);
  assign mul_4_1 = $signed({ia_data[55:48]}) * $signed(weight4_data[55:48]);
  assign mul_4_2 = $signed({ia_data[47:40]}) * $signed(weight4_data[47:40]);
  assign mul_4_3 = $signed({ia_data[39:32]}) * $signed(weight4_data[39:32]);
  assign mul_4_4 = $signed({ia_data[31:24]}) * $signed(weight4_data[31:24]);
  assign mul_4_5 = $signed({ia_data[23:16]}) * $signed(weight4_data[23:16]);
  assign mul_4_6 = $signed({ia_data[15:8]	}) * $signed(weight4_data[15:8]);
  assign mul_4_7 = $signed({ia_data[7:0]	}) * $signed(weight4_data[7:0]);

  assign row_4 = {mul_4_0, mul_4_1, mul_4_2, mul_4_3, mul_4_4, mul_4_5, mul_4_6, mul_4_7};


  // 5행
  assign mul_5_0 = $signed({ia_data[63:56]}) * $signed(weight5_data[63:56]);
  assign mul_5_1 = $signed({ia_data[55:48]}) * $signed(weight5_data[55:48]);
  assign mul_5_2 = $signed({ia_data[47:40]}) * $signed(weight5_data[47:40]);
  assign mul_5_3 = $signed({ia_data[39:32]}) * $signed(weight5_data[39:32]);
  assign mul_5_4 = $signed({ia_data[31:24]}) * $signed(weight5_data[31:24]);
  assign mul_5_5 = $signed({ia_data[23:16]}) * $signed(weight5_data[23:16]);
  assign mul_5_6 = $signed({ia_data[15:8] }) * $signed(weight5_data[15:8]);
  assign mul_5_7 = $signed({ia_data[7:0]  }) * $signed(weight5_data[7:0]);

  assign row_5 = {mul_5_0, mul_5_1, mul_5_2, mul_5_3, mul_5_4, mul_5_5, mul_5_6, mul_5_7};

  // 6행
  assign mul_6_0 = $signed({ia_data[63:56]}) * $signed(weight6_data[63:56]);
  assign mul_6_1 = $signed({ia_data[55:48]}) * $signed(weight6_data[55:48]);
  assign mul_6_2 = $signed({ia_data[47:40]}) * $signed(weight6_data[47:40]);
  assign mul_6_3 = $signed({ia_data[39:32]}) * $signed(weight6_data[39:32]);
  assign mul_6_4 = $signed({ia_data[31:24]}) * $signed(weight6_data[31:24]);
  assign mul_6_5 = $signed({ia_data[23:16]}) * $signed(weight6_data[23:16]);
  assign mul_6_6 = $signed({ia_data[15:8] }) * $signed(weight6_data[15:8]);
  assign mul_6_7 = $signed({ia_data[7:0]  }) * $signed(weight6_data[7:0]);

  assign row_6 = {mul_6_0, mul_6_1, mul_6_2, mul_6_3, mul_6_4, mul_6_5, mul_6_6, mul_6_7};

  // 7행
  assign mul_7_0 = $signed({ia_data[63:56]}) * $signed(weight7_data[63:56]);
  assign mul_7_1 = $signed({ia_data[55:48]}) * $signed(weight7_data[55:48]);
  assign mul_7_2 = $signed({ia_data[47:40]}) * $signed(weight7_data[47:40]);
  assign mul_7_3 = $signed({ia_data[39:32]}) * $signed(weight7_data[39:32]);
  assign mul_7_4 = $signed({ia_data[31:24]}) * $signed(weight7_data[31:24]);
  assign mul_7_5 = $signed({ia_data[23:16]}) * $signed(weight7_data[23:16]);
  assign mul_7_6 = $signed({ia_data[15:8] }) * $signed(weight7_data[15:8]);
  assign mul_7_7 = $signed({ia_data[7:0]  }) * $signed(weight7_data[7:0]);

  assign row_7 = {mul_7_0, mul_7_1, mul_7_2, mul_7_3, mul_7_4, mul_7_5, mul_7_6, mul_7_7};

  // 8행
  assign mul_8_0 = $signed({ia_data[63:56]}) * $signed(weight8_data[63:56]);
  assign mul_8_1 = $signed({ia_data[55:48]}) * $signed(weight8_data[55:48]);
  assign mul_8_2 = $signed({ia_data[47:40]}) * $signed(weight8_data[47:40]);
  assign mul_8_3 = $signed({ia_data[39:32]}) * $signed(weight8_data[39:32]);
  assign mul_8_4 = $signed({ia_data[31:24]}) * $signed(weight8_data[31:24]);
  assign mul_8_5 = $signed({ia_data[23:16]}) * $signed(weight8_data[23:16]);
  assign mul_8_6 = $signed({ia_data[15:8] }) * $signed(weight8_data[15:8]);
  assign mul_8_7 = $signed({ia_data[7:0]  }) * $signed(weight8_data[7:0]);

  assign row_8 = {mul_8_0, mul_8_1, mul_8_2, mul_8_3, mul_8_4, mul_8_5, mul_8_6, mul_8_7};

  // 9행
  assign mul_9_0 = $signed({ia_data[63:56]}) * $signed(weight9_data[63:56]);
  assign mul_9_1 = $signed({ia_data[55:48]}) * $signed(weight9_data[55:48]);
  assign mul_9_2 = $signed({ia_data[47:40]}) * $signed(weight9_data[47:40]);
  assign mul_9_3 = $signed({ia_data[39:32]}) * $signed(weight9_data[39:32]);
  assign mul_9_4 = $signed({ia_data[31:24]}) * $signed(weight9_data[31:24]);
  assign mul_9_5 = $signed({ia_data[23:16]}) * $signed(weight9_data[23:16]);
  assign mul_9_6 = $signed({ia_data[15:8] }) * $signed(weight9_data[15:8]);
  assign mul_9_7 = $signed({ia_data[7:0]  }) * $signed(weight9_data[7:0]);

  assign row_9 = {mul_9_0, mul_9_1, mul_9_2, mul_9_3, mul_9_4, mul_9_5, mul_9_6, mul_9_7};

  // 10행
  assign mul_10_0 = $signed({ia_data[63:56]}) * $signed(weight10_data[63:56]);
  assign mul_10_1 = $signed({ia_data[55:48]}) * $signed(weight10_data[55:48]);
  assign mul_10_2 = $signed({ia_data[47:40]}) * $signed(weight10_data[47:40]);
  assign mul_10_3 = $signed({ia_data[39:32]}) * $signed(weight10_data[39:32]);
  assign mul_10_4 = $signed({ia_data[31:24]}) * $signed(weight10_data[31:24]);
  assign mul_10_5 = $signed({ia_data[23:16]}) * $signed(weight10_data[23:16]);
  assign mul_10_6 = $signed({ia_data[15:8] }) * $signed(weight10_data[15:8]);
  assign mul_10_7 = $signed({ia_data[7:0]  }) * $signed(weight10_data[7:0]);

  assign row_10 = {mul_10_0, mul_10_1, mul_10_2, mul_10_3, mul_10_4, mul_10_5, mul_10_6, mul_10_7};

  // 11행
  assign mul_11_0 = $signed({ia_data[63:56]}) * $signed(weight11_data[63:56]);
  assign mul_11_1 = $signed({ia_data[55:48]}) * $signed(weight11_data[55:48]);
  assign mul_11_2 = $signed({ia_data[47:40]}) * $signed(weight11_data[47:40]);
  assign mul_11_3 = $signed({ia_data[39:32]}) * $signed(weight11_data[39:32]);
  assign mul_11_4 = $signed({ia_data[31:24]}) * $signed(weight11_data[31:24]);
  assign mul_11_5 = $signed({ia_data[23:16]}) * $signed(weight11_data[23:16]);
  assign mul_11_6 = $signed({ia_data[15:8] }) * $signed(weight11_data[15:8]);
  assign mul_11_7 = $signed({ia_data[7:0]  }) * $signed(weight11_data[7:0]);

  assign row_11 = {mul_11_0, mul_11_1, mul_11_2, mul_11_3, mul_11_4, mul_11_5, mul_11_6, mul_11_7};

  // 12행
  assign mul_12_0 = $signed({ia_data[63:56]}) * $signed(weight12_data[63:56]);
  assign mul_12_1 = $signed({ia_data[55:48]}) * $signed(weight12_data[55:48]);
  assign mul_12_2 = $signed({ia_data[47:40]}) * $signed(weight12_data[47:40]);
  assign mul_12_3 = $signed({ia_data[39:32]}) * $signed(weight12_data[39:32]);
  assign mul_12_4 = $signed({ia_data[31:24]}) * $signed(weight12_data[31:24]);
  assign mul_12_5 = $signed({ia_data[23:16]}) * $signed(weight12_data[23:16]);
  assign mul_12_6 = $signed({ia_data[15:8] }) * $signed(weight12_data[15:8]);
  assign mul_12_7 = $signed({ia_data[7:0]  }) * $signed(weight12_data[7:0]);

  assign row_12 = {mul_12_0, mul_12_1, mul_12_2, mul_12_3, mul_12_4, mul_12_5, mul_12_6, mul_12_7};

  // 13행
  assign mul_13_0 = $signed({ia_data[63:56]}) * $signed(weight13_data[63:56]);
  assign mul_13_1 = $signed({ia_data[55:48]}) * $signed(weight13_data[55:48]);
  assign mul_13_2 = $signed({ia_data[47:40]}) * $signed(weight13_data[47:40]);
  assign mul_13_3 = $signed({ia_data[39:32]}) * $signed(weight13_data[39:32]);
  assign mul_13_4 = $signed({ia_data[31:24]}) * $signed(weight13_data[31:24]);
  assign mul_13_5 = $signed({ia_data[23:16]}) * $signed(weight13_data[23:16]);
  assign mul_13_6 = $signed({ia_data[15:8] }) * $signed(weight13_data[15:8]);
  assign mul_13_7 = $signed({ia_data[7:0]  }) * $signed(weight13_data[7:0]);

  assign row_13 = {mul_13_0, mul_13_1, mul_13_2, mul_13_3, mul_13_4, mul_13_5, mul_13_6, mul_13_7};

  // 14행
  assign mul_14_0 = $signed({ia_data[63:56]}) * $signed(weight14_data[63:56]);
  assign mul_14_1 = $signed({ia_data[55:48]}) * $signed(weight14_data[55:48]);
  assign mul_14_2 = $signed({ia_data[47:40]}) * $signed(weight14_data[47:40]);
  assign mul_14_3 = $signed({ia_data[39:32]}) * $signed(weight14_data[39:32]);
  assign mul_14_4 = $signed({ia_data[31:24]}) * $signed(weight14_data[31:24]);
  assign mul_14_5 = $signed({ia_data[23:16]}) * $signed(weight14_data[23:16]);
  assign mul_14_6 = $signed({ia_data[15:8] }) * $signed(weight14_data[15:8]);
  assign mul_14_7 = $signed({ia_data[7:0]  }) * $signed(weight14_data[7:0]);

  assign row_14 = {mul_14_0, mul_14_1, mul_14_2, mul_14_3, mul_14_4, mul_14_5, mul_14_6, mul_14_7};


  // 15행
  assign mul_15_0 = $signed({ia_data[63:56]}) * $signed(weight15_data[63:56]);
  assign mul_15_1 = $signed({ia_data[55:48]}) * $signed(weight15_data[55:48]);
  assign mul_15_2 = $signed({ia_data[47:40]}) * $signed(weight15_data[47:40]);
  assign mul_15_3 = $signed({ia_data[39:32]}) * $signed(weight15_data[39:32]);
  assign mul_15_4 = $signed({ia_data[31:24]}) * $signed(weight15_data[31:24]);
  assign mul_15_5 = $signed({ia_data[23:16]}) * $signed(weight15_data[23:16]);
  assign mul_15_6 = $signed({ia_data[15:8] }) * $signed(weight15_data[15:8]);
  assign mul_15_7 = $signed({ia_data[7:0]  }) * $signed(weight15_data[7:0]);

  assign row_15 = {mul_15_0, mul_15_1, mul_15_2, mul_15_3, mul_15_4, mul_15_5, mul_15_6, mul_15_7};

endmodule
