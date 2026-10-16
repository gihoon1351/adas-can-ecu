`timescale 1ns / 1ps

module adder_tree (
    input [16*8*16-1:0] mul_data,  // 2048bit // 16bit씩 8열로 16행
    output [19*16-1 : 0] psum_result  // 304bit
);

  wire [127:0] row_0, row_1, row_2, row_3, row_4, row_5, row_6, row_7, row_8;
  wire [127:0] row_9, row_10, row_11, row_12, row_13, row_14, row_15;

  assign row_0  = mul_data[2047:1920];
  assign row_1  = mul_data[1919:1792];
  assign row_2  = mul_data[1791:1664];
  assign row_3  = mul_data[1663:1536];
  assign row_4  = mul_data[1535:1408];
  assign row_5  = mul_data[1407:1280];
  assign row_6  = mul_data[1279:1152];
  assign row_7  = mul_data[1151:1024];
  assign row_8  = mul_data[1023:896];
  assign row_9  = mul_data[895:768];
  assign row_10 = mul_data[767:640];
  assign row_11 = mul_data[639:512];
  assign row_12 = mul_data[511:384];
  assign row_13 = mul_data[383:256];
  assign row_14 = mul_data[255:128];
  assign row_15 = mul_data[127:0];

  wire [18:0] psum0, psum1, psum2, psum3, psum4, psum5, psum6, psum7, psum8;
  wire [18:0] psum9, psum10, psum11, psum12, psum13, psum14, psum15;


  assign psum0 = $signed(
      row_0[127:112]
  ) + $signed(
      row_0[111:96]
  ) + $signed(
      row_0[95:80]
  ) + $signed(
      row_0[79:64]
  ) + $signed(
      row_0[63:48]
  ) + $signed(
      row_0[47:32]
  ) + $signed(
      row_0[31:16]
  ) + $signed(
      row_0[15:0]
  );

  assign psum1 = $signed(
      row_1[127:112]
  ) + $signed(
      row_1[111:96]
  ) + $signed(
      row_1[95:80]
  ) + $signed(
      row_1[79:64]
  ) + $signed(
      row_1[63:48]
  ) + $signed(
      row_1[47:32]
  ) + $signed(
      row_1[31:16]
  ) + $signed(
      row_1[15:0]
  );

  assign psum2 = $signed(
      row_2[127:112]
  ) + $signed(
      row_2[111:96]
  ) + $signed(
      row_2[95:80]
  ) + $signed(
      row_2[79:64]
  ) + $signed(
      row_2[63:48]
  ) + $signed(
      row_2[47:32]
  ) + $signed(
      row_2[31:16]
  ) + $signed(
      row_2[15:0]
  );

  assign psum3 = $signed(
      row_3[127:112]
  ) + $signed(
      row_3[111:96]
  ) + $signed(
      row_3[95:80]
  ) + $signed(
      row_3[79:64]
  ) + $signed(
      row_3[63:48]
  ) + $signed(
      row_3[47:32]
  ) + $signed(
      row_3[31:16]
  ) + $signed(
      row_3[15:0]
  );

  assign psum4 = $signed(
      row_4[127:112]
  ) + $signed(
      row_4[111:96]
  ) + $signed(
      row_4[95:80]
  ) + $signed(
      row_4[79:64]
  ) + $signed(
      row_4[63:48]
  ) + $signed(
      row_4[47:32]
  ) + $signed(
      row_4[31:16]
  ) + $signed(
      row_4[15:0]
  );

  assign psum5 = $signed(
      row_5[127:112]
  ) + $signed(
      row_5[111:96]
  ) + $signed(
      row_5[95:80]
  ) + $signed(
      row_5[79:64]
  ) + $signed(
      row_5[63:48]
  ) + $signed(
      row_5[47:32]
  ) + $signed(
      row_5[31:16]
  ) + $signed(
      row_5[15:0]
  );

  assign psum6 = $signed(
      row_6[127:112]
  ) + $signed(
      row_6[111:96]
  ) + $signed(
      row_6[95:80]
  ) + $signed(
      row_6[79:64]
  ) + $signed(
      row_6[63:48]
  ) + $signed(
      row_6[47:32]
  ) + $signed(
      row_6[31:16]
  ) + $signed(
      row_6[15:0]
  );

  assign psum7 = $signed(
      row_7[127:112]
  ) + $signed(
      row_7[111:96]
  ) + $signed(
      row_7[95:80]
  ) + $signed(
      row_7[79:64]
  ) + $signed(
      row_7[63:48]
  ) + $signed(
      row_7[47:32]
  ) + $signed(
      row_7[31:16]
  ) + $signed(
      row_7[15:0]
  );

  assign psum8 = $signed(
      row_8[127:112]
  ) + $signed(
      row_8[111:96]
  ) + $signed(
      row_8[95:80]
  ) + $signed(
      row_8[79:64]
  ) + $signed(
      row_8[63:48]
  ) + $signed(
      row_8[47:32]
  ) + $signed(
      row_8[31:16]
  ) + $signed(
      row_8[15:0]
  );

assign psum9 = $signed(
      row_9[127:112]
  ) + $signed(
      row_9[111:96]
  ) + $signed(
      row_9[95:80]
  ) + $signed(
      row_9[79:64]
  ) + $signed(
      row_9[63:48]
  ) + $signed(
      row_9[47:32]
  ) + $signed(
      row_9[31:16]
  ) + $signed(
      row_9[15:0]
  );

  assign psum10 = $signed(
      row_10[127:112]
  ) + $signed(
      row_10[111:96]
  ) + $signed(
      row_10[95:80]
  ) + $signed(
      row_10[79:64]
  ) + $signed(
      row_10[63:48]
  ) + $signed(
      row_10[47:32]
  ) + $signed(
      row_10[31:16]
  ) + $signed(
      row_10[15:0]
  );

  assign psum11 = $signed(
      row_11[127:112]
  ) + $signed(
      row_11[111:96]
  ) + $signed(
      row_11[95:80]
  ) + $signed(
      row_11[79:64]
  ) + $signed(
      row_11[63:48]
  ) + $signed(
      row_11[47:32]
  ) + $signed(
      row_11[31:16]
  ) + $signed(
      row_11[15:0]
  );

  assign psum12 = $signed(
      row_12[127:112]
  ) + $signed(
      row_12[111:96]
  ) + $signed(
      row_12[95:80]
  ) + $signed(
      row_12[79:64]
  ) + $signed(
      row_12[63:48]
  ) + $signed(
      row_12[47:32]
  ) + $signed(
      row_12[31:16]
  ) + $signed(
      row_12[15:0]
  );

  assign psum13 = $signed(
      row_13[127:112]
  ) + $signed(
      row_13[111:96]
  ) + $signed(
      row_13[95:80]
  ) + $signed(
      row_13[79:64]
  ) + $signed(
      row_13[63:48]
  ) + $signed(
      row_13[47:32]
  ) + $signed(
      row_13[31:16]
  ) + $signed(
      row_13[15:0]
  );

  assign psum14 = $signed(
      row_14[127:112]
  ) + $signed(
      row_14[111:96]
  ) + $signed(
      row_14[95:80]
  ) + $signed(
      row_14[79:64]
  ) + $signed(
      row_14[63:48]
  ) + $signed(
      row_14[47:32]
  ) + $signed(
      row_14[31:16]
  ) + $signed(
      row_14[15:0]
  );

  assign psum15 = $signed(
      row_15[127:112]
  ) + $signed(
      row_15[111:96]
  ) + $signed(
      row_15[95:80]
  ) + $signed(
      row_15[79:64]
  ) + $signed(
      row_15[63:48]
  ) + $signed(
      row_15[47:32]
  ) + $signed(
      row_15[31:16]
  ) + $signed(
      row_15[15:0]
  );


  assign psum_result = {
    psum0,
    psum1,
    psum2,
    psum3,
    psum4,
    psum5,
    psum6,
    psum7,
    psum8,
    psum9,
    psum10,
    psum11,
    psum12,
    psum13,
    psum14,
    psum15
  };

endmodule