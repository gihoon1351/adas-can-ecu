`timescale 1ns / 1ps

module top_conv (
    input 					clk								,
    input 					resetn							,
    input 					cu_valid_conv					,  // bram으로부터 유효한지 판단
    input [63:0] 			bram_rgb_conv					,
    input [63:0] 			bram_weight0_conv				,
    input [63:0] 			bram_weight1_conv				,
    input [63:0] 			bram_weight2_conv				,
    input [63:0] 			bram_weight3_conv				,
    input [63:0] 			bram_weight4_conv				,
    input [63:0] 			bram_weight5_conv				,
    input [63:0] 			bram_weight6_conv				,
    input [63:0] 			bram_weight7_conv				,
    input [63:0] 			bram_weight8_conv				,
    input [63:0] 			bram_weight9_conv				,
    input [63:0] 			bram_weight10_conv				,
    input [63:0] 			bram_weight11_conv				,
    input [63:0] 			bram_weight12_conv				,
    input [63:0] 			bram_weight13_conv				,
    input [63:0] 			bram_weight14_conv				,
    input [63:0] 			bram_weight15_conv				,
    input 					cu_padding_sel_conv				,
    input 					cu_acc_valid_sel_conv			,
    input 					cu_pipe_clear_conv				,
    input 					cu_buffer_sel_conv				,
    output [32*16-1:0] 		conv_out_ppu					 
);

  localparam MUL_RESULT_W = 16 * 8 * 16;  // 2048
  localparam PSUM_W = 19 * 16;  // 304
  localparam ACC_W = 32 * 16;  // (double buffer) 512

  wire [            63:0] o_mux_mul;
  wire [       ACC_W-1:0] o_mux_alu;

  wire [MUL_RESULT_W-1:0] mul_result;
  wire [MUL_RESULT_W-1:0] mul_pipe_result;
  wire                    mul_pipe_valid;

  wire [      PSUM_W-1:0] add_tree_result;
  wire [      PSUM_W-1:0] psum_pipe_result;
  wire                    psum_pipe_valid;

  wire [ACC_W-1:0] acc_out_mux, acc_out_ppu;


  wire [ACC_W-1:0] o_alu;

  (* max_fanout = 16 *) reg [63:0] mul_ia_q;
  reg [63:0] mul_weight_q [0:15];

  reg operand_valid_q;

  always @(posedge clk) begin
	mul_ia_q <= o_mux_mul;
  	mul_weight_q[0]	<= bram_weight0_conv;
  	mul_weight_q[1]	<= bram_weight1_conv;
  	mul_weight_q[2]	<= bram_weight2_conv;
  	mul_weight_q[3]	<= bram_weight3_conv;
  	mul_weight_q[4]	<= bram_weight4_conv;
  	mul_weight_q[5]	<= bram_weight5_conv;
  	mul_weight_q[6]	<= bram_weight6_conv;
  	mul_weight_q[7]	<= bram_weight7_conv;
  	mul_weight_q[8]	<= bram_weight8_conv;
  	mul_weight_q[9]	<= bram_weight9_conv;
  	mul_weight_q[10]	<= bram_weight10_conv;
  	mul_weight_q[11]	<= bram_weight11_conv;
  	mul_weight_q[12]	<= bram_weight12_conv;
  	mul_weight_q[13]	<= bram_weight13_conv;
  	mul_weight_q[14]	<= bram_weight14_conv;
  	mul_weight_q[15]	<= bram_weight15_conv;
  end

  always @(posedge clk or negedge resetn) begin
	  if (!resetn) begin
		  operand_valid_q <= 1'b0;
	  end else begin
		  operand_valid_q <= cu_valid_conv;
	  end
  end

  mux_mul U_mux_mul (
      .cu_padding_sel_conv(cu_padding_sel_conv),
      .i_mux              (bram_rgb_conv),
      .o_mux              (o_mux_mul)
  );

  multiplier U_multiplier (
      .ia_data      (mul_ia_q),
      .weight0_data (mul_weight_q[0]),
      .weight1_data (mul_weight_q[1]),
      .weight2_data (mul_weight_q[2]),
      .weight3_data (mul_weight_q[3]),
      .weight4_data (mul_weight_q[4]),
      .weight5_data (mul_weight_q[5]),
      .weight6_data (mul_weight_q[6]),
      .weight7_data (mul_weight_q[7]),
      .weight8_data (mul_weight_q[8]),
      .weight9_data (mul_weight_q[9]),
      .weight10_data(mul_weight_q[10]),
      .weight11_data(mul_weight_q[11]),
      .weight12_data(mul_weight_q[12]),
      .weight13_data(mul_weight_q[13]),
      .weight14_data(mul_weight_q[14]),
      .weight15_data(mul_weight_q[15]),
      .mul_result   (mul_result)
  );


  mul_pipe_line_add U_mul_pipe_line_add (
      .clk               (clk),
      .resetn            (resetn),
      .cu_pipe_clear_conv(cu_pipe_clear_conv),

      .i_valid(operand_valid_q),
      .i_pipe (mul_result),

      .o_valid(mul_pipe_valid),
      .o_pipe (mul_pipe_result)
  );


  adder_tree U_adder_tree (
      .mul_data   (mul_pipe_result),
      .psum_result(add_tree_result)
  );


  add_pipe_line_alu U_add_pipe_line_alu (
      .clk               (clk),
      .resetn            (resetn),
      .cu_pipe_clear_conv(cu_pipe_clear_conv),

      .i_valid(mul_pipe_valid),
      .i_pipe (add_tree_result),

      .o_valid(psum_pipe_valid),
      .o_pipe (psum_pipe_result)
  );

  mux_alu U_mux_alu (
      //   .clk(clk),
      //   .resetn(resetn),
      .cu_acc_valid_sel_conv(cu_acc_valid_sel_conv),
      //   .cu_pipe_clear_conv(cu_pipe_clear_conv),
      .i_mux(acc_out_mux),
      .o_mux(o_mux_alu)
  );

  alu U_alu (
      .pipe_in_alu(psum_pipe_result),
      .mux_in_alu(o_mux_alu),
      .o_alu(o_alu)
  );

  acc_buffer U_acc_buffer (
      .clk(clk),
      .resetn(resetn),
      .psum_pipe_valid(psum_pipe_valid),
      //   .cu_pipe_clear_conv(cu_pipe_clear_conv),
      .in_acc_buffer(o_alu),  // 512bit
      .cu_buffer_sel_conv(cu_buffer_sel_conv),
      .acc_out_ppu(acc_out_ppu),  // 512bit
      .acc_out_mux(acc_out_mux)
  );

  assign conv_out_ppu = acc_out_ppu;


endmodule
