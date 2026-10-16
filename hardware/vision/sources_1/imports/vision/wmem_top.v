`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/20 12:00:52
// Design Name: 
// Module Name: wmem_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module wmem_top(
	input	wire				clk							,
	input	wire				w_we						,
	input	wire	[ 8:0]		w_raddr						,
	input	wire	[ 8:0]		w_waddr						,
	input	wire	[63:0]		weight0_input_conv			,
	input	wire	[63:0]		weight1_input_conv			,
	input	wire	[63:0]		weight2_input_conv			,
	input	wire	[63:0]		weight3_input_conv			,
	input	wire	[63:0]		weight4_input_conv			,
	input	wire	[63:0]		weight5_input_conv			,
	input	wire	[63:0]		weight6_input_conv			,
	input	wire	[63:0]		weight7_input_conv			,
	input	wire	[63:0]		weight8_input_conv			,
	input	wire	[63:0]		weight9_input_conv			,
	input	wire	[63:0]		weight10_input_conv			,
	input	wire	[63:0]		weight11_input_conv			,
	input	wire	[63:0]		weight12_input_conv			,
	input	wire	[63:0]		weight13_input_conv			,
	input	wire	[63:0]		weight14_input_conv			,
	input	wire	[63:0]		weight15_input_conv			,

	output	wire	[63:0]		weight0_output_conv			,
	output	wire	[63:0]		weight1_output_conv			,
	output	wire	[63:0]		weight2_output_conv			,
	output	wire	[63:0]		weight3_output_conv			,
	output	wire	[63:0]		weight4_output_conv			,
	output	wire	[63:0]		weight5_output_conv			,
	output	wire	[63:0]		weight6_output_conv			,
	output	wire	[63:0]		weight7_output_conv			,
	output	wire	[63:0]		weight8_output_conv			,
	output	wire	[63:0]		weight9_output_conv			,
	output	wire	[63:0]		weight10_output_conv		,
	output	wire	[63:0]		weight11_output_conv		,
	output	wire	[63:0]		weight12_output_conv		,
	output	wire	[63:0]		weight13_output_conv		,
	output	wire	[63:0]		weight14_output_conv		,
	output	wire	[63:0]		weight15_output_conv		 
);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_0 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight0_input_conv),
		.rdata	(weight0_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_1 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight1_input_conv),
		.rdata	(weight1_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_2 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight2_input_conv),
		.rdata	(weight2_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_3 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight3_input_conv),
		.rdata	(weight3_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_4 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight4_input_conv),
		.rdata	(weight4_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_5 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight5_input_conv),
		.rdata	(weight5_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_6 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight6_input_conv),
		.rdata	(weight6_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_7 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight7_input_conv),
		.rdata	(weight7_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_8 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight8_input_conv),
		.rdata	(weight8_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_9 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight9_input_conv),
		.rdata	(weight9_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_10 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight10_input_conv),
		.rdata	(weight10_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_11 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight11_input_conv),
		.rdata	(weight11_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_12 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight12_input_conv),
		.rdata	(weight12_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_13 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight13_input_conv),
		.rdata	(weight13_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_14 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight14_input_conv),
		.rdata	(weight14_output_conv)
	);
	bram #(.BW(64), .DEPTH(288))
	U_WMEM_15 (
		.clk	(clk),
		.we		(w_we),
		.waddr	(w_waddr),
		.raddr	(w_raddr),
		.wdata	(weight15_input_conv),
		.rdata	(weight15_output_conv)
	);
endmodule
