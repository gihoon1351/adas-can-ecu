`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/19 19:56:29
// Design Name: 
// Module Name: datapath
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


module datapath(
    input wire param_bias_we,
    input wire param_scale_we,
    input wire [6:0] param_waddr,
    input wire [3:0] param_wlane,
    input wire [31:0] param_wdata,

	input	wire			clk								,
	input	wire			resetn							,
    input 					cu_valid_conv					,
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
    input  wire         	conv_valid_ppu					,
    input  wire [4:0]   	ppu_channel_group_idx			,
    input  wire [2:0]   	ppu_layer_idx					,
    input  wire         	out_data_mux_sel				,
    output wire         	ppu_ready_conv					,
    output wire [63:0]  	ppu_out_data					,
    output wire         	ppu_out_valid					,
    output wire         	pool_line_start					,
    output wire         	pool_line_end					 
);

	wire	[32*16-1:0]	conv_out_ppu;

	top_conv U_CONV (
		.clk							(clk					),
		.resetn							(resetn					),
		.cu_valid_conv					(cu_valid_conv			),
		.bram_rgb_conv					(bram_rgb_conv			),
		.bram_weight0_conv				(bram_weight0_conv		),
		.bram_weight1_conv				(bram_weight1_conv		),
		.bram_weight2_conv				(bram_weight2_conv		),
		.bram_weight3_conv				(bram_weight3_conv		),
		.bram_weight4_conv				(bram_weight4_conv		),
		.bram_weight5_conv				(bram_weight5_conv		),
		.bram_weight6_conv				(bram_weight6_conv		),
		.bram_weight7_conv				(bram_weight7_conv		),
		.bram_weight8_conv				(bram_weight8_conv		),
		.bram_weight9_conv				(bram_weight9_conv		),
		.bram_weight10_conv				(bram_weight10_conv		),
		.bram_weight11_conv				(bram_weight11_conv		),
		.bram_weight12_conv				(bram_weight12_conv		),
		.bram_weight13_conv				(bram_weight13_conv		),
		.bram_weight14_conv				(bram_weight14_conv		),
		.bram_weight15_conv				(bram_weight15_conv		),
		.cu_padding_sel_conv			(cu_padding_sel_conv	),
		.cu_acc_valid_sel_conv			(cu_acc_valid_sel_conv	),
		.cu_pipe_clear_conv				(cu_pipe_clear_conv		),
		.cu_buffer_sel_conv				(cu_buffer_sel_conv		),
		.conv_out_ppu					(conv_out_ppu			) 
	);

	ppu_top U_PPU (
        .param_bias_we(param_bias_we),
        .param_scale_we(param_scale_we),
        .param_waddr(param_waddr),
        .param_wlane(param_wlane),
        .param_wdata(param_wdata),

		.clk							(clk					),
		.rst_n							(resetn					),
		.conv_out_ppu					(conv_out_ppu			),
		.conv_valid_ppu					(conv_valid_ppu			),
		.ppu_channel_group_idx			(ppu_channel_group_idx	),
		.ppu_layer_idx					(ppu_layer_idx			),
		.out_data_mux_sel				(out_data_mux_sel		),
		.ppu_ready_conv					(ppu_ready_conv			),
		.ppu_out_data					(ppu_out_data			),
		.ppu_out_valid					(ppu_out_valid			),
		.pool_line_start				(pool_line_start		),
		.pool_line_end					(pool_line_end			) 
	);
endmodule
