`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/19 14:15:41
// Design Name: 
// Module Name: cnn_top
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


module cnn_top #(parameter BW = 64) (
    input wire param_bias_we,
    input wire param_scale_we,
    input wire [6:0] param_waddr,
    input wire [3:0] param_wlane,
    input wire [31:0] param_wdata,

	input	wire					clk					,
	input	wire					resetn				,
	input	wire	[63:0]			lb_output			,

	input	wire					lb_tvalid			,
	input	wire					o_ready				,
	input	wire					w_word_vld			,

	input	wire	[63:0]			w_wdata_0			,
	input	wire	[63:0]			w_wdata_1			,
	input	wire	[63:0]			w_wdata_2			,
	input	wire	[63:0]			w_wdata_3			,
	input	wire	[63:0]			w_wdata_4			,
	input	wire	[63:0]			w_wdata_5			,
	input	wire	[63:0]			w_wdata_6			,
	input	wire	[63:0]			w_wdata_7			,
	input	wire	[63:0]			w_wdata_8			,
	input	wire	[63:0]			w_wdata_9			,
	input	wire	[63:0]			w_wdata_10			,
	input	wire	[63:0]			w_wdata_11			,
	input	wire	[63:0]			w_wdata_12			,
	input	wire	[63:0]			w_wdata_13			,
	input	wire	[63:0]			w_wdata_14			,
	input	wire	[63:0]			w_wdata_15			,

	output	wire					done				,
	output	wire	[7:0]			lb_raddr			,
	output	wire	[7:0]			lb_waddr			,
	output	wire	[1:0]			lb_sel_in			,
	output	wire	[1:0]			lb_sel_out			,
	output	wire					lb_tready			,
	output	wire					lb_we				,

	output	wire					o_valid				,
	output	wire					w_word_rdy			,

	input	wire					cr_srt				,
	output	wire	[5:0]			sr_in_channel_group	,
	output	wire	[4:0]			sr_out_channel_group,
	output	wire	[2:0]			sr_layer			,
	output	wire					sr_busy				,
	output	wire					sr_idle				,

	output	wire	[31:0]			oa					 
);
	wire			a_we					;
	wire	[14:0]	a_waddr					;
	wire	[14:0]	a_raddr					;
	wire	[63:0]	a_rdata					;
	wire			b_we					;
	wire	[13:0]	b_waddr					;
	wire	[13:0]	b_raddr					;
	wire			w_we					;
	wire	[ 8:0]	w_waddr					;
	wire	[ 8:0]	w_raddr					;
	wire	[63:0]	b_rdata					;
	wire	[63:0]	dp_output				;
	wire	[ 1:0]	dp_input_sel			;
	wire	[63:0]	bram_rgb_conv			;
	wire	[63:0]	w_rdata_0				;
	wire	[63:0]	w_rdata_1				;
	wire	[63:0]	w_rdata_2				;
	wire	[63:0]	w_rdata_3				;
	wire	[63:0]	w_rdata_4				;
	wire	[63:0]	w_rdata_5				;
	wire	[63:0]	w_rdata_6				;
	wire	[63:0]	w_rdata_7				;
	wire	[63:0]	w_rdata_8				;
	wire	[63:0]	w_rdata_9				;
	wire	[63:0]	w_rdata_10				;
	wire	[63:0]	w_rdata_11				;
	wire	[63:0]	w_rdata_12				;
	wire	[63:0]	w_rdata_13				;
	wire	[63:0]	w_rdata_14				;
	wire	[63:0]	w_rdata_15				;
	wire			cu_valid_conv			;
	wire			cu_padding_sel_conv		;
	wire			cu_acc_valid_sel_conv	;
	wire			cu_pipe_clear_conv		;
	wire			cu_buffer_sel_conv		;
	wire			conv_valid_ppu			;
	wire	[4:0]	ppu_channel_group_idx	;
	wire	[2:0]	ppu_layer_idx			;
	wire			out_data_mux_sel		;
	wire			ppu_ready_conv			;
	wire			ppu_out_valid			;
	wire			oa_half_sel				;

	bram #(.BW(64), .DEPTH(25088))
	U_AMEM (
		.clk	(clk		),
		.we		(a_we		),
		.waddr	(a_waddr	),
		.raddr	(a_raddr	),
		.wdata	(dp_output	),
		.rdata	(a_rdata	) 
	);

	bram #(.BW(64), .DEPTH(12544))
	U_BMEM (
		.clk	(clk		),
		.we		(b_we		),
		.waddr	(b_waddr	),
		.raddr	(b_raddr	),
		.wdata	(dp_output	),
		.rdata	(b_rdata	) 
	);

	mux_4to1 U_DP_INPUT_MUX (
		.in0(lb_output		),
		.in1(a_rdata		),
		.in2(b_rdata		),
		.in3(64'd0			),
		.sel(dp_input_sel	),
		.out(bram_rgb_conv	) 
	);

	wmem_top U_WMEM_TOP(
		.clk						(clk		),
		.w_we						(w_we		),
		.w_raddr					(w_raddr	),
		.w_waddr					(w_waddr	),
		.weight0_input_conv			(w_wdata_0	),
		.weight1_input_conv			(w_wdata_1	),
		.weight2_input_conv			(w_wdata_2	),
		.weight3_input_conv			(w_wdata_3	),
		.weight4_input_conv			(w_wdata_4	),
		.weight5_input_conv			(w_wdata_5	),
		.weight6_input_conv			(w_wdata_6	),
		.weight7_input_conv			(w_wdata_7	),
		.weight8_input_conv			(w_wdata_8	),
		.weight9_input_conv			(w_wdata_9	),
		.weight10_input_conv		(w_wdata_10	),
		.weight11_input_conv		(w_wdata_11	),
		.weight12_input_conv		(w_wdata_12	),
		.weight13_input_conv		(w_wdata_13	),
		.weight14_input_conv		(w_wdata_14	),
		.weight15_input_conv		(w_wdata_15	),
		.weight0_output_conv		(w_rdata_0	),
		.weight1_output_conv		(w_rdata_1	),
		.weight2_output_conv		(w_rdata_2	),
		.weight3_output_conv		(w_rdata_3	),
		.weight4_output_conv		(w_rdata_4	),
		.weight5_output_conv		(w_rdata_5	),
		.weight6_output_conv		(w_rdata_6	),
		.weight7_output_conv		(w_rdata_7	),
		.weight8_output_conv		(w_rdata_8	),
		.weight9_output_conv		(w_rdata_9	),
		.weight10_output_conv		(w_rdata_10	),
		.weight11_output_conv		(w_rdata_11	),
		.weight12_output_conv		(w_rdata_12	),
		.weight13_output_conv		(w_rdata_13	),
		.weight14_output_conv		(w_rdata_14	),
		.weight15_output_conv		(w_rdata_15	) 
	);


	datapath U_DP(
        .param_bias_we(param_bias_we),
        .param_scale_we(param_scale_we),
        .param_waddr(param_waddr),
        .param_wlane(param_wlane),
        .param_wdata(param_wdata),

		.clk							(clk					),
		.resetn							(resetn					),
		.cu_valid_conv					(cu_valid_conv			),
		.bram_rgb_conv					(bram_rgb_conv			),
		.bram_weight0_conv				(w_rdata_0				),
		.bram_weight1_conv				(w_rdata_1				),
		.bram_weight2_conv				(w_rdata_2				),
		.bram_weight3_conv				(w_rdata_3				),
		.bram_weight4_conv				(w_rdata_4				),
		.bram_weight5_conv				(w_rdata_5				),
		.bram_weight6_conv				(w_rdata_6				),
		.bram_weight7_conv				(w_rdata_7				),
		.bram_weight8_conv				(w_rdata_8				),
		.bram_weight9_conv				(w_rdata_9				),
		.bram_weight10_conv				(w_rdata_10				),
		.bram_weight11_conv				(w_rdata_11				),
		.bram_weight12_conv				(w_rdata_12				),
		.bram_weight13_conv				(w_rdata_13				),
		.bram_weight14_conv				(w_rdata_14				),
		.bram_weight15_conv				(w_rdata_15				),
		.cu_padding_sel_conv			(cu_padding_sel_conv	),
		.cu_acc_valid_sel_conv			(cu_acc_valid_sel_conv	),
		.cu_pipe_clear_conv				(cu_pipe_clear_conv		),
		.cu_buffer_sel_conv				(cu_buffer_sel_conv		),
		.conv_valid_ppu					(conv_valid_ppu			),
		.ppu_channel_group_idx			(ppu_channel_group_idx	),
		.ppu_layer_idx					(ppu_layer_idx			),
		.out_data_mux_sel				(out_data_mux_sel		),
		.ppu_ready_conv					(ppu_ready_conv			),
		.ppu_out_data					(dp_output				),
		.ppu_out_valid					(ppu_out_valid			),
		.pool_line_start				(),
		.pool_line_end					() 
);

	control_unit U_CU(
		.clk					(clk					),
		.resetn					(resetn					),
		.lb_tvalid				(lb_tvalid				),
		.w_word_vld				(w_word_vld				),
		.o_ready				(o_ready				),
		.ppu_ready_conv			(ppu_ready_conv			),
		.ppu_out_vld			(ppu_out_valid			),
		.lb_waddr				(lb_waddr				),
		.lb_raddr				(lb_raddr				),
		.lb_sel_in				(lb_sel_in				),
		.lb_sel_out				(lb_sel_out				),
		.lb_we					(lb_we					),
		.lb_tready				(lb_tready				),
		.w_word_rdy				(w_word_rdy				),
		.a_we					(a_we					),
		.a_waddr				(a_waddr				),
		.a_raddr				(a_raddr				),
		.b_we					(b_we					),
		.b_waddr				(b_waddr				),
		.b_raddr				(b_raddr				),
		.w_we					(w_we					),
		.w_waddr				(w_waddr				),
		.w_raddr				(w_raddr				),
		.dp_input_sel			(dp_input_sel			),
		.cu_valid_conv			(cu_valid_conv			),
		.cu_padding_sel_conv	(cu_padding_sel_conv	),
		.cu_pipe_clear_conv		(cu_pipe_clear_conv		),
		.cu_acc_vld_sel_conv	(cu_acc_valid_sel_conv	),
		.cu_buffer_sel_conv		(cu_buffer_sel_conv		),
		.conv_valid_ppu			(conv_valid_ppu			),
		.ppu_channel_group_idx	(ppu_channel_group_idx	),
		.ppu_layer_idx			(ppu_layer_idx			),
		.out_data_mux_sel		(out_data_mux_sel		),
		.oa_half_sel			(oa_half_sel			),
		.o_valid				(o_valid				),
		.cr_srt					(cr_srt					),
		.sr_in_channel_group	(sr_in_channel_group	),
		.sr_out_channel_group	(sr_out_channel_group	),
		.sr_layer				(sr_layer				),
		.sr_busy				(sr_busy				),
		.sr_idle				(sr_idle				),
		.done					(done					) 
	);

	mux_2to1 U_OA_MUX (
		.in0(a_rdata[63:32]),
		.in1(a_rdata[31:0]),
		.sel(oa_half_sel),
		.out(oa) 
	);
endmodule

module mux_4to1 (
	input	wire	[63:0]	in0,
	input	wire	[63:0]	in1,
	input	wire	[63:0]	in2,
	input	wire	[63:0]	in3,
	input	wire	[ 1:0]	sel,
	output	wire	[63:0]	out 
);
	reg	[63:0]	out_r;

	assign out = out_r;

	always @(*) begin
		case (sel)
			2'b00: out_r = in0;
			2'b01: out_r = in1;
			2'b10: out_r = in2;
			2'b11: out_r = in3;
			default:	out_r = 0;
		endcase
	end
endmodule

module mux_2to1 (
	input	wire	[31:0]	in0,
	input	wire	[31:0]	in1,
	input	wire			sel,
	output	wire	[31:0]	out 
);
	reg	[31:0]	out_r;

	assign out = out_r;

	always @(*) begin
		case (sel)
			1'b0: out_r = in0;
			1'b1: out_r = in1;
			default:	out_r = 0;
		endcase
	end
endmodule
