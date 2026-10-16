`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/22 10:11:00
// Design Name: 
// Module Name: cnn_stream
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


module cnn_stream (
    input wire [31:0] s_axis_param_tdata,
    input wire [3:0] s_axis_param_tkeep,
    input wire s_axis_param_tvalid,
    output wire s_axis_param_tready,
    input wire s_axis_param_tlast,

    input wire cr_param_abort,
    output wire sr_param_loaded,
    output wire sr_param_loading,
    output wire sr_param_error,

	input	wire					clk					,
	input	wire					resetn				,
	input	wire	[23:0]			s_axis_lb_tdata		,
	input	wire					s_axis_lb_tvalid	,
	output	wire					s_axis_lb_tready	,
	input	wire					s_axis_lb_tuser		,
	input	wire					s_axis_lb_tlast		,
	input	wire	[31:0]			s_axis_w_tdata		,
	input	wire					s_axis_w_tvalid		,
	output	wire					s_axis_w_tready		,
	input	wire					s_axis_w_tuser		,
	input	wire					s_axis_w_tlast		,
	output	wire	[31:0]			m_axis_oa_tdata		,
	output	wire					m_axis_oa_tvalid	,
	input	wire					m_axis_oa_tready	,
	output	wire	[3:0]			m_axis_oa_tkeep		,
	output	wire					m_axis_oa_tuser		,
	output	wire					m_axis_oa_tlast		,
	input	wire					cr_srt				,
	output	wire	[5:0]			sr_in_channel_group	,
	output	wire	[4:0]			sr_out_channel_group,
	output	wire	[2:0]			sr_layer			,
	output	wire					sr_busy				,
	output	wire					sr_idle				,
	output	wire					irq					 
);

    wire param_bias_we, param_scale_we;
    wire [6:0] param_waddr;
    wire [3:0] param_wlane;
    wire [31:0] param_wdata;
    wire cnn_start;
    // Parameter traffic has priority over a simultaneous START request.
    // START keeps its original level-sensitive semantics; it is not queued.
    assign cnn_start = cr_srt && sr_param_loaded && !sr_param_loading
                       && !sr_param_error && !s_axis_param_tvalid
                       && !cr_param_abort;
    param_stream_loader U_PARAM_LOADER (
        .clk(clk), .resetn(resetn), .cnn_idle(sr_idle),
        .abort_load(cr_param_abort),
        .s_axis_tdata(s_axis_param_tdata),
        .s_axis_tkeep(s_axis_param_tkeep),
        .s_axis_tvalid(s_axis_param_tvalid),
        .s_axis_tready(s_axis_param_tready),
        .s_axis_tlast(s_axis_param_tlast),
        .bias_we(param_bias_we), .scale_we(param_scale_we),
        .wr_addr(param_waddr), .wr_lane(param_wlane), .wr_data(param_wdata),
        .loaded(sr_param_loaded), .loading(sr_param_loading), .error(sr_param_error)
    );

	wire					w_word_vld		;
	wire					w_word_rdy		;

	wire	[31:0]			oa				;
	wire					o_ready			;
	wire					o_valid			;

	wire					lb_tvalid		;
	wire					lb_tready		;
	wire	[7:0]			lb_raddr		;
	wire	[7:0]			lb_waddr		;
	wire	[1:0]			lb_sel_in		;
	wire	[1:0]			lb_sel_out		;
	wire					lb_we			;
	wire	[63:0]			lb_output		;


	wire	[63:0]			weight_in_0		;
	wire	[63:0]			weight_in_1		;
	wire	[63:0]			weight_in_2		;
	wire	[63:0]			weight_in_3		;
	wire	[63:0]			weight_in_4		;
	wire	[63:0]			weight_in_5		;
	wire	[63:0]			weight_in_6		;
	wire	[63:0]			weight_in_7		;
	wire	[63:0]			weight_in_8		;
	wire	[63:0]			weight_in_9		;
	wire	[63:0]			weight_in_10	;
	wire	[63:0]			weight_in_11	;
	wire	[63:0]			weight_in_12	;
	wire	[63:0]			weight_in_13	;
	wire	[63:0]			weight_in_14	;
	wire	[63:0]			weight_in_15	;

	wire					done			;

	assign m_axis_oa_tkeep = 4'b1111;
	assign irq				= done;

	cnn_top U_CNN_TOP(
        .param_bias_we(param_bias_we),
        .param_scale_we(param_scale_we),
        .param_waddr(param_waddr),
        .param_wlane(param_wlane),
        .param_wdata(param_wdata),

		.clk					(clk					),
		.resetn					(resetn					),
		.lb_output				(lb_output				),
		.lb_tvalid				(lb_tvalid				),
		.o_ready				(o_ready				),
		.w_word_vld				(w_word_vld				),
		.w_wdata_0				(weight_in_0			),
		.w_wdata_1				(weight_in_1			),
		.w_wdata_2				(weight_in_2			),
		.w_wdata_3				(weight_in_3			),
		.w_wdata_4				(weight_in_4			),
		.w_wdata_5				(weight_in_5			),
		.w_wdata_6				(weight_in_6			),
		.w_wdata_7				(weight_in_7			),
		.w_wdata_8				(weight_in_8			),
		.w_wdata_9				(weight_in_9			),
		.w_wdata_10				(weight_in_10			),
		.w_wdata_11				(weight_in_11			),
		.w_wdata_12				(weight_in_12			),
		.w_wdata_13				(weight_in_13			),
		.w_wdata_14				(weight_in_14			),
		.w_wdata_15				(weight_in_15			),
		.done					(done					),
		.lb_raddr				(lb_raddr				),
		.lb_waddr				(lb_waddr				),
		.lb_sel_in				(lb_sel_in				),
		.lb_sel_out				(lb_sel_out				),
		.lb_tready				(lb_tready				),
		.lb_we					(lb_we					),
		.o_valid				(o_valid				),
		.w_word_rdy				(w_word_rdy				),
		.cr_srt(cnn_start),
		.sr_in_channel_group	(sr_in_channel_group	),
		.sr_out_channel_group	(sr_out_channel_group	),
		.sr_layer				(sr_layer				),
		.sr_busy				(sr_busy				),
		.sr_idle				(sr_idle				),
		.oa						(oa						) 
	);

	vdma_linebuffer U_VDMA_LB (
		.axis_aclk			(clk				),
		.axis_aresetn		(resetn				),
		.lb_raddr			(lb_raddr			),
		.lb_waddr			(lb_waddr			),
		.lb_sel_in			(lb_sel_in			),
		.lb_sel_out			(lb_sel_out			),
		.we					(lb_we				),
		.tready				(lb_tready			),
		.lb_tvalid			(lb_tvalid			),
		.s_axis_tdata		(s_axis_lb_tdata	),
		.s_axis_tvalid		(s_axis_lb_tvalid	),
		.s_axis_tready		(s_axis_lb_tready	),
		.s_axis_tuser		(s_axis_lb_tuser	),
		.s_axis_tlast		(s_axis_lb_tlast	),
		.lb_output			(lb_output			) 
);

	weight_flow_controller U_W_FC (
		.axis_aclk		(clk					),
		.axis_aresetn	(resetn					),
		.w_word_rdy		(w_word_rdy				),
		.w_word_vld		(w_word_vld				),
		.s_axis_tdata	(s_axis_w_tdata			),  //weight 8bit x 4 set
		.s_axis_tvalid	(s_axis_w_tvalid		),
		.s_axis_tready	(s_axis_w_tready		),
		.s_axis_tuser	(s_axis_w_tuser			),
		.s_axis_tlast	(s_axis_w_tlast			),
		.weight_in_0	(weight_in_0			),
		.weight_in_1	(weight_in_1			),
		.weight_in_2	(weight_in_2			),
		.weight_in_3	(weight_in_3			),
		.weight_in_4	(weight_in_4			),
		.weight_in_5	(weight_in_5			),
		.weight_in_6	(weight_in_6			),
		.weight_in_7	(weight_in_7			),
		.weight_in_8	(weight_in_8			),
		.weight_in_9	(weight_in_9			),
		.weight_in_10	(weight_in_10			),
		.weight_in_11	(weight_in_11			),
		.weight_in_12	(weight_in_12			),
		.weight_in_13	(weight_in_13			),
		.weight_in_14	(weight_in_14			),
		.weight_in_15	(weight_in_15			) 
	);

	oa_flow_controller U_OA_FC(
		.clk			(clk				),
		.resetn			(resetn				),
		.oa				(oa					),
		.o_valid		(o_valid			),
		.o_ready		(o_ready			),
		.m_axis_tdata	(m_axis_oa_tdata	),
		.m_axis_tvalid	(m_axis_oa_tvalid	),
		.m_axis_tready	(m_axis_oa_tready	),
		.m_axis_tuser	(m_axis_oa_tuser	),
		.m_axis_tlast	(m_axis_oa_tlast	) 
	);
endmodule
