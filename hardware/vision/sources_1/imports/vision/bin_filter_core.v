`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/11 09:34:37
// Design Name: 
// Module Name: bin_filter_core
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


module yellow_filter_core(
	input	wire			axis_aclk			,
	input	wire			axis_aresetn		,

	input	wire			cfg_en				,
	input	wire	[15:0]	cfg_thresh_bright	,		// msb: high, lsb: low
	input	wire	[15:0]	cfg_thresh_blue		,
	input	wire	[15:0]	cfg_thresh_red		,

	input	wire	[23:0]	s_axis_tdata		,
	input	wire			s_axis_tvalid		,
	output	wire			s_axis_tready		,
	input	wire			s_axis_tuser		,	//	1: SoF
	input	wire			s_axis_tlast		,	//	1: EoL

	output	reg		[23:0]	m_axis_tdata		,
	output	reg				m_axis_tvalid		,
	input	wire			m_axis_tready		,
	output	reg				m_axis_tuser		,
	output	reg				m_axis_tlast		 
);
	//	Flow Control

	wire		accept;
	wire		sof;

	reg			en_act;
	reg	 [15:0]	thresh_act_bright	;
	reg	 [15:0]	thresh_act_blue		;
	reg	 [15:0]	thresh_act_red		;

	wire		en_use;
	wire [15:0]	thresh_use_bright	;
	wire [15:0]	thresh_use_blue		;
	wire [15:0]	thresh_use_red		;

	wire		above;
	wire		white;
	wire [7:0]	bin;

	wire [7:0]	bright	;
	wire [7:0]	cblue	;
	wire [7:0]	cred	;

	wire		y	;
	wire		cb	;
	wire		cr	;
	wire		bright_yellow;
	wire		shadow_yellow;
	wire		shadow_chroma_relation;

	assign s_axis_tready	= (~m_axis_tvalid) | m_axis_tready;
	assign accept			= s_axis_tvalid & s_axis_tready;	//	handshake
	assign sof				= accept & s_axis_tuser;

	//	shadow -> active latch

	assign	bright	 = s_axis_tdata[23:16];
	assign	cblue	 = s_axis_tdata[15: 8];
	assign	cred	 = s_axis_tdata[ 7: 0];

	assign en_use				= sof ? cfg_en : en_act;
	assign thresh_use_bright	= sof ? cfg_thresh_bright	 : thresh_act_bright	;
	assign thresh_use_blue		= sof ? cfg_thresh_blue		 : thresh_act_blue		;
	assign thresh_use_red		= sof ? cfg_thresh_red		 : thresh_act_red		;

	always @(posedge axis_aclk) begin
		if (!axis_aresetn) begin
			en_act		<= 1'b0;
			thresh_act_bright		<= 16'd0;
			thresh_act_blue			<= 16'd0;
			thresh_act_red			<= 16'd0;
		end else begin
			if (sof) begin
				en_act		<= cfg_en;
				thresh_act_bright		<= cfg_thresh_bright	;
				thresh_act_blue			<= cfg_thresh_blue		;
				thresh_act_red			<= cfg_thresh_red		;
			end
		end
	end

	//	binary filter
	
	/* Keep the programmable bright-yellow path unchanged.  The shadow path
	 * is intentionally narrow and is based on measured lane pixels from the
	 * supplied normal/shadow images. */
	assign bright_yellow = y && cb && cr;
	assign shadow_chroma_relation = {1'b0, cred} >= ({1'b0, cblue} + 9'd15);
	assign shadow_yellow = (bright >= 8'd75)  && (bright <= 8'd125) &&
	                       (cblue >= 8'd95)  && (cblue <= 8'd120) &&
	                       (cred >= 8'd132)  && (cred <= 8'd145) &&
	                       shadow_chroma_relation;
	assign above	= bright_yellow || shadow_yellow;
	assign white	= above;
	assign bin		= white ? 8'hff : 8'h00;

	assign	y		= (bright >= thresh_use_bright[7:0]) && (bright <= thresh_use_bright[15:8]);
	assign	cb		= (cblue  >= thresh_use_blue  [7:0]) && (cblue  <= thresh_use_blue  [15:8]);
	assign	cr		= (cred   >= thresh_use_red   [7:0]) && (cred   <= thresh_use_red   [15:8]);

	always @(posedge axis_aclk) begin
		if (!axis_aresetn) begin
			m_axis_tdata	<= 24'b0;
			m_axis_tvalid	<= 1'b0;
			m_axis_tuser	<= 1'b0;
			m_axis_tlast	<= 1'b0;
		end else begin
			if (accept) begin
				m_axis_tdata	<= en_use ? {bin, bin, bin} : s_axis_tdata;
				m_axis_tvalid	<= 1'b1;
				m_axis_tuser	<= s_axis_tuser;
				m_axis_tlast	<= s_axis_tlast;
			end else if (m_axis_tready) begin
				m_axis_tvalid	<= 1'b0;
			end
		end
	end
endmodule
