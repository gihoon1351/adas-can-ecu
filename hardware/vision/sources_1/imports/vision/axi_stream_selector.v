`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/29 14:35:45
// Design Name: 
// Module Name: axi_stream_selector
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


module axi_stream_selector(
	input	wire				s_axis_0_aclk		,
	input	wire				s_axis_0_aresetn	,
	input	wire	[23:0]		s_axis_0_tdata		,
	input	wire				s_axis_0_tlast		,
	input	wire				s_axis_0_tvalid		,
	output	wire				s_axis_0_tready		,
	input	wire				s_axis_0_tuser		,

	input	wire				s_axis_1_aclk		,
	input	wire				s_axis_1_aresetn	,
	input	wire	[23:0]		s_axis_1_tdata		,
	input	wire				s_axis_1_tlast		,
	input	wire				s_axis_1_tvalid		,
	output	wire				s_axis_1_tready		,
	input	wire				s_axis_1_tuser		,

	input	wire				s_axis_select		,

	input	wire				m_axis_aclk			,
	input	wire				m_axis_aresetn		,
	output	wire	[23:0]		m_axis_tdata		,
	output	wire				m_axis_tlast		,
	output	wire				m_axis_tvalid		,
	input	wire				m_axis_tready		,
	output	wire				m_axis_tuser		 
);
	assign s_axis_0_tready = m_axis_tready;
	assign s_axis_1_tready = m_axis_tready;

	assign m_axis_tdata 	= s_axis_select ? s_axis_0_tdata : s_axis_1_tdata;
	assign m_axis_tlast 	= s_axis_select ? s_axis_0_tlast : s_axis_1_tlast;
	assign m_axis_tvalid	= s_axis_select ? s_axis_0_tvalid : s_axis_1_tvalid;
	assign m_axis_tuser		= s_axis_select ? s_axis_0_tuser : s_axis_1_tuser;


endmodule
