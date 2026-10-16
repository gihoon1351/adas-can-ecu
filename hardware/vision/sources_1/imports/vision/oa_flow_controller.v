`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/22 09:47:59
// Design Name: 
// Module Name: oa_flow_controller
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


module oa_flow_controller(
	input	wire			clk				,
	input	wire			resetn			,
	input	wire	[31:0]	oa				,
	input	wire			o_valid			,
	output	wire			o_ready			,
	output	wire	[31:0]	m_axis_tdata	,
	output	wire			m_axis_tvalid	,
	input	wire			m_axis_tready	,
	output	wire			m_axis_tuser	,
	output	wire			m_axis_tlast	 
);
	reg	[8:0]	out_cnt;

	assign m_axis_tvalid	= o_valid;
	assign m_axis_tdata		= oa;
	assign o_ready			= m_axis_tready;
	assign m_axis_tuser		= 1'b0;
	assign m_axis_tlast		= m_axis_tvalid && (out_cnt == 9'd391);

	always @(posedge clk or negedge resetn) begin
		if (!resetn) begin
			out_cnt	<= 9'd0;
		end else begin
			if (o_valid && o_ready) begin
				if (out_cnt == 9'd391) begin
					out_cnt	<= 9'd0;
				end else begin
					out_cnt	<= out_cnt + 9'd1;
				end
			end
		end
	end
endmodule
