`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/19 16:05:23
// Design Name: 
// Module Name: memory
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


module bram #(parameter BW = 64, DEPTH = 1024, ADDR_BW = $clog2(DEPTH))(
	input	wire					clk				,
	input	wire					we				,
	input	wire	[ADDR_BW-1:0]	waddr			,
	input	wire	[ADDR_BW-1:0]	raddr			,
	input	wire	[BW-1:0]		wdata			,
	output	wire	[BW-1:0]		rdata			 
);
	reg	[BW-1:0]	ram[0:DEPTH-1];
	reg	[BW-1:0]	rdata_r;

	assign rdata = rdata_r;

	always @(posedge clk) begin
		if (we) begin
			ram[waddr] <= wdata;
		end
	end

	always @(posedge clk) begin
		rdata_r	<= ram[raddr];
	end
endmodule
