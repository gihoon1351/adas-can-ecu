
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/26 13:39:11
// Design Name: 
// Module Name: x_axis_finder
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


module x_axis_finder #(parameter WIDTH = 1280, HEIGHT = 720)(
	input	wire			axis_aclk				,
	input	wire			axis_aresetn			,
	input	wire	[23:0]	s_axis_tdata			,
	input	wire			s_axis_tlast			,		//	eol
	output	wire			s_axis_tready			,
	input	wire			s_axis_tvalid			,
	input	wire			s_axis_tuser			,		//	sof

	input	wire	[10:0]	crop_x_up				,		//	srt
	input	wire	[10:0]	crop_x_dn				,
	input	wire	[10:0]	center_x_up				,
	input	wire	[10:0]	center_x_dn				,

	input	wire	[ 9:0]	up_srt					,
	input	wire	[ 9:0]	dn_srt					,

	input	wire	[ 2:0]	up_size					,
	input	wire	[ 2:0]	dn_size					,

	input	wire			en						,
	output	reg				done					,
	output	reg		[15:0]	x_axis_up				,
	output	reg		[15:0]	x_axis_dn				 
);
	localparam IDLE       		= 4'd0;
	localparam LINE_ACC_UP   	= 4'd1;
	localparam LINE_DONE_UP  	= 4'd2;
	localparam WAIT_START_UP 	= 4'd3;
	localparam FINAL_AVG_UP  	= 4'd4;
	localparam WAIT_DN			= 4'd5;
	localparam LINE_ACC_DN   	= 4'd6;
	localparam LINE_DONE_DN  	= 4'd7;
	localparam WAIT_START_DN 	= 4'd8;
	localparam FINAL_AVG_DN  	= 4'd9;
	localparam SEND       		= 4'd10;

	wire		accept;
	reg	[10:0]	cnt_x;
	reg	[9:0]	cnt_y;

	reg [3:0]	state;
	reg			found_first;
	reg [10:0]	first_x;
	reg [10:0]	last_x;
	reg [19:0]	center_sum_up;
	reg [19:0]	center_sum_dn;
	reg [5:0]	valid_rows_up;
	reg [5:0]	valid_rows_dn;
	wire [35:0] center_scaled_up;
	wire [35:0] center_scaled_dn;
	reg [15:0]	final_center_up;
	reg [15:0]	final_center_dn;
	reg			final_valid_up;
	reg			final_valid_dn;
	wire		sobel_frame_buffer_wdata_x_axis_decoder;
	wire [10:0] up_rows;
	wire [10:0] dn_rows;
	wire [11:0] cfg_x_srt_up;
	wire [11:0] cfg_x_end_up;
	wire [11:0] cfg_x_srt_dn;
	wire [11:0] cfg_x_end_dn;

	reg	[10:0]	y_srt_up		;
	reg	[10:0]	y_end_up		;
	reg	[11:0]	x_srt_up		;
	reg	[11:0]	x_end_up		;
	reg	[11:0]	x_arm_up		;
	reg	[ 2:0]	up_size_r		;

	reg	[10:0]	y_srt_dn		;
	reg	[10:0]	y_end_dn		;
	reg	[11:0]	x_srt_dn		;
	reg	[11:0]	x_end_dn		;
	reg	[11:0]	x_arm_dn		;
	reg	[ 2:0]	dn_size_r		;

	assign accept = s_axis_tready & s_axis_tvalid;
	assign s_axis_tready = 1'b1;
	assign sobel_frame_buffer_wdata_x_axis_decoder = s_axis_tdata[23] & s_axis_tdata[15] & s_axis_tdata[7];
	/* Dual-ROI size code 5 selects the tuned 40-row band. */
	assign up_rows = (up_size == 3'd5) ? 11'd40 : (11'd1 << up_size);
	assign dn_rows = (dn_size == 3'd5) ? 11'd40 : (11'd1 << dn_size);
	/* For the reachable 20-bit sums, floor(sum/80) is exactly
	 * floor((sum*52429)/2^22). This maps to a fast constant multiplier
	 * instead of a long combinational divider. */
	assign center_scaled_up = center_sum_up * 16'd52429;
	assign center_scaled_dn = center_sum_dn * 16'd52429;

	function [11:0] clamp_roi_start;
		input [10:0] center;
		input [10:0] roi_width;
		reg [11:0] center_ext;
		reg [11:0] half_width;
		reg [11:0] max_start;
		begin
			center_ext = {1'b0, center};
			half_width = {1'b0, roi_width} >> 1;
			max_start = WIDTH - {1'b0, roi_width};
			if ((roi_width == 0) || (roi_width >= WIDTH))
				clamp_roi_start = 12'd0;
			else if (center_ext <= half_width)
				clamp_roi_start = 12'd0;
			else if ((center_ext - half_width) > max_start)
				clamp_roi_start = max_start;
			else
				clamp_roi_start = center_ext - half_width;
		end
	endfunction

	assign cfg_x_srt_up = clamp_roi_start(center_x_up, crop_x_up);
	assign cfg_x_end_up = (crop_x_up == 0) ? cfg_x_srt_up :
	                      (cfg_x_srt_up + {1'b0, crop_x_up} - 12'd1);
	assign cfg_x_srt_dn = clamp_roi_start(center_x_dn, crop_x_dn);
	assign cfg_x_end_dn = (crop_x_dn == 0) ? cfg_x_srt_dn :
	                      (cfg_x_srt_dn + {1'b0, crop_x_dn} - 12'd1);

	always @(posedge axis_aclk or negedge axis_aresetn) begin
		if (!axis_aresetn) begin
			cnt_x	<= 11'd0;
			cnt_y	<= 10'd0;
		end else begin
			if (accept) begin
				if (s_axis_tuser) begin
					cnt_y	<= 10'd0;
					cnt_x	<= (s_axis_tlast) ? 11'd0 : 11'd1;
				end else if (s_axis_tlast) begin
					cnt_y	<= cnt_y + 10'd1;
					cnt_x	<= 11'd0;
				end else begin
					cnt_x	<= cnt_x + 11'd1;
				end
			end
		end
	end

	always @(posedge axis_aclk or negedge axis_aresetn) begin
		if (!axis_aresetn) begin
			y_srt_up	<= 0	;
			y_end_up	<= 0	;
			x_srt_up	<= 0	;
			x_end_up	<= 0	;
			x_arm_up	<= 0	;
			y_srt_dn	<= 0	;
			y_end_dn	<= 0	;
			x_srt_dn	<= 0	;
			x_end_dn	<= 0	;
			x_arm_dn	<= 0	;
			up_size_r	<= 0	;
			dn_size_r	<= 0	;
		end else begin
			if (s_axis_tuser && accept) begin
				y_srt_up	<= up_srt;
				y_end_up	<= up_srt + up_rows - 1;
				x_srt_up	<= cfg_x_srt_up;
				x_end_up	<= cfg_x_end_up;
				x_arm_up	<= (cfg_x_srt_up == 0) ? 12'd0 :
				                  (cfg_x_srt_up - 12'd1);
				if ((up_srt + up_rows - 1) >= dn_srt) begin
					y_srt_dn	<= (up_srt + up_rows);
					y_end_dn	<= (up_srt + up_rows) + dn_rows - 1;
					x_srt_dn	<= cfg_x_srt_dn;
					x_end_dn	<= cfg_x_end_dn;
					x_arm_dn	<= (cfg_x_srt_dn == 0) ? 12'd0 :
					                  (cfg_x_srt_dn - 12'd1);
				end else begin
					y_srt_dn	<= dn_srt;
					y_end_dn	<= dn_srt + dn_rows - 1;
					x_srt_dn	<= cfg_x_srt_dn;
					x_end_dn	<= cfg_x_end_dn;
					x_arm_dn	<= (cfg_x_srt_dn == 0) ? 12'd0 :
					                  (cfg_x_srt_dn - 12'd1);
				end
				up_size_r	<= up_size	;
				dn_size_r	<= dn_size	;
			end
		end
	end

	always @(posedge axis_aclk or negedge axis_aresetn) begin
	    if(!axis_aresetn) begin
	        state			<= IDLE;
	        found_first 	<= 1'b0;
	        first_x			<= 0;
	        last_x  		<= 0;
	        center_sum_up	<= 0;
	        center_sum_dn 	<= 0;
	        valid_rows_up	<= 0;
	        valid_rows_dn	<= 0;
	        final_center_up	<= 0;
	        final_center_dn	<= 0;
	        final_valid_up	<= 1'b0;
	        final_valid_dn	<= 1'b0;
			done			<= 0;
			x_axis_up		<= 0;
			x_axis_dn		<= 0;
	    end else begin
			done			<= 1'b0;
				case(state)
				IDLE:
				begin
					if (accept) begin
						found_first		<= 1'b0;
						first_x			<= 0;
						last_x  		<= 0;
						center_sum_up	<= 0;
						valid_rows_up	<= 0;
						if(cnt_y == y_srt_up && cnt_x == x_arm_up && en)
						begin
							state <= LINE_ACC_UP;
						end
					end
				end
				LINE_ACC_UP:
				begin
					if (accept) begin
						if(cnt_x >= x_srt_up &&
						   cnt_x <= x_end_up)
						begin
							if(sobel_frame_buffer_wdata_x_axis_decoder) begin
								if(!found_first) begin
									first_x <= cnt_x;
									found_first <= 1'b1;
								end
								last_x <= cnt_x;
							end
						end
						if(cnt_x == x_end_up)
							state <= LINE_DONE_UP;
					end
				end
				LINE_DONE_UP:
				begin
					if(found_first) begin
						center_sum_up <= center_sum_up
						            + {{9{1'b0}}, first_x}
						            + {{9{1'b0}}, last_x};
						valid_rows_up <= valid_rows_up + 1'b1;
					end else
						center_sum_up <= center_sum_up + (WIDTH);
					found_first <= 1'b0;
					first_x <= 11'd0;
					last_x  <= 11'd0;
					if(cnt_y == y_end_up)
						state <= FINAL_AVG_UP;
					else
						state <= WAIT_START_UP;
				end
				WAIT_START_UP:
				begin
					if (accept) begin
						if(cnt_y >= y_srt_up &&
						   cnt_y <= y_end_up &&
						   cnt_x == x_arm_up)
						begin
							state <= LINE_ACC_UP;
						end
					end
				end
				FINAL_AVG_UP:
				begin
					/* (first + last) / 2, averaged over 128 rows. */
					final_center_up <= (up_size_r == 3'd5) ?
					                   (center_scaled_up >> 22) :
					                   ((center_sum_up >> up_size_r) >> 1);
					final_valid_up <= (up_size_r == 3'd5) ?
					                  (valid_rows_up >= 6'd8) :
					                  (valid_rows_up >= (6'd1 << up_size_r));
					state <= WAIT_DN;
				end
				WAIT_DN:
				begin
					if (accept) begin
						found_first		<= 1'b0;
						first_x			<= 0;
						last_x  		<= 0;
						center_sum_dn	<= 0;
						valid_rows_dn	<= 0;
						if(cnt_y == y_srt_dn && cnt_x == x_arm_dn && en)
						begin
							state <= LINE_ACC_DN;
						end
					end
				end
				LINE_ACC_DN:
				begin
					if (accept) begin
						if(cnt_x >= x_srt_dn &&
						   cnt_x <= x_end_dn)
						begin
							if(sobel_frame_buffer_wdata_x_axis_decoder) begin
								if(!found_first) begin
									first_x <= cnt_x;
									found_first <= 1'b1;
								end
								last_x <= cnt_x;
							end
						end
						if(cnt_x == x_end_dn)
							state <= LINE_DONE_DN;
					end
				end
				LINE_DONE_DN:
				begin
					if(found_first) begin
						center_sum_dn <= center_sum_dn
						            + {{9{1'b0}}, first_x}
						            + {{9{1'b0}}, last_x};
						valid_rows_dn <= valid_rows_dn + 1'b1;
					end else
						center_sum_dn <= center_sum_dn + (WIDTH);
					found_first <= 1'b0;
					first_x <= 11'd0;
					last_x  <= 11'd0;
					if(cnt_y == y_end_dn)
						state <= FINAL_AVG_DN;
					else
						state <= WAIT_START_DN;
				end
				WAIT_START_DN:
				begin
					if (accept) begin
						if(cnt_y >= y_srt_dn &&
						   cnt_y <= y_end_dn &&
						   cnt_x == x_arm_dn)
						begin
							state <= LINE_ACC_DN;
						end
					end
				end
				FINAL_AVG_DN:
				begin
					/* (first + last) / 2, averaged over 128 rows. */
					final_center_dn <= (dn_size_r == 3'd5) ?
					                   (center_scaled_dn >> 22) :
					                   ((center_sum_dn >> dn_size_r) >> 1);
					final_valid_dn <= (dn_size_r == 3'd5) ?
					                  (valid_rows_dn >= 6'd12) :
					                  (valid_rows_dn >= (6'd1 << dn_size_r));
					state <= SEND;
				end
				SEND:
				begin
					/* Bit 15 carries ROI validity; x remains in bits 10:0. */
					x_axis_up <= {final_valid_up, 4'd0, final_center_up[10:0]};
					x_axis_dn <= {final_valid_dn, 4'd0, final_center_dn[10:0]};
					done <= 1'b1;
					state <= IDLE;
				end
				default:
					state <= IDLE;
				endcase
			end
	end
endmodule
