`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/09/19 19:33:03
// Design Name: 
// Module Name: control_unit
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


module control_unit(
	input	wire				clk						,
	input	wire				resetn					,
	input	wire				lb_tvalid				,
	input	wire				w_word_vld				,
	input	wire				o_ready					,
	input	wire				ppu_ready_conv			,
	input	wire				ppu_out_vld				,

	output	wire	[7:0]		lb_waddr				,
	output	reg		[7:0]		lb_raddr				,
	output	wire	[1:0]		lb_sel_in				,
	output	reg		[1:0]		lb_sel_out				,
	output	wire				lb_we					,
	output	wire				lb_tready				,
	output	wire				w_word_rdy				,
	output	wire				a_we					,
	output	reg		[14:0]		a_waddr					,
	output	wire	[14:0]		a_raddr					,
	output	wire				b_we					,
	output	reg		[13:0]		b_waddr					,
	output	wire	[13:0]		b_raddr					,
	output	wire				w_we					,
	output	reg		[ 8:0]		w_waddr					,
	output	wire	[ 8:0]		w_raddr					,
	output	reg		[1:0]		dp_input_sel			,
	output	wire				cu_valid_conv			,
	output	wire				cu_padding_sel_conv		,
	output	wire				cu_pipe_clear_conv		,
	output	wire				cu_acc_vld_sel_conv		,
	output	wire				cu_buffer_sel_conv		,
	output	wire				conv_valid_ppu			,
	output	reg		[4:0]		ppu_channel_group_idx	,
	output	reg		[2:0]		ppu_layer_idx			,
	output	wire				out_data_mux_sel		,
	output	wire				oa_half_sel				,
	output	wire				o_valid					,

	input	wire				cr_srt					,
	output	wire	[5:0]		sr_in_channel_group		,
	output	wire	[4:0]		sr_out_channel_group	,
	output	wire	[2:0]		sr_layer				,
	output	wire				sr_busy					,
	output	wire				sr_idle					,

	output	wire				done					 
);

	localparam
		L0_WH				= 224,
		L1_WH				= 112,
		L2_WH				=  56,
		L3_WH				=  28,
		L4_WH				=  14,
		L5_WH				=   7,
		L6_WH				=   7,
		L7_WH				=   7,
		L8_WH				=   7;

	localparam
		L0_C				=   3,
		L1_C				=  16,
		L2_C				=  32,
		L3_C				=  64,
		L4_C				= 128,
		L5_C				= 256,
		L6_C				= 512,
		L7_C				= 512,
		L8_C				=  30;

	localparam
		L0_RS				=   3,
		L1_RS				= 	3,
		L2_RS				= 	3,
		L3_RS				= 	3,
		L4_RS				= 	3,
		L5_RS				= 	3,
		L6_RS				= 	3,
		L7_RS				= 	3,
		L8_RS				= 	1;

	localparam
		L0_INPUT_GROUP		=  1,
		L1_INPUT_GROUP		=  2,
		L2_INPUT_GROUP		=  4,
		L3_INPUT_GROUP		=  8,
		L4_INPUT_GROUP		= 16,
		L5_INPUT_GROUP		= 32,
		L6_INPUT_GROUP		= 64,
		L7_INPUT_GROUP		= 64;

	localparam
		M_IDLE              = 0,
		M_L0_LAYER_PREP     = 1,
		M_L0_WAIT_LOAD      = 2,
		M_L0_RUN_LAYER      = 3,
		M_LAYER_PREP        = 4,
		M_WAIT_LOAD         = 5,
		M_RUN_LAYER         = 6,
		M_NEXT_LAYER        = 7,
		M_OA_STORE          = 8,
		M_DONE              = 9;

	localparam
		W_IDLE              = 0,
		W_LOAD              = 1,
		W_DONE              = 2;

	localparam
		IMG_IDLE            = 0,
		IMG_LOAD            = 1,
		IMG_WAIT            = 2,
		IMG_DONE            = 3;

	localparam
		DP_IDLE             = 0,
		DP_CNN              = 1,
		DP_DRAIN            = 2,
		DP_DONE             = 3;

	localparam
		FM_IDLE             = 0,
		FM_STORE 	        = 1;

	localparam
		OA_IDLE             = 0,
		OA_READ				= 1,
		OA_SEND             = 2,
		OA_DONE             = 3;

	reg	[8:0]	weight_word_count[0:6];
	reg	[5:0]	weight_load_count[0:6];

	reg	[3:0]	ma_cstate, ma_nstate;
	reg	[1:0]	we_cstate, we_nstate;
	reg	[1:0]	im_cstate, im_nstate;
	reg	[1:0]	dp_cstate, dp_nstate;
	reg			fm_cstate, fm_nstate;
	reg	[1:0]	oa_cstate, oa_nstate;

	reg	[7:0]			img_x_cnt       			;
	reg	[7:0]			img_row_cnt     			;
	reg	[1:0]			img_bank_sel    			;
	reg	[2:0]			layer						;
	reg	[5:0]			channel_in_group_cnt		;
	reg	[6:0]			channel_in_group			;
	reg	[4:0]			channel_out_group			;
	reg	[7:0]			x_in						;
	reg	[7:0]			y_in						;
	reg	[7:0]			conv_width					;
	reg	[7:0]			conv_height					;
	reg	[1:0]			r							;
	reg	[1:0]			s							;
	reg	[1:0]			kernel_size					;
	reg					cu_padding_sel_conv_d1		;
	reg					cu_padding_sel_conv_d2		;
	reg					cu_vld_conv_d1				;
	reg					cu_vld_conv_d2				;
	reg					cu_acc_vld_sel_conv_d1		;
	reg					cu_acc_vld_sel_conv_d2		;
	reg					cu_acc_vld_sel_conv_d3		;
	reg					cu_acc_vld_sel_conv_d4		;
	reg					cu_acc_vld_sel_conv_d5		;
	reg					cu_buffer_sel_conv_d1		;
	reg					cu_buffer_sel_conv_d2		;
	reg					cu_buffer_sel_conv_d3		;
	reg					cu_buffer_sel_conv_d4		;
	reg					cu_buffer_sel_conv_d5		;
	reg					accumulation_bank			;
	reg					conv_vld_ppu_d1 			;
	reg					conv_vld_ppu_d2 			;
	reg					conv_vld_ppu_d3 			;
	reg					conv_vld_ppu_d4				;
	reg					conv_vld_ppu_d5				;
	reg					conv_vld_ppu_d6				;
	reg	[7:0]			oa_raddr					;
	reg					oa_half_sel_q				;
	wire				layer0_line_done    		;
	wire				last_out_group				;
	wire				weight_srt					;
	wire				weight_done					;
	wire				weight_done_ack				;
	wire				img_srt						;
	wire				img_window_rdy				;
	wire				img_done					;
	wire				img_done_ack				;
	wire				dp_srt						;
	wire				dp_done						;
	wire				dp_done_ack					;
	wire				last_cnn_input				;
	wire				last_channel_in_group		;
	wire				last_kernel					;
	wire				dp_issue					;
	wire				cu_padding_sel_conv_now		;
	wire				cu_acc_vld_sel_conv_now		;
	wire				conv_vld_ppu_now			;
	wire				fm_done						;
	wire				oa_srt						;
	wire				oa_done						;
	wire				oa_done_ack					;
	wire				oa_last						;
	wire signed [8:0]	src_x						;
	wire signed [8:0]	src_y						;
	wire 		[4:0]	group16						;
	reg	[14:0]			pixel_addr					;
	reg	[14:0]			group_base					;
	reg	[14:0]			feature_raddr				;
	reg	[13:0]			dp_b_raddr					;
	wire		[8:0]	src_y_mod3					;

	reg			[7:0]	lb_raddr_next				;
	reg			[1:0]	lb_sel_out_next				;
	reg			[14:0]	a_raddr_next				;
	reg			[13:0]	dp_b_raddr_next				;
	reg			[8:0]	w_raddr_next				;

	reg			[14:0]	a_raddr_q					;
	reg			[13:0]	dp_b_raddr_q				;
	reg			[8:0]	w_raddr_q					;

	initial begin
		weight_word_count[0]	= 10'd9;
		weight_word_count[1]	= 10'd18;
		weight_word_count[2]	= 10'd36;
		weight_word_count[3]	= 10'd72;
		weight_word_count[4]	= 10'd144;
		weight_word_count[5]	= 10'd288;
		weight_word_count[6]	= 10'd64;
	end

	initial begin
		weight_load_count[0] = 6'd1;
		weight_load_count[1] = 6'd2;
		weight_load_count[2] = 6'd4;
		weight_load_count[3] = 6'd8;
		weight_load_count[4] = 6'd16;
		weight_load_count[5] = 6'd32;
		weight_load_count[6] = 6'd2;
	end

	//	axi register

	assign sr_in_channel_group	= channel_in_group_cnt;
	assign sr_out_channel_group	= channel_out_group;
	assign sr_layer				= layer;
	assign sr_busy				= (ma_cstate != M_IDLE);
	assign sr_idle				= (ma_cstate == M_IDLE);

	//	Combinational Logic

	//	main FSM

	assign weight_srt	= ((ma_cstate == M_L0_LAYER_PREP) || (ma_cstate == M_LAYER_PREP));
	assign img_srt		= (ma_cstate == M_L0_LAYER_PREP);
	assign dp_srt		= ((ma_cstate == M_RUN_LAYER) || (ma_cstate == M_L0_RUN_LAYER));
	assign oa_srt		= (ma_cstate == M_OA_STORE);

	assign weight_done_ack	= (((ma_cstate == M_L0_WAIT_LOAD) && (weight_done && img_window_rdy)) || ((ma_cstate == M_WAIT_LOAD) && weight_done));
	assign img_done_ack		= ((ma_cstate == M_L0_RUN_LAYER) && (dp_done && img_done));
	assign dp_done_ack		= (((ma_cstate == M_L0_RUN_LAYER) && (dp_done && img_done)) || ((ma_cstate == M_RUN_LAYER) && dp_done));
	assign oa_done_ack		= ((ma_cstate == M_OA_STORE) && oa_done);

	assign last_out_group	= ({1'b0, channel_out_group} == (weight_load_count[layer] - 6'd1));

	assign done				= (ma_cstate == M_DONE);

	//	weight FSM

	assign w_we			= ((we_cstate == W_LOAD) && w_word_vld);
	assign weight_done	= (we_cstate == W_DONE);
	assign w_word_rdy	= (we_cstate == W_LOAD);

	//	fm FSM

	assign a_we				= resetn && ((ppu_out_vld || fm_cstate == FM_STORE) && (!layer[0]));
	assign b_we				= resetn && ((ppu_out_vld || fm_cstate == FM_STORE) && (layer[0]));
	assign out_data_mux_sel	= fm_cstate;
	assign fm_done			= ((fm_cstate == FM_STORE) && (a_we || b_we));

	//	oa FSM

	assign o_valid		= (oa_cstate == OA_SEND);
	assign oa_done		= (oa_cstate == OA_DONE);
	assign oa_last		= ((oa_raddr == 8'd195) && (oa_half_sel_q));
	assign oa_half_sel	= oa_half_sel_q;

	//	img FSM

	assign lb_tready		= (im_cstate == IMG_LOAD);
	assign lb_we			= lb_tready && lb_tvalid;
	assign lb_waddr			= img_x_cnt;
	assign lb_sel_in		= img_bank_sel;
	assign img_window_rdy	= ((im_cstate == IMG_WAIT) || (im_cstate == IMG_DONE));
	assign img_done			= (im_cstate == IMG_DONE);

	//	dp FSM

	assign b_raddr		= dp_b_raddr_q;
	assign w_raddr		= w_raddr_q;
	assign a_raddr		= (ma_cstate == M_OA_STORE) ? {7'd0, oa_raddr} : a_raddr_q;
	assign cu_pipe_clear_conv		= 1'b0;
	assign dp_done					= (dp_cstate == DP_DONE);
	assign last_kernel				= ((kernel_size - 2'b01) == r) && ((kernel_size - 2'b01) == s);
	assign last_channel_in_group	= (channel_in_group_cnt == (channel_in_group - 7'd1));
	assign dp_issue					= (dp_cstate == DP_CNN) && ((layer != 3'b000) || img_window_rdy);
	assign cu_padding_sel_conv_now	= !((kernel_size != 1) ? ((x_in == 0 && r == 0) || (x_in == conv_width - 8'd1 && r == kernel_size - 1) || (y_in == 0 && s == 0) || (y_in == conv_height - 1 && s == kernel_size - 1)) : 0);
	assign cu_acc_vld_sel_conv_now	= dp_issue && !((r == 2'b00) && ((s == 2'b00) && (channel_in_group_cnt == 6'd0)));
	assign conv_vld_ppu_now			= (dp_issue && last_kernel) && last_channel_in_group;
	assign layer0_line_done			= (layer == 3'b000) && (conv_vld_ppu_now && x_in == (conv_width - 8'd1));
	assign last_cnn_input			= conv_vld_ppu_now && ((x_in == (conv_width - 8'd1)) && (y_in == (conv_height - 8'd1)));
	assign src_x					= x_in + r - ((kernel_size == 2'b01) ? 0 : 1);
	assign src_y					= y_in + s - ((kernel_size == 2'b01) ? 0 : 1);
	assign group16					= channel_in_group_cnt[5:1];
	assign src_y_mod3				= $unsigned(src_y) % 9'd3;

	always @(posedge clk) begin
		if (!resetn) begin
			lb_raddr		<= 8'd0;
			lb_sel_out		<= 2'd0;
			a_raddr_q		<= 15'd0;
			dp_b_raddr_q	<= 14'd0;
			w_raddr_q		<= 10'd0;
		end else begin
			lb_raddr		<= lb_raddr_next	;
			lb_sel_out		<= lb_sel_out_next	;
			a_raddr_q		<= a_raddr_next		;
			dp_b_raddr_q	<= dp_b_raddr_next	;
			w_raddr_q		<= w_raddr_next		;
		end
	end

	always @(*) begin
	    lb_raddr_next   = 8'd0;
	    lb_sel_out_next = 2'd0;
	    a_raddr_next    = 15'd0;
	    dp_b_raddr_next = 14'd0;
	    w_raddr_next    = 9'd0;
	
	    pixel_addr      = 15'd0;
	    group_base      = 15'd0;
	    feature_raddr   = 15'd0;
	
	    if (dp_issue) begin
	        // Weight address
	        if (kernel_size == 2'b01) begin
	            w_raddr_next = channel_in_group_cnt;
	        end else begin
	            w_raddr_next =
	                (channel_in_group_cnt << 3)
	              + channel_in_group_cnt
	              + (s << 1) + s + r;
	        end
	
	        if (cu_padding_sel_conv_now) begin
	            if (layer == 3'b000) begin
	                lb_raddr_next   = src_x[7:0];
	                lb_sel_out_next = src_y_mod3[1:0];
	            end else begin
	                case (layer)
	                    3'b001: begin
	                        pixel_addr = (src_y << 7)
	                                   - (src_y << 4)
	                                   + src_x;
	                        group_base = 15'd0;
	                    end
	
	                    3'b010: begin
	                        pixel_addr = (src_y << 6)
	                                   - (src_y << 3)
	                                   + src_x;
	                        group_base = (group16 << 12)
	                                   + (group16 << 11)
	                                   + (group16 << 7);
	                    end
	
	                    3'b011: begin
	                        pixel_addr = (src_y << 5)
	                                   - (src_y << 2)
	                                   + src_x;
	                        group_base = (group16 << 10)
	                                   + (group16 << 9)
	                                   + (group16 << 5);
	                    end
	
	                    3'b100: begin
	                        pixel_addr = (src_y << 4)
	                                   - (src_y << 1)
	                                   + src_x;
	                        group_base = (group16 << 8)
	                                   + (group16 << 7)
	                                   + (group16 << 3);
	                    end
	
	                    default: begin
	                        pixel_addr = (src_y << 3)
	                                   - src_y
	                                   + src_x;
	                        group_base = (group16 << 6)
	                                   + (group16 << 5)
	                                   + (group16 << 1);
	                    end
	                endcase
	
	                feature_raddr =
	                    group_base
	                  + (pixel_addr << 1)
	                  + channel_in_group_cnt[0];
	
	                if (dp_input_sel == 2'b01)
	                    a_raddr_next = feature_raddr;
	                else if (dp_input_sel == 2'b10)
	                    dp_b_raddr_next = feature_raddr[13:0];
	            end
	        end
	    end
	end
	//	output assign

	assign cu_valid_conv		= cu_vld_conv_d2;
	assign cu_padding_sel_conv	= cu_padding_sel_conv_d2;
	assign cu_acc_vld_sel_conv	= cu_acc_vld_sel_conv_d5;
	assign cu_buffer_sel_conv	= cu_buffer_sel_conv_d5;
	assign conv_valid_ppu		= conv_vld_ppu_d6;

	always @(*) begin
		if (layer == 3'b000) begin
			dp_input_sel	= 2'b00;
		end else if (layer[0]) begin
			dp_input_sel	= 2'b01;
		end else begin
			dp_input_sel	= 2'b10;
		end
	end

	//	Sequantial Logic

	//	main FSM

	always @(posedge clk or negedge resetn) begin
		if (!resetn) begin
			layer	<= 3'b000;
		end else begin
			if (ma_cstate == M_IDLE) begin
				layer	<= 3'b000;
			end else if ((ma_cstate == M_NEXT_LAYER) && (layer < 3'b110)) begin
				layer	<= layer + 3'b001;
			end
		end
	end

	always @(posedge clk or negedge resetn) begin
		if (!resetn) begin
			channel_out_group	<= 5'b00000;
		end else begin
			if (ma_cstate == M_IDLE) begin
				channel_out_group	<= 5'b00000;
			end else if ((ma_cstate == M_RUN_LAYER) && ((dp_done) && (!last_out_group))) begin
				channel_out_group	<= channel_out_group + 5'b00001;
			end else if (ma_cstate == M_NEXT_LAYER) begin
				channel_out_group	<= 5'b00000;
			end
		end
	end

	//	weight FSM

	always @(posedge clk) begin
		if (!resetn) begin
			w_waddr	<= 10'd0;
		end else begin
			if ((we_cstate == W_IDLE) && weight_srt) begin
				w_waddr	<= 10'd0;
			end else if ((we_cstate == W_LOAD) && w_we) begin
				if (w_waddr < (weight_word_count[layer] - 10'd1)) begin
					w_waddr	<= w_waddr + 10'd1;
				end
			end
		end
	end

	//	fm FSM

	always @(posedge clk) begin
		if (!resetn) begin
			a_waddr	<= 15'd0;
			b_waddr	<= 14'd0;
		end else begin
			if ((ma_cstate == M_IDLE) || (ma_cstate == M_NEXT_LAYER)) begin
				a_waddr	<= 15'd0;
				b_waddr	<= 14'd0;
			end else begin
				if (a_we)	a_waddr	<= a_waddr + 15'd1;
				if (b_we)	b_waddr	<= b_waddr + 14'd1;
			end
		end
	end

	//	oa FSM

	always @(posedge clk or negedge resetn) begin
		if (!resetn) begin
			oa_half_sel_q	<= 1'b0;
		end else begin
			if (oa_cstate == OA_IDLE) begin
				oa_half_sel_q	<= 1'b0;
			end else if ((o_valid && o_ready) && (oa_cstate == OA_SEND)) begin
				oa_half_sel_q	<= ~oa_half_sel_q;
			end
		end
	end

	always @(posedge clk) begin
		if (!resetn) begin
			oa_raddr	<= 8'd0;
		end else begin
			if (oa_cstate == OA_IDLE) begin
				oa_raddr	<= 8'd0;
			end else if ((o_valid && o_ready) && (oa_cstate == OA_SEND)) begin
				if (oa_half_sel) begin
					if (oa_raddr < 8'd195) begin
						oa_raddr	<= oa_raddr + 8'd1;
					end
				end
			end
		end
	end

	//	img FSM

	always @(posedge clk) begin
		if (!resetn) begin
			img_x_cnt		<= 8'd0;
			img_row_cnt		<= 8'd0;
			img_bank_sel	<= 2'b00;
		end else begin
			if (im_cstate == IMG_IDLE) begin
				img_x_cnt		<= 8'd0;
				img_row_cnt		<= 8'd0;
				img_bank_sel	<= 2'b00;
			end else if ((im_cstate == IMG_LOAD) && lb_we) begin
				if (img_x_cnt == (L0_WH - 1)) begin
					img_x_cnt	<= 8'd0;
					if (img_row_cnt < (L0_WH - 1)) begin
						img_row_cnt	<= img_row_cnt + 8'd1;
						if (img_bank_sel == 2'b10) begin
							img_bank_sel	<= 2'b00;
						end else begin
							img_bank_sel	<= img_bank_sel + 2'b01;
						end
					end
				end else begin
					img_x_cnt	<= img_x_cnt + 8'd1;
				end
			end
		end
	end

	//	dp FSM

	always @(posedge clk or negedge resetn) begin
		if (!resetn) begin
			kernel_size		<= L0_RS;
		end else begin
			if ((dp_nstate == DP_CNN) && (dp_cstate == DP_IDLE)) begin
				if (layer == 3'b110) begin
					kernel_size		<= L8_RS;
				end else begin
					kernel_size		<= L1_RS;
				end
			end
		end
	end

	always @(posedge clk or negedge resetn) begin
		if (!resetn) begin
			ppu_layer_idx	<= 0;
		end else begin
			if (dp_cstate == DP_IDLE) begin
				ppu_layer_idx	<= 0;
			end else if (dp_issue) begin
				ppu_layer_idx	<= layer;
			end
		end
	end

	always @(posedge clk or negedge resetn) begin
		if (!resetn) begin
			ppu_channel_group_idx	<= 0;
		end else begin
			if (dp_cstate == DP_IDLE) begin
				ppu_channel_group_idx	<= 0;
			end else if (dp_issue) begin
				ppu_channel_group_idx	<= channel_out_group;
			end
		end
	end

	always @(posedge clk or negedge resetn)  begin
		if (!resetn) begin
			r						<= 0;
			s						<= 0;
			channel_in_group_cnt	<= 0;
			x_in					<= 0;
			y_in					<= 0;
		end else begin
			if (dp_cstate == DP_IDLE) begin
				r						<= 0;
				s						<= 0;
				channel_in_group_cnt	<= 0;
				x_in					<= 0;
				y_in					<= 0;
			end else if (dp_issue) begin
				if (!last_kernel) begin
					if (r == (kernel_size - 1)) begin
						r	<= 0;
						s	<= s + 1;
					end else begin
						r	<= r + 1;
					end
				end else begin
					r	<= 0;
					s	<= 0;
					if (!last_channel_in_group) begin
						channel_in_group_cnt	<= channel_in_group_cnt + 1;
					end else begin
						channel_in_group_cnt	<= 0;
						if (x_in == (conv_width - 1)) begin
							x_in	<= 0;
							y_in	<= y_in + 1;
						end else begin
							x_in	<= x_in + 1;
						end
					end
				end
			end
		end
	end

	always @(posedge clk or negedge resetn) begin
		if (!resetn) begin
			channel_in_group	<= L0_INPUT_GROUP;
			conv_width			<= L0_WH;
			conv_height			<= L0_WH;
		end else begin
			if ((dp_nstate == DP_CNN) && (dp_cstate == DP_IDLE)) begin
				case (layer)
					3'b000:	begin
						channel_in_group	<= L0_INPUT_GROUP;
						conv_width			<= L0_WH;
						conv_height			<= L0_WH;
					end
					3'b001:	begin
						channel_in_group	<= L1_INPUT_GROUP;
						conv_width			<= L1_WH;
						conv_height			<= L1_WH;
					end
					3'b010:	begin
						channel_in_group	<= L2_INPUT_GROUP;
						conv_width			<= L2_WH;
						conv_height			<= L2_WH;
					end
					3'b011:	begin
						channel_in_group	<= L3_INPUT_GROUP;
						conv_width			<= L3_WH;
						conv_height			<= L3_WH;
					end
					3'b100:	begin
						channel_in_group	<= L4_INPUT_GROUP;
						conv_width			<= L4_WH;
						conv_height			<= L4_WH;
					end
					3'b101:	begin
						channel_in_group	<= L5_INPUT_GROUP;
						conv_width			<= L5_WH;
						conv_height			<= L5_WH;
					end
					3'b110:	begin
						channel_in_group	<= L6_INPUT_GROUP;
						conv_width			<= L6_WH;
						conv_height			<= L6_WH;
					end
					3'b111:	begin
						channel_in_group	<= L7_INPUT_GROUP;
						conv_width			<= L7_WH;
						conv_height			<= L7_WH;
					end
					default:begin
						channel_in_group	<= L0_INPUT_GROUP;
						conv_width			<= L0_WH;
						conv_height			<= L0_WH;
					end
				endcase
			end
		end
	end

	//	output delay

	always @(posedge clk or negedge resetn) begin
		if (!resetn) begin
			cu_vld_conv_d1			<= 1'b0;
			cu_vld_conv_d2			<= 1'b0;
			cu_padding_sel_conv_d1	<= 1'b0;
			cu_padding_sel_conv_d2	<= 1'b0;
			cu_acc_vld_sel_conv_d1	<= 1'b0;
			cu_acc_vld_sel_conv_d2	<= 1'b0;
			cu_acc_vld_sel_conv_d3	<= 1'b0;
			cu_acc_vld_sel_conv_d4	<= 1'b0;
			cu_acc_vld_sel_conv_d5	<= 1'b0;
			accumulation_bank		<= 1'b0;
			cu_buffer_sel_conv_d1	<= 1'b0;
			cu_buffer_sel_conv_d2	<= 1'b0;
			cu_buffer_sel_conv_d3	<= 1'b0;
			cu_buffer_sel_conv_d4	<= 1'b0;
			cu_buffer_sel_conv_d5	<= 1'b0;
			conv_vld_ppu_d1			<= 1'b0;
			conv_vld_ppu_d2			<= 1'b0;
			conv_vld_ppu_d3			<= 1'b0;
			conv_vld_ppu_d4			<= 1'b0;
			conv_vld_ppu_d5			<= 1'b0;
			conv_vld_ppu_d6			<= 1'b0;
		end else begin
			cu_vld_conv_d1			<= dp_issue					;
			cu_vld_conv_d2			<= cu_vld_conv_d1			;
			cu_padding_sel_conv_d1	<= cu_padding_sel_conv_now	;
			cu_padding_sel_conv_d2	<= cu_padding_sel_conv_d1	;
			cu_acc_vld_sel_conv_d1	<= cu_acc_vld_sel_conv_now	;
			cu_acc_vld_sel_conv_d2	<= cu_acc_vld_sel_conv_d1	;
			cu_acc_vld_sel_conv_d3	<= cu_acc_vld_sel_conv_d2	;
			cu_acc_vld_sel_conv_d4	<= cu_acc_vld_sel_conv_d3	;
			cu_acc_vld_sel_conv_d5	<= cu_acc_vld_sel_conv_d4	;
			cu_buffer_sel_conv_d1	<= accumulation_bank		;
			cu_buffer_sel_conv_d2	<= cu_buffer_sel_conv_d1	;
			cu_buffer_sel_conv_d3	<= cu_buffer_sel_conv_d2	;
			cu_buffer_sel_conv_d4	<= cu_buffer_sel_conv_d3	;
			cu_buffer_sel_conv_d5	<= cu_buffer_sel_conv_d4	;
			conv_vld_ppu_d1			<= conv_vld_ppu_now			;
			conv_vld_ppu_d2			<= conv_vld_ppu_d1			;
			conv_vld_ppu_d3			<= conv_vld_ppu_d2			;
			conv_vld_ppu_d4			<= conv_vld_ppu_d3			;
			conv_vld_ppu_d5			<= conv_vld_ppu_d4			;
			conv_vld_ppu_d6			<= conv_vld_ppu_d5			;
			if (conv_vld_ppu_now)	accumulation_bank	<= accumulation_bank ^ 1'b1;
		end
	end

	//	FSM

	//	FSM Sequantial Logic

	always @(posedge clk or negedge resetn) begin
		if (!resetn) begin
			ma_cstate	<= M_IDLE;
			we_cstate	<= W_IDLE;
			im_cstate	<= IMG_IDLE;
			dp_cstate	<= DP_IDLE;
			fm_cstate	<= FM_IDLE;
			oa_cstate	<= OA_IDLE;
		end else begin
			ma_cstate	<= ma_nstate;
			we_cstate	<= we_nstate;
			im_cstate	<= im_nstate;
			dp_cstate	<= dp_nstate;
			fm_cstate	<= fm_nstate;
			oa_cstate	<= oa_nstate;
		end
	end

	//	Main FSM Combinational Logic

	always @(*) begin
		ma_nstate = ma_cstate;
		case (ma_cstate)
			M_IDLE         	:	begin
				if (cr_srt) ma_nstate = M_L0_LAYER_PREP;
			end
			M_L0_LAYER_PREP	:	begin
				ma_nstate = M_L0_WAIT_LOAD;
			end
			M_L0_WAIT_LOAD 	:	begin
				if (weight_done && img_window_rdy) ma_nstate = M_L0_RUN_LAYER;
			end
			M_L0_RUN_LAYER 	:	begin
				if (dp_done && img_done) ma_nstate = M_NEXT_LAYER;
			end
			M_LAYER_PREP   	:	begin
				ma_nstate = M_WAIT_LOAD;
			end
			M_WAIT_LOAD    	:	begin
				if (weight_done) ma_nstate = M_RUN_LAYER;
			end
			M_RUN_LAYER    	:	begin
				if (dp_done) begin
					if (last_out_group)	ma_nstate = M_NEXT_LAYER;
					else				ma_nstate = M_LAYER_PREP;
				end
			end
			M_NEXT_LAYER   	:	begin
				if (layer == 3'b110)	ma_nstate = M_OA_STORE;
				else					ma_nstate = M_LAYER_PREP;
			end
			M_OA_STORE     	:	begin
				if (oa_done)	ma_nstate = M_DONE;
			end
			M_DONE         	:	begin
				ma_nstate = M_IDLE;
			end
			default:			begin
				ma_nstate = M_IDLE;
			end
		endcase
	end

	//	Weight FSM Combinational Logic

	always @(*) begin
		we_nstate = we_cstate;
		case (we_cstate)
			W_IDLE:	begin
				if (weight_srt)	we_nstate = W_LOAD;
			end
			W_LOAD:	begin
				if (w_we && (w_waddr == (weight_word_count[layer] - 10'd1)))	we_nstate = W_DONE;
			end
			W_DONE:	begin
				if (weight_done_ack) we_nstate = W_IDLE;
			end
			default:begin
				we_nstate = W_IDLE;
			end
		endcase
	end

	//	IMG FSM Combinational Logic

	always @(*) begin
		im_nstate = im_cstate;
		case (im_cstate)
			IMG_IDLE:	begin
				if (img_srt)	im_nstate = IMG_LOAD;
			end
			IMG_LOAD:	begin
				if (lb_we && (img_x_cnt == L0_WH - 1)) begin
					if (img_row_cnt == L0_WH - 1) begin
						im_nstate = IMG_DONE;
					end else if (img_row_cnt >= 2) begin
						im_nstate = IMG_WAIT;
					end
				end
			end
			IMG_WAIT:	begin
				if (layer0_line_done && ((y_in >= 1) && (y_in <= L0_WH - 3)))	im_nstate = IMG_LOAD;
			end
			IMG_DONE:	begin
				if (img_done_ack) im_nstate = IMG_IDLE;
			end
			default:	begin
				im_nstate = IMG_IDLE;
			end
		endcase
	end

	//	DP FSM Combinational Logic

	always @(*) begin
		dp_nstate = dp_cstate;
		case (dp_cstate)
			DP_IDLE	:	begin
				if (dp_srt)	dp_nstate = DP_CNN;
			end
			DP_CNN	:	begin
				if (last_cnn_input)	dp_nstate = DP_DRAIN;
			end
			DP_DRAIN:	begin
				if (fm_done)	dp_nstate = DP_DONE;
			end
			DP_DONE	:	begin
				if (dp_done_ack)	dp_nstate = DP_IDLE;
			end
			default	:	begin
				dp_nstate = DP_IDLE;
			end
		endcase
	end

	//	Feature map Combinational Logic

	always @(*) begin
		fm_nstate = fm_cstate;
		case (fm_cstate)
			FM_IDLE:	begin
				if (ppu_out_vld)	fm_nstate = FM_STORE;
			end
			FM_STORE:	begin
				fm_nstate = FM_IDLE;
			end
			default:	begin
				fm_nstate = FM_IDLE;
			end
		endcase
	end

	//	OA Combinational Logic

	always @(*) begin
		oa_nstate = oa_cstate;
		case (oa_cstate)
			OA_IDLE:	begin
				if (oa_srt)	oa_nstate = OA_READ;
			end
			OA_READ:	begin
				oa_nstate = OA_SEND;
			end
			OA_SEND:	begin
				if (o_valid && o_ready) begin
					if (oa_last) begin
						oa_nstate = OA_DONE;
					end else if (oa_half_sel) begin
						oa_nstate = OA_READ;
					end else begin
						oa_nstate = OA_SEND;
					end
				end
			end
			OA_DONE:	begin
				if (oa_done_ack)	oa_nstate = OA_IDLE;
			end
			default:	begin
				oa_nstate = OA_IDLE;
			end
		endcase
	end

endmodule
