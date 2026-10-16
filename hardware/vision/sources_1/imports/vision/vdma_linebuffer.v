`timescale 1ns / 1ps

module vdma_linebuffer (
    input [7:0] lb_raddr,
    input [7:0] lb_waddr,
    input [1:0] lb_sel_in,
    input [1:0] lb_sel_out,
    input we,
    input tready,

    output lb_tvalid,

    //axi stream
    input axis_aclk,
    input axis_aresetn,
    input [23:0] s_axis_tdata,  //
    input s_axis_tvalid,
    output s_axis_tready,
    input wire s_axis_tuser,
    input wire s_axis_tlast,

    output [63:0] lb_output
);

  wire [23:0] o_bram_0, o_bram_1, o_bram_2;
  wire [23:0] o_mux_3x1;
  wire we_0, we_1, we_2;
  wire	[23:0]	cnn_input_data;
  assign s_axis_tready = axis_aresetn && tready && (lb_sel_in != 2'd3);
  assign lb_output = {o_mux_3x1, 40'd0};
  assign lb_tvalid = s_axis_tvalid;
  reg	[1:0]	lb_sel_out_d1;

  assign cnn_input_data = {1'b0, s_axis_tdata[23:17], 1'b0, s_axis_tdata[15:9], 1'b0, s_axis_tdata[7:1]};

  always @(posedge axis_aclk or negedge axis_aresetn) begin
	  if (!axis_aresetn) begin
		  lb_sel_out_d1	<= 2'b00;
	  end else begin
		  lb_sel_out_d1	<= lb_sel_out;
	  end
  end

  demux_1x3 U_demux_1x3 (
      .we(we),
      .lb_sel_in(lb_sel_in),
      .o_we_0(we_0),
      .o_we_1(we_1),
      .o_we_2(we_2)
  );

  video_bram U_video_bram_0 (
      .we(we_0),
      .axis_aclk(axis_aclk),
      .i_data(cnn_input_data),
      .lb_raddr(lb_raddr),
      .lb_waddr(lb_waddr),
      .o_data(o_bram_0) 
  );

  video_bram U_video_bram_1 (
      .we(we_1),
      .axis_aclk(axis_aclk),
      .i_data(cnn_input_data),
      .lb_raddr(lb_raddr),
      .lb_waddr(lb_waddr),
      .o_data(o_bram_1) 
  );

  video_bram U_video_bram_2 (
      .we(we_2),
      .axis_aclk(axis_aclk),
      .i_data(cnn_input_data),
      .lb_raddr(lb_raddr),
      .lb_waddr(lb_waddr),
      .o_data(o_bram_2) 
  );

  mux_3x1 U_mux_3x1 (
      .in_a (o_bram_0),
      .in_b (o_bram_1),
      .in_c (o_bram_2),
      .sel  (lb_sel_out_d1),
      .o_mux(o_mux_3x1)
  );

endmodule


module demux_1x3 (
    input we,
    input [1:0] lb_sel_in,
    output reg o_we_0,
    output reg o_we_1,
    output reg o_we_2
);

  always @(*) begin
    o_we_0 = 1'b0;
    o_we_1 = 1'b0;
    o_we_2 = 1'b0;
    case ({
      lb_sel_in, we
    })
      3'b001: begin
        o_we_0 = 1'b1;
        o_we_1 = 1'b0;
        o_we_2 = 1'b0;
      end
      3'b011: begin
        o_we_0 = 1'b0;
        o_we_1 = 1'b1;
        o_we_2 = 1'b0;
      end
      3'b101: begin
        o_we_0 = 1'b0;
        o_we_1 = 1'b0;
        o_we_2 = 1'b1;
      end
      default: begin
        o_we_0 = 1'b0;
        o_we_1 = 1'b0;
        o_we_2 = 1'b0;
      end
    endcase
  end

endmodule


module video_bram (
		input we,
		input axis_aclk,
		input [23:0] i_data,
		input [7:0] lb_raddr,
		input [7:0] lb_waddr,
		output reg [23:0] o_data 
	);
	reg [23:0] video_bram[0:223];

	always @(posedge axis_aclk) begin
		o_data <= video_bram[lb_raddr];
	end

	always @(posedge axis_aclk) begin
		if (we) begin
			video_bram[lb_waddr] <= i_data;
		end
	end

endmodule



module mux_3x1 (
		input [23:0] in_a,
		input [23:0] in_b,
		input [23:0] in_c,
		input [1:0] sel,
		output reg [23:0] o_mux
	);
	always @(*) begin
		o_mux = 24'd0;
		case (sel)
			2'd0: o_mux = in_a;
			2'd1: o_mux = in_b;
			2'd2: o_mux = in_c;
			default: o_mux = 24'd0;
		endcase
	end

endmodule
