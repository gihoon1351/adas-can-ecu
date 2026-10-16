`timescale 1ns / 1ps

module sobel_filter_stream (
    input wire axis_aclk,
    input wire axis_aresetn,

    // input wire        cfg_en,
    input wire [15:0] cfg_thresh,

    input wire [23:0] s_axis_tdata,
    input wire s_axis_tvalid,
    output wire s_axis_tready,
    input wire s_axis_tuser,  //	1: SoF
    input wire s_axis_tlast,  //	1: EoL

    output wire [23:0] m_axis_tdata,
    output wire m_axis_tvalid,
    input wire m_axis_tready,
    output wire m_axis_tuser,
    output wire m_axis_tlast
);
  wire        pipe_ce;
  wire        accept;

  wire [23:0] sobel_data;
  wire        sobel_valid;
  wire        sobel_user;
  wire        sobel_last;

  assign pipe_ce = (~sobel_valid) | m_axis_tready;

  assign s_axis_tready = pipe_ce;
  assign accept = s_axis_tvalid & s_axis_tready;

  assign m_axis_tdata = sobel_data;
  assign m_axis_tvalid = sobel_valid;
  assign m_axis_tuser = sobel_user;
  assign m_axis_tlast = sobel_last;

  sobel_top U_ST (
      .axis_aclk   (axis_aclk),
      .axis_aresetn(axis_aresetn),
      .pipe_ce     (pipe_ce),

      .i_data   (s_axis_tdata),
      .sof      (accept && s_axis_tuser),
      .eol      (accept && s_axis_tlast),
      .hs       (accept),
      .threshold(cfg_thresh),

      .sof_o (sobel_user),
      .eol_o (sobel_last),
      .o_data(sobel_data),
      .vld   (sobel_valid)
  );
endmodule

module sobel_top (
    input wire axis_aclk,
    input wire axis_aresetn,
    input wire pipe_ce,

    input wire [23:0] i_data,
    input wire        sof,
    input wire        eol,
    input wire        hs,
    input wire [15:0] threshold,

    output wire        sof_o,
    output wire        eol_o,
    output wire [23:0] o_data,
    output wire        vld
);
  wire we;
  wire [10:0] lb_waddr;
  wire [10:0] lb_raddr;
  wire [1:0] sel_line;
  wire [1:0] swap_sobel;

  wire [23:0] data0;
  wire [23:0] data1;
  wire [23:0] data2;

  sobel_controller U_SC (
      .clk    (axis_aclk),
      .resetn (axis_aresetn),
      .pipe_ce(pipe_ce),

      .sof(sof),
      .hs (hs),
      .eol(eol),

      .we        (we),
      .lb_waddr  (lb_waddr),
      .lb_raddr  (lb_raddr),
      .sel_line  (sel_line),
      .swap_sobel(swap_sobel),
      .sof_o     (sof_o),
      .eol_o     (eol_o),
      .vld       (vld)
  );

  line_buffer U_LB (
      .clk  (axis_aclk ),
      .sel  (sel_line ),
      .we   (we  ),
      .waddr  (lb_waddr ),
      .wdata  (i_data ),
      .raddr  (lb_raddr ),
      .data0  (data0 ),
      .data1  (data1 ),
      .data2  (data2 )
  );

  sobel_filter U_SF (
      .clk   (axis_aclk),
      .resetn(axis_aresetn),
      .ce    (hs),

      .data0     (data0),
      .data1     (data1),
      .data2     (data2),
      .swap      (swap_sobel),
      .threshold (threshold),
      .sobel_data(o_data)
  );
endmodule

module sobel_controller (
    input  wire        clk,
    input  wire        resetn,
    input  wire        sof,
    input  wire        hs,
    input  wire        eol,
    input  wire        pipe_ce,
    output wire        sof_o,
    output wire        eol_o,
    output wire        vld,
    output wire        we,
    output wire [10:0] lb_waddr,
    output wire [10:0] lb_raddr,
    output wire [ 1:0] sel_line,
    output wire [ 1:0] swap_sobel
);
  reg [10:0] waddr_r;
  reg [10:0] raddr_r;
  reg [ 1:0] sel_r;
  reg [ 1:0] swap_r;

  reg [ 4:0] sof_r;
  reg [ 4:0] eol_r;
  reg [ 4:0] vld_r;

  assign we = hs;
  assign lb_waddr = waddr_r;
  assign lb_raddr = raddr_r;
  assign sel_line = sel_r;
  assign swap_sobel = sel_r;  // swap_r 순서가 sel 순서와 동일해서 일단 ㄱㄱ


  assign sof_o = sof_r[4];
  assign eol_o = eol_r[4];
  assign vld = vld_r[4];

  always @(posedge clk) begin
    if (!resetn) begin
      waddr_r <= 0;
      raddr_r <= 0;
      sel_r   <= 2'b00;
      swap_r  <= 2'b00;
    end else begin
      if (hs) begin
        if (eol) begin
          waddr_r <= 0;
          raddr_r <= 0;
          sel_r   <= (sel_r == 2'b10) ? 2'b00 : (sel_r + 2'b01);
        end else begin
          waddr_r <= waddr_r + 1;
          raddr_r <= raddr_r + 1;
        end
      end
    end
  end

  always @(posedge clk) begin
    if (!resetn) begin
      sof_r <= 0;
      eol_r <= 0;
      vld_r <= 0;
    end else if (pipe_ce) begin
      sof_r <= {sof_r[3:0], sof};
      eol_r <= {eol_r[3:0], eol};
      vld_r <= {vld_r[3:0], hs};
    end
  end
endmodule

module line_buffer (
    input   clk   ,
    input [1:0] sel   ,
    input   we   ,
    input [10:0] waddr  ,
    input [23:0] wdata  ,
    input [10:0] raddr  ,
    output [23:0] data0  ,
    output [23:0] data1  ,
    output [23:0] data2
);
  reg [23:0] line0_ram  [0:1279];
  reg [23:0] line1_ram  [0:1279];
  reg [23:0] line2_ram  [0:1279];

  reg [23:0] line_data0;
  reg [23:0] line_data1;
  reg [23:0] line_data2;

  assign data0 = line_data0;
  assign data1 = line_data1;
  assign data2 = line_data2;

  always @(posedge clk) begin
    if (we && (sel == 2'b00)) begin
      line0_ram[waddr] <= wdata;
    end
  end

  always @(posedge clk) begin
    if (we && (sel == 2'b01)) begin
      line1_ram[waddr] <= wdata;
    end
  end

  always @(posedge clk) begin
    if (we && (sel == 2'b10)) begin
      line2_ram[waddr] <= wdata;
    end
  end

  always @(posedge clk) begin
    line_data0 <= line0_ram[raddr];
    line_data1 <= line1_ram[raddr];
    line_data2 <= line2_ram[raddr];
  end
endmodule

module sobel_filter (
    input   clk   ,
    input   resetn  ,
    input [23:0] data0  ,
    input [23:0] data1  ,
    input [23:0] data2  ,
    input [1:0] swap  ,
    input [15:0] threshold ,
    input ce,
    output [23:0] sobel_data
);
  reg [23:0] line0[0:2];
  reg [23:0] line1[0:2];
  reg [23:0] line2[0:2];

  wire [23:0] line0_input;
  wire [23:0] line1_input;
  wire [23:0] line2_input;

  wire [11:0] dx_red;
  wire [11:0] dx_blu;
  wire [11:0] dx_gre;

  reg [11:0] dx_red_r;
  reg [11:0] dx_blu_r;
  reg [11:0] dx_gre_r;

  wire [11:0] dy_red;
  wire [11:0] dy_blu;
  wire [11:0] dy_gre;

  reg [11:0] dy_red_r;
  reg [11:0] dy_blu_r;
  reg [11:0] dy_gre_r;

  wire [13:0] gradient_red;
  wire [13:0] gradient_blu;
  wire [13:0] gradient_gre;

  assign dx_red = - {1'b0, line0[0][23:16]} + {1'b0, line0[2][23:16]} - {1'b0, line1[0][23:16], 1'b0} + {1'b0, line1[2][23:16], 1'b0} - {1'b0, line2[0][23:16]} + {1'b0, line2[2][23:16]};
  assign dy_red = - {1'b0, line0[0][23:16]} + {1'b0, line2[0][23:16]} - {1'b0, line0[1][23:16], 1'b0} + {1'b0, line2[1][23:16], 1'b0} - {1'b0, line0[2][23:16]} + {1'b0, line2[2][23:16]};
  assign gradient_red = dx_red_r + dy_red_r;

  assign dx_blu = - {1'b0, line0[0][15:8]} + {1'b0, line0[2][15:8]} - {1'b0, line1[0][15:8], 1'b0} + {1'b0, line1[2][15:8], 1'b0} - {1'b0, line2[0][15:8]} + {1'b0, line2[2][15:8]};
  assign dy_blu = - {1'b0, line0[0][15:8]} + {1'b0, line2[0][15:8]} - {1'b0, line0[1][15:8], 1'b0} + {1'b0, line2[1][15:8], 1'b0} - {1'b0, line0[2][15:8]} + {1'b0, line2[2][15:8]};
  assign gradient_blu = dx_blu_r + dy_blu_r;

  assign dx_gre = - {1'b0, line0[0][7:0]} + {1'b0, line0[2][7:0]} - {1'b0, line1[0][7:0], 1'b0} + {1'b0, line1[2][7:0], 1'b0} - {1'b0, line2[0][7:0]} + {1'b0, line2[2][7:0]};
  assign dy_gre = - {1'b0, line0[0][7:0]} + {1'b0, line2[0][7:0]} - {1'b0, line0[1][7:0], 1'b0} + {1'b0, line2[1][7:0], 1'b0} - {1'b0, line0[2][7:0]} + {1'b0, line2[2][7:0]};
  assign gradient_gre = dx_gre_r + dy_gre_r;

  assign sobel_data[23:16] = (gradient_red > threshold) ? 8'hff : 8'h00;
  assign sobel_data[15:8] = (gradient_blu > threshold) ? 8'hff : 8'h00;
  assign sobel_data[7:0] = (gradient_gre > threshold) ? 8'hff : 8'h00;

  mux_4_1_sobel U_MUX_LINE0 (
      .in0(data0),
      .in1(data1),
      .in2(data2),
      .in3(24'h000000),
      .sel(swap),
      .out(line0_input)
  );
  mux_4_1_sobel U_MUX_LINE1 (
      .in0(data1),
      .in1(data2),
      .in2(data0),
      .in3(24'h000000),
      .sel(swap),
      .out(line1_input)
  );

  mux_4_1_sobel U_MUX_LINE2 (
      .in0(data2),
      .in1(data0),
      .in2(data1),
      .in3(24'h000000),
      .sel(swap),
      .out(line2_input)
  );
  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      dx_blu_r <= 0;
      dy_blu_r <= 0;
    end else if (ce) begin
      dx_blu_r <= dx_blu[11] ? -dx_blu : dx_blu;
      dy_blu_r <= dy_blu[11] ? -dy_blu : dy_blu;
    end
  end

  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      dx_gre_r <= 0;
      dy_gre_r <= 0;
    end else if (ce) begin
      dx_gre_r <= dx_gre[11] ? -dx_gre : dx_gre;
      dy_gre_r <= dy_gre[11] ? -dy_gre : dy_gre;
    end
  end

  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      dx_red_r <= 0;
      dy_red_r <= 0;
    end else if (ce) begin
      dx_red_r <= dx_red[11] ? -dx_red : dx_red;
      dy_red_r <= dy_red[11] ? -dy_red : dy_red;
    end
  end

  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      line0[2] <= 8'h00;
      line0[1] <= 8'h00;
      line0[0] <= 8'h00;
      line1[2] <= 8'h00;
      line1[1] <= 8'h00;
      line1[0] <= 8'h00;
      line2[2] <= 8'h00;
      line2[1] <= 8'h00;
      line2[0] <= 8'h00;
    end else if (ce) begin
      line0[2] <= line0[1];
      line0[1] <= line0[0];
      line0[0] <= line0_input;
      line1[2] <= line1[1];
      line1[1] <= line1[0];
      line1[0] <= line1_input;
      line2[2] <= line2[1];
      line2[1] <= line2[0];
      line2[0] <= line2_input;
    end
  end
endmodule

module mux_4_1_sobel (
    input  [23:0] in0,
    input  [23:0] in1,
    input  [23:0] in2,
    input  [23:0] in3,
    input  [ 1:0] sel,
    output [23:0] out
);
  reg [23:0] out_r;
  assign out = out_r;
  always @(*) begin
    case (sel)
      2'b00:   out_r = in0;
      2'b01:   out_r = in1;
      2'b10:   out_r = in2;
      2'b11:   out_r = in3;
      default: out_r = 24'hxxxxxx;
    endcase
  end
endmodule