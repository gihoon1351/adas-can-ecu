`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/10/02 14:48:47
// Design Name: 
// Module Name: rgb2ycbcr
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


module rgb2ycbcr(
    input  wire        axis_aclk,
    input  wire        axis_aresetn,
    input  wire        en,

    // AXI4-Stream slave
    input  wire [23:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tuser,  // high -> start of frame
    input  wire        s_axis_tlast,  // high -> end of line

    // AXI4-Stream master
    output reg  [23:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tuser,
    output reg         m_axis_tlast
);

  // Three-stage elastic pipeline:
  //   stage 1: RGB constant multiplications
  //   stage 2: partial additions
  //   stage 3: final addition, shift and output register

  reg         valid_s1;
  reg  [15:0] r_mul_s1;
  reg  [15:0] g_mul_s1;
  reg  [15:0] b_mul_s1;
  reg  [23:0] rgb_s1;
  reg         en_s1;
  reg         user_s1;
  reg         last_s1;

  reg         valid_s2;
  reg  [15:0] rg_sum_s2;
  reg  [15:0] b_round_s2;
  reg  [23:0] rgb_s2;
  reg         en_s2;
  reg         user_s2;
  reg         last_s2;

  wire ready_s3;
  wire ready_s2;
  wire ready_s1;
  wire accept;

  assign ready_s3 = (~m_axis_tvalid) | m_axis_tready;
  assign ready_s2 = (~valid_s2)       | ready_s3;
  assign ready_s1 = (~valid_s1)       | ready_s2;

  assign s_axis_tready = ready_s1;
  assign accept = s_axis_tvalid & s_axis_tready;

  // Existing input packing is R-B-G.
  wire [7:0] pix_r;
  wire [7:0] pix_g;
  wire [7:0] pix_b;

  assign pix_r = s_axis_tdata[23:16];
  assign pix_g = s_axis_tdata[7:0];
  assign pix_b = s_axis_tdata[15:8];

  // Sample en on an accepted start-of-frame beat and hold it for the frame.
  reg  en_act;
  wire sof;
  wire en_use;

  assign sof = accept & s_axis_tuser;
  assign en_use = sof ? en : en_act;

  always @(posedge axis_aclk) begin
    if (!axis_aresetn) begin
      en_act <= 1'b0;
    end else if (sof) begin
      en_act <= en;
    end
  end

  // Maximum is (77 + 150 + 29) * 255 + 128 = 65408.
  wire [16:0] luma_sum_s2;
  wire [ 7:0] luma_s2;

  assign luma_sum_s2 = {1'b0, rg_sum_s2} + {1'b0, b_round_s2};
  assign luma_s2 = luma_sum_s2[15:8];

  always @(posedge axis_aclk) begin
    if (!axis_aresetn) begin
      valid_s1      <= 1'b0;
      r_mul_s1      <= 16'd0;
      g_mul_s1      <= 16'd0;
      b_mul_s1      <= 16'd0;
      rgb_s1        <= 24'd0;
      en_s1         <= 1'b0;
      user_s1       <= 1'b0;
      last_s1       <= 1'b0;

      valid_s2      <= 1'b0;
      rg_sum_s2     <= 16'd0;
      b_round_s2    <= 16'd0;
      rgb_s2        <= 24'd0;
      en_s2         <= 1'b0;
      user_s2       <= 1'b0;
      last_s2       <= 1'b0;

      m_axis_tvalid <= 1'b0;
      m_axis_tdata  <= 24'd0;
      m_axis_tuser  <= 1'b0;
      m_axis_tlast  <= 1'b0;
    end else begin
      // Stage 3 holds all output fields while the downstream is stalled.
      if (ready_s3) begin
        m_axis_tvalid <= valid_s2;
        if (valid_s2) begin
          m_axis_tdata <= en_s2 ? {luma_s2, luma_s2, luma_s2} : rgb_s2;
          m_axis_tuser <= user_s2;
          m_axis_tlast <= last_s2;
        end
      end

      // Stage 2: partial sums and aligned sideband delay.
      if (ready_s2) begin
        valid_s2 <= valid_s1;
        if (valid_s1) begin
          rg_sum_s2  <= r_mul_s1 + g_mul_s1;
          b_round_s2 <= b_mul_s1 + 16'd128;
          rgb_s2     <= rgb_s1;
          en_s2      <= en_s1;
          user_s2    <= user_s1;
          last_s2    <= last_s1;
        end
      end

      // Stage 1: constant multiplications and input capture.
      if (ready_s1) begin
        valid_s1 <= s_axis_tvalid;
        if (s_axis_tvalid) begin
          r_mul_s1 <= 16'd77  * pix_r;
          g_mul_s1 <= 16'd150 * pix_g;
          b_mul_s1 <= 16'd29  * pix_b;
          rgb_s1   <= s_axis_tdata;
          en_s1    <= en_use;
          user_s1  <= s_axis_tuser;
          last_s1  <= s_axis_tlast;
        end
      end
    end
  end

endmodule
