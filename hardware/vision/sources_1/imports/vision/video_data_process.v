`timescale 1ns / 1ps

module video_data_process #(
    parameter integer IN_W   = 1280,
    parameter integer IN_H   = 720,
    parameter integer OUT_W  = 224,
    parameter integer OUT_H  = 224,
    parameter integer CROP_X = (IN_W - 3*OUT_W)/2, // 304
    parameter integer CROP_Y = (IN_H - 3*OUT_H)/2  // 24
) (
    input  wire        axis_aclk,
    input  wire        axis_aresetn,

    input  wire [23:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tuser, // sof
    input  wire        s_axis_tlast, // eol

    output reg  [23:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tuser, // sof 224x224
    output reg         m_axis_tlast  // eol 224-pixel
);
  localparam integer XW = $clog2(IN_W + 1);
  localparam integer YW = $clog2(IN_H + 1);
  // Default 1280x720 -> 224x224 sampling coordinates:
  // X: 305, 308, ..., 974 / Y: 25, 28, ..., 694 (224 points each).
  localparam integer FIRST_X = CROP_X + 1;
  localparam integer FIRST_Y = CROP_Y + 1;
  localparam integer LAST_X  = CROP_X + 3*(OUT_W-1) + 1;
  localparam integer LAST_Y  = CROP_Y + 3*(OUT_H-1) + 1;
  localparam integer X_PHASE = FIRST_X % 3;
  localparam integer Y_PHASE = FIRST_Y % 3;

  reg [XW-1:0] x; // 현재 x 좌표
  reg [YW-1:0] y; // 현재 y 좌표를 나타냄
  reg [1:0] x_mod3;
  reg [1:0] y_mod3;

  wire [XW-1:0] pixel_x = s_axis_tuser ? {XW{1'b0}} : x;
  wire [YW-1:0] pixel_y = s_axis_tuser ? {YW{1'b0}} : y;
  wire [1:0] pixel_x_mod3 = s_axis_tuser ? 2'd0 : x_mod3;
  wire [1:0] pixel_y_mod3 = s_axis_tuser ? 2'd0 : y_mod3;

  wire keep_pixel =
      (pixel_x >= FIRST_X) && (pixel_x <= LAST_X) &&
      (pixel_y >= FIRST_Y) && (pixel_y <= LAST_Y) &&
      (pixel_x_mod3 == X_PHASE) && (pixel_y_mod3 == Y_PHASE);

  // Discarded source pixels need no room in the CNN output register.
  // A selected pixel waits if the previous selected pixel is still pending.
  assign s_axis_tready = axis_aresetn &&
                         (!keep_pixel || !m_axis_tvalid || m_axis_tready);

  wire in_fire = s_axis_tvalid && s_axis_tready;

  always @(posedge axis_aclk) begin
    if (!axis_aresetn) begin
      x             <= {XW{1'b0}};
      y             <= {YW{1'b0}};
      x_mod3        <= 2'd0;
      y_mod3        <= 2'd0;
      m_axis_tdata  <= 24'd0;
      m_axis_tvalid <= 1'b0;
      m_axis_tuser  <= 1'b0;
      m_axis_tlast  <= 1'b0;
    end else begin
      if (!m_axis_tvalid || m_axis_tready) begin
        m_axis_tvalid <= 1'b0;
        if (in_fire && keep_pixel) begin // video crop
          m_axis_tdata  <= s_axis_tdata;
          m_axis_tvalid <= 1'b1;
          m_axis_tuser  <= (pixel_x == FIRST_X) && (pixel_y == FIRST_Y);
          m_axis_tlast  <= (pixel_x == LAST_X);
        end
      end

    
      if (in_fire) begin
        if (s_axis_tuser) begin
          x      <= {{(XW-1){1'b0}}, 1'b1}; 
          y      <= {YW{1'b0}};
          x_mod3 <= 2'd1;
          y_mod3 <= 2'd0;
        end else if (s_axis_tlast) begin //eol이면 다음 줄로
          x      <= {XW{1'b0}};
          y      <= y + 1'b1;
          x_mod3 <= 2'd0;
          y_mod3 <= (y_mod3 == 2'd2) ? 2'd0 : y_mod3 + 1'b1; // 0, 1, 2 반복
        end else begin
          x      <= x + 1'b1;
          x_mod3 <= (x_mod3 == 2'd2) ? 2'd0 : x_mod3 + 1'b1; // 0, 1, 2 반복
        end
      end
    end
  end
endmodule
