`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/10/05 13:52:37
// Design Name: 
// Module Name: cnn_result_box
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


module cnn_result_box #(
    parameter integer FRAME_WIDTH  = 1280,
    parameter integer FRAME_HEIGHT = 720,
    parameter integer CNN_WIDTH    = 224,
    parameter integer CNN_HEIGHT   = 224,
    parameter integer CROP_X       = 600,
    parameter integer CROP_Y       = 0,
    parameter integer CROP_WIDTH   = 224 * 3,
    parameter integer CROP_HEIGHT  = 224 * 3
)(
    input  wire         s_axis_aclk,
    input  wire         s_axis_aresetn,
    input  wire [7:0]   x0,
    input  wire [7:0]   x1,
    input  wire [7:0]   y0,
    input  wire [7:0]   y1,
    input  wire         cfg_en,
    input  wire         cfg_fill,
    input  wire [23:0]  cfg_color,
    input  wire [7:0]   cfg_thickness,
    input  wire [23:0]  s_axis_tdata,
    input  wire         s_axis_tvalid,
    input  wire         s_axis_tuser,
    input  wire         s_axis_tlast,
    output wire         s_axis_tready,
    output wire [23:0]  m_axis_tdata,
    output wire         m_axis_tvalid,
    output wire         m_axis_tuser,
    output wire         m_axis_tlast,
    input  wire         m_axis_tready
);
    // Include the saturated out-of-frame counter value as well as valid pixels.
    localparam integer X_W = (FRAME_WIDTH < 2) ? 1 : $clog2(FRAME_WIDTH+1);
    localparam integer Y_W = (FRAME_HEIGHT < 2) ? 1 : $clog2(FRAME_HEIGHT+1);
    localparam integer FIFO_DEPTH = 8;
    // Inclusive CNN coordinates -> inclusive rectangle within the source crop.
    // Left/top = crop origin + floor(c * crop_size/cnn_size).
    // Right/bottom include the final scaled CNN cell.
    // All divisions below have ONLY elaboration-time operands. Constant
    // lookup wiring prevents synthesis from building arithmetic dividers.
    wire [X_W-1:0] x_left_lut [0:255];
    wire [X_W-1:0] x_right_lut[0:255];
    wire [Y_W-1:0] y_top_lut  [0:255];
    wire [Y_W-1:0] y_bot_lut  [0:255];
    genvar map_idx;
    generate for (map_idx=0; map_idx<256; map_idx=map_idx+1) begin : g_map
        localparam integer CX = (map_idx < CNN_WIDTH) ? map_idx : CNN_WIDTH-1;
        localparam integer CY = (map_idx < CNN_HEIGHT) ? map_idx : CNN_HEIGHT-1;
        localparam integer X_END = ((CX+1)*CROP_WIDTH)/CNN_WIDTH;
        localparam integer Y_END = ((CY+1)*CROP_HEIGHT)/CNN_HEIGHT;
        assign x_left_lut[map_idx] = CROP_X + (CX*CROP_WIDTH)/CNN_WIDTH;
        assign y_top_lut[map_idx] = CROP_Y + (CY*CROP_HEIGHT)/CNN_HEIGHT;
        // Sentinel marks an empty cell when downscaling.
        assign x_right_lut[map_idx] = (X_END==0) ? {X_W{1'b1}} : CROP_X+X_END-1;
        assign y_bot_lut[map_idx] = (Y_END==0) ? {Y_W{1'b1}} : CROP_Y+Y_END-1;
    end endgenerate
    wire [X_W-1:0] scaled_x0 = x_left_lut[x0];
    wire [X_W-1:0] scaled_x1 = x_right_lut[x1];
    wire [Y_W-1:0] scaled_y0 = y_top_lut[y0];
    wire [Y_W-1:0] scaled_y1 = y_bot_lut[y1];
    // Reject reversed boxes and boxes beginning outside the CNN image.
    wire box_valid = (x0 <= x1) && (y0 <= y1) &&
                     (x0 < CNN_WIDTH) && (y0 < CNN_HEIGHT) &&
                     (scaled_x0 <= scaled_x1) && (scaled_y0 <= scaled_y1) &&
                     (scaled_x1 < FRAME_WIDTH) && (scaled_y1 < FRAME_HEIGHT);

    reg [X_W-1:0] active_x0, active_x1;
    reg [Y_W-1:0] active_y0, active_y1;
    reg        active_en, active_fill;
    reg [23:0] active_color;
    reg [7:0]  active_thickness;

    // The next input location. TLAST increments the row; TUSER resynchronizes.
    // Saturation prevents malformed overlong lines/frames from wrapping around.
    reg [X_W-1:0] next_x;
    reg [Y_W-1:0] next_y;
    wire [X_W-1:0] input_x = s_axis_tuser ? {X_W{1'b0}} : next_x;
    wire [Y_W-1:0] input_y = s_axis_tuser ? {Y_W{1'b0}} : next_y;

    // Stage A: input pixel and its screen location.
    reg [23:0] a_data;
    reg [X_W-1:0] a_x;
    reg [Y_W-1:0] a_y;
    reg a_valid, a_user, a_last;

    // Stage B: four parallel distances, already-registered box membership.
    // Color/mode/thickness belong to this pixel's frame, not the newest SOF.
    reg [23:0] b_data, b_color;
    reg [X_W-1:0] b_left, b_right;
    reg [Y_W-1:0] b_top, b_bottom;
    reg [7:0] b_thickness;
    reg b_valid, b_user, b_last, b_inside, b_fill;

    // Stage C: thickness comparisons and final color selection.
    reg [23:0] c_data;
    reg c_valid, c_user, c_last;

    // Each admitted pixel reserves one output FIFO slot immediately. Keeping
    // outstanding = fifo_count + a_valid + b_valid + c_valid <= FIFO_DEPTH
    // lets the pipeline run without any downstream-ready combinational chain.
    reg [3:0] outstanding;
    reg input_ready_q;
    assign s_axis_tready = s_axis_aresetn && input_ready_q;
    wire input_fire = s_axis_tvalid && s_axis_tready;

    // Small distributed FIFO; payload is not reset to allow LUT RAM inference.
    // A write cannot target a stalled valid head: all in-flight pixels already
    // reserve slots, so c_valid cannot be high while fifo_count == FIFO_DEPTH.
    (* ram_style = "distributed" *) reg [25:0] pixel_fifo [0:FIFO_DEPTH-1];
    reg [2:0] read_ptr, write_ptr;
    reg [3:0] fifo_count;
    wire [25:0] fifo_head = pixel_fifo[read_ptr];
    assign m_axis_tdata = fifo_head[23:0];
    assign m_axis_tlast = fifo_head[24];
    assign m_axis_tuser = fifo_head[25];
    assign m_axis_tvalid = s_axis_aresetn && (fifo_count != 0);
    wire output_fire = m_axis_tvalid && m_axis_tready;

    wire a_in_crop = (a_x >= CROP_X) && (a_x < CROP_X+CROP_WIDTH) &&
                     (a_y >= CROP_Y) && (a_y < CROP_Y+CROP_HEIGHT);
    wire a_inside = active_en && a_in_crop &&
                    (a_x < FRAME_WIDTH) && (a_y < FRAME_HEIGHT) &&
                    (a_x >= active_x0) && (a_x <= active_x1) &&
                    (a_y >= active_y0) && (a_y <= active_y1);
    // No subtraction in this stage; all four operands come from B registers.
    wire b_border = (b_left < b_thickness) || (b_right < b_thickness) ||
                    (b_top < b_thickness) || (b_bottom < b_thickness);
    wire b_paint = b_inside && (b_fill || b_border);

    always @(posedge s_axis_aclk) begin
        if (!s_axis_aresetn) begin
            outstanding <= 0;
            input_ready_q <= 1;
            fifo_count <= 0;
            read_ptr <= 0;
            write_ptr <= 0;
        end else begin
            case ({input_fire, output_fire})
                2'b10: begin
                    outstanding <= outstanding + 1'b1;
                    input_ready_q <= (outstanding < FIFO_DEPTH-1);
                end
                2'b01: begin
                    outstanding <= outstanding - 1'b1;
                    input_ready_q <= 1;
                end
                default: begin end
            endcase
            if (c_valid) write_ptr <= write_ptr + 1'b1;
            if (output_fire) read_ptr <= read_ptr + 1'b1;
            case ({c_valid, output_fire})
                2'b10: fifo_count <= fifo_count + 1'b1;
                2'b01: fifo_count <= fifo_count - 1'b1;
                default: begin end
            endcase
        end
    end
    always @(posedge s_axis_aclk) begin
        if (s_axis_aresetn && c_valid)
            pixel_fifo[write_ptr] <= {c_user, c_last, c_data};
    end

    always @(posedge s_axis_aclk) begin
        if (!s_axis_aresetn) begin
            next_x <= 0;
            next_y <= 0;
            active_x0 <= 0; active_x1 <= 0;
            active_y0 <= 0; active_y1 <= 0;
            active_en <= 0; active_fill <= 0;
            active_color <= 24'hFF0000;
            active_thickness <= 1;
            a_valid <= 0;
            b_valid <= 0;
            c_valid <= 0;
        end else begin
            a_valid <= input_fire;
            b_valid <= a_valid;
            c_valid <= b_valid;
            if (b_valid) begin
                c_data <= b_paint ? b_color : b_data;
                c_user <= b_user;
                c_last <= b_last;
            end
            // On the edge accepting a new SOF, A still holds the last pixel
            // from the old frame. NBA ordering uses the OLD active_* here.
            if (a_valid) begin
                b_data <= a_data;
                b_user <= a_user;
                b_last <= a_last;
                b_left <= a_x - active_x0;
                b_right <= active_x1 - a_x;
                b_top <= a_y - active_y0;
                b_bottom <= active_y1 - a_y;
                b_inside <= a_inside;
                b_color <= active_color;
                b_fill <= active_fill;
                b_thickness <= active_thickness;
            end
            if (input_fire) begin
                a_data <= s_axis_tdata;
                a_x <= input_x;
                a_y <= input_y;
                a_user <= s_axis_tuser;
                a_last <= s_axis_tlast;
                if (s_axis_tuser) begin
                    active_x0 <= scaled_x0; active_x1 <= scaled_x1;
                    active_y0 <= scaled_y0; active_y1 <= scaled_y1;
                    active_en <= cfg_en && box_valid;
                    active_fill <= cfg_fill;
                    active_color <= cfg_color;
                    active_thickness <= (cfg_thickness == 0) ? 8'd1 : cfg_thickness;
                end
                if (s_axis_tlast) begin
                    next_x <= 0;
                    next_y <= (input_y < FRAME_HEIGHT) ? input_y + 1'b1 : input_y;
                end else begin
                    next_x <= (input_x < FRAME_WIDTH) ? input_x + 1'b1 : input_x;
                    next_y <= input_y;
                end
            end
        end
    end

    // synthesis translate_off
    initial begin
        if (FRAME_WIDTH < 1 || FRAME_WIDTH > 65535 ||
            FRAME_HEIGHT < 1 || FRAME_HEIGHT > 65535 ||
            CNN_WIDTH < 1 || CNN_WIDTH > 256 ||
            CNN_HEIGHT < 1 || CNN_HEIGHT > 256)
            $fatal(1, "Unsupported video/CNN dimensions");
        if (CROP_X < 0 || CROP_Y < 0 || CROP_WIDTH < 1 || CROP_HEIGHT < 1 ||
            CROP_X+CROP_WIDTH > FRAME_WIDTH || CROP_Y+CROP_HEIGHT > FRAME_HEIGHT)
            $fatal(1, "Crop must be positive-sized and fully inside the video frame");
    end
    // synthesis translate_on
endmodule

module cnn_result_box_v1_0 #(
    parameter integer FRAME_WIDTH = 1280,
    parameter integer FRAME_HEIGHT = 720,
    parameter integer CNN_WIDTH = 224,
    parameter integer CNN_HEIGHT = 224,
    parameter integer C_S00_AXI_ADDR_WIDTH = 5,
    parameter integer CROP_X = 600,
    parameter integer CROP_Y = 0,
    parameter integer CROP_WIDTH = 224 * 3,
    parameter integer CROP_HEIGHT = 224 * 3
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s00_axi_aclk CLK",
       X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S00_AXI:S_AXIS:M_AXIS, ASSOCIATED_RESET s00_axi_aresetn" *)
    input wire s00_axi_aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s00_axi_aresetn RST",
       X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input wire s00_axi_aresetn,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI AWADDR" *)
    input wire [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI AWPROT" *)
    input wire [2:0] s00_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI AWVALID" *)
    input wire s00_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI AWREADY" *)
    output wire s00_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI WDATA" *)
    input wire [31:0] s00_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI WSTRB" *)
    input wire [3:0] s00_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI WVALID" *)
    input wire s00_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI WREADY" *)
    output wire s00_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI BRESP" *)
    output wire [1:0] s00_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI BVALID" *)
    output wire s00_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI BREADY" *)
    input wire s00_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI ARADDR" *)
    input wire [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI ARPROT" *)
    input wire [2:0] s00_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI ARVALID" *)
    input wire s00_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI ARREADY" *)
    output wire s00_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI RDATA" *)
    output wire [31:0] s00_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI RRESP" *)
    output wire [1:0] s00_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI RVALID" *)
    output wire s00_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S00_AXI RREADY" *)
    input wire s00_axi_rready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    input wire [23:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *)
    input wire s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TUSER" *)
    input wire s_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *)
    input wire s_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *)
    output wire s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    output wire [23:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
    output wire m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TUSER" *)
    output wire m_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
    output wire m_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
    input wire m_axis_tready
);
    wire cfg_en, cfg_fill;
    wire [7:0] x0, x1, y0, y1, thickness;
    wire [23:0] color;
    cnn_result_box_v1_0_S00_AXI #(
        .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH),
        .FRAME_WIDTH(FRAME_WIDTH), .FRAME_HEIGHT(FRAME_HEIGHT),
        .CNN_WIDTH(CNN_WIDTH), .CNN_HEIGHT(CNN_HEIGHT)
    ) regs (
        .S_AXI_ACLK(s00_axi_aclk), .S_AXI_ARESETN(s00_axi_aresetn),
        .S_AXI_AWADDR(s00_axi_awaddr), .S_AXI_AWPROT(s00_axi_awprot),
        .S_AXI_AWVALID(s00_axi_awvalid), .S_AXI_AWREADY(s00_axi_awready),
        .S_AXI_WDATA(s00_axi_wdata), .S_AXI_WSTRB(s00_axi_wstrb),
        .S_AXI_WVALID(s00_axi_wvalid), .S_AXI_WREADY(s00_axi_wready),
        .S_AXI_BRESP(s00_axi_bresp), .S_AXI_BVALID(s00_axi_bvalid), .S_AXI_BREADY(s00_axi_bready),
        .S_AXI_ARADDR(s00_axi_araddr), .S_AXI_ARPROT(s00_axi_arprot),
        .S_AXI_ARVALID(s00_axi_arvalid), .S_AXI_ARREADY(s00_axi_arready),
        .S_AXI_RDATA(s00_axi_rdata), .S_AXI_RRESP(s00_axi_rresp),
        .S_AXI_RVALID(s00_axi_rvalid), .S_AXI_RREADY(s00_axi_rready),
        .cfg_en(cfg_en), .cfg_fill(cfg_fill),
        .cfg_x0(x0), .cfg_x1(x1), .cfg_y0(y0), .cfg_y1(y1),
        .cfg_color(color), .cfg_thickness(thickness)
    );
    cnn_result_box #(
        .FRAME_WIDTH(FRAME_WIDTH), .FRAME_HEIGHT(FRAME_HEIGHT),
        .CNN_WIDTH(CNN_WIDTH), .CNN_HEIGHT(CNN_HEIGHT),
        .CROP_X(CROP_X), .CROP_Y(CROP_Y),
        .CROP_WIDTH(CROP_WIDTH), .CROP_HEIGHT(CROP_HEIGHT)
    ) core (
        .s_axis_aclk(s00_axi_aclk), .s_axis_aresetn(s00_axi_aresetn),
        .x0(x0), .x1(x1), .y0(y0), .y1(y1),
        .cfg_en(cfg_en), .cfg_fill(cfg_fill), .cfg_color(color), .cfg_thickness(thickness),
        .s_axis_tdata(s_axis_tdata), .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tuser(s_axis_tuser), .s_axis_tlast(s_axis_tlast), .s_axis_tready(s_axis_tready),
        .m_axis_tdata(m_axis_tdata), .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tuser(m_axis_tuser), .m_axis_tlast(m_axis_tlast), .m_axis_tready(m_axis_tready)
    );
endmodule

module cnn_result_box_v1_0_S00_AXI #(
    parameter integer C_S_AXI_ADDR_WIDTH = 5,
    parameter integer FRAME_WIDTH = 1280,
    parameter integer FRAME_HEIGHT = 720,
    parameter integer CNN_WIDTH = 224,
    parameter integer CNN_HEIGHT = 224
)(
    input  wire S_AXI_ACLK,
    input  wire S_AXI_ARESETN,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_AWADDR,
    input  wire [2:0] S_AXI_AWPROT,
    input  wire S_AXI_AWVALID,
    output wire S_AXI_AWREADY,
    input  wire [31:0] S_AXI_WDATA,
    input  wire [3:0] S_AXI_WSTRB,
    input  wire S_AXI_WVALID,
    output wire S_AXI_WREADY,
    output reg [1:0] S_AXI_BRESP,
    output reg S_AXI_BVALID,
    input  wire S_AXI_BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_ARADDR,
    input  wire [2:0] S_AXI_ARPROT,
    input  wire S_AXI_ARVALID,
    output wire S_AXI_ARREADY,
    output reg [31:0] S_AXI_RDATA,
    output reg [1:0] S_AXI_RRESP,
    output reg S_AXI_RVALID,
    input  wire S_AXI_RREADY,
    output reg cfg_en,
    output reg cfg_fill,
    output reg [7:0] cfg_x0, cfg_x1, cfg_y0, cfg_y1,
    output reg [23:0] cfg_color,
    output reg [7:0] cfg_thickness
);
    localparam [31:0] VIDEO_SIZE = (FRAME_HEIGHT << 16) | FRAME_WIDTH;
    localparam [31:0] CNN_SIZE = (CNN_HEIGHT << 16) | CNN_WIDTH;
    reg [31:0] ctrl_shadow, coords_shadow, color_shadow, thickness_shadow;
    reg aw_pending, w_pending;
    reg [C_S_AXI_ADDR_WIDTH-1:0] awaddr_q;
    reg [31:0] wdata_q;
    reg [3:0] wstrb_q;
    assign S_AXI_AWREADY = S_AXI_ARESETN && !aw_pending && !S_AXI_BVALID;
    assign S_AXI_WREADY  = S_AXI_ARESETN && !w_pending  && !S_AXI_BVALID;
    assign S_AXI_ARREADY = S_AXI_ARESETN && !S_AXI_RVALID;

    function [31:0] merge_bytes;
        input [31:0] old_value, new_value;
        input [3:0] strobe;
        integer i;
        begin
            merge_bytes = old_value;
            for (i = 0; i < 4; i = i + 1)
                if (strobe[i]) merge_bytes[8*i +: 8] = new_value[8*i +: 8];
        end
    endfunction

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            aw_pending <= 0; w_pending <= 0;
            awaddr_q <= 0; wdata_q <= 0; wstrb_q <= 0;
            S_AXI_BVALID <= 0; S_AXI_BRESP <= 0;
            ctrl_shadow <= 0;
            coords_shadow <= 0;
            color_shadow <= 32'h00FF0000;
            thickness_shadow <= 32'd2;
            cfg_en <= 0; cfg_fill <= 0;
            cfg_x0 <= 0; cfg_x1 <= 0; cfg_y0 <= 0; cfg_y1 <= 0;
            cfg_color <= 24'hFF0000;
            cfg_thickness <= 8'd2;
        end else begin
            if (S_AXI_BVALID && S_AXI_BREADY) S_AXI_BVALID <= 0;
            if (S_AXI_AWVALID && S_AXI_AWREADY) begin
                aw_pending <= 1;
                awaddr_q <= S_AXI_AWADDR;
            end
            if (S_AXI_WVALID && S_AXI_WREADY) begin
                w_pending <= 1;
                wdata_q <= S_AXI_WDATA;
                wstrb_q <= S_AXI_WSTRB;
            end
            if (aw_pending && w_pending && !S_AXI_BVALID) begin
                aw_pending <= 0;
                w_pending <= 0;
                S_AXI_BVALID <= 1;
                S_AXI_BRESP <= 2'b00;
                case (awaddr_q)
                    'h00: ctrl_shadow <= merge_bytes(ctrl_shadow, wdata_q, wstrb_q) & 32'h3;
                    'h04: coords_shadow <= merge_bytes(coords_shadow, wdata_q, wstrb_q);
                    'h08: color_shadow <= merge_bytes(color_shadow, wdata_q, wstrb_q) & 32'h00FFFFFF;
                    'h0C: thickness_shadow <= merge_bytes(thickness_shadow, wdata_q, wstrb_q) & 32'hFF;
                    'h10: if (wstrb_q[0] && wdata_q[0]) begin
                        cfg_en <= ctrl_shadow[0];
                        cfg_fill <= ctrl_shadow[1];
                        cfg_x0 <= coords_shadow[7:0];
                        cfg_x1 <= coords_shadow[15:8];
                        cfg_y0 <= coords_shadow[23:16];
                        cfg_y1 <= coords_shadow[31:24];
                        cfg_color <= color_shadow[23:0];
                        cfg_thickness <= thickness_shadow[7:0];
                    end
                    default: S_AXI_BRESP <= 2'b10; // SLVERR: RO, unmapped or unaligned
                endcase
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_RVALID <= 0;
            S_AXI_RDATA <= 0;
            S_AXI_RRESP <= 0;
        end else begin
            if (S_AXI_RVALID && S_AXI_RREADY) S_AXI_RVALID <= 0;
            if (S_AXI_ARVALID && S_AXI_ARREADY) begin
                S_AXI_RVALID <= 1;
                S_AXI_RRESP <= 2'b00;
                case (S_AXI_ARADDR)
                    'h00: S_AXI_RDATA <= ctrl_shadow;
                    'h04: S_AXI_RDATA <= coords_shadow;
                    'h08: S_AXI_RDATA <= color_shadow;
                    'h0C: S_AXI_RDATA <= thickness_shadow;
                    'h10: S_AXI_RDATA <= 0; // COMMIT is a write-one command, not busy
                    'h14: S_AXI_RDATA <= VIDEO_SIZE;
                    'h18: S_AXI_RDATA <= CNN_SIZE;
                    'h1C: S_AXI_RDATA <= 32'h00010000; // interface version 1.0
                    default: begin
                        S_AXI_RDATA <= 0;
                        S_AXI_RRESP <= 2'b10;
                    end
                endcase
            end
        end
    end
    // AWPROT/ARPROT are accepted and ignored: no access-level distinction.
    // synthesis translate_off
    initial if (C_S_AXI_ADDR_WIDTH < 5)
        $fatal(1, "C_S_AXI_ADDR_WIDTH must be >= 5");
    // synthesis translate_on
endmodule
