`timescale 1 ns / 1 ps

module finder_see #(
    parameter integer WIDTH      = 1280,
    parameter integer HEIGHT     = 720,
    parameter integer LINE_WIDTH = 4,
    parameter [23:0]  RED_COLOR  = 24'hFF0000
)(
    input  wire        axis_aclk,
    input  wire        axis_aresetn,

    input  wire [23:0] s_axis_tdata,
    input  wire        s_axis_tlast,
    input  wire        s_axis_tvalid,
    input  wire        s_axis_tuser,
    output wire        s_axis_tready,

    input  wire [10:0] crop_x_up,
    input  wire [10:0] crop_x_dn,
    input  wire [10:0] center_x_up,
    input  wire [10:0] center_x_dn,
    input  wire [9:0]  up_srt,
    input  wire [9:0]  dn_srt,
    input  wire [2:0]  up_size,
    input  wire [2:0]  dn_size,
    input  wire        en,

    output wire [23:0] m_axis_tdata,
    output wire        m_axis_tlast,
    output wire        m_axis_tvalid,
    output wire        m_axis_tuser,
    input  wire        m_axis_tready
);

    localparam integer LINE_LAST =
        (LINE_WIDTH > 0) ? (LINE_WIDTH - 1) : 0;

    wire accept;

    reg [10:0] cnt_x;
    reg [9:0]  cnt_y;

    /* Precalculated ROI boundaries. Updated only once at SOF. */
    reg [11:0] x_start_up_r;
    reg [11:0] x_end_up_r;
    reg [11:0] x_left_end_up_r;
    reg [11:0] x_right_start_up_r;
    reg [10:0] y_start_up_r;
    reg [10:0] y_end_up_r;
    reg [10:0] y_top_end_up_r;
    reg [10:0] y_bottom_start_up_r;

    reg [11:0] x_start_dn_r;
    reg [11:0] x_end_dn_r;
    reg [11:0] x_left_end_dn_r;
    reg [11:0] x_right_start_dn_r;
    reg [10:0] y_start_dn_r;
    reg [10:0] y_end_dn_r;
    reg [10:0] y_top_end_dn_r;
    reg [10:0] y_bottom_start_dn_r;
    reg        en_frame_r;

    /* Configuration-side calculations are not in the per-pixel data path. */
    wire [11:0] cfg_x_start_up;
    wire [11:0] cfg_x_end_up;
    wire [11:0] cfg_x_start_dn;
    wire [11:0] cfg_x_end_dn;
    wire [10:0] cfg_up_rows;
    wire [10:0] cfg_dn_rows;
    wire [10:0] cfg_y_start_up;
    wire [10:0] cfg_y_end_up;
    wire [10:0] cfg_y_start_dn;
    wire [10:0] cfg_y_end_dn;

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

    assign cfg_x_start_up = clamp_roi_start(center_x_up, crop_x_up);
    assign cfg_x_end_up   = (crop_x_up == 0) ? cfg_x_start_up :
                            (cfg_x_start_up + {1'b0, crop_x_up} - 12'd1);
    assign cfg_x_start_dn = clamp_roi_start(center_x_dn, crop_x_dn);
    assign cfg_x_end_dn   = (crop_x_dn == 0) ? cfg_x_start_dn :
                            (cfg_x_start_dn + {1'b0, crop_x_dn} - 12'd1);

    /* Keep the displayed boxes identical to the detector's ROI decoding. */
    assign cfg_up_rows    = (up_size == 3'd5) ? 11'd40 :
                            (11'd1 << up_size);
    assign cfg_dn_rows    = (dn_size == 3'd5) ? 11'd40 :
                            (11'd1 << dn_size);
    assign cfg_y_start_up = {1'b0, up_srt};
    assign cfg_y_end_up   = cfg_y_start_up + cfg_up_rows - 11'd1;

    assign cfg_y_start_dn = ({1'b0, dn_srt} <= cfg_y_end_up) ?
                            (cfg_y_end_up + 11'd1) :
                            {1'b0, dn_srt};
    assign cfg_y_end_dn   = cfg_y_start_dn + cfg_dn_rows - 11'd1;

    /* Current input pixel position. */
    wire [10:0] pixel_x;
    wire [9:0]  pixel_y;
    wire [11:0] pixel_x_ext;
    wire [10:0] pixel_y_ext;

    assign pixel_x     = s_axis_tuser ? 11'd0 : cnt_x;
    assign pixel_y     = s_axis_tuser ? 10'd0 : cnt_y;
    assign pixel_x_ext = {1'b0, pixel_x};
    assign pixel_y_ext = {1'b0, pixel_y};

    /* Per-pixel path: registered boundaries plus comparisons only. */
    wire inside_up;
    wire inside_dn;
    wire border_up;
    wire border_dn;
    wire draw_red_now;

    assign inside_up = (pixel_x_ext >= x_start_up_r) &&
                       (pixel_x_ext <= x_end_up_r) &&
                       (pixel_y_ext >= y_start_up_r) &&
                       (pixel_y_ext <= y_end_up_r);

    assign inside_dn = (pixel_x_ext >= x_start_dn_r) &&
                       (pixel_x_ext <= x_end_dn_r) &&
                       (pixel_y_ext >= y_start_dn_r) &&
                       (pixel_y_ext <= y_end_dn_r);

    assign border_up = inside_up &&
                       ((pixel_x_ext <= x_left_end_up_r) ||
                        (pixel_x_ext >= x_right_start_up_r) ||
                        (pixel_y_ext <= y_top_end_up_r) ||
                        (pixel_y_ext >= y_bottom_start_up_r));

    assign border_dn = inside_dn &&
                       ((pixel_x_ext <= x_left_end_dn_r) ||
                        (pixel_x_ext >= x_right_start_dn_r) ||
                        (pixel_y_ext <= y_top_end_dn_r) ||
                        (pixel_y_ext >= y_bottom_start_dn_r));

    assign draw_red_now = en_frame_r && (border_up || border_dn);

    /* Two-stage elastic pipeline: compare first, color-select second. */
    reg [23:0] data_s1;
    reg        last_s1;
    reg        user_s1;
    reg        valid_s1;
    reg        red_s1;

    reg [23:0] data_s2;
    reg        last_s2;
    reg        user_s2;
    reg        valid_s2;

    wire ready_s2;
    wire ready_s1;

    assign ready_s2      = (!valid_s2) || m_axis_tready;
    assign ready_s1      = (!valid_s1) || ready_s2;
    assign s_axis_tready = ready_s1;
    assign accept        = s_axis_tvalid && s_axis_tready;

    assign m_axis_tdata  = data_s2;
    assign m_axis_tlast  = last_s2;
    assign m_axis_tvalid = valid_s2;
    assign m_axis_tuser  = user_s2;

    always @(posedge axis_aclk or negedge axis_aresetn) begin
        if (!axis_aresetn) begin
            data_s1  <= 24'd0;
            last_s1  <= 1'b0;
            user_s1  <= 1'b0;
            valid_s1 <= 1'b0;
            red_s1   <= 1'b0;
        end else if (ready_s1) begin
            valid_s1 <= s_axis_tvalid;

            if (s_axis_tvalid) begin
                data_s1 <= s_axis_tdata;
                last_s1 <= s_axis_tlast;
                user_s1 <= s_axis_tuser;
                red_s1  <= draw_red_now;
            end
        end
    end

    always @(posedge axis_aclk or negedge axis_aresetn) begin
        if (!axis_aresetn) begin
            data_s2  <= 24'd0;
            last_s2  <= 1'b0;
            user_s2  <= 1'b0;
            valid_s2 <= 1'b0;
        end else if (ready_s2) begin
            valid_s2 <= valid_s1;

            if (valid_s1) begin
                data_s2 <= red_s1 ? RED_COLOR : data_s1;
                last_s2 <= last_s1;
                user_s2 <= user_s1;
            end
        end
    end

    /* AXIS coordinates advance only for accepted input pixels. */
    always @(posedge axis_aclk or negedge axis_aresetn) begin
        if (!axis_aresetn) begin
            cnt_x <= 11'd0;
            cnt_y <= 10'd0;
        end else if (accept) begin
            if (s_axis_tuser) begin
                cnt_y <= 10'd0;
                cnt_x <= s_axis_tlast ? 11'd0 : 11'd1;
            end else if (s_axis_tlast) begin
                cnt_y <= cnt_y + 10'd1;
                cnt_x <= 11'd0;
            end else begin
                cnt_x <= cnt_x + 11'd1;
            end
        end
    end

    /* Calculate all line boundaries once per frame. */
    always @(posedge axis_aclk or negedge axis_aresetn) begin
        if (!axis_aresetn) begin
            x_start_up_r        <= 12'd0;
            x_end_up_r          <= 12'd0;
            x_left_end_up_r     <= 12'd0;
            x_right_start_up_r  <= 12'd0;
            y_start_up_r        <= 11'd0;
            y_end_up_r          <= 11'd0;
            y_top_end_up_r      <= 11'd0;
            y_bottom_start_up_r <= 11'd0;
            x_start_dn_r        <= 12'd0;
            x_end_dn_r          <= 12'd0;
            x_left_end_dn_r     <= 12'd0;
            x_right_start_dn_r  <= 12'd0;
            y_start_dn_r        <= 11'd0;
            y_end_dn_r          <= 11'd0;
            y_top_end_dn_r      <= 11'd0;
            y_bottom_start_dn_r <= 11'd0;
            en_frame_r          <= 1'b0;
        end else if (accept && s_axis_tuser) begin
            x_start_up_r    <= cfg_x_start_up;
            x_end_up_r      <= cfg_x_end_up;
            x_left_end_up_r <= cfg_x_start_up + LINE_LAST;
            y_start_up_r    <= cfg_y_start_up;
            y_end_up_r      <= cfg_y_end_up;
            y_top_end_up_r  <= cfg_y_start_up + LINE_LAST;

            x_start_dn_r    <= cfg_x_start_dn;
            x_end_dn_r      <= cfg_x_end_dn;
            x_left_end_dn_r <= cfg_x_start_dn + LINE_LAST;
            y_start_dn_r    <= cfg_y_start_dn;
            y_end_dn_r      <= cfg_y_end_dn;
            y_top_end_dn_r  <= cfg_y_start_dn + LINE_LAST;

            if (crop_x_up >= LINE_WIDTH) begin
                x_right_start_up_r <= cfg_x_end_up - LINE_LAST;
            end else begin
                x_right_start_up_r <= cfg_x_start_up;
            end

            if (crop_x_dn >= LINE_WIDTH) begin
                x_right_start_dn_r <= cfg_x_end_dn - LINE_LAST;
            end else begin
                x_right_start_dn_r <= cfg_x_start_dn;
            end

            if (cfg_up_rows >= LINE_WIDTH) begin
                y_bottom_start_up_r <= cfg_y_end_up - LINE_LAST;
            end else begin
                y_bottom_start_up_r <= cfg_y_start_up;
            end

            if (cfg_dn_rows >= LINE_WIDTH) begin
                y_bottom_start_dn_r <= cfg_y_end_dn - LINE_LAST;
            end else begin
                y_bottom_start_dn_r <= cfg_y_start_dn;
            end

            en_frame_r <= en;
        end
    end

    /* Retain HEIGHT as an interface-consistency parameter. */
    wire unused_height;
    assign unused_height = (HEIGHT == 0);

endmodule
