`timescale 1ns / 1ps

//module rgb2ycbcr (
//    input  wire        axis_aclk,
//    input  wire        axis_aresetn,
//    input  wire        en,
//
//    input  wire [23:0] s_axis_tdata,
//    input  wire        s_axis_tvalid,
//    output wire        s_axis_tready,
//    input  wire        s_axis_tuser,  // start of frame
//    input  wire        s_axis_tlast,  // end of line
//
//    output reg  [23:0] m_axis_tdata,
//    output reg         m_axis_tvalid,
//    input  wire        m_axis_tready,
//    output reg         m_axis_tuser,
//    output reg         m_axis_tlast
//);
//
//    // Three-stage elastic pipeline:
//    //   1: signed constant multiplications + input capture
//    //   2: partial sums + rounding/chroma offsets
//    //   3: final sums, arithmetic shift, saturation + output register
//    // Each stage advances only when its downstream stage can accept data.
//    reg valid_s1;
//    reg signed [16:0] y_r_s1,  y_g_s1,  y_b_s1;
//    reg signed [16:0] cb_r_s1, cb_g_s1, cb_b_s1;
//    reg signed [16:0] cr_r_s1, cr_g_s1, cr_b_s1;
//    reg [23:0] rgb_s1;
//    reg en_s1, user_s1, last_s1;
//
//    reg valid_s2;
//    reg signed [17:0] y_rg_s2,  y_b_round_s2;
//    reg signed [17:0] cb_rg_s2, cb_b_offset_s2;
//    reg signed [17:0] cr_rg_s2, cr_b_offset_s2;
//    reg [23:0] rgb_s2;
//    reg en_s2, user_s2, last_s2;
//
//    wire ready_s3, ready_s2, ready_s1;
//    wire accept, sof, en_use;
//    reg en_act;
//
//    assign ready_s3 = (~m_axis_tvalid) | m_axis_tready;
//    assign ready_s2 = (~valid_s2) | ready_s3;
//    assign ready_s1 = (~valid_s1) | ready_s2;
//    // Prevent an apparent input handshake while reset discards all state.
//    assign s_axis_tready = axis_aresetn & ready_s1;
//    assign accept = s_axis_tvalid & s_axis_tready;
//    assign sof = accept & s_axis_tuser;
//    assign en_use = sof ? en : en_act;
//
//    // Zero-extend before interpreting unsigned pixels as signed operands.
//    wire signed [8:0] pix_r = $signed({1'b0, s_axis_tdata[23:16]});
//    wire signed [8:0] pix_g = $signed({1'b0, s_axis_tdata[7:0]});
//    wire signed [8:0] pix_b = $signed({1'b0, s_axis_tdata[15:8]});
//
//    wire signed [17:0] y_sum_s2  = y_rg_s2  + y_b_round_s2;
//    wire signed [17:0] cb_sum_s2 = cb_rg_s2 + cb_b_offset_s2;
//    wire signed [17:0] cr_sum_s2 = cr_rg_s2 + cr_b_offset_s2;
//
//    // Keep the signed result wide until AFTER the range checks.
//    // In particular pure blue/red can round Cb/Cr to 256; clamp to 255.
//    function [7:0] clip_u8;
//        input signed [17:0] value;
//        begin
//            if (value < 18'sd0)
//                clip_u8 = 8'd0;
//            else if (value > 18'sd255)
//                clip_u8 = 8'd255;
//            else
//                clip_u8 = value[7:0];
//        end
//    endfunction
//
//    wire [7:0] y_s2  = clip_u8(y_sum_s2  >>> 8);
//    wire [7:0] cb_s2 = clip_u8(cb_sum_s2 >>> 8);
//    wire [7:0] cr_s2 = clip_u8(cr_sum_s2 >>> 8);
//
//    always @(posedge axis_aclk) begin
//        if (!axis_aresetn)
//            en_act <= 1'b0;
//        else if (sof)
//            en_act <= en;
//    end
//
//    always @(posedge axis_aclk) begin
//        if (!axis_aresetn) begin
//            valid_s1 <= 1'b0;
//            y_r_s1   <= 17'sd0;
//            y_g_s1   <= 17'sd0;
//            y_b_s1   <= 17'sd0;
//            cb_r_s1  <= 17'sd0;
//            cb_g_s1  <= 17'sd0;
//            cb_b_s1  <= 17'sd0;
//            cr_r_s1  <= 17'sd0;
//            cr_g_s1  <= 17'sd0;
//            cr_b_s1  <= 17'sd0;
//            rgb_s1   <= 24'd0;
//            en_s1    <= 1'b0;
//            user_s1  <= 1'b0;
//            last_s1  <= 1'b0;
//
//            valid_s2       <= 1'b0;
//            y_rg_s2        <= 18'sd0;
//            y_b_round_s2   <= 18'sd0;
//            cb_rg_s2       <= 18'sd0;
//            cb_b_offset_s2 <= 18'sd0;
//            cr_rg_s2       <= 18'sd0;
//            cr_b_offset_s2 <= 18'sd0;
//            rgb_s2         <= 24'd0;
//            en_s2          <= 1'b0;
//            user_s2        <= 1'b0;
//            last_s2        <= 1'b0;
//
//            m_axis_tdata  <= 24'd0;
//            m_axis_tvalid <= 1'b0;
//            m_axis_tuser  <= 1'b0;
//            m_axis_tlast  <= 1'b0;
//        end else begin
//            // Hold data and all sidebands together during output stalls.
//            if (ready_s3) begin
//                m_axis_tvalid <= valid_s2;
//                if (valid_s2) begin
//                    m_axis_tdata <= en_s2 ? {y_s2, cb_s2, cr_s2} : rgb_s2;
//                    m_axis_tuser <= user_s2;
//                    m_axis_tlast <= last_s2;
//                end
//            end
//
//            if (ready_s2) begin
//                valid_s2 <= valid_s1;
//                if (valid_s1) begin
//                    // Explicit sign extension before every 18-bit sum.
//                    y_rg_s2 <= $signed({y_r_s1[16], y_r_s1})
//                             + $signed({y_g_s1[16], y_g_s1});
//                    y_b_round_s2 <= $signed({y_b_s1[16], y_b_s1})
//                                  + 18'sd128;
//                    cb_rg_s2 <= $signed({cb_r_s1[16], cb_r_s1})
//                              + $signed({cb_g_s1[16], cb_g_s1});
//                    cb_b_offset_s2 <= $signed({cb_b_s1[16], cb_b_s1})
//                                    + 18'sd32896;
//                    cr_rg_s2 <= $signed({cr_r_s1[16], cr_r_s1})
//                              + $signed({cr_g_s1[16], cr_g_s1});
//                    cr_b_offset_s2 <= $signed({cr_b_s1[16], cr_b_s1})
//                                    + 18'sd32896;
//                    rgb_s2  <= rgb_s1;
//                    en_s2   <= en_s1;
//                    user_s2 <= user_s1;
//                    last_s2 <= last_s1;
//                end
//            end
//
//            if (ready_s1) begin
//                valid_s1 <= s_axis_tvalid;
//                if (s_axis_tvalid) begin
//                    y_r_s1  <=  17'sd77  * pix_r;
//                    y_g_s1  <=  17'sd150 * pix_g;
//                    y_b_s1  <=  17'sd29  * pix_b;
//                    cb_r_s1 <= -17'sd43  * pix_r;
//                    cb_g_s1 <= -17'sd85  * pix_g;
//                    cb_b_s1 <=  17'sd128 * pix_b;
//                    cr_r_s1 <=  17'sd128 * pix_r;
//                    cr_g_s1 <= -17'sd107 * pix_g;
//                    cr_b_s1 <= -17'sd21  * pix_b;
//                    rgb_s1  <= s_axis_tdata;
//                    en_s1   <= en_use;
//                    user_s1 <= s_axis_tuser;
//                    last_s1 <= s_axis_tlast;
//                end
//            end
//        end
//    end
//endmodule
