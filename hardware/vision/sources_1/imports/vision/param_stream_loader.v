`timescale 1ns / 1ps
// One packet = 2048 bias words followed by 2048 scale words.
// Within each table: row 0..127, lane 0..15 (lane 0 at [511:480]).
// TLAST must accompany word 4095; TKEEP must be 4'hf on every beat.
module param_stream_loader (
    input wire clk,
    input wire resetn,
    input wire cnn_idle,
    input wire abort_load,
    input wire [31:0] s_axis_tdata,
    input wire [3:0] s_axis_tkeep,
    input wire s_axis_tvalid,
    output wire s_axis_tready,
    input wire s_axis_tlast,
    output wire bias_we,
    output wire scale_we,
    output wire [6:0] wr_addr,
    output wire [3:0] wr_lane,
    output wire [31:0] wr_data,
    output reg loaded,
    output reg loading,
    output reg error
);
    reg [11:0] word_count;
    reg dropping;
    wire accept;
    wire last_word;
    wire good_beat;
    assign s_axis_tready = resetn && cnn_idle && !abort_load;
    assign accept = s_axis_tvalid && s_axis_tready;
    assign last_word = (word_count == 12'd4095);
    assign good_beat = accept && !dropping && (s_axis_tkeep == 4'hf)
                       && (s_axis_tlast == last_word);
    assign bias_we = good_beat && !word_count[11];
    assign scale_we = good_beat && word_count[11];
    assign wr_addr = word_count[10:4];
    assign wr_lane = word_count[3:0];
    assign wr_data = s_axis_tdata;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            word_count <= 0;
            dropping <= 0;
            loaded <= 0;
            loading <= 0;
            error <= 0;
        end else if (abort_load && cnn_idle) begin
            word_count <= 0;
            dropping <= 0;
            loaded <= 0;
            loading <= 0;
            error <= 0;
        end else if (accept) begin
            loaded <= 0;
            if (dropping) begin
                // Consume the malformed packet through its actual boundary.
                if (s_axis_tlast) begin
                    dropping <= 0;
                    loading <= 0;
                    word_count <= 0;
                end
            end else if (!good_beat) begin
                error <= 1;
                word_count <= 0;
                dropping <= !s_axis_tlast;
                loading <= !s_axis_tlast;
            end else begin
                if (word_count == 0)
                    error <= 0;
                if (last_word) begin
                    word_count <= 0;
                    loading <= 0;
                    loaded <= 1;
                    error <= 0;
                end else begin
                    word_count <= word_count + 1'b1;
                    loading <= 1;
                end
            end
        end
    end
endmodule
