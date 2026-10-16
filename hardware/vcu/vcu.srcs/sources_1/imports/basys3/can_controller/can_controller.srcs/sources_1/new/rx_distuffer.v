`timescale 1ns / 1ps

module rx_distuffer (
    // input top
    input wire clk,
    input wire resetn,
    input wire software_reset,

    // input from tq_gen
    input wire sample_tick,
    input wire sample_data,

    // input from rx_frame
    input wire destuff_en,

    // output to rx_frame
    output reg data_valid,
    output reg data_in,
    output reg stuff_error
);

    reg data_valid_next;
    reg data_in_next;
    reg stuff_error_next;

    reg [2:0] same_cnt_reg;
    reg [2:0] same_cnt_next;

    reg prev_data_reg;
    reg prev_data_next;

    always @(posedge clk or negedge resetn) begin
        if (!resetn || software_reset) begin
            data_valid    <= 1'b0;
            data_in       <= 1'b0;
            stuff_error   <= 1'b0;
            prev_data_reg <= 1'b1;
            same_cnt_reg  <= 3'd0;
        end else begin
            data_valid    <= data_valid_next;
            data_in       <= data_in_next;
            stuff_error   <= stuff_error_next;
            prev_data_reg <= prev_data_next;
            same_cnt_reg  <= same_cnt_next;
        end
    end

    always @(*) begin
        data_valid_next  = 1'b0;
        data_in_next     = data_in;
        stuff_error_next = 1'b0;
        prev_data_next   = prev_data_reg;
        same_cnt_next    = same_cnt_reg;
        if (!destuff_en) begin
            prev_data_next = 1'b1;
            same_cnt_next  = 3'd0;
            if (sample_tick) begin
                data_valid_next = 1'b1;
                data_in_next    = sample_data;
            end
        end else begin
            if (sample_tick) begin
                if (same_cnt_reg == 3'd4) begin
                    if (sample_data != prev_data_reg) begin
                        data_valid_next = 1'b0;
                        prev_data_next  = sample_data;
                        same_cnt_next   = 3'd0;
                    end else begin
                        data_valid_next = 1'b0;
                        stuff_error_next = 1'b1;
                        prev_data_next = sample_data;
                        same_cnt_next = 3'd0;
                    end
                end else begin
                    data_valid_next = 1'b1;
                    data_in_next    = sample_data;
                    if (sample_data == prev_data_reg) begin
                        same_cnt_next = same_cnt_reg + 3'd1;
                    end else begin
                        same_cnt_next = 3'd0;
                    end
                    prev_data_next = sample_data;
                end
            end
        end
    end
endmodule
