`timescale 1ns / 1ps

module rx_crc15 (
    // input top
    input  wire clk,
    input  wire resetn,
    input  wire software_reset,
    // input from can_distuff
    input  wire data_valid,
    input  wire data_in,
    // intput from can_rx_frame
    input  wire crc_calc_en,
    input  wire crc_rx_en,
    input  wire crc_check_en,
    // output to can_rx_frame
    output reg  crc_error
);
    reg [14:0] crc_calc_reg, crc_calc_next;
    reg [14:0] crc_rx_reg, crc_rx_next;

    reg crc_error_next;

    reg feedback;

    always @(posedge clk or negedge resetn) begin
        if (!resetn || software_reset) begin
            crc_calc_reg <= 15'd0;
            crc_rx_reg   <= 15'd0;
            crc_error    <= 1'b0;
        end else begin
            crc_calc_reg <= crc_calc_next;
            crc_rx_reg   <= crc_rx_next;
            crc_error    <= crc_error_next;
        end
    end

    always @(*) begin
        crc_calc_next  = crc_calc_reg;
        crc_rx_next    = crc_rx_reg;
        crc_error_next = crc_error;
        feedback       = 1'b0;

        if (crc_calc_en) begin
            crc_error_next = 1'b0;
            if (data_valid) begin
                feedback = data_in ^ crc_calc_reg[14];
                crc_calc_next = {crc_calc_reg[13:0], 1'b0};
                if (feedback) begin
                    crc_calc_next = crc_calc_next ^ 15'h4599;
                end
            end
        end else if (crc_rx_en) begin
            if (data_valid) begin
                crc_rx_next = {crc_rx_reg[13:0], data_in};
            end
        end else if (crc_check_en) begin
            if (crc_calc_reg == crc_rx_reg) begin
                crc_error_next = 1'b0;
            end else begin
                crc_error_next = 1'b1;
            end
        end else begin
            crc_calc_next  = 15'd0;
            crc_rx_next    = 15'd0;
            crc_error_next = crc_error;
        end
    end
endmodule
