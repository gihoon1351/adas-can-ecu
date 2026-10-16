`timescale 1ns / 1ps

module fault_log_memory #(
    parameter LOG_SIZE = 16
)(
    input wire wr_clk,
    input wire wr_en,
    input wire [$clog2(LOG_SIZE)-1:0] wr_addr,
    input wire [63:0] wr_data,

    input wire rd_clk,
    input wire [$clog2(LOG_SIZE)-1:0] rd_addr,
    output reg [63:0] rd_data
);

reg [63:0] bram [0:LOG_SIZE-1];

    always@(posedge wr_clk) begin
        if(wr_en) begin
            bram[wr_addr] <= wr_data;
        end
    end

    always@(posedge rd_clk) begin
        rd_data <= bram[rd_addr];
    end

endmodule
