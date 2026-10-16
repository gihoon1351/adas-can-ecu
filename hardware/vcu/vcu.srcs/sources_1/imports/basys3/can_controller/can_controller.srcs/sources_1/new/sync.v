`timescale 1ns / 1ps

module sync (
    input  wire clk,
    input  wire resetn,
    input  wire software_reset,
    input  wire rx,
    output reg  sync2ff,
    output wire falling_edge
);
    reg sync1ff, sync2ff_d;

    always @(posedge clk or negedge resetn) begin
        if (!resetn || software_reset) begin
            sync1ff   <= 1;
            sync2ff   <= 1;
            sync2ff_d <= 1;
        end else begin
            sync1ff   <= rx;
            sync2ff   <= sync1ff;
            sync2ff_d <= sync2ff;
        end
    end
    assign falling_edge = sync2ff_d & ~sync2ff;
endmodule