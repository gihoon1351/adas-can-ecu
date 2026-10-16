`timescale 1ns / 1ps
// Single-port distributed RAM: 16 banks, each 128 x 32 bits.
// Loading and inference are mutually exclusive in cnn_stream.
// Multiplexing the common read/write address avoids dual-port LUT duplication.
// Outputs during loading are intentionally unused; no RAM reset/init is needed.
// Async read preserves the original ROM-to-PPU timing with zero added BRAM.
module param_ram (
    input wire clk,
    input wire we,
    input wire [6:0] waddr,
    input wire [3:0] wlane,
    input wire [31:0] wdata,
    input wire [6:0] raddr,
    output wire [511:0] rdata
);
    wire [6:0] access_addr = we ? waddr : raddr;
    genvar lane;
    generate for (lane = 0; lane < 16; lane = lane + 1) begin : GEN_LANE
        localparam [3:0] LANE_IDX = lane;
        (* ram_style = "distributed" *) reg [31:0] mem [0:127];
        always @(posedge clk) begin
            if (we && wlane == LANE_IDX)
                mem[access_addr] <= wdata;
        end
        assign rdata[(15-lane)*32 +: 32] = mem[access_addr];
    end endgenerate
endmodule
