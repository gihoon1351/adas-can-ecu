`timescale 1ns / 1ps

// safety_clk = 10 MHz
// ts_divider = 10_000 -> timestamp resolution = 1 ms

module timestamp_generator (
    input  wire        safety_clk,
    input  wire        resetn,

    // Register Map Configuration
    input  wire [15:0] ts_divider,

    output reg  [63:0] timestamp
);

    reg [15:0] count;

    // ts_divider가 0이면 최소 1 clock으로 제한
    wire [15:0] ts_divider_value;

    assign ts_divider_value =
        (ts_divider == 16'd0) ? 16'd1 : ts_divider;


    always @(posedge safety_clk or negedge resetn) begin
        if (!resetn) begin
            count     <= 16'd0;
            timestamp <= 64'd0;
        end
        else begin
            if (count >= ts_divider_value - 1'b1) begin
                count     <= 16'd0;
                timestamp <= timestamp + 64'd1;
            end
            else begin
                count <= count + 1'b1;
            end
        end
    end

endmodule