`timescale 1ns / 1ps

module safety_watchdog (
    input  wire        ref_clk,
    input  wire        resetn,

    input  wire [31:0] wdt_timeout,

    input  wire        wdt_en,
    input  wire        wdt_kick,
    output wire        wdt_fault
);

    reg [31:0] wdt_count;
    reg        wdt_fault_reg;

    wire [31:0] timeout_value;

    // timeout이 0이면 최소 1 clock으로 제한
    assign timeout_value =
        (wdt_timeout == 32'd0) ? 32'd1 : wdt_timeout;

    always @(posedge ref_clk or negedge resetn) begin
        if (!resetn) begin
            wdt_count     <= 32'd0;
            wdt_fault_reg <= 1'b0;
        end
        else if (!wdt_en) begin
            wdt_count     <= 32'd0;
            wdt_fault_reg <= 1'b0;
        end
        else if (wdt_kick) begin
            wdt_count     <= 32'd0;
            wdt_fault_reg <= 1'b0;
        end
        else if (wdt_count >= timeout_value - 1'b1) begin
            wdt_count     <= wdt_count;
            wdt_fault_reg <= 1'b1;
        end
        else begin
            wdt_count     <= wdt_count + 1'b1;
            wdt_fault_reg <= 1'b0;
        end
    end

    assign wdt_fault = wdt_fault_reg;

endmodule