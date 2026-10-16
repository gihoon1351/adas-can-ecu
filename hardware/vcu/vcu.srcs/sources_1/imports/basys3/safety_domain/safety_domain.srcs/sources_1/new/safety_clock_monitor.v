`timescale 1ns / 1ps

module safety_clock_monitor (
    input  wire        ref_clk,     // 기준 clock: safety_clk 10 MHz
    input  wire        mon_clk,     // 감시 대상 clock: function clk 80 MHz
    input  wire        resetn,

    // Register Map Configuration
    input  wire [15:0] ref_window,
    input  wire [31:0] min_count,
    input  wire [31:0] max_count,

    input  wire        mon_en,
    output wire        clk_fault
);

    // ============================================================
    // Configuration Protection
    // ref_window = 0이면 최소 1 clock으로 제한
    // ============================================================

    wire [15:0] ref_window_value;

    assign ref_window_value =
        (ref_window == 16'd0) ? 16'd1 : ref_window;


    // ============================================================
    // 1. Monitor Clock Domain
    // Free-running Binary Counter
    // ============================================================

    reg [31:0] mon_count_bin;

    // resetn is released in the 10 MHz safety domain.  Assert reset
    // asynchronously, but release it through two mon_clk flip-flops so the
    // 80 MHz counter never sees an asynchronous reset deassertion.
    (* ASYNC_REG = "TRUE" *) reg [1:0] mon_resetn_sync;
    wire mon_resetn;

    always @(posedge mon_clk or negedge resetn) begin
        if (!resetn)
            mon_resetn_sync <= 2'b00;
        else
            mon_resetn_sync <= {mon_resetn_sync[0], 1'b1};
    end

    assign mon_resetn = mon_resetn_sync[1];

    always @(posedge mon_clk or negedge mon_resetn) begin
        if (!mon_resetn)
            mon_count_bin <= 32'd0;
        else
            mon_count_bin <= mon_count_bin + 1'b1;
    end


    // ============================================================
    // 2. Binary -> Gray
    // ============================================================

    wire [31:0] mon_count_gray;

    assign mon_count_gray =
        mon_count_bin ^ (mon_count_bin >> 1);


    // ============================================================
    // 3. Gray Code CDC Synchronizer
    // mon_clk -> ref_clk
    // ============================================================

    (* ASYNC_REG = "TRUE" *) reg [31:0] mon_gray_sync1;
    (* ASYNC_REG = "TRUE" *) reg [31:0] mon_gray_sync2;

    always @(posedge ref_clk or negedge resetn) begin
        if (!resetn) begin
            mon_gray_sync1 <= 32'd0;
            mon_gray_sync2 <= 32'd0;
        end
        else begin
            mon_gray_sync1 <= mon_count_gray;
            mon_gray_sync2 <= mon_gray_sync1;
        end
    end


    // ============================================================
    // 4. Gray -> Binary
    // ============================================================

    function [31:0] gray_to_binary;
        input [31:0] gray;
        integer i;

        begin
            gray_to_binary[31] = gray[31];

            for (i = 30; i >= 0; i = i - 1)
                gray_to_binary[i] =
                    gray_to_binary[i+1] ^ gray[i];
        end
    endfunction


    wire [31:0] mon_count_sync;

    assign mon_count_sync =
        gray_to_binary(mon_gray_sync2);


    // ============================================================
    // 5. Reference Window Counter
    // ============================================================

    reg [15:0] ref_count;
    reg [31:0] mon_count_prev;

    wire [31:0] mon_delta;

    assign mon_delta =
        mon_count_sync - mon_count_prev;


    // ============================================================
    // 6. Clock Fault Detection
    // ============================================================

    reg clk_fault_reg;

    always @(posedge ref_clk or negedge resetn) begin
        if (!resetn) begin
            ref_count      <= 16'd0;
            mon_count_prev <= 32'd0;
            clk_fault_reg  <= 1'b0;
        end
        else begin

            if (!mon_en) begin
                ref_count      <= 16'd0;
                mon_count_prev <= mon_count_sync;

                // Sticky fault
                // clk_fault_reg는 reset 전까지 유지
            end

            else begin
                if (ref_count >= ref_window_value - 1'b1) begin

                    ref_count      <= 16'd0;
                    mon_count_prev <= mon_count_sync;

                    if ((mon_delta < min_count) ||
                        (mon_delta > max_count)) begin

                        clk_fault_reg <= 1'b1;
                    end
                end

                else begin
                    ref_count <= ref_count + 1'b1;
                end
            end
        end
    end


    // ============================================================
    // Output
    // ============================================================

    assign clk_fault = clk_fault_reg;

endmodule
