`timescale 1ns / 1ps

module tb_safety_domain_top;

    localparam integer LOG_SIZE = 16;

    reg         safety_clk;
    reg         mon_clk;
    reg         mon_clk_run;
    reg         safety_resetn;
    reg         wdt_kick;
    reg         can_bus_off;
    reg         fault_clear;
    reg         mon_en;
    reg         wdt_en;
    reg  [15:0] ref_window;
    reg  [31:0] min_count;
    reg  [31:0] max_count;
    reg  [31:0] wdt_timeout;
    reg  [15:0] ts_divider;
    reg  [ 7:0] reset_delay;
    reg  [ 7:0] fault_log_rd_addr;

    wire        reset_req;
    wire        fault_irq;
    wire        safe_state;
    wire [55:0] fault_log_rd_data;

    integer errors;
    integer reset_pulses;
    integer log_index;

    safety_domain_top #(
        .LOG_SIZE(LOG_SIZE)
    ) dut (
        .safety_clk        (safety_clk),
        .mon_clk           (mon_clk),
        .safety_resetn     (safety_resetn),
        .wdt_kick          (wdt_kick),
        .can_bus_off       (can_bus_off),
        .fault_clear       (fault_clear),
        .mon_en            (mon_en),
        .wdt_en            (wdt_en),
        .ref_window        (ref_window),
        .min_count         (min_count),
        .max_count         (max_count),
        .wdt_timeout       (wdt_timeout),
        .ts_divider        (ts_divider),
        .reset_delay       (reset_delay),
        .reset_req         (reset_req),
        .fault_irq         (fault_irq),
        .safe_state        (safe_state),
        .fault_log_rd_addr (fault_log_rd_addr),
        .fault_log_rd_data (fault_log_rd_data)
    );

    initial safety_clk = 1'b0;
    always #5 safety_clk = ~safety_clk;

    initial mon_clk = 1'b0;
    always #1 begin
        if (mon_clk_run)
            mon_clk = ~mon_clk;
        else
            mon_clk = 1'b0;
    end

    always @(posedge safety_clk) begin
        if (reset_req)
            reset_pulses = reset_pulses + 1;
    end

    task check;
        input condition;
        input [8*96-1:0] message;
        begin
            if (condition) begin
                $display("[PASS] %0s", message);
            end else begin
                errors = errors + 1;
                $display("[FAIL] %0s (time=%0t)", message, $time);
            end
        end
    endtask

    task apply_reset;
        begin
            safety_resetn     = 1'b0;
            wdt_kick          = 1'b0;
            can_bus_off       = 1'b0;
            fault_clear       = 1'b0;
            mon_en            = 1'b0;
            wdt_en            = 1'b0;
            fault_log_rd_addr = 8'd0;
            // The RTL RAM has no reset/valid bit. Seed it with valid SEC-DED
            // zero codewords so an unread entry cannot create a false ECC fault.
            for (log_index = 0; log_index < LOG_SIZE; log_index = log_index + 1)
                dut.u_fault_log_memory.bram[log_index] = 64'd0;
            repeat (3) @(posedge safety_clk);
            safety_resetn = 1'b1;
            repeat (4) @(posedge safety_clk);
        end
    endtask

    task pulse_wdt_kick;
        begin
            @(negedge safety_clk);
            wdt_kick = 1'b1;
            @(negedge safety_clk);
            wdt_kick = 1'b0;
        end
    endtask

    task pulse_fault_clear;
        begin
            @(negedge safety_clk);
            fault_clear = 1'b1;
            @(negedge safety_clk);
            fault_clear = 1'b0;
        end
    endtask

    task read_log;
        input [7:0] address;
        begin
            fault_log_rd_addr = address;
            // Fault log memory has a registered/synchronous read port.
            repeat (2) @(posedge safety_clk);
            #1;
        end
    endtask

    task wait_for_reset_pulse;
        input integer max_cycles;
        integer i;
        integer before;
        begin
            before = reset_pulses;
            for (i = 0; (i < max_cycles) && (reset_pulses == before); i = i + 1)
                @(posedge safety_clk);
            check(reset_pulses == before + 1, "critical fault generates one reset request pulse");
        end
    endtask

    initial begin
        errors        = 0;
        reset_pulses  = 0;
        mon_clk_run   = 1'b1;
        safety_resetn = 1'b0;
        wdt_kick      = 1'b0;
        can_bus_off   = 1'b0;
        fault_clear   = 1'b0;
        mon_en        = 1'b0;
        wdt_en        = 1'b0;
        ref_window    = 16'd10;
        min_count     = 32'd35;
        max_count     = 32'd65;
        wdt_timeout   = 32'd6;
        ts_divider    = 16'd2;
        reset_delay   = 8'd3;
        fault_log_rd_addr = 8'd0;

        // ------------------------------------------------------------
        // 1. Normal operation
        // ------------------------------------------------------------
        $display("\n[TEST 1] Normal operation");
        apply_reset();
        mon_en = 1'b1;
        wdt_en = 1'b1;
        repeat (5) begin
            repeat (3) @(posedge safety_clk);
            pulse_wdt_kick();
        end
        check(safe_state === 1'b1, "normal operation keeps safe_state high");
        check(fault_irq === 1'b0, "normal operation keeps fault_irq low");
        check(reset_pulses == 0, "normal operation does not request reset");

        // ------------------------------------------------------------
        // 2. Watchdog timeout and log
        // ------------------------------------------------------------
        $display("\n[TEST 2] Watchdog timeout");
        apply_reset();
        reset_pulses = 0;
        wdt_en = 1'b1;
        wait (dut.wdt_fault === 1'b1);
        repeat (2) @(posedge safety_clk);
        check(safe_state === 1'b0, "watchdog timeout enters critical safe state");
        wait_for_reset_pulse(10);
        read_log(8'd0);
        check(fault_log_rd_data[7:0] === 8'h02, "watchdog fault is recorded in log bit 1");
        pulse_wdt_kick();
        pulse_fault_clear();
        repeat (3) @(posedge safety_clk);
        check(safe_state === 1'b1, "watchdog kick plus fault clear recovers to normal");

        // ------------------------------------------------------------
        // 3. Monitored clock stop
        // ------------------------------------------------------------
        $display("\n[TEST 3] Monitored clock stop");
        apply_reset();
        reset_pulses = 0;
        mon_en = 1'b1;
        repeat (15) @(posedge safety_clk);
        check(dut.clk_fault === 1'b0, "normal monitored clock stays in range");
        mon_clk_run = 1'b0;
        wait (dut.clk_fault === 1'b1);
        repeat (2) @(posedge safety_clk);
        check(safe_state === 1'b0, "stopped monitored clock enters critical state");
        wait_for_reset_pulse(10);
        read_log(8'd0);
        check(fault_log_rd_data[7:0] === 8'h01, "clock fault is recorded in log bit 0");
        mon_clk_run = 1'b1;

        // ------------------------------------------------------------
        // 4. CAN bus-off and clear
        // ------------------------------------------------------------
        $display("\n[TEST 4] CAN bus-off and clear");
        apply_reset();
        reset_pulses = 0;
        @(negedge safety_clk);
        can_bus_off = 1'b1;
        repeat (3) @(posedge safety_clk);
        check(fault_irq === 1'b1, "CAN bus-off asserts fault_irq");
        check(safe_state === 1'b1, "CAN bus-off does not enter critical safe state");
        check(reset_pulses == 0, "CAN bus-off does not request CPU reset");
        pulse_fault_clear();
        check(fault_irq === 1'b1, "active CAN source cannot be cleared");
        @(negedge safety_clk);
        can_bus_off = 1'b0;
        pulse_fault_clear();
        repeat (3) @(posedge safety_clk);
        check(fault_irq === 1'b0, "CAN fault clears after source removal");
        read_log(8'd0);
        check(fault_log_rd_data[7:0] === 8'h04, "CAN fault is recorded in log bit 2");

        // ------------------------------------------------------------
        // 5. ECC single-bit error: correction + warning
        // ------------------------------------------------------------
        $display("\n[TEST 5] ECC single-bit error");
        apply_reset();
        // All-zero is a valid SEC-DED codeword. Flip one protected bit.
        dut.u_fault_log_memory.bram[10] = 64'd0;
        dut.u_fault_log_memory.bram[10][5] = 1'b1;
        fault_log_rd_addr = 8'd10;
        repeat (2) @(posedge safety_clk);
        #1;
        check(dut.sec_ecc_fault === 1'b1, "single-bit ECC error asserts SEC warning");
        check(dut.ded_ecc_fault === 1'b0, "single-bit ECC error does not assert DED");
        check(fault_log_rd_data === 56'd0, "single-bit ECC error is corrected");
        repeat (3) @(posedge safety_clk);
        read_log(8'd0);
        check(fault_log_rd_data[7:0] === 8'h08, "SEC warning is recorded in log bit 3");

        // ------------------------------------------------------------
        // 6. ECC double-bit error: critical + reset
        // ------------------------------------------------------------
        $display("\n[TEST 6] ECC double-bit error");
        apply_reset();
        reset_pulses = 0;
        dut.u_fault_log_memory.bram[11] = 64'd0;
        dut.u_fault_log_memory.bram[11][5] = 1'b1;
        dut.u_fault_log_memory.bram[11][6] = 1'b1;
        fault_log_rd_addr = 8'd11;
        wait (dut.ded_ecc_fault === 1'b1);
        repeat (2) @(posedge safety_clk);
        check(safe_state === 1'b0, "double-bit ECC error enters critical state");
        wait_for_reset_pulse(10);
        read_log(8'd0);
        check(fault_log_rd_data[7:0] === 8'h10, "DED fault is recorded in log bit 4");

        if (errors == 0)
            $display("\n========================================");
        if (errors == 0)
            $display("ALL SAFETY DOMAIN TESTS PASSED");
        else
            $display("\nSAFETY DOMAIN TEST FAILED: %0d error(s)", errors);
        if (errors == 0)
            $display("========================================\n");

        $finish;
    end

    initial begin
        #20000;
        $display("[FAIL] Global simulation timeout");
        $finish;
    end

endmodule
