`timescale 1ns / 1ps

module safety_domain_top #(
    parameter integer LOG_SIZE = 16
) (
    input wire safety_clk,
    input wire mon_clk,
    input wire safety_resetn,

    // ============================================================
    // Control
    // ============================================================

    input wire wdt_kick,
    input wire can_bus_off,
    input wire fault_clear,
    
    (* keep = "true", mark_debug = "true" *) input wire mon_en,
    (* keep = "true", mark_debug = "true" *) input wire wdt_en,

    // ============================================================
    // Register Map Configuration
    // ============================================================

    input wire [15:0] ref_window,
    input wire [31:0] min_count,
    input wire [31:0] max_count,

    input wire [31:0] wdt_timeout,
    input wire [15:0] ts_divider,
    input wire [7:0] reset_delay,

    // ============================================================
    // Fault Status
    // ============================================================

    (* keep = "true", mark_debug = "true" *) output wire reset_req,
    output wire fault_irq,
    output wire safe_state,

    // ============================================================
    // Fault Log Read
    // ============================================================

    input  wire [7:0]  fault_log_rd_addr,
    output wire [55:0] fault_log_rd_data
);

    (* keep = "true", mark_debug = "true" *) wire clk_fault;
    (* keep = "true", mark_debug = "true" *) wire wdt_fault;


    wire [63:0] timestamp;

    // Fault Manager -> ECC Encoder
    wire        fault_log_wr_en;
    wire [7:0]  fault_log_wr_addr;
    wire [55:0] fault_log_payload;

    // ECC Encoder -> Fault Log RAM
    wire [63:0] fault_log_codeword;

    // Fault Log RAM -> ECC Decoder
    wire [63:0] fault_log_rd_codeword;

    (* keep = "true", mark_debug = "true" *) wire sec_ecc_fault;
    (* keep = "true", mark_debug = "true" *) wire ded_ecc_fault;


    // ============================================================
    // Clock Monitor
    // ============================================================

    safety_clock_monitor u_safety_clock_monitor (
        .ref_clk    (safety_clk),
        .mon_clk    (mon_clk),
        .resetn     (safety_resetn),

        .ref_window (ref_window),
        .min_count  (min_count),
        .max_count  (max_count),

        .mon_en     (mon_en),
        .clk_fault  (clk_fault)
    );


    // ============================================================
    // Watchdog
    // ============================================================

    safety_watchdog u_safety_watchdog (
        .ref_clk     (safety_clk),
        .resetn      (safety_resetn),

        .wdt_timeout (wdt_timeout),

        .wdt_en      (wdt_en),
        .wdt_kick    (wdt_kick),
        .wdt_fault   (wdt_fault)
    );


    // ============================================================
    // Timestamp Generator
    // ============================================================

    timestamp_generator u_timestamp_generator (
        .safety_clk  (safety_clk),
        .resetn      (safety_resetn),

        .ts_divider  (ts_divider),

        .timestamp   (timestamp)
    );


    // ============================================================
    // Fault Manager
    // ============================================================

    fault_manager #(
        .LOG_SIZE(LOG_SIZE)
    ) u_fault_manager (
        .safety_clk     (safety_clk),
        .resetn         (safety_resetn),

        .reset_delay    (reset_delay),

        .can_bus_off    (can_bus_off),
        .wdt_fault      (wdt_fault),
        .clk_fault      (clk_fault),
        .sec_ecc_fault  (sec_ecc_fault),
        .ded_ecc_fault  (ded_ecc_fault),

        .fault_clear    (fault_clear),
        .timestamp      (timestamp),

        .reset_req      (reset_req),
        .fault_irq      (fault_irq),
        .safe_state     (safe_state),

        .log_wr_en      (fault_log_wr_en),
        .log_wr_addr    (fault_log_wr_addr),
        .log_wr_data    (fault_log_payload)
    );


    // ============================================================
    // SEC-DED Encoder
    // ============================================================

    secded_encoder u_secded_encoder (
        .data_in  (fault_log_payload),
        .code_out (fault_log_codeword)
    );


    // ============================================================
    // Fault Log Memory
    // ============================================================

    fault_log_memory #(
        .LOG_SIZE(LOG_SIZE)
    ) u_fault_log_memory (
        .wr_clk  (safety_clk),
        .wr_en   (fault_log_wr_en),
        .wr_addr (fault_log_wr_addr),
        .wr_data (fault_log_codeword),

        .rd_clk  (safety_clk),
        .rd_addr (fault_log_rd_addr),
        .rd_data (fault_log_rd_codeword)
    );


    // ============================================================
    // SEC-DED Decoder
    // ============================================================

    secded_decoder u_secded_decoder (
        .data_in       (fault_log_rd_codeword),
        .data_out      (fault_log_rd_data),
        .sec_ecc_fault (sec_ecc_fault),
        .ded_ecc_fault (ded_ecc_fault)
    );

endmodule