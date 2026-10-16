`timescale 1ns / 1ps

module can_bit_timing (
    // input top
    input wire clk,
    input wire resetn,
    input wire software_reset,
    input wire rx,
    input wire tx,

    // input from axi_lite
    input wire [7:0] axi_lite_brp_can_bit_timing,
    input wire [7:0] axi_lite_tseg1_can_bit_timing,
    input wire [7:0] axi_lite_tseg2_can_bit_timing,
    input wire [7:0] axi_lite_sjw_can_bit_timing,

    // input from can_rx
    input wire can_rx_rx_idle_can_bit_timing,

    // output to can
    output wire can_bit_timing_bit_tick_can,
    output wire can_bit_timing_falling_edge_can,
    output wire can_bit_timing_sample_tick_can,
    output wire can_bit_timing_sample_data_can
);
    wire w_falling_edge;
    wire w_hard_sync;
    wire w_tq_tick;
    wire w_sync2ff;

    wire w_bit_tick;
    wire w_sample_tick;
    wire w_sample_data;

    assign can_bit_timing_bit_tick_can = w_bit_tick;
    assign can_bit_timing_falling_edge_can = w_falling_edge;
    assign can_bit_timing_sample_tick_can = w_sample_tick;
    assign can_bit_timing_sample_data_can = w_sample_data;

    tq_gen u_tq_gen (
        // input top
        .clk           (clk),
        .resetn        (resetn),
        .software_reset(software_reset),
        .brp           (axi_lite_brp_can_bit_timing),
        // input bit_timing
        .hard_sync     (w_hard_sync),
        // output u_bit_timing
        .tq_tick       (w_tq_tick)
    );

    sync u_sync (
        // input top
        .clk           (clk),
        .resetn        (resetn),
        .software_reset(software_reset),
        .rx            (rx),
        // output u_bit_timing
        .sync2ff         (w_sync2ff),
        // output u_bit_timing / output
        .falling_edge  (w_falling_edge)
    );

    bit_timing u_bit_timing (
        // input top
        .clk           (clk),
        .resetn        (resetn),
        .software_reset(software_reset),
        .tseg1        (axi_lite_tseg1_can_bit_timing),
        .tseg2        (axi_lite_tseg2_can_bit_timing),
        .sjw           (axi_lite_sjw_can_bit_timing),
        // input from tq_gen
        .tq_tick       (w_tq_tick),
        // input from rx_sync
        .sync2ff         (w_sync2ff),
        // suppress positive resync on this node's own dominant TX edge
        .tx              (tx),
        // input from can_rx
        .rx_idle       (can_rx_rx_idle_can_bit_timing),
        // input from rx_sync
        .falling_edge  (w_falling_edge),
        // output to top
        .sample_tick   (w_sample_tick),
        .sample_data   (w_sample_data),
        // output to can_tx
        .bit_tick      (w_bit_tick),
        // output to tq_gen
        .hard_sync     (w_hard_sync)
    );

endmodule
