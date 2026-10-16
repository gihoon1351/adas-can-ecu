`timescale 1ns / 1ps

module can_controller (
    input wire clk,
    input wire resetn,
    input wire axi_lite_sw_reset_can_controller,

    input wire [7:0] axi_lite_brp_can_bit_timing,
    input wire [7:0] axi_lite_tseg1_can_bit_timing,
    input wire [7:0] axi_lite_tseg2_can_bit_timing,
    input wire [7:0] axi_lite_sjw_can_bit_timing,

    // input from cpu
    input wire [10:0] axi_lite_tx_id_can_tx,
    input wire        axi_lite_tx_valid_can_tx,
    input wire        axi_lite_tx_rtr_can_tx,
    input wire        axi_lite_tx_ide_can_tx,
    input wire [ 3:0] axi_lite_tx_dlc_can_tx,
    input wire [63:0] axi_lite_tx_data_can_tx,

    // output to cpu
    output wire can_tx_bus_off_flag_axi_lite,
    output wire can_tx_tx_error_state_flag_axi_lite,
    output wire can_tx_tx_ready_axi_lite,

    // output to axi_lite
    output wire        can_rx_frame_done_axi_lite,
    output wire [10:0] can_rx_rx_id_axi_lite,
    output wire [ 3:0] can_rx_rx_dlc_axi_lite,
    output wire [63:0] can_rx_rx_data_axi_lite,
    output wire        can_rx_rx_rtr_axi_lite,
    output wire        can_rx_rx_ide_axi_lite,

    input  wire rx,
    output wire tx
);


    // can_bit_timing -> can_rx, can_tx
    wire w_can_bit_timing_bit_tick_can;
    wire w_can_bit_timing_falling_edge_can;
    wire w_can_bit_timing_sample_tick_can;
    wire w_can_bit_timing_sample_data_can;

    // can_rx -> can_bit_timing, can_tx
    wire w_can_rx_rx_idle_can_bit_timing;
    wire w_can_rx_ack_out_can_tx;
    wire w_can_rx_rx_error_flag_can_tx;
    wire w_can_rx_rx_rec_passive_can_tx;

    can_bit_timing u_can_bit_timing (
        .clk(clk),
        .resetn(resetn),
        .software_reset(axi_lite_sw_reset_can_controller),

        .rx(rx),
        .tx(tx),
        .axi_lite_brp_can_bit_timing(axi_lite_brp_can_bit_timing),
        .axi_lite_tseg1_can_bit_timing(axi_lite_tseg1_can_bit_timing),
        .axi_lite_tseg2_can_bit_timing(axi_lite_tseg2_can_bit_timing),
        .axi_lite_sjw_can_bit_timing(axi_lite_sjw_can_bit_timing),

        .can_rx_rx_idle_can_bit_timing(w_can_rx_rx_idle_can_bit_timing),

        .can_bit_timing_bit_tick_can(w_can_bit_timing_bit_tick_can),
        .can_bit_timing_falling_edge_can(w_can_bit_timing_falling_edge_can),
        .can_bit_timing_sample_tick_can(w_can_bit_timing_sample_tick_can),
        .can_bit_timing_sample_data_can(w_can_bit_timing_sample_data_can)
    );

    can_rx u_can_rx (
        .clk(clk),
        .resetn(resetn),
        .software_reset(axi_lite_sw_reset_can_controller),

        .tq_gen_falling_edge_can_rx(w_can_bit_timing_falling_edge_can),

        .tq_gen_sample_tick_can_rx(w_can_bit_timing_sample_tick_can),
        .tq_gen_sample_data_can_rx(w_can_bit_timing_sample_data_can),

        .can_tx_bus_off_flag_can_rx(can_tx_bus_off_flag_axi_lite),
        .can_rx_ack_out_can_tx(w_can_rx_ack_out_can_tx),
        .can_rx_rx_error_flag_can_tx(w_can_rx_rx_error_flag_can_tx),
        .can_rx_rx_rec_passive_can_tx(w_can_rx_rx_rec_passive_can_tx),
        .can_rx_rx_idle_tq_gen(w_can_rx_rx_idle_can_bit_timing),

        .can_rx_frame_done_axi_lite(can_rx_frame_done_axi_lite),
        .can_rx_rx_id_axi_lite(can_rx_rx_id_axi_lite),
        .can_rx_rx_dlc_axi_lite(can_rx_rx_dlc_axi_lite),
        .can_rx_rx_data_axi_lite(can_rx_rx_data_axi_lite),
        .can_rx_rx_rtr_axi_lite(can_rx_rx_rtr_axi_lite),
        .can_rx_rx_ide_axi_lite(can_rx_rx_ide_axi_lite)
    );

    can_tx u_can_tx (
        .clk(clk),
        .resetn(resetn),
        .software_reset(axi_lite_sw_reset_can_controller),

        .tx(tx),
        .tq_gen_bit_tick_can_tx(w_can_bit_timing_bit_tick_can),
        .tq_gen_sample_tick_can_tx(w_can_bit_timing_sample_tick_can),
        .tq_gen_sample_data_can_tx(w_can_bit_timing_sample_data_can),

        .axi_lite_tx_id_can_tx(axi_lite_tx_id_can_tx),
        .axi_lite_tx_valid_can_tx(axi_lite_tx_valid_can_tx),
        .axi_lite_tx_rtr_can_tx(axi_lite_tx_rtr_can_tx),
        .axi_lite_tx_ide_can_tx(axi_lite_tx_ide_can_tx),
        .axi_lite_tx_dlc_can_tx(axi_lite_tx_dlc_can_tx),
        .axi_lite_tx_data_can_tx(axi_lite_tx_data_can_tx),

        .can_tx_bus_off_flag_axi_lite(can_tx_bus_off_flag_axi_lite),

        .can_rx_ack_out_can_tx(w_can_rx_ack_out_can_tx),

        .can_tx_tx_error_state_flag_axi_lite(can_tx_tx_error_state_flag_axi_lite),
        .can_tx_tx_ready_axi_lite(can_tx_tx_ready_axi_lite),

        .can_rx_rx_error_flag_can_tx (w_can_rx_rx_error_flag_can_tx),
        .can_rx_rx_rec_passive_can_tx(w_can_rx_rx_rec_passive_can_tx)
    );

endmodule
