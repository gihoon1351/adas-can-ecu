`timescale 1ns / 1ps

module can_rx (
    // input top
    input wire clk,
    input wire resetn,
    input wire software_reset,

    // input from tq_gen
    input wire tq_gen_falling_edge_can_rx,
    input wire tq_gen_sample_tick_can_rx,
    input wire tq_gen_sample_data_can_rx,

    // input from can_tx
    input wire can_tx_bus_off_flag_can_rx,

    // output to can_tx
    output wire can_rx_ack_out_can_tx,
    output wire can_rx_rx_error_flag_can_tx,
    output wire can_rx_rx_rec_passive_can_tx,

    // output to tq_gen
    output wire can_rx_rx_idle_tq_gen,

    // output to axi_lite
    output wire        can_rx_frame_done_axi_lite,
    output wire [10:0] can_rx_rx_id_axi_lite,
    output wire [ 3:0] can_rx_rx_dlc_axi_lite,
    output wire [63:0] can_rx_rx_data_axi_lite,
    output wire        can_rx_rx_rtr_axi_lite,
    output wire        can_rx_rx_ide_axi_lite
);

    // can_rx_distuffer -> can_rx_frame, can_rx_crc15
    wire can_rx_distuffer_data_valid_can_rx_frame_crc15;
    wire can_rx_distuffer_data_in_can_rx_frame_crc15;

    // can_rx_distuffer -> can_rx_frame
    wire can_rx_distuffer_stuff_error_can_rx_frame;

    // can_rx_frame -> can_rx_distuffer
    wire can_rx_frame_destuff_en_can_rx_distuffer;

    // can_rx_frame -> can_rx_crc15
    wire can_rx_frame_crc_calc_en_can_rx_crc15;
    wire can_rx_frame_crc_rx_en_can_rx_crc15;
    wire can_rx_frame_crc_check_en_can_rx_crc15;

    // can_rx_crc15 -> can_rx_frame
    wire can_rx_crc15_crc_error_can_rx_frame;

    rx_distuffer u_rx_distuffer (
        .clk   (clk),
        .resetn(resetn),
        .software_reset(software_reset),

        .sample_tick(tq_gen_sample_tick_can_rx),
        .sample_data(tq_gen_sample_data_can_rx),

        .destuff_en(can_rx_frame_destuff_en_can_rx_distuffer),

        .data_valid (can_rx_distuffer_data_valid_can_rx_frame_crc15),
        .data_in    (can_rx_distuffer_data_in_can_rx_frame_crc15),
        .stuff_error(can_rx_distuffer_stuff_error_can_rx_frame)
    );

    rx_crc15 u_rx_crc15 (
        .clk   (clk),
        .resetn(resetn),
        .software_reset(software_reset),

        .data_valid(can_rx_distuffer_data_valid_can_rx_frame_crc15),
        .data_in   (can_rx_distuffer_data_in_can_rx_frame_crc15),

        .crc_calc_en (can_rx_frame_crc_calc_en_can_rx_crc15),
        .crc_rx_en   (can_rx_frame_crc_rx_en_can_rx_crc15),
        .crc_check_en(can_rx_frame_crc_check_en_can_rx_crc15),

        .crc_error(can_rx_crc15_crc_error_can_rx_frame)
    );

    rx_frame u_rx_frame (
        .clk   (clk),
        .resetn(resetn),
        .software_reset(software_reset),
        // input from can_tx
        .bus_off(can_tx_bus_off_flag_can_rx),

        // input from tq_gen
        .falling_edge(tq_gen_falling_edge_can_rx),

        // input from can_rx_distuffer
        .data_valid (can_rx_distuffer_data_valid_can_rx_frame_crc15),
        .data_in    (can_rx_distuffer_data_in_can_rx_frame_crc15),
        .stuff_error(can_rx_distuffer_stuff_error_can_rx_frame),

        // input from can_rx_crc15
        .crc_error(can_rx_crc15_crc_error_can_rx_frame),

        // output to can_rx_crc15
        .crc_calc_en (can_rx_frame_crc_calc_en_can_rx_crc15),
        .crc_rx_en   (can_rx_frame_crc_rx_en_can_rx_crc15),
        .crc_check_en(can_rx_frame_crc_check_en_can_rx_crc15),

        // output to can_rx_distuffer
        .destuff_en(can_rx_frame_destuff_en_can_rx_distuffer),

        // output to can_tx
        .rx_error_flag (can_rx_rx_error_flag_can_tx),
        .rx_rec_passive(can_rx_rx_rec_passive_can_tx),
        .ack_out       (can_rx_ack_out_can_tx),

        // output to tq_gen
        .rx_idle(can_rx_rx_idle_tq_gen),

        // output to axi_lite
        .frame_done(can_rx_frame_done_axi_lite),
        .rx_id     (can_rx_rx_id_axi_lite),
        .rx_dlc    (can_rx_rx_dlc_axi_lite),
        .rx_data   (can_rx_rx_data_axi_lite),
        .rx_rtr    (can_rx_rx_rtr_axi_lite),
        .rx_ide    (can_rx_rx_ide_axi_lite)
    );

endmodule
