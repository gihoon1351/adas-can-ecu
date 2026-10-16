
`timescale 1 ns / 1 ps

module can_controller_v1_0 #(
    // Users to add parameters here

    // User parameters ends
    // Do not modify the parameters beyond this line


    // Parameters of Axi Slave Bus Interface S00_AXI
    parameter integer C_S00_AXI_DATA_WIDTH = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH = 6
) (
    input  wire rx,
    output wire tx,

    output wire can_tx_bus_off_flag_axi_lite,
    output wire rx_intr,
    // User ports ends
    // Do not modify the ports beyond this line


    // Ports of Axi Slave Bus Interface S00_AXI
    input wire s00_axi_aclk,
    input wire s00_axi_aresetn,
    input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_awaddr,
    input wire [2 : 0] s00_axi_awprot,
    input wire s00_axi_awvalid,
    output wire s00_axi_awready,
    input wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_wdata,
    input wire [(C_S00_AXI_DATA_WIDTH/8)-1 : 0] s00_axi_wstrb,
    input wire s00_axi_wvalid,
    output wire s00_axi_wready,
    output wire [1 : 0] s00_axi_bresp,
    output wire s00_axi_bvalid,
    input wire s00_axi_bready,
    input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_araddr,
    input wire [2 : 0] s00_axi_arprot,
    input wire s00_axi_arvalid,
    output wire s00_axi_arready,
    output wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_rdata,
    output wire [1 : 0] s00_axi_rresp,
    output wire s00_axi_rvalid,
    input wire s00_axi_rready
);
    wire        axi_lite_sw_reset_can_controller;
    // Users to add ports here
    wire [ 7:0] axi_lite_brp_can_bit_timing;
    wire [ 7:0] axi_lite_tseg1_can_bit_timing;
    wire [ 7:0] axi_lite_tseg2_can_bit_timing;
    wire [ 7:0] axi_lite_sjw_can_bit_timing;

    // input from cpu
    wire [10:0] axi_lite_tx_id_can_tx;
    wire        axi_lite_tx_valid_can_tx;
    wire        axi_lite_tx_rtr_can_tx;
    wire        axi_lite_tx_ide_can_tx;
    wire [ 3:0] axi_lite_tx_dlc_can_tx;
    wire [63:0] axi_lite_tx_data_can_tx;

    // output to cpu
    wire        can_tx_tx_error_state_flag_axi_lite;
    wire        can_tx_tx_ready_axi_lite;
    wire [8:0]  can_tx_tec_axi_lite;
    wire [4:0]  can_tx_last_error_state_axi_lite;
    wire [8:0]  can_tx_sticky_peak_tec_axi_lite;
    wire [4:0]  can_tx_sticky_error_state_axi_lite;
    wire        can_tx_sticky_bus_off_axi_lite;

    // output to axi_lite
    wire        can_rx_frame_done_axi_lite;
    wire [10:0] can_rx_rx_id_axi_lite;
    wire [ 3:0] can_rx_rx_dlc_axi_lite;
    wire [63:0] can_rx_rx_data_axi_lite;
    wire        can_rx_rx_rtr_axi_lite;
    wire        can_rx_rx_ide_axi_lite;

    // Instantiation of Axi Bus Interface S00_AXI
    can_controller_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
    ) can_controller_v1_0_S00_AXI_inst (
        .S_AXI_ACLK(s00_axi_aclk),
        .S_AXI_ARESETN(s00_axi_aresetn),
        .S_AXI_AWADDR(s00_axi_awaddr),
        .S_AXI_AWPROT(s00_axi_awprot),
        .S_AXI_AWVALID(s00_axi_awvalid),
        .S_AXI_AWREADY(s00_axi_awready),
        .S_AXI_WDATA(s00_axi_wdata),
        .S_AXI_WSTRB(s00_axi_wstrb),
        .S_AXI_WVALID(s00_axi_wvalid),
        .S_AXI_WREADY(s00_axi_wready),
        .S_AXI_BRESP(s00_axi_bresp),
        .S_AXI_BVALID(s00_axi_bvalid),
        .S_AXI_BREADY(s00_axi_bready),
        .S_AXI_ARADDR(s00_axi_araddr),
        .S_AXI_ARPROT(s00_axi_arprot),
        .S_AXI_ARVALID(s00_axi_arvalid),
        .S_AXI_ARREADY(s00_axi_arready),
        .S_AXI_RDATA(s00_axi_rdata),
        .S_AXI_RRESP(s00_axi_rresp),
        .S_AXI_RVALID(s00_axi_rvalid),
        .S_AXI_RREADY(s00_axi_rready),

        .axi_lite_sw_reset_can_controller(axi_lite_sw_reset_can_controller),
        .rx_intr(rx_intr),

        .axi_lite_brp_can_bit_timing  (axi_lite_brp_can_bit_timing),
        .axi_lite_tseg1_can_bit_timing(axi_lite_tseg1_can_bit_timing),
        .axi_lite_tseg2_can_bit_timing(axi_lite_tseg2_can_bit_timing),
        .axi_lite_sjw_can_bit_timing  (axi_lite_sjw_can_bit_timing),

        // input from cpu
        .axi_lite_tx_id_can_tx(axi_lite_tx_id_can_tx),
        .axi_lite_tx_valid_can_tx(axi_lite_tx_valid_can_tx),
        .axi_lite_tx_rtr_can_tx(axi_lite_tx_rtr_can_tx),
        .axi_lite_tx_ide_can_tx(axi_lite_tx_ide_can_tx),
        .axi_lite_tx_dlc_can_tx(axi_lite_tx_dlc_can_tx),
        .axi_lite_tx_data_can_tx(axi_lite_tx_data_can_tx),

        // output to cpu
        .can_tx_bus_off_flag_axi_lite(can_tx_bus_off_flag_axi_lite),
        .can_tx_tx_error_state_flag_axi_lite(can_tx_tx_error_state_flag_axi_lite),
        .can_tx_tx_ready_axi_lite(can_tx_tx_ready_axi_lite),
        .can_tx_tec_axi_lite(can_tx_tec_axi_lite),
        .can_tx_last_error_state_axi_lite(can_tx_last_error_state_axi_lite),
        .can_tx_sticky_peak_tec_axi_lite(can_tx_sticky_peak_tec_axi_lite),
        .can_tx_sticky_error_state_axi_lite(can_tx_sticky_error_state_axi_lite),
        .can_tx_sticky_bus_off_axi_lite(can_tx_sticky_bus_off_axi_lite),

        // output to axi_lite
        .can_rx_frame_done_axi_lite(can_rx_frame_done_axi_lite),
        .can_rx_rx_id_axi_lite(can_rx_rx_id_axi_lite),
        .can_rx_rx_dlc_axi_lite(can_rx_rx_dlc_axi_lite),
        .can_rx_rx_data_axi_lite(can_rx_rx_data_axi_lite),
        .can_rx_rx_rtr_axi_lite(can_rx_rx_rtr_axi_lite),
        .can_rx_rx_ide_axi_lite(can_rx_rx_ide_axi_lite)
    );

    // Add user logic here
    can_controller U_CAN_CONTROLLER (
        .clk(s00_axi_aclk),
        .resetn(s00_axi_aresetn),
        .axi_lite_sw_reset_can_controller(axi_lite_sw_reset_can_controller),

        .axi_lite_brp_can_bit_timing  (axi_lite_brp_can_bit_timing),
        .axi_lite_tseg1_can_bit_timing(axi_lite_tseg1_can_bit_timing),
        .axi_lite_tseg2_can_bit_timing(axi_lite_tseg2_can_bit_timing),
        .axi_lite_sjw_can_bit_timing  (axi_lite_sjw_can_bit_timing),

        // input from cpu
        .axi_lite_tx_id_can_tx(axi_lite_tx_id_can_tx),
        .axi_lite_tx_valid_can_tx(axi_lite_tx_valid_can_tx),
        .axi_lite_tx_rtr_can_tx(axi_lite_tx_rtr_can_tx),
        .axi_lite_tx_ide_can_tx(axi_lite_tx_ide_can_tx),
        .axi_lite_tx_dlc_can_tx(axi_lite_tx_dlc_can_tx),
        .axi_lite_tx_data_can_tx(axi_lite_tx_data_can_tx),

        // output to cpu
        .can_tx_bus_off_flag_axi_lite(can_tx_bus_off_flag_axi_lite),
        .can_tx_tx_error_state_flag_axi_lite(can_tx_tx_error_state_flag_axi_lite),
        .can_tx_tx_ready_axi_lite(can_tx_tx_ready_axi_lite),
        .can_tx_tec_axi_lite(can_tx_tec_axi_lite),
        .can_tx_last_error_state_axi_lite(can_tx_last_error_state_axi_lite),
        .can_tx_sticky_peak_tec_axi_lite(can_tx_sticky_peak_tec_axi_lite),
        .can_tx_sticky_error_state_axi_lite(can_tx_sticky_error_state_axi_lite),
        .can_tx_sticky_bus_off_axi_lite(can_tx_sticky_bus_off_axi_lite),

        // output to axi_lite
        .can_rx_frame_done_axi_lite(can_rx_frame_done_axi_lite),
        .can_rx_rx_id_axi_lite(can_rx_rx_id_axi_lite),
        .can_rx_rx_dlc_axi_lite(can_rx_rx_dlc_axi_lite),
        .can_rx_rx_data_axi_lite(can_rx_rx_data_axi_lite),
        .can_rx_rx_rtr_axi_lite(can_rx_rx_rtr_axi_lite),
        .can_rx_rx_ide_axi_lite(can_rx_rx_ide_axi_lite),

        .rx(rx),
        .tx(tx)
    );
    // User logic ends

endmodule
