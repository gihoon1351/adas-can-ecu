`timescale 1 ns / 1 ps

module safety_domain_v1_0 #(
    parameter integer LOG_SIZE = 16,

    parameter integer C_S00_AXI_DATA_WIDTH = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH = 6
) (
    // ============================================================
    // Safety Domain Clock / Reset
    // ============================================================

    input wire safety_clk,    // 10 MHz
    input wire mon_clk,       // 80 MHz function clock to monitor
    input wire safety_resetn,

    // ============================================================
    // External Fault Input
    // ============================================================

    input wire can_bus_off,

    // ============================================================
    // Safety Outputs
    // ============================================================

    output wire reset_req,
    output wire fault_irq,
    output wire safe_state,

    // ============================================================
    // AXI Slave Interface
    // ============================================================

    input wire s00_axi_aclk,
    input wire s00_axi_aresetn,

    input  wire [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_awaddr,
    input  wire [                     2:0] s00_axi_awprot,
    input  wire                            s00_axi_awvalid,
    output wire                            s00_axi_awready,

    input  wire [    C_S00_AXI_DATA_WIDTH-1:0] s00_axi_wdata,
    input  wire [(C_S00_AXI_DATA_WIDTH/8)-1:0] s00_axi_wstrb,
    input  wire                                s00_axi_wvalid,
    output wire                                s00_axi_wready,

    output wire [1:0] s00_axi_bresp,
    output wire       s00_axi_bvalid,
    input  wire       s00_axi_bready,

    input  wire [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_araddr,
    input  wire [                     2:0] s00_axi_arprot,
    input  wire                            s00_axi_arvalid,
    output wire                            s00_axi_arready,

    output wire [C_S00_AXI_DATA_WIDTH-1:0] s00_axi_rdata,
    output wire [                     1:0] s00_axi_rresp,
    output wire                            s00_axi_rvalid,
    input  wire                            s00_axi_rready
);


    // ============================================================
    // AXI Register -> Safety Domain
    // ============================================================

    (* keep = "true", mark_debug = "true" *)wire        wdt_kick;
    wire        fault_clear;

    wire [15:0] ref_window;
    wire [31:0] min_count;
    wire [31:0] max_count;

    wire [31:0] wdt_timeout;
    wire [15:0] ts_divider;
    wire [ 7:0] reset_delay;

    wire [ 7:0] fault_log_rd_addr;

    wire        mon_en;
    wire        wdt_en;


    // ============================================================
    // Safety Domain -> AXI Register
    // ============================================================

    wire [55:0] fault_log_rd_data;


    // ============================================================
    // AXI Slave Register Interface
    // ============================================================

    safety_domain_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
    ) safety_domain_v1_0_S00_AXI_inst (

        // --------------------------------------------------------
        // AXI
        // --------------------------------------------------------

        .S_AXI_ACLK   (s00_axi_aclk),
        .S_AXI_ARESETN(s00_axi_aresetn),

        .S_AXI_AWADDR (s00_axi_awaddr),
        .S_AXI_AWPROT (s00_axi_awprot),
        .S_AXI_AWVALID(s00_axi_awvalid),
        .S_AXI_AWREADY(s00_axi_awready),

        .S_AXI_WDATA (s00_axi_wdata),
        .S_AXI_WSTRB (s00_axi_wstrb),
        .S_AXI_WVALID(s00_axi_wvalid),
        .S_AXI_WREADY(s00_axi_wready),

        .S_AXI_BRESP (s00_axi_bresp),
        .S_AXI_BVALID(s00_axi_bvalid),
        .S_AXI_BREADY(s00_axi_bready),

        .S_AXI_ARADDR (s00_axi_araddr),
        .S_AXI_ARPROT (s00_axi_arprot),
        .S_AXI_ARVALID(s00_axi_arvalid),
        .S_AXI_ARREADY(s00_axi_arready),

        .S_AXI_RDATA (s00_axi_rdata),
        .S_AXI_RRESP (s00_axi_rresp),
        .S_AXI_RVALID(s00_axi_rvalid),
        .S_AXI_RREADY(s00_axi_rready),

        // --------------------------------------------------------
        // Control Register Output
        // --------------------------------------------------------

        .wdt_kick   (wdt_kick),
        .fault_clear(fault_clear),

        .mon_en(mon_en),
        .wdt_en(wdt_en),

        // --------------------------------------------------------
        // Configuration Register Output
        // --------------------------------------------------------

        .ref_window(ref_window),
        .min_count (min_count),
        .max_count (max_count),

        .wdt_timeout(wdt_timeout),
        .ts_divider (ts_divider),
        .reset_delay(reset_delay),

        .fault_log_rd_addr(fault_log_rd_addr),

        // --------------------------------------------------------
        // Status Input
        // --------------------------------------------------------

        .reset_req (reset_req),
        .fault_irq (fault_irq),
        .safe_state(safe_state),

        .fault_log_rd_data(fault_log_rd_data)
    );


    // ============================================================
    // Safety Domain
    // ============================================================

    safety_domain_top #(
        .LOG_SIZE(LOG_SIZE)
    ) u_safety_domain_top (
        .safety_clk   (safety_clk),
        .mon_clk      (mon_clk),
        .safety_resetn(safety_resetn),

        // Control
        .wdt_kick   (wdt_kick),
        .can_bus_off(can_bus_off),
        .fault_clear(fault_clear),

        .mon_en(mon_en),
        .wdt_en(wdt_en),

        // Register Map Configuration
        .ref_window(ref_window),
        .min_count (min_count),
        .max_count (max_count),

        .wdt_timeout(wdt_timeout),
        .ts_divider (ts_divider),
        .reset_delay(reset_delay),

        // Fault Status
        .reset_req (reset_req),
        .fault_irq (fault_irq),
        .safe_state(safe_state),

        // Fault Log
        .fault_log_rd_addr(fault_log_rd_addr),
        .fault_log_rd_data(fault_log_rd_data)
    );

endmodule
