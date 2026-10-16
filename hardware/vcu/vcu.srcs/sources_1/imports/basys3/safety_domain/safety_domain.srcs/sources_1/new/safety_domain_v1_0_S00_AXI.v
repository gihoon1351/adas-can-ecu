`timescale 1 ns / 1 ps

module safety_domain_v1_0_S00_AXI #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 6
) (
    // ============================================================
    // User Ports
    // ============================================================

    // Control pulse
    output wire wdt_kick,
    output wire fault_clear,

    output wire mon_en,
    output wire wdt_en,

    // Configuration
    output wire [15:0] ref_window,
    output wire [31:0] min_count,
    output wire [31:0] max_count,

    output wire [31:0] wdt_timeout,
    output wire [15:0] ts_divider,
    output wire [ 7:0] reset_delay,

    output wire [7:0] fault_log_rd_addr,

    // Status
    input wire reset_req,
    input wire fault_irq,
    input wire safe_state,

    // Fault Log
    input wire [55:0] fault_log_rd_data,

    // ============================================================
    // AXI4-Lite Interface
    // ============================================================

    input wire S_AXI_ACLK,
    input wire S_AXI_ARESETN,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_AWADDR,
    input  wire [                   2:0] S_AXI_AWPROT,
    input  wire                          S_AXI_AWVALID,
    output wire                          S_AXI_AWREADY,

    input  wire [    C_S_AXI_DATA_WIDTH-1:0] S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  wire                              S_AXI_WVALID,
    output wire                              S_AXI_WREADY,

    output wire [1:0] S_AXI_BRESP,
    output wire       S_AXI_BVALID,
    input  wire       S_AXI_BREADY,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0] S_AXI_ARADDR,
    input  wire [                   2:0] S_AXI_ARPROT,
    input  wire                          S_AXI_ARVALID,
    output wire                          S_AXI_ARREADY,

    output wire [C_S_AXI_DATA_WIDTH-1:0] S_AXI_RDATA,
    output wire [                   1:0] S_AXI_RRESP,
    output wire                          S_AXI_RVALID,
    input  wire                          S_AXI_RREADY
);

    // ============================================================
    // AXI Signals
    // ============================================================

    reg     [C_S_AXI_ADDR_WIDTH-1:0] axi_awaddr;
    reg                              axi_awready;
    reg                              axi_wready;

    reg     [                   1:0] axi_bresp;
    reg                              axi_bvalid;

    reg     [C_S_AXI_ADDR_WIDTH-1:0] axi_araddr;
    reg                              axi_arready;

    reg     [C_S_AXI_DATA_WIDTH-1:0] axi_rdata;
    reg     [                   1:0] axi_rresp;
    reg                              axi_rvalid;

    reg                              aw_en;

    integer                          byte_index;

    localparam integer ADDR_LSB = (C_S_AXI_DATA_WIDTH / 32) + 1;


    // ============================================================
    // Register Map
    // ============================================================

    // 0x00 : SAFETY_CR
    reg wdt_kick_reg;
    reg fault_clear_reg;
    reg mon_en_reg;
    reg wdt_en_reg;

    // 0x08 : WDT_TIMEOUT
    reg [31:0] reg_wdt_timeout;

    // 0x0C : REF_WINDOW
    reg [31:0] reg_ref_window;

    // 0x10 : MIN_COUNT
    reg [31:0] reg_min_count;

    // 0x14 : MAX_COUNT
    reg [31:0] reg_max_count;

    // 0x18 : TS_DIVIDER
    reg [31:0] reg_ts_divider;

    // 0x1C : RESET_DELAY
    reg [31:0] reg_reset_delay;

    // 0x20 : LOG_RD_ADDR
    reg [31:0] reg_log_rd_addr;


    // ============================================================
    // User Output Assignment
    // ============================================================

    assign wdt_kick = wdt_kick_reg;
    assign fault_clear = fault_clear_reg;
    assign mon_en = mon_en_reg;
    assign wdt_en = wdt_en_reg;

    assign wdt_timeout = reg_wdt_timeout;

    assign ref_window = reg_ref_window[15:0];
    assign min_count = reg_min_count;
    assign max_count = reg_max_count;

    assign ts_divider = reg_ts_divider[15:0];
    assign reset_delay = reg_reset_delay[7:0];

    assign fault_log_rd_addr = reg_log_rd_addr[7:0];


    // ============================================================
    // AXI Output Assignment
    // ============================================================

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY = axi_wready;

    assign S_AXI_BRESP = axi_bresp;
    assign S_AXI_BVALID = axi_bvalid;

    assign S_AXI_ARREADY = axi_arready;

    assign S_AXI_RDATA = axi_rdata;
    assign S_AXI_RRESP = axi_rresp;
    assign S_AXI_RVALID = axi_rvalid;


    // ============================================================
    // Write Address Ready
    // ============================================================

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_awready <= 1'b0;
            aw_en       <= 1'b1;
        end else begin
            if (!axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin

                axi_awready <= 1'b1;
                aw_en       <= 1'b0;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                axi_awready <= 1'b0;
                aw_en       <= 1'b1;
            end else begin
                axi_awready <= 1'b0;
            end
        end
    end


    // ============================================================
    // Write Address Latch
    // ============================================================

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_awaddr <= {C_S_AXI_ADDR_WIDTH{1'b0}};
        end else begin
            if (!axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin

                axi_awaddr <= S_AXI_AWADDR;
            end
        end
    end


    // ============================================================
    // Write Data Ready
    // ============================================================

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_wready <= 1'b0;
        end else begin
            if (!axi_wready && S_AXI_WVALID && S_AXI_AWVALID && aw_en) begin

                axi_wready <= 1'b1;
            end else begin
                axi_wready <= 1'b0;
            end
        end
    end


    wire slv_reg_wren;

    assign slv_reg_wren =
        axi_wready &&
        S_AXI_WVALID &&
        axi_awready &&
        S_AXI_AWVALID;


    // ============================================================
    // Register Write
    // ============================================================

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin

            // Pulse registers
            wdt_kick_reg    <= 1'b0;
            fault_clear_reg <= 1'b0;
            mon_en_reg <= 1'b0;
            wdt_en_reg <= 1'b0;

            // Default configuration
            reg_wdt_timeout <= 32'd10_000_000;

            // 10 MHz reference, 80 MHz monitored clock:
            // 10,000 ref clocks = 1 ms, nominal monitored count = 80,000.
            // Allow +/-5% frequency tolerance by default.
            reg_ref_window  <= 32'd10_000;
            reg_min_count   <= 32'd76_000;
            reg_max_count   <= 32'd84_000;

            reg_ts_divider  <= 32'd10_000;
            reg_reset_delay <= 32'd8;

            reg_log_rd_addr <= 32'd0;

        end else begin

            // ====================================================
            // SAFETY_CR signals are one-clock pulses
            // ====================================================

            wdt_kick_reg    <= 1'b0;
            fault_clear_reg <= 1'b0;


            if (slv_reg_wren) begin

                case (axi_awaddr[C_S_AXI_ADDR_WIDTH-1:ADDR_LSB])

                    // ============================================
                    // 0x00 : SAFETY_CR
                    // bit 0 : WDT_KICK
                    // bit 1 : FAULT_CLEAR
                    // bit 2 : MON_EN
                    // bit 3 : WDT_EN
                    // ============================================

                    4'h0: begin
                        if (S_AXI_WSTRB[0]) begin
                            wdt_kick_reg    <= S_AXI_WDATA[0];
                            fault_clear_reg <= S_AXI_WDATA[1];
                            mon_en_reg <= S_AXI_WDATA[2];
                            wdt_en_reg <= S_AXI_WDATA[3];
                        end
                    end


                    // ============================================
                    // 0x04 : SAFETY_SR
                    // Read Only
                    // ============================================

                    4'h1: begin
                    end


                    // ============================================
                    // 0x08 : WDT_TIMEOUT
                    // ============================================

                    4'h2: begin
                        for (
                            byte_index = 0;
                            byte_index < C_S_AXI_DATA_WIDTH / 8;
                            byte_index = byte_index + 1
                        ) begin

                            if (S_AXI_WSTRB[byte_index])
                                reg_wdt_timeout[
                                    (byte_index*8) +: 8
                                ] <= S_AXI_WDATA[
                                    (byte_index*8) +: 8
                                ];
                        end
                    end


                    // ============================================
                    // 0x0C : REF_WINDOW
                    // ============================================

                    4'h3: begin
                        for (
                            byte_index = 0;
                            byte_index < C_S_AXI_DATA_WIDTH / 8;
                            byte_index = byte_index + 1
                        ) begin

                            if (S_AXI_WSTRB[byte_index])
                                reg_ref_window[
                                    (byte_index*8) +: 8
                                ] <= S_AXI_WDATA[
                                    (byte_index*8) +: 8
                                ];
                        end
                    end


                    // ============================================
                    // 0x10 : MIN_COUNT
                    // ============================================

                    4'h4: begin
                        for (
                            byte_index = 0;
                            byte_index < C_S_AXI_DATA_WIDTH / 8;
                            byte_index = byte_index + 1
                        ) begin

                            if (S_AXI_WSTRB[byte_index])
                                reg_min_count[
                                    (byte_index*8) +: 8
                                ] <= S_AXI_WDATA[
                                    (byte_index*8) +: 8
                                ];
                        end
                    end


                    // ============================================
                    // 0x14 : MAX_COUNT
                    // ============================================

                    4'h5: begin
                        for (
                            byte_index = 0;
                            byte_index < C_S_AXI_DATA_WIDTH / 8;
                            byte_index = byte_index + 1
                        ) begin

                            if (S_AXI_WSTRB[byte_index])
                                reg_max_count[
                                    (byte_index*8) +: 8
                                ] <= S_AXI_WDATA[
                                    (byte_index*8) +: 8
                                ];
                        end
                    end


                    // ============================================
                    // 0x18 : TS_DIVIDER
                    // ============================================

                    4'h6: begin
                        for (
                            byte_index = 0;
                            byte_index < C_S_AXI_DATA_WIDTH / 8;
                            byte_index = byte_index + 1
                        ) begin

                            if (S_AXI_WSTRB[byte_index])
                                reg_ts_divider[
                                    (byte_index*8) +: 8
                                ] <= S_AXI_WDATA[
                                    (byte_index*8) +: 8
                                ];
                        end
                    end


                    // ============================================
                    // 0x1C : RESET_DELAY
                    // ============================================

                    4'h7: begin
                        for (
                            byte_index = 0;
                            byte_index < C_S_AXI_DATA_WIDTH / 8;
                            byte_index = byte_index + 1
                        ) begin

                            if (S_AXI_WSTRB[byte_index])
                                reg_reset_delay[
                                    (byte_index*8) +: 8
                                ] <= S_AXI_WDATA[
                                    (byte_index*8) +: 8
                                ];
                        end
                    end


                    // ============================================
                    // 0x20 : LOG_RD_ADDR
                    // ============================================

                    4'h8: begin
                        for (
                            byte_index = 0;
                            byte_index < C_S_AXI_DATA_WIDTH / 8;
                            byte_index = byte_index + 1
                        ) begin

                            if (S_AXI_WSTRB[byte_index])
                                reg_log_rd_addr[
                                    (byte_index*8) +: 8
                                ] <= S_AXI_WDATA[
                                    (byte_index*8) +: 8
                                ];
                        end
                    end


                    // 0x24 : LOG_DATA_L - RO
                    // 0x28 : LOG_DATA_H - RO

                    default: begin
                    end

                endcase
            end
        end
    end


    // ============================================================
    // Write Response
    // ============================================================

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_bvalid <= 1'b0;
            axi_bresp  <= 2'b00;
        end else begin

            if (axi_awready &&
                S_AXI_AWVALID &&
                !axi_bvalid &&
                axi_wready &&
                S_AXI_WVALID) begin

                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b00;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                axi_bvalid <= 1'b0;
            end
        end
    end


    // ============================================================
    // Read Address Ready / Latch
    // ============================================================

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_arready <= 1'b0;
            axi_araddr  <= {C_S_AXI_ADDR_WIDTH{1'b0}};
        end else begin

            if (!axi_arready && S_AXI_ARVALID) begin
                axi_arready <= 1'b1;
                axi_araddr  <= S_AXI_ARADDR;
            end else begin
                axi_arready <= 1'b0;
            end
        end
    end


    // ============================================================
    // Read Valid / Response
    // ============================================================

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_rvalid <= 1'b0;
            axi_rresp  <= 2'b00;
        end else begin

            if (axi_arready && S_AXI_ARVALID && !axi_rvalid) begin

                axi_rvalid <= 1'b1;
                axi_rresp  <= 2'b00;
            end else if (axi_rvalid && S_AXI_RREADY) begin
                axi_rvalid <= 1'b0;
            end
        end
    end


    wire slv_reg_rden;

    assign slv_reg_rden = axi_arready && S_AXI_ARVALID && !axi_rvalid;


    // ============================================================
    // Register Read MUX
    // ============================================================

    reg [31:0] reg_data_out;

    always @(*) begin

        reg_data_out = 32'd0;

        case (axi_araddr[C_S_AXI_ADDR_WIDTH-1:ADDR_LSB])

            // 0x00 SAFETY_CR
            4'h0: begin
                // Pulse bits [1:0] always read as zero.  Return the
                // persistent enable bits so software read-modify-write
                // operations do not clear the other enable.
                reg_data_out = {28'd0, wdt_en_reg, mon_en_reg, 2'b00};
            end

            // 0x04 SAFETY_SR
            4'h1: begin
                reg_data_out = {29'd0, safe_state, reset_req, fault_irq};
            end

            // 0x08 WDT_TIMEOUT
            4'h2: begin
                reg_data_out = reg_wdt_timeout;
            end

            // 0x0C REF_WINDOW
            4'h3: begin
                reg_data_out = reg_ref_window;
            end

            // 0x10 MIN_COUNT
            4'h4: begin
                reg_data_out = reg_min_count;
            end

            // 0x14 MAX_COUNT
            4'h5: begin
                reg_data_out = reg_max_count;
            end

            // 0x18 TS_DIVIDER
            4'h6: begin
                reg_data_out = reg_ts_divider;
            end

            // 0x1C RESET_DELAY
            4'h7: begin
                reg_data_out = reg_reset_delay;
            end

            // 0x20 LOG_RD_ADDR
            4'h8: begin
                reg_data_out = reg_log_rd_addr;
            end

            // 0x24 LOG_DATA_L
            4'h9: begin
                reg_data_out = fault_log_rd_data[31:0];
            end

            // 0x28 LOG_DATA_H
            4'hA: begin
                reg_data_out = {8'd0, fault_log_rd_data[55:32]};
            end

            default: begin
                reg_data_out = 32'd0;
            end

        endcase
    end


    // ============================================================
    // AXI Read Data
    // ============================================================

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_rdata <= 32'd0;
        end else begin
            if (slv_reg_rden) axi_rdata <= reg_data_out;
        end
    end

endmodule
