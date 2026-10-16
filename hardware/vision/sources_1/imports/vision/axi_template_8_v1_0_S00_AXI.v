
`timescale 1 ns / 1 ps

module axi_template_8_v1_0_S00_AXI #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 4
)(
    output wire        cfg_en,
    output wire [10:0] cfg_crop_x_up,
    output wire [10:0] cfg_crop_x_dn,
    output wire [9:0]  cfg_up_srt,
    output wire [9:0]  cfg_dn_srt,
    output wire [2:0]  cfg_up_size,
    output wire [2:0]  cfg_dn_size,
    output wire [10:0] cfg_center_x_up,
    output wire [10:0] cfg_center_x_dn,

    input  wire        result_event,
    input  wire [31:0] result_data,
    output wire        irq,

    input  wire                              S_AXI_ACLK,
    input  wire                              S_AXI_ARESETN,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     S_AXI_AWADDR,
    input  wire [2:0]                        S_AXI_AWPROT,
    input  wire                              S_AXI_AWVALID,
    output wire                              S_AXI_AWREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]     S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] S_AXI_WSTRB,
    input  wire                              S_AXI_WVALID,
    output wire                              S_AXI_WREADY,
    output wire [1:0]                        S_AXI_BRESP,
    output wire                              S_AXI_BVALID,
    input  wire                              S_AXI_BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     S_AXI_ARADDR,
    input  wire [2:0]                        S_AXI_ARPROT,
    input  wire                              S_AXI_ARVALID,
    output wire                              S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1:0]     S_AXI_RDATA,
    output wire [1:0]                        S_AXI_RRESP,
    output wire                              S_AXI_RVALID,
    input  wire                              S_AXI_RREADY
);

    localparam integer ADDR_LSB = 2;

    localparam [1:0] REG_CR0 = 2'd0; /* 0x00 */
    localparam [1:0] REG_CR1 = 2'd1; /* 0x04 */
    localparam [1:0] REG_SR  = 2'd2; /* 0x08 read status/write centers */
    localparam [1:0] REG_DR  = 2'd3; /* 0x0C */

    localparam integer CR0_EN_BIT         = 0;
    localparam integer CR0_IRQ_EN_BIT     = 1;
    localparam integer CR0_DONE_CLEAR_BIT = 2;

    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_awaddr;
    reg                           axi_awready;
    reg                           axi_wready;
    reg [1:0]                     axi_bresp;
    reg                           axi_bvalid;
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_araddr;
    reg                           axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1:0] axi_rdata;
    reg [1:0]                     axi_rresp;
    reg                           axi_rvalid;
    reg                           aw_enable;

    reg [31:0] cr0_reg;
    reg [31:0] cr1_reg;
    reg [31:0] center_reg;
    reg        done_status;
    reg [31:0] data_reg;

    wire slv_reg_wren;
    wire slv_reg_rden;
    wire done_clear_write;
    wire [1:0] write_index;
    wire [1:0] read_index;

    integer byte_index;

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = axi_bresp;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = axi_rresp;
    assign S_AXI_RVALID  = axi_rvalid;

    assign write_index = axi_awaddr[ADDR_LSB+1:ADDR_LSB];
    assign read_index  = axi_araddr[ADDR_LSB+1:ADDR_LSB];

    assign slv_reg_wren = axi_awready && S_AXI_AWVALID &&
                          axi_wready  && S_AXI_WVALID;

    assign slv_reg_rden = axi_arready && S_AXI_ARVALID &&
                          (!axi_rvalid);

    assign done_clear_write = slv_reg_wren &&
                              (write_index == REG_CR0) &&
                              S_AXI_WSTRB[0] &&
                              S_AXI_WDATA[CR0_DONE_CLEAR_BIT];

    /* CR0: [26:16] crop_x_up, [15:6] up_srt, [5:3] up_size. */
    assign cfg_en        = cr0_reg[CR0_EN_BIT];
    assign cfg_up_size   = cr0_reg[5:3];
    assign cfg_up_srt    = cr0_reg[15:6];
    assign cfg_crop_x_up = cr0_reg[26:16];

    /* CR1: [23:13] crop_x_dn, [12:3] dn_srt, [2:0] dn_size. */
    assign cfg_dn_size   = cr1_reg[2:0];
    assign cfg_dn_srt    = cr1_reg[12:3];
    assign cfg_crop_x_dn = cr1_reg[23:13];

    /* 0x08 write data: [26:16] NEAR center X, [10:0] FAR center X. */
    assign cfg_center_x_up = center_reg[10:0];
    assign cfg_center_x_dn = center_reg[26:16];

    assign irq = done_status && cr0_reg[CR0_IRQ_EN_BIT];

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_awready <= 1'b0;
            axi_awaddr  <= {C_S_AXI_ADDR_WIDTH{1'b0}};
            aw_enable   <= 1'b1;
        end else begin
            if ((!axi_awready) && S_AXI_AWVALID && S_AXI_WVALID && aw_enable) begin
                axi_awready <= 1'b1;
                axi_awaddr  <= S_AXI_AWADDR;
                aw_enable   <= 1'b0;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                aw_enable   <= 1'b1;
                axi_awready <= 1'b0;
            end else begin
                axi_awready <= 1'b0;
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_wready <= 1'b0;
        end else begin
            if ((!axi_wready) && S_AXI_WVALID && S_AXI_AWVALID && aw_enable) begin
                axi_wready <= 1'b1;
            end else begin
                axi_wready <= 1'b0;
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            cr0_reg <= 32'd0;
            cr1_reg <= 32'd0;
            center_reg <= 32'h02800280;
        end else if (slv_reg_wren) begin
            case (write_index)
                REG_CR0: begin
                    for (byte_index = 0; byte_index < 4; byte_index = byte_index + 1) begin
                        if (S_AXI_WSTRB[byte_index]) begin
                            cr0_reg[(byte_index*8) +: 8] <=
                                S_AXI_WDATA[(byte_index*8) +: 8];
                        end
                    end
                    /* DONE_CLEAR is write-one-pulse and is never stored. */
                    cr0_reg[CR0_DONE_CLEAR_BIT] <= 1'b0;
                end

                REG_CR1: begin
                    for (byte_index = 0; byte_index < 4; byte_index = byte_index + 1) begin
                        if (S_AXI_WSTRB[byte_index]) begin
                            cr1_reg[(byte_index*8) +: 8] <=
                                S_AXI_WDATA[(byte_index*8) +: 8];
                        end
                    end
                end

                REG_SR: begin
                    for (byte_index = 0; byte_index < 4; byte_index = byte_index + 1) begin
                        if (S_AXI_WSTRB[byte_index]) begin
                            center_reg[(byte_index*8) +: 8] <=
                                S_AXI_WDATA[(byte_index*8) +: 8];
                        end
                    end
                end

                default: begin
                    cr0_reg <= cr0_reg;
                    cr1_reg <= cr1_reg;
                    center_reg <= center_reg;
                end
            endcase
        end
    end

    /* New hardware result has priority over a simultaneous software clear. */
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            done_status <= 1'b0;
            data_reg    <= 32'd0;
        end else begin
            if (done_clear_write) begin
                done_status <= 1'b0;
            end

            if (result_event) begin
                data_reg    <= result_data;
                done_status <= 1'b1;
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_bvalid <= 1'b0;
            axi_bresp  <= 2'b00;
        end else begin
            if (slv_reg_wren && (!axi_bvalid)) begin
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b00;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                axi_bvalid <= 1'b0;
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_arready <= 1'b0;
            axi_araddr  <= {C_S_AXI_ADDR_WIDTH{1'b0}};
        end else begin
            if ((!axi_arready) && S_AXI_ARVALID && (!axi_rvalid)) begin
                axi_arready <= 1'b1;
                axi_araddr  <= S_AXI_ARADDR;
            end else begin
                axi_arready <= 1'b0;
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_rvalid <= 1'b0;
            axi_rresp  <= 2'b00;
        end else begin
            if (slv_reg_rden) begin
                axi_rvalid <= 1'b1;
                axi_rresp  <= 2'b00;
            end else if (axi_rvalid && S_AXI_RREADY) begin
                axi_rvalid <= 1'b0;
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            axi_rdata <= {C_S_AXI_DATA_WIDTH{1'b0}};
        end else if (slv_reg_rden) begin
            case (read_index)
                REG_CR0: axi_rdata <= cr0_reg;
                REG_CR1: axi_rdata <= cr1_reg;
                REG_SR:  axi_rdata <= {4'd0, center_reg[26:16], 5'd0,
                                       center_reg[10:0], done_status};
                REG_DR:  axi_rdata <= data_reg;
                default: axi_rdata <= {C_S_AXI_DATA_WIDTH{1'b0}};
            endcase
        end
    end

    /* Protection inputs are intentionally unused. */
    wire unused_prot;
    assign unused_prot = ^{S_AXI_AWPROT, S_AXI_ARPROT};

endmodule
