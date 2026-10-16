
`timescale 1 ns / 1 ps

module can_controller_v1_0_S00_AXI #(
    // Users to add parameters here

    // User parameters ends
    // Do not modify the parameters beyond this line

    // Width of S_AXI data bus
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    // Width of S_AXI address bus
    parameter integer C_S_AXI_ADDR_WIDTH = 6
) (
    // Users to add ports here
    output wire axi_lite_sw_reset_can_controller,
    output wire rx_int,

    output wire [7:0] axi_lite_brp_can_bit_timing,
    output wire [7:0] axi_lite_tseg1_can_bit_timing,
    output wire [7:0] axi_lite_tseg2_can_bit_timing,
    output wire [7:0] axi_lite_sjw_can_bit_timing,

    // input from cpu
    output wire [10:0] axi_lite_tx_id_can_tx,
    output wire        axi_lite_tx_valid_can_tx,
    output wire        axi_lite_tx_rtr_can_tx,
    output wire        axi_lite_tx_ide_can_tx,
    output wire [ 3:0] axi_lite_tx_dlc_can_tx,
    output wire [63:0] axi_lite_tx_data_can_tx,

    // output to cpu
    input wire can_tx_bus_off_flag_axi_lite,
    input wire can_tx_tx_error_state_flag_axi_lite,
    input wire can_tx_tx_ready_axi_lite,

    // output to axi_lite
    input wire        can_rx_frame_done_axi_lite,
    input wire [10:0] can_rx_rx_id_axi_lite,
    input wire [ 3:0] can_rx_rx_dlc_axi_lite,
    input wire [63:0] can_rx_rx_data_axi_lite,
    input wire        can_rx_rx_rtr_axi_lite,
    input wire        can_rx_rx_ide_axi_lite,
    // User ports ends
    // Do not modify the ports beyond this line

    // Global Clock Signal
    input wire S_AXI_ACLK,
    // Global Reset Signal. This Signal is Active LOW
    input wire S_AXI_ARESETN,
    // Write address (issued by master, acceped by Slave)
    input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_AWADDR,
    // Write channel Protection type. This signal indicates the
    // privilege and security level of the transaction, and whether
    // the transaction is a data access or an instruction access.
    input wire [2 : 0] S_AXI_AWPROT,
    // Write address valid. This signal indicates that the master signaling
    // valid write address and control information.
    input wire S_AXI_AWVALID,
    // Write address ready. This signal indicates that the slave is ready
    // to accept an address and associated control signals.
    output wire S_AXI_AWREADY,
    // Write data (issued by master, acceped by Slave) 
    input wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_WDATA,
    // Write strobes. This signal indicates which byte lanes hold
    // valid data. There is one write strobe bit for each eight
    // bits of the write data bus.    
    input wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,
    // Write valid. This signal indicates that valid write
    // data and strobes are available.
    input wire S_AXI_WVALID,
    // Write ready. This signal indicates that the slave
    // can accept the write data.
    output wire S_AXI_WREADY,
    // Write response. This signal indicates the status
    // of the write transaction.
    output wire [1 : 0] S_AXI_BRESP,
    // Write response valid. This signal indicates that the channel
    // is signaling a valid write response.
    output wire S_AXI_BVALID,
    // Response ready. This signal indicates that the master
    // can accept a write response.
    input wire S_AXI_BREADY,
    // Read address (issued by master, acceped by Slave)
    input wire [C_S_AXI_ADDR_WIDTH-1 : 0] S_AXI_ARADDR,
    // Protection type. This signal indicates the privilege
    // and security level of the transaction, and whether the
    // transaction is a data access or an instruction access.
    input wire [2 : 0] S_AXI_ARPROT,
    // Read address valid. This signal indicates that the channel
    // is signaling valid read address and control information.
    input wire S_AXI_ARVALID,
    // Read address ready. This signal indicates that the slave is
    // ready to accept an address and associated control signals.
    output wire S_AXI_ARREADY,
    // Read data (issued by slave)
    output wire [C_S_AXI_DATA_WIDTH-1 : 0] S_AXI_RDATA,
    // Read response. This signal indicates the status of the
    // read transfer.
    output wire [1 : 0] S_AXI_RRESP,
    // Read valid. This signal indicates that the channel is
    // signaling the required read data.
    output wire S_AXI_RVALID,
    // Read ready. This signal indicates that the master can
    // accept the read data and response information.
    input wire S_AXI_RREADY
);

    // AXI4LITE signals
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_awaddr;
    reg axi_awready;
    reg axi_wready;
    reg [1 : 0] axi_bresp;
    reg axi_bvalid;
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_araddr;
    reg axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1 : 0] axi_rdata;
    reg [1 : 0] axi_rresp;
    reg axi_rvalid;

    // Example-specific design signals
    // local parameter for addressing 32 bit / 64 bit C_S_AXI_DATA_WIDTH
    // ADDR_LSB is used for addressing 32/64 bit registers/memories
    // ADDR_LSB = 2 for 32 bits (n downto 2)
    // ADDR_LSB = 3 for 64 bits (n downto 3)
    localparam integer ADDR_LSB = (C_S_AXI_DATA_WIDTH / 32) + 1;
    localparam integer OPT_MEM_ADDR_BITS = 3;
    //----------------------------------------------
    //-- Signals for user logic register space example
    //------------------------------------------------
    //-- Number of Slave Registers 9
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg0;  //CAN_BTR 
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg3;  //CAN_TX_MAILBOX
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg4;  //CAN_TX_DATA0
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg5;  //CAN_TX_DATA1
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg6;  //CAN_RX_MAILBOX
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg7;  //CAN_RX_DATA0
    reg [C_S_AXI_DATA_WIDTH-1:0] slv_reg8;  //CAN_RX_DATA1
    wire slv_reg_rden;
    wire slv_reg_wren;

    // tx_valid 용 신호
    wire can_cr_write;
    reg tx_valid_reg;
    // rx_int 용 신호
    wire can_sr_write;
    wire rx_interrupt_clear;
    reg rx_interrupt_reg;
    //software reset
    reg sw_reset_reg;

    reg [C_S_AXI_DATA_WIDTH-1:0] reg_data_out;
    integer byte_index;
    reg aw_en;

    // I/O Connections assignments

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY = axi_wready;
    assign S_AXI_BRESP = axi_bresp;
    assign S_AXI_BVALID = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA = axi_rdata;
    assign S_AXI_RRESP = axi_rresp;
    assign S_AXI_RVALID = axi_rvalid;

    // CAN TX request
    assign axi_lite_tx_valid_can_tx = tx_valid_reg;
    // CAN RX interrupt
    assign rx_int = rx_interrupt_reg;
    // CAN software reset
    assign axi_lite_sw_reset_can_controller = sw_reset_reg;
    // CAN_BTR
    assign axi_lite_brp_can_bit_timing = slv_reg0[31:24];
    assign axi_lite_tseg1_can_bit_timing = slv_reg0[23:16];
    assign axi_lite_tseg2_can_bit_timing = slv_reg0[15:8];
    assign axi_lite_sjw_can_bit_timing = slv_reg0[7:0];
    // CAN_TX_MAILBOX
    assign axi_lite_tx_dlc_can_tx = slv_reg3[16:13];
    assign axi_lite_tx_id_can_tx = slv_reg3[12:2];
    assign axi_lite_tx_ide_can_tx = slv_reg3[1];
    assign axi_lite_tx_rtr_can_tx = slv_reg3[0];
    // CAN_TX_DATA
    assign axi_lite_tx_data_can_tx = {
        slv_reg4[7:0],		//DATA0 -> [63:56]
        slv_reg4[15:8],		//DATA1 -> [55:48]
        slv_reg4[23:16],	//DATA2 -> [47:40]
        slv_reg4[31:24],	//DATA3 -> [39:32]
        slv_reg5[7:0],		//DATA4 -> [31:24]
        slv_reg5[15:8],		//DATA5 -> [23:16]
        slv_reg5[23:16],	//DATA6 -> [15:8]
        slv_reg5[31:24]		//DATA7 -> [7:0]
    };
    // Implement axi_awready generation
    // axi_awready is asserted for one S_AXI_ACLK clock cycle when both
    // S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_awready is
    // de-asserted when reset is low.

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_awready <= 1'b0;
            aw_en <= 1'b1;
        end else begin
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin
                // slave is ready to accept write address when 
                // there is a valid write address and write data
                // on the write address and data bus. This design 
                // expects no outstanding transactions. 
                axi_awready <= 1'b1;
                aw_en <= 1'b0;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                aw_en <= 1'b1;
                axi_awready <= 1'b0;
            end else begin
                axi_awready <= 1'b0;
            end
        end
    end

    // Implement axi_awaddr latching
    // This process is used to latch the address when both 
    // S_AXI_AWVALID and S_AXI_WVALID are valid. 

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_awaddr <= 0;
        end else begin
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin
                // Write Address latching 
                axi_awaddr <= S_AXI_AWADDR;
            end
        end
    end

    // Implement axi_wready generation
    // axi_wready is asserted for one S_AXI_ACLK clock cycle when both
    // S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_wready is 
    // de-asserted when reset is low. 

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_wready <= 1'b0;
        end else begin
            if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID && aw_en) begin
                // slave is ready to accept write data when 
                // there is a valid write address and write data
                // on the write address and data bus. This design 
                // expects no outstanding transactions. 
                axi_wready <= 1'b1;
            end else begin
                axi_wready <= 1'b0;
            end
        end
    end

    // Implement memory mapped register select and write logic generation
    // The write data is accepted and written to memory mapped registers when
    // axi_awready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted. Write strobes are used to
    // select byte enables of slave registers while writing.
    // These registers are cleared when reset (active low) is applied.
    // Slave register write enable is asserted when valid address and data are available
    // and the slave is ready to accept the write address and write data.
    assign slv_reg_wren = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

    assign can_cr_write = slv_reg_wren && (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] == 4'h1) && S_AXI_WSTRB[0];
    assign can_sr_write = slv_reg_wren && (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB] == 4'h2) && S_AXI_WSTRB[0];
    assign rx_interrupt_clear = can_sr_write && S_AXI_WDATA[2];

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            sw_reset_reg <= 1'b0;
        end else begin
            sw_reset_reg <= 1'b0;

            //CAN_CR[1] 에 1을 쓴 경우에만 1
            if (can_cr_write && S_AXI_WDATA[1]) sw_reset_reg <= 1'b1;
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            tx_valid_reg <= 1'b0;
        end else begin
            tx_valid_reg <= 1'b0;

            //CAN_CR[0] 에 1을 쓴 경우에만 1
            if (can_cr_write && S_AXI_WDATA[0] && !S_AXI_WDATA[1])
                tx_valid_reg <= 1'b1;
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN || sw_reset_reg) begin
            rx_interrupt_reg <= 1'b0;
        end else begin
            if (can_rx_frame_done_axi_lite) rx_interrupt_reg <= 1'b1;
            else if (rx_interrupt_clear) rx_interrupt_reg <= 1'b0;
        end
    end

	always @(posedge S_AXI_ACLK) begin
    if (!S_AXI_ARESETN || sw_reset_reg) begin
        slv_reg6 <= 32'h0000_0000;
        slv_reg7 <= 32'h0000_0000;
        slv_reg8 <= 32'h0000_0000;
    end else begin
        if (can_rx_frame_done_axi_lite) begin
            // CAN_RX_MAILBOX
            slv_reg6 <= {
                15'b0,
                can_rx_rx_dlc_axi_lite,
                can_rx_rx_id_axi_lite,
                can_rx_rx_ide_axi_lite,
                can_rx_rx_rtr_axi_lite
            };

            // CAN_RX_DATA0: DATA3~DATA0
            slv_reg7 <= {
                can_rx_rx_data_axi_lite[39:32], // DATA3
                can_rx_rx_data_axi_lite[47:40], // DATA2
                can_rx_rx_data_axi_lite[55:48], // DATA1
                can_rx_rx_data_axi_lite[63:56]  // DATA0
            };

            // CAN_RX_DATA1: DATA7~DATA4
            slv_reg8 <= {
                can_rx_rx_data_axi_lite[7:0],   // DATA7
                can_rx_rx_data_axi_lite[15:8],  // DATA6
                can_rx_rx_data_axi_lite[23:16], // DATA5
                can_rx_rx_data_axi_lite[31:24]  // DATA4
            };
        end
    end
end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            slv_reg0 <= 32'h090D0202;
            slv_reg3 <= 32'h00000000;
            slv_reg4 <= 32'h00000000;
            slv_reg5 <= 32'h00000000;
        end else begin
            if (slv_reg_wren) begin
                case (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
                    4'h0:
                    for (
                        byte_index = 0;
                        byte_index <= (C_S_AXI_DATA_WIDTH / 8) - 1;
                        byte_index = byte_index + 1
                    )
                    if (S_AXI_WSTRB[byte_index] == 1) begin
                        // Respective byte enables are asserted as per write strobes 
                        // Slave register 0
                        slv_reg0[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                    end
                    4'h3:
                    for (
                        byte_index = 0;
                        byte_index <= (C_S_AXI_DATA_WIDTH / 8) - 1;
                        byte_index = byte_index + 1
                    )
                    if (S_AXI_WSTRB[byte_index] == 1) begin
                        // Respective byte enables are asserted as per write strobes 
                        // Slave register 3
                        slv_reg3[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                    end
                    4'h4:
                    for (
                        byte_index = 0;
                        byte_index <= (C_S_AXI_DATA_WIDTH / 8) - 1;
                        byte_index = byte_index + 1
                    )
                    if (S_AXI_WSTRB[byte_index] == 1) begin
                        // Respective byte enables are asserted as per write strobes 
                        // Slave register 4
                        slv_reg4[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                    end
                    4'h5:
                    for (
                        byte_index = 0;
                        byte_index <= (C_S_AXI_DATA_WIDTH / 8) - 1;
                        byte_index = byte_index + 1
                    )
                    if (S_AXI_WSTRB[byte_index] == 1) begin
                        // Respective byte enables are asserted as per write strobes 
                        // Slave register 5
                        slv_reg5[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
                    end
                    default: begin
                        slv_reg0 <= slv_reg0;
                        slv_reg3 <= slv_reg3;
                        slv_reg4 <= slv_reg4;
                        slv_reg5 <= slv_reg5;
                    end
                endcase
            end
        end
    end

    // Implement write response logic generation
    // The write response and response valid signals are asserted by the slave 
    // when axi_wready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted.  
    // This marks the acceptance of address and indicates the status of 
    // write transaction.

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_bvalid <= 0;
            axi_bresp  <= 2'b0;
        end else begin
            if (axi_awready && S_AXI_AWVALID && ~axi_bvalid && axi_wready && S_AXI_WVALID)
	        begin
                // indicates a valid write response is available
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b0;  // 'OKAY' response 
            end                   // work error responses in future
	      else
	        begin
                if (S_AXI_BREADY && axi_bvalid) 
	            //check if bready is asserted while bvalid is high) 
	            //(there is a possibility that bready is always asserted high)   
	            begin
                    axi_bvalid <= 1'b0;
                end
            end
        end
    end

    // Implement axi_arready generation
    // axi_arready is asserted for one S_AXI_ACLK clock cycle when
    // S_AXI_ARVALID is asserted. axi_awready is 
    // de-asserted when reset (active low) is asserted. 
    // The read address is also latched when S_AXI_ARVALID is 
    // asserted. axi_araddr is reset to zero on reset assertion.

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_arready <= 1'b0;
            axi_araddr  <= 32'b0;
        end else begin
            if (~axi_arready && S_AXI_ARVALID) begin
                // indicates that the slave has acceped the valid read address
                axi_arready <= 1'b1;
                // Read address latching
                axi_araddr  <= S_AXI_ARADDR;
            end else begin
                axi_arready <= 1'b0;
            end
        end
    end

    // Implement axi_arvalid generation
    // axi_rvalid is asserted for one S_AXI_ACLK clock cycle when both 
    // S_AXI_ARVALID and axi_arready are asserted. The slave registers 
    // data are available on the axi_rdata bus at this instance. The 
    // assertion of axi_rvalid marks the validity of read data on the 
    // bus and axi_rresp indicates the status of read transaction.axi_rvalid 
    // is deasserted on reset (active low). axi_rresp and axi_rdata are 
    // cleared to zero on reset (active low).  
    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_rvalid <= 0;
            axi_rresp  <= 0;
        end else begin
            if (axi_arready && S_AXI_ARVALID && ~axi_rvalid) begin
                // Valid read data is available at the read data bus
                axi_rvalid <= 1'b1;
                axi_rresp  <= 2'b0;  // 'OKAY' response
            end else if (axi_rvalid && S_AXI_RREADY) begin
                // Read data is accepted by the master
                axi_rvalid <= 1'b0;
            end
        end
    end

    // Implement memory mapped register select and read logic generation
    // Slave register read enable is asserted when valid address is available
    // and the slave is ready to accept the read address.
    assign slv_reg_rden = axi_arready & S_AXI_ARVALID & ~axi_rvalid;
    always @(*) begin
        // Address decoding for reading registers
        case (axi_araddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
            4'h0: reg_data_out = 32'h0000_0000;  //0x00: CAN_BTR, Write Only
            4'h1: reg_data_out = 32'h0000_0000;  //0x04: CAN_CR, Write Only
            4'h2:
            reg_data_out = {
                28'b0,
                can_tx_tx_ready_axi_lite,
                rx_interrupt_reg,
                can_tx_tx_error_state_flag_axi_lite,
                can_tx_bus_off_flag_axi_lite
            };  //0x08: CAN_SR ,RO + W1C
            4'h3: reg_data_out = 32'h0000_0000;  //0x0C: CAN_TX_MAILBOX. Write Only
            4'h4: reg_data_out = 32'h0000_0000;  //0x10: CAN_TX_DATA0. Write Only
            4'h5: reg_data_out = 32'h0000_0000;  //0x14: CAN_TX_DATA1. Write Only
            4'h6: reg_data_out = slv_reg6;  //0x18: CAN_RX_MAILBOX, Read Only
            4'h7: reg_data_out = slv_reg7;  //0x1C: CAN_RX_DATA0, Read Only
            4'h8: reg_data_out = slv_reg8;  //0x20: CAN_RX_DATA1, Read Only
            default: reg_data_out = 32'h0000_0000;
        endcase
    end

    // Output register or memory read data
    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_rdata <= 0;
        end else begin
            // When there is a valid read address (S_AXI_ARVALID) with 
            // acceptance of read address by the slave (axi_arready), 
            // output the read dada 
            if (slv_reg_rden) begin
                axi_rdata <= reg_data_out;  // register read data
            end
        end
    end

    // Add user logic here

    // User logic ends

endmodule
