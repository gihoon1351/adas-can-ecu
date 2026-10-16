
`timescale 1 ns / 1 ps

module timer_v1_0 #(
    // Users to add parameters here

    // User parameters ends
    // Do not modify the parameters beyond this line


    // Parameters of Axi Slave Bus Interface S00_AXI
    parameter integer C_S00_AXI_DATA_WIDTH = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH = 4
) (
    // Users to add ports here
    output wire intr,
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
    wire        cnt_en;  //[0]
    wire        intr_en;  //[1]
    wire [31:0] psc;
    wire [31:0] arr;
    wire        cnt_valid;  //[2]
    wire [31:0] i_cnt;
    wire [31:0] o_cnt;

    // Instantiation of Axi Bus Interface S00_AXI
    timer_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
    ) timer_v1_0_S00_AXI_inst (
        .cnt_en       (cnt_en),
        .intr_en      (intr_en),
        .psc          (psc),
        .arr          (arr),
        .i_cnt        (i_cnt),
        .cnt_valid    (cnt_valid),
        .o_cnt        (o_cnt),
        .S_AXI_ACLK   (s00_axi_aclk),
        .S_AXI_ARESETN(s00_axi_aresetn),
        .S_AXI_AWADDR (s00_axi_awaddr),
        .S_AXI_AWPROT (s00_axi_awprot),
        .S_AXI_AWVALID(s00_axi_awvalid),
        .S_AXI_AWREADY(s00_axi_awready),
        .S_AXI_WDATA  (s00_axi_wdata),
        .S_AXI_WSTRB  (s00_axi_wstrb),
        .S_AXI_WVALID (s00_axi_wvalid),
        .S_AXI_WREADY (s00_axi_wready),
        .S_AXI_BRESP  (s00_axi_bresp),
        .S_AXI_BVALID (s00_axi_bvalid),
        .S_AXI_BREADY (s00_axi_bready),
        .S_AXI_ARADDR (s00_axi_araddr),
        .S_AXI_ARPROT (s00_axi_arprot),
        .S_AXI_ARVALID(s00_axi_arvalid),
        .S_AXI_ARREADY(s00_axi_arready),
        .S_AXI_RDATA  (s00_axi_rdata),
        .S_AXI_RRESP  (s00_axi_rresp),
        .S_AXI_RVALID (s00_axi_rvalid),
        .S_AXI_RREADY (s00_axi_rready)
    );

    // Add user logic here


    TimerCounter U_TIMER (
        .clk      (s00_axi_aclk),
        .rst_n    (s00_axi_aresetn),
        .cnt_en   (cnt_en),
        .intr_en  (intr_en),
        .psc      (psc),
        .arr      (arr),
        .cnt_valid(cnt_valid),
        .i_cnt    (i_cnt),
        .o_cnt    (o_cnt),
        .intr     (intr)
    );
    // User logic ends

endmodule



module TimerCounter (
    input  wire        clk,
    input  wire        rst_n,
    //  internal signals
    input  wire        cnt_en,
    input  wire        intr_en,
    input  wire [31:0] psc,
    input  wire [31:0] arr,
    input  wire        cnt_valid,
    input  wire [31:0] i_cnt,
    output wire [31:0] o_cnt,
    output wire        intr
);

    reg [31:0] psc_counter;
    reg psc_tick;
    reg [31:0] counter;
    reg intr_tick;

    assign o_cnt = counter;
    assign intr  = intr_tick & intr_en;

    always @(posedge clk) begin
        if (!rst_n) begin
            psc_counter <= 0;
            psc_tick    <= 1'b0;
        end else begin
            if (cnt_en) begin
                if (psc_counter == psc) begin
                    psc_counter <= 0;
                    psc_tick    <= 1'b1;
                end else begin
                    psc_counter <= psc_counter + 1;
                    psc_tick    <= 1'b0;
                end
            end
        end
    end


    always @(posedge clk) begin
        if (!rst_n) begin
            counter   <= 0;
            intr_tick <= 1'b0;
        end else begin
            intr_tick <= 1'b0;
            if (cnt_valid) begin
                counter <= i_cnt;
            end else if (cnt_en) begin
                if (psc_tick) begin
                    if (counter == arr) begin
                        counter   <= 0;
                        intr_tick <= 1'b1;
                    end else begin
                        counter   <= counter + 1;
                        intr_tick <= 1'b0;
                    end
                end
            end
        end
    end
endmodule
