
`timescale 1 ns / 1 ps

module axi_x_axis_finder_v1_0 #(
    parameter integer WIDTH                = 1280,
    parameter integer HEIGHT               = 720,
    parameter integer C_S00_AXI_DATA_WIDTH = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH = 4
)(
    input  wire                               axis_aclk,
    input  wire                               axis_aresetn,
    input  wire [23:0]                        s_axis_tdata,
    input  wire                               s_axis_tlast,
    output wire                               s_axis_tready,
    input  wire                               s_axis_tvalid,
    input  wire                               s_axis_tuser,

    output wire                               irq,

	output wire		[10:0]					  crop_x_up			,
	output wire		[10:0]					  crop_x_dn			,
	output wire		[10:0]					  center_x_up		,
	output wire		[10:0]					  center_x_dn		,
	output wire		[ 9:0]					  up_srt			,
	output wire		[ 9:0]					  dn_srt			,
	output wire		[ 2:0]					  up_size			,
	output wire		[ 2:0]					  dn_size			,

    input  wire                               s00_axi_aclk,
    input  wire                               s00_axi_aresetn,
    input  wire [C_S00_AXI_ADDR_WIDTH-1:0]    s00_axi_awaddr,
    input  wire [2:0]                         s00_axi_awprot,
    input  wire                               s00_axi_awvalid,
    output wire                               s00_axi_awready,
    input  wire [C_S00_AXI_DATA_WIDTH-1:0]    s00_axi_wdata,
    input  wire [(C_S00_AXI_DATA_WIDTH/8)-1:0] s00_axi_wstrb,
    input  wire                               s00_axi_wvalid,
    output wire                               s00_axi_wready,
    output wire [1:0]                         s00_axi_bresp,
    output wire                               s00_axi_bvalid,
    input  wire                               s00_axi_bready,
    input  wire [C_S00_AXI_ADDR_WIDTH-1:0]    s00_axi_araddr,
    input  wire [2:0]                         s00_axi_arprot,
    input  wire                               s00_axi_arvalid,
    output wire                               s00_axi_arready,
    output wire [C_S00_AXI_DATA_WIDTH-1:0]    s00_axi_rdata,
    output wire [1:0]                         s00_axi_rresp,
    output wire                               s00_axi_rvalid,
    input  wire                               s00_axi_rready
);

    wire        cfg_en_axi;
    wire [10:0] cfg_crop_x_up_axi;
    wire [10:0] cfg_crop_x_dn_axi;
    wire [9:0]  cfg_up_srt_axi;
    wire [9:0]  cfg_dn_srt_axi;
    wire [2:0]  cfg_up_size_axi;
    wire [2:0]  cfg_dn_size_axi;
    wire [10:0] cfg_center_x_up_axi;
    wire [10:0] cfg_center_x_dn_axi;

    (* ASYNC_REG = "TRUE" *) reg        cfg_en_meta;
    (* ASYNC_REG = "TRUE" *) reg        cfg_en_axis;
    (* ASYNC_REG = "TRUE" *) reg [10:0] cfg_crop_x_up_meta;
    (* ASYNC_REG = "TRUE" *) reg [10:0] cfg_crop_x_up_axis;
    (* ASYNC_REG = "TRUE" *) reg [10:0] cfg_crop_x_dn_meta;
    (* ASYNC_REG = "TRUE" *) reg [10:0] cfg_crop_x_dn_axis;
    (* ASYNC_REG = "TRUE" *) reg [9:0]  cfg_up_srt_meta;
    (* ASYNC_REG = "TRUE" *) reg [9:0]  cfg_up_srt_axis;
    (* ASYNC_REG = "TRUE" *) reg [9:0]  cfg_dn_srt_meta;
    (* ASYNC_REG = "TRUE" *) reg [9:0]  cfg_dn_srt_axis;
    (* ASYNC_REG = "TRUE" *) reg [2:0]  cfg_up_size_meta;
    (* ASYNC_REG = "TRUE" *) reg [2:0]  cfg_up_size_axis;
    (* ASYNC_REG = "TRUE" *) reg [2:0]  cfg_dn_size_meta;
    (* ASYNC_REG = "TRUE" *) reg [2:0]  cfg_dn_size_axis;
    (* ASYNC_REG = "TRUE" *) reg [10:0] cfg_center_x_up_meta;
    (* ASYNC_REG = "TRUE" *) reg [10:0] cfg_center_x_up_axis;
    (* ASYNC_REG = "TRUE" *) reg [10:0] cfg_center_x_dn_meta;
    (* ASYNC_REG = "TRUE" *) reg [10:0] cfg_center_x_dn_axis;

    wire        finder_done;
    wire [15:0] finder_x_up;
    wire [15:0] finder_x_dn;

    reg [31:0] result_hold_axis;
    reg        result_toggle_axis;

    (* ASYNC_REG = "TRUE" *) reg [31:0] result_data_meta;
    (* ASYNC_REG = "TRUE" *) reg [31:0] result_data_sync;
    (* ASYNC_REG = "TRUE" *) reg        result_toggle_meta;
    (* ASYNC_REG = "TRUE" *) reg        result_toggle_sync;
    reg result_toggle_seen;
    reg result_event_axi;

    /* Static configuration crosses to the video domain and is sampled at SOF. */
    always @(posedge axis_aclk or negedge axis_aresetn) begin
        if (!axis_aresetn) begin
            cfg_en_meta        <= 1'b0;
            cfg_en_axis        <= 1'b0;
            cfg_crop_x_up_meta <= 11'd0;
            cfg_crop_x_up_axis <= 11'd0;
            cfg_crop_x_dn_meta <= 11'd0;
            cfg_crop_x_dn_axis <= 11'd0;
            cfg_up_srt_meta    <= 10'd0;
            cfg_up_srt_axis    <= 10'd0;
            cfg_dn_srt_meta    <= 10'd0;
            cfg_dn_srt_axis    <= 10'd0;
            cfg_up_size_meta   <= 3'd0;
            cfg_up_size_axis   <= 3'd0;
            cfg_dn_size_meta   <= 3'd0;
            cfg_dn_size_axis   <= 3'd0;
            cfg_center_x_up_meta <= 11'd640;
            cfg_center_x_up_axis <= 11'd640;
            cfg_center_x_dn_meta <= 11'd640;
            cfg_center_x_dn_axis <= 11'd640;
        end else begin
            cfg_en_meta        <= cfg_en_axi;
            cfg_en_axis        <= cfg_en_meta;
            cfg_crop_x_up_meta <= cfg_crop_x_up_axi;
            cfg_crop_x_up_axis <= cfg_crop_x_up_meta;
            cfg_crop_x_dn_meta <= cfg_crop_x_dn_axi;
            cfg_crop_x_dn_axis <= cfg_crop_x_dn_meta;
            cfg_up_srt_meta    <= cfg_up_srt_axi;
            cfg_up_srt_axis    <= cfg_up_srt_meta;
            cfg_dn_srt_meta    <= cfg_dn_srt_axi;
            cfg_dn_srt_axis    <= cfg_dn_srt_meta;
            cfg_up_size_meta   <= cfg_up_size_axi;
            cfg_up_size_axis   <= cfg_up_size_meta;
            cfg_dn_size_meta   <= cfg_dn_size_axi;
            cfg_dn_size_axis   <= cfg_dn_size_meta;
            cfg_center_x_up_meta <= cfg_center_x_up_axi;
            cfg_center_x_up_axis <= cfg_center_x_up_meta;
            cfg_center_x_dn_meta <= cfg_center_x_dn_axi;
            cfg_center_x_dn_axis <= cfg_center_x_dn_meta;
        end
    end

    /* Result data is held while a toggle transfers the event across clocks. */
    always @(posedge axis_aclk or negedge axis_aresetn) begin
        if (!axis_aresetn) begin
            result_hold_axis   <= 32'd0;
            result_toggle_axis <= 1'b0;
        end else if (finder_done) begin
            result_hold_axis   <= {finder_x_dn, finder_x_up};
            result_toggle_axis <= ~result_toggle_axis;
        end
    end

    always @(posedge s00_axi_aclk or negedge s00_axi_aresetn) begin
        if (!s00_axi_aresetn) begin
            result_data_meta   <= 32'd0;
            result_data_sync   <= 32'd0;
            result_toggle_meta <= 1'b0;
            result_toggle_sync <= 1'b0;
            result_toggle_seen <= 1'b0;
            result_event_axi   <= 1'b0;
        end else begin
            result_data_meta   <= result_hold_axis;
            result_data_sync   <= result_data_meta;
            result_toggle_meta <= result_toggle_axis;
            result_toggle_sync <= result_toggle_meta;
            result_event_axi   <= 1'b0;

            if (result_toggle_sync != result_toggle_seen) begin
                result_toggle_seen <= result_toggle_sync;
                result_event_axi   <= 1'b1;
            end
        end
    end

    x_axis_finder #(
        .WIDTH  (WIDTH),
        .HEIGHT (HEIGHT)
    ) x_axis_finder_inst (
        .axis_aclk     (axis_aclk),
        .axis_aresetn  (axis_aresetn),
        .s_axis_tdata  (s_axis_tdata),
        .s_axis_tlast  (s_axis_tlast),
        .s_axis_tready (s_axis_tready),
        .s_axis_tvalid (s_axis_tvalid),
        .s_axis_tuser  (s_axis_tuser),
        .crop_x_up     (cfg_crop_x_up_axis),
        .crop_x_dn     (cfg_crop_x_dn_axis),
        .center_x_up   (cfg_center_x_up_axis),
        .center_x_dn   (cfg_center_x_dn_axis),
        .up_srt        (cfg_up_srt_axis),
        .dn_srt        (cfg_dn_srt_axis),
        .up_size       (cfg_up_size_axis),
        .dn_size       (cfg_dn_size_axis),
        .en            (cfg_en_axis),
        .done          (finder_done),
        .x_axis_up     (finder_x_up),
        .x_axis_dn     (finder_x_dn)
    );
	assign	crop_x_up	= cfg_crop_x_up_axi		;
	assign	crop_x_dn	= cfg_crop_x_dn_axi		;
	assign	center_x_up	= cfg_center_x_up_axis	;
	assign	center_x_dn	= cfg_center_x_dn_axis	;
	assign	up_srt		= cfg_up_srt_axi		;
	assign	dn_srt		= cfg_dn_srt_axi		;
	assign	up_size		= cfg_up_size_axi		;
	assign	dn_size		= cfg_dn_size_axi		;

    axi_template_8_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH (C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH (C_S00_AXI_ADDR_WIDTH)
    ) axi_template_8_v1_0_S00_AXI_inst (
        .cfg_en        (cfg_en_axi),
        .cfg_crop_x_up (cfg_crop_x_up_axi	),
        .cfg_crop_x_dn (cfg_crop_x_dn_axi	),
        .cfg_up_srt    (cfg_up_srt_axi		),
        .cfg_dn_srt    (cfg_dn_srt_axi		),
        .cfg_up_size   (cfg_up_size_axi		),
        .cfg_dn_size   (cfg_dn_size_axi		),
        .cfg_center_x_up (cfg_center_x_up_axi),
        .cfg_center_x_dn (cfg_center_x_dn_axi),
        .result_event  (result_event_axi),
        .result_data   (result_data_sync),
        .irq           (irq),
        .S_AXI_ACLK    (s00_axi_aclk),
        .S_AXI_ARESETN (s00_axi_aresetn),
        .S_AXI_AWADDR  (s00_axi_awaddr),
        .S_AXI_AWPROT  (s00_axi_awprot),
        .S_AXI_AWVALID (s00_axi_awvalid),
        .S_AXI_AWREADY (s00_axi_awready),
        .S_AXI_WDATA   (s00_axi_wdata),
        .S_AXI_WSTRB   (s00_axi_wstrb),
        .S_AXI_WVALID  (s00_axi_wvalid),
        .S_AXI_WREADY  (s00_axi_wready),
        .S_AXI_BRESP   (s00_axi_bresp),
        .S_AXI_BVALID  (s00_axi_bvalid),
        .S_AXI_BREADY  (s00_axi_bready),
        .S_AXI_ARADDR  (s00_axi_araddr),
        .S_AXI_ARPROT  (s00_axi_arprot),
        .S_AXI_ARVALID (s00_axi_arvalid),
        .S_AXI_ARREADY (s00_axi_arready),
        .S_AXI_RDATA   (s00_axi_rdata),
        .S_AXI_RRESP   (s00_axi_rresp),
        .S_AXI_RVALID  (s00_axi_rvalid),
        .S_AXI_RREADY  (s00_axi_rready)
    );

endmodule
