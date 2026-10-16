
`timescale 1 ns / 1 ps

	module cnn_v1_0 #
	(
		// Users to add parameters here

		// User parameters ends
		// Do not modify the parameters beyond this line


		// Parameters of Axi Slave Bus Interface S00_AXI
		parameter integer C_S00_AXI_DATA_WIDTH	= 32,
		parameter integer C_S00_AXI_ADDR_WIDTH	= 4
	)
	(
		// Users to add ports here
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS_PARAM TDATA" *)
    input wire [31:0] s_axis_param_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS_PARAM TKEEP" *)
    input wire [3:0] s_axis_param_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS_PARAM TVALID" *)
    input wire s_axis_param_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS_PARAM TREADY" *)
    output wire s_axis_param_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS_PARAM TLAST" *)
    input wire s_axis_param_tlast,


		input	wire	[23:0]			s_axis_lb_tdata		,
		input	wire					s_axis_lb_tvalid	,
		output	wire					s_axis_lb_tready	,
		input	wire					s_axis_lb_tuser		,
		input	wire					s_axis_lb_tlast		,
		input	wire	[31:0]			s_axis_w_tdata		,
		input	wire					s_axis_w_tvalid		,
		output	wire					s_axis_w_tready		,
		input	wire					s_axis_w_tuser		,
		input	wire					s_axis_w_tlast		,
		output	wire	[31:0]			m_axis_oa_tdata		,
		output	wire					m_axis_oa_tvalid	,
		input	wire					m_axis_oa_tready	,
		output	wire	[3:0]			m_axis_oa_tkeep		,
		output	wire					m_axis_oa_tuser		,
		output	wire					m_axis_oa_tlast		,

		output	wire					intr				,

		// User ports ends
		// Do not modify the ports beyond this line


		// Ports of Axi Slave Bus Interface S00_AXI
		(* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S00_AXI:S_AXIS_LB:S_AXIS_W:S_AXIS_PARAM:M_AXIS_OA, ASSOCIATED_RESET s00_axi_aresetn" *)
        input wire  s00_axi_aclk,
		input wire  s00_axi_aresetn,
		input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_awaddr,
		input wire [2 : 0] s00_axi_awprot,
		input wire  s00_axi_awvalid,
		output wire  s00_axi_awready,
		input wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_wdata,
		input wire [(C_S00_AXI_DATA_WIDTH/8)-1 : 0] s00_axi_wstrb,
		input wire  s00_axi_wvalid,
		output wire  s00_axi_wready,
		output wire [1 : 0] s00_axi_bresp,
		output wire  s00_axi_bvalid,
		input wire  s00_axi_bready,
		input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_araddr,
		input wire [2 : 0] s00_axi_arprot,
		input wire  s00_axi_arvalid,
		output wire  s00_axi_arready,
		output wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_rdata,
		output wire [1 : 0] s00_axi_rresp,
		output wire  s00_axi_rvalid,
		input wire  s00_axi_rready
	);

    wire cr_param_abort, sr_param_loaded, sr_param_loading, sr_param_error;
	wire			cr_irq_en				;
	wire			cr_srt					;
	wire	[5:0]	sr_in_channel_group		;
	wire	[4:0]	sr_out_channel_group	;
	wire	[2:0]	sr_layer				;
	wire			sr_busy					;
	wire			sr_idle					;

	wire			irq						;

// Instantiation of Axi Bus Interface S00_AXI
	cnn_v1_0_S00_AXI # ( 
		.C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
		.C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
	) cnn_v1_0_S00_AXI_inst (
        .cr_param_abort(cr_param_abort),
        .sr_param_loaded(sr_param_loaded),
        .sr_param_loading(sr_param_loading),
        .sr_param_error(sr_param_error),

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
		.cr_irq_en				(cr_irq_en				),
		.cr_srt					(cr_srt					),
		.sr_in_channel_group	(sr_in_channel_group	),
		.sr_out_channel_group	(sr_out_channel_group	),
		.sr_layer				(sr_layer				),
		.sr_busy				(sr_busy				),
		.sr_idle				(sr_idle				),
		.done					(irq					) 
	);

	// Add user logic here

	assign	intr = irq & cr_irq_en;

	cnn_stream U_CNN_STREAM (
        .s_axis_param_tdata(s_axis_param_tdata),
        .s_axis_param_tkeep(s_axis_param_tkeep),
        .s_axis_param_tvalid(s_axis_param_tvalid),
        .s_axis_param_tready(s_axis_param_tready),
        .s_axis_param_tlast(s_axis_param_tlast),

        .cr_param_abort(cr_param_abort),
        .sr_param_loaded(sr_param_loaded),
        .sr_param_loading(sr_param_loading),
        .sr_param_error(sr_param_error),

		.clk						(s00_axi_aclk			),
		.resetn						(s00_axi_aresetn		),
		.s_axis_lb_tdata			(s_axis_lb_tdata		),
		.s_axis_lb_tvalid			(s_axis_lb_tvalid		),
		.s_axis_lb_tready			(s_axis_lb_tready		),
		.s_axis_lb_tuser			(s_axis_lb_tuser		),
		.s_axis_lb_tlast			(s_axis_lb_tlast		),
		.s_axis_w_tdata				(s_axis_w_tdata			),
		.s_axis_w_tvalid			(s_axis_w_tvalid		),
		.s_axis_w_tready			(s_axis_w_tready		),
		.s_axis_w_tuser				(s_axis_w_tuser			),
		.s_axis_w_tlast				(s_axis_w_tlast			),
		.m_axis_oa_tdata			(m_axis_oa_tdata		),
		.m_axis_oa_tvalid			(m_axis_oa_tvalid		),
		.m_axis_oa_tready			(m_axis_oa_tready		),
		.m_axis_oa_tkeep			(m_axis_oa_tkeep		),
		.m_axis_oa_tuser			(m_axis_oa_tuser		),
		.m_axis_oa_tlast			(m_axis_oa_tlast		),
		.cr_srt						(cr_srt					),
		.sr_in_channel_group		(sr_in_channel_group	),
		.sr_out_channel_group		(sr_out_channel_group	),
		.sr_layer					(sr_layer				),
		.sr_busy					(sr_busy				),
		.sr_idle					(sr_idle				),
		.irq						(irq					) 
	);

	// User logic ends

	endmodule
