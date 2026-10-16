`timescale 1ns / 1ps

module ppu_top (
    input wire param_bias_we,
    input wire param_scale_we,
    input wire [6:0] param_waddr,
    input wire [3:0] param_wlane,
    input wire [31:0] param_wdata,

    input  wire         clk,
    input  wire         rst_n,

    // from Conv / external control
    input  wire [511:0] conv_out_ppu,
    input  wire         conv_valid_ppu,
    input  wire [4:0]   ppu_channel_group_idx,
    input  wire [2:0]   ppu_layer_idx,

    // from external output control
    input  wire         out_data_mux_sel,

    // to Conv / external control
    output wire         ppu_ready_conv,

    // to external output control
    output wire [63:0]  ppu_out_data,
    output wire         ppu_out_valid,
    output wire         pool_line_start,
    output wire         pool_line_end
);

    // control signals
    wire [6:0] param_addr;
    wire [1:0] layer_mode;
    wire [7:0] feature_width;
    wire       pool_in_valid;
    wire       pool_clear;

    // ROM outputs
    wire [511:0] bias_value;
    wire [511:0] scale_value;

    // Scale ROM output delay
    // bias_add has 1clk pipeline, so scale_value must be delayed 1clk.
    reg [511:0] scale_value_d1;

    // datapath signals
    wire [527:0] bias_data;    // 33bit x 16
    wire [127:0] quant_data;   // 8bit x 16
    wire [127:0] relu_data;    // 8bit x 16
    wire [127:0] pool_data;    // 8bit x 16
    wire [127:0] layer_data;   // selected 8bit x 16

    // MaxPool feedback signals
    wire pool_valid;
    wire pool_line_start_w;
    wire pool_line_end_w;

    // output packer signals
    wire [63:0] out_data_0;
    wire [63:0] out_data_1;

	reg [63:0]	hold;

    // --------------------------------------------------
    // PPU Control Unit
    // --------------------------------------------------
    ppu_control_unit u_ppu_control_unit (
        .clk(clk),
        .rst_n(rst_n),

        .conv_valid_ppu(conv_valid_ppu),
        .ppu_channel_group_idx(ppu_channel_group_idx),
        .ppu_layer_idx(ppu_layer_idx),

        .pool_valid(pool_valid),
        .pool_line_start_in(pool_line_start_w),
        .pool_line_end_in(pool_line_end_w),

        .ppu_ready_conv(ppu_ready_conv),
        .ppu_out_valid(ppu_out_valid),
        .pool_line_start(pool_line_start),
        .pool_line_end(pool_line_end),

        .param_addr(param_addr),

        .feature_width(feature_width),
        .pool_in_valid(pool_in_valid),
        .pool_clear(pool_clear),

        .layer_mode(layer_mode)
    );

    // --------------------------------------------------
    // Bias / Scale distributed RAM (original asynchronous-read timing)
    // --------------------------------------------------
    bias_rom u_bias_rom (
        .clk(clk),
        .we(param_bias_we),
        .waddr(param_waddr),
        .wlane(param_wlane),
        .wdata(param_wdata),
        .addr(param_addr),
        .dout(bias_value)
    );

    scale_rom u_scale_rom (
        .clk(clk),
        .we(param_scale_we),
        .waddr(param_waddr),
        .wlane(param_wlane),
        .wdata(param_wdata),
        .addr(param_addr),
        .dout(scale_value)
    );

    // --------------------------------------------------
    // Scale pipeline register
    // --------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scale_value_d1 <= 512'd0;
        end else begin
            scale_value_d1 <= scale_value;
        end
    end

    // --------------------------------------------------
    // Bias Add
    // --------------------------------------------------
    bias_add #(
        .LANES(16),
        .IN_W(32),
        .OUT_W(33)
    ) u_bias_add (
        .clk(clk),
        .rst_n(rst_n),

        .ppu_in_data(conv_out_ppu),
        .bias_value(bias_value),

        .bias_data(bias_data)
    );

    // --------------------------------------------------
    // Quant
    // --------------------------------------------------
    quant #(
        .LANES(16),
        .IN_W(33),
        .SCALE_W(32),
        .OUT_W(8),
        .SHIFT(20)
    ) u_quant (
        .clk(clk),
        .rst_n(rst_n),

        .bias_data(bias_data),
        .scale_value(scale_value_d1),

        .quant_data(quant_data)
    );

    // --------------------------------------------------
    // ReLU
    // --------------------------------------------------
    relu u_relu (
        .quant_data(quant_data),
        .relu_data(relu_data)
    );

    // --------------------------------------------------
    // MaxPool
    // --------------------------------------------------
    max_pool #(
        .LANES(16),
        .DATA_W(8),
        .ADDR_W(8),
        .MAX_WIDTH(224)
    ) u_max_pool (
        .clk(clk),
        .rst_n(rst_n),

        .relu_data(relu_data),
        .pool_in_valid(pool_in_valid),
        .pool_clear(pool_clear),
        .feature_width(feature_width),

        .pool_data(pool_data),
        .pool_valid(pool_valid),
        .pool_line_start(pool_line_start_w),
        .pool_line_end(pool_line_end_w)
    );

    // --------------------------------------------------
    // Layer MUX
    // layer_mode:
    // 00 = pool_data
    // 01 = relu_data
    // 10 = quant_data
    // --------------------------------------------------
    layer_mux #(
        .DATA_W(128)
    ) u_layer_mux (
        .quant_data(quant_data),
        .relu_data(relu_data),
        .pool_data(pool_data),
        .layer_mode(layer_mode),

        .layer_data(layer_data)
    );

    // --------------------------------------------------
    // Output Packer
    // 128bit -> 64bit x 2
    // --------------------------------------------------
    output_packer u_output_packer (
        .layer_data(layer_data),
        .out_data_0(out_data_0),
        .out_data_1(out_data_1)
    );


	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			hold <= 0;
		end else begin
			hold <= out_data_1;
		end
	end

    // --------------------------------------------------
    // Output MUX
    // out_data_mux_sel = 0: lane0~lane7
    // out_data_mux_sel = 1: lane8~lane15
    // --------------------------------------------------
    output_mux u_output_mux (
        .out_data_0(out_data_0),
        .out_data_1(hold),
        .out_data_mux_sel(out_data_mux_sel),

        .ppu_out_data(ppu_out_data)
    );

endmodule
