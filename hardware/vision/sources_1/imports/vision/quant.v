`timescale 1ns / 1ps
// lane15 = data[LSB side]
// lane0  = data[MSB side]

module quant #(
    parameter LANES   = 16,
    parameter IN_W    = 33,
    parameter SCALE_W = 32,
    parameter OUT_W   = 8,
    parameter SHIFT   = 20
) (
    input  wire                         clk,
    input  wire                         rst_n,

    input  wire [LANES*IN_W-1:0]        bias_data,
    input  wire [LANES*SCALE_W-1:0]     scale_value,

    output reg  [LANES*OUT_W-1:0]       quant_data
);

    genvar i;

	generate
		for (i = 0; i < LANES; i = i + 1) begin : GEN_QUANT

			wire signed [IN_W-1:0]    bias_lane;
			wire        [SCALE_W-1:0] scale_lane;

			wire signed [SCALE_W:0] scale_ext;

			(* use_dsp = "yes" *)
			reg signed [IN_W+SCALE_W:0] mul_pipe;

			wire signed [IN_W+SCALE_W:0] shift_data;

			reg signed [OUT_W-1:0] quant_comb;

			assign bias_lane =
				bias_data[(LANES-1-i)*IN_W +: IN_W];

			assign scale_lane =
				scale_value[(LANES-1-i)*SCALE_W +: SCALE_W];

			assign scale_ext = $signed({1'b0, scale_lane});

			// Stage 1: multiply
			// reset을 넣지 않아야 DSP 내부 출력 레지스터로 추론되기 유리함
			always @(posedge clk) begin
				mul_pipe <= bias_lane * scale_ext;
			end

			// Stage 2: shift + saturation
			assign shift_data = mul_pipe >>> SHIFT;

			always @(*) begin
				if (shift_data > 127)
					quant_comb = 8'sd127;
				else if (shift_data < -128)
					quant_comb = -8'sd128;
				else
					quant_comb = shift_data[OUT_W-1:0];
			end

			always @(posedge clk or negedge rst_n) begin
				if (!rst_n)
					quant_data[(LANES-1-i)*OUT_W +: OUT_W]
						<= {OUT_W{1'b0}};
				else
					quant_data[(LANES-1-i)*OUT_W +: OUT_W]
						<= quant_comb;
			end

		end
	endgenerate
endmodule
