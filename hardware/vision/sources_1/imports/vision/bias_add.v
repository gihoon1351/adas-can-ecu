`timescale 1ns / 1ps
//lane15 = data[31:0]
//lane0 = data[511:480]
module bias_add #(
    parameter LANES = 16,
    parameter IN_W  = 32,
    parameter OUT_W = 33
) (
    input wire clk,
    input wire rst_n,

    input wire [LANES*IN_W-1:0] ppu_in_data,
    input wire [LANES*IN_W-1:0] bias_value,

    output reg [LANES*OUT_W-1:0] bias_data
);

  genvar i;

  generate
    for (i = 0; i < LANES; i = i + 1) begin : GEN_BIAS_ADD

      wire signed [ IN_W-1:0] conv_lane;
      wire signed [ IN_W-1:0] bias_lane;
      wire signed [OUT_W-1:0] conv_ext;
      wire signed [OUT_W-1:0] bias_ext;
      wire signed [OUT_W-1:0] add_result;

      assign conv_lane  = ppu_in_data[(LANES-1-i)*IN_W+:IN_W];
      assign bias_lane  = bias_value[(LANES-1-i)*IN_W+:IN_W];

      assign conv_ext   = {conv_lane[IN_W-1], conv_lane};
      assign bias_ext   = {bias_lane[IN_W-1], bias_lane};

      assign add_result = conv_ext + bias_ext;

      always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          bias_data[(LANES-1-i)*OUT_W+:OUT_W] <= {OUT_W{1'b0}};
        end else begin
          bias_data[(LANES-1-i)*OUT_W+:OUT_W] <= add_result;
        end
      end

    end
  endgenerate

endmodule
