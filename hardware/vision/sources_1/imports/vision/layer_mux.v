`timescale 1ns / 1ps

module layer_mux #(
    parameter DATA_W = 128
) (
    input  wire [DATA_W-1:0] quant_data,
    input  wire [DATA_W-1:0] relu_data,
    input  wire [DATA_W-1:0] pool_data,
    input  wire [1:0]        layer_mode,

    output reg  [DATA_W-1:0] layer_data
);

    always @(*) begin
        case (layer_mode)
            2'b00: begin
                layer_data = pool_data;
            end

            2'b01: begin
                layer_data = relu_data;
            end

            2'b10: begin
                layer_data = quant_data;
            end

            default: begin
                layer_data = {DATA_W{1'b0}};
            end
        endcase
    end

endmodule