`timescale 1ns / 1ps

module output_packer (
    input  wire [127:0] layer_data,

    output wire [63:0]  out_data_0,
    output wire [63:0]  out_data_1
);

    // lane0~lane7
    assign out_data_0 = layer_data[127:64];

    // lane8~lane15
    assign out_data_1 = layer_data[63:0];

endmodule