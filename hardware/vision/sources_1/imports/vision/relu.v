`timescale 1ns / 1ps

// lane0  = data[127:120]
// lane15 = data[7:0]

module relu (
    input  wire [127:0] quant_data,
    output wire [127:0] relu_data
);

    assign relu_data[127:120] = quant_data[127] ? 8'd0 : quant_data[127:120];
    assign relu_data[119:112] = quant_data[119] ? 8'd0 : quant_data[119:112];
    assign relu_data[111:104] = quant_data[111] ? 8'd0 : quant_data[111:104];
    assign relu_data[103:96] = quant_data[103] ? 8'd0 : quant_data[103:96];
    assign relu_data[95:88] = quant_data[95] ? 8'd0 : quant_data[95:88];
    assign relu_data[87:80] = quant_data[87] ? 8'd0 : quant_data[87:80];
    assign relu_data[79:72] = quant_data[79] ? 8'd0 : quant_data[79:72];
    assign relu_data[71:64] = quant_data[71] ? 8'd0 : quant_data[71:64];
    assign relu_data[63:56] = quant_data[63] ? 8'd0 : quant_data[63:56];
    assign relu_data[55:48] = quant_data[55] ? 8'd0 : quant_data[55:48];
    assign relu_data[47:40] = quant_data[47] ? 8'd0 : quant_data[47:40];
    assign relu_data[39:32] = quant_data[39] ? 8'd0 : quant_data[39:32];
    assign relu_data[31:24] = quant_data[31] ? 8'd0 : quant_data[31:24];
    assign relu_data[23:16] = quant_data[23] ? 8'd0 : quant_data[23:16];
    assign relu_data[15:8] = quant_data[15] ? 8'd0 : quant_data[15:8];
    assign relu_data[7:0] = quant_data[7] ? 8'd0 : quant_data[7:0];

endmodule