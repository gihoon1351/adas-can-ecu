`timescale 1ns / 1ps

module output_mux (
    input wire [63:0] out_data_0,
    input wire [63:0] out_data_1,
    input wire        out_data_mux_sel,

    output wire [63:0] ppu_out_data
);

  assign ppu_out_data = (out_data_mux_sel == 1'b0) ? out_data_0 : out_data_1;

endmodule
