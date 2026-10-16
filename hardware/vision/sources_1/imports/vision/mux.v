`timescale 1ns / 1ps

module mux_mul(
        input cu_padding_sel_conv,
        input [63:0] i_mux,
        output [63:0] o_mux
    );

    assign o_mux = (cu_padding_sel_conv) ? i_mux : 64'd0;

endmodule
