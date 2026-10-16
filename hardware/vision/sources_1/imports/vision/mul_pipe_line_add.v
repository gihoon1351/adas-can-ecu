`timescale 1ns / 1ps

module mul_pipe_line_add (
    input                         clk,
    input                         resetn,
    input                         cu_pipe_clear_conv,

    input                         i_valid,
    input      [16*8*16-1:0]      i_pipe,   // INT16 × 8 × 16

    output reg                    o_valid,
    output reg [16*8*16-1:0]      o_pipe
);

  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      o_valid <= 1'b0;
      o_pipe  <= 0;
    end else begin
      o_valid <= i_valid;
      if (i_valid)
        o_pipe <= i_pipe;
    end
  end

endmodule
