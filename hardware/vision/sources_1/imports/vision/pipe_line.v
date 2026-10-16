`timescale 1ns / 1ps

module add_pipe_line_alu (
    input                       clk,
    input                       resetn,
    input                       cu_pipe_clear_conv,

    input                       i_valid,
    input      [19*16-1:0]      i_pipe,

    output reg                  o_valid,
    output reg [19*16-1:0]      o_pipe
);

  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      o_valid <= 1'b0;
      o_pipe  <= 0;
    end else if (cu_pipe_clear_conv) begin
      o_valid <= 1'b0;
      o_pipe  <= 0;
    end else begin
      o_valid <= i_valid;

      if (i_valid)
        o_pipe <= i_pipe;
    end
  end

endmodule