`timescale 1ns / 1ps

module acc_buffer (
    input clk,
    input resetn,
    input psum_pipe_valid,
    // input cu_pipe_clear_conv,
    input [32 * 16 -1 : 0] in_acc_buffer,  // 512bit
    input cu_buffer_sel_conv,
    output [32 * 16 -1 : 0] acc_out_ppu,  // 512bit
    output [32 * 16 -1 : 0] acc_out_mux
);

  reg [32 * 16 - 1 : 0] buffer_a, buffer_b;
  // reg buffer_sel_sync_1, buffer_sel_sync_2;

  // always @(posedge clk, negedge resetn) begin
  //   if (!resetn) begin
  //     buffer_sel_sync_1 <= 1'b0;
  //     buffer_sel_sync_2 <= 1'b0;
  //   end else if (cu_pipe_clear_conv) begin
  //     buffer_sel_sync_1 <= 1'b0;
  //     buffer_sel_sync_2 <= 1'b0;
  //   end else begin
  //     buffer_sel_sync_1 <= cu_buffer_sel_conv;
  //     buffer_sel_sync_2 <= buffer_sel_sync_1;
  //   end
  // end

  always @(posedge clk, negedge resetn) begin
    if (!resetn) begin
      buffer_a <= 0;
      buffer_b <= 0;
    end else begin
      if (psum_pipe_valid) begin
        if (!cu_buffer_sel_conv) begin
          buffer_a <= in_acc_buffer;
        end else begin
          buffer_b <= in_acc_buffer;
        end
      end
    end
  end
  
  assign acc_out_ppu = (cu_buffer_sel_conv) ? buffer_a : buffer_b;
  assign acc_out_mux = (cu_buffer_sel_conv) ? buffer_b : buffer_a;

endmodule