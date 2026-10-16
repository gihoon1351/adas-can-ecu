`timescale 1ns / 1ps

module mux_alu (
    // input clk,
    // input resetn,
    input cu_acc_valid_sel_conv,
    // input cu_pipe_clear_conv,
    input [32 * 16 - 1:0] i_mux,
    output [32 * 16 - 1:0] o_mux
);

  // reg acc_valid_sel_sync_1;
  // reg acc_valid_sel_sync_2;

  // always @(posedge clk or negedge resetn) begin
  //   if (!resetn) begin
  //     acc_valid_sel_sync_1 <= 1'b0;
  //     acc_valid_sel_sync_2 <= 1'b0;
  //   end else if (cu_pipe_clear_conv) begin
  //     acc_valid_sel_sync_1 <= 1'b0;
  //     acc_valid_sel_sync_2 <= 1'b0;
  //   end else begin
  //     acc_valid_sel_sync_1 <= cu_acc_valid_sel_conv;
  //     acc_valid_sel_sync_2 <= acc_valid_sel_sync_1;
  //   end
  // end

  assign o_mux = (cu_acc_valid_sel_conv) ? i_mux : 512'd0;  // 512bit

endmodule
