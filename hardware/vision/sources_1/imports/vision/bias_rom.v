`timescale 1ns / 1ps
// Module name retained for project compatibility; this is now writable RAM.
// The old MEM_FILE/ROM initialization is intentionally removed.
module bias_rom (
    input wire clk,
    input wire we,
    input wire [6:0] waddr,
    input wire [3:0] wlane,
    input wire [31:0] wdata,
    input wire [6:0] addr,
    output wire [511:0] dout
);
    param_ram u_ram (
        .clk(clk), .we(we), .waddr(waddr), .wlane(wlane), .wdata(wdata),
        .raddr(addr), .rdata(dout)
    );
endmodule
