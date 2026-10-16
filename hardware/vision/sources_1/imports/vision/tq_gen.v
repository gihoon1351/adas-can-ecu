`timescale 1ns / 1ps

module tq_gen (
    // input top
    input wire clk,
    input wire resetn,
    // input from cpu (AXI)
    input wire [7:0] brp,  // TQ 분주비: TQ주기 = (brp+1) 클럭
    input wire software_reset,
    // input from rx
    input wire hard_sync,  // 펄스 → brp_cnt 리셋
    // output to can_tx, can_rx
    output reg tq_tick
);

    reg [4:0] brp_cnt, brp_cnt_next;

    always @(posedge clk, negedge resetn) begin
        if (!resetn || software_reset) begin
            brp_cnt <= 0;
        end else begin
            brp_cnt <= brp_cnt_next;
        end
    end

    always @(*) begin
        brp_cnt_next = brp_cnt;
        tq_tick = 0;

        if (hard_sync) begin
            brp_cnt_next = 0;
        end else begin
            if (brp_cnt >= brp) begin
                brp_cnt_next = 0;
                tq_tick = 1;
            end else begin
                brp_cnt_next = brp_cnt + 1;
            end
        end
    end

endmodule
