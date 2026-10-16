`timescale 1ns / 1ps

module bit_timing (
    input  wire       clk,
    input  wire       resetn,
    input  wire       software_reset,

    input  wire       tq_tick,
    input  wire       sync2ff,
    input  wire       rx_idle,
    input  wire       tx,

    input  wire [7:0] tseg1,
    input  wire [7:0] tseg2,
    input  wire [7:0] sjw,

    input  wire       falling_edge,
    
    output reg        bit_tick,
    output reg        sample_tick,
    output reg        sample_data,
    output wire       hard_sync
);

    localparam [1:0] SYNC = 2'd0, TSEG1 = 2'd1, TSEG2 = 2'd2;

    reg [1:0] c_state, n_state;

    reg [7:0] tq_cnt_reg, tq_cnt_next;

    reg [7:0] tseg1_sjw_reg, tseg1_sjw_next;
    reg [7:0] tseg2_sjw_reg, tseg2_sjw_next;

    reg resync_done_reg, resync_done_next;

    wire re_sync;

    assign hard_sync = rx_idle && falling_edge;
    assign re_sync   = !rx_idle && falling_edge;

    always @(posedge clk or negedge resetn) begin
        if (!resetn || software_reset) begin
            c_state         <= SYNC;
            tq_cnt_reg      <= 8'd0;
            tseg1_sjw_reg   <= 8'd0;
            tseg2_sjw_reg   <= 8'd0;
            resync_done_reg <= 1'b0;
        end else begin
            tq_cnt_reg      <= tq_cnt_next;
            c_state         <= n_state;
            tseg1_sjw_reg   <= tseg1_sjw_next;
            tseg2_sjw_reg   <= tseg2_sjw_next;
            resync_done_reg <= resync_done_next;
        end
    end


    always @(*) begin
        n_state          = c_state;
        bit_tick         = 1'b0;
        sample_tick      = 1'b0;
        sample_data      = 1'b0;
        tq_cnt_next      = tq_cnt_reg;
        tseg1_sjw_next   = tseg1_sjw_reg;
        tseg2_sjw_next   = tseg2_sjw_reg;
        resync_done_next = resync_done_reg;
        // Wrong priority kept for reference:
        //   if (hard_sync) begin ... n_state = SYNC; end
        //   case (c_state) ... endcase
        // The case still ran after hard_sync.  When hard_sync and tq_tick
        // coincided, the old TSEG1/TSEG2 branch could overwrite the reset
        // state/counter and even assert sample_tick or bit_tick on the SOF
        // edge.  That made the first transmitted bit phase-dependent.
        if (hard_sync) begin
            tq_cnt_next      = 8'd0;
            tseg1_sjw_next   = 8'd0;
            tseg2_sjw_next   = 8'd0;
            resync_done_next = 1'b0;
            n_state          = SYNC;
        end else begin
          case (c_state)
            SYNC: begin
                if (tq_tick) begin
                    tq_cnt_next = 8'd0;
                    n_state     = TSEG1;
                end
            end

            TSEG1: begin
                // Wrong: if (re_sync && !resync_done_reg)
                // A transmitter must not resynchronize on the delayed
                // recessive-to-dominant edge of a dominant bit it is driving
                // when that edge has positive phase error.  Doing so stretched
                // data-dependent TX bits from 2.000 us to 2.125 us.  RX nodes
                // and a transmitter sending recessive still resynchronize.
                if (re_sync && tx && !resync_done_reg) begin
                    if ((tq_cnt_reg + 8'd1) > sjw) begin
                        tseg1_sjw_next = sjw;
                    end else begin
                        tseg1_sjw_next = tq_cnt_reg + 8'd1;
                    end
                    resync_done_next = 1'b1;
                end
                if (tq_tick) begin
                    if (tq_cnt_reg == (tseg1 + tseg1_sjw_next - 8'd1)) begin
                        tq_cnt_next = 8'd0;
                        sample_data = sync2ff;
                        sample_tick = 1'b1;
                        n_state = TSEG2;
                    end else begin
                        tq_cnt_next = tq_cnt_reg + 8'd1;
                    end
                end
            end

            TSEG2: begin
                if (re_sync && !resync_done_reg) begin
                    if (((tseg2 - 8'd1) - tq_cnt_reg) > sjw) begin
                        tseg2_sjw_next = sjw;
                    end else begin
                        tseg2_sjw_next = (tseg2 - 8'd1) - tq_cnt_reg;
                    end
                    resync_done_next = 1'b1;
                end
                if (tq_tick) begin
                    if (tq_cnt_reg == ((tseg2 - 8'd1) - tseg2_sjw_next)) begin
                        tq_cnt_next = 8'd0;
                        tseg1_sjw_next = 8'd0;
                        tseg2_sjw_next = 8'd0;
                        resync_done_next = 1'b0;
                        bit_tick = 1'b1;
                        n_state = SYNC;
                    end else begin
                        tq_cnt_next = tq_cnt_reg + 8'd1;
                    end
                end
            end

            default: begin
                n_state = SYNC;
            end

          endcase
        end
    end

endmodule
