`timescale 1ns / 1ps

module can_tx (
    // input top
    input  wire        clk,
    input  wire        resetn,
    input  wire        software_reset,
    // output top
    output wire        tx,
    // input from bit_timing
    input  wire        tq_gen_bit_tick_can_tx,
    input  wire        tq_gen_sample_tick_can_tx,
    input  wire        tq_gen_sample_data_can_tx,
    // input from cpu
    input  wire [10:0] axi_lite_tx_id_can_tx,
    input  wire        axi_lite_tx_valid_can_tx,
    input  wire        axi_lite_tx_rtr_can_tx,
    input  wire        axi_lite_tx_ide_can_tx,
    input  wire [ 3:0] axi_lite_tx_dlc_can_tx,
    input  wire [63:0] axi_lite_tx_data_can_tx,              // [7:0]*8 flatten
    // output to cpu
    output wire        can_tx_bus_off_flag_axi_lite,
    output wire        can_tx_tx_error_state_flag_axi_lite,
    output wire        can_tx_tx_ready_axi_lite,
    output wire [8:0]  can_tx_tec_axi_lite,
    output wire [4:0]  can_tx_last_error_state_axi_lite,
    output wire [8:0]  can_tx_sticky_peak_tec_axi_lite,
    output wire [4:0]  can_tx_sticky_error_state_axi_lite,
    output wire        can_tx_sticky_bus_off_axi_lite,
    // input from can_rx
    input  wire        can_rx_ack_out_can_tx,
    input  wire        can_rx_rx_error_flag_can_tx,
    input  wire        can_rx_rx_rec_passive_can_tx
);

    // ── state ──────────────────────────────────────────────────────────────
    parameter [4:0] TX_BUS_INTEGRATION = 5'd0;
    parameter [4:0] TX_IDLE = 5'd1;
    parameter [4:0] TX_SOF = 5'd2;
    parameter [4:0] TX_ARBITRATION = 5'd3;
    parameter [4:0] TX_CONTROL = 5'd4;
    parameter [4:0] TX_DATA = 5'd5;
    parameter [4:0] TX_CRC = 5'd6;
    parameter [4:0] TX_CRC_DEL = 5'd7;
    parameter [4:0] TX_ACK = 5'd8;
    parameter [4:0] TX_ACK_DEL = 5'd9;
    parameter [4:0] TX_EOF = 5'd10;
    parameter [4:0] TX_IFS = 5'd11;
    parameter [4:0] TX_ERROR = 5'd12;
    parameter [4:0] TX_ERROR_DEL = 5'd13;
    parameter [4:0] TX_WAIT = 5'd14;
    parameter [4:0] TX_STUFF = 5'd15;
    parameter [4:0] TX_BUS_OFF = 5'd16;
    parameter [4:0] TX_RETRY_BACKOFF = 5'd17;
    parameter [11:0] RETRY_BACKOFF_BITS = 12'd2500; // 5 ms at 500 kbit/s
    reg [4:0] current_state, next_state, cb_state, cb_state_next;

    // ── tx output ──────────────────────────────────────────────────────────
    reg tx_reg, tx_reg_next;

    // ── frame 래치 ─────────────────────────────────────────────────────────
    reg [11:0] arbitration_reg, arbitration_next;  // {tx_id[10:0], tx_rtr}
    reg [5:0] control_reg, control_next;  // {tx_ide, r0, tx_dlc}
    reg [63:0] tx_data_reg, tx_data_next;  // 페이로드 flatten
    reg [3:0] byte_cnt, byte_cnt_next;  // DATA 바이트 인덱스

    // ── tx_crc ─────────────────────────────────────────────────────────────
    reg [14:0] tx_crc_reg, tx_crc_reg_next;
    reg tx_crc_en_reg, tx_crc_en_next;
    reg tx_crc_clear_reg, tx_crc_clear_next;

    // ── 에러 / 상태 플래그 ─────────────────────────────────────────────────
    reg tx_error_flag_reg, tx_error_flag_next;  // tx error passive
    reg tx_error_state_flag_reg, tx_error_state_flag_next;  // tx error state
    reg bus_off_reg, bus_off_next;  // 내부 bus-off
    reg bus_off_flag_reg, bus_off_flag_next;  // 외부 출력용
    reg bit_error_reg, bit_error_next;
    reg arbitration_lost_reg, arbitration_lost_next;
    reg error_off_reg, error_off_next;  // error delimiter
    reg pending_reg, pending_next;  // 재전송 대기
    reg frame_success_reg, frame_success_next;
    reg [1:0] retry_count_reg, retry_count_next;
    reg [11:0] retry_backoff_reg, retry_backoff_next;

    // ── TEC ────────────────────────────────────────────────────────────────
    reg [8:0] TEC_reg, TEC_next;
    reg [4:0] last_error_state_reg, last_error_state_next;
    reg [8:0] sticky_peak_tec_reg;
    reg [4:0] sticky_error_state_reg;
    reg sticky_bus_off_reg;

    // ── bus integration ────────────────────────────────────────────────────
    reg bus_integration_reg, bus_integration_next;
    reg [3:0] rx_cnt_reg, rx_cnt_next;

    // ── bit stuffing ───────────────────────────────────────────────────────
    reg [2:0] stuff_cnt_reg, stuff_cnt_next;
    reg prev_reg, prev_next;

    // ── 카운터 ─────────────────────────────────────────────────────────────
    reg [3:0] bit_cnt, bit_cnt_next;
    reg [6:0] bus_cnt_reg, bus_cnt_next;

    // ── tx ready ───────────────────────────────────────────────────────────
    reg tx_ready_reg, tx_ready_next;

    // ── wire alias ─────────────────────────────────────────────────────────
    wire bit_tick, sample_tick, sample_data;
    wire [10:0] tx_id;
    wire tx_valid, tx_rtr, tx_ide;
    wire [ 3:0] tx_dlc;
    wire [63:0] tx_data;
    wire ack_out, rx_error_flag, rx_rec_passive;

    // ── 현재 바이트/비트 추출 (flatten 접근) ───────────────────────────────
    wire current_data_bit;
    assign current_data_bit = tx_data_reg[(7-byte_cnt)*8+(7-bit_cnt)];

    // input from bit_timing
    assign bit_tick = tq_gen_bit_tick_can_tx;
    assign sample_tick = tq_gen_sample_tick_can_tx;
    assign sample_data = tq_gen_sample_data_can_tx;
    // input from cpu
    assign tx_id = axi_lite_tx_id_can_tx;
    assign tx_valid = axi_lite_tx_valid_can_tx;
    assign tx_rtr = axi_lite_tx_rtr_can_tx;
    assign tx_ide = axi_lite_tx_ide_can_tx;
    assign tx_dlc = axi_lite_tx_dlc_can_tx;
    assign tx_data = axi_lite_tx_data_can_tx;
    // output to cpu
    assign can_tx_bus_off_flag_axi_lite = bus_off_flag_reg;
    assign can_tx_tx_error_state_flag_axi_lite = tx_error_state_flag_reg;
    assign can_tx_tx_ready_axi_lite = tx_ready_reg;
    assign can_tx_tec_axi_lite = TEC_reg;
    assign can_tx_last_error_state_axi_lite = last_error_state_reg;
    assign can_tx_sticky_peak_tec_axi_lite = sticky_peak_tec_reg;
    assign can_tx_sticky_error_state_axi_lite = sticky_error_state_reg;
    assign can_tx_sticky_bus_off_axi_lite = sticky_bus_off_reg;
    // input from can_rx
    assign ack_out = can_rx_ack_out_can_tx;
    assign rx_error_flag = can_rx_rx_error_flag_can_tx;
    assign rx_rec_passive = can_rx_rx_rec_passive_can_tx;
    // top output
    assign tx = tx_reg;

    always @(posedge clk, negedge resetn) begin
        if (!resetn || software_reset) begin
            // ── state ───────────────────────────────
            current_state           <= TX_BUS_INTEGRATION;
            cb_state                <= TX_IDLE;
            // ── tx output ───────────────────────────
            tx_reg                  <= 1;
            tx_ready_reg            <= 0;
            // ── frame 래치 ──────────────────────────
            arbitration_reg         <= 0;
            control_reg             <= 0;
            tx_data_reg             <= 64'h0;
            byte_cnt                <= 0;
            // ── tx_crc ──────────────────────────────
            tx_crc_reg              <= 0;
            tx_crc_en_reg           <= 0;
            tx_crc_clear_reg        <= 0;
            // ── 에러 / 상태 플래그 ──────────────────
            tx_error_flag_reg       <= 0;
            bus_off_reg             <= 0;
            bus_off_flag_reg        <= 0;
            bit_error_reg           <= 0;
            arbitration_lost_reg    <= 0;
            error_off_reg           <= 0;
            pending_reg             <= 0;
            frame_success_reg       <= 0;
            retry_count_reg         <= 0;
            retry_backoff_reg       <= 0;
            tx_error_state_flag_reg <= 0;
            // ── TEC ─────────────────────────────────
            TEC_reg                 <= 0;
            last_error_state_reg    <= TX_IDLE;
            sticky_peak_tec_reg     <= 0;
            sticky_error_state_reg  <= TX_IDLE;
            sticky_bus_off_reg      <= 0;
            // ── bus integration ─────────────────────
            bus_integration_reg     <= 1;
            rx_cnt_reg              <= 0;
            // ── bit stuffing ────────────────────────
            stuff_cnt_reg           <= 0;
            prev_reg                <= 0;
            // ── 카운터 ──────────────────────────────
            bit_cnt                 <= 0;
            bus_cnt_reg             <= 0;
        end else begin
            // ── state ───────────────────────────────
            current_state           <= next_state;
            cb_state                <= cb_state_next;
            // ── tx output ───────────────────────────
            tx_reg                  <= tx_reg_next;
            tx_ready_reg            <= tx_ready_next;
            // ── frame 래치 ──────────────────────────
            arbitration_reg         <= arbitration_next;
            control_reg             <= control_next;
            tx_data_reg             <= tx_data_next;
            byte_cnt                <= byte_cnt_next;
            // ── tx_crc ──────────────────────────────
            tx_crc_reg              <= tx_crc_reg_next;
            tx_crc_en_reg           <= tx_crc_en_next;
            tx_crc_clear_reg        <= tx_crc_clear_next;
            // ── 에러 / 상태 플래그 ──────────────────
            tx_error_flag_reg       <= tx_error_flag_next;
            bus_off_reg             <= bus_off_next;
            bus_off_flag_reg        <= bus_off_flag_next;
            bit_error_reg           <= bit_error_next;
            arbitration_lost_reg    <= arbitration_lost_next;
            error_off_reg           <= error_off_next;
            pending_reg             <= pending_next;
            frame_success_reg       <= frame_success_next;
            retry_count_reg         <= retry_count_next;
            retry_backoff_reg       <= retry_backoff_next;
            tx_error_state_flag_reg <= tx_error_state_flag_next;
            // ── TEC ─────────────────────────────────
            TEC_reg                 <= TEC_next;
            last_error_state_reg    <= last_error_state_next;
            if (TEC_next > sticky_peak_tec_reg) begin
                sticky_peak_tec_reg    <= TEC_next;
                sticky_error_state_reg <= current_state;
            end
            if (bus_off_next)
                sticky_bus_off_reg <= 1'b1;
            // ── bus integration ─────────────────────
            bus_integration_reg     <= bus_integration_next;
            rx_cnt_reg              <= rx_cnt_next;
            // ── bit stuffing ────────────────────────
            stuff_cnt_reg           <= stuff_cnt_next;
            prev_reg                <= prev_next;
            // ── 카운터 ──────────────────────────────
            bit_cnt                 <= bit_cnt_next;
            bus_cnt_reg             <= bus_cnt_next;
        end
    end

    always @(*) begin
        // ── state ───────────────────────────────
        next_state               = current_state;
        cb_state_next            = cb_state;
        // ── tx output ───────────────────────────
        tx_reg_next              = tx_reg;
        tx_ready_next            = tx_ready_reg;
        // ── frame 래치 ──────────────────────────
        arbitration_next         = arbitration_reg;
        control_next             = control_reg;
        tx_data_next             = tx_data_reg;
        byte_cnt_next            = byte_cnt;
        // ── tx_crc ──────────────────────────────
        tx_crc_reg_next          = tx_crc_reg;
        tx_crc_en_next           = tx_crc_en_reg;
        tx_crc_clear_next        = tx_crc_clear_reg;
        // ── 에러 / 상태 플래그 ──────────────────
        tx_error_flag_next       = tx_error_flag_reg;
        bus_off_next             = bus_off_reg;
        bus_off_flag_next        = bus_off_flag_reg;
        bit_error_next           = bit_error_reg;
        arbitration_lost_next    = arbitration_lost_reg;
        error_off_next           = error_off_reg;
        pending_next             = pending_reg;
        frame_success_next       = frame_success_reg;
        retry_count_next         = retry_count_reg;
        retry_backoff_next       = retry_backoff_reg;
        tx_error_state_flag_next = tx_error_state_flag_reg;
        // ── TEC ─────────────────────────────────
        TEC_next                 = TEC_reg;
        last_error_state_next    = last_error_state_reg;
        // ── bus integration ─────────────────────
        bus_integration_next     = bus_integration_reg;
        rx_cnt_next              = rx_cnt_reg;
        // ── bit stuffing ────────────────────────
        stuff_cnt_next           = stuff_cnt_reg;
        prev_next                = prev_reg;
        // ── 카운터 ──────────────────────────────
        bit_cnt_next             = bit_cnt;
        bus_cnt_next             = bus_cnt_reg;

        // TEC/REC error state and bus-off status.
        // Wrong error-passive test kept for reference:
        //   if (TEC_reg < 128 || rx_rec_passive == 0)
        //       tx_error_flag_next = 0;
        //   else
        //       tx_error_flag_next = 1;
        // That expression requires BOTH TEC>=128 and REC>=128.  CAN becomes
        // error-passive when EITHER counter reaches 128.  The old code also
        // never asserted bus_off_flag_reg, so the top-level LED3 stayed low.
        if (TEC_reg >= 9'd256) begin
            tx_error_flag_next = 1'b1;
            bus_off_next       = 1'b1;
            bus_off_flag_next  = 1'b1;
        end else begin
            tx_error_flag_next = (TEC_reg >= 9'd128) || rx_rec_passive;
            bus_off_next       = 1'b0;
            bus_off_flag_next  = 1'b0;
        end

        // ── tx_crc LFSR ─────────────────────────
        if (tx_crc_clear_reg) begin
            tx_crc_reg_next = 15'h0000;
        end else if (tx_crc_en_reg && bit_tick) begin
            // Wrong manual tap implementation kept for reference: it omitted
            // feedback taps 7 and 3 and incorrectly added tap 1.  An all-zero
            // ID/control/data stream still produces CRC=0, hiding that defect;
            // the first recessive data bit makes its CRC diverge immediately.
            //
            // Bosch CAN 2.0: shift first, then XOR polynomial 0x4599 when
            // NXTBIT XOR CRC_RG[14] is one.  This vector form prevents another
            // per-bit tap transcription error.
            tx_crc_reg_next = {tx_crc_reg[13:0], 1'b0};
            if (tx_reg ^ tx_crc_reg[14])
                tx_crc_reg_next = tx_crc_reg_next ^ 15'h4599;
        end

        // ── FSM ─────────────────────────────────
        case (current_state)
            TX_BUS_INTEGRATION: begin
                if (sample_tick) begin
                    if (sample_data == 1) begin
                        if (rx_cnt_reg == 10) begin
                            rx_cnt_next          = 0;
                            bus_integration_next = 0;
                        end else begin
                            rx_cnt_next          = rx_cnt_next + 1;
                            bus_integration_next = 1;
                        end
                    end else begin
                        rx_cnt_next          = 0;
                        bus_integration_next = 1;
                    end
                end
                if (bit_tick) begin
                    if (bus_integration_reg == 0) begin
                        bus_integration_next = 1;
                        next_state           = TX_IDLE;
                        tx_ready_next        = 1;
                    end else begin
                        next_state = TX_BUS_INTEGRATION;
                    end
                end
            end
            TX_IDLE: begin
                tx_error_state_flag_next = 0;
                tx_ready_next            = 1;
                tx_reg_next              = 1;
                tx_crc_en_next           = 0;
                tx_crc_clear_next        = 1;

                // 새 요청 저장은 버스 상태 판단과 별도로 처리
                if (tx_ready_reg && tx_valid && !pending_reg) begin
                    arbitration_next   = {tx_id[10:0], tx_rtr};
                    control_next       = {tx_ide, 1'b0, tx_dlc};
                    tx_data_next       = tx_data;
                    pending_next       = 1;
                    frame_success_next = 0;
                    tx_ready_next      = 0;
                end

                // 버스가 사용 중이면 요청을 보관한 채 수신 대기
                if (sample_tick && sample_data == 0) begin
                    bit_cnt_next  = 0;
                    tx_ready_next = 0;
                    next_state    = TX_WAIT;

                // Start SOF only at a CAN bit boundary.  Entering TX_SOF as
                // soon as the AXI request arrived could assert dominant just
                // before the free-running idle sample point.  RX then sampled
                // recessive before the synchronized falling edge reached the
                // bit-timing block, producing intermittent SOF_BIT errors and
                // malformed frames that were not ACKed by any ECU.
                end else if (pending_next && bit_tick) begin
                    bit_cnt_next  = 0;
                    tx_ready_next = 0;
                    next_state    = TX_SOF;
                end
            end
            TX_SOF: begin
                tx_ready_next     = 0;
                tx_reg_next       = 0;
                tx_crc_clear_next = 0;
                tx_crc_en_next    = 1;
                if (sample_tick) begin
                    if (sample_data == 0) begin
                        prev_next      = 0;
                        stuff_cnt_next = 0;
                    end else begin
                        bit_error_next = 1;
                    end
                end
                if (bit_tick) begin
                    if (rx_error_flag == 0) begin
                        if (bit_error_reg == 0) begin
                            next_state = TX_ARBITRATION;
                        end else begin
                            bit_error_next = 0;
                            TEC_next       = TEC_reg + 8;
                            next_state     = TX_ERROR;
                        end
                    end else begin
                        bit_error_next = 0;
                        TEC_next       = TEC_reg + 8;
                        next_state     = TX_ERROR;
                    end
                end
            end
            TX_ARBITRATION: begin
                if (stuff_cnt_reg == 4) begin
                    cb_state_next = TX_ARBITRATION;
                    next_state    = TX_STUFF;
                end else begin
                    tx_reg_next    = arbitration_reg[11-bit_cnt];
                    tx_crc_en_next = 1;
                    if (sample_tick) begin
                        if (arbitration_reg[11-bit_cnt] == 0 && sample_data == 1)
                            bit_error_next = 1;
                        if (!(sample_data == arbitration_reg[11-bit_cnt]))
                            arbitration_lost_next = 1;
                    end
                end
                if (bit_tick) begin
                    if (rx_error_flag == 0) begin
                        if (bit_error_reg == 0) begin
                            if (arbitration_lost_reg == 0) begin
                                prev_next = arbitration_reg[11-bit_cnt];
                                if (prev_reg == arbitration_reg[11-bit_cnt])
                                    stuff_cnt_next = stuff_cnt_reg + 1;
                                else stuff_cnt_next = 0;
                                if (bit_cnt == 11) begin
                                    bit_cnt_next = 0;
                                    next_state   = TX_CONTROL;
                                end else begin
                                    bit_cnt_next = bit_cnt + 1;
                                end
                            end else begin
                                // arbitration lost → WAIT, 재전송 대기 유지
                                arbitration_lost_next = 0;
                                bit_cnt_next          = 0;
                                tx_reg_next           = 1'b1;
                                next_state            = TX_WAIT;
                            end
                        end else begin
                            bit_error_next        = 0;
                            arbitration_lost_next = 0;
                            TEC_next              = TEC_reg + 8;
                            next_state            = TX_ERROR;
                        end
                    end else begin
                        bit_error_next        = 0;
                        arbitration_lost_next = 0;
                        TEC_next              = TEC_reg + 8;
                        next_state            = TX_ERROR;
                    end
                end
            end
            TX_CONTROL: begin
                if (stuff_cnt_reg == 4) begin
                    cb_state_next = TX_CONTROL;
                    next_state    = TX_STUFF;
                end else begin
                    tx_reg_next    = control_reg[5-bit_cnt];
                    tx_crc_en_next = 1;
                    if (sample_tick) begin
                        if (!(sample_data == control_reg[5-bit_cnt]))
                            bit_error_next = 1;
                    end
                end
                if (bit_tick) begin
                    if (rx_error_flag == 0) begin
                        if (bit_error_reg == 0) begin
                            if (prev_reg == control_reg[5-bit_cnt])
                                stuff_cnt_next = stuff_cnt_reg + 1;
                            else stuff_cnt_next = 0;
                            prev_next = control_reg[5-bit_cnt];
                            if (bit_cnt == 5) begin
                                bit_cnt_next  = 0;
                                byte_cnt_next = 0;
                                if (control_reg[3:0] == 0 || arbitration_reg[0] == 1)
                                    next_state = TX_CRC;
                                else next_state = TX_DATA;
                            end else begin
                                bit_cnt_next = bit_cnt + 1;
                            end
                        end else begin
                            bit_error_next = 0;
                            TEC_next       = TEC_reg + 8;
                            next_state     = TX_ERROR;
                        end
                    end else begin
                        bit_error_next = 0;
                        TEC_next       = TEC_reg + 8;
                        next_state     = TX_ERROR;
                    end
                end
            end
            TX_DATA: begin
                if (stuff_cnt_reg == 4) begin
                    cb_state_next = TX_DATA;
                    next_state    = TX_STUFF;
                end else begin
                    tx_reg_next    = current_data_bit;
                    tx_crc_en_next = 1;
                    if (sample_tick) begin
                        if (!(sample_data == current_data_bit))
                            bit_error_next = 1;
                    end
                end
                if (bit_tick) begin
                    if (rx_error_flag == 0) begin
                        if (bit_error_reg == 0) begin
                            // Previously missing: update the bit-stuffing history.
                            prev_next = current_data_bit;
                            if (prev_reg == current_data_bit)
                                stuff_cnt_next = stuff_cnt_reg + 1;
                            else stuff_cnt_next = 0;
                            if (bit_cnt == 7) begin
                                bit_cnt_next = 0;
                                if (byte_cnt == (control_reg[3:0] - 1)) begin
                                    byte_cnt_next = 0;
                                    next_state    = TX_CRC;
                                end else begin
                                    byte_cnt_next = byte_cnt + 1;
                                end
                            end else begin
                                bit_cnt_next = bit_cnt + 1;
                            end
                        end else begin
                            bit_error_next = 0;
                            TEC_next       = TEC_reg + 8;
                            next_state     = TX_ERROR;
                        end
                    end else begin
                        bit_error_next = 0;
                        TEC_next       = TEC_reg + 8;
                        next_state     = TX_ERROR;
                    end
                end
            end
            TX_CRC: begin
                if (stuff_cnt_reg == 4) begin
                    cb_state_next = TX_CRC;
                    next_state    = TX_STUFF;
                end else begin
                    tx_reg_next    = tx_crc_reg[14-bit_cnt];
                    tx_crc_en_next = 0;
                    if (sample_tick) begin
                        if (!(sample_data == tx_crc_reg[14-bit_cnt]))
                            bit_error_next = 1;
                    end
                end
                if (bit_tick) begin
                    if (rx_error_flag == 0) begin
                        if (bit_error_reg == 0) begin
                            if (prev_reg == tx_crc_reg[14-bit_cnt])
                                stuff_cnt_next = stuff_cnt_reg + 1;
                            else stuff_cnt_next = 0;
                            prev_next = tx_crc_reg[14-bit_cnt];
                            if (bit_cnt == 14) begin
                                bit_cnt_next = 0;
                                // Wrong: next_state = TX_CRC_DEL;
                                // CAN bit stuffing remains active through the
                                // final CRC sequence bit.  If that bit completes
                                // a run of five, one stuff bit must be sent before
                                // the CRC delimiter.  Skipping it makes validity
                                // depend on ID/DLC/data because those fields change
                                // the CRC suffix (for example, ID 101h/DLC 1 has
                                // affected payload values even though DATA 01h is
                                // not one of them).
                                if ((prev_reg == tx_crc_reg[0]) &&
                                    (stuff_cnt_reg == 3'd3)) begin
                                    cb_state_next = TX_CRC_DEL;
                                    next_state    = TX_STUFF;
                                end else begin
                                    next_state = TX_CRC_DEL;
                                end
                            end else begin
                                bit_cnt_next = bit_cnt + 1;
                            end
                        end else begin
                            bit_error_next = 0;
                            TEC_next       = TEC_reg + 8;
                            next_state     = TX_ERROR;
                        end
                    end else begin
                        bit_error_next = 0;
                        TEC_next       = TEC_reg + 8;
                        next_state     = TX_ERROR;
                    end
                end
            end
            TX_CRC_DEL: begin
                tx_reg_next    = 1;
                tx_crc_en_next = 0;
                if (sample_tick) begin
                    if (!(sample_data == 1)) bit_error_next = 1;
                end
                if (bit_tick) begin
                    if (rx_error_flag == 0) begin
                        if (bit_error_reg == 0) begin
                            next_state = TX_ACK;
                        end else begin
                            bit_error_next = 0;
                            TEC_next       = TEC_reg + 8;
                            next_state     = TX_ERROR;
                        end
                    end else begin
                        bit_error_next = 0;
                        TEC_next       = TEC_reg + 8;
                        next_state     = TX_ERROR;
                    end
                end
            end
            TX_ACK: begin
                tx_reg_next = 1;

                if (sample_tick) begin
                    if (!(sample_data == 0)) bit_error_next = 1;
                end

                if (bit_tick) begin
                    if (bit_error_reg == 0) begin
                        next_state = TX_ACK_DEL;
                    end else begin
                        bit_error_next = 0;
                        TEC_next       = TEC_reg + 8;
                        next_state     = TX_ERROR;
                    end
                end
            end
            TX_ACK_DEL: begin
                tx_reg_next    = 1;
                tx_crc_en_next = 0;
                if (sample_tick) begin
                    if (!(sample_data == 1)) bit_error_next = 1;
                end
                if (bit_tick) begin
                    if (bit_error_reg == 0) begin
                        next_state = TX_EOF;
                    end else begin
                        bit_error_next = 0;
                        TEC_next       = TEC_reg + 8;
                        next_state     = TX_ERROR;
                    end
                end
            end
            TX_EOF: begin
                tx_reg_next    = 1;
                tx_crc_en_next = 0;
                if (sample_tick) begin
                    if (!(sample_data == 1)) bit_error_next = 1;
                end
                if (bit_tick) begin
                    if (bit_error_reg == 0) begin
                        if (bit_cnt == 6) begin
                            bit_cnt_next = 0;
                            frame_success_next = 1;
                            pending_next = 0;
                            retry_count_next = 0;
                            retry_backoff_next = 0;
                            next_state = TX_IFS;

                            if (TEC_reg != 9'd0) begin
                                TEC_next = TEC_reg - 9'd1;
                            end
                        end else begin
                            bit_cnt_next = bit_cnt + 1;
                        end
                    end else begin
                        bit_error_next = 0;
                        TEC_next       = TEC_reg + 8;
                        next_state     = TX_ERROR;
                    end
                end
            end
            TX_IFS: begin
                tx_reg_next    = 1'b1;
                tx_crc_en_next = 1'b0;
                tx_ready_next  = 1'b0;
                bit_error_next = 1'b0;

                // RX와 동일하게 샘플 시점에 연속 recessive를 계산
                if (sample_tick) begin
                    if (sample_data == 1'b1) begin
                        if (bit_cnt < 4'd3) bit_cnt_next = bit_cnt + 4'd1;
                    end else begin
                        bit_cnt_next = 4'd0;
                    end
                end

                // 세 번째 recessive 비트가 끝난 뒤 이동
                if (bit_tick && (bit_cnt == 4'd3)) begin
                    bit_cnt_next       = 4'd0;
                    frame_success_next = 1'b0;
                    next_state         = TX_IDLE;

                    if (frame_success_reg) begin
                        pending_next  = 1'b0;
                        tx_ready_next = 1'b1;
                    end else if (pending_reg) begin
                        // Keep the failed request, but do not retry forever at
                        // full bus speed.  Three consecutive failed frames are
                        // followed by a 5 ms idle-bus backoff.
                        tx_ready_next = 1'b0;
                        if (retry_count_reg >= 2'd3) begin
                            retry_backoff_next = 0;
                            next_state = TX_RETRY_BACKOFF;
                        end
                    end else begin
                        tx_ready_next = 1'b1;
                    end
                end
            end
            TX_RETRY_BACKOFF: begin
                tx_reg_next      = 1'b1;
                tx_ready_next    = 1'b0;
                tx_crc_en_next   = 1'b0;
                tx_crc_clear_next = 1'b1;
                bit_error_next   = 1'b0;

                // Count only idle (recessive) CAN bit times.  Bus activity
                // restarts the delay so this transmitter never cuts into an
                // active frame when the backoff expires.
                if (sample_tick && (sample_data == 1'b0))
                    retry_backoff_next = 0;
                else if (bit_tick) begin
                    if (retry_backoff_reg >= (RETRY_BACKOFF_BITS - 1'b1)) begin
                        retry_backoff_next = 0;
                        retry_count_next   = 0;
                        next_state         = TX_IDLE;
                    end else begin
                        retry_backoff_next = retry_backoff_reg + 1'b1;
                    end
                end
            end
            TX_ERROR: begin
                tx_error_state_flag_next = 1;
                frame_success_next = 0;
                tx_reg_next = tx_error_flag_reg;
                if (sample_tick) begin
                    if (!(sample_data == tx_error_flag_reg)) begin
                        if (tx_error_flag_reg == 0) TEC_next = TEC_reg + 8;
                    end
                end
                if (bit_tick) begin
                    if (bit_cnt == 5) begin
                        bit_cnt_next = 0;
                        next_state   = TX_ERROR_DEL;
                    end else begin
                        bit_cnt_next = bit_cnt + 1;
                    end
                end
            end
            TX_ERROR_DEL: begin
                tx_reg_next = 1;
                if (sample_tick) begin
                    // Wrong: once set, error_off_reg was never cleared by a
                    // dominant bit.  The delimiter must be eight CONSECUTIVE
                    // recessive bits, otherwise recovery timing depends on the
                    // previous error pattern.
                    if (sample_data == 1) begin
                        error_off_next = 1;
                    end else begin
                        error_off_next = 0;
                        bit_cnt_next   = 0;
                    end
                end
                if (bit_tick) begin
                    if (bit_cnt == 7) begin
                        error_off_next = 0;
                        if (bus_off_reg == 1) begin
                            bit_cnt_next = 0;
                            next_state = TX_BUS_OFF;
                            tx_error_state_flag_next = 0;
                        end else begin
                            bit_cnt_next = 0;
                            next_state = TX_IFS;
                            tx_error_state_flag_next = 0;
                        end
                    end else begin
                        if (error_off_reg == 1) bit_cnt_next = bit_cnt + 1;
                    end
                end
            end
            TX_WAIT: begin
                // CRC reset
                tx_crc_en_next = 1'b0;
                tx_crc_clear_next = 1'b1;

                if (bit_tick) begin
                    tx_reg_next = ack_out;
                end

                // Wrong logic kept for reference. A dominant sample reset the
                // counter here, but the later bit_tick block incremented it
                // again during the same CAN bit.
                // if (sample_tick) begin
                //     if (!(sample_data == 1)) bit_cnt_next = 0;
                // end
                // if (bit_tick) begin
                //     if (bit_cnt == 10) begin
                //         bit_cnt_next = 0;
                //         tx_ready_next = 1;
                //         next_state = TX_IDLE;
                //     end else begin
                //         bit_cnt_next = bit_cnt + 1;
                //     end
                // end

                if (sample_tick) begin
                    if (sample_data == 1) begin
                        bit_cnt_next = bit_cnt + 1;
                    end else begin
                        bit_cnt_next = 0;
                    end
                end

                if (bit_tick && (bit_cnt >= 11)) begin
                    bit_cnt_next = 0;
                    // Wrong when arbitration was lost: tx_ready_next = 1;
                    // pending_reg is still set in that case.  A ready pulse
                    // makes the top believe the reflection completed even
                    // though this frame has not yet been transmitted.
                    if (pending_reg) begin
                        tx_ready_next = 0;
                        next_state    = TX_SOF;
                    end else begin
                        tx_ready_next = 1;
                        next_state    = TX_IDLE;
                    end
                end
            end
            TX_STUFF: begin
                tx_reg_next    = ~prev_reg;
                tx_crc_en_next = 0;
                if (sample_tick) begin
                    if (sample_data == prev_reg) bit_error_next = 1;
                end
                if (bit_tick) begin
                    if (rx_error_flag == 0) begin
                        if (bit_error_reg == 0) begin
                            prev_next      = ~prev_reg;
                            stuff_cnt_next = 0;
                            next_state     = cb_state;
                        end else begin
                            bit_error_next = 0;
                            TEC_next       = TEC_reg + 8;
                            next_state     = TX_ERROR;
                        end
                    end else begin
                        bit_error_next = 0;
                        TEC_next       = TEC_reg + 8;
                        next_state     = TX_ERROR;
                    end
                end
            end
            TX_BUS_OFF: begin
                if (sample_tick) begin
                    if (sample_data == 1) bit_cnt_next = bit_cnt + 1;
                    else bit_cnt_next = 0;
                end
                if (bit_tick) begin
                    if (bit_cnt == 11) begin
                        if (bus_cnt_reg == 127) begin
                            tx_ready_next = 1;
                            next_state    = TX_IDLE;
                            TEC_next      = 0;
                            bus_off_next  = 0;
                            bus_off_flag_next = 0;
                            tx_error_state_flag_next = 0;
                            bus_cnt_next  = 0;
                            pending_next  = 0;  // bus-off 후 재전송 취소
                            retry_count_next = 0;
                            retry_backoff_next = 0;
                        end else begin
                            bus_cnt_next = bus_cnt_reg + 1;
                            bit_cnt_next = 0;
                        end
                    end
                end
            end

            default: next_state = TX_IDLE;
        endcase

        // Preserve the exact transmitter phase that caused the most recent
        // TEC increase. Arbitration loss does not increase TEC and therefore
        // is intentionally not reported as an error.
        if (TEC_next > TEC_reg)
            last_error_state_next = current_state;

        // Every active/passive error flag starts at bit zero.  Previously the
        // bit counter from ARBITRATION/CONTROL/DATA/CRC was reused, so an error
        // detected late in a field could produce fewer than six error bits.
        if ((current_state != TX_ERROR) && (next_state == TX_ERROR)) begin
            bit_cnt_next       = 0;
            error_off_next     = 0;
            frame_success_next = 0;

            if (retry_count_reg < 2'd3)
                retry_count_next = retry_count_reg + 1'b1;

            tx_crc_en_next     = 1'b0;
            tx_crc_clear_next  = 1'b1;
        end
    end
endmodule
