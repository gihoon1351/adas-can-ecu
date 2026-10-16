`timescale 1ns / 1ps
// UART_CR [0] : RXEN
// UART_CR [1] : TXEN
// UART_CR [2] : RXTXEN
// UART_CR [5:3] : BRR

// UART_SR [0] : RXE rx_empty
// UART_SR [1] : RXF rx_full
// UART_SR [2] : TXE tx_empty
// UART_SR [3] : TXF tx_full
// UART_SR [4] : BUSY tx_busy
// UART_SR [5] : BUSY rx_busy

// UART_IER [0] : TXIE tx_interrupt enable
// UART_IER [1] : RXIE rx_interrupt enable

// UART_IFR [0] : TXIF tx_interrupt flag
// UART_IFR [1] : RXIF rx_interrupt flag

// UART_ICR [0] : TXIC tx_interrupt clear
// UART_ICR [1] : RXIC rx_interrupt clear
module uart_core (
    input logic clk,
    input logic rst_n,

    input logic uart_en, tx_en, rx_en,
    input logic [2:0] uart_brr,
    input logic [7:0] tx_data,
    output logic [7:0] rx_data,
    output logic rx_empty, rx_full, tx_empty, tx_full, tx_busy, rx_busy,
    input logic tx_irq_en, rx_irq_en,
    output logic tx_irq_flag, rx_irq_flag,
    input logic tx_irq_clear, rx_irq_clear,

    input logic uart_dr_we,
    input logic uart_dr_re,

    output logic irq,

    input  logic rx,
    output logic tx
);
    logic w_tick;

    logic [7:0] tx_fifo_pop_data;

    logic [7:0] rx_fifo_pop_data;
    logic [7:0] rx_fifo_push_data;

    logic tx_send;
    logic tx_fifo_push;
    logic tx_fifo_pop;
    logic rx_fifo_push;
    logic rx_fifo_pop;

    logic tx_done;
    logic rx_done;
    logic tx_irq_pending;
    logic rx_irq_pending;
    logic tx_active;
    logic rx_active;

    assign tx_active    = uart_en && tx_en;
    assign rx_active    = uart_en && rx_en;
    assign tx_send      = tx_active && !tx_busy && !tx_empty;

    assign tx_fifo_push = uart_dr_we && !tx_full;
    assign tx_fifo_pop  = tx_send;

    assign rx_fifo_push = rx_done && !rx_full;
    assign rx_fifo_pop  = uart_dr_re && !rx_empty;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_irq_pending <= 1'b0;
            rx_irq_pending <= 1'b0;
        end else begin
            if (!uart_en) begin
                tx_irq_pending <= 1'b0;
                rx_irq_pending <= 1'b0;
            end else begin
                // TX interrupt flag
                if (tx_irq_clear) begin
                    tx_irq_pending <= 1'b0;
                end
                if (tx_done && tx_empty && !tx_fifo_push) begin
                    tx_irq_pending <= 1'b1;
                end

                // RX interrupt flag
                if (rx_irq_clear) begin
                    rx_irq_pending <= !rx_empty;
                end
                if (rx_fifo_push) begin
                    rx_irq_pending <= 1'b1;
                end
            end
        end
    end

    assign tx_irq_flag = tx_irq_pending;
    assign rx_irq_flag = rx_irq_pending;
    assign irq = uart_en && ((tx_irq_en && tx_irq_pending) || (rx_irq_en && rx_irq_pending));
    assign rx_data = rx_empty ? 8'd0 : rx_fifo_pop_data;

    uart_rx u1_uart_rx (
        .clk    (clk),
        .rst_n  (rst_n),
        .tick   (w_tick),
        .rx_en  (rx_active),
        .rx_data(rx_fifo_push_data),
        .rx_done(rx_done),
        .rx_busy(rx_busy),
        .rx     (rx)
    );

    uart_tx u2_uart_tx (
        .clk    (clk),
        .rst_n  (rst_n),
        .tick   (w_tick),
        .tx_en  (tx_active),
        .tx_data(tx_fifo_pop_data),
        .tx_send(tx_send),
        .tx_done(tx_done),
        .tx_busy(tx_busy),
        .tx     (tx)
    );

    uart_baud_rate u3_uart_baud_rate (
        .clk     (clk),
        .rst_n   (rst_n),
        .uart_en (uart_en),
        .UART_BRR(uart_brr),
        .tick    (w_tick)
    );

    fifo u4_tx_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .pop_data (tx_fifo_pop_data),
        .push_data(tx_data),
        .push     (tx_fifo_push),
        .pop      (tx_fifo_pop),
        .full     (tx_full),
        .empty    (tx_empty)
    );

    fifo u5_rx_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .pop_data (rx_fifo_pop_data),
        .push_data(rx_fifo_push_data),
        .push     (rx_fifo_push),
        .pop      (rx_fifo_pop),
        .full     (rx_full),
        .empty    (rx_empty)
    );
endmodule

module uart_tx (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       tick,
    input  logic       tx_en,
    input  logic [7:0] tx_data,
    input  logic       tx_send,
    output logic       tx_done,
    output logic       tx_busy,
    output logic       tx
);
    typedef enum logic [1:0] {
        IDLE,
        START,
        DATA,
        STOP
    } state_t;

    state_t state;

    logic [7:0] tx_data_reg;
    logic [3:0] tick_cnt;
    logic [2:0] data_idx;

    assign tx_busy = tx_en && (state != IDLE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || !tx_en) begin
            state       <= IDLE;
            tx_data_reg <= 8'd0;
            tx          <= 1'b1;
            tick_cnt    <= 4'd0;
            data_idx    <= 3'd0;
            tx_done     <= 1'b0;
        end else begin
            tx_done <= 1'b0;
            case (state)
                IDLE: begin
                    tx       <= 1'b1;
                    tick_cnt <= 4'd0;
                    data_idx <= 3'd0;
                    if (tx_send) begin
                        tx_data_reg <= tx_data;
                        tx          <= 1'b0;
                        state       <= START;
                    end
                end
                START: begin
                    tx <= 1'b0;
                    if (tick) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            state    <= DATA;
                        end else begin
                            tick_cnt <= tick_cnt + 4'd1;
                        end
                    end
                end
                DATA: begin
                    tx <= tx_data_reg[0];
                    if (tick) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            if (data_idx == 3'd7) begin
                                state    <= STOP;
                                tx       <= 1'b1;
                                data_idx <= 3'd0;
                            end else begin
                                tx_data_reg <= {1'b0, tx_data_reg[7:1]};
                                data_idx    <= data_idx + 3'd1;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 4'd1;
                        end
                    end
                end
                STOP: begin
                    tx <= 1'b1;
                    if (tick) begin
                        if (tick_cnt == 4'd15) begin
                            tx_done  <= 1'b1;
                            tick_cnt <= 4'd0;
                            state    <= IDLE;
                        end else begin
                            tick_cnt <= tick_cnt + 4'd1;
                        end
                    end
                end
                default: begin
                    state       <= IDLE;
                    tx          <= 1'b1;
                    tx_data_reg <= 8'd0;
                    tick_cnt    <= 4'd0;
                    data_idx    <= 3'd0;
                end
            endcase
        end
    end
endmodule

module uart_rx (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       tick,
    input  logic       rx_en,
    output logic [7:0] rx_data,
    output logic       rx_done,
    output logic       rx_busy,
    input  logic       rx
);

    typedef enum logic [1:0] {
        IDLE,
        START,
        DATA,
        STOP
    } state_t;

    state_t       state;

    logic   [3:0] tick_cnt;
    logic   [2:0] data_idx;
    logic   [7:0] rx_data_reg;
    logic         rx_sync_1ff;
    logic         rx_sync_2ff;

    assign rx_busy = rx_en && (state != IDLE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync_1ff <= 1'b1;
            rx_sync_2ff <= 1'b1;
        end else begin
            rx_sync_1ff <= rx;
            rx_sync_2ff <= rx_sync_1ff;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || !rx_en) begin
            state       <= IDLE;
            tick_cnt    <= 4'd0;
            rx_data     <= 8'd0;
            rx_data_reg <= 8'd0;
            data_idx    <= 3'd0;
            rx_done     <= 1'b0;
        end else begin
            rx_done <= 1'b0;
            case (state)
                IDLE: begin
                    tick_cnt <= 4'd0;
                    data_idx <= 3'd0;
                    if (rx_sync_2ff == 1'b0) begin
                        state    <= START;
                        tick_cnt <= 4'd0;
                    end
                end
                START: begin
                    if (tick) begin
                        if (tick_cnt == 4'd7) begin
                            tick_cnt <= 4'd0;
                            if (rx_sync_2ff == 1'b0) begin
                                state <= DATA;
                            end else begin
                                state <= IDLE;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 4'd1;
                        end
                    end
                end
                DATA: begin
                    if (tick) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            rx_data_reg <= {rx_sync_2ff, rx_data_reg[7:1]};
                            if (data_idx == 3'd7) begin
                                data_idx <= 3'd0;
                                rx_data <= {rx_sync_2ff, rx_data_reg[7:1]};
                                state <= STOP;
                            end else begin
                                data_idx <= data_idx + 3'd1;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 4'd1;
                        end
                    end
                end
                STOP: begin
                    if (tick) begin
                        if (tick_cnt == 4'd15) begin
                            tick_cnt <= 4'd0;
                            state    <= IDLE;
                            if (rx_sync_2ff == 1'b1) begin
                                rx_done <= 1'b1;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 4'd1;
                        end
                    end
                end
                default: begin
                    state       <= IDLE;
                    tick_cnt    <= 4'd0;
                    rx_data     <= 8'd0;
                    rx_data_reg <= 8'd0;
                    data_idx    <= 3'd0;
                    rx_done     <= 1'b0;
                end
            endcase
        end
    end
endmodule

// 16 oversampling
module uart_baud_rate (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       uart_en,
    input  logic [2:0] UART_BRR,  // 115200
    output logic       tick
);
    logic [31:0] cnt;

    logic [31:0] div_cnt;

    // 80MHz 기준, 16x oversampling
    // div_cnt = round(80_000_000 / (baud * 16)) - 1

    always_comb begin
        case (UART_BRR)
            3'b000:  div_cnt = 32'd520;  // 521 clocks // 9600
            3'b001:  div_cnt = 32'd259;  // 260 clocks // 19200
            3'b010:  div_cnt = 32'd129;  // 130 clocks // 38400
            3'b011:  div_cnt = 32'd86;   // 87 clocks  // 57600
            3'b100:  div_cnt = 32'd42;   // 43 clocks  // 115200
            3'b101:  div_cnt = 32'd21;   // 22 clocks  // 230400
            default: div_cnt = 32'd42;   // default    // 115200
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt  <= 32'd0;
            tick <= 1'b0;
        end else begin
            tick <= 1'b0;
            if (!uart_en) begin
                cnt <= 32'd0;
            end else if (cnt >= div_cnt) begin
                cnt  <= 32'd0;
                tick <= 1'b1;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
endmodule

module fifo (
    input  logic       clk,
    input  logic       rst_n,
    output logic [7:0] pop_data,
    input  logic [7:0] push_data,
    input  logic       push,
    input  logic       pop,
    output logic       full,
    output logic       empty
);
    logic [7:0] w_wptr;
    logic [7:0] w_rptr;

    fifo_data_path u1_fifo_data_path (
        .clk  (clk),
        .wdata(push_data),
        .rdata(pop_data),
        .waddr(w_wptr),
        .raddr(w_rptr),
        .we   (push && !full)
    );

    fifo_control_unit u2_fifo_control_unit (
        .clk  (clk),
        .rst_n(rst_n),
        .wptr (w_wptr),
        .rptr (w_rptr),
        .push (push),
        .pop  (pop),
        .empty(empty),
        .full (full)
    );
endmodule

module fifo_data_path (
    input  logic       clk,
    input  logic [7:0] wdata,
    output logic [7:0] rdata,
    input  logic [7:0] waddr,
    input  logic [7:0] raddr,
    input  logic       we
);
    logic [7:0] register_file[0:255];

    always_ff @(posedge clk) begin
        if (we) begin
            register_file[waddr] <= wdata;
        end
    end
    assign rdata = register_file[raddr];
endmodule

module fifo_control_unit (
    input  logic       clk,
    input  logic       rst_n,
    output logic [7:0] wptr,
    output logic [7:0] rptr,
    input  logic       push,
    input  logic       pop,
    output logic       empty,
    output logic       full
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rptr  <= 8'd0;
            wptr  <= 8'd0;
            empty <= 1'b1;
            full  <= 1'd0;
        end else begin
            case ({
                push, pop
            })
                2'b00: begin
                    wptr  <= wptr;
                    rptr  <= rptr;
                    empty <= empty;
                    full  <= full;
                end
                2'b01: begin
                    if (!empty) begin
                        rptr  <= rptr + 8'd1;
                        empty <= (rptr + 8'd1 == wptr);
                        full  <= 1'd0;
                    end
                end
                2'b10: begin
                    if (!full) begin
                        wptr  <= wptr + 8'd1;
                        full  <= (wptr + 8'd1 == rptr);
                        empty <= 1'd0;
                    end
                end
                2'b11: begin
                    if (full) begin
                        rptr  <= rptr + 8'd1;
                        full  <= 1'b0;
                        empty <= 1'b0;
                    end else if (empty) begin
                        wptr  <= wptr + 8'd1;
                        full  <= 1'b0;
                        empty <= 1'b0;
                    end else begin
                        wptr <= wptr + 8'd1;
                        rptr <= rptr + 8'd1;
                    end
                end
            endcase
        end
    end
endmodule
