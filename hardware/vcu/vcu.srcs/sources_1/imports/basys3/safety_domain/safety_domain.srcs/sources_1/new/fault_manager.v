`timescale 1ns / 1ps

module fault_manager #(
    parameter integer LOG_SIZE = 16,
    // CRITICAL fault가 제거되지 않았을 때 CPU reset을 다시 요청하는 주기.
    // safety_clk = 10 MHz 기준 기본값은 1초이다.
    parameter integer RESET_RETRY_CYCLES = 10_000_000
) (
    input wire safety_clk,
    input wire resetn,

    input wire [7:0] reset_delay,

    input wire can_bus_off,
    input wire wdt_fault,
    input wire clk_fault,
    input wire sec_ecc_fault,
    input wire ded_ecc_fault,

    input wire fault_clear,
    input wire [63:0] timestamp,

    output wire reset_req,
    output wire fault_irq,
    output wire safe_state,

    output wire log_wr_en,
    output wire [7:0] log_wr_addr,
    output wire [55:0] log_wr_data
);
    localparam [1:0] NORMAL = 2'd0;
    localparam [1:0] CRITICAL = 2'd1;
    localparam [1:0] FAULT = 2'd2;
    localparam [1:0] WARNING = 2'd3;

    (* keep = "true", mark_debug = "true" *) reg  [1:0] state;
    reg  [1:0] n_state;

    // bit[0] : Clock Monitor Fault
    // bit[1] : Watchdog Fault
    // bit[2] : CAN Bus-Off
    // bit[3] : ECC SEC
    // bit[4] : ECC DED

    wire [7:0] fault_vector;

    assign fault_vector = {
        3'b000, ded_ecc_fault, sec_ecc_fault, can_bus_off, wdt_fault, clk_fault
    };

    reg  [7:0] fault_vector_d;
    wire [7:0] new_fault_event;

    assign new_fault_event = fault_vector & ~fault_vector_d;



    always @(posedge safety_clk or negedge resetn) begin

        if (!resetn) fault_vector_d <= 8'd0;

        else fault_vector_d <= fault_vector;

    end





    // ============================================================

    // Fault Classification

    // ============================================================



    wire critical_event;

    wire major_event;

    wire warning_event;



    assign critical_event =

        new_fault_event[0] |

        new_fault_event[1] |

        new_fault_event[4];



    assign major_event = new_fault_event[2];



    assign warning_event = new_fault_event[3];





    // ============================================================

    // Fault Registers

    // ============================================================



    reg critical_fault_reg;

    reg major_fault_reg;

    reg warning_fault_reg;



    reg critical_fault_next;

    reg major_fault_next;

    reg warning_fault_next;



    reg [7:0] fault_id_reg;

    reg [7:0] fault_id_next;





    // ============================================================

    // State Sequential Logic

    // ============================================================



    always @(posedge safety_clk or negedge resetn) begin

        if (!resetn) begin

            state              <= NORMAL;



            critical_fault_reg <= 1'b0;

            major_fault_reg    <= 1'b0;

            warning_fault_reg  <= 1'b0;



            fault_id_reg       <= 8'd0;

        end else begin

            state              <= n_state;



            critical_fault_reg <= critical_fault_next;

            major_fault_reg    <= major_fault_next;

            warning_fault_reg  <= warning_fault_next;



            fault_id_reg       <= fault_id_next;

        end

    end





    // ============================================================

    // Next State / Fault Latch Logic

    // ============================================================



    always @(*) begin



        n_state = state;



        critical_fault_next = critical_fault_reg;

        major_fault_next    = major_fault_reg;

        warning_fault_next  = warning_fault_reg;



        fault_id_next = fault_id_reg;





        // --------------------------------------------------------

        // Critical Fault

        // --------------------------------------------------------



        if (clk_fault) begin

            critical_fault_next = 1'b1;

            fault_id_next[0]    = 1'b1;

        end



        if (wdt_fault) begin

            critical_fault_next = 1'b1;

            fault_id_next[1]    = 1'b1;

        end



        if (ded_ecc_fault) begin

            critical_fault_next = 1'b1;

            fault_id_next[4]    = 1'b1;

        end





        // --------------------------------------------------------

        // Major Fault

        // --------------------------------------------------------



        if (can_bus_off) begin

            major_fault_next = 1'b1;

            fault_id_next[2] = 1'b1;

        end





        // --------------------------------------------------------

        // Warning

        // --------------------------------------------------------



        if (warning_event) begin

            warning_fault_next = 1'b1;

            fault_id_next[3]   = 1'b1;

        end





        // --------------------------------------------------------

        // Fault Clear

        // --------------------------------------------------------



        if (fault_clear) begin



            // Critical source가 실제로 제거된 경우에만 clear

            if (!clk_fault && !wdt_fault && !ded_ecc_fault) begin



                critical_fault_next = 1'b0;



                fault_id_next[0] = 1'b0;

                fault_id_next[1] = 1'b0;

                fault_id_next[4] = 1'b0;

            end





            // CAN Bus-Off 해제 후 clear

            if (!can_bus_off) begin

                major_fault_next = 1'b0;

                fault_id_next[2] = 1'b0;

            end

        end





        // ========================================================

        // FSM

        //

        // Priority:

        // CRITICAL > FAULT > WARNING

        // ========================================================



        case (state)



            NORMAL: begin

                if (critical_fault_reg) n_state = CRITICAL;

                else if (major_fault_reg) n_state = FAULT;

                else if (warning_fault_reg) n_state = WARNING;

            end





            // ----------------------------------------------------

            // WARNING

            // SEC ECC

            // 1 clock 유지 후 자동 복귀

            // ----------------------------------------------------



            WARNING: begin



                warning_fault_next = 1'b0;



                if (critical_fault_reg) n_state = CRITICAL;

                else if (major_fault_reg) n_state = FAULT;

                else n_state = NORMAL;

            end





            // ----------------------------------------------------

            // FAULT

            // CAN Bus-Off

            // MCU fault_clear 전까지 유지

            // ----------------------------------------------------



            FAULT: begin

                if (critical_fault_reg) n_state = CRITICAL;

                else if (major_fault_reg) n_state = FAULT;

                else if (warning_fault_reg) n_state = WARNING;

                else n_state = NORMAL;

            end





            // ----------------------------------------------------

            // CRITICAL

            // WDT / Clock Monitor / ECC DED

            // ----------------------------------------------------



            CRITICAL: begin

                if (critical_fault_reg) n_state = CRITICAL;

                else if (major_fault_reg) n_state = FAULT;

                else if (warning_fault_reg) n_state = WARNING;

                else n_state = NORMAL;

            end





            default: begin

                n_state = NORMAL;

            end



        endcase

    end





    // ============================================================

    // Fault Log Writer

    //

    // new_fault_event 발생 시 1회 저장

    //

    // [55:8] = timestamp[47:0]

    // [ 7:0] = fault event

    // ============================================================



    reg        log_wr_en_reg;

    reg [ 7:0] log_wr_addr_reg;

    reg [55:0] log_wr_data_reg;



    reg [ 7:0] log_ptr;





    always @(posedge safety_clk or negedge resetn) begin

        if (!resetn) begin



            log_wr_en_reg   <= 1'b0;

            log_wr_addr_reg <= 8'd0;

            log_wr_data_reg <= 56'd0;

            log_ptr         <= 8'd0;



        end else begin



            // Default : 1 clock pulse

            log_wr_en_reg <= 1'b0;





            if (|new_fault_event) begin



                log_wr_en_reg   <= 1'b1;

                log_wr_addr_reg <= log_ptr;



                log_wr_data_reg <= {timestamp[47:0], new_fault_event};





                // Circular Buffer

                if (log_ptr >= LOG_SIZE - 1) log_ptr <= 8'd0;

                else log_ptr <= log_ptr + 1'b1;

            end

        end

    end





    assign log_wr_en   = log_wr_en_reg;

    assign log_wr_addr = log_wr_addr_reg;

    assign log_wr_data = log_wr_data_reg;





    // ============================================================

    // Fault IRQ

    //

    // CAN Bus-Off 발생 시 MCU interrupt

    // ============================================================



    assign fault_irq   = major_fault_reg;





    // ============================================================

    // Safe State

    // ============================================================



    assign safe_state  = ~critical_fault_reg;





    // ============================================================

    // Active Critical Source

    //

    // critical_fault_reg는 fault 이력/상태 유지를 위한 sticky latch이다.

    // 실제 reset_req는 현재 Critical source가 살아 있을 때만 발생한다.

    //

    // Watchdog:

    //   wdt_fault = 1 -> reset sequence 동작

    //   wdt_fault = 0 -> reset sequence 즉시 중단

    // ============================================================



    (* keep = "true", mark_debug = "true" *) wire critical_source_active;



    assign critical_source_active = clk_fault | wdt_fault | ded_ecc_fault;



    // ============================================================

    // Critical Reset Sequencer

    //

    // reset_delay는 Register Map에서 설정

    //

    // reset_delay = 0 또는 1

    //     → CRITICAL 진입 후 즉시 reset_req

    //

    // reset_delay > 1

    //     → 설정된 safety_clk 만큼 대기

    // ============================================================



    reg [ 7:0] reset_cnt;

    reg [31:0] reset_retry_cnt;

    reg        reset_req_reg;

    reg        reset_issued;





    always @(posedge safety_clk or negedge resetn) begin

        if (!resetn) begin



            reset_cnt       <= 8'd0;

            reset_retry_cnt <= 32'd0;

            reset_req_reg   <= 1'b0;

            reset_issued    <= 1'b0;



        end else begin



            // reset_req = 1 clock pulse

            reset_req_reg <= 1'b0;





            // FSM의 CRITICAL latch가 남아 있더라도,

            // 실제 Critical source가 제거되면 reset을 더 이상 발생시키지 않는다.

            //

            // Watchdog:

            //   kick 정상 -> wdt_fault = 0 -> reset sequencer idle

            //   kick 중단 -> timeout -> wdt_fault = 1 -> reset sequence

            if ((state != CRITICAL) || !critical_source_active) begin



                reset_cnt    <= 8'd0;

                reset_retry_cnt <= 32'd0;

                reset_issued <= 1'b0;



            end else begin



                if (!reset_issued) begin



                    if (reset_delay <= 8'd1) begin



                        reset_req_reg <= 1'b1;

                        reset_issued <= 1'b1;

                        reset_retry_cnt <= 32'd0;



                    end else if (reset_cnt >= reset_delay - 1'b1) begin



                        reset_req_reg <= 1'b1;

                        reset_issued <= 1'b1;

                        reset_retry_cnt <= 32'd0;



                    end else begin



                        reset_cnt <= reset_cnt + 1'b1;



                    end

                end else begin



                    // 첫 reset 이후에도 CRITICAL fault가 계속되면 CPU가

                    // watchdog을 kick할 기회를 준 뒤 reset pulse를 재발행한다.

                    if ((RESET_RETRY_CYCLES <= 1) ||

                        (reset_retry_cnt >= RESET_RETRY_CYCLES - 1)) begin



                        reset_req_reg   <= 1'b1;

                        reset_retry_cnt <= 32'd0;



                    end else begin



                        reset_retry_cnt <= reset_retry_cnt + 1'b1;



                    end

                end

            end

        end

    end





    assign reset_req = reset_req_reg;


endmodule
