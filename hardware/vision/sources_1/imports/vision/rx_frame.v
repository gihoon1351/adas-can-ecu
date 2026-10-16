`timescale 1ns / 1ps

module rx_frame (
    // input top
    input wire clk,
    input wire resetn,
    input wire software_reset,

    // input from can_tx
    input wire bus_off,

    // input from tq_gen
    input wire falling_edge,
    input wire data_valid,
    input wire data_in,
    input wire stuff_error,
    input wire crc_error,

    // output to crc15
    output reg crc_calc_en,
    output reg crc_rx_en,
    output reg crc_check_en,
    output reg destuff_en,

    // output to tx
    output reg  rx_error_flag,
    output wire rx_rec_passive,
    output reg  ack_out,
    output wire rx_idle,
    output reg  frame_done,

    // output to cpu
    output reg [10:0] rx_id,
    output reg [ 3:0] rx_dlc,
    output reg [63:0] rx_data,
    output reg        rx_rtr,
    output reg        rx_ide
);

    localparam [4:0]
        RX_BUS_INTEGRATION = 5'd0,
        RX_IDLE            = 5'd1,
        RX_SOF             = 5'd2,
        RX_ID              = 5'd3,
        RX_RTR             = 5'd4,
        RX_IDE             = 5'd5,
        RX_R0              = 5'd6,
        RX_DLC             = 5'd7,
        RX_DATA            = 5'd8,
        RX_CRC             = 5'd9,
        RX_CRC_DELIMITER   = 5'd10,
        RX_ACK             = 5'd11,
        RX_ACK_DELIMITER   = 5'd12,
        RX_EOF             = 5'd13,
        RX_IFS             = 5'd14,
        RX_ERROR_FLAG      = 5'd15,
        RX_ERROR_WAIT      = 5'd16,
        RX_ERROR_DELIMITER = 5'd17;

    reg [4:0] c_state, n_state;

    reg [6:0] bit_cnt_reg, bit_cnt_next;
    reg [3:0] recessive_cnt_reg, recessive_cnt_next;

    reg [7:0] rec_reg, rec_next;

    reg [10:0] rx_id_shift_reg, rx_id_shift_next;
    reg [3:0] rx_dlc_shift_reg, rx_dlc_shift_next;
    reg [63:0] rx_data_shift_reg, rx_data_shift_next;
    reg [14:0] rx_crc_shift_reg, rx_crc_shift_next;

    reg crc_error_reg, crc_error_next;
    reg         ack_out_next;

    reg  [10:0] rx_id_next;
    reg  [ 3:0] rx_dlc_next;
    reg  [63:0] rx_data_next;
    reg         rx_rtr_next;
    reg         rx_ide_next;

    wire [ 6:0] data_count;

    assign data_count     = (rx_dlc > 4'd8) ? 7'd64 : {rx_dlc, 3'b000};

    assign rx_rec_passive = (rec_reg >= 8'd128);
    assign rx_idle        = (c_state == RX_IDLE);

    // can_tx consumes byte 0 from data[63:56].  Keep the same interface at the
    // RX boundary so every user of rx_data sees bytes in CAN wire order.
    function [63:0] align_rx_data_msb;
        input [63:0] data_in_value;
        input [ 3:0] dlc_value;
        begin
            case (dlc_value)
                4'd0: align_rx_data_msb = 64'd0;
                4'd1: align_rx_data_msb = {data_in_value[ 7:0], 56'd0};
                4'd2: align_rx_data_msb = {data_in_value[15:0], 48'd0};
                4'd3: align_rx_data_msb = {data_in_value[23:0], 40'd0};
                4'd4: align_rx_data_msb = {data_in_value[31:0], 32'd0};
                4'd5: align_rx_data_msb = {data_in_value[39:0], 24'd0};
                4'd6: align_rx_data_msb = {data_in_value[47:0], 16'd0};
                4'd7: align_rx_data_msb = {data_in_value[55:0],  8'd0};
                default: align_rx_data_msb = data_in_value;
            endcase
        end
    endfunction

    always @(posedge clk or negedge resetn) begin
        if (!resetn || software_reset) begin
            c_state           <= RX_BUS_INTEGRATION;
            bit_cnt_reg       <= 7'd0;
            recessive_cnt_reg <= 4'd0;
            rec_reg           <= 8'd0;

            rx_id_shift_reg   <= 11'd0;
            rx_dlc_shift_reg  <= 4'd0;
            rx_data_shift_reg <= 64'd0;
            rx_crc_shift_reg  <= 15'd0;

            crc_error_reg     <= 1'b0;
            ack_out           <= 1'b1;

            rx_id             <= 11'd0;
            rx_dlc            <= 4'd0;
            rx_data           <= 64'd0;
            rx_rtr            <= 1'b0;
            rx_ide            <= 1'b0;
        end else begin
            c_state           <= n_state;
            bit_cnt_reg       <= bit_cnt_next;
            recessive_cnt_reg <= recessive_cnt_next;

            if (bus_off) begin
                rec_reg <= 8'd0;
            end else begin
                rec_reg <= rec_next;
            end

            rx_id_shift_reg   <= rx_id_shift_next;
            rx_dlc_shift_reg  <= rx_dlc_shift_next;
            rx_data_shift_reg <= rx_data_shift_next;
            rx_crc_shift_reg  <= rx_crc_shift_next;

            crc_error_reg     <= crc_error_next;
            ack_out           <= ack_out_next;

            rx_id             <= rx_id_next;
            rx_dlc            <= rx_dlc_next;
            rx_data           <= rx_data_next;
            rx_rtr            <= rx_rtr_next;
            rx_ide            <= rx_ide_next;
        end
    end

    always @(*) begin
        n_state            = c_state;
        bit_cnt_next       = bit_cnt_reg;
        recessive_cnt_next = recessive_cnt_reg;
        rec_next           = rec_reg;

        rx_id_shift_next   = rx_id_shift_reg;
        rx_dlc_shift_next  = rx_dlc_shift_reg;
        rx_data_shift_next = rx_data_shift_reg;
        rx_crc_shift_next  = rx_crc_shift_reg;

        crc_error_next     = crc_error_reg;
        ack_out_next       = ack_out;

        rx_id_next         = rx_id;
        rx_dlc_next        = rx_dlc;
        rx_data_next       = rx_data;
        rx_rtr_next        = rx_rtr;
        rx_ide_next        = rx_ide;

        crc_calc_en        = 1'b0;
        crc_rx_en          = 1'b0;
        crc_check_en       = 1'b0;
        destuff_en         = 1'b0;
        rx_error_flag      = 1'b0;
        frame_done         = 1'b0;

        case (c_state)
            RX_BUS_INTEGRATION: begin
                ack_out_next = 1'b1;

                if (data_valid) begin
                    if (data_in) begin
                        if (recessive_cnt_reg == 4'd10) begin
                            recessive_cnt_next = 4'd0;
                            n_state            = RX_IDLE;
                        end else begin
                            recessive_cnt_next = recessive_cnt_reg + 4'd1;
                        end
                    end else begin
                        recessive_cnt_next = 4'd0;
                    end
                end
            end

            RX_IDLE: begin
                ack_out_next = 1'b1;

                if (falling_edge) begin
                    bit_cnt_next       = 7'd0;
                    rx_id_shift_next   = 11'd0;
                    rx_dlc_shift_next  = 4'd0;
                    rx_data_shift_next = 64'd0;
                    rx_crc_shift_next  = 15'd0;
                    crc_error_next     = 1'b0;

                    rx_id_next         = 11'd0;
                    rx_dlc_next        = 4'd0;
                    rx_data_next       = 64'd0;
                    rx_rtr_next        = 1'b0;
                    rx_ide_next        = 1'b0;

                    n_state            = RX_SOF;
                end
            end

            RX_SOF: begin
                crc_calc_en = 1'b1;
                destuff_en  = 1'b1;
                if (stuff_error) begin
                    if (rec_reg < 8'hff) begin
                        rec_next = rec_reg + 8'd1;
                    end else begin
                        rec_next = 8'hff;
                    end
                    bit_cnt_next = 7'd0;
                    n_state      = RX_ERROR_FLAG;
                end else if (data_valid) begin
                    if (!data_in) begin
                        n_state = RX_ID;
                    end else begin
                        n_state = RX_IDLE;
                    end
                end
            end

            RX_ID: begin
                crc_calc_en = 1'b1;
                destuff_en  = 1'b1;
                if (stuff_error) begin
                    if (rec_reg < 8'hff) begin
                        rec_next = rec_reg + 8'd1;
                    end else begin
                        rec_next = 8'hff;
                    end
                    bit_cnt_next = 7'd0;
                    n_state      = RX_ERROR_FLAG;
                end else if (data_valid) begin
                    rx_id_shift_next = {rx_id_shift_reg[9:0], data_in};
                    if (bit_cnt_reg == 7'd10) begin
                        rx_id_next   = {rx_id_shift_reg[9:0], data_in};
                        bit_cnt_next = 7'd0;
                        n_state      = RX_RTR;
                    end else begin
                        bit_cnt_next = bit_cnt_reg + 7'd1;
                    end
                end
            end

            RX_RTR: begin
                crc_calc_en = 1'b1;
                destuff_en  = 1'b1;
                if (stuff_error) begin
                    if (rec_reg < 8'hff) begin
                        rec_next = rec_reg + 8'd1;
                    end else begin
                        rec_next = 8'hff;
                    end
                    bit_cnt_next = 7'd0;
                    n_state      = RX_ERROR_FLAG;
                end else if (data_valid) begin
                    rx_rtr_next = data_in;
                    n_state     = RX_IDE;
                end
            end

            RX_IDE: begin
                crc_calc_en = 1'b1;
                destuff_en  = 1'b1;
                if (stuff_error) begin
                    if (rec_reg < 8'hff) begin
                        rec_next = rec_reg + 8'd1;
                    end else begin
                        rec_next = 8'hff;
                    end
                    bit_cnt_next = 7'd0;
                    n_state      = RX_ERROR_FLAG;
                end else if (data_valid) begin
                    rx_ide_next = data_in;
                    n_state     = RX_R0;
                end
            end

            RX_R0: begin
                crc_calc_en = 1'b1;
                destuff_en  = 1'b1;
                if (stuff_error) begin
                    if (rec_reg < 8'hff) begin
                        rec_next = rec_reg + 8'd1;
                    end else begin
                        rec_next = 8'hff;
                    end
                    bit_cnt_next = 7'd0;
                    n_state      = RX_ERROR_FLAG;
                end else if (data_valid) begin
                    bit_cnt_next = 7'd0;
                    n_state      = RX_DLC;
                end
            end

            RX_DLC: begin
                crc_calc_en = 1'b1;
                destuff_en  = 1'b1;
                if (stuff_error) begin
                    if (rec_reg < 8'hff) begin
                        rec_next = rec_reg + 8'd1;
                    end else begin
                        rec_next = 8'hff;
                    end
                    bit_cnt_next = 7'd0;
                    n_state      = RX_ERROR_FLAG;
                end else if (data_valid) begin
                    rx_dlc_shift_next = {rx_dlc_shift_reg[2:0], data_in};
                    if (bit_cnt_reg == 7'd3) begin
                        rx_dlc_next  = {rx_dlc_shift_reg[2:0], data_in};
                        bit_cnt_next = 7'd0;
                        if (rx_rtr || (rx_dlc_next == 4'd0)) begin
                            n_state = RX_CRC;
                        end else begin
                            n_state = RX_DATA;
                        end
                    end else begin
                        bit_cnt_next = bit_cnt_reg + 7'd1;
                    end
                end
            end

            RX_DATA: begin
                crc_calc_en = 1'b1;
                destuff_en  = 1'b1;
                if (stuff_error) begin
                    if (rec_reg < 8'hff) begin
                        rec_next = rec_reg + 8'd1;
                    end else begin
                        rec_next = 8'hff;
                    end
                    bit_cnt_next = 7'd0;
                    n_state      = RX_ERROR_FLAG;
                end else if (data_valid) begin
                    rx_data_shift_next = {rx_data_shift_reg[62:0], data_in};
                    if (bit_cnt_reg == (data_count - 7'd1)) begin
                        // Wrong RX interface format for DLC<8:
                        // rx_data_next = {rx_data_shift_reg[62:0], data_in};
                        // That leaves a short payload in the LSBs and forced
                        // every caller to realign it before passing it to TX.
                        rx_data_next = align_rx_data_msb(
                            {rx_data_shift_reg[62:0], data_in}, rx_dlc);
                        bit_cnt_next = 7'd0;
                        n_state      = RX_CRC;
                    end else begin
                        bit_cnt_next = bit_cnt_reg + 7'd1;
                    end
                end
            end

            RX_CRC: begin
                crc_rx_en  = 1'b1;
                destuff_en = 1'b1;
                if (stuff_error) begin
                    if (rec_reg < 8'hff) begin
                        rec_next = rec_reg + 8'd1;
                    end else begin
                        rec_next = 8'hff;
                    end
                    bit_cnt_next = 7'd0;
                    n_state      = RX_ERROR_FLAG;
                end else if (data_valid) begin
                    rx_crc_shift_next = {rx_crc_shift_reg[13:0], data_in};
                    if (bit_cnt_reg == 7'd14) begin
                        bit_cnt_next = 7'd0;
                        n_state      = RX_CRC_DELIMITER;
                    end else begin
                        bit_cnt_next = bit_cnt_reg + 7'd1;
                    end
                end
            end

            RX_CRC_DELIMITER: begin
                crc_check_en = 1'b1;
                destuff_en   = 1'b1;
                if (stuff_error) begin
                    if (rec_reg < 8'hff) begin
                        rec_next = rec_reg + 8'd1;
                    end else begin
                        rec_next = 8'hff;
                    end
                    bit_cnt_next = 7'd0;
                    n_state      = RX_ERROR_FLAG;
                end else if (data_valid) begin
                    crc_error_next = crc_error;
                    if (!data_in) begin
                        if (rec_reg < 8'hff) begin
                            rec_next = rec_reg + 8'd1;
                        end else begin
                            rec_next = 8'hff;
                        end
                        bit_cnt_next = 7'd0;
                        n_state      = RX_ERROR_FLAG;
                    end else if (crc_error) begin
                        if (rec_reg != 8'hff) begin
                            rec_next = rec_reg + 8'd1;
                        end
                        ack_out_next = 1'b1;
                        n_state      = RX_ACK;
                    end else begin
                        ack_out_next = 1'b0;
                        n_state      = RX_ACK;
                    end
                end
            end

            RX_ACK: begin
                if (data_valid) begin
                    ack_out_next = 1'b1;
                    n_state      = RX_ACK_DELIMITER;
                end
            end

            RX_ACK_DELIMITER: begin
                if (data_valid) begin
                    if (!data_in) begin
                        if (rec_reg < 8'hff) begin
                            rec_next = rec_reg + 8'd1;
                        end else begin
                            rec_next = 8'hff;
                        end
                        bit_cnt_next = 7'd0;
                        n_state      = RX_ERROR_FLAG;
                    end else if (crc_error_reg) begin
                        bit_cnt_next = 7'd0;
                        n_state      = RX_ERROR_FLAG;
                    end else begin
                        bit_cnt_next = 7'd0;
                        n_state      = RX_EOF;
                    end
                end
            end

            RX_EOF: begin
                if (data_valid) begin
                    if (!data_in && (bit_cnt_reg < 7'd6)) begin
                        if (rec_reg < 8'hff) begin
                            rec_next = rec_reg + 8'd1;
                        end else begin
                            rec_next = 8'hff;
                        end
                        bit_cnt_next = 7'd0;
                        n_state      = RX_ERROR_FLAG;
                    end else if (bit_cnt_reg == 7'd6) begin
                        bit_cnt_next = 7'd0;
                        frame_done   = 1'b1;
                        if (rec_reg == 8'd0) begin
                            rec_next = 8'd0;
                        end else if (rec_reg <= 8'd127) begin
                            rec_next = rec_reg - 8'd1;
                        end else begin
                            rec_next = 8'd127;
                        end
                        n_state = RX_IFS;
                    end else begin
                        bit_cnt_next = bit_cnt_reg + 7'd1;
                    end
                end
            end

            RX_IFS: begin
                if (data_valid) begin
                    if (data_in) begin
                        if (bit_cnt_reg == 7'd2) begin
                            bit_cnt_next = 7'd0;
                            n_state      = RX_IDLE;
                        end else begin
                            bit_cnt_next = bit_cnt_reg + 7'd1;
                        end
                    end else begin
                        bit_cnt_next = 7'd0;
                    end
                end
            end

            RX_ERROR_FLAG: begin
                rx_error_flag = 1'b1;
                if (data_valid) begin
                    if (bit_cnt_reg == 7'd5) begin
                        bit_cnt_next = 7'd0;
                        n_state      = RX_ERROR_WAIT;
                    end else begin
                        bit_cnt_next = bit_cnt_reg + 7'd1;
                    end
                end
            end

            RX_ERROR_WAIT: begin
                if (data_valid) begin
                    if (data_in) begin
                        bit_cnt_next = 7'd0;
                        n_state      = RX_ERROR_DELIMITER;
                    end else begin
                        bit_cnt_next = 7'd0;
                    end
                end
            end

            RX_ERROR_DELIMITER: begin
                if (data_valid) begin
                    if (data_in) begin
                        if (bit_cnt_reg == 7'd6) begin
                            bit_cnt_next = 7'd0;
                            n_state      = RX_IFS;
                        end else begin
                            bit_cnt_next = bit_cnt_reg + 7'd1;
                        end
                    end else begin
                        bit_cnt_next = 7'd0;
                        n_state      = RX_ERROR_WAIT;
                    end
                end
            end

            default: begin
                n_state = RX_IDLE;
            end
        endcase

    end

endmodule
