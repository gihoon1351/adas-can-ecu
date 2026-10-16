`timescale 1ns / 1ps

module secded_decoder (
    input  wire [63:0] data_in,
    output reg  [55:0] data_out,
    output reg         sec_ecc_fault,
    output reg         ded_ecc_fault
);
    integer i;
    integer data_idx;

    reg [61:0] hamming_code;
    reg [61:0] corrected_code;

    reg overall_check;
    reg [5:0] syndrome;

    always @(*) begin
        // -------------------------------------------------
        // Default
        // -------------------------------------------------
        hamming_code  = data_in[61:0];
        corrected_code = data_in[61:0];

        syndrome     = 6'd0;
        overall_check = 1'b0;

        sec_ecc_fault = 1'b0;
        ded_ecc_fault = 1'b0;

        data_out = 56'd0;
        data_idx = 0;
        // -------------------------------------------------
        // Syndrome 계산
        //
        // Hamming position:
        // 1, 2, 4, 8, 16, 32 = parity
        // -------------------------------------------------
        for (i = 1; i <= 62; i = i + 1) begin
            if ((i & 1) != 0)
                syndrome[0] = syndrome[0] ^ hamming_code[i-1];
            if ((i & 2) != 0)
                syndrome[1] = syndrome[1] ^ hamming_code[i-1];
            if ((i & 4) != 0)
                syndrome[2] = syndrome[2] ^ hamming_code[i-1];
            if ((i & 8) != 0)
                syndrome[3] = syndrome[3] ^ hamming_code[i-1];
            if ((i & 16) != 0)
                syndrome[4] = syndrome[4] ^ hamming_code[i-1];
            if ((i & 32) != 0)
                syndrome[5] =syndrome[5] ^ hamming_code[i-1];
        end
        // -------------------------------------------------
        // Overall parity 검사
        //
        // data_in[62] = Overall parity
        // -------------------------------------------------
        overall_check = (^hamming_code) ^ data_in[62];

        // =================================================
        // SECDED 판정
        // =================================================

        // -------------------------------------------------
        // 1. No Error
        // -------------------------------------------------
        if ((syndrome == 0) && (overall_check == 0)) begin
            sec_ecc_fault = 1'b0;
            ded_ecc_fault = 1'b0;
        end

        // -------------------------------------------------
        // 2. Single-bit Error
        //
        // syndrome으로 error position 확인 가능
        // -------------------------------------------------
        else if ((syndrome != 0) && (overall_check == 1)) begin
            if (syndrome <= 62) begin
                corrected_code[syndrome-1] = ~corrected_code[syndrome-1];
                sec_ecc_fault = 1'b1;
                ded_ecc_fault = 1'b0;
            end else begin
                // syndrome = 63
                // 정상적인 1-bit error 위치가 아님
                sec_ecc_fault = 1'b0;
                ded_ecc_fault = 1'b1;
            end
        end
        // -------------------------------------------------
        // 3. Double-bit Error
        // -------------------------------------------------
        else if ((syndrome != 0) && (overall_check == 0)) begin
            sec_ecc_fault = 1'b0;
            ded_ecc_fault = 1'b1;
        end
        // -------------------------------------------------
        // 4. Overall parity bit error
        //
        // data 자체는 정상
        // -------------------------------------------------
        else begin
            // syndrome == 0
            // overall_check == 1
            sec_ecc_fault = 1'b1;
            ded_ecc_fault = 1'b0;
        end
        // =================================================
        // Hamming parity 제거 → 56-bit Data 추출
        // =================================================
        data_idx = 0;
        for (i = 1; i <= 62; i = i + 1) begin
            if ((i != 1)  && (i != 2)  && (i != 4)  && (i != 8)  && (i != 16) && (i != 32)) begin
                data_out[data_idx] = corrected_code[i-1];
                data_idx = data_idx + 1;
            end
        end
    end
endmodule