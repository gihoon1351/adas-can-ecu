`timescale 1ns / 1ps

module secded_encoder (
    input  wire [55:0] data_in,
    output reg  [63:0] code_out
);

    integer        i;
    integer        data_idx;

    reg     [61:0] hamming_code;
    reg     [ 5:0] parity;
    reg            overall_parity;

    always @(*) begin

        hamming_code   = 62'd0;
        parity         = 6'd0;
        overall_parity = 1'b0;
        data_idx       = 0;

        // 56-bit Data 배치
        for (i = 1; i <= 62; i = i + 1) begin
            if ((i != 1)  && (i != 2)  && (i != 4)  && (i != 8)  && (i != 16) && (i != 32)) begin
                hamming_code[i-1] = data_in[data_idx];
                data_idx = data_idx + 1;
            end
        end

        // Hamming parity 계산
        for (i = 1; i <= 62; i = i + 1) begin
            if ((i & 1) != 0) parity[0] = parity[0] ^ hamming_code[i-1];
            if ((i & 2) != 0) parity[1] = parity[1] ^ hamming_code[i-1];
            if ((i & 4) != 0) parity[2] = parity[2] ^ hamming_code[i-1];
            if ((i & 8) != 0) parity[3] = parity[3] ^ hamming_code[i-1];
            if ((i & 16) != 0) parity[4] = parity[4] ^ hamming_code[i-1];
            if ((i & 32) != 0) parity[5] = parity[5] ^ hamming_code[i-1];
        end

        // Parity 삽입
        hamming_code[0] = parity[0];
        hamming_code[1] = parity[1];
        hamming_code[3] = parity[2];
        hamming_code[7] = parity[3];
        hamming_code[15] = parity[4];
        hamming_code[31] = parity[5];

        // Extended Hamming overall parity
        overall_parity = ^hamming_code;

        // 64-bit 출력
        code_out = {1'b0, overall_parity, hamming_code};
    end
endmodule
