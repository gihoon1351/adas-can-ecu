`timescale 1ns / 1ps
// 2x2 MaxPool, stride 2
// lane0  = data[127:120]
// lane15 = data[7:0]

module max_pool #(
    parameter LANES     = 16,
    parameter DATA_W    = 8,
    parameter ADDR_W    = 8,
    parameter MAX_WIDTH = 224
) (
    input  wire                         clk,
    input  wire                         rst_n,

    input  wire [LANES*DATA_W-1:0]      relu_data,
    input  wire                         pool_in_valid,
    input  wire                         pool_clear,
    input  wire [ADDR_W-1:0]            feature_width,

    output reg  [LANES*DATA_W-1:0]      pool_data,
    output reg                          pool_valid,
    output reg                          pool_line_start,
    output reg                          pool_line_end
);

    reg [ADDR_W-1:0] x_cnt;
    reg [ADDR_W-1:0] y_cnt;

    reg write_bank_sel;

	(* ram_style = "block" *)
    reg [LANES*DATA_W-1:0] line_buf0 [0:MAX_WIDTH-1];
	(* ram_style = "block" *)
    reg [LANES*DATA_W-1:0] line_buf1 [0:MAX_WIDTH-1];

	reg [127:0]	line_buf0_rdata;
	reg [127:0]	line_buf1_rdata;

    reg [LANES*DATA_W-1:0] left_data;
    reg [LANES*DATA_W-1:0] upper_left_data;

    wire [LANES*DATA_W-1:0] upper_data;
    wire [LANES*DATA_W-1:0] max_result;

    wire [ADDR_W-1:0] last_x;
	wire do_pool_d1;

	reg [127:0]	relu_data_d1;
	reg	[7:0]	x_cnt_d1;
	reg	[7:0]	y_cnt_d1;
	reg			bank_sel_d1;
	reg			valid_d1;

	wire write_en0;
	wire write_en1;

	assign write_en0 =
		pool_in_valid && !pool_clear && !write_bank_sel;

	assign write_en1 =
		pool_in_valid && !pool_clear && write_bank_sel;

	always @(posedge clk) begin
		if (write_en0)
			line_buf0[x_cnt] <= relu_data;

		line_buf0_rdata <= line_buf0[x_cnt];
	end

	always @(posedge clk) begin
		if (write_en1)
			line_buf1[x_cnt] <= relu_data;

		line_buf1_rdata <= line_buf1[x_cnt];
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			valid_d1	<= 1'b0;
		end else if (pool_clear) begin
			valid_d1	<= 1'b0;
		end else begin
			valid_d1	<= pool_in_valid;
			if (pool_in_valid) begin
				relu_data_d1	<= relu_data;
				x_cnt_d1		<= x_cnt;
				y_cnt_d1		<= y_cnt;
				bank_sel_d1		<= write_bank_sel;
			end
		end
	end

    assign last_x = feature_width - 1'b1;

    // 2x2 stride 2 기준으로 오른쪽 아래 위치에서 pool 결과 생성
	assign do_pool_d1 = valid_d1 && x_cnt_d1[0] && y_cnt_d1[0];

    // 현재 쓰는 bank의 반대쪽 bank가 이전 line
    assign upper_data = bank_sel_d1 ? line_buf0_rdata : line_buf1_rdata;


    function [DATA_W-1:0] max2;
        input [DATA_W-1:0] a;
        input [DATA_W-1:0] b;
        begin
            max2 = (a >= b) ? a : b;
        end
    endfunction

    genvar i;

    generate
        for (i = 0; i < LANES; i = i + 1) begin : GEN_MAX_POOL
            wire [DATA_W-1:0] current_lane;
            wire [DATA_W-1:0] left_lane;
            wire [DATA_W-1:0] upper_lane;
            wire [DATA_W-1:0] upper_left_lane;

            wire [DATA_W-1:0] max_upper;
            wire [DATA_W-1:0] max_lower;
            wire [DATA_W-1:0] max_lane;

            assign current_lane    = relu_data_d1      [(LANES-1-i)*DATA_W +: DATA_W];
            assign left_lane       = left_data      [(LANES-1-i)*DATA_W +: DATA_W];
            assign upper_lane      = upper_data     [(LANES-1-i)*DATA_W +: DATA_W];
            assign upper_left_lane = upper_left_data[(LANES-1-i)*DATA_W +: DATA_W];

            assign max_upper = max2(upper_left_lane, upper_lane);
            assign max_lower = max2(left_lane, current_lane);
            assign max_lane  = max2(max_upper, max_lower);

            assign max_result[(LANES-1-i)*DATA_W +: DATA_W] = max_lane;
        end
    endgenerate

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			x_cnt          <= 0;
			y_cnt          <= 0;
			write_bank_sel <= 0;
		end else if (pool_clear) begin
			x_cnt          <= 0;
			y_cnt          <= 0;
			write_bank_sel <= 0;
		end else if (pool_in_valid) begin
			if (x_cnt == last_x) begin
				x_cnt <= 0;

				if (y_cnt == last_x)
					y_cnt <= 0;
				else
					y_cnt <= y_cnt + 1'b1;

				write_bank_sel <= ~write_bank_sel;
			end else begin
				x_cnt <= x_cnt + 1'b1;
			end
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			left_data       <= 0;
			upper_left_data <= 0;
			pool_data       <= 0;
			pool_valid      <= 0;
			pool_line_start <= 0;
			pool_line_end   <= 0;
		end else if (pool_clear) begin
			left_data       <= 0;
			upper_left_data <= 0;
			pool_data       <= 0;
			pool_valid      <= 0;
			pool_line_start <= 0;
			pool_line_end   <= 0;
		end else begin
			pool_valid      <= 1'b0;
			pool_line_start <= 1'b0;
			pool_line_end   <= 1'b0;

			if (valid_d1) begin
				if (do_pool_d1) begin
					pool_data       <= max_result;
					pool_valid      <= 1'b1;
					pool_line_start <= (x_cnt_d1 == 1);
					pool_line_end   <= (x_cnt_d1 == last_x);
				end

				left_data       <= relu_data_d1;
				upper_left_data <= upper_data;
			end
		end
	end

endmodule
