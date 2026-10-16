`timescale 1ns / 1ps

module ppu_control_unit (
    input wire clk,
    input wire rst_n,

    // from external control unit / Conv
    input wire       conv_valid_ppu,
    input wire [4:0] ppu_channel_group_idx,
    input wire [2:0] ppu_layer_idx,

    // from MaxPool
    input wire pool_valid,
    input wire pool_line_start_in,
    input wire pool_line_end_in,

    // to Conv / external control unit
    output wire ppu_ready_conv,
    output reg  ppu_out_valid,
    output reg  pool_line_start,
    output reg  pool_line_end,

    // to ROM
    output reg [6:0] param_addr,

    // to MaxPool
    output reg  [7:0] feature_width,
    output wire       pool_in_valid,
    output reg        pool_clear,

    // to layer mux
    output reg [1:0] layer_mode
);

  localparam MODE_POOL  = 2'b00;
  localparam MODE_RELU  = 2'b01;
  localparam MODE_QUANT = 2'b10;

  reg valid_d1;
  reg valid_d2;
  reg valid_d3;

  reg [6:0] layer_base_addr;

  reg [2:0] prev_layer_idx;
  reg [4:0] prev_channel_group_idx;

  wire conv_transfer;
  wire quant_valid;
  wire relu_valid;

  wire selected_valid;
  reg  selected_valid_d1;

  // PPU는 현재 내부 stall이 없으므로 AXI ready를 항상 1로 유지
  assign ppu_ready_conv = 1'b1;

  // AXI handshake 기준 실제 입력 데이터가 PPU로 들어온 순간
  assign conv_transfer = conv_valid_ppu && ppu_ready_conv;

  // Quant/ReLU path valid
  assign quant_valid = valid_d3;
  assign relu_valid  = valid_d3;

  // MaxPool input valid
  assign pool_in_valid = valid_d3 && (layer_mode == MODE_POOL);

  // 128bit 결과 기준 selected valid
  assign selected_valid =
      (layer_mode == MODE_POOL)  ? pool_valid  :
      (layer_mode == MODE_RELU)  ? relu_valid   :
      (layer_mode == MODE_QUANT) ? quant_valid  :
                                   1'b0;

  // Bias Add reg + Quant reg latency 맞추기
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      valid_d1 <= 1'b0;
      valid_d2 <= 1'b0;
	  valid_d3 <= 1'b0;
    end else begin
      valid_d1 <= conv_transfer;
      valid_d2 <= valid_d1;
      valid_d3 <= valid_d2;
    end
  end

  // selected_valid를 1clk delay해서
  // 최종 ppu_out_valid를 2clk 동안 유지
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      selected_valid_d1 <= 1'b0;
    end else begin
      selected_valid_d1 <= selected_valid;
    end
  end

  // layer_idx 기준 mode, feature_width, parameter base address 생성
  always @(*) begin
    layer_mode      = MODE_QUANT;
    feature_width   = 8'd0;
    layer_base_addr = 7'd0;

    case (ppu_layer_idx)
      3'd0: begin
        layer_mode      = MODE_POOL;
        feature_width   = 8'd224;
        layer_base_addr = 7'd0;
      end

      3'd1: begin
        layer_mode      = MODE_POOL;
        feature_width   = 8'd112;
        layer_base_addr = 7'd1;
      end

      3'd2: begin
        layer_mode      = MODE_POOL;
        feature_width   = 8'd56;
        layer_base_addr = 7'd3;
      end

      3'd3: begin
        layer_mode      = MODE_POOL;
        feature_width   = 8'd28;
        layer_base_addr = 7'd7;
      end

      3'd4: begin
        layer_mode      = MODE_POOL;
        feature_width   = 8'd14;
        layer_base_addr = 7'd15;
      end

      3'd5: begin
        layer_mode      = MODE_RELU;
        feature_width   = 8'd7;
        layer_base_addr = 7'd31;
      end

      3'd6: begin
        layer_mode      = MODE_QUANT;
        feature_width   = 8'd7;
        layer_base_addr = 7'd63;
      end

      default: begin
        layer_mode      = MODE_QUANT;
        feature_width   = 8'd0;
        layer_base_addr = 7'd0;
      end
    endcase
  end

  // Bias_ROM / Scale_ROM 공통 주소
  always @(*) begin
    param_addr = layer_base_addr + ppu_channel_group_idx;
  end

  // pool_clear 생성
  // layer 또는 channel group이 바뀌면 MaxPool 내부 x/y counter 초기화
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      prev_layer_idx         <= 3'd0;
      prev_channel_group_idx <= 5'd0;
      pool_clear             <= 1'b1;
    end else begin
      prev_layer_idx         <= ppu_layer_idx;
      prev_channel_group_idx <= ppu_channel_group_idx;

      if ((ppu_layer_idx != prev_layer_idx) ||
          (ppu_channel_group_idx != prev_channel_group_idx)) begin
        pool_clear <= 1'b1;
      end else begin
        pool_clear <= 1'b0;
      end
    end
  end

  // 최종 output valid / line 정보
  always @(*) begin
    // selected_valid가 뜬 clk + 다음 clk까지 총 2clk 동안 valid 유지
    ppu_out_valid   = selected_valid | selected_valid_d1;

    pool_line_start = 1'b0;
    pool_line_end   = 1'b0;

    if (layer_mode == MODE_POOL) begin
      pool_line_start = pool_line_start_in;
      pool_line_end   = pool_line_end_in;
    end
  end

endmodule
