`timescale 1ns / 1ps

module weight_flow_controller (
    //contorl unit
    input 					w_word_rdy		,
    output reg				w_word_vld		,
    input 					axis_aclk		,
    input 					axis_aresetn	,
    input [31:0] 			s_axis_tdata	,  //weight 8bit x 4 set
    input					s_axis_tvalid	,
    output					s_axis_tready	,
    input wire				s_axis_tuser	,
    input wire				s_axis_tlast	,
    output [63:0] 			weight_in_0		,
    output [63:0] 			weight_in_1		,
    output [63:0] 			weight_in_2		,
    output [63:0] 			weight_in_3		,
    output [63:0] 			weight_in_4		,
    output [63:0] 			weight_in_5		,
    output [63:0] 			weight_in_6		,
    output [63:0] 			weight_in_7		,
    output [63:0] 			weight_in_8		,
    output [63:0] 			weight_in_9		,
    output [63:0] 			weight_in_10	,
    output [63:0] 			weight_in_11	,
    output [63:0] 			weight_in_12	,
    output [63:0] 			weight_in_13	,
    output [63:0] 			weight_in_14	,
    output [63:0] 			weight_in_15	 
);
  assign s_axis_tready = w_word_rdy;
  reg [4:0] count;  //가중치가 (8bit x 4)로 32세트가 들어오면 출력 
  reg [1023:0] weight_flow_buffer;

  always @(posedge axis_aclk) begin
    if (!axis_aresetn) begin
      count              <= 5'd0;
      weight_flow_buffer <= 1024'd0;
      w_word_vld         <= 1'b0;
    end else begin
      // Control Unit이 완성된 word를 받으면 valid 해제
      if (w_word_vld && w_word_rdy) w_word_vld <= 1'b0;

      if (s_axis_tvalid && s_axis_tready) begin
        weight_flow_buffer <= {weight_flow_buffer[991:0], s_axis_tdata};

        if (count == 5'd31) begin
          count      <= 5'd0;
          w_word_vld <= 1'b1;
        end else begin
          count <= count + 1'b1;
        end
      end
    end
  end

  assign weight_in_15 = weight_flow_buffer[63:0];
  assign weight_in_14 = weight_flow_buffer[127:64];
  assign weight_in_13 = weight_flow_buffer[191:128];
  assign weight_in_12 = weight_flow_buffer[255:192];
  assign weight_in_11 = weight_flow_buffer[319:256];
  assign weight_in_10 = weight_flow_buffer[383:320];
  assign weight_in_9  = weight_flow_buffer[447:384];
  assign weight_in_8  = weight_flow_buffer[511:448];
  assign weight_in_7  = weight_flow_buffer[575:512];
  assign weight_in_6  = weight_flow_buffer[639:576];
  assign weight_in_5  = weight_flow_buffer[703:640];
  assign weight_in_4  = weight_flow_buffer[767:704];
  assign weight_in_3  = weight_flow_buffer[831:768];
  assign weight_in_2  = weight_flow_buffer[895:832];
  assign weight_in_1  = weight_flow_buffer[959:896];
  assign weight_in_0  = weight_flow_buffer[1023:960];

endmodule
