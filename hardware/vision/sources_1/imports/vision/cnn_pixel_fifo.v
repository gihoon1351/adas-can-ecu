`timescale 1ns / 1ps

// One clock domain: crop output -> selected-pixel FIFO -> CNN line buffer.
// SOF starts a frame; 224 accepted EOL beats complete it. The FIFO carries
// RGB24 + SOF + EOL together so backpressure cannot detach metadata.
// In WAIT_SOF, stray non-SOF pixels are consumed and discarded.
// After one complete input frame, accept/drop subsequent frames without
// blocking Sobel. A new frame is captured only when the CNN is ready at SOF;
// otherwise the whole frame is consumed and discarded without backpressure.
module cnn_pixel_fifo #(
    parameter integer ADDR_W = 8,             // 256 x 26 bits by default
    parameter integer FRAME_LINES = 224
) (
    input  wire        clk,
    input  wire        resetn,
    input  wire        cnn_done, // cnn_v1_0_0/irq_cnn; same FCLK0 domain
    input  wire [23:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tuser,
    input  wire        s_axis_tlast,
    output wire [23:0] m_axis_tdata,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready,
    output wire        m_axis_tuser,
    output wire        m_axis_tlast
);
  localparam integer DEPTH = 1 << ADDR_W; // 256
  localparam [1:0] WAIT_SOF = 2'd0, CAPTURE = 2'd1, DROP = 2'd2;

  // Prefer LUT RAM while on-chip BRAM is scarce. Neither the memory nor its
  // unoccupied output is reset; only pointers, count and frame state are.
  (* ram_style = "distributed" *) reg [25:0] mem [0:DEPTH-1]; //256
  reg [ADDR_W-1:0] wr_ptr, rd_ptr;
  reg [ADDR_W:0] count;
  reg [1:0] frame_state;
  reg [7:0] line_count;
  reg completed;
  reg cnn_done_d;

  // Keep an outstanding beat stable even when the CNN has finished: AXI4-S
  // does not allow TVALID to disappear while TREADY is low.
  assign m_axis_tvalid = (count != 0);
  assign {m_axis_tuser, m_axis_tlast, m_axis_tdata} = mem[rd_ptr];

  wire pop = m_axis_tvalid && m_axis_tready;
  wire space = (count < DEPTH) || pop;
  // Completion follows consumption of all 224x224 words. Returning to
  // WAIT_SOF does not itself start capture: the next frame starts only if
  // m_axis_tready is high together with its SOF beat.
  wire rearm = (frame_state == DROP) && completed &&
                (count == 0);

  wire start_capture = (frame_state == WAIT_SOF) &&
                       s_axis_tuser && m_axis_tready;

  // WAIT_SOF and DROP must never block the shared AXI broadcaster. This keeps
  // the parallel Gray/Sobel/x-axis branch running while the CNN is inactive.
  // CAPTURE may apply backpressure only after a CNN frame has actually begun.
  assign s_axis_tready = resetn &&
      ((frame_state == DROP) ||
       (frame_state == WAIT_SOF) || space);
  wire accept = s_axis_tvalid && s_axis_tready;
  wire push = accept &&
              ((frame_state == CAPTURE) ||
               start_capture);

  always @(posedge clk) begin
    if (!resetn) begin
      cnn_done_d <= 1'b0;
      completed  <= 1'b0;
    end else begin
      cnn_done_d <= cnn_done;
      if (rearm)
        completed <= 1'b0;
      else if (cnn_done && !cnn_done_d && (frame_state == DROP))
        completed <= 1'b1;
    end
  end

  always @(posedge clk) begin
    if (!resetn) begin
      wr_ptr      <= {ADDR_W{1'b0}};
      rd_ptr      <= {ADDR_W{1'b0}};
      count       <= {(ADDR_W+1){1'b0}};
      frame_state <= WAIT_SOF;
      line_count  <= 8'd0;
    end else begin
      if (push) begin
        mem[wr_ptr] <= {s_axis_tuser, s_axis_tlast, s_axis_tdata};
        wr_ptr <= wr_ptr + 1'b1;
      end
      if (pop) rd_ptr <= rd_ptr + 1'b1;
      case ({push, pop})
        2'b10: count <= count + 1'b1;
        2'b01: count <= count - 1'b1;
        default: count <= count;
      endcase

      if (rearm) begin
        frame_state <= WAIT_SOF;
        line_count <= 8'd0;
      end else if (push) begin
        if (frame_state == WAIT_SOF) begin
          frame_state <= CAPTURE;
          line_count <= 8'd0;
        end else if (s_axis_tlast) begin
          if (line_count == FRAME_LINES - 1) begin
            frame_state <= DROP;
            line_count <= 8'd0;
          end else begin
            line_count <= line_count + 1'b1;
          end
        end
      end
    end
  end
endmodule
