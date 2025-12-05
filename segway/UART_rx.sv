//==============================================================
// Module: UART_rx
// Description:
//   UART receiver for 8-bit data with start and stop bit framing.
//   Asserts 'rdy' when a full byte is received and valid.
//   Includes double-flop synchronizer to mitigate metastability on RX input.
//==============================================================
module UART_rx (
    input  logic       clk,      // System clock
    input  logic       rst_n,    // Active-low synchronous reset
    input  logic       RX,       // Asynchronous serial input line
    input  logic       clr_rdy,  // External signal to clear 'rdy' flag
    output logic [7:0] rx_data,  // Received 8-bit UART data
    output logic       rdy       // High when a complete byte is ready
);

  //------------------------------------------------------------
  // Internal Signals
  //------------------------------------------------------------
  // Control signals for UART reception flow
  logic       start;  // Asserted on detection of start bit
  logic       set_rdy;  // Asserted when a full byte is received
  logic       receiving;  // High during active reception
  logic       shift;  // Triggers bit sampling at baud intervals
  logic [3:0] bit_cnt;  // Counts received bits (start + 8 data + stop)

  //------------------------------------------------------------
  // Bit Counter
  //------------------------------------------------------------
  // Tracks progress through the 10-bit UART frame
  // Reset on start bit; incremented on each baud-aligned sample
  always_ff @(posedge clk) begin
    if (start) bit_cnt <= 0;
    else if (shift) bit_cnt <= bit_cnt + 1;
  end

  //------------------------------------------------------------
  // Baud Rate Counter
  //------------------------------------------------------------
  // Generates timing for bit sampling at 9600 baud (assuming 50 MHz clk)
  // Aligns sampling to center of each bit for robustness
  // Uses countdown logic to trigger 'shift' signal
  logic [12:0] baud_cnt;
  localparam baud_full = 13'd5208;  // 50_000_000 / 9600

  always_ff @(posedge clk) begin
    if (start | shift) begin
      if (start) baud_cnt <= baud_full >> 1;  // Half period for mid-bit sampling
      else baud_cnt <= baud_full;  // Full period for data/stop bits
    end else if (receiving) baud_cnt <= baud_cnt - 1;
  end

  assign shift = ~(|baud_cnt);  // Shift asserted when counter reaches zero

  //------------------------------------------------------------
  // RX Synchronizer
  //------------------------------------------------------------
  // Double-flop synchronizer for asynchronous RX input
  // Prevents metastability by aligning RX to clk domain
  logic single_flopped_RX, double_flopped_RX;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      single_flopped_RX <= 1;
      double_flopped_RX <= 1;
    end else begin
      single_flopped_RX <= RX;
      double_flopped_RX <= single_flopped_RX;
    end
  end

  //------------------------------------------------------------
  // Shift Register
  //------------------------------------------------------------
  // Serially shifts in 9 bits: start + 8 data
  // Stop bit is checked separately in state machine
  // rx_data extracts the final 8-bit payload
  logic [8:0] rx_shift_reg;
  always_ff @(posedge clk) begin
    if (shift) rx_shift_reg <= {double_flopped_RX, rx_shift_reg[8:1]};
  end

  assign rx_data = rx_shift_reg[7:0];

  //------------------------------------------------------------
  // Ready Flag Logic
  //------------------------------------------------------------
  // 'rdy' indicates valid byte reception
  // Cleared on new frame start or external clear
  // Set when stop bit is valid and full frame received
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) rdy <= 0;
    else if (start | clr_rdy) rdy <= 0;
    else if (set_rdy) rdy <= 1;
  end

  //------------------------------------------------------------
  // UART State Machine
  //------------------------------------------------------------
  // Two-state FSM: IDLE and RECEIVE
  // IDLE: Waits for falling edge (start bit)
  // RECEIVE: Samples bits and asserts 'rdy' when complete
  typedef enum logic [0:0] {
    IDLE,
    RECEIVE
  } rx_states;

  rx_states curr_state, nxt_state;

  // State register
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) curr_state <= IDLE;
    else curr_state <= nxt_state;
  end

  // Next-state and control logic
  always_comb begin
    // Default assignments
    nxt_state = curr_state;
    start     = 0;
    set_rdy   = 0;
    receiving = 0;

    unique case (curr_state)
      // Wait for start bit (falling edge) to initiate reception
      IDLE:
      if (~double_flopped_RX) begin
        start     = 1;  // Begin receiving new byte
        nxt_state = RECEIVE;
      end

      // Shift in data bits, assert ready flag when byte complete
      RECEIVE:
      if (bit_cnt != 4'b1010) receiving = 1;  // Continue receiving until all 10 bits captured
      else begin
        set_rdy   = 1;  // Stop bit valid → byte complete
        nxt_state = IDLE;  // Return to idle
      end

    endcase
  end

endmodule
