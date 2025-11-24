//======================================================================
// Module: Auth_blk
// Description:
//   Implements an authentication block for a "Segway"-style system.
//   The module listens on a UART RX line for specific authorization
//   bytes ('G' = Go, 'S' = Stop). It asserts 'pwr_up' when the rider
//   is authorized and deasserts it when the rider steps off.
//
//   Functionality summary:
//   - Starts in OFF state after reset.
//   - Transitions to ON when it receives 0x47 ('G').
//   - Remains ON when 'G' or 'S' are received while the rider is on.
//   - Returns to OFF when 'rider_off' is asserted.
//
// Dependencies:
//   - UART_rx submodule (handles byte reception and ready signaling)
//======================================================================

module Auth_blk (
    input  logic RX,         // UART receive line input from BLE module
    input  logic rider_off,  // High when rider leaves the platform
    input  logic clk,        // System clock
    input  logic rst_n,      // Active-low reset
    output logic pwr_up      // Power enable signal for main system
);

  // ------------------------------------------------------------
  // Internal signals for UART interface
  // ------------------------------------------------------------
  logic       clr_rx_rdy;  // Clear flag for received byte ready
  logic       rx_rdy;  // Indicates a new byte has been received
  logic [7:0] rx_data;  // Captured received data byte

  // ------------------------------------------------------------
  // UART Receiver Instantiation
  // ------------------------------------------------------------
  // Receives bytes from BLE UART stream and sets rx_rdy high when
  // a byte is fully received. The ready flag is cleared by clr_rx_rdy.
  UART_rx iRX (
      .RX     (RX),
      .clk    (clk),
      .rst_n  (rst_n),
      .clr_rdy(clr_rx_rdy),
      .rx_data(rx_data),
      .rdy    (rx_rdy)
  );

  // ------------------------------------------------------------
  // FSM Definition
  // ------------------------------------------------------------
  // Two-state FSM:
  //   OFF : Waiting for valid 'G' authorization code
  //   ON  : Power enabled; maintains power until rider_off
  typedef enum logic [1:0] {
    IDLE,   // 0
    CONNECTED,
    DISCONNECTED     // 1
  } auth_blk_states;

  auth_blk_states curr_state, nxt_state;

  // ------------------------------------------------------------
  // Constant UART command codes
  // ------------------------------------------------------------
  localparam G = 8'h47;  // 'G' = Go / Grant authorization
  localparam S = 8'h53;  // 'S' = Stop / Standby request

  // ------------------------------------------------------------
  // Sequential logic: State register
  // ------------------------------------------------------------
  // On reset, FSM starts in OFF state.
  // Otherwise, updates state at each rising clock edge.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) curr_state <= IDLE;
    else curr_state <= nxt_state;
  end

  // ------------------------------------------------------------
  // Combinational logic: Next state and output control
  // ------------------------------------------------------------
  // Determines next state and output (pwr_up) behavior
  // based on current state, UART input, and rider_off.
  always_comb begin
    // Default assignments
    nxt_state = curr_state;  // Stay in current state unless a transition occurs
    pwr_up    = 0;  // Default power-down (avoids inferred latch)
    clr_rx_rdy = 0;  // Default: do not clear rx_rdy

    case (curr_state)

      //===========================================================
      // OFF State: Wait for Authorization
      //===========================================================
      IDLE: begin
        // If UART receives 'G' (Go), authorize and power up
        if (rx_rdy && (rx_data == G)) begin
          pwr_up    = 1;
          clr_rx_rdy = 1;
          nxt_state = CONNECTED;
        end
      end

      //===========================================================
      // ON State: Maintain Power Until Rider Steps Off
      //===========================================================
      CONNECTED: begin
        if (rx_rdy && (rx_data == S)) begin
          if (rider_off) begin
            clr_rx_rdy = 1;
            nxt_state  = IDLE;
          end else begin
            pwr_up = 1;
            clr_rx_rdy = 1;
            nxt_state = DISCONNECTED;
          end
        end else pwr_up = 1;
      end

      DISCONNECTED: begin
        if(rider_off) begin
          nxt_state = IDLE;
        end else if (rx_rdy && (rx_data == G)) begin
          pwr_up = 1;
          clr_rx_rdy = 1;
          nxt_state = CONNECTED;
        end else pwr_up = 1;
      end

    endcase
  end

endmodule
