`timescale 1ns/1ps
//==============================================================
// Module: UART_tx
// Description: 
//   UART transmitter for sending 8-bit data serially. 
//   Designed to provide a single-cycle completion pulse (tx_done) and 
//   handle start/data/stop bits with a fixed baud rate.
//==============================================================
module UART_tx (
    input clk,
    input rst_n,
    input trmt,
    input [7:0] tx_data,
    output TX,
    output logic tx_done
);

  //------------------------------------------------------------
  // Internal Signals
  //------------------------------------------------------------
  // Signals exist to manage loading new data, ongoing transmission,
  // shifting bits at correct timing, and signaling completion.
  logic load;
  logic transmitting;
  logic shift;
  logic set_done;

  //------------------------------------------------------------
  // Bit Counter
  //------------------------------------------------------------
  // Tracks the progress through the UART frame. 
  // This is needed to know when transmission is complete 
  // and when to apply stop bits.
  logic [3:0] bit_cnt;
  always_ff @(posedge clk) begin
    if (load) bit_cnt <= 4'b000;  // Reset for new transmission
    else if (shift) bit_cnt <= bit_cnt + 1;  // Advance frame bit
  end

  //------------------------------------------------------------
  // Baud Rate Generator
  //------------------------------------------------------------
  // Ensures each UART bit is transmitted for the correct duration.
  // The counter structure isolates timing logic from shifting logic.
  localparam time_to_shift = 13'd5208;
  logic [12:0] baud_cnt;

  always_ff @(posedge clk) begin
    if (load | shift) baud_cnt <= 0;  // Restart counting for new bit
    else if (transmitting) baud_cnt <= baud_cnt + 1;
  end
  assign shift = (baud_cnt == time_to_shift);  // Indicates it’s time to shift next bit

  //------------------------------------------------------------
  // Transmit Shift Register
  //------------------------------------------------------------
  // Holds the start bit, data, and stop bit.
  // Using a shift register ensures bits are transmitted in order 
  // without additional control logic for each bit.
  logic [8:0] tx_shft_reg;

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) tx_shft_reg <= 9'hFFF;  // Idle high line
    else begin
      if (load) tx_shft_reg <= {tx_data, 1'b0};  // Insert start bit
      else if (shift) tx_shft_reg <= {1'b1, tx_shft_reg[8:1]};  // Shift out next bit, append stop
    end
  end
  assign TX = tx_shft_reg[0];

  //------------------------------------------------------------
  // Transmission Completion Flag
  //------------------------------------------------------------
  // Signals external modules that the byte has been sent.
  // Being one-cycle ensures synchronous detection without holding the signal unnecessarily.
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) tx_done <= 0;
    else if (set_done) tx_done <= 1;
    else if (load) tx_done <= 0;
  end

  //------------------------------------------------------------
  // State Machine
  //------------------------------------------------------------
  // Two states (IDLE/TRANSMIT) manage initiation and completion.
  // Using a minimal state machine isolates control from timing/shifting logic
  // and simplifies extension to larger UART features if needed.
  typedef enum logic [0:0] {
    IDLE,
    TRANSMIT
  } tx_states;

  tx_states curr_state, nxt_state;

  // State register
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) curr_state <= IDLE;
    else curr_state <= nxt_state;
  end

  // Control and next-state logic
  always_comb begin
    nxt_state    = curr_state;
    set_done     = 0;
    load         = 0;
    transmitting = 0;

    unique case (curr_state)
      IDLE: begin
        if (trmt) begin
          load      = 1;  // Start a new transmission
          nxt_state = TRANSMIT;
        end
      end
      TRANSMIT: begin
        if (bit_cnt == 4'b1010) begin  // Transmission complete
          set_done  = 1;  // Notify completion
          nxt_state = IDLE;  // Return to waiting
        end else begin
          transmitting = 1;  // Continue shifting bits
        end
      end
    endcase
  end

endmodule
