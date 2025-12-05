module piezo_drv (
    input  logic clk,       // System clock
    input  logic rst_n,     // Active low reset
    input  logic en_steer,  // Enable steering signal
    input  logic too_fast,  // Too fast signal
    input  logic batt_low,  // Battery low signal
    output logic piezo,     // Piezo driver output
    output logic piezo_n    // Inverted piezo driver output
);

  parameter fast_sim = 1;

  //----------------------------------------------------------------------
  // Note frequency periods (full-period counts at 50 MHz)
  //----------------------------------------------------------------------

  localparam logic [15:0] G6_PERIOD = 16'h7C90;
  localparam logic [15:0] C7_PERIOD = 16'h5D51;
  localparam logic [15:0] E7_PERIOD = 16'h4A11;
  localparam logic [15:0] G7_PERIOD = 16'h3E48;

  //----------------------------------------------------------------------
  // Durations (all use ONE shared 28-bit counter)
  //   - relative sizes kept the same as your original powers of two
  //   - 3-second delay between fanfares is also done with this counter
  //----------------------------------------------------------------------

  // 2^23, 2^22, 2^25 (as in your original design)
  localparam logic [27:0] DUR_2P23 = 28'h800_000;  // 2^23
  localparam logic [27:0] DUR_2P22 = 28'h400_000;  // 2^22
  localparam logic [27:0] DUR_2P25 = 28'h2_000_000;  // 2^25

  // ~3 seconds at 50 MHz = 150,000,000 cycles
  localparam logic [27:0] DUR_3S = 28'h470D180;  // leftover from 150,000,000 decimal

  //----------------------------------------------------------------------
  // Shared duration counter (for both notes and 3-second wait)
  //----------------------------------------------------------------------

  logic [27:0] dur_cnt;
  logic [27:0] dur_limit;
  logic        dur_done;
  logic        dur_restart;

  generate
    if (fast_sim) begin
      // fast simulation mode: count down in larger steps
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          dur_cnt <= 28'd0;
        end else if (dur_restart) begin
          dur_cnt <= dur_limit >> 6;
        end else if (!dur_done) begin
          dur_cnt <= dur_cnt - 28'd64;
        end
      end
      assign dur_done = (dur_cnt <= 28'd64);
    end else begin
      // real mode: count down normally
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          dur_cnt <= 28'd0;
        end else if (dur_restart) begin
          dur_cnt <= dur_limit;
        end else if (!dur_done) begin
          dur_cnt <= dur_cnt - 28'd1;
        end
      end
      assign dur_done = ~(|dur_cnt);
    end
  endgenerate

  //----------------------------------------------------------------------
  // Square wave generator for piezo (single 16-bit counter)
  //   - runs only when we are actually "playing a note"
  //----------------------------------------------------------------------

  logic [15:0] period_timer;
  logic [15:0] note_period;
  logic        no_note;  // high => silence
  logic       half_period_reached;
  logic       new_note;
  generate
    if (fast_sim) begin
      // fast simulation mode
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          period_timer <= 16'd0;
        end else if (no_note) begin
          // silence
          period_timer <= 16'h0FFF;
        end else if (half_period_reached | new_note) begin
          // toggle piezo output at the end of each full period
          period_timer <= note_period >> 7;
        end else begin
          // increment period timer
          period_timer <= period_timer - 8'h40;
        end
      end

      assign half_period_reached = ($signed(period_timer) < 0);
    end else begin
      // real mode
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          period_timer <= 16'd0;
        end else if (no_note) begin
          // silence
          period_timer <= 16'd0;
        end else if (half_period_reached | new_note) begin
          // toggle piezo output at the end of each full period
          period_timer <= note_period >>> 1;
        end else begin
          // increment period timer
          period_timer <= period_timer - 16'd1;
        end
      end
      assign half_period_reached = ~(|period_timer);
    end
  endgenerate

  logic piezo_ff;
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      piezo_ff <= 1'b0;
    end else if (half_period_reached) begin
      piezo_ff <= piezo_n;
    end
  end

  assign piezo   = piezo_ff;
  assign piezo_n = ~piezo_ff;

  //----------------------------------------------------------------------
  // Fanfaré state machine
  //   - Added WAIT_3S state to replace separate three_sec_timer
  //   - Same musical sequence as your original code
  //----------------------------------------------------------------------

  typedef enum logic [2:0] {
    IDLE,
    WAIT_3S,   // new: replaces separate three_sec_timer
    G6,
    C7,
    E7_LONG,
    G7_SHORT,
    E7_SHORT,
    G7_LONG
  } piezo_state_t;

  piezo_state_t curr_state, nxt_state;

  // state flop
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      curr_state <= IDLE;
    end else begin
      curr_state <= nxt_state;
    end
  end

  //----------------------------------------------------------------------
  // Next-state and control logic
  //----------------------------------------------------------------------

  always_comb begin
    // defaults
    nxt_state   = curr_state;
    no_note     = 1'b1;  // default: silence
    note_period = 16'hFFFF;
    dur_limit   = DUR_2P23;
    dur_restart = 1'b0;
    new_note    = 1'b0;

    unique case (curr_state)

      //----------------------------------------------------------------
      // IDLE: no sound
      //   - too_fast  => immediate start (G6)
      //   - batt_low / en_steer => start 3-second wait before fanfare
      //----------------------------------------------------------------
      IDLE: begin
        no_note = 1'b1;

        if (too_fast) begin
          // urgent: start G6 immediately
          nxt_state   = G6;
          dur_limit   = DUR_2P23;
          new_note    = 1'b1;
          dur_restart = 1'b1;
        end else if (batt_low) begin
          // start the 3-second delay before the periodic fanfare
          nxt_state   = G7_LONG;
          dur_limit   = DUR_2P25;
          note_period = G7_PERIOD;
          no_note     = 1'b0;
          new_note    = 1'b1;
          dur_restart = 1'b1;
        end else if (en_steer) begin
          // start the 3-second delay before the periodic fanfare
          nxt_state   = G6;
          dur_limit   = DUR_2P23;
          new_note    = 1'b1;
          dur_restart = 1'b1;
        end
      end

      //----------------------------------------------------------------
      // WAIT_3S: silent 3-second wait between fanfares
      //   - too_fast  => override and jump to G6 immediately
      //   - after 3s, batt_low => G7_LONG fanfare
      //   - after 3s, en_steer => normal fanfare starting at G6
      //----------------------------------------------------------------
      WAIT_3S: begin
        no_note   = 1'b1;
        dur_limit = DUR_3S;

        if (too_fast) begin
          nxt_state   = G6;
          dur_limit   = DUR_2P23;
          dur_restart = 1'b1;
        end else if (dur_done) begin
          if (batt_low) begin
            nxt_state   = G7_LONG;
            dur_limit   = DUR_2P25;
            note_period = G7_PERIOD;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end else if (en_steer) begin
            nxt_state   = G6;
            dur_limit   = DUR_2P23;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end else begin
            // inputs went away; go idle
            nxt_state = IDLE;
          end
        end
      end

      //----------------------------------------------------------------
      // G6 note
      //----------------------------------------------------------------
      G6: begin
        no_note     = 1'b0;
        note_period = G6_PERIOD;
        dur_limit   = DUR_2P23;

        if (dur_done) begin
          if (too_fast) begin
            nxt_state   = C7;
            dur_limit   = DUR_2P23;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end else if (batt_low) begin
            // go silent when battery low after G6 (like your IDLE path)
            dur_limit   = DUR_3S;
            dur_restart = 1'b1;
            new_note    = 1'b1;
            nxt_state = WAIT_3S;
          end else begin
            // normal / en_steer case
            nxt_state   = C7;
            dur_limit   = DUR_2P23;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end
        end
      end

      //----------------------------------------------------------------
      // C7 note
      //----------------------------------------------------------------
      C7: begin
        no_note     = 1'b0;
        note_period = C7_PERIOD;
        dur_limit   = DUR_2P23;

        if (dur_done) begin
          if (too_fast) begin
            nxt_state   = E7_LONG;
            dur_limit   = DUR_2P23;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end else if (batt_low) begin
            nxt_state   = G6;
            dur_limit   = DUR_2P23;
            note_period = G6_PERIOD;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end else begin
            nxt_state   = E7_LONG;
            dur_limit   = DUR_2P23;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end
        end
      end

      //----------------------------------------------------------------
      // E7_LONG note
      //----------------------------------------------------------------
      E7_LONG: begin
        no_note     = 1'b0;
        note_period = E7_PERIOD;
        dur_limit   = DUR_2P23;

        if (dur_done) begin
          if (too_fast) begin
            nxt_state   = G6;
            dur_limit   = DUR_2P23;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end else if (batt_low) begin
            nxt_state   = C7;
            dur_limit   = DUR_2P23;
            note_period = C7_PERIOD;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end else if (en_steer) begin
            nxt_state   = G7_SHORT;
            dur_limit   = DUR_2P23 + DUR_2P22;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end else begin
            nxt_state = IDLE;
          end
        end
      end

      //----------------------------------------------------------------
      // G7_SHORT note
      //----------------------------------------------------------------
      G7_SHORT: begin
        no_note     = 1'b0;
        note_period = G7_PERIOD;
        dur_limit   = DUR_2P23 + DUR_2P22;

        if (dur_done) begin
          if (batt_low) begin
            nxt_state   = E7_LONG;
            dur_limit   = DUR_2P23;
            note_period = E7_PERIOD;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end else begin
            nxt_state   = E7_SHORT;
            dur_limit   = DUR_2P22;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end
        end
      end

      //----------------------------------------------------------------
      // E7_SHORT note
      //----------------------------------------------------------------
      E7_SHORT: begin
        no_note     = 1'b0;
        note_period = E7_PERIOD;
        dur_limit   = DUR_2P22;

        if (dur_done) begin
          if (batt_low) begin
            nxt_state   = G7_SHORT;
            dur_limit   = DUR_2P23 + DUR_2P22;
            note_period = G7_PERIOD;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end else begin
            nxt_state   = G7_LONG;
            dur_limit   = DUR_2P25;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end
        end
      end

      //----------------------------------------------------------------
      // G7_LONG note
      //----------------------------------------------------------------
      G7_LONG: begin
        no_note     = 1'b0;
        note_period = G7_PERIOD;
        dur_limit   = DUR_2P25;

        if (dur_done) begin
          if (batt_low) begin
            // battery low: end with E7_SHORT
            nxt_state   = E7_SHORT;
            dur_limit   = DUR_2P22;
            note_period = E7_PERIOD;
            new_note    = 1'b1;
            dur_restart = 1'b1;
          end else begin
            // steering OK: go idle, next fanfare after another 3s
            dur_limit   = DUR_3S;
            dur_restart = 1'b1;
            new_note    = 1'b1;
            nxt_state = WAIT_3S;
          end
        end
      end

    endcase
  end

endmodule