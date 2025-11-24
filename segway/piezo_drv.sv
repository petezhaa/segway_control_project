// generate me a shell for piezo_drv.sv including the I/O ports and module declaration. The interface is as follows:
// clk,rst_n in 50MHz clk
// en_steer in “normal” operation
// too_fast in priority over other inputs
// batt_low in
// Charge backwards
// piezo, piezo_n out Differential piezo drive

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

  localparam [15:0] G6_period = 16'h7C90;
  localparam [15:0] C7_period = 16'h5D51;
  localparam [15:0] E7_period = 16'h4A11;
  localparam [15:0] G7_period = 16'h3E48;

  localparam [23:0] TWO_POW_23_DURATION = 24'h800000;  // 2^23
  localparam [22:0] TWO_POW_22_DURATION = 24'h400000;  // 2^22
  localparam [25:0] TWO_POW_25_DURATION = 26'h2000000;  // 2^25

  logic new_note;
  logic no_note;
  logic [15:0] note_period;

  // duration timer to keep track of how long each note to play
  logic [25:0] duration_timer;
  logic [25:0] note_duration;
  logic duration_over;
  generate
    if (fast_sim) begin

      always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
          duration_timer <= 26'd0;
        end else if (new_note | no_note) begin
          duration_timer <= 0;
        end else begin
          duration_timer <= duration_timer + 8'h40;
        end
      end

      assign duration_over = (duration_timer >= (note_duration >> 6));  // 1us for fast sim
    end else begin
      always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
          duration_timer <= 26'd0;
        end else if (new_note | no_note) begin
          duration_timer <= 0;
        end else begin
          duration_timer <= duration_timer + 1;
        end
      end

      assign duration_over = (duration_timer == note_duration);  // real duration
    end
  endgenerate

  // period/frequency timer to generate the piezo square wave
  logic signed [15:0] period_timer;
  logic half_period_reached;
  generate
    if (fast_sim) begin

      always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
          period_timer <= 0;
        end else if (no_note) begin
          period_timer <= 16'h0FFF;
        end else if (new_note | half_period_reached) begin
          period_timer <= $signed(note_period >> 7);
        end else begin
          period_timer <= period_timer - 8'h40;
        end
      end

      assign half_period_reached = (period_timer < 0);  // 1us for fast sim
    end else begin

      always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
          period_timer <= 0;
        end else if (no_note) begin
          period_timer <= '1;
        end else if (new_note | half_period_reached) begin
          period_timer <= $signed(note_period >> 1);
        end else begin
          period_timer <= period_timer - 1'b1;
        end
      end

      assign half_period_reached = (period_timer == 0);  // real period
    end
  endgenerate

  // piezo output flip-flop
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

  // repeat timer to ensure fanfare played once every 3 seconds
  logic signed [27:0] three_sec_timer;
  logic three_sec_up, clr_three_sec_timer;
  localparam THREE_SECONDS_at_50MHZ = 28'h8F0D180;

  generate
    if (fast_sim) begin

      always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
          three_sec_timer <= 29'd0;
        end else if (clr_three_sec_timer) begin
          three_sec_timer <= (THREE_SECONDS_at_50MHZ >> 6);
        end else if (!three_sec_up) begin
          three_sec_timer <= three_sec_timer - 8'h40;
        end
      end

      assign three_sec_up = (three_sec_timer <= 0);  // 3 seconds at 50MHz


    end else begin

      always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
          three_sec_timer <= 29'd0;
        end else if (clr_three_sec_timer) begin
          three_sec_timer <= THREE_SECONDS_at_50MHZ;
        end else if (!three_sec_up) begin
          three_sec_timer <= three_sec_timer - 1'b1;
        end
      end

      assign three_sec_up = (three_sec_timer == 0);  // 3 seconds at 50MHz

    end
  endgenerate

  // the state machine to control the piezo driver
  typedef enum logic [2:0] {
    IDLE,
    G6,
    C7,
    E7_LONG,
    G7_SHORT,
    E7_SHORT,
    G7_LONG
  } piezo_state_t;

  piezo_state_t curr_state, nxt_state;

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      curr_state <= IDLE;
    end else begin
      curr_state <= nxt_state;
    end
  end

  // always_comb begin
  //   nxt_state = curr_state;
  //   new_note = 1'b0;
  //   clr_three_sec_timer = 1'b0;
  //   no_note = 1'b0;
  //   note_period = 16'hFFFF;
  //   note_duration = '1;


  //   case (curr_state)

  //     IDLE: begin
  //       no_note = 1'b1;
  //       if (too_fast) begin
  //         new_note = 1'b1;
  //         nxt_state = G6;
  //         note_duration = TWO_POW_23_DURATION;
  //       end else if (three_sec_up) begin
  //         if (batt_low) begin
  //           clr_three_sec_timer = 1'b1;
  //           //new_note = 1'b1;
  //           nxt_state = G7_LONG;
  //           note_duration = TWO_POW_25_DURATION;
  //         end else if (en_steer) begin
  //           clr_three_sec_timer = 1'b1;
  //           //new_note = 1'b1;
  //           nxt_state = G6;
  //           note_duration = TWO_POW_23_DURATION;
  //         end
  //       end
  //     end

    //   G6: begin
    //     note_period   = G6_period;
    //     note_duration = TWO_POW_23_DURATION;
    //     if (duration_over) begin
    //       if (too_fast) begin
    //         new_note  = 1'b1;
    //         nxt_state = C7;
    //       end else if (batt_low) begin
    //         new_note  = 1'b1;
    //         nxt_state = IDLE;
    //       end else if (en_steer) begin
    //         new_note  = 1'b1;
    //         nxt_state = C7;
    //       end
    //     end
    //   end

    //   C7: begin
    //     note_period   = C7_period;
    //     note_duration = TWO_POW_23_DURATION;
    //     if (duration_over) begin
    //       if (too_fast) begin
    //         new_note  = 1'b1;
    //         nxt_state = E7_LONG;
    //       end else if (batt_low) begin
    //         nxt_state = G6;
    //       end else if (en_steer) begin
    //         new_note  = 1'b1;
    //         nxt_state = E7_LONG;
    //       end
    //     end
    //   end

    //   E7_LONG: begin
    //     note_period   = E7_period;
    //     note_duration = TWO_POW_23_DURATION;
    //     if (duration_over) begin
    //       if (too_fast) begin
    //         new_note  = 1'b1;
    //         nxt_state = G6;
    //       end else if (batt_low) begin
    //         new_note  = 1'b1;
    //         nxt_state = C7;
    //       end else if (en_steer) begin
    //         new_note  = 1'b1;
    //         nxt_state = G7_SHORT;
    //       end else begin
    //         nxt_state = IDLE;
    //       end
    //     end
    //   end

    //   G7_SHORT: begin
    //     note_period   = G7_period;
    //     note_duration = TWO_POW_23_DURATION + TWO_POW_22_DURATION;
    //     if (too_fast) begin
    //       new_note  = 1'b1;
    //       nxt_state = G6;
    //     end else if (duration_over) begin
    //       if (batt_low) begin
    //         new_note  = 1'b1;
    //         nxt_state = E7_LONG;
    //       end else if (en_steer) begin
    //         new_note  = 1'b1;
    //         nxt_state = E7_SHORT;
    //       end
    //     end
    //   end

    //   E7_SHORT: begin
    //     note_period   = E7_period;
    //     note_duration = TWO_POW_22_DURATION;
    //     if (too_fast) begin
    //       new_note  = 1'b1;
    //       nxt_state = G6;
    //     end else if (duration_over) begin
    //       if (batt_low) begin
    //         new_note  = 1'b1;
    //         nxt_state = G7_SHORT;
    //       end else if (en_steer) begin
    //         new_note  = 1'b1;
    //         nxt_state = G7_LONG;
    //       end
    //     end
    //   end

    //   G7_LONG: begin
    //     note_period   = G7_period;
    //     note_duration = TWO_POW_25_DURATION;
    //     if (too_fast) begin
    //       new_note  = 1'b1;
    //       nxt_state = G6;
    //     end else if (duration_over) begin
    //       if (batt_low) begin
    //         new_note  = 1'b1;
    //         nxt_state = E7_SHORT;
    //       end else if (en_steer) begin
    //         nxt_state = IDLE;
    //       end
    //     end
    //   end

    // endcase

      always_comb begin
        nxt_state = curr_state;
        new_note  = 1'b0;
        clr_three_sec_timer = 1'b0;
        no_note   = 1'b0;
        note_period = 16'hFFFF;
        note_duration = '1;


        case (curr_state)

          IDLE: begin
            no_note = 1'b1;
            if (too_fast) begin
              new_note  = 1'b1;
              nxt_state = G6;
              note_duration = TWO_POW_23_DURATION;
            end else if (three_sec_up) begin
              if (batt_low) begin
                clr_three_sec_timer = 1'b1;
                //new_note = 1'b1;
                nxt_state = G7_LONG;
                note_duration = TWO_POW_25_DURATION;
              end else if (en_steer) begin
                clr_three_sec_timer = 1'b1;
                //new_note = 1'b1;
                nxt_state = G6;
                note_duration = TWO_POW_23_DURATION;
              end
            end
          end

          G6: begin
            note_period   = G6_period;
            note_duration = TWO_POW_23_DURATION;
            if (duration_over) begin
              if (too_fast) begin
                new_note  = 1'b1;
                nxt_state = C7;
              end else if (batt_low) begin
                no_note  = 1'b1;
                nxt_state = IDLE;
              end else begin
                new_note  = 1'b1;
                nxt_state = C7;
              end
            end
          end

          C7: begin
            note_period   = C7_period;
            note_duration = TWO_POW_23_DURATION;
            if (duration_over) begin
              if (too_fast) begin
                new_note  = 1'b1;
                nxt_state = E7_LONG;
              end else if (batt_low) begin
                new_note  = 1'b1;
                nxt_state = G6;
              end else begin
                new_note  = 1'b1;
                nxt_state = E7_LONG;
              end
            end
          end

          E7_LONG: begin
            note_period   = E7_period;
            note_duration = TWO_POW_23_DURATION;
            if (duration_over) begin
              if (too_fast) begin
                new_note  = 1'b1;
                nxt_state = G6;
              end else if (batt_low) begin
                new_note  = 1'b1;
                nxt_state = C7;
              end else if (en_steer) begin
                new_note  = 1'b1;
                nxt_state = G7_SHORT;
              end else begin
                nxt_state = IDLE;
              end
            end
          end

          G7_SHORT: begin
            note_period   = G7_period;
            note_duration = TWO_POW_23_DURATION + TWO_POW_22_DURATION;
            if (duration_over) begin
              if (batt_low) begin
                new_note  = 1'b1;
                nxt_state = E7_LONG;
              end else begin
                new_note  = 1'b1;
                nxt_state = E7_SHORT;
              end
            end
          end

          E7_SHORT: begin
            note_period   = E7_period;
            note_duration = TWO_POW_22_DURATION;
            if (duration_over) begin
              if (batt_low) begin
                new_note  = 1'b1;
                nxt_state = G7_SHORT;
              end else begin
                new_note  = 1'b1;
                nxt_state = G7_LONG;
              end
            end
          end

          G7_LONG: begin
            note_period   = G7_period;
            note_duration = TWO_POW_25_DURATION;
            if (duration_over) begin
              if(batt_low) begin
                new_note  = 1'b1;
                nxt_state = E7_SHORT;
              end else begin
                nxt_state = IDLE;
              end
            end
          end

        endcase

  end

endmodule
