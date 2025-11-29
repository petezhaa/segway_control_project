module steer_en_SM(clk,rst_n,tmr_full,sum_gt_min,sum_lt_min,diff_gt_1_4,
                   diff_gt_15_16,clr_tmr,en_steer,rider_off);

  input logic clk;				// 50MHz clock
  input logic rst_n;				// Active low asynch reset
  input logic tmr_full;			// asserted when timer reaches 1.3 sec
  input logic sum_gt_min;			// asserted when left and right load cells together exceed min rider weight
  input logic sum_lt_min;			// asserted when left_and right load cells are less than min_rider_weight
  /////////////////////////////////////////////////////////////////////////////
  // HEY HOFFMAN...you are a moron.  sum_gt_min would simply be ~sum_lt_min.
  // Why have both signals coming to this unit??  ANSWER: What if we had a rider
  // (a child) who's weight was right at the threshold of MIN_RIDER_WEIGHT?
  // We would enable steering and then disable steering then enable it again,
  // ...  We would make that child crash(children are light and flexible and
  // resilient so we don't care about them, but it might damage our Segway).
  // We can solve this issue by adding hysteresis.  So sum_gt_min is asserted
  // when the sum of the load cells exceeds MIN_RIDER_WEIGHT + HYSTERESIS and
  // sum_lt_min is asserted when the sum of the load cells is less than
  // MIN_RIDER_WEIGHT - HYSTERESIS.  Now we have noise rejection for a rider
  // who's weight is right at the threshold.  This hysteresis trick is as old
  // as the hills, but very handy...remember it.
  /////////////////////////////////////////////////////////////////////////////

  input logic diff_gt_1_4;		// asserted if load cell difference exceeds 1/4 sum (rider not situated)
  input logic diff_gt_15_16;		// asserted if load cell difference is great (rider stepping off)
  output logic clr_tmr;		// clears the 1.3sec timer
  output logic en_steer;	// enables steering (goes to balance_cntrl)
  output logic rider_off;	// held high in intitial state when waiting for sum_gt_min

  // You fill out the rest...use good SM coding practices ///

  // State encoding
  typedef enum logic [1:0] {INIT, BALANCE_TIMER,STEER_EN} state_t;

  // State register
  state_t state, next_state;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      state <= INIT; // reset to INIT state
    else
      state <= next_state; // move to next state
  end

  always_comb begin

    // Default outputs and next state
    next_state = state; // default to state to optimize SM
    clr_tmr = 1'b0;
    en_steer = 1'b0;
    rider_off = 1'b0;

    case(state)
      INIT: begin
        rider_off = 1'b1; // rider off board for now
        if (sum_gt_min) begin
          next_state = BALANCE_TIMER;
          clr_tmr = 1'b1; // clear timer when entering BALANCE_TIMER state
          // implicit else remains in INIT state and outputs remains 0
        end
      end

      BALANCE_TIMER: begin
        if (sum_lt_min)
          next_state = INIT;
        else if (tmr_full)
          next_state = STEER_EN;
        else if (diff_gt_1_4)
          // implicit remain in state
          clr_tmr = 1'b1; // keep clearing timer if rider not balanced
        // implicit else remain in state
      end

      STEER_EN: begin
        en_steer = 1'b1;
        if (sum_lt_min) // priority to disable steering if rider falls off
          next_state = INIT;
        else if (diff_gt_15_16) begin
          next_state = BALANCE_TIMER;
          clr_tmr = 1'b1; // rider stepping off, go back to BALANCE_TIMER state
        end
        // implicit else remain in state
      end

      default: next_state = INIT; // default to reset state
    endcase
  end

endmodule