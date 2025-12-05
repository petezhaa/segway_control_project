module steer_en #(parameter fast_sim = 1'b1) (
  input  logic         clk,         // 50 MHz system clock
  input  logic         rst_n,       // active-low reset
  input  logic [11:0]  lft_ld,      // left load-cell reading
  input  logic [11:0]  rght_ld,     // right load-cell reading

  output logic en_steer,           // steering enable output
  output logic rider_off           // asserted when no rider present
);

  // Intermediate comparison signals used by the state machine
  logic sum_gt_min, sum_lt_min, diff_gt_15_16, diff_gt_1_4, clr;

  // Rider weight minimum threshold and hysteresis band
  localparam MIN_RIDER_WT  = 12'h200; // nominal minimum rider weight
  localparam WT_HYSTERESIS = 8'h40;   // hysteresis margin around threshold

  // Precomputed upper hysteresis boundary
  logic [12:0] MIN_WT_HYS_SUM;
  assign MIN_WT_HYS_SUM = MIN_RIDER_WT + WT_HYSTERESIS;

  // Precomputed lower hysteresis boundary
  logic [12:0] MIN_WT_HYS_DIFF;
  assign MIN_WT_HYS_DIFF = MIN_RIDER_WT - WT_HYSTERESIS;

  // Total measured rider weight
  logic [12:0] rider_weight;
  assign rider_weight = lft_ld + rght_ld;

  // Check below lower hysteresis threshold
  assign sum_lt_min = (rider_weight < MIN_WT_HYS_DIFF);

  // Check above upper hysteresis threshold
  assign sum_gt_min = (rider_weight > MIN_WT_HYS_SUM);

  // Signed difference between right and left load cells
  logic signed [12:0] lft_rght_diff;
  assign lft_rght_diff = $signed({1'b0, rght_ld}) - $signed({1'b0, lft_ld});

  // Absolute value of load-cell difference
  logic [12:0] abs_lft_rght_diff;
  assign abs_lft_rght_diff = (lft_rght_diff[12]) ? -lft_rght_diff : lft_rght_diff;

  // 1/4 scaled rider weight for coarse imbalance threshold
  logic [12:0] rider_weight_scaled_1_4;
  assign rider_weight_scaled_1_4 = rider_weight / 4;

  // 15/16 scaled rider weight for extreme imbalance threshold
  logic [12:0] rider_weight_scaled_15_16;
  assign rider_weight_scaled_15_16 = rider_weight - (rider_weight / 16);

  // Compare diff > 1/4 total rider weight
  assign diff_gt_1_4 = (abs_lft_rght_diff > rider_weight_scaled_1_4);

  // Compare diff > 15/16 total rider weight
  assign diff_gt_15_16 = (abs_lft_rght_diff > rider_weight_scaled_15_16);

  // ------------------------------------------------------------------
  // Timer for debouncing / settling time inside steer_en_SM
  // FULL_CYCLES ≈ 1.34s at 50 MHz
  // In fast_sim mode, timer asserts full more frequently for quick sims
  // ------------------------------------------------------------------

  //localparam integer FULL_CYCLES = 67000000; // ~1.34 s worth of cycles
  logic [25:0] timer;
  logic full;

  // Free-running or cleared counter depending on clr
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n)
      timer <= '0;     // reset counter
    else if (clr)
      timer <= '0;     // state machine requests reset
    else
      timer <= timer + 1; // increment counter
  end

  // Generate tmr_full based on simulation mode
  generate
    if (fast_sim)
      // In fast-sim: assert when low 15 bits == all ones (fast rollover)
      assign full = &timer[14:0];
    else begin
      // In real hardware: check full width for rollover timing
      assign full = &timer;
    end
  endgenerate

  // ------------------------------------------------------------------
  // Steering enable state machine
  // Processes:
  //  - weight thresholds (sum_lt_min / sum_gt_min)
  //  - imbalance thresholds (diff_gt_1_4 / diff_gt_15_16)
  //  - timer expiration
  // Outputs:
  //  - en_steer : steering allowed
  //  - rider_off : rider not detected
  // ------------------------------------------------------------------

  steer_en_SM u_steer_en_SM (
    .clk(clk),
    .rst_n(rst_n),
    .tmr_full(full),
    .sum_gt_min(sum_gt_min),
    .sum_lt_min(sum_lt_min),
    .diff_gt_1_4(diff_gt_1_4),
    .diff_gt_15_16(diff_gt_15_16),
    .clr_tmr(clr),
    .en_steer(en_steer),
    .rider_off(rider_off)
  );

endmodule
