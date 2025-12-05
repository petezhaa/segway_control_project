module steer_en #(parameter fast_sim = 1'b1) (
  input  logic         clk,         // 50 MHz clock
  input  logic         rst_n,       // active-low async reset
  input  logic [11:0]  lft_ld,   // left load cell reading
  input  logic [11:0]  rght_ld,  // right load cell reading

  output logic en_steer,
  output logic rider_off
);

  logic sum_gt_min, sum_lt_min, diff_gt_15_16, diff_gt_1_4, clr;

    localparam MIN_RIDER_WT = 12'h200;
    localparam WT_HYSTERESIS = 8'h40;

    logic [12:0] MIN_WT_HYS_SUM;
    assign MIN_WT_HYS_SUM = MIN_RIDER_WT + WT_HYSTERESIS;

    logic [12:0] MIN_WT_HYS_DIFF;
    assign MIN_WT_HYS_DIFF = MIN_RIDER_WT - WT_HYSTERESIS;

    logic [12:0] rider_weight;
    assign rider_weight = lft_ld + rght_ld;

    assign sum_lt_min = (rider_weight < MIN_WT_HYS_DIFF);

    assign sum_gt_min = (rider_weight > MIN_WT_HYS_SUM);

    logic signed [12:0] lft_rght_diff;
    assign lft_rght_diff =  $signed({1'b0, rght_ld}) - $signed({1'b0, lft_ld});

    logic [12:0] abs_lft_rght_diff;
    assign abs_lft_rght_diff = (lft_rght_diff[12]) ? -lft_rght_diff : lft_rght_diff;
    logic [12:0] rider_weight_scaled_1_4;
    assign rider_weight_scaled_1_4 = rider_weight/4;

    logic [12:0] rider_weight_scaled_15_16;
    assign rider_weight_scaled_15_16 = rider_weight - (rider_weight/16);

    assign diff_gt_1_4 = (abs_lft_rght_diff > rider_weight_scaled_1_4) ? 1'b1 : 1'b0;

    assign diff_gt_15_16 = (abs_lft_rght_diff > rider_weight_scaled_15_16) ? 1'b1 : 1'b0;

  // Timer counter and tmr_full
  localparam integer FULL_CYCLES = 67000000; // ~1.34 s @ 50 MHz
  logic [25:0] timer;
  logic full;

  // ------------------------------------------------------------------
  // Timer: 50 MHz clock -> count FULL_CYCLES cycles for ~1.34 s
  // - counter cleared by `clr` coming from the state-machine
  // - when fast_sim==1, only look at bits [14:0] to produce faster `tmr_full` events
  // ------------------------------------------------------------------
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n)
      timer <= '0;
    else if (clr)
      timer <= '0;
    else
      timer <= timer + 1;
  end

  // tmr_full generation
  generate
    if (fast_sim)
    // Fast simulation: assert when low 15 bits roll to all ones (periodic fast event)
    assign full = &timer[14:0];
   else begin
    // Real timing: compare full counter to desired target
    assign full = &timer;
  end
  endgenerate

  // ------------------------------------------------------------------
  // Instantiate the steer_en_SM state machine and connect signals
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
