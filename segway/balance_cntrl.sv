module balance_cntrl (
    input logic signed [15:0] ptch,  // Signed 16-bit pitch signal from inertial interface
    input logic signed [15:0] ptch_rt,  // Signed 16-bit pitch rate from inertial interface
    input logic clk,  // System clock
    input logic rst_n,  // Active-low reset
    input logic vld,  // Valid signal for updating integral term
    input logic pwr_up,  // Power-up enable signal for soft start
    input logic rider_off,  // Asserted when no rider detected. Zeros out integrator.
    input logic [11:0] steer_pot,  // Steering potentiometer input (unsigned)
    input logic en_steer,  // Steering enable flag
    output logic signed [11:0] lft_spd,  // Computed signed wheel speed for left motor
    output logic signed [11:0] rght_spd,  // Computed signed wheel speed for right motor
    output logic too_fast  // Overspeed flag
);

  parameter fast_sim = 1;

  logic [11:0] PID_cntrl;
  logic [ 7:0] ss_tmr;

  // ===========================================================
  //  PIPELINE STAGE #1 — Register PID output
  // ===========================================================
  logic signed [11:0] PID_cntrl_p1;

  always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n)
          PID_cntrl_p1 <= '0;
      else
          PID_cntrl_p1 <= PID_cntrl;
  end

  PID #(.fast_sim(fast_sim)) iPID (
      .ptch(ptch),
      .ptch_rt(ptch_rt),
      .clk(clk),
      .rst_n(rst_n),
      .vld(vld),
      .pwr_up(pwr_up),
      .rider_off(rider_off),
      .ss_tmr(ss_tmr),
      .PID_cntrl(PID_cntrl)
  );

  logic signed [11:0] lft_spd_comb, rght_spd_comb;
  logic               too_fast_comb;

  SegwayMath iSegwayMath (
      .PID_cntrl(PID_cntrl_p1),
      .ss_tmr(ss_tmr),
      .steer_pot(steer_pot),
      .en_steer(en_steer),
      .pwr_up(pwr_up),
      .lft_spd(lft_spd_comb),     
      .rght_spd(rght_spd_comb),
      .too_fast(too_fast_comb)
  );

  // ===========================================================
  //  PIPELINE STAGE #2 — Register SegwayMath outputs
  // ===========================================================
  logic signed [11:0] lft_spd_p2, rght_spd_p2;
  logic               too_fast_p2;

  always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
          lft_spd_p2  <= '0;
          rght_spd_p2 <= '0;
          too_fast_p2 <= 1'b0;
      end else begin
          lft_spd_p2  <= lft_spd_comb;
          rght_spd_p2 <= rght_spd_comb;
          too_fast_p2 <= too_fast_comb;
      end
  end

  assign lft_spd  = lft_spd_p2;
  assign rght_spd = rght_spd_p2;
  assign too_fast = too_fast_p2;

endmodule