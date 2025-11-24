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

  PID #(
      .fast_sim(fast_sim)
  ) iPID (
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

  SegwayMath iSegwayMath (
      .PID_cntrl(PID_cntrl),
      .ss_tmr(ss_tmr),
      .steer_pot(steer_pot),
      .en_steer(en_steer),
      .pwr_up(pwr_up),
      .lft_spd(lft_spd),
      .rght_spd(rght_spd),
      .too_fast(too_fast)
  );

endmodule
