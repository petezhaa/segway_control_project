
module SegwayMath (
    input logic signed [11:0] PID_cntrl,  // Signed PID output from balance controller
    input logic [7:0] ss_tmr,  //Soft-start timer for ramping the PID control
    input logic [11:0] steer_pot,  // Steering potentiometer input (unsigned)
    input logic en_steer,  // Steering enable flag
    input logic pwr_up,  // Power-up flag (0=off, 1=on)
    input logic clk,
    input logic rst_n,
    output logic signed [11:0] lft_spd,  // Computed signed wheel speeds for left motor
    output logic signed [11:0] rght_spd,  // Computed signed wheel speeds for right motor
    output logic too_fast  // Overspeed flag
);

  // internal signals
  logic signed [19:0] PID_cntrl_mult_ss_tmr;  // intermediate signal for PID_cntrl * ss_tmr
  logic signed [11:0] PID_ss;  // PID scaled by soft-start timer

  // steering related signals
  logic [11:0] steer_pot_limited;  // limited steering potentiometer value
  logic signed [11:0] steer_pot_limited_signed; // signed version of limited steering potentiometer
  logic signed [11:0] signed_steer_pot_limited_3_16th; // 3/16 of signed limited steering potentiometer
  logic signed [12:0] lft_torque;  // left wheel torque before deadzone shaping
  logic signed [12:0] lft_torque_pipe;
  logic signed [12:0] rght_torque_pipe;
  logic signed [12:0] rght_torque;  // right wheel torque before deadzone shaping
  localparam [11:0] STEER_POT_UPPER_LIMIT = 12'hE00;  // upper limit for steering potentiometer
  localparam [11:0] STEER_POT_LOWER_LIMIT = 12'h200;  // lower limit for steering potentiometer
  //localparam [11:0] BITS_12_ONES = 12'h7FF; // conversion factor for unsigned to signed, and saturation

  localparam signed [12:0] MIN_DUTY = $signed(
      13'h0A8
  );  // minimum duty cycle to overcome motor deadzone
  localparam signed [12:0] MIN_DUTY_NEG = $signed(
      13'hFF58
  );  // negative minimum duty cycle to overcome motor deadzone

  localparam signed [6:0] LOW_TORQUE_BAND = $signed(7'h2A);  // torque band for low torque region
  localparam [1:0] GAIN_MULT = 2'd2;  // gain multiplier for low torque region, used as shift
  logic signed [12:0] lft_torque_comp;  // left torque after deadzone compensation
  logic signed [12:0] lft_shaped;  // left torque after deadzone shaping
  logic signed [12:0] rght_torque_comp;  // right torque after deadzone compensation
  logic signed [12:0] rght_shaped;  // right torque after deadzone shaping

  localparam signed [11:0] overspeed_thresh = 12'd1536;  // overspeed threshold

  // scaling with the soft start
  assign PID_cntrl_mult_ss_tmr = (PID_cntrl) * $signed({2'b0, ss_tmr});
  //assign PID_ss = PID_cntrl_mult_ss_tmr[19:8];  // divide by 256

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) PID_ss <= 0;
    else PID_ss <= PID_cntrl_mult_ss_tmr[19:8];
  end

  assign steer_pot_limited = (&steer_pot[11:9] && (|steer_pot[8:0])) ? (STEER_POT_UPPER_LIMIT) :
                              (~(|steer_pot[11:9]) ) ? (STEER_POT_LOWER_LIMIT) :
                               steer_pot;

  logic signed [12:0] steer_scaled_x;
  logic signed [12:0] steer_shift_4;
  logic signed [12:0] steer_shift_3;
  //logic signed [12:0] steer_scaled_x_pipe;

  //assign steer_scaled_x = (steer_pot_limited >>> 4) + (steer_pot_limited >>> 3);

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      steer_shift_4  <= 0;
      steer_shift_3  <= 0;
      steer_scaled_x <= 0;
    end else begin
      steer_shift_4  <= (steer_pot_limited >>> 4);
      steer_shift_3  <= (steer_pot_limited >>> 3);
      steer_scaled_x <= steer_shift_4 + steer_shift_3;
    end
  end

  // Compile-time constant for (3/16)*C
  localparam signed [12:0] STEER_CONST_3_16 = (12'sh801 >>> 4) + (12'sh801 >>> 3);

  // Add constant at the end (constant-add is cheap)
  assign signed_steer_pot_limited_3_16th = steer_scaled_x + STEER_CONST_3_16;

  // the left and right speed calculations
  logic signed [12:0] steer_term;
  // if steering disabled, make the term 0
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      steer_term <= '0;
    end else if (en_steer) begin
      steer_term <= signed_steer_pot_limited_3_16th;
    end else begin
      steer_term <= '0;
    end
  end
  // assign steer_term  = en_steer ? signed_steer_pot_limited_3_16th : 0;
  //assign lft_torque  = PID_ss + steer_term;
  //assign rght_torque = PID_ss - steer_term;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      lft_torque  <= '0;
      rght_torque <= '0;
    end else begin
      lft_torque  <= PID_ss + steer_term;
      rght_torque <= PID_ss - steer_term;
    end
  end

  // deadzone shaping for left side
  logic signed [12:0] lft_torque_mult_or_min_duty;
  localparam signed [6:0] LOW_TORQUE_BAND_NEG = $signed(
      7'sh56
  );  // torque band for low torque region
  logic lft_exceed;
  assign lft_torque_comp = lft_torque + (lft_torque[12] ? MIN_DUTY_NEG : MIN_DUTY);
  assign lft_exceed = (lft_torque > LOW_TORQUE_BAND) || (lft_torque < LOW_TORQUE_BAND_NEG);
  assign lft_torque_mult_or_min_duty = (lft_exceed) ? lft_torque_comp : (lft_torque <<< GAIN_MULT);
  assign lft_shaped = (pwr_up) ? lft_torque_mult_or_min_duty : 0;

  // deadzone shaping for right side
  logic signed [12:0] rght_torque_mult_or_min_duty;
  logic rght_exceed;
  assign rght_torque_comp = rght_torque + (rght_torque[12] ? MIN_DUTY_NEG : MIN_DUTY);
  assign rght_exceed = (rght_torque > LOW_TORQUE_BAND) || (rght_torque < LOW_TORQUE_BAND_NEG);
  assign rght_torque_mult_or_min_duty =  (rght_exceed) ? rght_torque_comp : 
  (rght_torque <<< GAIN_MULT);
  assign rght_shaped = (pwr_up) ? rght_torque_mult_or_min_duty : 0;

  // final saturation and overspeed detection
  assign lft_spd = (!lft_shaped[12] && lft_shaped[11]) ? 12'h7FF :
                               (lft_shaped[12] && !lft_shaped[11]) ? 12'h800 :
                               lft_shaped[11:0];


  assign rght_spd = (!rght_shaped[12] && rght_shaped[11]) ? 12'h7FF :
                               (rght_shaped[12] && !rght_shaped[11]) ? 12'h800 :
                               rght_shaped[11:0];

  assign too_fast = (rght_shaped > overspeed_thresh) || (lft_shaped > overspeed_thresh);

endmodule
