//==============================================================
// Module: PID
// Description: 
//   Shell implementation of a discrete PID controller for pitch stabilization.
//   Takes signed pitch and pitch-rate inputs and produces a signed control output.
//   Includes proportional, integral, and derivative terms with saturation protection
//   and a soft-start timer for controlled power-up behavior.
//==============================================================
module PID (
    input  logic signed [15:0] ptch,       // Signed 16-bit pitch signal from inertial interface
    input  logic signed [15:0] ptch_rt,    // Signed 16-bit pitch rate from inertial interface
    input  logic               clk,        // System clock
    input  logic               rst_n,      // Active-low reset
    input  logic               vld,        // Valid signal for updating integral term
    input  logic               pwr_up,     // Power-up enable signal for soft start
    input  logic               rider_off,  // Reset integrator when rider is off
    output logic signed [11:0] PID_cntrl,  // 12-bit signed PID control output
    output logic        [ 7:0] ss_tmr      // 8-bit soft-start timer value
);

  parameter fast_sim = 1;

  //------------------------------------------------------------
  // Local Parameters
  //------------------------------------------------------------
  //localparam P_COEFF = 5'h09;  // Proportional gain coefficient
  localparam P_COEFF_shft = 2'h3;

  //------------------------------------------------------------
  // Internal Signals
  //------------------------------------------------------------
  logic signed [ 9:0] ptch_err_sat;  // Saturated pitch error (to prevent overflow)
  logic signed [14:0] P_term;  // Proportional term
  logic signed [17:0] integrator;  // Integrator register (accumulates pitch error)
  logic signed [17:0] accum_val;  // Sum of pitch error and integrator value
  logic signed [17:0] intergrator_MUX_1;  // MUX-selected integrator input value
  logic signed [14:0] I_term;  // Integral term (scaled down)
  logic signed [12:0] D_term;  // Derivative term
  logic signed [16:0] PID_sum;  // Combined PID output before saturation
  logic signed [26:0] long_tmr;  // Soft-start timer counter
  logic               overflowed;  // Overflow flag for integrator

  //------------------------------------------------------------
  // Proportional (P) Term
  //------------------------------------------------------------
  // Saturate pitch error to prevent overflow during multiplication
  assign ptch_err_sat = (!ptch[15] && |ptch[14:9]) ? 10'h1FF :  // Clamp to max positive
      (ptch[15] && !(&ptch[14:9])) ? 10'h200 :  // Clamp to max negative
      ptch[9:0];

  // Compute proportional term: P = error * coefficient
  assign P_term = (ptch_err_sat <<< P_COEFF_shft) + ptch_err_sat;

  //------------------------------------------------------------
  // Integral (I) Term
  //------------------------------------------------------------
  // Compute accumulator update and detect overflow
  assign accum_val = ptch_err_sat + integrator;

  assign overflowed = (ptch_err_sat[9] == integrator[17]) && (accum_val[17] != integrator[17]);

  // Update integrator only when valid and not overflowed
  assign intergrator_MUX_1 = (vld & ~overflowed) ? accum_val : integrator;

  // Integrator register with reset and rider-off clear
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) integrator <= 18'h00000;
    else if (rider_off) integrator <= 18'h00000;
    else integrator <= intergrator_MUX_1;
  end

  // Scale down integrator value to form I-term
  generate
    if (fast_sim) begin
      assign I_term = (!integrator[17] & |integrator[17:15]) ? 15'h3FFF:
      (integrator[17] & ~(&integrator[17:15])) ? 15'h4000:
      integrator[15:1];
    end else begin
      assign I_term = {{3{integrator[17]}}, integrator[17:6]};
    end
  endgenerate

  //------------------------------------------------------------
  // Derivative (D) Term
  //------------------------------------------------------------
  // Negate and scale pitch rate to form derivative component
  assign D_term = ~{{3{ptch_rt[15]}}, ptch_rt[15:6]} + 1'b1;

  //------------------------------------------------------------
  // PID Output Computation
  //------------------------------------------------------------
  // Combine P, I, and D components
  assign PID_sum = P_term + I_term + D_term;

  // Output saturation to 12-bit signed control range
  assign PID_cntrl = (!PID_sum[16] && |PID_sum[15:11]) ? 12'h7FF :  // Clamp positive
      (PID_sum[16] && !(&PID_sum[15:11])) ? 12'h800 :  // Clamp negative
      PID_sum[11:0];

  //------------------------------------------------------------
  // Soft-Start Timer
  //------------------------------------------------------------
  generate
    if (fast_sim) begin
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) long_tmr <= 27'h0000000;
        else if (!pwr_up) long_tmr <= 27'h0000000;
        else if (~(&long_tmr[26:19]))  // Stop incrementing once max count reached
          //long_tmr <= long_tmr + 9'h100;
          long_tmr <= long_tmr + 20'h01000;
      end
    end else
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) long_tmr <= 27'h0000000;
        else if (!pwr_up) long_tmr <= 27'h0000000;
        else if (~(&long_tmr[26:19]))  // Stop incrementing once max count reached
          long_tmr <= long_tmr + 1'b1;
      end
  endgenerate

  // Expose upper 8 bits of the timer as soft-start indicator
  assign ss_tmr = long_tmr[26:19];

endmodule
