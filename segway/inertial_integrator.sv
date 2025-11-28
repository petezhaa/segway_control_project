//===========================================================
//  Module: inertial_integrator
//  Purpose: Integrate gyro pitch rate readings into pitch angle
//            and fuse them with accelerometer readings to prevent drift.
// Teamname: Gucci Flip flops
//===========================================================
module inertial_integrator (
    input  logic        clk,      // System clock
    input  logic        rst_n,    // Active-low reset
    input  logic        vld,      // High when a new valid inertial reading is available
    input  logic signed [15:0] ptch_rt,  // Gyro pitch rate input (degrees per second)
    input  logic signed [15:0] AZ,       // Accelerometer Z-axis reading
    output logic signed [15:0] ptch      // Fused pitch angle output (degrees)
);

  //------------------------------------------------------------
  // 1. Compensate gyro bias (fixed offset due to mounting)
  //------------------------------------------------------------
  localparam [15:0] PTCH_RT_OFFSET = 16'h0050;  // Factory/measured gyro offset
  logic signed [15:0] ptch_rt_comp;  // Bias-compensated gyro rate

  // Subtract the known offset to remove steady-state bias
  assign ptch_rt_comp = ptch_rt - $signed(PTCH_RT_OFFSET);

  //------------------------------------------------------------
  // 2. Compensate accelerometer bias and estimate pitch from AZ
  //------------------------------------------------------------
  localparam [15:0] AZ_OFFSET = 16'h00A0;  // Measured offset for AZ channel
  logic signed [15:0] AZ_comp;  // Bias-compensated AZ reading
  logic signed [25:0] ptch_acc_product;  // Intermediate multiplication result
  logic signed [15:0] ptch_acc;  // Estimated pitch from accelerometer
  logic signed [11:0] fusion_ptch_offset;  // Fusion correction term (±1024)

  // Subtract offset to remove accelerometer bias
  assign AZ_comp = AZ - $signed(AZ_OFFSET);
  //ptch_acc_product = (ptch_rt_comp << 8) + (ptch_rt_comp << 6) + (ptch_rt_comp << 2) + (ptch_rt_comp << 1) + ptch_rt_comp; use this line if it makes Area smaller

  // Approximate pitch ≈ AZ_comp * 327, for small angles (no trig needed)
  // (327 is an empirically determined scale factor)
  assign ptch_acc_product = AZ_comp * $signed(327);
  //assign ptch_acc_product = (ptch_rt_comp <<< 8) + (ptch_rt_comp <<< 6) + (ptch_rt_comp <<< 2) + (ptch_rt_comp <<< 1) + ptch_rt_comp; // use this line if it makes Area smaller

  // Scale back to 16 bits by right-shifting 13 bits
  // and sign-extending to preserve the sign
  assign ptch_acc = {{3{ptch_acc_product[25]}}, ptch_acc_product[25:13]};

  //------------------------------------------------------------
  // 3. Determine fusion correction term
  //------------------------------------------------------------
  // If the accelerometer's computed pitch (ptch_acc) is greater
  // than the gyro-integrated pitch (ptch), we add a small positive
  // correction into the integrator (+1024).
  // If it’s smaller, we subtract (-1024).
  // This slowly “leaks” the integrator toward the accel reference
  // to cancel out long-term drift from gyro bias.
  //------------------------------------------------------------
  assign fusion_ptch_offset = (ptch_acc > ptch) ? 12'sh400 : -12'sh400;

  //------------------------------------------------------------
  // 4. Integrate the gyro rate over time to get pitch
  //------------------------------------------------------------
  // 27-bit accumulator: wide enough to hold summed values over time
  logic signed [26:0] ptch_int;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // Asynchronous reset clears integrator
      ptch_int <= 27'sd0;
    end else if (vld) begin
      // On each valid reading:
      //   Subtract compensated gyro rate (orientation-dependent)
      //   Add the small fusion correction from the accelerometer
      ptch_int <= ptch_int
                        - {{11{ptch_rt_comp[15]}}, ptch_rt_comp}
                        + {{15{fusion_ptch_offset[11]}}, fusion_ptch_offset}; // sign-extend to 27 bits
    end
    // Else: hold the integrator value (no new reading)
  end

  //------------------------------------------------------------
  // 5. Output scaling
  //------------------------------------------------------------
  // The final pitch angle is the upper 16 bits of the integrator.
  // This effectively divides the integrated value by 2^11,
  // scaling down to a human-readable pitch angle.
  //------------------------------------------------------------
  assign ptch = ptch_int[26:11];

endmodule
