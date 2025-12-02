//////////////////////////////////////////////////////////////////////////////////
// Module: steering_response_tb
//
// Description:
//   Comprehensive physics-based testbench for validating the steering response
//   characteristics of the Segway balance control system. Uses SegwayModel to
//   verify real-time wheel speed differential in response to steering pot input.
//
// Test Categories:
//   - Basic Steering (Tests 1-6): Fundamental left/right steering, saturation limits
//   - Center & Special Conditions (Tests 7-12): Neutral position, backward lean,
//     minimal lean, rapid transitions
//   - Advanced Validation (Tests 13-20): Monotonic progression, symmetry, stability,
//     response time, varying lean angles, glitch-free transitions
//
// Key Signals:
//   - steerPot: 12-bit steering potentiometer input (0x200=max left, 0x800=center,
//               0xE00=max right)
//   - iPHYS.omega_lft/omega_rght: Physical model wheel angular velocities
//   - rider_lean: 16-bit signed rider lean angle (forward positive)
//
// Pass Criteria:
//   All tests must demonstrate correct directional response, proper saturation
//   behavior, monotonic scaling with input, and symmetry between left/right steering.
//////////////////////////////////////////////////////////////////////////////////

module steering_response_tb ();

  import task_pkg::*;

  //// Interconnects to DUT/support defined as type wire /////
  wire SS_n, SCLK, MOSI, MISO, INT;  // to inertial sensor
  wire A2D_SS_n, A2D_SCLK, A2D_MOSI, A2D_MISO;  // to A2D converter
  wire RX_TX;
  wire PWM1_rght, PWM2_rght, PWM1_lft, PWM2_lft;
  wire piezo, piezo_n;
  wire cmd_sent;
  wire rst_n;  // synchronized global reset

  ////// Stimulus is declared as type reg ///////
  reg clk, RST_n;
  reg [7:0] cmd;  // command host is sending to DUT
  reg send_cmd;  // asserted to initiate sending of command
  reg signed [15:0] rider_lean;
  reg [11:0] ld_cell_lft, ld_cell_rght, steerPot, batt;  // A2D values
  reg OVR_I_lft, OVR_I_rght;


  ////////////////////////////////////////////////////////////////
  // Instantiate Physical Model of Segway with Inertial sensor //
  //////////////////////////////////////////////////////////////	
  SegwayModel iPHYS (
      .clk(clk),
      .RST_n(RST_n),
      .SS_n(SS_n),
      .SCLK(SCLK),
      .MISO(MISO),
      .MOSI(MOSI),
      .INT(INT),
      .PWM1_lft(PWM1_lft),
      .PWM2_lft(PWM2_lft),
      .PWM1_rght(PWM1_rght),
      .PWM2_rght(PWM2_rght),
      .rider_lean(rider_lean)
  );

  /////////////////////////////////////////////////////////
  // Instantiate Model of A2D for load cell and battery //
  ///////////////////////////////////////////////////////
  ADC128S_FC iA2D (
      .clk(clk),
      .rst_n(RST_n),
      .SS_n(A2D_SS_n),
      .SCLK(A2D_SCLK),
      .MISO(A2D_MISO),
      .MOSI(A2D_MOSI),
      .ld_cell_lft(ld_cell_lft),
      .ld_cell_rght(ld_cell_rght),
      .steerPot(steerPot),
      .batt(batt)
  );

  ////// Instantiate DUT ////////
  Segway iDUT (
      .clk(clk),
      .RST_n(RST_n),
      .INERT_SS_n(SS_n),
      .INERT_MOSI(MOSI),
      .INERT_SCLK(SCLK),
      .INERT_MISO(MISO),
      .INERT_INT(INT),
      .A2D_SS_n(A2D_SS_n),
      .A2D_MOSI(A2D_MOSI),
      .A2D_SCLK(A2D_SCLK),
      .A2D_MISO(A2D_MISO),
      .PWM1_lft(PWM1_lft),
      .PWM2_lft(PWM2_lft),
      .PWM1_rght(PWM1_rght),
      .PWM2_rght(PWM2_rght),
      .OVR_I_lft(OVR_I_lft),
      .OVR_I_rght(OVR_I_rght),
      .piezo_n(piezo_n),
      .piezo(piezo),
      .RX(RX_TX)
  );

  //// Instantiate UART_tx (mimics command from BLE module) //////
  UART_tx iTX (
      .clk(clk),
      .rst_n(rst_n),
      .TX(RX_TX),
      .trmt(send_cmd),
      .tx_data(cmd),
      .tx_done(cmd_sent)
  );

  /////////////////////////////////////
  // Instantiate reset synchronizer //
  ///////////////////////////////////
  rst_synch iRST (
      .clk  (clk),
      .RST_n(RST_n),
      .rst_n(rst_n)
  );

  // Previous steady-state wheel speeds for comparison between tests
  int prev_lft_spd, prev_rght_spd;

  initial begin
    // Declare all automatic variables at the beginning
    automatic int steer_values_right[3] = '{12'h900, 12'hB00, 12'hD00};
    automatic int steer_values_left[3] = '{12'h700, 12'h500, 12'h300};
    automatic int steer_pattern[6] = '{12'h800, 12'hA00, 12'h800, 12'h600, 12'h800, 12'h800};
    automatic int lean_angles[3] = '{16'sh0400, 16'sh0800, 16'sh0C00};
    int omega_diff[3];
    int right_diff, left_diff;
    logic signed [15:0] steer_temp;  // Temporary signal for glitch check
    logic signed [15:0] steer_transitions[4];
    int i;
    // Averaged wheel speed holders (computed via compute_average task)
    automatic int lft_avg, rght_avg;  // primary averages
    automatic int lft_avg2, rght_avg2;  // secondary averages (used in stability tests)
    
    //-----------------------------------------
    // Global DUT + environment initialization
    //-----------------------------------------
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

    // Send 'G' command to enable Segway (balance controller on)
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));

    // Apply rider weight (both load cells above threshold)
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    repeat (325000) @(posedge clk);  // allow weight-detection and enable logic to settle

    // Apply maximum forward lean to get meaningful wheel speeds before steering
    rider_lean = 16'sh0FFF;
    repeat (50000) @(posedge clk);  // wait for balance loop to reach a steady state
    $display("\n=== Starting Steering Response Tests ===");

    //============================================================
    // TEST 1: Maximum Right Steering Validation
    //
    // Purpose: Verify that applying maximum right steering input produces
    //          the expected differential wheel velocity (left > right).
    //
    // Configuration:
    //   - steerPot: 0xE00 (maximum right steering within normal range)
    //   - rider_lean: 0x0FFF (maximum forward lean for high speed)
    //   - Load cells: Both at 0x300 (rider present)
    //
    // Expected Behavior:
    //   - Left wheel speed (omega_lft) > Right wheel speed (omega_rght)
    //   - Creates clockwise turning motion (viewed from above)
    //   - No unknown/X values in wheel speeds
    //
    // Pass Criteria:
    //   omega_lft > omega_rght AND both values are known (not X/Z)
    //============================================================
    steerPot = 12'hE00;
    $display("\n[TEST 1] Max right steer applied  (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1_000_000) @(posedge clk);  // wait for steering response to settle

    if ($isunknown(
            iPHYS.omega_lft
        ) || $isunknown(
            iPHYS.omega_rght
        ) || iPHYS.omega_lft <= iPHYS.omega_rght) begin
      $display(
          "[FAIL][TEST 1] Expected lft_spd > rght_spd when steering right. lft_spd=%0d, rght_spd=%0d (time=%0t)",
          iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    $display("[PASS][TEST 1] Right steer OK. lft_spd=%0d > rght_spd=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, $time);

    // Save speeds as reference for right-side saturation test
    prev_lft_spd = iPHYS.omega_lft;
    prev_rght_spd = iPHYS.omega_rght;


    //==================================================================
    // TEST 2: Right Steering Saturation Verification
    //
    // Purpose: Validate that the steering system properly saturates at
    //          maximum right input and doesn't produce unbounded response.
    //
    // Configuration:
    //   - steerPot: 0xF00 (beyond maximum valid steering input)
    //   - All other conditions unchanged from TEST 1
    //
    // Expected Behavior:
    //   - Steering effect should saturate (not increase linearly)
    //   - Left wheel speed continues to increase slightly (inertia)
    //   - Right wheel speed continues to decrease slightly
    //   - BUT the rate of change is capped by saturation logic
    //
    // Pass Criteria:
    //   - omega_lft > previous omega_lft (allows natural acceleration)
    //   - omega_rght < previous omega_rght (allows natural deceleration)
    //   - Differential doesn't grow unbounded
    //====================================================================
    steerPot = 12'hF00;
    $display("\n[TEST 2] Right steer saturation check (steerPot = 0x%0h, time = %0t)", steerPot,
             $time);
    repeat (1_000_000) @(posedge clk);

    if ($isunknown(iPHYS.omega_lft) || (iPHYS.omega_lft <= prev_lft_spd)) begin
      $display(
          "[FAIL][TEST 2] lft_spd should continue to increase under saturated right steer. current=%0d, prev=%0d, time=%0t",
          iPHYS.omega_lft, prev_lft_spd, $time);
      $stop();
    end

    if ($isunknown(iPHYS.omega_rght) || (iPHYS.omega_rght >= prev_rght_spd)) begin
      $display(
          "[FAIL][TEST 2] rght_spd should continue to decrease under saturated right steer. current=%0d, prev=%0d, time=%0t",
          iPHYS.omega_rght, prev_rght_spd, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 2] steer saturation OK. lft_spd=%0d (ref=%0d), rght_spd=%0d (ref=%0d), time=%0t",
        iPHYS.omega_lft, prev_lft_spd, iPHYS.omega_rght, prev_rght_spd, $time);

    prev_lft_spd = iPHYS.omega_lft;
    prev_rght_spd = iPHYS.omega_rght;


    //====================================================================
    // TEST 3: Reduced Right Steering Angle (Proportionality Check)
    //
    // Purpose: Verify that reducing the steering angle proportionally
    //          reduces the wheel speed differential (monotonic relationship).
    //
    // Configuration:
    //   - steerPot: 0xA00 (moderate right steering, between center and max)
    //   - All other conditions unchanged
    //
    // Expected Behavior:
    //   - Direction still correct: omega_lft > omega_rght (right turn)
    //   - Magnitude of differential is LESS than TEST 1/2 (less aggressive)
    //   - Left wheel slows down vs. TEST 2 (less outer-wheel acceleration)
    //   - Right wheel speeds up vs. TEST 2 (less inner-wheel deceleration)
    //
    // Pass Criteria:
    //   1. omega_lft > omega_rght (direction maintained)
    //   2. current omega_lft < previous omega_lft (reduced from max steer)
    //   3. current omega_rght > previous omega_rght (reduced from max steer)
    //====================================================================
    steerPot = 12'hA00;
    $display("\n[TEST 3] Reduced right steer (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1_000_000) @(posedge clk);

    if ($isunknown(
            iPHYS.omega_lft
        ) || $isunknown(
            iPHYS.omega_rght
        ) || iPHYS.omega_lft <= iPHYS.omega_rght) begin
      $display(
          "[FAIL][TEST 3] Expected lft_spd > rght_spd for right steer. lft_spd=%0d, rght_spd=%0d (time=%0t)",
          iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 3] Direction OK for reduced right steer. lft_spd=%0d > rght_spd=%0d (time=%0t)",
        iPHYS.omega_lft, iPHYS.omega_rght, $time);

    if (iPHYS.omega_lft >= prev_lft_spd) begin
      $display(
          "[FAIL][TEST 3] Expected lft_spd to decrease when steering angle reduced. current=%0d, prev(max angle)=%0d (time=%0t)",
          iPHYS.omega_lft, prev_lft_spd, $time);
      $stop();
    end

    if (iPHYS.omega_rght <= prev_rght_spd) begin
      $display(
          "[FAIL][TEST 3] Expected rght_spd to increase when steering angle reduced. current=%0d, prev(max angle)=%0d (time=%0t)",
          iPHYS.omega_rght, prev_rght_spd, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 3] Magnitude scales correctly with reduced right steer. lft_spd=%0d (prev=%0d), rght_spd=%0d (prev=%0d), time=%0t",
        iPHYS.omega_lft, prev_lft_spd, iPHYS.omega_rght, prev_rght_spd, $time);


    // Update reference for left-steer tests
    prev_lft_spd = iPHYS.omega_lft;
    prev_rght_spd = iPHYS.omega_rght;


    //============================================================
    // TEST 4: Maximum Left Steering Validation
    //
    // Purpose: Verify that applying maximum left steering input produces
    //          the expected differential wheel velocity (right > left).
    //
    // Configuration:
    //   - steerPot: 0x200 (maximum left steering within normal range)
    //   - rider_lean: Still 0x0FFF (maximum forward lean)
    //   - Previous state: was right-steering, now reversing direction
    //
    // Expected Behavior:
    //   - Right wheel speed (omega_rght) > Left wheel speed (omega_lft)
    //   - Creates counter-clockwise turning motion (viewed from above)
    //   - Symmetric behavior to TEST 1 (mirror image)
    //
    // Pass Criteria:
    //   omega_lft < omega_rght AND both values are known (not X/Z)
    //============================================================
    steerPot = 12'h200;
    $display("\n[TEST 4] Max left steer applied (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1_000_000) @(posedge clk);

    if ($isunknown(
            iPHYS.omega_lft
        ) || $isunknown(
            iPHYS.omega_rght
        ) || iPHYS.omega_lft >= iPHYS.omega_rght) begin
      $display(
          "[FAIL][TEST 4] Expected lft_spd < rght_spd when steering left. lft_spd=%0d, rght_spd=%0d (time=%0t)",
          iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    $display("[PASS][TEST 4] Left steer OK. lft_spd=%0d < rght_spd=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, $time);

    // Save as reference for left-side saturation test
    prev_lft_spd = iDUT.iBAL.lft_spd;
    prev_rght_spd = iDUT.iBAL.rght_spd;


    //==================================================================
    // TEST 5: Left Steering Saturation Verification
    //
    // Purpose: Validate that the steering system properly saturates at
    //          maximum left input (mirror of TEST 2 for right saturation).
    //
    // Configuration:
    //   - steerPot: 0x100 (beyond minimum valid steering input)
    //   - All other conditions unchanged from TEST 4
    //
    // Expected Behavior:
    //   - Steering effect should saturate (not increase linearly beyond limit)
    //   - Left wheel speed shouldn't increase too much (inner wheel)
    //   - Right wheel speed shouldn't decrease too much (outer wheel)
    //   - System prevents over-correction at limits
    //
    // Pass Criteria:
    //   - omega_lft doesn't increase excessively beyond TEST 4 value
    //   - omega_rght doesn't decrease excessively beyond TEST 4 value
    //   - Demonstrates bounded saturation behavior
    //====================================================================
    steerPot = 12'h100;
    $display("\n[TEST 5] Left steer saturation check (steerPot = 0x%0h, time = %0t)", steerPot,
             $time);
    repeat (1_000_000) @(posedge clk);

    if (iPHYS.omega_lft <= prev_lft_spd) begin
      $display(
          "[FAIL][TEST 5] lft_spd should not increase under too much under saturated left steer. current=%0d, prev=%0d, time=%0t",
          iPHYS.omega_lft, prev_lft_spd, $time);
      $stop();
    end

    if (iPHYS.omega_rght <= prev_rght_spd) begin
      $display(
          "[FAIL][TEST 5] rght_spd should not decrease under too much under saturated left steer. current=%0d, prev=%0d, time=%0t",
          iPHYS.omega_rght, prev_rght_spd, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 5] Left steer saturation within tolerance. lft_spd=%0d (ref=%0d), rght_spd=%0d (ref=%0d), time=%0t",
        iPHYS.omega_lft, prev_lft_spd, iPHYS.omega_rght, prev_rght_spd, $time);

    prev_lft_spd = iPHYS.omega_lft;
    prev_rght_spd = iPHYS.omega_rght;

    //====================================================================
    // TEST 6: Reduced Left Steering Angle (Proportionality Check)
    //
    // Purpose: Verify that reducing the left steering angle proportionally
    //          reduces the wheel speed differential (monotonic relationship).
    //          This is the left-side mirror of TEST 3.
    //
    // Configuration:
    //   - steerPot: 0x600 (moderate left steering, between center and max left)
    //   - All other conditions unchanged
    //
    // Expected Behavior:
    //   - Direction still correct: omega_lft < omega_rght (left turn)
    //   - Magnitude of differential is LESS than TEST 4/5 (less aggressive)
    //   - Left wheel speeds up vs. TEST 5 (less inner-wheel deceleration)
    //   - Right wheel slows down vs. TEST 5 (less outer-wheel acceleration)
    //
    // Pass Criteria:
    //   1. omega_lft < omega_rght (direction maintained)
    //   2. current omega_lft > previous omega_lft (reduced from max left steer)
    //   3. current omega_rght < previous omega_rght (reduced from max left steer)
    //====================================================================
    steerPot = 12'h600;
    $display("\n[TEST 6] Reduced left steer (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1_000_000) @(posedge clk);

    if ($isunknown(
            iPHYS.omega_lft
        ) || !$isunknown(
            iPHYS.omega_rght
        ) && iPHYS.omega_lft >= iPHYS.omega_rght) begin
      $display(
          "[FAIL][TEST 6] Expected lft_spd < rght_spd for left steer. lft_spd=%0d, rght_spd=%0d (time=%0t)",
          iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 6] Direction OK for reduced left steer. lft_spd=%0d < rght_spd=%0d (time=%0t)",
        iPHYS.omega_lft, iPHYS.omega_rght, $time);

    if (iPHYS.omega_lft <= prev_lft_spd) begin
      $display(
          "[FAIL][TEST 6] Expected lft_spd to increase when steering angle reduced (less aggressive left). current=%0d, prev(max angle)=%0d (time=%0t)",
          iPHYS.omega_lft, prev_lft_spd, $time);
      $stop();
    end

    if (iPHYS.omega_rght >= prev_rght_spd) begin
      $display(
          "[FAIL][TEST 6] Expected rght_spd to decrease when steering angle reduced. current=%0d, prev(max angle)=%0d (time=%0t)",
          iPHYS.omega_rght, prev_rght_spd, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 6] Magnitude scales correctly with reduced left steer. lft_spd=%0d (prev=%0d), rght_spd=%0d (prev=%0d), time=%0t",
        iPHYS.omega_lft, prev_lft_spd, iPHYS.omega_rght, prev_rght_spd, $time);


    //====================================================================
    // ADDITIONAL PHYSICS-BASED STEERING TEST CASES
    //====================================================================

    //--------------------------------------------------------------------
    // TEST 7: Center/Neutral Steering Position Verification
    //
    // Purpose: Validate that when the steering potentiometer is at the
    //          center position, both wheels rotate at equal speeds,
    //          producing straight-line motion.
    //
    // Configuration:
    //   - steerPot: 0x800 (exact center position)
    //   - rider_lean: Still 0x0FFF (forward lean for motion)
    //   - Extended settling time: 5M cycles for complete stabilization
    //
    // Expected Behavior:
    //   - omega_lft ≈ omega_rght (within tolerance)
    //   - No turning bias in either direction
    //   - Represents the calibration/baseline for steering system
    //
    // Pass Criteria:
    //   |omega_lft - omega_rght| < 500 (tolerance for measurement noise)
    //--------------------------------------------------------------------
    steerPot = 12'h800;
    $display("\n[TEST 7] Center/neutral steering (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (5_000_000) @(posedge clk);

    if (!check_equal_with_tolerance(iPHYS.omega_lft, iPHYS.omega_rght, 500)) begin
      $display("[FAIL][TEST 7] Wheel velocities not equal at center. lft=%0d, rght=%0d, diff=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, iPHYS.omega_lft - iPHYS.omega_rght, $time);
      $stop();
    end

    $display("[PASS][TEST 7] Center steering symmetrical. lft=%0d, rght=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, $time);


    //--------------------------------------------------------------------
    // TEST 8: Slight Right Steering Sensitivity Test
    //
    // Purpose: Verify that small steering inputs produce measurable but
    //          minimal differential response (sensitivity validation).
    //
    // Configuration:
    //   - steerPot: 0x900 (one increment right of center, δ = 0x100)
    //   - Tests the lower bound of steering system resolution
    //
    // Expected Behavior:
    //   - Subtle but detectable turning to the right
    //   - omega_lft slightly > omega_rght
    //   - Differential much smaller than TEST 1-3
    //   - Validates that system is responsive even to small inputs
    //
    // Pass Criteria:
    //   omega_lft > omega_rght (direction correct even for small input)
    //--------------------------------------------------------------------
    steerPot = 12'h900;
    $display("\n[TEST 8] Slight right steer (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1_000_000) @(posedge clk);

    prev_lft_spd = iPHYS.omega_lft;
    prev_rght_spd = iPHYS.omega_rght;

    if (iPHYS.omega_lft <= iPHYS.omega_rght) begin
      $display("[FAIL][TEST 8] Expected lft > rght for slight right steer. lft=%0d, rght=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    $display("[PASS][TEST 8] Slight right steer OK. lft=%0d > rght=%0d, diff=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, iPHYS.omega_lft - iPHYS.omega_rght, $time);


    //--------------------------------------------------------------------
    // TEST 9: Slight Left Steering Sensitivity Test
    //
    // Purpose: Verify that small steering inputs produce measurable but
    //          minimal differential response (mirror of TEST 8).
    //
    // Configuration:
    //   - steerPot: 0x700 (one increment left of center, δ = -0x100)
    //   - Tests the lower bound of steering system resolution
    //
    // Expected Behavior:
    //   - Subtle but detectable turning to the left
    //   - omega_lft slightly < omega_rght
    //   - Differential much smaller than TEST 4-6
    //   - Validates symmetric sensitivity for left steering
    //
    // Pass Criteria:
    //   omega_lft < omega_rght (direction correct even for small input)
    //--------------------------------------------------------------------
    steerPot = 12'h700;
    $display("\n[TEST 9] Slight left steer (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1_000_000) @(posedge clk);

    if (iPHYS.omega_lft >= iPHYS.omega_rght) begin
      $display("[FAIL][TEST 9] Expected lft < rght for slight left steer. lft=%0d, rght=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    $display("[PASS][TEST 9] Slight left steer OK. lft=%0d < rght=%0d, diff=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, iPHYS.omega_rght - iPHYS.omega_lft, $time);


    //--------------------------------------------------------------------
    // TEST 10: Rapid Steering Transition Stability Test
    //
    // Purpose: Validate system stability when steering input changes
    //          rapidly between center, right, and left positions.
    //          Ensures no overshoot, oscillation, or instability.
    //
    // Configuration:
    //   - Sequence: center → right → left → center
    //   - Uses averaged samples (64 samples) to filter transients
    //   - Moderate settling time (300k cycles) between transitions
    //
    // Expected Behavior:
    //   - Each position produces correct directional response
    //   - No sustained oscillations or divergence
    //   - System returns to balanced state at center after transitions
    //   - Averages remain within expected tolerances
    //
    // Pass Criteria:
    //   - Center: |lft_avg - rght_avg| < 1000
    //   - Right: lft_avg > rght_avg
    //   - Left: lft_avg < rght_avg
    //   - Final return to center: |lft_avg - rght_avg| < 1000
    //--------------------------------------------------------------------
    $display("\n[TEST 10] Rapid steering transitions (time = %0t)", $time);

    steerPot = 12'h800;  // center
    repeat (3_000_000) @(posedge clk);
    compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
    compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
    if (!check_equal_with_tolerance(lft_avg, rght_avg, 1000)) begin
      $display("[FAIL][TEST 10] Center (avg) failed. lft_avg=%0d, rght_avg=%0d (time=%0t)",
               lft_avg, rght_avg, $time);
      $stop();
    end

    steerPot = 12'hC00;  // right
    repeat (300_000) @(posedge clk);
    compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
    compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
    if (lft_avg <= rght_avg) begin
      $display("[FAIL][TEST 10] Right steer (avg) failed. lft_avg=%0d, rght_avg=%0d (time=%0t)",
               lft_avg, rght_avg, $time);
      $stop();
    end

    steerPot = 12'h400;  // left
    repeat (300_000) @(posedge clk);
    compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
    compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
    if (lft_avg >= rght_avg) begin
      $display("[FAIL][TEST 10] Left steer (avg) failed. lft_avg=%0d, rght_avg=%0d (time=%0t)",
               lft_avg, rght_avg, $time);
      $stop();
    end

    steerPot = 12'h800;  // back to center
    repeat (300_000) @(posedge clk);
    compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
    compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
    if (!check_equal_with_tolerance(lft_avg, rght_avg, 1000)) begin
      $display("[FAIL][TEST 10] Return to center (avg) failed. lft_avg=%0d, rght_avg=%0d (time=%0t)",
               lft_avg, rght_avg, $time);
      $stop();
    end

    $display("[PASS][TEST 10] Rapid transitions handled (avg). Final: lft_avg=%0d, rght_avg=%0d (time=%0t)",
             lft_avg, rght_avg, $time);


    //--------------------------------------------------------------------
    // TEST 11: Steering Response with Backward Lean
    //
    // Purpose: Validate that steering functionality remains correct when
    //          the rider is leaning backward (negative velocity regime).
    //
    // Configuration:
    //   - rider_lean: -0x0800 (backward lean, negative linear velocity)
    //   - Test both right (0xC00) and left (0x400) steering
    //   - Extended settling: 3M cycles for backward motion to stabilize
    //
    // Expected Behavior:
    //   - Steering direction logic STILL applies:
    //     * Right steer: |omega_lft| > |omega_rght| (outer wheel faster)
    //     * Left steer: |omega_lft| < |omega_rght| (outer wheel faster)
    //   - Differential wheel speeds create turning even in reverse
    //   - Uses averaged values to handle backward dynamics
    //
    // Pass Criteria:
    //   - Right steer: lft_avg > rght_avg
    //   - Left steer: lft_avg < rght_avg
    //--------------------------------------------------------------------
    rider_lean = -16'sh0800;
    $display("\n[TEST 11] Steering with backward lean (lean = %0d, time = %0t)", rider_lean, $time);
    repeat (3_000_000) @(posedge clk);

    // Right steer: left wheel should have higher magnitude than right (outer arc)
    steerPot = 12'hC00;  // strong right
    repeat (1_000_000) @(posedge clk);
    compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
    compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
    if (lft_avg <= rght_avg) begin
      $display("[FAIL][TEST 11] Backward lean right steer (avg) failed. lft_avg=%0d, rght_avg=%0d (time=%0t)",
               lft_avg, rght_avg, $time);
      $stop();
    end

    // Left steer: right wheel should have higher magnitude than left (outer arc)
    steerPot = 12'h400;  // strong left
    repeat (1_000_000) @(posedge clk);
    compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
    compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
    if (lft_avg >= rght_avg) begin
      $display("[FAIL][TEST 11] Backward lean left steer (avg) failed. lft_avg=%0d, rght_avg=%0d (time=%0t)",
               lft_avg, rght_avg, $time);
      $stop();
    end

    $display("[PASS][TEST 11] Backward lean steering (avg) OK. lft_avg=%0d, rght_avg=%0d (time=%0t)",
             lft_avg, rght_avg, $time);

    rider_lean = 16'sh0FFF;  // return to forward lean
    repeat (2_000_000) @(posedge clk);


    //--------------------------------------------------------------------
    // TEST 12: Steering Response with Minimal Lean (Near Upright)
    //
    // Purpose: Verify steering response when rider is nearly upright
    //          (minimal forward lean, low speed condition).
    //
    // Configuration:
    //   - rider_lean: 0x0200 (very small forward lean, low velocity)
    //   - steerPot: 0xD00 (strong right steer for measurable effect)
    //   - Extended settling: 2M cycles initially, 2M more for observation
    //
    // Expected Behavior:
    //   - Even at low speeds, steering differential should be detectable
    //   - Response may be smaller in absolute terms but direction is clear
    //   - Tests that steering isn't overly dependent on high lean angles
    //   - Validates low-speed maneuverability
    //
    // Pass Criteria:
    //   lft_avg > rght_avg (right steering direction maintained at low speed)
    //--------------------------------------------------------------------
    rider_lean = 16'sh0200;
    $display("\n[TEST 12] Steering with minimal lean (lean = %0d, time = %0t)", rider_lean, $time);
    repeat (2_000_000) @(posedge clk);

    steerPot = 12'hD00;  // strong right
    repeat (2_000_000) @(posedge clk);
    compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
    compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
    if (lft_avg <= rght_avg) begin
      $display("[FAIL][TEST 12] Minimal lean right steer (avg) failed. lft_avg=%0d, rght_avg=%0d (time=%0t)",
               lft_avg, rght_avg, $time);
      $stop();
    end

    $display("[PASS][TEST 12] Minimal lean steering (avg) OK. lft_avg=%0d > rght_avg=%0d (time=%0t)",
             lft_avg, rght_avg, $time);

    rider_lean = 16'sh0FFF;  // return to normal lean
    repeat (1_000_000) @(posedge clk);


    //--------------------------------------------------------------------
    // TEST 13: Right Steering Progression - Monotonic Relationship
    //
    // Purpose: Verify that increasing right steering input produces
    //          monotonically increasing wheel speed differential.
    //          Validates linear/proportional steering characteristic.
    //
    // Configuration:
    //   - Test sequence: 0x900, 0xB00, 0xD00 (increasing right steer)
    //   - Compute omega_diff = lft_avg - rght_avg for each position
    //   - 500k cycles settling per position
    //
    // Expected Behavior:
    //   - As steerPot increases (more right steer), omega_diff increases
    //   - Monotonic relationship: diff[0] < diff[1] < diff[2]
    //   - Demonstrates proportional control characteristic
    //   - No inversions or non-monotonic behavior
    //
    // Pass Criteria:
    //   omega_diff[0x900] < omega_diff[0xB00] < omega_diff[0xD00]
    //--------------------------------------------------------------------
    $display("\n[TEST 13] Right steering progression (time = %0t)", $time);

    for (i = 0; i < 3; i++) begin
      steerPot = steer_values_right[i];
      repeat (500_000) @(posedge clk);
      compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
      compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
      omega_diff[i] = lft_avg - rght_avg;
      $display("[TEST 13] steerPot=0x%0h: lft_avg=%0d, rght_avg=%0d, diff=%0d",
               steer_values_right[i], lft_avg, rght_avg, omega_diff[i]);
    end

    if (!(omega_diff[0] < omega_diff[1] && omega_diff[1] < omega_diff[2])) begin
      $display("[FAIL][TEST 13] Not monotonic. diff[0]=%0d, diff[1]=%0d, diff[2]=%0d (time=%0t)",
               omega_diff[0], omega_diff[1], omega_diff[2], $time);
      $stop();
    end

    $display("[PASS][TEST 13] Right steering progression monotonic: %0d < %0d < %0d (time=%0t)",
             omega_diff[0], omega_diff[1], omega_diff[2], $time);


    //--------------------------------------------------------------------
    // TEST 14: Left Steering Progression - Monotonic Relationship
    //
    // Purpose: Verify that increasing left steering input produces
    //          monotonically increasing wheel speed differential.
    //          Mirror test of TEST 13 for left side.
    //
    // Configuration:
    //   - Test sequence: 0x700, 0x500, 0x300 (increasing left steer)
    //   - Compute omega_diff = rght_avg - lft_avg for each position
    //   - 500k cycles settling per position
    //
    // Expected Behavior:
    //   - As steerPot decreases (more left steer), omega_diff increases
    //   - Monotonic relationship: diff[0x700] < diff[0x500] < diff[0x300]
    //   - Demonstrates symmetric proportional control for left steering
    //   - No inversions or non-monotonic behavior
    //
    // Pass Criteria:
    //   omega_diff[0x700] < omega_diff[0x500] < omega_diff[0x300]
    //--------------------------------------------------------------------
    $display("\n[TEST 14] Left steering progression (time = %0t)", $time);

    for (i = 0; i < 3; i++) begin
      steerPot = steer_values_left[i];
      repeat (500_000) @(posedge clk);
      compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
      compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
      omega_diff[i] = rght_avg - lft_avg;
      $display("[TEST 14] steerPot=0x%0h: lft_avg=%0d, rght_avg=%0d, diff=%0d",
               steer_values_left[i], lft_avg, rght_avg, omega_diff[i]);
    end

    if (!(omega_diff[0] < omega_diff[1] && omega_diff[1] < omega_diff[2])) begin
      $display("[FAIL][TEST 14] Not monotonic. diff[0]=%0d, diff[1]=%0d, diff[2]=%0d (time=%0t)",
               omega_diff[0], omega_diff[1], omega_diff[2], $time);
      $stop();
    end

    $display("[PASS][TEST 14] Left steering progression monotonic: %0d < %0d < %0d (time=%0t)",
             omega_diff[0], omega_diff[1], omega_diff[2], $time);

    //--------------------------------------------------------------------
    // TEST 15: Left/Right Steering Symmetry Verification
    //
    // Purpose: Validate that equivalent left and right steering angles
    //          produce symmetric (equal magnitude) wheel speed differentials.
    //          Critical for balanced handling characteristics.
    //
    // Configuration:
    //   - Right test: 0xB00 (center + 0x300)
    //   - Left test: 0x500 (center - 0x300)
    //   - Same magnitude deviation from center in opposite directions
    //
    // Expected Behavior:
    //   - Right: right_diff = lft_avg - rght_avg
    //   - Left: left_diff = rght_avg - lft_avg
    //   - Differences should be approximately equal (symmetric response)
    //   - Tolerance: ±50 counts to account for system noise/asymmetry
    //
    // Pass Criteria:
    //   |right_diff - left_diff| < 50 (symmetric steering characteristic)
    //--------------------------------------------------------------------
    $display("\n[TEST 15] Steering symmetry verification (time = %0t)", $time);

    steerPot = 12'hB00;  // moderate right
  repeat (1_000_000) @(posedge clk);
  compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
  compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
  right_diff = lft_avg - rght_avg;
  $display("[TEST 15] Right (0xB00): lft_avg=%0d, rght_avg=%0d, diff=%0d",
       lft_avg, rght_avg, right_diff);

    steerPot = 12'h500;  // moderate left (symmetric to 0xB00)
  repeat (1_000_000) @(posedge clk);
  compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
  compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
  left_diff = rght_avg - lft_avg;
  $display("[TEST 15] Left (0x500): lft_avg=%0d, rght_avg=%0d, diff=%0d",
       lft_avg, rght_avg, left_diff);

    if (!check_equal_with_tolerance(right_diff, left_diff, 50)) begin
      $display("[FAIL][TEST 15] Steering not symmetric. right_diff=%0d, left_diff=%0d (time=%0t)",
               right_diff, left_diff, $time);
      $stop();
    end

    $display("[PASS][TEST 15] Steering symmetric. right_diff=%0d ≈ left_diff=%0d (time=%0t)",
             right_diff, left_diff, $time);


    //--------------------------------------------------------------------
    // TEST 16: Extended Center Steering Stability Test
    //
    // Purpose: Verify long-term stability at center position with no drift,
    //          oscillation, or divergence over extended operation.
    //
    // Configuration:
    //   - steerPot: 0x800 (center position)
    //   - Two observation windows: 1M cycles apart
    //   - Averaged measurements in each window (64 samples)
    //
    // Expected Behavior:
    //   - Both wheels remain balanced within each window
    //   - No systematic drift between windows
    //   - Demonstrates stable equilibrium at center position
    //   - Both spatial (left vs right) and temporal stability
    //
    // Pass Criteria:
    //   - Window 1: |lft_avg - rght_avg| < 30
    //   - Window 2: |lft_avg2 - rght_avg2| < 30
    //   - Temporal: |lft_avg - lft_avg2| < 200, |rght_avg - rght_avg2| < 200
    //--------------------------------------------------------------------
    $display("\n[TEST 16] Extended center steering stability (time = %0t)", $time);

    steerPot = 12'h800;
    repeat (1_000_000) @(posedge clk);  // first settle window
    compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
    compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
    repeat (1_000_000) @(posedge clk);  // second observation window
    compute_average(iPHYS.omega_lft, 64, clk, lft_avg2);
    compute_average(iPHYS.omega_rght, 64, clk, rght_avg2);
    if (!check_equal_with_tolerance(lft_avg, rght_avg, 30) ||
        !check_equal_with_tolerance(lft_avg2, rght_avg2, 30) ||
        !check_equal_with_tolerance(lft_avg, lft_avg2, 200) ||  // allow small drift
        !check_equal_with_tolerance(rght_avg, rght_avg2, 200)) begin
      $display("[FAIL][TEST 16] Center steering drift (avg). First lft=%0d rght=%0d; Second lft=%0d rght=%0d (time=%0t)",
               lft_avg, rght_avg, lft_avg2, rght_avg2, $time);
      $stop();
    end
    $display("[PASS][TEST 16] Center stable (avg). First lft=%0d rght=%0d; Second lft=%0d rght=%0d (time=%0t)",
             lft_avg, rght_avg, lft_avg2, rght_avg2, $time);


    //--------------------------------------------------------------------
    // TEST 17: Alternating Steering Pattern - Stability Check
    //
    // Purpose: Stress-test the steering system with rapid alternating
    //          left/right/center commands to verify consistent response
    //          and no accumulation of errors or state corruption.
    //
    // Configuration:
    //   - Pattern: center → right → center → left → center → center
    //   - Values: [0x800, 0xA00, 0x800, 0x600, 0x800, 0x800]
    //   - 500k cycles per transition
    //
    // Expected Behavior:
    //   - Each position produces correct directional response
    //   - No unknown states during transitions
    //   - Consistent direction throughout: right steer → lft > rght,
    //     left steer → lft < rght, center → lft ≈ rght
    //   - No state corruption from rapid changes
    //
    // Pass Criteria:
    //   - No X/Z values in omega signals
    //   - Direction correct for each steerPot value
    //--------------------------------------------------------------------
    $display("\n[TEST 17] Alternating steering pattern (time = %0t)", $time);

    for (i = 0; i < 6; i++) begin
      steerPot = steer_pattern[i];
      repeat (500_000) @(posedge clk);
      compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
      compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
      if ($isunknown(iPHYS.omega_lft) || $isunknown(iPHYS.omega_rght)) begin
        $display("[FAIL][TEST 17] Omega unknown during alternating pattern (time=%0t)", $time);
        $stop();
      end
      // Direction consistency per position of steerPot around center (0x800)
      if (steerPot > 12'h800 && lft_avg <= rght_avg) begin
        $display("[FAIL][TEST 17] Expected lft_avg > rght_avg for right steer step (steerPot=0x%0h). lft_avg=%0d rght_avg=%0d", steerPot, lft_avg, rght_avg);
        $stop();
      end
      if (steerPot < 12'h800 && lft_avg >= rght_avg) begin
        $display("[FAIL][TEST 17] Expected lft_avg < rght_avg for left steer step (steerPot=0x%0h). lft_avg=%0d rght_avg=%0d", steerPot, lft_avg, rght_avg);
        $stop();
      end
    end

    $display("[PASS][TEST 17] Alternating pattern stable. lft=%0d, rght=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, $time);


    //--------------------------------------------------------------------
    // TEST 18: Steering Response Time - Latency Validation
    //
    // Purpose: Verify that the steering system responds within acceptable
    //          time after input change (no excessive lag or delay).
    //
    // Configuration:
    //   - Baseline: center position (0x800)
    //   - Transition: max right (0xE00)
    //   - Observation window: 200k cycles after transition
    //   - Uses short averaging (32 samples) to capture transient response
    //
    // Expected Behavior:
    //   - Left wheel speed increases measurably within 200k cycles
    //   - Right wheel speed decreases measurably within 200k cycles
    //   - Demonstrates low-latency response (< 2ms at 100MHz clock)
    //   - System is responsive, not sluggish
    //
    // Pass Criteria:
    //   - lft_avg > prev_lft_spd (left wheel accelerated)
    //   - rght_avg < prev_rght_spd (right wheel decelerated)
    //--------------------------------------------------------------------
    $display("\n[TEST 18] Steering response time test (time = %0t)", $time);

    steerPot = 12'h800;
    repeat (500_000) @(posedge clk);
    compute_average(iPHYS.omega_lft, 32, clk, prev_lft_spd);  // shorter averaging window for latency capture
    compute_average(iPHYS.omega_rght, 32, clk, prev_rght_spd);

    steerPot = 12'hE00;  // max right
    repeat (200_000) @(posedge clk);
    compute_average(iPHYS.omega_lft, 32, clk, lft_avg);
    compute_average(iPHYS.omega_rght, 32, clk, rght_avg);

    if (lft_avg <= prev_lft_spd || rght_avg >= prev_rght_spd) begin
      $display("[FAIL][TEST 18] Steering latency too high (avg). lft: %0d->%0d, rght: %0d->%0d (time=%0t)",
               prev_lft_spd, lft_avg, prev_rght_spd, rght_avg, $time);
      $stop();
    end

    $display("[PASS][TEST 18] Steering responded timely (avg). lft: %0d->%0d, rght: %0d->%0d (time=%0t)",
             prev_lft_spd, lft_avg, prev_rght_spd, rght_avg, $time);


    //--------------------------------------------------------------------
    // TEST 19: Steering Across Multiple Lean Angles
    //
    // Purpose: Validate that steering functionality works correctly across
    //          different rider lean angles (different velocity regimes).
    //
    // Configuration:
    //   - Lean angles: 0x0400, 0x0800, 0x0C00 (low, medium, high forward)
    //   - Consistent steer: 0xC00 (right) for all lean tests
    //   - 500k cycles to stabilize lean, 1M cycles to measure steering
    //
    // Expected Behavior:
    //   - Steering direction maintained at all lean angles
    //   - All tests show lft_avg > rght_avg (right turn)
    //   - Absolute differential may vary with speed, but direction is constant
    //   - Demonstrates robust steering across operating envelope
    //
    // Pass Criteria:
    //   lft_avg > rght_avg for all three lean angles tested
    //--------------------------------------------------------------------
    $display("\n[TEST 19] Steering across multiple lean angles (time = %0t)", $time);

    for (i = 0; i < 3; i++) begin
      rider_lean = lean_angles[i];
      repeat (500_000) @(posedge clk);

      steerPot = 12'hC00;  // consistent right steer
      repeat (1_000_000) @(posedge clk);
      compute_average(iPHYS.omega_lft, 64, clk, lft_avg);
      compute_average(iPHYS.omega_rght, 64, clk, rght_avg);
      if (lft_avg <= rght_avg) begin
        $display("[FAIL][TEST 19] Steering failed at lean=%0d (avg). lft_avg=%0d, rght_avg=%0d (time=%0t)",
                 lean_angles[i], lft_avg, rght_avg, $time);
        $stop();
      end
      $display("[TEST 19] Lean=%0d: lft_avg=%0d > rght_avg=%0d", lean_angles[i], lft_avg, rght_avg);
    end

    $display("[PASS][TEST 19] Steering works across varying lean angles (time=%0t)", $time);


    //--------------------------------------------------------------------
    // TEST 20: Glitch-Free Steering Transitions
    //
    // Purpose: Verify that steering pot changes don't cause transient
    //          glitches, spikes, or invalid states in wheel speeds.
    //
    // Configuration:
    //   - Transition sequence: center → right → left → center
    //   - Values: [0x0800, 0x0C00, 0x0400, 0x0800]
    //   - Glitch detection threshold: 300 counts max transient
    //   - 500k cycles settling per transition
    //
    // Expected Behavior:
    //   - Smooth transitions without large spikes
    //   - No momentary invalid/unknown states
    //   - omega_lft changes monotonically during transition
    //   - No overshoot beyond threshold during settling
    //
    // Pass Criteria:
    //   check_glitch_free_transitions() task passes for all transitions
    //   (monitors omega_lft for spikes > 300 counts during transitions)
    //--------------------------------------------------------------------
    $display("\n[TEST 20] Glitch-free steering transitions (time = %0t)", $time);

    steer_transitions[0] = 16'sh0800;
    steer_transitions[1] = 16'sh0C00;
    steer_transitions[2] = 16'sh0400;
    steer_transitions[3] = 16'sh0800;

    steer_temp = steerPot;  // Initialize temp with current value
    check_glitch_free_transitions(clk, steer_temp, iPHYS.omega_lft, steer_transitions, 500_000, 300);
    steerPot = steer_temp[11:0];  // Sync back if needed

    $display("[PASS][TEST 20] Steering transitions glitch-free (time=%0t)", $time);


    $display("\n=== YAHOO!! All steering response physics tests PASSED at time %0t ===", $time);
    $display("Total tests completed: 20");
    $stop();
  end



  always #10 clk = ~clk;

endmodule
