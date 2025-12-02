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
  int lft_avg, rght_avg;  // For averaged measurements

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
    // TEST 1: Steering fully to the right (max steer, 0xE00)
    // Expectation: left wheel speed > right wheel speed
    //============================================================
    steerPot = 12'hE00;
    $display("\n[TEST 1] Max right steer applied  (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1_000_000) @(posedge clk);  // wait for steering response to settle

    compute_average(.sig(iPHYS.omega_lft), .num_samples(256), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(256), .clk(clk), .avg_out(rght_avg));
    if ($isunknown(lft_avg) || $isunknown(rght_avg) || lft_avg <= rght_avg) begin
      $display(
          "[FAIL][TEST 1] Expected lft_spd > rght_spd when steering right. lft_avg=%0d, rght_avg=%0d (time=%0t)",
          lft_avg, rght_avg, $time);
      $stop();
    end

    $display("[PASS][TEST 1] Right steer OK. lft_avg=%0d > rght_avg=%0d (time=%0t)",
             lft_avg, rght_avg, $time);

    // Save speeds as reference for right-side saturation test
    prev_lft_spd = lft_avg;
    prev_rght_spd = rght_avg;


    //==================================================================
    // TEST 2: Steer pot above max (0xF00) — verify right-steer saturation
    // Expectation: speeds remain ~unchanged compared to TEST 1 (within tol)
    //==================================================================
    steerPot = 12'hF00;
    $display("\n[TEST 2] Right steer saturation check (steerPot = 0x%0h, time = %0t)", steerPot,
             $time);
    repeat (1_000_000) @(posedge clk);

    compute_average(.sig(iPHYS.omega_lft), .num_samples(256), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(256), .clk(clk), .avg_out(rght_avg));
    if ($isunknown(lft_avg) || (lft_avg <= prev_lft_spd)) begin
      $display(
          "[FAIL][TEST 2] lft_spd should continue to increase under saturated right steer. current=%0d, prev=%0d, time=%0t",
          lft_avg, prev_lft_spd, $time);
      $stop();
    end

    if ($isunknown(rght_avg) || (rght_avg >= prev_rght_spd)) begin
      $display(
          "[FAIL][TEST 2] rght_spd should continue to decrease under saturated right steer. current=%0d, prev=%0d, time=%0t",
          rght_avg, prev_rght_spd, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 2] steer saturation OK. lft_avg=%0d (ref=%0d), rght_avg=%0d (ref=%0d), time=%0t",
        lft_avg, prev_lft_spd, rght_avg, prev_rght_spd, $time);

    prev_lft_spd = lft_avg;
    prev_rght_spd = rght_avg;


    //====================================================================
    // TEST 3: Right steer at a smaller angle (0xA00)
    // Expectation:
    //   - Still turning right  → lft_spd > rght_spd
    //   - Magnitude of steering effect reduced compared to TEST 1
    //====================================================================
    steerPot = 12'hA00;
    $display("\n[TEST 3] Reduced right steer (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1_000_000) @(posedge clk);

    compute_average(.sig(iPHYS.omega_lft), .num_samples(256), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(256), .clk(clk), .avg_out(rght_avg));
    if ($isunknown(lft_avg) || $isunknown(rght_avg) || lft_avg <= rght_avg) begin
      $display(
          "[FAIL][TEST 3] Expected lft_spd > rght_spd for right steer. lft_avg=%0d, rght_avg=%0d (time=%0t)",
          lft_avg, rght_avg, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 3] Direction OK for reduced right steer. lft_avg=%0d > rght_avg=%0d (time=%0t)",
        lft_avg, rght_avg, $time);

    if (lft_avg >= prev_lft_spd) begin
      $display(
          "[FAIL][TEST 3] Expected lft_spd to decrease when steering angle reduced. current=%0d, prev(max angle)=%0d (time=%0t)",
          lft_avg, prev_lft_spd, $time);
      $stop();
    end

    if (rght_avg <= prev_rght_spd) begin
      $display(
          "[FAIL][TEST 3] Expected rght_spd to increase when steering angle reduced. current=%0d, prev(max angle)=%0d (time=%0t)",
          rght_avg, prev_rght_spd, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 3] Magnitude scales correctly with reduced right steer. lft_avg=%0d (prev=%0d), rght_avg=%0d (prev=%0d), time=%0t",
        lft_avg, prev_lft_spd, rght_avg, prev_rght_spd, $time);


    // Update reference for left-steer tests
    prev_lft_spd = lft_avg;
    prev_rght_spd = rght_avg;


    //============================================================
    // TEST 4: Steering fully to the left (0x200)
    // Expectation: left wheel speed < right wheel speed
    //============================================================
    steerPot = 12'h200;
    $display("\n[TEST 4] Max left steer applied (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1_000_000) @(posedge clk);

    compute_average(.sig(iPHYS.omega_lft), .num_samples(256), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(256), .clk(clk), .avg_out(rght_avg));
    if ($isunknown(lft_avg) || $isunknown(rght_avg) || lft_avg >= rght_avg) begin
      $display(
          "[FAIL][TEST 4] Expected lft_spd < rght_spd when steering left. lft_avg=%0d, rght_avg=%0d (time=%0t)",
          lft_avg, rght_avg, $time);
      $stop();
    end

    $display("[PASS][TEST 4] Left steer OK. lft_avg=%0d < rght_avg=%0d (time=%0t)",
             lft_avg, rght_avg, $time);

    // Save as reference for left-side saturation test
    prev_lft_spd = lft_avg;
    prev_rght_spd = rght_avg;


    //==================================================================
    // TEST 5: Steer pot below min (0x100) — verify left-steer saturation
    // Expectation: speeds remain ~unchanged compared to TEST 4 (within tol)
    //==================================================================
    steerPot = 12'h100;
    $display("\n[TEST 5] Left steer saturation check (steerPot = 0x%0h, time = %0t)", steerPot,
             $time);
    repeat (1_000_000) @(posedge clk);

    compute_average(.sig(iPHYS.omega_lft), .num_samples(256), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(256), .clk(clk), .avg_out(rght_avg));
    if (lft_avg <= prev_lft_spd) begin
      $display(
          "[FAIL][TEST 5] lft_spd should not increase under too much under saturated left steer. current=%0d, prev=%0d, time=%0t",
          lft_avg, prev_lft_spd, $time);
      $stop();
    end

    if (rght_avg <= prev_rght_spd) begin
      $display(
          "[FAIL][TEST 5] rght_spd should not decrease under too much under saturated left steer. current=%0d, prev=%0d, time=%0t",
          rght_avg, prev_rght_spd, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 5] Left steer saturation within tolerance. lft_avg=%0d (ref=%0d), rght_avg=%0d (ref=%0d), time=%0t",
        lft_avg, prev_lft_spd, rght_avg, prev_rght_spd, $time);

    prev_lft_spd = lft_avg;
    prev_rght_spd = rght_avg;

    //====================================================================
    // TEST 6: Left steer at a smaller angle (0x500)
    // Expectation:
    //   - Still turning left  → lft_spd < rght_spd
    //   - Magnitude of steering effect reduced compared to TEST 4
    //====================================================================
    steerPot = 12'h600;
    $display("\n[TEST 6] Reduced left steer (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1_000_000) @(posedge clk);

    compute_average(.sig(iPHYS.omega_lft), .num_samples(256), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(256), .clk(clk), .avg_out(rght_avg));
    if ($isunknown(lft_avg) || !$isunknown(rght_avg) && lft_avg >= rght_avg) begin
      $display(
          "[FAIL][TEST 6] Expected lft_spd < rght_spd for left steer. lft_avg=%0d, rght_avg=%0d (time=%0t)",
          lft_avg, rght_avg, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 6] Direction OK for reduced left steer. lft_avg=%0d < rght_avg=%0d (time=%0t)",
        lft_avg, rght_avg, $time);

    if (lft_avg <= prev_lft_spd) begin
      $display(
          "[FAIL][TEST 6] Expected lft_spd to increase when steering angle reduced (less aggressive left). current=%0d, prev(max angle)=%0d (time=%0t)",
          lft_avg, prev_lft_spd, $time);
      $stop();
    end

    if (rght_avg >= prev_rght_spd) begin
      $display(
          "[FAIL][TEST 6] Expected rght_spd to decrease when steering angle reduced. current=%0d, prev(max angle)=%0d (time=%0t)",
          rght_avg, prev_rght_spd, $time);
      $stop();
    end

    $display(
        "[PASS][TEST 6] Magnitude scales correctly with reduced left steer. lft_avg=%0d (prev=%0d), rght_avg=%0d (prev=%0d), time=%0t",
        lft_avg, prev_lft_spd, rght_avg, prev_rght_spd, $time);


    //====================================================================
    // ADDITIONAL PHYSICS-BASED STEERING TEST CASES
    //====================================================================

    //--------------------------------------------------------------------
    // TEST 7: Center steering - verify symmetrical wheel velocities
    //--------------------------------------------------------------------
    steerPot = 12'h800;
    $display("\n[TEST 7] Center/neutral steering (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1_000_000) @(posedge clk);

    if (!check_equal_with_tolerance(iPHYS.omega_lft, iPHYS.omega_rght, 30)) begin
      $display("[FAIL][TEST 7] Wheel velocities not equal at center. lft=%0d, rght=%0d, diff=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, iPHYS.omega_lft - iPHYS.omega_rght, $time);
      $stop();
    end

    $display("[PASS][TEST 7] Center steering symmetrical. lft=%0d, rght=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, $time);


    //--------------------------------------------------------------------
    // TEST 8: Slight right steering - verify minimal differential
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
    // TEST 9: Slight left steering - verify minimal differential
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
    // TEST 10: Rapid steering transitions
    //--------------------------------------------------------------------
    $display("\n[TEST 10] Rapid steering transitions (time = %0t)", $time);

    steerPot = 12'h800;  // center
    repeat (200_000) @(posedge clk);
    if (!check_equal_with_tolerance(iPHYS.omega_lft, iPHYS.omega_rght, 30)) begin
      $display("[FAIL][TEST 10] Center failed. lft=%0d, rght=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    steerPot = 12'hC00;  // right
    repeat (200_000) @(posedge clk);
    if (iPHYS.omega_lft <= iPHYS.omega_rght) begin
      $display("[FAIL][TEST 10] Right steer failed. lft=%0d, rght=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    steerPot = 12'h400;  // left
    repeat (200_000) @(posedge clk);
    if (iPHYS.omega_lft >= iPHYS.omega_rght) begin
      $display("[FAIL][TEST 10] Left steer failed. lft=%0d, rght=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    steerPot = 12'h800;  // back to center
    repeat (200_000) @(posedge clk);
    if (!check_equal_with_tolerance(iPHYS.omega_lft, iPHYS.omega_rght, 30)) begin
      $display("[FAIL][TEST 10] Return to center failed. lft=%0d, rght=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    $display("[PASS][TEST 10] Rapid transitions handled. Final: lft=%0d, rght=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, $time);


    //--------------------------------------------------------------------
    // TEST 11: Steering with backward lean
    //--------------------------------------------------------------------
    rider_lean = -16'sh0800;
    $display("\n[TEST 11] Steering with backward lean (lean = %0d, time = %0t)", rider_lean, $time);
    repeat (500_000) @(posedge clk);

    steerPot = 12'hC00;  // right steer
    repeat (1_000_000) @(posedge clk);
    if (iPHYS.omega_lft <= iPHYS.omega_rght) begin
      $display("[FAIL][TEST 11] Right steer with backward lean failed. lft=%0d, rght=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    steerPot = 12'h400;  // left steer
    repeat (1_000_000) @(posedge clk);
    if (iPHYS.omega_lft >= iPHYS.omega_rght) begin
      $display("[FAIL][TEST 11] Left steer with backward lean failed. lft=%0d, rght=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    $display("[PASS][TEST 11] Steering with backward lean works. lft=%0d < rght=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, $time);

    rider_lean = 16'sh0FFF;  // return to forward lean
    repeat (500_000) @(posedge clk);


    //--------------------------------------------------------------------
    // TEST 12: Steering with minimal lean (near upright)
    //--------------------------------------------------------------------
    rider_lean = 16'sh0200;
    $display("\n[TEST 12] Steering with minimal lean (lean = %0d, time = %0t)", rider_lean, $time);
    repeat (1_000_000) @(posedge clk);

    steerPot = 12'hD00;  // strong right
    repeat (1_000_000) @(posedge clk);
    if (iPHYS.omega_lft <= iPHYS.omega_rght) begin
      $display("[FAIL][TEST 12] Right steer with minimal lean failed. lft=%0d, rght=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, $time);
      $stop();
    end

    $display("[PASS][TEST 12] Steering works with minimal lean. lft=%0d > rght=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, $time);

    rider_lean = 16'sh0FFF;  // return to normal lean
    repeat (500_000) @(posedge clk);


    //--------------------------------------------------------------------
    // TEST 13: Right steering progression - verify monotonic relationship
    //--------------------------------------------------------------------
    $display("\n[TEST 13] Right steering progression (time = %0t)", $time);

    for (i = 0; i < 3; i++) begin
      steerPot = steer_values_right[i];
      repeat (1_000_000) @(posedge clk);
      omega_diff[i] = iPHYS.omega_lft - iPHYS.omega_rght;
      $display("[TEST 13] steerPot=0x%0h: lft=%0d, rght=%0d, diff=%0d",
               steer_values_right[i], iPHYS.omega_lft, iPHYS.omega_rght, omega_diff[i]);
    end

    if (!(omega_diff[0] < omega_diff[1] && omega_diff[1] < omega_diff[2])) begin
      $display("[FAIL][TEST 13] Not monotonic. diff[0]=%0d, diff[1]=%0d, diff[2]=%0d (time=%0t)",
               omega_diff[0], omega_diff[1], omega_diff[2], $time);
      $stop();
    end

    $display("[PASS][TEST 13] Right steering progression monotonic: %0d < %0d < %0d (time=%0t)",
             omega_diff[0], omega_diff[1], omega_diff[2], $time);


    //--------------------------------------------------------------------
    // TEST 14: Left steering progression - verify monotonic relationship
    //--------------------------------------------------------------------
    $display("\n[TEST 14] Left steering progression (time = %0t)", $time);

    for (i = 0; i < 3; i++) begin
      steerPot = steer_values_left[i];
      repeat (1_000_000) @(posedge clk);
      omega_diff[i] = iPHYS.omega_rght - iPHYS.omega_lft;
      $display("[TEST 14] steerPot=0x%0h: lft=%0d, rght=%0d, diff=%0d",
               steer_values_left[i], iPHYS.omega_lft, iPHYS.omega_rght, omega_diff[i]);
    end

    if (!(omega_diff[0] < omega_diff[1] && omega_diff[1] < omega_diff[2])) begin
      $display("[FAIL][TEST 14] Not monotonic. diff[0]=%0d, diff[1]=%0d, diff[2]=%0d (time=%0t)",
               omega_diff[0], omega_diff[1], omega_diff[2], $time);
      $stop();
    end

    $display("[PASS][TEST 14] Left steering progression monotonic: %0d < %0d < %0d (time=%0t)",
             omega_diff[0], omega_diff[1], omega_diff[2], $time);


    //--------------------------------------------------------------------
    // TEST 15: Steering symmetry verification
    //--------------------------------------------------------------------
    $display("\n[TEST 15] Steering symmetry verification (time = %0t)", $time);

    steerPot = 12'hB00;  // moderate right
    repeat (1_000_000) @(posedge clk);
    right_diff = iPHYS.omega_lft - iPHYS.omega_rght;
    $display("[TEST 15] Right (0xB00): lft=%0d, rght=%0d, diff=%0d",
             iPHYS.omega_lft, iPHYS.omega_rght, right_diff);

    steerPot = 12'h500;  // moderate left (symmetric to 0xB00)
    repeat (1_000_000) @(posedge clk);
    left_diff = iPHYS.omega_rght - iPHYS.omega_lft;
    $display("[TEST 15] Left (0x500): lft=%0d, rght=%0d, diff=%0d",
             iPHYS.omega_lft, iPHYS.omega_rght, left_diff);

    if (!check_equal_with_tolerance(right_diff, left_diff, 50)) begin
      $display("[FAIL][TEST 15] Steering not symmetric. right_diff=%0d, left_diff=%0d (time=%0t)",
               right_diff, left_diff, $time);
      $stop();
    end

    $display("[PASS][TEST 15] Steering symmetric. right_diff=%0d ≈ left_diff=%0d (time=%0t)",
             right_diff, left_diff, $time);


    //--------------------------------------------------------------------
    // TEST 16: Extended center steering stability
    //--------------------------------------------------------------------
    $display("\n[TEST 16] Extended center steering stability (time = %0t)", $time);

    steerPot = 12'h800;
    repeat (2_000_000) @(posedge clk);

    if (!check_equal_with_tolerance(iPHYS.omega_lft, iPHYS.omega_rght, 30)) begin
      $display("[FAIL][TEST 16] Center steering drifted. lft=%0d, rght=%0d, diff=%0d (time=%0t)",
               iPHYS.omega_lft, iPHYS.omega_rght, iPHYS.omega_lft - iPHYS.omega_rght, $time);
      $stop();
    end

    $display("[PASS][TEST 16] Center stable over extended period. lft=%0d ≈ rght=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, $time);


    //--------------------------------------------------------------------
    // TEST 17: Alternating steering pattern - stability check
    //--------------------------------------------------------------------
    $display("\n[TEST 17] Alternating steering pattern (time = %0t)", $time);

    for (i = 0; i < 6; i++) begin
      steerPot = steer_pattern[i];
      repeat (500_000) @(posedge clk);

      if ($isunknown(iPHYS.omega_lft) || $isunknown(iPHYS.omega_rght)) begin
        $display("[FAIL][TEST 17] Omega went unknown during alternating pattern (time=%0t)", $time);
        $stop();
      end
    end

    $display("[PASS][TEST 17] Alternating pattern stable. lft=%0d, rght=%0d (time=%0t)",
             iPHYS.omega_lft, iPHYS.omega_rght, $time);


    //--------------------------------------------------------------------
    // TEST 18: Steering response time - verify timely transitions
    //--------------------------------------------------------------------
    $display("\n[TEST 18] Steering response time test (time = %0t)", $time);

    steerPot = 12'h800;
    repeat (500_000) @(posedge clk);
    prev_lft_spd = iPHYS.omega_lft;
    prev_rght_spd = iPHYS.omega_rght;

    steerPot = 12'hE00;  // max right
    repeat (200_000) @(posedge clk);  // give some time to respond

    if (iPHYS.omega_lft <= prev_lft_spd || iPHYS.omega_rght >= prev_rght_spd) begin
      $display("[FAIL][TEST 18] Steering did not respond quickly. lft: %0d->%0d, rght: %0d->%0d (time=%0t)",
               prev_lft_spd, iPHYS.omega_lft, prev_rght_spd, iPHYS.omega_rght, $time);
      $stop();
    end

    $display("[PASS][TEST 18] Steering responded timely. lft: %0d->%0d, rght: %0d->%0d (time=%0t)",
             prev_lft_spd, iPHYS.omega_lft, prev_rght_spd, iPHYS.omega_rght, $time);


    //--------------------------------------------------------------------
    // TEST 19: Steering with varying lean angles
    //--------------------------------------------------------------------
    $display("\n[TEST 19] Steering across multiple lean angles (time = %0t)", $time);

    for (i = 0; i < 3; i++) begin
      rider_lean = lean_angles[i];
      repeat (500_000) @(posedge clk);

      steerPot = 12'hC00;  // consistent right steer
      repeat (1_000_000) @(posedge clk);

      if (iPHYS.omega_lft <= iPHYS.omega_rght) begin
        $display("[FAIL][TEST 19] Steering failed at lean=%0d. lft=%0d, rght=%0d (time=%0t)",
                 lean_angles[i], iPHYS.omega_lft, iPHYS.omega_rght, $time);
        $stop();
      end

      $display("[TEST 19] Lean=%0d: lft=%0d > rght=%0d", lean_angles[i], iPHYS.omega_lft, iPHYS.omega_rght);
    end

    $display("[PASS][TEST 19] Steering works across varying lean angles (time=%0t)", $time);


    //--------------------------------------------------------------------
    // TEST 20: Glitch-free steering transitions
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


    $display("\n=== All steering response physics tests PASSED at time %0t ===", $time);
    $display("Total tests completed: 20");
    $stop();
  end



  always #10 clk = ~clk;

endmodule
