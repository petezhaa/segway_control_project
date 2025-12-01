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
    int steer_diff_1, steer_diff_2, steer_diff_3;
    int right_diff, left_diff;
    
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
    repeat (1000000) @(posedge clk);

    if (!$isunknown(iDUT.iBAL.lft_spd) &&
        !$isunknown(iDUT.iBAL.rght_spd) &&
         iDUT.iBAL.lft_spd <= iDUT.iBAL.rght_spd) begin
      $display("[FAIL][TEST 1] Expected lft_spd > rght_spd when steering right. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 1] Right steer OK. lft_spd=%0d > rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);

    // Save speeds as reference for right-side saturation test
    prev_lft_spd  = iDUT.iBAL.lft_spd;
    prev_rght_spd = iDUT.iBAL.rght_spd;


    //==================================================================
    // TEST 2: Steer pot above max (0xF00) — verify right-steer saturation
    // Expectation: speeds remain ~unchanged compared to TEST 1 (within tol)
    //==================================================================
    steerPot = 12'hF00;
    $display("\n[TEST 2] Right steer saturation check (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1000000) @(posedge clk);

    if (!check_equal_with_tolerance(iDUT.iBAL.lft_spd,  prev_lft_spd,  50)) begin
      $display("[FAIL][TEST 2] lft_spd deviated too much under saturated right steer. current=%0d, prev=%0d, time=%0t",
               iDUT.iBAL.lft_spd, prev_lft_spd, $time);
      $stop();
    end

    if (!check_equal_with_tolerance(iDUT.iBAL.rght_spd, prev_rght_spd, 50)) begin
      $display("[FAIL][TEST 2] rght_spd deviated too much under saturated right steer. current=%0d, prev=%0d, time=%0t",
               iDUT.iBAL.rght_spd, prev_rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 2] Right steer saturation within tolerance. lft_spd=%0d (ref=%0d), rght_spd=%0d (ref=%0d), time=%0t",
             iDUT.iBAL.lft_spd,  prev_lft_spd,
             iDUT.iBAL.rght_spd, prev_rght_spd, $time);


    //====================================================================
    // TEST 3: Right steer at a smaller angle (0xA00)
    // Expectation:
    //   - Still turning right  → lft_spd > rght_spd
    //   - Magnitude of steering effect reduced compared to TEST 1
    //====================================================================
    steerPot = 12'hA00;
    $display("\n[TEST 3] Reduced right steer (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1000000) @(posedge clk);

    if (!$isunknown(iDUT.iBAL.lft_spd) &&
        !$isunknown(iDUT.iBAL.rght_spd) &&
         iDUT.iBAL.lft_spd <= iDUT.iBAL.rght_spd) begin
      $display("[FAIL][TEST 3] Expected lft_spd > rght_spd for right steer. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 3] Direction OK for reduced right steer. lft_spd=%0d > rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);

    if (iDUT.iBAL.lft_spd >= prev_lft_spd) begin
      $display("[FAIL][TEST 3] Expected lft_spd to decrease when steering angle reduced. current=%0d, prev(max angle)=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, prev_lft_spd, $time);
      $stop();
    end

    if (iDUT.iBAL.rght_spd <= prev_rght_spd) begin
      $display("[FAIL][TEST 3] Expected rght_spd to increase when steering angle reduced. current=%0d, prev(max angle)=%0d (time=%0t)",
               iDUT.iBAL.rght_spd, prev_rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 3] Magnitude scales correctly with reduced right steer. lft_spd=%0d (prev=%0d), rght_spd=%0d (prev=%0d), time=%0t",
             iDUT.iBAL.lft_spd,  prev_lft_spd,
             iDUT.iBAL.rght_spd, prev_rght_spd, $time);


    // Update reference for left-steer tests
    prev_lft_spd  = iDUT.iBAL.lft_spd;
    prev_rght_spd = iDUT.iBAL.rght_spd;


    //============================================================
    // TEST 4: Steering fully to the left (0x200)
    // Expectation: left wheel speed < right wheel speed
    //============================================================
    steerPot = 12'h200;
    $display("\n[TEST 4] Max left steer applied (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1000000) @(posedge clk);

    if (!$isunknown(iDUT.iBAL.lft_spd) &&
        !$isunknown(iDUT.iBAL.rght_spd) &&
         iDUT.iBAL.lft_spd >= iDUT.iBAL.rght_spd) begin
      $display("[FAIL][TEST 4] Expected lft_spd < rght_spd when steering left. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 4] Left steer OK. lft_spd=%0d < rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);

    // Save as reference for left-side saturation test
    prev_lft_spd  = iDUT.iBAL.lft_spd;
    prev_rght_spd = iDUT.iBAL.rght_spd;


    //==================================================================
    // TEST 5: Steer pot below min (0x100) — verify left-steer saturation
    // Expectation: speeds remain ~unchanged compared to TEST 4 (within tol)
    //==================================================================
    steerPot = 12'h100;
    $display("\n[TEST 5] Left steer saturation check (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1000000) @(posedge clk);

    if (!check_equal_with_tolerance(iDUT.iBAL.lft_spd,  prev_lft_spd,  50)) begin
      $display("[FAIL][TEST 5] lft_spd deviated too much under saturated left steer. current=%0d, prev=%0d, time=%0t",
               iDUT.iBAL.lft_spd, prev_lft_spd, $time);
      $stop();
    end

    if (!check_equal_with_tolerance(iDUT.iBAL.rght_spd, prev_rght_spd, 50)) begin
      $display("[FAIL][TEST 5] rght_spd deviated too much under saturated left steer. current=%0d, prev=%0d, time=%0t",
               iDUT.iBAL.rght_spd, prev_rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 5] Left steer saturation within tolerance. lft_spd=%0d (ref=%0d), rght_spd=%0d (ref=%0d), time=%0t",
             iDUT.iBAL.lft_spd,  prev_lft_spd,
             iDUT.iBAL.rght_spd, prev_rght_spd, $time);


    //====================================================================
    // TEST 6: Left steer at a smaller angle (0x500)
    // Expectation:
    //   - Still turning left  → lft_spd < rght_spd
    //   - Magnitude of steering effect reduced compared to TEST 4
    //====================================================================
    steerPot = 12'h500;
    $display("\n[TEST 6] Reduced left steer (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1000000) @(posedge clk);

    if (!$isunknown(iDUT.iBAL.lft_spd) &&
        !$isunknown(iDUT.iBAL.rght_spd) &&
         iDUT.iBAL.lft_spd >= iDUT.iBAL.rght_spd) begin
      $display("[FAIL][TEST 6] Expected lft_spd < rght_spd for left steer. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 6] Direction OK for reduced left steer. lft_spd=%0d < rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);

    if (iDUT.iBAL.lft_spd <= prev_lft_spd) begin
      $display("[FAIL][TEST 6] Expected lft_spd to increase when steering angle reduced (less aggressive left). current=%0d, prev(max angle)=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, prev_lft_spd, $time);
      $stop();
    end

    if (iDUT.iBAL.rght_spd >= prev_rght_spd) begin
      $display("[FAIL][TEST 6] Expected rght_spd to decrease when steering angle reduced. current=%0d, prev(max angle)=%0d (time=%0t)",
               iDUT.iBAL.rght_spd, prev_rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 6] Magnitude scales correctly with reduced left steer. lft_spd=%0d (prev=%0d), rght_spd=%0d (prev=%0d), time=%0t",
             iDUT.iBAL.lft_spd,  prev_lft_spd,
             iDUT.iBAL.rght_spd, prev_rght_spd, $time);


    //============================================================
    // TEST 7: Center steering (neutral position, 0x800)
    // Expectation: left and right wheel speeds should be equal
    //============================================================
    steerPot = 12'h800;
    $display("\n[TEST 7] Center/neutral steer applied (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1000000) @(posedge clk);

    if (!check_equal_with_tolerance(iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, 20)) begin
      $display("[FAIL][TEST 7] Expected lft_spd ≈ rght_spd for center steer. lft_spd=%0d, rght_spd=%0d, diff=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, 
               iDUT.iBAL.lft_spd - iDUT.iBAL.rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 7] Center steer OK. lft_spd=%0d ≈ rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);

    prev_lft_spd  = iDUT.iBAL.lft_spd;
    prev_rght_spd = iDUT.iBAL.rght_spd;


    //====================================================================
    // TEST 8: Slight right steer from center (0x900)
    // Expectation: lft_spd > rght_spd, but difference should be small
    //====================================================================
    steerPot = 12'h900;
    $display("\n[TEST 8] Slight right steer from center (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1000000) @(posedge clk);

    if (!$isunknown(iDUT.iBAL.lft_spd) &&
        !$isunknown(iDUT.iBAL.rght_spd) &&
         iDUT.iBAL.lft_spd <= iDUT.iBAL.rght_spd) begin
      $display("[FAIL][TEST 8] Expected lft_spd > rght_spd for slight right steer. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 8] Slight right steer OK. lft_spd=%0d > rght_spd=%0d, diff=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, 
             iDUT.iBAL.lft_spd - iDUT.iBAL.rght_spd, $time);


    //====================================================================
    // TEST 9: Slight left steer from center (0x700)
    // Expectation: lft_spd < rght_spd, but difference should be small
    //====================================================================
    steerPot = 12'h700;
    $display("\n[TEST 9] Slight left steer from center (steerPot = 0x%0h, time = %0t)", steerPot, $time);
    repeat (1000000) @(posedge clk);

    if (!$isunknown(iDUT.iBAL.lft_spd) &&
        !$isunknown(iDUT.iBAL.rght_spd) &&
         iDUT.iBAL.lft_spd >= iDUT.iBAL.rght_spd) begin
      $display("[FAIL][TEST 9] Expected lft_spd < rght_spd for slight left steer. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 9] Slight left steer OK. lft_spd=%0d < rght_spd=%0d, diff=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd,
             iDUT.iBAL.rght_spd - iDUT.iBAL.lft_spd, $time);


    //====================================================================
    // TEST 10: Rapid steering transitions (center -> right -> left -> center)
    // Expectation: System should track each steering change appropriately
    //====================================================================
    $display("\n[TEST 10] Rapid steering transitions test (time = %0t)", $time);
    
    // Start at center
    steerPot = 12'h800;
    repeat (100000) @(posedge clk);
    if (!check_equal_with_tolerance(iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, 20)) begin
      $display("[FAIL][TEST 10] Center position failed. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end
    $display("[TEST 10] Center confirmed. lft_spd=%0d, rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);

    // Quick right
    steerPot = 12'hC00;
    repeat (100000) @(posedge clk);
    if (iDUT.iBAL.lft_spd <= iDUT.iBAL.rght_spd) begin
      $display("[FAIL][TEST 10] Right steer failed. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end
    $display("[TEST 10] Right steer confirmed. lft_spd=%0d > rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);

    // Quick left
    steerPot = 12'h400;
    repeat (100000) @(posedge clk);
    if (iDUT.iBAL.lft_spd >= iDUT.iBAL.rght_spd) begin
      $display("[FAIL][TEST 10] Left steer failed. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end
    $display("[TEST 10] Left steer confirmed. lft_spd=%0d < rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);

    // Back to center
    steerPot = 12'h800;
    repeat (100000) @(posedge clk);
    if (!check_equal_with_tolerance(iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, 20)) begin
      $display("[FAIL][TEST 10] Return to center failed. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 10] Rapid transitions handled correctly. Final: lft_spd=%0d, rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);


    //====================================================================
    // TEST 11: Steering with different lean angles (backward lean)
    // Expectation: Steering behavior should work with backward motion too
    //====================================================================
    rider_lean = -16'sh0800;  // backward lean
    $display("\n[TEST 11] Steering with backward lean (lean = %0d, time = %0t)", rider_lean, $time);
    repeat (50000) @(posedge clk);  // settle to backward motion

    // Right steer while going backward
    steerPot = 12'hC00;
    repeat (100000) @(posedge clk);
    if (iDUT.iBAL.lft_spd <= iDUT.iBAL.rght_spd) begin
      $display("[FAIL][TEST 11] Right steer with backward lean failed. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end
    $display("[TEST 11] Right steer with backward lean OK. lft_spd=%0d > rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);

    // Left steer while going backward
    steerPot = 12'h400;
    repeat (100000) @(posedge clk);
    if (iDUT.iBAL.lft_spd >= iDUT.iBAL.rght_spd) begin
      $display("[FAIL][TEST 11] Left steer with backward lean failed. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 11] Steering with backward lean works correctly. lft_spd=%0d < rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);

    // Return to forward lean for remaining tests
    rider_lean = 16'sh0FFF;
    repeat (50000) @(posedge clk);


    //====================================================================
    // TEST 12: Steering with minimal lean (near upright)
    // Expectation: Steering effect should still be observable
    //====================================================================
    rider_lean = 16'sh0200;  // minimal forward lean
    $display("\n[TEST 12] Steering with minimal lean (lean = %0d, time = %0t)", rider_lean, $time);
    repeat (100000) @(posedge clk);

    // Right steer with minimal lean
    steerPot = 12'hD00;
    repeat (100000) @(posedge clk);
    if (iDUT.iBAL.lft_spd <= iDUT.iBAL.rght_spd) begin
      $display("[FAIL][TEST 12] Right steer with minimal lean failed. lft_spd=%0d, rght_spd=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 12] Steering works with minimal lean. lft_spd=%0d > rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);

    // Return to normal forward lean
    rider_lean = 16'sh0FFF;
    repeat (50000) @(posedge clk);


    //====================================================================
    // TEST 13: Intermediate steering angles progression (right side)
    // Expectation: Verify monotonic relationship between steer angle and speed differential
    //====================================================================
    $display("\n[TEST 13] Intermediate right steering angles progression (time = %0t)", $time);
    
    steerPot = 12'h900;  // slight right
    repeat (100000) @(posedge clk);
    steer_diff_1 = iDUT.iBAL.lft_spd - iDUT.iBAL.rght_spd;
    $display("[TEST 13] steerPot=0x900: diff=%0d (lft=%0d, rght=%0d)", 
             steer_diff_1, iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd);

    steerPot = 12'hB00;  // moderate right
    repeat (100000) @(posedge clk);
    steer_diff_2 = iDUT.iBAL.lft_spd - iDUT.iBAL.rght_spd;
    $display("[TEST 13] steerPot=0xB00: diff=%0d (lft=%0d, rght=%0d)", 
             steer_diff_2, iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd);

    steerPot = 12'hD00;  // strong right
    repeat (100000) @(posedge clk);
    steer_diff_3 = iDUT.iBAL.lft_spd - iDUT.iBAL.rght_spd;
    $display("[TEST 13] steerPot=0xD00: diff=%0d (lft=%0d, rght=%0d)", 
             steer_diff_3, iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd);

    if (!(steer_diff_1 < steer_diff_2 && steer_diff_2 < steer_diff_3)) begin
      $display("[FAIL][TEST 13] Speed differential not monotonically increasing. diff1=%0d, diff2=%0d, diff3=%0d (time=%0t)",
               steer_diff_1, steer_diff_2, steer_diff_3, $time);
      $stop();
    end

    $display("[PASS][TEST 13] Right steering progression is monotonic: %0d < %0d < %0d (time=%0t)",
             steer_diff_1, steer_diff_2, steer_diff_3, $time);


    //====================================================================
    // TEST 14: Intermediate steering angles progression (left side)
    // Expectation: Verify monotonic relationship between steer angle and speed differential
    //====================================================================
    $display("\n[TEST 14] Intermediate left steering angles progression (time = %0t)", $time);
    
    steerPot = 12'h700;  // slight left
    repeat (100000) @(posedge clk);
    steer_diff_1 = iDUT.iBAL.rght_spd - iDUT.iBAL.lft_spd;
    $display("[TEST 14] steerPot=0x700: diff=%0d (rght=%0d, lft=%0d)", 
             steer_diff_1, iDUT.iBAL.rght_spd, iDUT.iBAL.lft_spd);

    steerPot = 12'h500;  // moderate left
    repeat (100000) @(posedge clk);
    steer_diff_2 = iDUT.iBAL.rght_spd - iDUT.iBAL.lft_spd;
    $display("[TEST 14] steerPot=0x500: diff=%0d (rght=%0d, lft=%0d)", 
             steer_diff_2, iDUT.iBAL.rght_spd, iDUT.iBAL.lft_spd);

    steerPot = 12'h300;  // strong left
    repeat (100000) @(posedge clk);
    steer_diff_3 = iDUT.iBAL.rght_spd - iDUT.iBAL.lft_spd;
    $display("[TEST 14] steerPot=0x300: diff=%0d (rght=%0d, lft=%0d)", 
             steer_diff_3, iDUT.iBAL.rght_spd, iDUT.iBAL.lft_spd);

    if (!(steer_diff_1 < steer_diff_2 && steer_diff_2 < steer_diff_3)) begin
      $display("[FAIL][TEST 14] Speed differential not monotonically increasing. diff1=%0d, diff2=%0d, diff3=%0d (time=%0t)",
               steer_diff_1, steer_diff_2, steer_diff_3, $time);
      $stop();
    end

    $display("[PASS][TEST 14] Left steering progression is monotonic: %0d < %0d < %0d (time=%0t)",
             steer_diff_1, steer_diff_2, steer_diff_3, $time);


    //====================================================================
    // TEST 15: Steering symmetry verification
    // Expectation: Same magnitude steer angle in opposite directions should 
    //              produce similar magnitude speed differentials
    //====================================================================
    $display("\n[TEST 15] Steering symmetry verification (time = %0t)", $time);
    
    // Moderate right steer
    steerPot = 12'hB00;
    repeat (100000) @(posedge clk);
    right_diff = iDUT.iBAL.lft_spd - iDUT.iBAL.rght_spd;
    $display("[TEST 15] Right steer (0xB00): diff=%0d (lft=%0d, rght=%0d)", 
             right_diff, iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd);

    // Moderate left steer (symmetric to 0xB00 around 0x800)
    steerPot = 12'h500;
    repeat (100000) @(posedge clk);
    left_diff = iDUT.iBAL.rght_spd - iDUT.iBAL.lft_spd;
    $display("[TEST 15] Left steer (0x500): diff=%0d (rght=%0d, lft=%0d)", 
             left_diff, iDUT.iBAL.rght_spd, iDUT.iBAL.lft_spd);

    if (!check_equal_with_tolerance(right_diff, left_diff, 30)) begin
      $display("[FAIL][TEST 15] Steering not symmetric. right_diff=%0d, left_diff=%0d (time=%0t)",
               right_diff, left_diff, $time);
      $stop();
    end

    $display("[PASS][TEST 15] Steering is symmetric. right_diff=%0d ≈ left_diff=%0d (time=%0t)",
             right_diff, left_diff, $time);


    //====================================================================
    // TEST 16: No steering drift at center over extended period
    // Expectation: With center steering, speeds should remain equal
    //====================================================================
    $display("\n[TEST 16] Extended center steering stability test (time = %0t)", $time);
    
    steerPot = 12'h800;
    repeat (500000) @(posedge clk);  // longer observation window
    
    if (!check_equal_with_tolerance(iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, 20)) begin
      $display("[FAIL][TEST 16] Center steering drifted over time. lft_spd=%0d, rght_spd=%0d, diff=%0d (time=%0t)",
               iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd,
               iDUT.iBAL.lft_spd - iDUT.iBAL.rght_spd, $time);
      $stop();
    end

    $display("[PASS][TEST 16] Center steering stable over extended period. lft_spd=%0d ≈ rght_spd=%0d (time=%0t)",
             iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, $time);


    $display("\n=== All steering response tests PASSED at time %0t ===", $time);
    $display("Total tests completed: 16");
    $stop();
  end



  always #10 clk = ~clk;

endmodule
