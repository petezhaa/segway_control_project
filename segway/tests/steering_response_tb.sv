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
    repeat (300000) @(posedge clk);  // allow weight-detection and enable logic to settle

    // Apply maximum forward lean to get meaningful wheel speeds before steering
    rider_lean = 16'sh0FFF;
    repeat (1000000) @(posedge clk);  // wait for balance loop to reach a steady state


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


    $display("  All steering response tests PASSED at time %0t", $time);
    $stop();
  end



  always #10 clk = ~clk;

endmodule
