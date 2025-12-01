module over_I_tb ();

  import task_pkg::*;
  import over_I_task_pkg::*;

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

  // Previous steady-state wheel speeds for comparison between tests
  int prev_lft_spd, prev_rght_spd;

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

  initial begin
    //-----------------------------------------
    // Global DUT + environment initialization
    //-----------------------------------------
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

    // Send 'G' command to enable Segway (balance controller on)
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));

    repeat (3000) @(posedge clk);  // wait for some time
    ld_cell_lft  = 12'h300;  // simulate rider getting on
    ld_cell_rght = 12'h300;  // simulate rider getting on
    repeat (325000) @(posedge clk);  // wait for some time

    $display("\n=== Starting Over-Current Tests ===");
    rider_lean = 16'h0FFF;  // simulate rider leaning forward
    repeat (3_000_000) @(posedge clk);  // wait for balance loop to reach a steady state

    $display("Applying left over-current within blanking window");
    pulse_overcurrent_cycles(.cycles(45), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_lft.PWM_synch),
                             .ovr_I_blank(iDUT.iDRV.iPWM_lft.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
                             .OVR_I_rght(OVR_I_rght), .left_or_right(1'b1));
    if(iDUT.iDRV.PWM1_lft === 0 && iDUT.iDRV.PWM2_lft === 0 && iDUT.iDRV.PWM1_rght === 0 && iDUT.iDRV.PWM2_rght === 0) begin
      $display(" Error: PWM outputs disabled during blanking window!");
      $stop();
    end else begin
      $display(" PWM outputs correctly remain enabled during blanking window.");
    end

    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_lft), .target_val(16'h3d00),
                             .tol(16'h0F00));
    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_rght), .target_val(16'h3d00),
                             .tol(16'h0F00));

    $display("Applying left over-current within blanking window did not affect speed");

    $display("Applying right over-current within blanking window");
    pulse_overcurrent_cycles(.cycles(45), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_lft.PWM_synch),
                             .ovr_I_blank(iDUT.iDRV.iPWM_lft.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
                             .OVR_I_rght(OVR_I_rght), .left_or_right(1'b0));
    if(iDUT.iDRV.PWM1_lft === 0 && iDUT.iDRV.PWM2_lft === 0 && iDUT.iDRV.PWM1_rght === 0 && iDUT.iDRV.PWM2_rght === 0) begin
      $display(" Error: PWM outputs disabled during blanking window!");
      $stop();
    end else begin
      $display(" PWM outputs correctly remain enabled during blanking window.");
    end

    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_lft), .target_val(16'h3F00),
                             .tol(16'h0F00));
    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_rght), .target_val(16'h3F00),
                             .tol(16'h0F00));
    $display("Applying right over-current within blanking window did not affect speed");

    $display("Applying over-current on left motor");
    inject_overcurrent_outside_blank(
        .cycles(45), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_lft.PWM_synch),
        .ovr_I_blank(iDUT.iDRV.iPWM_lft.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
        .OVR_I_rght(OVR_I_rght), .left_or_right(1'b1));

    if(iDUT.iDRV.PWM1_lft !== 0 || iDUT.iDRV.PWM2_lft !== 0 || iDUT.iDRV.PWM1_rght !== 0 || iDUT.iDRV.PWM2_rght !== 0) begin
      $display("Error: PWM outputs not disabled after left over-current!");
      $stop();
    end else begin
      $display(" PWM outputs correctly disabled  left over-current.");
    end

    repeat (4_000_000) @(posedge clk);  // wait for some time
    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_lft), .target_val(16'h0A00),
                             .tol(16'h0900));
    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_rght), .target_val(16'h0A00),
                             .tol(16'h0900));
    $display("Left over-current caused speed to drop as expected.");

    $display("reinitialzing DUT for right over-current test");
    //-----------------------------------------
    // Re-initialize DUT + environment
    //-----------------------------------------
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

    // Send 'G' command to enable Segway (balance controller on)
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));

    repeat (3000) @(posedge clk);  // wait for some time
    ld_cell_lft  = 12'h300;  // simulate rider getting on
    ld_cell_rght = 12'h300;  // simulate rider getting on
    repeat (325000) @(posedge clk);  // wait for some time
    rider_lean = 16'h0FFF;
    repeat (4_000_000) @(posedge clk);  // wait for balance loop to reach a steady state
    if ((iPHYS.omega_lft < 16'h1500) || (iPHYS.omega_rght < 16'h1500)) begin
      $display("Error: omega values too low after re-initialization!");
      $stop();
    end
    inject_overcurrent_outside_blank(
        .cycles(45), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_rght.PWM_synch),
        .ovr_I_blank(iDUT.iDRV.iPWM_rght.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
        .OVR_I_rght(OVR_I_rght), .left_or_right(1'b0));

    if(iDUT.iDRV.PWM1_lft !== 0 || iDUT.iDRV.PWM2_lft !== 0 || iDUT.iDRV.PWM1_rght !== 0 || iDUT.iDRV.PWM2_rght !== 0) begin
      $display("Error: PWM outputs not disabled after right over-current!");
      $stop();
    end else begin
      $display(" PWM outputs correctly disabled  right over-current.");
    end

    repeat (4_000_000) @(posedge clk);  // wait for some time
    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_lft), .target_val(16'h0A00),
                             .tol(16'h0900));
    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_rght), .target_val(16'h0A00),
                             .tol(16'h0900));
    $display("Right over-current caused speed to drop as expected.");

    //--------------------------------------------------------------------
    // Multiple rapid over-current pulses within blanking window
    // Expectation: System should continue operating normally
    //--------------------------------------------------------------------
    $display("\nTesting multiple rapid over-current pulses within blanking window");
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (3000) @(posedge clk);
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    repeat (325000) @(posedge clk);
    rider_lean = 16'h0FFF;
    repeat (3_000_000) @(posedge clk);

    $display("Applying 10 rapid over-current pulses within blanking window on left");
    for (int i = 0; i < 10; i++) begin
      pulse_overcurrent_cycles(.cycles(20), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_lft.PWM_synch),
                               .ovr_I_blank(iDUT.iDRV.iPWM_lft.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
                               .OVR_I_rght(OVR_I_rght), .left_or_right(1'b1));
      repeat (100) @(posedge clk);
    end

    if(iDUT.iDRV.PWM1_lft === 0 && iDUT.iDRV.PWM2_lft === 0) begin
      $display("Error: PWM outputs incorrectly disabled after blanking window pulses!");
      $stop();
    end

    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_lft), .target_val(16'h3d00), .tol(16'h0F00));
    $display("Multiple blanking window pulses handled correctly");


    //--------------------------------------------------------------------
    // Over-current during backward lean
    // Expectation: PWM should disable, backward motion should slow
    //--------------------------------------------------------------------
    $display("\nTesting over-current during backward lean");
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (3000) @(posedge clk);
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    repeat (325000) @(posedge clk);
    rider_lean = -16'h0FFF;  // backward lean
    repeat (3_000_000) @(posedge clk);

    $display("Applying over-current on right motor during backward lean");
    inject_overcurrent_outside_blank(.cycles(45), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_rght.PWM_synch),
                                     .ovr_I_blank(iDUT.iDRV.iPWM_rght.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
                                     .OVR_I_rght(OVR_I_rght), .left_or_right(1'b0));

    if(iDUT.iDRV.PWM1_rght !== 0 || iDUT.iDRV.PWM2_rght !== 0) begin
      $display("Error: PWM outputs not disabled after over-current during backward lean!");
      $stop();
    end

    repeat (4_000_000) @(posedge clk);
    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_rght), .target_val(16'h0A00), .tol(16'h0900));
    $display("Over-current during backward lean handled correctly");


    //--------------------------------------------------------------------
    // Over-current with steering applied
    // Expectation: PWM should disable, steering differential should collapse
    //--------------------------------------------------------------------
    $display("\nTesting over-current with steering applied");
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (3000) @(posedge clk);
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    repeat (325000) @(posedge clk);
    rider_lean = 16'h0FFF;
    steerPot = 12'hE00;  // max right steer
    repeat (3_000_000) @(posedge clk);

    $display("Applying over-current on left motor with max right steering");
    inject_overcurrent_outside_blank(.cycles(45), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_lft.PWM_synch),
                                     .ovr_I_blank(iDUT.iDRV.iPWM_lft.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
                                     .OVR_I_rght(OVR_I_rght), .left_or_right(1'b1));

    if(iDUT.iDRV.PWM1_lft !== 0 || iDUT.iDRV.PWM2_lft !== 0) begin
      $display("Error: Left PWM not disabled during steering!");
      $stop();
    end

    repeat (4_000_000) @(posedge clk);
    $display("Over-current with steering handled correctly");


    //--------------------------------------------------------------------
    // Simultaneous over-current on both motors
    // Expectation: Both PWM outputs should disable
    //--------------------------------------------------------------------
    $display("\nTesting simultaneous over-current on both motors");
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (3000) @(posedge clk);
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    repeat (325000) @(posedge clk);
    rider_lean = 16'h0FFF;
    repeat (3_000_000) @(posedge clk);

    $display("Applying simultaneous over-current on both motors");
    repeat (45) @(posedge iDUT.iDRV.iPWM_lft.PWM_synch) begin
      @(posedge clk);
      wait (iDUT.iDRV.iPWM_lft.ovr_I_blank === 1'b0);
      @(posedge clk);
      OVR_I_lft  = 1'b1;
      OVR_I_rght = 1'b1;
      @(posedge clk);
      OVR_I_lft  = 1'b0;
      OVR_I_rght = 1'b0;
    end

    if(iDUT.iDRV.PWM1_lft !== 0 || iDUT.iDRV.PWM2_lft !== 0 || 
       iDUT.iDRV.PWM1_rght !== 0 || iDUT.iDRV.PWM2_rght !== 0) begin
      $display("Error: PWM outputs not disabled after simultaneous over-current!");
      $stop();
    end

    repeat (4_000_000) @(posedge clk);
    $display("Simultaneous over-current handled correctly");


    //--------------------------------------------------------------------
    // Over-current at low battery
    // Expectation: Over-current protection should still work with low battery
    //--------------------------------------------------------------------
    $display("\nTesting over-current at low battery");
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (3000) @(posedge clk);
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    batt = 12'h850;  // just above battery low threshold
    repeat (325000) @(posedge clk);
    rider_lean = 16'h0800;  // moderate lean
    repeat (3_000_000) @(posedge clk);

    $display("Applying over-current with low battery");
    inject_overcurrent_outside_blank(.cycles(45), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_lft.PWM_synch),
                                     .ovr_I_blank(iDUT.iDRV.iPWM_lft.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
                                     .OVR_I_rght(OVR_I_rght), .left_or_right(1'b1));

    if(iDUT.iDRV.PWM1_lft !== 0 || iDUT.iDRV.PWM2_lft !== 0) begin
      $display("Error: PWM not disabled with low battery!");
      $stop();
    end

    $display("Over-current protection works at low battery");


    //--------------------------------------------------------------------
    // Intermittent over-current (on/off pattern)
    // Expectation: Protection should trigger on sustained fault
    //--------------------------------------------------------------------
    $display("\nTesting intermittent over-current pattern");
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (3000) @(posedge clk);
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    batt = 12'hC00;
    repeat (325000) @(posedge clk);
    rider_lean = 16'h0FFF;
    repeat (3_000_000) @(posedge clk);

    $display("Applying intermittent over-current (3 short bursts)");
    for (int i = 0; i < 3; i++) begin
      inject_overcurrent_outside_blank(.cycles(15), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_rght.PWM_synch),
                                       .ovr_I_blank(iDUT.iDRV.iPWM_rght.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
                                       .OVR_I_rght(OVR_I_rght), .left_or_right(1'b0));
      repeat (5000) @(posedge clk);  // brief recovery period
    end

    if(iDUT.iDRV.PWM1_rght !== 0 || iDUT.iDRV.PWM2_rght !== 0) begin
      $display("Error: PWM not disabled after intermittent over-current!");
      $stop();
    end

    $display("Intermittent over-current handled correctly");


    //--------------------------------------------------------------------
    // Over-current at minimal rider lean
    // Expectation: Protection should work regardless of lean angle
    //--------------------------------------------------------------------
    $display("\nTesting over-current at minimal rider lean");
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (3000) @(posedge clk);
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    repeat (325000) @(posedge clk);
    rider_lean = 16'h0100;  // minimal lean
    repeat (3_000_000) @(posedge clk);

    $display("Applying over-current with minimal rider lean");
    inject_overcurrent_outside_blank(.cycles(45), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_lft.PWM_synch),
                                     .ovr_I_blank(iDUT.iDRV.iPWM_lft.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
                                     .OVR_I_rght(OVR_I_rght), .left_or_right(1'b1));

    if(iDUT.iDRV.PWM1_lft !== 0 || iDUT.iDRV.PWM2_lft !== 0) begin
      $display("Error: PWM not disabled at minimal lean!");
      $stop();
    end

    $display("Over-current protection works at minimal lean");


    //--------------------------------------------------------------------
    // Over-current edge timing test (exactly at blanking boundary)
    // Expectation: Verify behavior at blanking window boundary
    //--------------------------------------------------------------------
    $display("\nTesting over-current at blanking window boundary");
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (3000) @(posedge clk);
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    repeat (325000) @(posedge clk);
    rider_lean = 16'h0FFF;
    repeat (3_000_000) @(posedge clk);

    $display("Applying over-current exactly at blanking window falling edge");
    repeat (5) @(posedge iDUT.iDRV.iPWM_lft.PWM_synch) begin
      @(negedge iDUT.iDRV.iPWM_lft.ovr_I_blank);  // catch falling edge
      @(posedge clk);
      OVR_I_lft = 1'b1;
      repeat (30) @(posedge clk);  // hold for several cycles
      OVR_I_lft = 1'b0;
    end

    if(iDUT.iDRV.PWM1_lft !== 0 || iDUT.iDRV.PWM2_lft !== 0) begin
      $display("Error: PWM not disabled at blanking boundary!");
      $stop();
    end

    $display("Over-current at blanking boundary handled correctly");


    //--------------------------------------------------------------------
    // Over-current during rapid steering changes
    // Expectation: Protection should work during dynamic steering
    //--------------------------------------------------------------------
    $display("\nTesting over-current during rapid steering changes");
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (3000) @(posedge clk);
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    repeat (325000) @(posedge clk);
    rider_lean = 16'h0FFF;
    repeat (3_000_000) @(posedge clk);

    // Start rapid steering changes
    fork
      begin
        for (int i = 0; i < 5; i++) begin
          steerPot = 12'hE00;
          repeat (100000) @(posedge clk);
          steerPot = 12'h200;
          repeat (100000) @(posedge clk);
        end
      end
      begin
        repeat (250000) @(posedge clk);  // wait a bit then inject fault
        $display("Injecting over-current during steering transitions");
        inject_overcurrent_outside_blank(.cycles(45), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_rght.PWM_synch),
                                         .ovr_I_blank(iDUT.iDRV.iPWM_rght.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
                                         .OVR_I_rght(OVR_I_rght), .left_or_right(1'b0));
      end
    join

    if(iDUT.iDRV.PWM1_rght !== 0 || iDUT.iDRV.PWM2_rght !== 0) begin
      $display("Error: PWM not disabled during steering changes!");
      $stop();
    end

    $display("Over-current during steering changes handled correctly");


    //--------------------------------------------------------------------
    // Verify no false triggers from very short glitches
    // Expectation: Single-cycle glitches should be filtered out
    //--------------------------------------------------------------------
    $display("\nTesting single-cycle glitch filter");
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (3000) @(posedge clk);
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    repeat (325000) @(posedge clk);
    rider_lean = 16'h0FFF;
    repeat (3_000_000) @(posedge clk);

    $display("Applying very short single-cycle glitches (should be ignored)");
    for (int i = 0; i < 100; i++) begin
      @(posedge clk);
      OVR_I_lft = 1'b1;
      @(posedge clk);
      OVR_I_lft = 1'b0;
      repeat (100) @(posedge clk);
    end

    if(iDUT.iDRV.PWM1_lft === 0 || iDUT.iDRV.PWM2_lft === 0) begin
      $display("Error: PWM incorrectly disabled by single-cycle glitches!");
      $stop();
    end

    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_lft), .target_val(16'h3d00), .tol(16'h0F00));
    $display("Single-cycle glitches correctly ignored");


    //--------------------------------------------------------------------
    // Over-current recovery verification
    // Expectation: System should stay disabled until reset
    //--------------------------------------------------------------------
    $display("\nTesting over-current recovery verification");
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (3000) @(posedge clk);
    ld_cell_lft  = 12'h300;
    ld_cell_rght = 12'h300;
    repeat (325000) @(posedge clk);
    rider_lean = 16'h0FFF;
    repeat (3_000_000) @(posedge clk);

    $display("Trigger over-current and verify no auto-recovery");
    inject_overcurrent_outside_blank(.cycles(45), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_lft.PWM_synch),
                                     .ovr_I_blank(iDUT.iDRV.iPWM_lft.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
                                     .OVR_I_rght(OVR_I_rght), .left_or_right(1'b1));

    // Wait extended period and verify outputs stay disabled
    repeat (10_000_000) @(posedge clk);
    if(iDUT.iDRV.PWM1_lft !== 0 || iDUT.iDRV.PWM2_lft !== 0 ||
       iDUT.iDRV.PWM1_rght !== 0 || iDUT.iDRV.PWM2_rght !== 0) begin
      $display("Error: System auto-recovered from over-current (should require reset)!");
      $stop();
    end

    $display("System correctly stays disabled after over-current");


    //--------------------------------------------------------------------
    // Over-current with varying PWM duty cycles
    // Expectation: Protection should work at all duty cycles
    //--------------------------------------------------------------------
    $display("\nTesting over-current at different lean angles (duty cycles)");
    int lean_angles[4] = '{16'sh0400, 16'sh0800, 16'sh0C00, 16'sh0FFF};

    for (int i = 0; i < 4; i++) begin
      init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
               .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
               .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));
      SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
      repeat (3000) @(posedge clk);
      ld_cell_lft  = 12'h300;
      ld_cell_rght = 12'h300;
      repeat (325000) @(posedge clk);
      rider_lean = lean_angles[i];
      repeat (2_000_000) @(posedge clk);

      $display("Testing over-current at lean angle 0x%0h", lean_angles[i]);
      inject_overcurrent_outside_blank(.cycles(45), .clk(clk), .PWM_synch(iDUT.iDRV.iPWM_lft.PWM_synch),
                                       .ovr_I_blank(iDUT.iDRV.iPWM_lft.ovr_I_blank), .OVR_I_lft(OVR_I_lft),
                                       .OVR_I_rght(OVR_I_rght), .left_or_right(1'b1));

      if(iDUT.iDRV.PWM1_lft !== 0 || iDUT.iDRV.PWM2_lft !== 0) begin
        $display("Error: PWM not disabled at lean 0x%0h!", lean_angles[i]);
        $stop();
      end
    end

    $display("Over-current protection works at all duty cycles");


    $display("\n=== Over-Current Tests Complete ===");
    $stop();
  end



  always #10 clk = ~clk;

endmodule
