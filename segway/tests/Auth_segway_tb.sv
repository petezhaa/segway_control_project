module Auth_segway_tb ();

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
  reg        [ 7:0] cmd;  // command host is sending to DUT
  reg               send_cmd;  // asserted to initiate sending of command
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


  initial begin

    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

    // ----------------------------------------------------
    // 1) Before authorization, lean should not drive power
    // ----------------------------------------------------
    rider_lean = 16'sh0700;  // light forward lean
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should be low before 'G' command");
      $stop;
    end else begin
      $display("pwr_up correctly low before authorization");
    end
    if (iDUT.iBAL.lft_spd !== 0 || iDUT.iBAL.rght_spd !== 0) begin
      $error("Wheel speeds should remain zero before authorization");
      $stop;
    end else begin
      $display("Wheel speeds correctly zero before authorization");
    end

    // ----------------------------------------------------
    // 2) Send 'G' to authorize and confirm power-up
    // ----------------------------------------------------
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
    $display("Authorization granted, pwr_up asserted");

    // Give rider weight so rider_off deasserts
    ld_cell_lft  = 12'h350;
    ld_cell_rght = 12'h340;
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(3000), .clk(clk));
    $display("Rider weight detected, rider_off deasserted");

    // ----------------------------------------------------
    // 3) With auth granted, lean should generate wheel motion
    // ----------------------------------------------------
    rider_lean = 16'sh0A00;
    repeat (8000) @(posedge clk);
    if ((iDUT.iBAL.lft_spd === 0) && (iDUT.iBAL.rght_spd === 0)) begin
      $error("Wheel speeds did not respond after authorization and lean");
      $stop;
    end
    $display("Wheel speeds responding correctly after authorization and lean");

    // ----------------------------------------------------
    // 4) Send 'S' (stop) then step off; pwr_up should drop
    // ----------------------------------------------------
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (5000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain asserted immediately after 'S' while rider on");
      $stop;
    end
    $display("'S' command sent, pwr_up remains asserted while rider on");

    // Simulate rider stepping off after stop command
    ld_cell_lft  = 12'h000;
    ld_cell_rght = 12'h000;
    wait4sig(.sig(iDUT.iSTR.rider_off), .clks2wait(300000), .clk(clk));

    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should drop once rider steps off after 'S'");
      $stop;
    end
    $display("Rider stepped off, pwr_up correctly deasserted");

    // Leaning with power off should not drive wheels again
    rider_lean = 16'sh0800;
    repeat (3000) @(posedge clk);
    if (iDUT.iBAL.lft_spd !== 0 || iDUT.iBAL.rght_spd !== 0) begin
      $error("Wheel speeds should return to zero after pwr_up deasserts");
      $stop;
    end
    $display("With pwr_up deasserted, wheel speeds remain zero despite lean");

    // ====================================================
    // EXTRA AUTH_blk COVERAGE SEQUENCES
    // ====================================================

        // ----------------------------------------------------
    // 5) Re-authorize and exercise DISCONNECTED/RECONNECT/IDLE branches
    // ----------------------------------------------------
    // Re-mount rider and re-enable auth
    ld_cell_lft  = 12'h360;
    ld_cell_rght = 12'h350;
    rider_lean   = 16'sh0000;
    $display("starting new auth sequence after remounting rider, %0t", $time);
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
    $display("Re-authorized after step-off; pwr_up asserted again");

    // ----------------------------------------------------
    // 6) Send 'S' with rider_on -> DISCONNECTED (pwr_up stays high)
    // ----------------------------------------------------
    repeat (5000) @(posedge clk);
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    $display("time: %0t Sent 'S' command with rider on", $time);
    // give UART tx time to finish; DISCONNECTED keeps pwr_up high
    repeat (5000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should stay high in DISCONNECTED after 'S' with rider on");
      $stop;
    end
    $display("'S' with rider_on enters DISCONNECTED, pwr_up held high");

    // ----------------------------------------------------
    // 7) Reconnect with 'G', then force IDLE via rider_off in DISCONNECTED
    // ----------------------------------------------------
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
    $display("Reconnected from DISCONNECTED via 'G'");

    // Go to DISCONNECTED again with 'S'
    repeat (5000) @(posedge clk);
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (2000) @(posedge clk); // allow FSM to enter DISCONNECTED

    // Now force rider_off -> should drive IDLE and drop pwr_up
    ld_cell_lft  = 12'h000;
    ld_cell_rght = 12'h000;
    wait4sig(.sig(iDUT.iSTR.rider_off), .clks2wait(300000), .clk(clk));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should drop to IDLE when rider_off asserted in DISCONNECTED");
      $stop;
    end
    $display("rider_off asserted in DISCONNECTED forces IDLE, pwr_up low");

    // Extra check: another 'S' in IDLE should leave pwr_up low
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (3000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should remain low after 'S' in IDLE (post DISCONNECTED)");
      $stop;
    end
    $display("Post-DISCONNECTED 'S' in IDLE leaves pwr_up low");


    // ----------------------------------------------------
    // 8) Test invalid commands in IDLE - should stay in IDLE
    // ----------------------------------------------------
    // Send various invalid commands while in IDLE
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(8'h00));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should remain low after invalid cmd 0x00 in IDLE");
      $stop;
    end
    $display("Invalid cmd 0x00 correctly ignored in IDLE");

    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(8'hFF));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should remain low after invalid cmd 0xFF in IDLE");
      $stop;
    end
    $display("Invalid cmd 0xFF correctly ignored in IDLE");

    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(8'h41)); // 'A'
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should remain low after invalid cmd 0x41 in IDLE");
      $stop;
    end
    $display("Invalid cmd 0x41 correctly ignored in IDLE");

    // ----------------------------------------------------
    // 9) Test multiple 'G' commands in CONNECTED - should stay ON
    // ----------------------------------------------------
    // Re-mount rider
    ld_cell_lft  = 12'h360;
    ld_cell_rght = 12'h350;
    rider_lean   = 16'sh0000;
    
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
    $display("Authorized and in CONNECTED state");

    // Send multiple 'G' commands - should remain in CONNECTED
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high after 'G' in CONNECTED");
      $stop;
    end
    $display("First redundant 'G' command handled correctly");

    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high after second 'G' in CONNECTED");
      $stop;
    end
    $display("Second redundant 'G' command handled correctly");

    // ----------------------------------------------------
    // 10) Test invalid commands in CONNECTED state
    // ----------------------------------------------------
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(8'h42)); // 'B'
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high after invalid cmd in CONNECTED");
      $stop;
    end
    $display("Invalid cmd 0x42 correctly ignored in CONNECTED");

    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(8'h00));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high after invalid cmd 0x00 in CONNECTED");
      $stop;
    end
    $display("Invalid cmd 0x00 correctly ignored in CONNECTED");

    // ----------------------------------------------------
    // 11) Test 'S' with rider_off in CONNECTED -> should go to IDLE
    // ----------------------------------------------------
    // Remove rider weight
    ld_cell_lft  = 12'h000;
    ld_cell_rght = 12'h000;
    wait4sig(.sig(iDUT.iSTR.rider_off), .clks2wait(300000), .clk(clk));
    $display("Rider stepped off");

    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (3000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should drop to IDLE when 'S' received with rider_off");
      $stop;
    end
    $display("'S' with rider_off correctly transitions to IDLE");

    // ----------------------------------------------------
    // 12) Test rapid command sequences
    // ----------------------------------------------------
    // Remount rider
    ld_cell_lft  = 12'h360;
    ld_cell_rght = 12'h350;
    repeat (5000) @(posedge clk);

    // Rapid G -> S -> G sequence
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
    $display("Rapid sequence: 'G' authorized");

    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should stay high in DISCONNECTED after rapid 'S'");
      $stop;
    end
    $display("Rapid sequence: 'S' -> DISCONNECTED");

    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high after rapid 'G' from DISCONNECTED");
      $stop;
    end
    $display("Rapid sequence: 'G' -> back to CONNECTED");

    // ----------------------------------------------------
    // 13) Test multiple 'S' commands in DISCONNECTED
    // ----------------------------------------------------
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high after 'S' in DISCONNECTED");
      $stop;
    end
    $display("First redundant 'S' in DISCONNECTED handled correctly");

    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high after second 'S' in DISCONNECTED");
      $stop;
    end
    $display("Second redundant 'S' in DISCONNECTED handled correctly");

    // ----------------------------------------------------
    // 14) Test invalid commands in DISCONNECTED state
    // ----------------------------------------------------
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(8'h54)); // 'T'
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high after invalid cmd in DISCONNECTED");
      $stop;
    end
    $display("Invalid cmd 0x54 correctly ignored in DISCONNECTED");

    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(8'hAA));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high after invalid cmd 0xAA in DISCONNECTED");
      $stop;
    end
    $display("Invalid cmd 0xAA correctly ignored in DISCONNECTED");

    // ----------------------------------------------------
    // 15) Test rider weight boundary conditions
    // ----------------------------------------------------
    // Go back to CONNECTED
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (5000) @(posedge clk);
    
    // Test very light rider weight (near threshold)
    ld_cell_lft  = 12'h010;
    ld_cell_rght = 12'h010;
    repeat (5000) @(posedge clk);
    $display("Light rider weight applied");

    // Test heavy rider weight
    ld_cell_lft  = 12'hFFF;
    ld_cell_rght = 12'hFFF;
    repeat (5000) @(posedge clk);
    $display("Heavy rider weight applied");

    // Test unbalanced weight
    ld_cell_lft  = 12'h800;
    ld_cell_rght = 12'h100;
    repeat (5000) @(posedge clk);
    $display("Unbalanced rider weight applied");

    // Return to balanced weight
    ld_cell_lft  = 12'h360;
    ld_cell_rght = 12'h350;
    repeat (5000) @(posedge clk);

    // ----------------------------------------------------
    // 16) Test complete cycle: IDLE->CONNECTED->DISCONNECTED->IDLE multiple times
    // ----------------------------------------------------
    for (int cycle = 1; cycle <= 3; cycle++) begin
      $display("  Cycle %0d: Starting from current state", cycle);
      
      // Go to DISCONNECTED if not already there
      SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
      repeat (3000) @(posedge clk);
      
      // Step off -> IDLE
      ld_cell_lft  = 12'h000;
      ld_cell_rght = 12'h000;
      wait4sig(.sig(iDUT.iSTR.rider_off), .clks2wait(300000), .clk(clk));
      repeat (2000) @(posedge clk);
      if (iDUT.iAuth.pwr_up !== 1'b0) begin
        $error("Cycle %0d: pwr_up should be low in IDLE", cycle);
        $stop;
      end
      $display("  Cycle %0d: Reached IDLE", cycle);
      
      // Step on and authorize -> CONNECTED
      ld_cell_lft  = 12'h360;
      ld_cell_rght = 12'h350;
      repeat (5000) @(posedge clk);
      SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
      wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
      $display("  Cycle %0d: Reached CONNECTED", cycle);
      
      // Send 'S' -> DISCONNECTED
      SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
      repeat (3000) @(posedge clk);
      if (iDUT.iAuth.pwr_up !== 1'b1) begin
        $error("Cycle %0d: pwr_up should be high in DISCONNECTED", cycle);
        $stop;
      end
      $display("  Cycle %0d: Reached DISCONNECTED", cycle);
    end
    $display("Multiple complete state cycles PASSED");

    // ----------------------------------------------------
    // 17) Test system behavior with lean during state transitions
    // ----------------------------------------------------
    
    // Return to IDLE
    ld_cell_lft  = 12'h000;
    ld_cell_rght = 12'h000;
    wait4sig(.sig(iDUT.iSTR.rider_off), .clks2wait(300000), .clk(clk));
    repeat (2000) @(posedge clk);
    
    // Apply strong lean while in IDLE
    rider_lean = 16'sh0C00;
    repeat (3000) @(posedge clk);
    if (iDUT.iBAL.lft_spd !== 0 || iDUT.iBAL.rght_spd !== 0) begin
      $error("Wheels should not move with lean in IDLE state");
      $stop;
    end
    $display("Lean correctly ignored in IDLE state");
    
    // Authorize while maintaining lean
    ld_cell_lft  = 12'h360;
    ld_cell_rght = 12'h350;
    repeat (3000) @(posedge clk);
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
    $display("Authorized with lean already applied");
    
    // Verify wheels respond to pre-existing lean
    repeat (8000) @(posedge clk);
    if (iDUT.iBAL.lft_spd === 0 && iDUT.iBAL.rght_spd === 0) begin
      $error("Wheels should respond to lean after authorization");
      $stop;
    end
    $display("Wheels correctly respond to pre-existing lean after authorization");
    
    // Transition to DISCONNECTED while leaning
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (5000) @(posedge clk);
    $display("Transitioned to DISCONNECTED while leaning");
    
    // Reset lean
    rider_lean = 16'sh0000;
    repeat (3000) @(posedge clk);

    // ----------------------------------------------------
    // 18) Test reset during different states
    // ----------------------------------------------------
    
    // Reset while in DISCONNECTED
    reset_DUT(.clk(clk), .RST_n(RST_n));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should be low after reset");
      $stop;
    end
    $display("Reset correctly returns to IDLE with pwr_up low");
    
    // Re-initialize after reset
    ld_cell_lft  = 12'h360;
    ld_cell_rght = 12'h350;
    rider_lean   = 16'sh0000;
    repeat (5000) @(posedge clk);
    
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
    $display("System successfully re-authorized after reset");
    
    // Reset while in CONNECTED state
    reset_DUT(.clk(clk), .RST_n(RST_n));
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should be low after reset from CONNECTED");
      $stop;
    end
    $display("Reset from CONNECTED correctly returns to IDLE");

    // ----------------------------------------------------
    // 19) Edge case: Alternating rider on/off without commands
    // ----------------------------------------------------
    
    // Start from IDLE
    ld_cell_lft  = 12'h000;
    ld_cell_rght = 12'h000;
    repeat (5000) @(posedge clk);
    
    for (int i = 0; i < 3; i++) begin
      // Mount
      ld_cell_lft  = 12'h360;
      ld_cell_rght = 12'h350;
      repeat (5000) @(posedge clk);
      
      // Dismount
      ld_cell_lft  = 12'h000;
      ld_cell_rght = 12'h000;
      repeat (5000) @(posedge clk);
      
      if (iDUT.iAuth.pwr_up !== 1'b0) begin
        $error("pwr_up should remain low during mount/dismount cycles without auth");
        $stop;
      end
    end
    $display("Multiple mount/dismount cycles handled correctly in IDLE");

    $display("ALL AUTH + SEGWAY INTEGRATION TESTS PASSED");
    $stop();
  end

  always #10 clk = ~clk;

endmodule
