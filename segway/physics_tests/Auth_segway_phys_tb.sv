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
  reg [7:0] cmd;               // command host is sending to DUT
  reg send_cmd;                // asserted to initiate sending of command
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

  int prev_omega_lft, prev_omega_rght;
  initial begin

    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

    // ----------------------------------------------------
    // 1) Before authorization, lean should not drive power
    // ----------------------------------------------------
    rider_lean = 16'sh0700;  // light forward lean
    repeat (1_000_000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should be low before 'G' command");
      $stop;
    end else begin
      $display("pwr_up correctly low before authorization");
    end
    if (iPHYS.omega_lft !== 0 || iPHYS.omega_rght !== 0) begin
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
    repeat (200_000) @(posedge clk);
    if ((iPHYS.omega_lft === 0) && (iPHYS.omega_rght === 0)) begin
      $error("Wheel speeds did not respond after authorization and lean");
      $stop;
    end
    $display("Wheel speeds responding correctly after authorization and lean");
    prev_omega_lft  = iPHYS.omega_lft;
    prev_omega_rght = iPHYS.omega_rght;

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
    repeat (1_000_000) @(posedge clk);
    if (iPHYS.omega_lft >= prev_omega_lft || iPHYS.omega_rght >= prev_omega_rght) begin
      $error("Wheel speeds should return to zero after pwr_up deasserts");
      $stop;
    end
    $display("With pwr_up deasserted, wheel speeds remain zero despite lean");

    //--------------------------------------------------------------------
    // Re-authorize with 'G' after rider steps back on
    //--------------------------------------------------------------------
    $display("\nTesting re-authorization after rider returns");
    ld_cell_lft  = 12'h350;
    ld_cell_rght = 12'h340;
    repeat (300000) @(posedge clk);
    
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should reassert after 'G' command with rider on");
      $stop;
    end
    $display("Re-authorization successful, pwr_up asserted");
    
    rider_lean = 16'sh0800;
    repeat (500_000) @(posedge clk);
    if (iPHYS.omega_lft === 0 && iPHYS.omega_rght === 0) begin
      $error("Wheel speeds should respond after re-authorization");
      $stop;
    end
    $display("System functioning correctly after re-authorization");

    //--------------------------------------------------------------------
    // Multiple 'G' commands while already authorized
    //--------------------------------------------------------------------
    $display("\nTesting multiple 'G' commands while authorized");
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (5000) @(posedge clk);
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (5000) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain asserted after redundant 'G' commands");
      $stop;
    end
    $display("Redundant 'G' commands handled correctly");

    //--------------------------------------------------------------------
    // Stop command 'S' while rider still on platform
    //--------------------------------------------------------------------
    $display("\nTesting 'S' command while rider remains on platform");
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (5000) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should stay high after 'S' if rider still on");
      $stop;
    end
    $display("'S' command with rider on: pwr_up remains high");

    //--------------------------------------------------------------------
    // Send 'G' command to re-enable from disconnected state
    //--------------------------------------------------------------------
    $display("\nTesting 'G' command recovery from disconnected state");
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (5000) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should reassert after 'G' from disconnected state");
      $stop;
    end
    $display("Recovery from disconnected state successful");

    //--------------------------------------------------------------------
    // Invalid UART command (not 'G' or 'S')
    //--------------------------------------------------------------------
    $display("\nTesting invalid UART command");
    cmd = 8'h41;  // 'A' - invalid command
    send_cmd = 1;
    @(negedge clk);
    send_cmd = 0;
    repeat (UART_TX_FULL_FRAME) @(posedge clk);
    repeat (5000) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain unchanged after invalid command");
      $stop;
    end
    $display("Invalid command correctly ignored");

    //--------------------------------------------------------------------
    // Rider steps off without 'S' command first
    //--------------------------------------------------------------------
    $display("\nTesting rider stepping off without 'S' command");
    ld_cell_lft  = 12'h000;
    ld_cell_rght = 12'h000;
    wait4sig(.sig(iDUT.iSTR.rider_off), .clks2wait(300000), .clk(clk));
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high until 'S' command even if rider off");
      $stop;
    end
    $display("Rider stepped off without 'S': pwr_up correctly stays high");

    //--------------------------------------------------------------------
    // Send 'S' after rider already off
    //--------------------------------------------------------------------
    $display("\nTesting 'S' command after rider already stepped off");
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (5000) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should deassert after 'S' with rider off");
      $stop;
    end
    $display("'S' command with rider off: pwr_up correctly deasserted");

    //--------------------------------------------------------------------
    // Attempt 'G' authorization without rider on platform
    //--------------------------------------------------------------------
    $display("\nTesting 'G' authorization without rider weight");
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (5000) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should assert even without rider (auth granted)");
      $stop;
    end
    $display("Authorization granted without rider weight");
    
    // Verify no wheel motion without rider
    rider_lean = 16'sh0A00;
    repeat (500_000) @(posedge clk);
    if (iPHYS.omega_lft !== 0 || iPHYS.omega_rght !== 0) begin
      $error("Wheels should not move without rider weight");
      $stop;
    end
    $display("Wheels correctly remain stationary without rider");

    //--------------------------------------------------------------------
    // Rapid 'G' and 'S' command sequence
    //--------------------------------------------------------------------
    $display("\nTesting rapid 'G' and 'S' command sequence");
    ld_cell_lft  = 12'h350;
    ld_cell_rght = 12'h340;
    repeat (300000) @(posedge clk);
    
    for (int i = 0; i < 5; i++) begin
      SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
      repeat (1000) @(posedge clk);
      SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
      repeat (1000) @(posedge clk);
    end
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high with rider on after rapid commands");
      $stop;
    end
    $display("Rapid command sequence handled correctly");

    //--------------------------------------------------------------------
    // Power-up immediately after reset with 'G'
    //--------------------------------------------------------------------
    $display("\nTesting power-up immediately after reset");
    @(negedge clk);
    RST_n = 0;
    repeat (5) @(posedge clk);
    @(negedge clk);
    RST_n = 1;
    repeat (10) @(posedge clk);
    
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should assert after reset and 'G' command");
      $stop;
    end
    $display("Power-up after reset successful");

    //--------------------------------------------------------------------
    // Rider weight threshold testing (gradual step-on)
    //--------------------------------------------------------------------
    $display("\nTesting gradual rider weight application");
    ld_cell_lft  = 12'h000;
    ld_cell_rght = 12'h000;
    repeat (300000) @(posedge clk);
    
    ld_cell_lft  = 12'h100;
    ld_cell_rght = 12'h0F0;
    repeat (100000) @(posedge clk);
    
    ld_cell_lft  = 12'h200;
    ld_cell_rght = 12'h1F0;
    repeat (100000) @(posedge clk);
    
    ld_cell_lft  = 12'h350;
    ld_cell_rght = 12'h340;
    repeat (300000) @(posedge clk);
    
    if (iDUT.iSTR.rider_off !== 1'b0) begin
      $error("rider_off should be low after sufficient weight applied");
      $stop;
    end
    $display("Gradual weight application detected correctly");

    //--------------------------------------------------------------------
    // Unbalanced load cell values (asymmetric weight)
    //--------------------------------------------------------------------
    $display("\nTesting unbalanced rider weight");
    ld_cell_lft  = 12'h400;
    ld_cell_rght = 12'h200;
    repeat (300000) @(posedge clk);
    
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (5000) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should assert with unbalanced weight");
      $stop;
    end
    $display("Unbalanced weight handled correctly");

    //--------------------------------------------------------------------
    // State persistence across multiple operations
    //--------------------------------------------------------------------
    $display("\nTesting state persistence through operation sequence");
    rider_lean = 16'sh0600;
    repeat (500_000) @(posedge clk);
    
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (5000) @(posedge clk);
    
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (5000) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should be high after S->G sequence with rider on");
      $stop;
    end
    $display("State persistence verified through operations");

    //--------------------------------------------------------------------
    // Test with varying lean angles while authorized
    //--------------------------------------------------------------------
    $display("\nTesting system response with varying lean angles");
    int lean_values[5] = '{16'sh0200, 16'sh0600, 16'sh0A00, 16'sh0600, 16'sh0200};
    
    for (int i = 0; i < 5; i++) begin
      rider_lean = lean_values[i];
      repeat (200_000) @(posedge clk);
      
      if (iDUT.iAuth.pwr_up !== 1'b1) begin
        $error("pwr_up should remain high during lean variations");
        $stop;
      end
    end
    $display("System stable through lean variations");

    //--------------------------------------------------------------------
    // Simultaneous 'S' command and rider step-off
    //--------------------------------------------------------------------
    $display("\nTesting simultaneous 'S' command and rider step-off");
    fork
      begin
        SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
      end
      begin
        repeat (UART_TX_FULL_FRAME/2) @(posedge clk);
        ld_cell_lft  = 12'h000;
        ld_cell_rght = 12'h000;
      end
    join
    
    repeat (300000) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("pwr_up should deassert after 'S' and rider off");
      $stop;
    end
    $display("Simultaneous 'S' and step-off handled correctly");

    //--------------------------------------------------------------------
    // Extended operation test (long duration authorized)
    //--------------------------------------------------------------------
    $display("\nTesting extended operation duration");
    ld_cell_lft  = 12'h350;
    ld_cell_rght = 12'h340;
    repeat (300000) @(posedge clk);
    
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (5000) @(posedge clk);
    
    rider_lean = 16'sh0800;
    repeat (2_000_000) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain stable during extended operation");
      $stop;
    end
    $display("Extended operation duration successful");

    //--------------------------------------------------------------------
    // Multiple invalid commands mixed with valid ones
    //--------------------------------------------------------------------
    $display("\nTesting mixed valid and invalid commands");
    cmd = 8'h42;  // 'B'
    send_cmd = 1;
    @(negedge clk);
    send_cmd = 0;
    repeat (UART_TX_FULL_FRAME) @(posedge clk);
    
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (2000) @(posedge clk);
    
    cmd = 8'h58;  // 'X'
    send_cmd = 1;
    @(negedge clk);
    send_cmd = 0;
    repeat (UART_TX_FULL_FRAME) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $error("pwr_up should remain high despite invalid commands");
      $stop;
    end
    $display("Mixed command sequence handled correctly");

    //--------------------------------------------------------------------
    // Verify clean shutdown sequence
    //--------------------------------------------------------------------
    $display("\nTesting clean shutdown sequence");
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (5000) @(posedge clk);
    
    ld_cell_lft  = 12'h000;
    ld_cell_rght = 12'h000;
    repeat (300000) @(posedge clk);
    
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $error("Clean shutdown failed - pwr_up should be low");
      $stop;
    end
    
    rider_lean = 16'sh0000;
    repeat (500_000) @(posedge clk);
    
    if (iPHYS.omega_lft > 100 || iPHYS.omega_rght > 100) begin
      $error("Wheels should be stopped after clean shutdown");
      $stop;
    end
    $display("Clean shutdown sequence verified");

    $display("\nAUTH + SEGWAY INTEGRATION TEST PASSED");
    $display("All authorization and integration tests completed successfully");
    $stop();
  end

  always #10 clk = ~clk;

endmodule
