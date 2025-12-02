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

  // Local variables for averaging and checks
  int lft_avg, rght_avg;

  initial begin

    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

    // ----------------------------------------------------
    // 1) Before authorization, lean should not drive power
    // ----------------------------------------------------
    $display("\n=== TEST 1: No power before authorization ===");
    rider_lean = 16'sh0700;  // light forward lean
    repeat (2000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $display("[FAIL] pwr_up should be low before 'G' command");
      $stop;
    end else begin
      $display("[PASS] pwr_up correctly low before authorization");
    end
    
    compute_average(.sig(iPHYS.omega_rght), .num_samples(100), .clk(clk), .avg_out(rght_avg));
    compute_average(.sig(iPHYS.omega_lft), .num_samples(100), .clk(clk), .avg_out(lft_avg));
    if (!check_equal_with_tolerance(rght_avg, 0, 50) || !check_equal_with_tolerance(lft_avg, 0, 50)) begin
      $display("[FAIL] Wheel speeds should remain zero before authorization (lft_avg=%0d, rght_avg=%0d)", lft_avg, rght_avg);
      $stop;
    end else begin
      $display("[PASS] Wheel speeds correctly zero before authorization (lft_avg=%0d, rght_avg=%0d)", lft_avg, rght_avg);
    end

    // ----------------------------------------------------
    // 2) Send 'G' to authorize and confirm power-up
    // ----------------------------------------------------
    $display("\n=== TEST 2: Authorization with 'G' command ===");
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
    $display("[PASS] Authorization granted, pwr_up asserted");

    // Give rider weight so rider_off deasserts
    ld_cell_lft  = 12'h350;
    ld_cell_rght = 12'h340;
    repeat (300000) @(posedge clk);
    if (iDUT.iSTR.rider_off !== 1'b0) begin
      $display("[FAIL] rider_off should deassert with rider weight");
      $stop;
    end
    $display("[PASS] Rider weight detected, rider_off deasserted");

    // ----------------------------------------------------
    // 3) With auth granted, lean should generate wheel motion
    // ----------------------------------------------------
    $display("\n=== TEST 3: Wheel response after authorization ===");
    rider_lean = 16'sh0A00;
    repeat (1_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_rght), .num_samples(256), .clk(clk), .avg_out(rght_avg));
    compute_average(.sig(iPHYS.omega_lft), .num_samples(256), .clk(clk), .avg_out(lft_avg));
    if (check_equal_with_tolerance(rght_avg, 0, 100) && check_equal_with_tolerance(lft_avg, 0, 100)) begin
      $display("[FAIL] Wheel speeds did not respond after authorization and lean (lft_avg=%0d, rght_avg=%0d)", lft_avg, rght_avg);
      $stop;
    end
    $display("[PASS] Wheel speeds responding correctly after authorization (lft_avg=%0d, rght_avg=%0d)", lft_avg, rght_avg);

    // ----------------------------------------------------
    // 4) Send 'S' (stop) then step off; pwr_up should drop
    // ----------------------------------------------------
    $display("\n=== TEST 4: Stop command and step-off sequence ===");
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
    repeat (5000) @(posedge clk);
    if (iDUT.iAuth.pwr_up !== 1'b1) begin
      $display("[FAIL] pwr_up should remain asserted immediately after 'S' while rider on");
      $stop;
    end
    $display("[PASS] 'S' command sent, pwr_up remains asserted while rider on");

    // Simulate rider stepping off after stop command
    ld_cell_lft  = 12'h000;
    ld_cell_rght = 12'h000;
    wait4sig(.sig(iDUT.iSTR.rider_off), .clks2wait(300000), .clk(clk));

    if (iDUT.iAuth.pwr_up !== 1'b0) begin
      $display("[FAIL] pwr_up should drop once rider steps off after 'S'");
      $stop;
    end
    $display("[PASS] Rider stepped off, pwr_up correctly deasserted");

    // ----------------------------------------------------
    // 5) With power off, leaning should not drive wheels
    // ----------------------------------------------------
    $display("\n=== TEST 5: No wheel motion when powered down ===");
    rider_lean = 16'sh0800;
    repeat (1_500_000) @(posedge clk);
    
    // Check that wheels settle to near-zero (allow some physics settling time)
    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_lft), .target_val(16'd0000), .tol(16'd200));
    check_theta_steady_state(.clk(clk), .ptch(iPHYS.omega_rght), .target_val(16'd0000), .tol(16'd200));
    
    compute_average(.sig(iPHYS.omega_rght), .num_samples(256), .clk(clk), .avg_out(rght_avg));
    compute_average(.sig(iPHYS.omega_lft), .num_samples(256), .clk(clk), .avg_out(lft_avg));
    if (!check_equal_with_tolerance(rght_avg, 0, 250) || !check_equal_with_tolerance(lft_avg, 0, 250)) begin
      $display("[FAIL] Wheel speeds should return to near zero after pwr_up deasserts (lft_avg=%0d, rght_avg=%0d)", lft_avg, rght_avg);
      $stop;
    end
    $display("[PASS] With pwr_up deasserted, wheel speeds at zero despite lean (lft_avg=%0d, rght_avg=%0d)", lft_avg, rght_avg);

    // ----------------------------------------------------
    // 6) Re-authorization test: Send 'G' again
    // ----------------------------------------------------
    $display("\n=== TEST 6: Re-authorization after stop ===");
    ld_cell_lft  = 12'h350;
    ld_cell_rght = 12'h340;
    repeat (10000) @(posedge clk);
    
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    wait4sig(.sig(iDUT.iAuth.pwr_up), .clks2wait(350000), .clk(clk));
    $display("[PASS] Re-authorization granted, pwr_up re-asserted");
    
    rider_lean = 16'sh0C00;
    repeat (1_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_rght), .num_samples(256), .clk(clk), .avg_out(rght_avg));
    compute_average(.sig(iPHYS.omega_lft), .num_samples(256), .clk(clk), .avg_out(lft_avg));
    if (check_equal_with_tolerance(rght_avg, 0, 100) && check_equal_with_tolerance(lft_avg, 0, 100)) begin
      $display("[FAIL] Wheel speeds should respond after re-authorization (lft_avg=%0d, rght_avg=%0d)", lft_avg, rght_avg);
      $stop;
    end
    $display("[PASS] Wheels responding after re-authorization (lft_avg=%0d, rght_avg=%0d)", lft_avg, rght_avg);

    $display("\n=== AUTH + SEGWAY INTEGRATION TEST PASSED ===");
    $stop();
  end

  always #10 clk = ~clk;

endmodule
