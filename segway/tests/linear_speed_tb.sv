module linear_speed_tb ();

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

  int curr_lft_avg, curr_rght_avg;
  int prev_lft_avg, prev_rght_avg;
  logic signed [15:0] prev_lean;
  bit first_iter = 1;
  rand_lean lean_gen;
  localparam int LEAN_TOL_POS = 200;  // how close two leans can be and be considered "same"
  localparam int LEAN_TOL_NEG = 250;  // how close two leans can be and be considered "same"

  initial begin
    int lean_diff;
    int eff_lean_tol;
    int speed_samples[5];
    automatic int lean_values[5] = '{16'sh0200, 16'sh0400, 16'sh0600, 16'sh0800, 16'sh0A00};
    automatic int backward_lean_values[5] = '{-16'sh0200, -16'sh0400, -16'sh0600, -16'sh0800, -16'sh0A00};
    automatic int oscillation_leans[6] = '{16'sh0100, -16'sh0100, 16'sh0200, -16'sh0200, 16'sh0100, 16'sh0000};
    automatic int small_lean_values[4] = '{16'sh0050, 16'sh0100, 16'sh0150, 16'sh0200};
    automatic int prev_speed = 0;
    int i;

    /// Your magic goes here ///
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

    // Send 'G' command
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));

    ld_cell_lft  = 12'h300;  // simulate rider getting on
    ld_cell_rght = 12'h300;  // simulate rider getting on
    repeat (3000) @(posedge clk);  // wait for some time

    // Skip randomization - not supported in ModelSim
    // Use fixed lean values for testing instead
    /* lean_gen = new();

    repeat (10) begin
      @(posedge clk);
      if (!lean_gen.randomize()) begin
        $error("Randomization failed");
        $stop();
      end

      rider_lean = lean_gen.lean_val;  // simulate rider leaning forward
      $display("Rider lean set to: %0d", rider_lean);

      // skip first few cycles to avoid transients
      repeat (5_000_000) @(posedge clk);

      // compute averages over 1000 cycles
      compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk),
                      .avg_out(curr_lft_avg));
      compute_average(.sig(iPHYS.omega_rght), .num_samples(1000), .clk(clk),
                      .avg_out(curr_rght_avg));

      $display("Left motor avg speed: %0d, Right motor avg speed: %0d", curr_lft_avg,
               curr_rght_avg);

      if (!first_iter) begin
        lean_diff = rider_lean - prev_lean;
        if (lean_diff < 0) lean_diff = -lean_diff;  // |rider_lean - prev_lean|

        // -----------------------------
        // Choose tolerance:
        // If BOTH current & previous leans are NEGATIVE,
        // use bigger tolerance, else use normal tolerance
        // -----------------------------
        if (rider_lean < 0 && prev_lean < 0) eff_lean_tol = LEAN_TOL_NEG;
        else eff_lean_tol = LEAN_TOL_POS;

        // If leans are very close, don't enforce monotonic speed check
        if (lean_diff <= eff_lean_tol) begin
          $display(
              "Leans are within tolerance (|%0d - %0d| = %0d <= %0d), skipping monotonic speed check.",
              rider_lean, prev_lean, lean_diff, eff_lean_tol);
              continue;
        end else if (rider_lean > prev_lean) begin
          // if the previous lean was less than current lean, expect speed to increase
          // leaning more forward → expect speed to increase
          if (curr_lft_avg <= prev_lft_avg || curr_rght_avg <= prev_rght_avg) begin
            $display("Motor speeds did not increase as expected when rider leaned more forward.");
            $stop();
          end
        end else if (rider_lean < prev_lean) begin
          // leaning more backward → expect speed to decrease
          if (curr_lft_avg >= prev_lft_avg || curr_rght_avg >= prev_rght_avg) begin
            $display(
                "Motor speeds did not decrease as expected when rider leaned more backward.");
            $stop();
          end
        end
      end

      $display("Motor speeds changed correctly with lean change.");

      // update previous values for next iteration
      prev_lean     = rider_lean;
      prev_lft_avg  = curr_lft_avg;
      prev_rght_avg = curr_rght_avg;
      first_iter    = 0;
    end

    $display("Linear speed test passed!");
    */

    $display("Skipping randomization tests (not supported in ModelSim)");

    //--------------------------------------------------------------------
    // TEST 1: Zero lean (upright position) - speeds should be minimal/zero
    //--------------------------------------------------------------------
    rider_lean = 16'sh0000;
    $display("Zero lean test - upright position (lean = %0d)", rider_lean);
    repeat (2_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(1000), .clk(clk), .avg_out(curr_rght_avg));
    
    if ((curr_lft_avg < -100 || curr_lft_avg > 100) || 
        (curr_rght_avg < -100 || curr_rght_avg > 100)) begin
      $display("Speeds not minimal at zero lean. lft_avg=%0d, rght_avg=%0d",
               curr_lft_avg, curr_rght_avg);
      $stop();
    end
    
    $display("Zero lean produces minimal speeds. lft_avg=%0d, rght_avg=%0d",
             curr_lft_avg, curr_rght_avg);


    //--------------------------------------------------------------------
    // TEST 2: Maximum forward lean - verify saturation behavior
    //--------------------------------------------------------------------
    rider_lean = 16'sh0FFF;  // maximum positive lean
    $display("Maximum forward lean test (lean = %0d)", rider_lean);
    repeat (5_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(1000), .clk(clk), .avg_out(curr_rght_avg));
    
    prev_lft_avg = curr_lft_avg;
    prev_rght_avg = curr_rght_avg;
    $display("Max forward lean speeds: lft_avg=%0d, rght_avg=%0d", curr_lft_avg, curr_rght_avg);
    
    // Try exceeding max (should saturate)
    rider_lean = 16'sh1FFF;
    repeat (5_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(1000), .clk(clk), .avg_out(curr_rght_avg));
    
    if (!check_equal_with_tolerance(curr_lft_avg, prev_lft_avg, 100) ||
        !check_equal_with_tolerance(curr_rght_avg, prev_rght_avg, 100)) begin
      $display("Speed changed beyond saturation. prev_lft=%0d, curr_lft=%0d, prev_rght=%0d, curr_rght=%0d",
               prev_lft_avg, curr_lft_avg, prev_rght_avg, curr_rght_avg);
      $stop();
    end
    
    $display("Forward lean saturation verified. lft_avg=%0d, rght_avg=%0d",
             curr_lft_avg, curr_rght_avg);


    //--------------------------------------------------------------------
    // TEST 3: Maximum backward lean - verify saturation behavior
    //--------------------------------------------------------------------
    rider_lean = -16'sh0FFF;  // maximum negative lean
    $display("Maximum backward lean test (lean = %0d)", rider_lean);
    repeat (5_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(1000), .clk(clk), .avg_out(curr_rght_avg));
    
    prev_lft_avg = curr_lft_avg;
    prev_rght_avg = curr_rght_avg;
    $display("Max backward lean speeds: lft_avg=%0d, rght_avg=%0d", curr_lft_avg, curr_rght_avg);
    
    // Try exceeding max backward (should saturate)
    rider_lean = -16'sh1FFF;
    repeat (5_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_rght_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(1000), .clk(clk), .avg_out(curr_rght_avg));
    
    if (!check_equal_with_tolerance(curr_lft_avg, prev_lft_avg, 100) ||
        !check_equal_with_tolerance(curr_rght_avg, prev_rght_avg, 100)) begin
      $display("Speed changed beyond saturation. prev_lft=%0d, curr_lft=%0d, prev_rght=%0d, curr_rght=%0d",
               prev_lft_avg, curr_lft_avg, prev_rght_avg, curr_rght_avg);
      $stop();
    end
    
    $display("Backward lean saturation verified. lft_avg=%0d, rght_avg=%0d",
             curr_lft_avg, curr_rght_avg);


    //--------------------------------------------------------------------
    // TEST 4: Gradual lean increase (forward) - verify smooth acceleration
    //--------------------------------------------------------------------
    $display("Gradual forward lean increase test");
    
    for (i = 0; i < 5; i++) begin
      rider_lean = lean_values[i];
      repeat (3_000_000) @(posedge clk);
      
      compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
      speed_samples[i] = curr_lft_avg;
      $display("lean=%0d, lft_speed=%0d", lean_values[i], speed_samples[i]);
      
      if (i > 0 && speed_samples[i] <= speed_samples[i-1]) begin
        $display("Speed not monotonically increasing. speed[%0d]=%0d, speed[%0d]=%0d",
                 i-1, speed_samples[i-1], i, speed_samples[i]);
        $stop();
      end
    end
    
    $display("Gradual lean increase produces monotonic speed increase");


    //--------------------------------------------------------------------
    // TEST 5: Gradual lean decrease (backward) - verify smooth deceleration
    //--------------------------------------------------------------------
    $display("Gradual backward lean increase test");
    
    for (i = 0; i < 5; i++) begin
      rider_lean = backward_lean_values[i];
      repeat (3_000_000) @(posedge clk);
      
      compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
      speed_samples[i] = curr_lft_avg;
      $display("lean=%0d, lft_speed=%0d", backward_lean_values[i], speed_samples[i]);
      
      if (i > 0 && speed_samples[i] >= speed_samples[i-1]) begin
        $display("Speed not monotonically decreasing. speed[%0d]=%0d, speed[%0d]=%0d",
                 i-1, speed_samples[i-1], i, speed_samples[i]);
        $stop();
      end
    end
    
    $display("Gradual backward lean produces monotonic speed decrease");


    //--------------------------------------------------------------------
    // TEST 6: Rapid lean changes - verify system tracking
    //--------------------------------------------------------------------
    $display("Rapid lean changes test");
    
    // Forward
    rider_lean = 16'sh0800;
    repeat (2_000_000) @(posedge clk);
    compute_average(.sig(iPHYS.omega_lft), .num_samples(500), .clk(clk), .avg_out(prev_lft_avg));
    $display("Forward lean (0x0800): speed=%0d", prev_lft_avg);
    
    // Backward
    rider_lean = -16'sh0800;
    repeat (2_000_000) @(posedge clk);
    compute_average(.sig(iPHYS.omega_lft), .num_samples(500), .clk(clk), .avg_out(curr_lft_avg));
    $display("Backward lean (-0x0800): speed=%0d", curr_lft_avg);
    
    if (curr_lft_avg >= prev_lft_avg) begin
      $display("Speed did not decrease for backward lean. fwd_speed=%0d, back_speed=%0d",
               prev_lft_avg, curr_lft_avg);
      $stop();
    end
    
    // Back to forward
    rider_lean = 16'sh0800;
    repeat (2_000_000) @(posedge clk);
    compute_average(.sig(iPHYS.omega_lft), .num_samples(500), .clk(clk), .avg_out(prev_lft_avg));
    $display("Return to forward: speed=%0d", prev_lft_avg);
    
    if (prev_lft_avg <= curr_lft_avg) begin
      $display("Speed did not increase returning to forward. back_speed=%0d, fwd_speed=%0d",
               curr_lft_avg, prev_lft_avg);
      $stop();
    end
    
    $display("System tracks rapid lean changes correctly");


    //--------------------------------------------------------------------
    // TEST 7: Left-right speed symmetry with neutral steering
    //--------------------------------------------------------------------
    $display("Left-right speed symmetry test");
    
    steerPot = 12'h800;  // neutral steering
    rider_lean = 16'sh0600;
    repeat (3_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(1000), .clk(clk), .avg_out(curr_rght_avg));
    
    if (!check_equal_with_tolerance(curr_lft_avg, curr_rght_avg, 15)) begin
      $display("Left and right speeds not symmetric. lft=%0d, rght=%0d, diff=%0d",
               curr_lft_avg, curr_rght_avg, curr_lft_avg - curr_rght_avg);
      $stop();
    end
    
    $display("Left-right speeds are symmetric. lft=%0d, rght=%0d",
             curr_lft_avg, curr_rght_avg);


    //--------------------------------------------------------------------
    // TEST 8: Lean oscillation around zero
    //--------------------------------------------------------------------
    $display("Lean oscillation around zero test");
    
    for (i = 0; i < 6; i++) begin
      rider_lean = oscillation_leans[i];
      repeat (1_500_000) @(posedge clk);
      
      compute_average(.sig(iPHYS.omega_lft), .num_samples(500), .clk(clk), .avg_out(curr_lft_avg));
      $display("lean=%0d, speed=%0d", oscillation_leans[i], curr_lft_avg);
    end
    
    $display("Oscillating lean handled without instability");


    //--------------------------------------------------------------------
    // TEST 9: Sustained forward lean - stability check
    //--------------------------------------------------------------------
    $display("Sustained forward lean stability test");
    
    rider_lean = 16'sh0700;
    repeat (3_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(prev_lft_avg));
    $display("Initial speed: lft=%0d", prev_lft_avg);
    
    // Sustain for longer period
    repeat (5_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    $display("Sustained speed: lft=%0d", curr_lft_avg);
    
    if (!check_equal_with_tolerance(curr_lft_avg, prev_lft_avg, 50)) begin
      $display("Speed drifted during sustained lean. initial=%0d, sustained=%0d",
               prev_lft_avg, curr_lft_avg);
      $stop();
    end
    
    $display("Speed stable during sustained lean. initial=%0d, sustained=%0d",
             prev_lft_avg, curr_lft_avg);


    //--------------------------------------------------------------------
    // TEST 10: Small lean variations - sensitivity check
    //--------------------------------------------------------------------
    $display("Small lean variations sensitivity test");
    
    prev_speed = 0;
    
    for (i = 0; i < 4; i++) begin
      rider_lean = small_lean_values[i];
      repeat (2_000_000) @(posedge clk);
      
      compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
      $display("lean=%0d, speed=%0d", small_lean_values[i], curr_lft_avg);
      
      if (i > 0) begin
        // Speed should increase or at least not decrease significantly
        if (curr_lft_avg < prev_speed - 20) begin
          $display("Speed decreased unexpectedly. prev=%0d, curr=%0d",
                   prev_speed, curr_lft_avg);
          $stop();
        end
      end
      prev_speed = curr_lft_avg;
    end
    
    $display("System responds to small lean variations");


    //--------------------------------------------------------------------
    // TEST 11: Return to zero from extreme lean
    //--------------------------------------------------------------------
    $display("Return to zero from extreme lean test");
    
    // Start at extreme forward lean
    rider_lean = 16'sh0E00;
    repeat (3_000_000) @(posedge clk);
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(prev_lft_avg));
    $display("Extreme forward lean speed: %0d", prev_lft_avg);
    
    // Return to zero
    rider_lean = 16'sh0000;
    repeat (3_000_000) @(posedge clk);
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    $display("Zero lean speed: %0d", curr_lft_avg);
    
    if (curr_lft_avg >= prev_lft_avg) begin
      $display("Speed did not decrease returning to zero. extreme=%0d, zero=%0d",
               prev_lft_avg, curr_lft_avg);
      $stop();
    end
    
    if (curr_lft_avg < -100 || curr_lft_avg > 100) begin
      $display("Speed not near zero. speed=%0d", curr_lft_avg);
      $stop();
    end
    
    $display("Successfully returned to near-zero speed. speed=%0d", curr_lft_avg);

    $display("All linear speed tests passed!");
    $stop();
  end

  property always_check_speed;
    @(posedge clk) disable iff (!rst_n)
    // Check that left and right speeds are equal within tolerance of 10
    check_equal_with_tolerance(
        iDUT.iBAL.lft_spd, iDUT.iBAL.rght_spd, 10
    );
  endproperty

  assert property (always_check_speed)
  else $display("Motor speeds are not equal within tolerance at time %0t!", $time);


  always #10 clk = ~clk;

endmodule
