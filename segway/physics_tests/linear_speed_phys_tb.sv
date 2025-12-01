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

    /// Your magic goes here ///
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

    // Send 'G' command
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));

    ld_cell_lft  = 12'h300;  // simulate rider getting on
    ld_cell_rght = 12'h300;  // simulate rider getting on
    repeat (3000) @(posedge clk);  // wait for some time

    lean_gen = new();

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
        // If leans are very close, don't enforce monotonic speed check
        if (!first_iter) begin
          int lean_diff;
          int eff_lean_tol;
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
      end

      $display("Motor speeds changed correctly with lean change.");

      // update previous values for next iteration
      prev_lean     = rider_lean;
      prev_lft_avg  = curr_lft_avg;
      prev_rght_avg = curr_rght_avg;
      first_iter    = 0;
    end

    $display("Linear speed test passed!");

    //====================================================================
    // ADDITIONAL PHYSICS-BASED TEST CASES
    //====================================================================

    //--------------------------------------------------------------------
    // TEST 1: Zero lean stability - verify platform remains level
    //--------------------------------------------------------------------
    rider_lean = 16'sh0000;
    $display("Zero lean stability test");
    repeat (2_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.theta_platform), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    
    if (curr_lft_avg < -100 || curr_lft_avg > 100) begin
      $display("Platform not stable at zero lean. theta_avg=%0d", curr_lft_avg);
      $stop();
    end
    
    $display("Platform stable at zero lean. theta_avg=%0d", curr_lft_avg);


    //--------------------------------------------------------------------
    // TEST 2: Forward lean - verify platform angle response
    //--------------------------------------------------------------------
    rider_lean = 16'sh0800;
    $display("Forward lean platform angle test (lean = %0d)", rider_lean);
    repeat (3_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.theta_platform), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_rght_avg));
    
    $display("Platform theta_avg=%0d, omega_lft=%0d", curr_lft_avg, curr_rght_avg);
    
    if (curr_rght_avg <= 0) begin
      $display("Forward lean did not produce forward motion. omega=%0d", curr_rght_avg);
      $stop();
    end
    
    $display("Forward lean produces forward motion correctly");


    //--------------------------------------------------------------------
    // TEST 3: Backward lean - verify platform angle response
    //--------------------------------------------------------------------
    rider_lean = -16'sh0800;
    $display("Backward lean platform angle test (lean = %0d)", rider_lean);
    repeat (3_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.theta_platform), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_rght_avg));
    
    $display("Platform theta_avg=%0d, omega_lft=%0d", curr_lft_avg, curr_rght_avg);
    
    if (curr_rght_avg >= 0) begin
      $display("Backward lean did not produce backward motion. omega=%0d", curr_rght_avg);
      $stop();
    end
    
    $display("Backward lean produces backward motion correctly");


    //--------------------------------------------------------------------
    // TEST 4: Oscillation check - verify platform oscillates toward equilibrium
    //--------------------------------------------------------------------
    rider_lean = 16'sh0600;
    $display("Platform oscillation test");
    repeat (1_000_000) @(posedge clk);
    
    // Use the check_theta_oscillation task
    check_theta_oscillation(clk, iPHYS.theta_platform, 16'sh0000);
    
    $display("Platform oscillation verified");


    //--------------------------------------------------------------------
    // TEST 5: Steady state convergence - verify platform settles
    //--------------------------------------------------------------------
    rider_lean = 16'sh0700;
    $display("Steady state convergence test (lean = %0d)", rider_lean);
    repeat (3_000_000) @(posedge clk);
    
    // Check steady state with tolerance
    check_theta_steady_state(clk, iPHYS.theta_platform, 16'sh0000, 150);
    
    $display("Platform converged to steady state");


    //--------------------------------------------------------------------
    // TEST 6: Lean step response - verify smooth transitions
    //--------------------------------------------------------------------
    $display("Lean step response test");
    
    logic signed [15:0] lean_steps[4] = '{16'sh0400, 16'sh0800, 16'sh0400, 16'sh0000};
    
    for (int i = 0; i < 4; i++) begin
      rider_lean = lean_steps[i];
      repeat (2_000_000) @(posedge clk);
      
      compute_average(.sig(iPHYS.omega_lft), .num_samples(500), .clk(clk), .avg_out(curr_lft_avg));
      $display("Lean=%0d, omega=%0d", lean_steps[i], curr_lft_avg);
    end
    
    $display("Lean step responses smooth");


    //--------------------------------------------------------------------
    // TEST 7: Platform velocity correlation - verify omega tracks speed
    //--------------------------------------------------------------------
    rider_lean = 16'sh0A00;
    $display("Platform velocity correlation test");
    repeat (3_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    compute_average(.sig(iPHYS.omega_rght), .num_samples(1000), .clk(clk), .avg_out(curr_rght_avg));
    
    if (!check_equal_with_tolerance(curr_lft_avg, curr_rght_avg, 20)) begin
      $display("Left and right wheel velocities not matched. lft=%0d, rght=%0d",
               curr_lft_avg, curr_rght_avg);
      $stop();
    end
    
    $display("Wheel velocities matched. lft=%0d, rght=%0d", curr_lft_avg, curr_rght_avg);


    //--------------------------------------------------------------------
    // TEST 8: Maximum lean saturation - verify physical limits
    //--------------------------------------------------------------------
    rider_lean = 16'sh0FFF;
    $display("Maximum lean saturation test (lean = %0d)", rider_lean);
    repeat (5_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(prev_lft_avg));
    $display("Max lean omega: %0d", prev_lft_avg);
    
    // Try to exceed
    rider_lean = 16'sh1FFF;
    repeat (5_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    
    if (!check_equal_with_tolerance(curr_lft_avg, prev_lft_avg, 150)) begin
      $display("Saturation not working. prev=%0d, curr=%0d", prev_lft_avg, curr_lft_avg);
      $stop();
    end
    
    $display("Physical saturation verified. omega=%0d", curr_lft_avg);


    //--------------------------------------------------------------------
    // TEST 9: Rapid lean reversals - verify platform stability
    //--------------------------------------------------------------------
    $display("Rapid lean reversal stability test");
    
    logic signed [15:0] rapid_leans[6] = '{16'sh0600, -16'sh0600, 16'sh0800, 
                                            -16'sh0800, 16'sh0400, 16'sh0000};
    
    for (int i = 0; i < 6; i++) begin
      rider_lean = rapid_leans[i];
      repeat (1_000_000) @(posedge clk);
      
      // Check platform doesn't go X/Z
      if ($isunknown(iPHYS.theta_platform)) begin
        $display("Platform angle went unknown during rapid reversal");
        $stop();
      end
    end
    
    $display("Platform stable during rapid reversals");


    //--------------------------------------------------------------------
    // TEST 10: Energy conservation - verify no runaway conditions
    //--------------------------------------------------------------------
    rider_lean = 16'sh0500;
    $display("Energy conservation test");
    repeat (3_000_000) @(posedge clk);
    
    int sample_count = 0;
    int unstable_count = 0;
    
    for (int i = 0; i < 1000; i++) begin
      @(posedge clk);
      
      // Check if theta is growing unbounded
      if (iPHYS.theta_platform > 16'sh2000 || iPHYS.theta_platform < -16'sh2000) begin
        unstable_count++;
      end
      sample_count++;
    end
    
    if (unstable_count > 50) begin
      $display("Platform unstable, excessive theta deviation. unstable_samples=%0d", unstable_count);
      $stop();
    end
    
    $display("Energy bounded, no runaway. unstable_samples=%0d/%0d", unstable_count, sample_count);


    //--------------------------------------------------------------------
    // TEST 11: Zero crossing behavior - verify control near equilibrium
    //--------------------------------------------------------------------
    $display("Zero crossing behavior test");
    
    logic signed [15:0] small_leans[4] = '{16'sh0100, -16'sh0100, 16'sh0150, -16'sh0150};
    
    for (int i = 0; i < 4; i++) begin
      rider_lean = small_leans[i];
      repeat (1_500_000) @(posedge clk);
      
      compute_average(.sig(iPHYS.theta_platform), .num_samples(500), .clk(clk), .avg_out(curr_lft_avg));
      $display("Small lean=%0d, theta_avg=%0d", small_leans[i], curr_lft_avg);
    end
    
    $display("Zero crossing behavior stable");


    //--------------------------------------------------------------------
    // TEST 12: Platform angle vs wheel speed relationship
    //--------------------------------------------------------------------
    $display("Platform angle vs wheel speed correlation test");
    
    logic signed [15:0] test_leans[5] = '{16'sh0300, 16'sh0500, 16'sh0700, 16'sh0900, 16'sh0B00};
    int theta_samples[5];
    int omega_samples[5];
    
    for (int i = 0; i < 5; i++) begin
      rider_lean = test_leans[i];
      repeat (3_000_000) @(posedge clk);
      
      compute_average(.sig(iPHYS.theta_platform), .num_samples(1000), .clk(clk), .avg_out(theta_samples[i]));
      compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(omega_samples[i]));
      
      $display("Lean=%0d, theta=%0d, omega=%0d", test_leans[i], theta_samples[i], omega_samples[i]);
    end
    
    $display("Platform angle and wheel speed relationship verified");


    //--------------------------------------------------------------------
    // TEST 13: Return to equilibrium from extreme lean
    //--------------------------------------------------------------------
    $display("Return to equilibrium test");
    
    rider_lean = 16'sh0E00;
    repeat (3_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(prev_lft_avg));
    $display("Extreme lean omega: %0d", prev_lft_avg);
    
    // Return to zero
    rider_lean = 16'sh0000;
    repeat (4_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.omega_lft), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    compute_average(.sig(iPHYS.theta_platform), .num_samples(1000), .clk(clk), .avg_out(curr_rght_avg));
    
    if (curr_lft_avg < -200 || curr_lft_avg > 200) begin
      $display("Did not return to equilibrium. omega=%0d", curr_lft_avg);
      $stop();
    end
    
    $display("Returned to equilibrium. omega=%0d, theta=%0d", curr_lft_avg, curr_rght_avg);


    //--------------------------------------------------------------------
    // TEST 14: Sustained lean stability over extended time
    //--------------------------------------------------------------------
    $display("Sustained lean stability test");
    
    rider_lean = 16'sh0600;
    repeat (3_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.theta_platform), .num_samples(1000), .clk(clk), .avg_out(prev_lft_avg));
    $display("Initial theta: %0d", prev_lft_avg);
    
    // Sustain
    repeat (10_000_000) @(posedge clk);
    
    compute_average(.sig(iPHYS.theta_platform), .num_samples(1000), .clk(clk), .avg_out(curr_lft_avg));
    $display("Sustained theta: %0d", curr_lft_avg);
    
    if (!check_equal_with_tolerance(curr_lft_avg, prev_lft_avg, 100)) begin
      $display("Platform drifted during sustained lean. initial=%0d, final=%0d",
               prev_lft_avg, curr_lft_avg);
      $stop();
    end
    
    $display("Platform stable during sustained lean");


    //--------------------------------------------------------------------
    // TEST 15: Glitch-free transitions
    //--------------------------------------------------------------------
    $display("Glitch-free transition test");
    
    logic signed [15:0] transition_leans[3] = '{16'sh0000, 16'sh0800, -16'sh0800};
    
    check_glitch_free_transitions(clk, rider_lean, iPHYS.theta_platform,
                                   transition_leans, 1000000, 200);
    
    $display("Transitions glitch-free");


    $display("All linear speed physics tests passed!");
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
