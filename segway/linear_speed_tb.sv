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
      repeat (500000) @(posedge clk);

      // compute averages over 1000 cycles
      compute_average(.sig(iDUT.iBAL.lft_spd), .num_samples(1000), .clk(clk),
                      .avg_out(curr_lft_avg));
      compute_average(.sig(iDUT.iBAL.rght_spd), .num_samples(1000), .clk(clk),
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
