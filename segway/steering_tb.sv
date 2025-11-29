module steering_tb ();

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

  int lft_avg, rght_avg, diff_right, diff_left;

  initial begin

    /// Your magic goes here ///
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

    // Send 'G' command
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));

    ld_cell_lft  = 12'h300;  // simulate rider getting on
    ld_cell_rght = 12'h300;  // simulate rider getting on
    rider_lean   = 16'h0800; // forward lean for baseline speed
    batt         = 12'hFFF;  // full battery
    steerPot     = 12'h800;  // center steering
    repeat (150000) @(posedge clk);  // wait for system to stabilize

    //////////////////////////////////////////
    // Test: Center Steering Balance       //
    //////////////////////////////////////////
    compute_average(.sig(iDUT.lft_spd), .num_samples(256), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iDUT.rght_spd), .num_samples(256), .clk(clk), .avg_out(rght_avg));
    $display("Center steering - Left: %0d, Right: %0d, Diff: %0d", lft_avg, rght_avg,
             lft_avg - rght_avg);
    if ($abs(lft_avg - rght_avg) > 100) begin
      $display("Motor speeds not balanced at center.");
      $stop();
    end

    //////////////////////////////////////////
    // Test: Extreme Right Turn             //
    //////////////////////////////////////////
    steerPot = 12'hFFF;
    repeat (80000) @(posedge clk);
    compute_average(.sig(iDUT.lft_spd), .num_samples(128), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iDUT.rght_spd), .num_samples(128), .clk(clk), .avg_out(rght_avg));
    diff_right = lft_avg - rght_avg;
    $display("Right turn - Left: %0d, Right: %0d, Diff: %0d", lft_avg, rght_avg, diff_right);
    if (lft_avg <= rght_avg) begin
      $display("Left motor should be faster for right turn.");
      $stop();
    end

    //////////////////////////////////////////
    // Test: Extreme Left Turn              //
    //////////////////////////////////////////
    steerPot = 12'h000;
    repeat (80000) @(posedge clk);
    compute_average(.sig(iDUT.lft_spd), .num_samples(128), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iDUT.rght_spd), .num_samples(128), .clk(clk), .avg_out(rght_avg));
    diff_left = lft_avg - rght_avg;
    $display("Left turn - Left: %0d, Right: %0d, Diff: %0d", lft_avg, rght_avg, diff_left);
    if (rght_avg <= lft_avg) begin
      $display("Right motor should be faster for left turn.");
      $stop();
    end

    //////////////////////////////////////////
    // Test: Symmetry Check                 //
    //////////////////////////////////////////
    if ($abs(diff_right + diff_left) > 400) begin
      $display("Asymmetry detected: right_diff=%0d left_diff=%0d", diff_right, diff_left);
      $stop();
    end

    //////////////////////////////////////////
    // Test: Return to Center               //
    //////////////////////////////////////////
    steerPot = 12'h800;
    repeat (80000) @(posedge clk);
    compute_average(.sig(iDUT.lft_spd), .num_samples(256), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iDUT.rght_spd), .num_samples(256), .clk(clk), .avg_out(rght_avg));
    $display("Return to center - Left: %0d, Right: %0d, Diff: %0d", lft_avg, rght_avg,
             lft_avg - rght_avg);
    if ($abs(lft_avg - rght_avg) > 120) begin
      $display("Motors not balanced after return to center.");
      $stop();
    end

    //////////////////////////////////////////
    // Test: Rider Off Disables Steering    //
    //////////////////////////////////////////
    ld_cell_lft  = 12'h000;  // simulate rider stepping off
    ld_cell_rght = 12'h000;  // simulate rider stepping off
    repeat (60000) @(posedge clk);
    steerPot = 12'hF00;
    repeat (40000) @(posedge clk);
    compute_average(.sig(iDUT.lft_spd), .num_samples(64), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iDUT.rght_spd), .num_samples(64), .clk(clk), .avg_out(rght_avg));
    $display("Rider off - Left: %0d, Right: %0d", lft_avg, rght_avg);
    if (lft_avg != 0 || rght_avg != 0) begin
      $display("Speeds should be zero when rider is off.");
      $stop();
    end

    //////////////////////////////////////////
    // Test: Re-enable After Rider Returns  //
    //////////////////////////////////////////
    ld_cell_lft  = 12'h300;  // simulate rider getting back on
    ld_cell_rght = 12'h300;  // simulate rider getting back on
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
    repeat (100000) @(posedge clk);
    steerPot = 12'hE00;
    repeat (60000) @(posedge clk);
    compute_average(.sig(iDUT.lft_spd), .num_samples(128), .clk(clk), .avg_out(lft_avg));
    compute_average(.sig(iDUT.rght_spd), .num_samples(128), .clk(clk), .avg_out(rght_avg));
    $display("Re-enable right turn - Left: %0d, Right: %0d", lft_avg, rght_avg);
    if (lft_avg <= rght_avg) begin
      $display("Re-enable failed: left motor should be faster.");
      $stop();
    end

    $display("Steering test passed!");
    $stop();
  end

  always #10 clk = ~clk;

endmodule