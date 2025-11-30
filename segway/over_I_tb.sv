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
    repeat (500000) @(posedge clk);  // wait for balance loop to reach a steady state

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
    repeat (500000) @(posedge clk);  // wait for balance loop to reach a steady state
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


    $display("\n=== Over-Current Tests Complete ===");
    $stop();
  end



  always #10 clk = ~clk;

endmodule
