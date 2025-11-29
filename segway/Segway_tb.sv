module Segway_tb ();

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

  ///// Internal registers for testing purposes??? /////////


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

    /// Your magic goes here /// // use connect by name here 
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
             .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
             .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

    // Send 'G' command
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));

    repeat (3000) @(posedge clk);  // wait for some time
    ld_cell_lft  = 12'h300;  // simulate rider getting on
    ld_cell_rght = 12'h300;  // simulate rider getting on
    repeat (325000) @(posedge clk);  // wait for some time

    $display("=== Starting Lean Tests ===");
    $display("applying forward lean of 0FFFh (4095)");
    rider_lean = 16'h0FFF;  // simulate rider leaning forward
    check_theta_zero(.clk(clk), .ptch(iPHYS.theta_platform), .target_val(16'd0150), .tol(16'd0200));

    // ----------------------------------------------------------
    // 5) Abruptly remove lean (back to zero) and watch ring-down
    // ----------------------------------------------------------
    $display("removing lean to 0000h (0)");
    rider_lean = 16'sh0000;
    check_theta_zero(.clk(clk), .ptch(iPHYS.theta_platform), .target_val(16'd0150), .tol(16'd0200));

    // ----------------------------------------------------------
    // 6) Apply backward lean and watch response
    // ----------------------------------------------------------
    $display("applying backward lean of F000h (-4096)");
    rider_lean = 16'hF000;  // simulate rider leaning backward
    check_theta_zero(.clk(clk), .ptch(iPHYS.theta_platform), .target_val(16'd0150), .tol(16'd0200));
    // ----------------------------------------------------------
    // 7) Abruptly remove lean (back to zero) and watch ring-down
    // ----------------------------------------------------------
    $display("removing lean to 0000h (0)");
    rider_lean = 16'sh0000;
    check_theta_zero(.clk(clk), .ptch(iPHYS.theta_platform), .target_val(16'd0150), .tol(16'd0200));

    check_glitch_free_transitions(.clk(clk), .signal(rider_lean), .mon_sig(iPHYS.theta_platform),
                                  .values('{16'sh0FFF, 16'shF000, 16'sh0000}), .wait_cycles(5000),
                                  .tolerance(16'd0400));

    repeat (500000) @(posedge clk);  // wait for some time

    $display("=== Lean Tests Complete ===");
    $stop();
  end

  always #10 clk = ~clk;

endmodule
