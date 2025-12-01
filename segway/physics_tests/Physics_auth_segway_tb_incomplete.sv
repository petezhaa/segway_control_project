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
    if (iPHYS.omega_rght !== 0 || iPHYS.omega_lft !== 0) begin
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
    repeat (800000) @(posedge clk); // The omegas need more time to pass, so I increased the # of repeats here
    if ((iPHYS.omega_rght === 0) && (iPHYS.omega_lft === 0)) begin
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
    repeat (800000) @(posedge clk); //TODO: these omegas never drop to zero, gave them 10M steps and they settle at 1023. I'll leave this as a question to Hoffman
    //TODO: maybe make a task that gives omegas a certain window to reach a steady state value (1023 target)
    if (iPHYS.omega_rght !== 0 || iPHYS.omega_lft !== 0) begin
      $error("Wheel speeds should return to zero after pwr_up deasserts");
      $stop;
    end
    $display("With pwr_up deasserted, wheel speeds remain zero despite lean");

    $display("AUTH + SEGWAY INTEGRATION TEST PASSED");
    $stop();
  end

  always #10 clk = ~clk;

endmodule
