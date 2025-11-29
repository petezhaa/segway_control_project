// simultaneous_faults_tb.sv
// Testbench for simultaneous faults: low battery, overcurrent, and sensor noise



// -----------------------------------------------------------------------------
// Testbench: simultaneous_faults_tb
// Purpose:   Test Segway system response to simultaneous faults:
//            - Low battery
//            - Persistent overcurrent
//            - Sensor noise
// Structure: Matches style of Segway_tb.sv with clear section comments,
//            stepwise stimulus, and self-checks.
// -----------------------------------------------------------------------------
module simultaneous_faults_tb ();

  import task_pkg::*;

  //// Interconnects to DUT/support defined as type wire /////

  // --- Interconnect wires to DUT and support modules ---
  wire SS_n, SCLK, MOSI, MISO, INT;                // Inertial sensor
  wire A2D_SS_n, A2D_SCLK, A2D_MOSI, A2D_MISO;     // A2D converter
  wire RX_TX;                                      // UART
  wire PWM1_rght, PWM2_rght, PWM1_lft, PWM2_lft;   // Motor drivers
  wire piezo, piezo_n;                             // Piezo buzzer
  wire cmd_sent;                                   // UART TX done
  wire rst_n;                                      // Synchronized global reset

  ////// Stimulus is declared as type reg ///////

  // --- Stimulus registers ---
  reg clk, RST_n;                                  // Clock and async reset
  reg [7:0] cmd;                                   // Command to DUT
  reg send_cmd;                                    // Command strobe
  reg signed [15:0] rider_lean;                    // Simulated rider lean
  reg [11:0] ld_cell_lft, ld_cell_rght;            // Load cell A2D values
  reg [11:0] steerPot;                             // Steering potentiometer
  reg [11:0] batt;                                 // Battery A2D value
  reg OVR_I_lft, OVR_I_rght;                       // Overcurrent flags

  ////////////////////////////////////////////////////////////////
  // Instantiate Physical Model of Segway with Inertial sensor //
  //////////////////////////////////////////////////////////////
  // --- Instantiate Physical Model of Segway with Inertial sensor ---
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
  // --- Instantiate Model of A2D for load cell and battery ---
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
  // --- Instantiate Device Under Test (DUT) ---
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
  // --- Instantiate UART transmitter (mimics BLE module) ---
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
  // --- Instantiate reset synchronizer ---
  rst_synch iRST (
    .clk  (clk),
    .RST_n(RST_n),
    .rst_n(rst_n)
  );

  // Task for injecting sensor noise

  // -------------------------------------------------------------------------
  // Task: inject_sensor_noise
  // Purpose: Add random noise to rider_lean for a number of cycles
  // Args:
  //   cycles    - number of clock cycles to inject noise
  //   amplitude - max absolute value of noise to add/subtract
  // -------------------------------------------------------------------------
  task automatic inject_sensor_noise(input int cycles, input int amplitude);
    int i;
    for (i = 0; i < cycles; i++) begin
      @(posedge clk);
      // Add random signed noise to rider_lean
      rider_lean = rider_lean + $urandom_range(-amplitude, amplitude);
    end
  endtask


  initial begin
    // ---------------------------------------------------------------------
    // Initialization and Power-Up
    // ---------------------------------------------------------------------
    $display("=== Simultaneous Faults Test: Initialization ===");
    // Initialize all signals and connect testbench to DUT
    init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
         .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
         .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

    // Simulate power-up and rider mounting
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G)); // Send 'GO' command
    repeat (3000) @(posedge clk); // Small delay for command
    ld_cell_lft  = 12'h300;      // Rider steps on (left)
    ld_cell_rght = 12'h300;      // Rider steps on (right)
    rider_lean   = 16'sh0100;    // Small forward lean
    steerPot     = 12'h800;      // Center steering
    batt         = 12'hA00;      // Normal battery voltage
    OVR_I_lft    = 0;            // No overcurrent
    OVR_I_rght   = 0;            // No overcurrent
    repeat (200000) @(posedge clk); // Allow system to stabilize

    // Check: System powered up with rider present
    if (!iDUT.pwr_up) begin
      $display("FAIL: pwr_up not asserted after GO with rider on");
      $stop();
    end

    // ---------------------------------------------------------------------
    // Inject Simultaneous Faults
    // ---------------------------------------------------------------------
    $display("=== Injecting Simultaneous Faults ===");
    // 1. Battery drops to critical (simulate low battery)
    batt = 12'h350;
    // 2. Persistent overcurrent on right motor driver
    OVR_I_rght = 1;
    // 3. Sensor noise on rider_lean (simulate IMU noise)
    inject_sensor_noise(1000, 200); // 1000 cycles, +/-200 amplitude
    repeat (500000) @(posedge clk); // Let faults propagate

    // Check: System should assert batt_low and OVR_I_shtdwn
    if (!iDUT.batt_low) begin
      $display("FAIL: batt_low not asserted during simultaneous faults");
      $stop();
    end
    if (!iDUT.iDRV.OVR_I_shtdwn) begin
      $display("FAIL: OVR_I_shtdwn not asserted during simultaneous faults");
      $stop();
    end

    // ---------------------------------------------------------------------
    // Fault Recovery
    // ---------------------------------------------------------------------
    $display("=== Fault Recovery ===");
    // Remove faults and check system recovers
    batt = 12'h900;      // Battery recovers
    OVR_I_rght = 0;      // Overcurrent clears
    repeat (100000) @(posedge clk); // Wait for system to respond
    if (iDUT.batt_low) begin
      $display("FAIL: batt_low did not clear after battery recovery");
      $stop();
    end
    if (iDUT.iDRV.OVR_I_shtdwn) begin
      $display("FAIL: OVR_I_shtdwn did not clear after overcurrent recovery");
      $stop();
    end

    // ---------------------------------------------------------------------
    // Test Complete
    // ---------------------------------------------------------------------
    $display("Simultaneous faults test PASSED!");
    #100;
    $stop();
  end


  // --- Clock generation ---
  always #10 clk = ~clk;

endmodule
