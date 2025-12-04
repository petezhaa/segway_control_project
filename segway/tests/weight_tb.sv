module weight_tb ();

  import task_pkg::*;
  //import rand_pkg::*;

  //============================================================
  // Testbench/DUT interconnects (driven/observed as wires)
  //============================================================
  wire SS_n, SCLK, MOSI, MISO, INT;              // SPI to inertial sensor
  wire A2D_SS_n, A2D_SCLK, A2D_MOSI, A2D_MISO;   // SPI to A2D converter
  wire RX_TX;                                    // UART connection: TX from BLE model to DUT RX
  wire PWM1_rght, PWM2_rght, PWM1_lft, PWM2_lft; // Motor drive PWM outputs
  wire piezo, piezo_n;                           // Piezo driver outputs
  wire cmd_sent;                                 // UART_tx done flag
  wire rst_n;                                    // Synchronized, active-low reset to DUT

  //============================================================
  // Testbench stimulus (regs driven by procedural blocks)
  //============================================================
  reg clk, RST_n;                                // clk: TB free-running clock, RST_n: async reset into synchronizer
  reg [7:0] cmd;                                 // Command byte sent from host/BLE model into DUT
  reg send_cmd;                                  // Pulse high to trigger a UART transmit
  reg signed [15:0] rider_lean;                  // Rider lean angle into physical model
  reg [11:0] ld_cell_lft, ld_cell_rght;          // Load cell raw A2D values (left/right)
  reg [11:0] steerPot, batt;                     // Steering potentiometer and battery A2D values
  reg OVR_I_lft, OVR_I_rght;                     // Over-current flags for left/right motor channels

  // (No additional TB-only registers needed here; all other
  //  TB state is in local variables and the randomization class.)

  ////////////////////////////////////////////////////////////////
  // Instantiate physical Segway + inertial sensor behavioral model
  ////////////////////////////////////////////////////////////////
  SegwayModel iPHYS (
      .clk       (clk),
      .RST_n     (RST_n),
      .SS_n      (SS_n),
      .SCLK      (SCLK),
      .MISO      (MISO),
      .MOSI      (MOSI),
      .INT       (INT),
      .PWM1_lft  (PWM1_lft),
      .PWM2_lft  (PWM2_lft),
      .PWM1_rght (PWM1_rght),
      .PWM2_rght (PWM2_rght),
      .rider_lean(rider_lean)
  );

  /////////////////////////////////////////////////////////
  // Instantiate A2D model for load cells, steering, battery
  /////////////////////////////////////////////////////////
  ADC128S_FC iA2D (
      .clk         (clk),
      .rst_n       (RST_n),
      .SS_n        (A2D_SS_n),
      .SCLK        (A2D_SCLK),
      .MISO        (A2D_MISO),
      .MOSI        (A2D_MOSI),
      .ld_cell_lft (ld_cell_lft),
      .ld_cell_rght(ld_cell_rght),
      .steerPot    (steerPot),
      .batt        (batt)
  );

  ////////////////////////////////
  // Instantiate DUT (Segway)
  ////////////////////////////////
  Segway iDUT (
      .clk        (clk),
      .RST_n      (RST_n),
      .INERT_SS_n (SS_n),
      .INERT_MOSI (MOSI),
      .INERT_SCLK (SCLK),
      .INERT_MISO (MISO),
      .INERT_INT  (INT),
      .A2D_SS_n   (A2D_SS_n),
      .A2D_MOSI   (A2D_MOSI),
      .A2D_SCLK   (A2D_SCLK),
      .A2D_MISO   (A2D_MISO),
      .PWM1_lft   (PWM1_lft),
      .PWM2_lft   (PWM2_lft),
      .PWM1_rght  (PWM1_rght),
      .PWM2_rght  (PWM2_rght),
      .OVR_I_lft  (OVR_I_lft),
      .OVR_I_rght (OVR_I_rght),
      .piezo_n    (piezo_n),
      .piezo      (piezo),
      .RX         (RX_TX)
  );

  ///////////////////////////////////////////////////////
  // UART_tx instance (mimics BLE module sending command)
  ///////////////////////////////////////////////////////
  UART_tx iTX (
      .clk    (clk),
      .rst_n  (rst_n),
      .TX     (RX_TX),
      .trmt   (send_cmd),
      .tx_data(cmd),
      .tx_done(cmd_sent)
  );

  /////////////////////////////////////
  // Reset synchronizer into DUT
  /////////////////////////////////////
  rst_synch iRST (
      .clk  (clk),
      .RST_n(RST_n),   // async reset from TB
      .rst_n(rst_n)    // synchronized reset to UART/DUT
  );

  //============================================================
  // Random stimulus + helper signals for weight/steering checks
  //============================================================
  segway_rand rand_inst;                // Constrained random generator for load cells, etc.

  logic sum_gt_min, sum_lt_min;         // Sum above/below rider presence threshold (with hysteresis)
  logic [13:0] sum;                     // Total rider weight (left + right)
  logic signed [12:0] diff;             // Signed difference between left and right load cells
  logic [12:0] abs_diff;                // Absolute value of diff
  logic diff_gt_1_4, diff_gt_15_16;     // Balance metrics: > 1/4 sum, > 15/16 sum

  logic rider_off, en_steer;            // Internal DUT flags: rider detected off, steering enabled
  assign rider_off = iDUT.rider_off;
  assign en_steer  = iDUT.en_steer;

  logic vld_data;                       // Valid load-cell sample from DUT A2D interface
  assign vld_data = iDUT.vld;

  // Shadow copies of TB-driven load-cell values (for comparison with DUT internal values)
  logic [11:0] ld_cell_lft_reg, ld_cell_rght_reg;
  assign ld_cell_lft_reg  = ld_cell_lft;
  assign ld_cell_rght_reg = ld_cell_rght;

  // Probes of DUT’s internal registered load-cell values
  logic [11:0] ld_cell_lft_reg_DUT, ld_cell_rght_reg_DUT;
  assign ld_cell_lft_reg_DUT  = iDUT.iSTR.lft_ld;
  assign ld_cell_rght_reg_DUT = iDUT.iSTR.rght_ld;

  //============================================================
  // Main randomized test sequence
  //============================================================
  initial begin

    rand_inst = new();

    // ---------------------------------------------------------
    // Initialize DUT and all TB stimulus signals
    // (uses task from task_pkg; connect-by-name for clarity)
    // ---------------------------------------------------------
    init_DUT(
      .clk         (clk),
      .RST_n       (RST_n),
      .send_cmd    (send_cmd),
      .cmd         (cmd),
      .rider_lean  (rider_lean),
      .ld_cell_lft (ld_cell_lft),
      .ld_cell_rght(ld_cell_rght),
      .steerPot    (steerPot),
      .batt        (batt),
      .OVR_I_lft   (OVR_I_lft),
      .OVR_I_rght  (OVR_I_rght)
    );

    // Example of fixed stimulus (kept here for debug/reference):
    // rider_lean   = 16'h0fec;
    // ld_cell_lft  = 12'h6c6;
    // ld_cell_rght = 12'he27;
    // steerPot     = 12'h42b;
    // batt         = 12'hda8;

    // ---------------------------------------------------------
    // Send 'G' (Go) command to put DUT into normal operating mode
    // ---------------------------------------------------------
    SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));

    // ---------------------------------------------------------
    // Main random-weight loop:
    //  - Randomize left/right load cells
    //  - Check:
    //      * rider_off behavior vs sum_lt_min/sum_gt_min
    //      * steer enable behavior vs balance (abs_diff)
    //  - Use helper tasks to wait for internal DUT signals
    // ---------------------------------------------------------
    repeat (1000) begin
      if (!rand_inst.randomize()) begin
        $display("Randomization failed");
        $stop();
      end

      ld_cell_lft  = rand_inst.ld_cell_lft;
      ld_cell_rght = rand_inst.ld_cell_rght;

      $display("Testing with ld_cell_lft: %h, ld_cell_rght: %h ",
               ld_cell_lft, ld_cell_rght);

      // Compute sum and balance metrics based on randomized weights
      sum      = ld_cell_lft + ld_cell_rght;
      diff     = ld_cell_lft - ld_cell_rght;
      abs_diff = (diff[12]) ? -diff : diff;

      sum_gt_min  = (sum > (MIN_RIDER_WEIGHT + WT_HYSTERESIS));
      sum_lt_min  = (sum < (MIN_RIDER_WEIGHT - WT_HYSTERESIS));
      diff_gt_1_4 = (abs_diff > (sum / 4));
      diff_gt_15_16 = (abs_diff > (sum * 15 / 16));

      // First, ensure A2D interface and steer_en SM see updated weights
      wait4sig_eq(
        .clk      (clk),
        .sigA     (ld_cell_lft_reg),
        .sigB     (ld_cell_lft_reg_DUT),
        .clks2wait(50000)
      );
      wait4sig_eq(
        .clk      (clk),
        .sigA     (ld_cell_rght_reg),
        .sigB     (ld_cell_rght_reg_DUT),
        .clks2wait(50000)
      );

      // ---------------- Rider OFF case: weight below threshold ----------------
      if (sum_lt_min) begin
        $display("Rider off board: sum = %h", sum);

        // Expect rider_off to assert and steering to be disabled
        wait4sig(.clk(clk), .sig(rider_off), .clks2wait(50000));
        $display("Rider off confirmed when sum_lt_min = %h", sum);

        wait4sig_low(.clk(clk), .sig(en_steer), .clks2wait(50000));
        $display("Steering disabled confirmed when sum_lt_min = %h", sum);

      // ---------------- Rider ON case: weight above threshold -----------------
      end else if (sum_gt_min) begin
        $display("Rider on board: sum = %h", sum);

        // Expect rider_off to be cleared when rider is clearly on board
        wait4sig_low(.clk(clk), .sig(rider_off), .clks2wait(50000));
        $display("Rider off deassert confirmed when sum_gt_min = %h", sum);

        // Rider stepping off rapidly: huge imbalance while in "balanced" state
        if (diff_gt_15_16 && (iDUT.iSTR.u_steer_en_SM.state === 2'b10)) begin
          $display("Rider stepping off: abs_diff = %h", abs_diff);

          wait4sig_low(.clk(clk), .sig(en_steer), .clks2wait(50000));
          $display("Steering disabled confirmed when diff_gt_15_16 = %h",
                   abs_diff);

          wait4sig_low(.clk(clk), .sig(rider_off), .clks2wait(50000));
          $display("Rider off confirmed when diff_gt_15_16 = %h", abs_diff);

        // Rider reasonably balanced while in "balanced" state
        end else if (iDUT.iSTR.u_steer_en_SM.state === 2'b10) begin
          $display("Rider balanced: abs_diff = %h", abs_diff);

          wait4sig(.clk(clk), .sig(en_steer), .clks2wait(50000));
          $display("Steering enabled confirmed when rider balanced, abs_diff = %h",
                   abs_diff);

        // Rider unbalanced while in "pre-steer-enable" / intermediate state
        end else if (diff_gt_1_4 &&
                     (iDUT.iSTR.u_steer_en_SM.state === 2'b01)) begin
          $display("Rider not balanced: abs_diff = %h", abs_diff);

          wait4sig_low(.clk(clk), .sig(en_steer), .clks2wait(50000));
          $display("Steering disabled confirmed when diff_gt_1_4 = %h",
                   abs_diff);

          wait4sig_low(.clk(clk), .sig(rider_off), .clks2wait(50000));
          $display("Rider off confirmed when diff_gt_1_4 = %h", abs_diff);
        end
      end

    end // repeat(1000)

    $display("Random weight/steering test PASSED");
    $stop();
  end

  //============================================================
  // Clock generation: 50 MHz equivalent (20 ns period)
  //============================================================
  always #10 clk = ~clk;

endmodule
