package task_pkg;

  // ------------------------------------------------------------------
  // UART command constants
  // ------------------------------------------------------------------
  // ASCII 'G' and 'S' used as high-level command bytes in the UART
  // protocol (e.g., "Go", "Stop", or whatever your top-level defines).
  localparam G = 8'h47;
  localparam S = 8'h53;

  // Number of 50 MHz clock cycles required to transmit a *full* UART frame
  // (start bit + 8 data bits + stop bits, etc.) at 19,200 baud.
  // Used to wait long enough for one complete TX frame to leave the UART.
  localparam UART_TX_FULL_FRAME = 52080;

  // ------------------------------------------------------------------
  // Task: init_DUT
  // Purpose:
  //   Bring the DUT into a known, safe reset state and then release
  //   reset after a few clock cycles. Also initializes all key
  //   stimulus/control signals to benign defaults.
  //
  // Notes:
  //   - Keeps rider_lean at 0 (upright, no lean).
  //   - Centers steerPot around mid-scale (neutral steering).
  //   - Initializes battery to a healthy value (C00) and clears
  //     over-current flags.
  // ------------------------------------------------------------------
  task automatic init_DUT(ref logic clk, ref logic RST_n, ref logic send_cmd, ref logic [7:0] cmd,
                          ref logic signed [15:0] rider_lean, ref logic [11:0] ld_cell_lft,
                          ref logic [11:0] ld_cell_rght, ref logic [11:0] steerPot,
                          ref logic [11:0] batt, ref logic OVR_I_lft, ref logic OVR_I_rght);

    // Initialize all signals to a safe, idle state
    clk          = 0;
    RST_n        = 0;  // Hold DUT in reset initially
    send_cmd     = 0;
    cmd          = 8'h00;
    rider_lean   = 0;  // No forward/backward lean
    ld_cell_lft  = 0;  // No rider weight (empty platform)
    ld_cell_rght = 0;
    steerPot     = 12'h800;  // Center (neutral) steering potentiometer
    batt         = 12'hC00;  // Nominal/healthy battery level
    OVR_I_lft    = 0;  // No over-current conditions
    OVR_I_rght   = 0;

    // Provide at least one full clock edge before releasing reset
    @(posedge clk);
    @(negedge clk);
    RST_n = 1;  // Release reset

    // Allow DUT to run for a short time to exit any internal init states
    repeat (10) @(posedge clk);

  endtask

  // ------------------------------------------------------------------
  // Task: SendCmd
  // Purpose:
  //   Helper to send a single UART command byte into the DUT.
  //
  // Behavior:
  //   - Drives tx_data with the desired command byte.
  //   - Pulses trmt high for one cycle to trigger a UART transmit.
  //   - Waits UART_TX_FULL_FRAME cycles to ensure the full frame has
  //     cleared the transmitter before returning.
  // ------------------------------------------------------------------
  task automatic SendCmd(ref logic clk, ref logic trmt, ref logic [7:0] tx_data,
                         input logic [7:0] cmd);
    // Arm the UART with the new command on a clean clock edge
    @(negedge clk);
    tx_data = cmd;
    trmt    = 1;
    @(negedge clk);
    trmt = 0;  // De-assert transmit request

    // Wait long enough for the entire UART frame to leave the DUT
    repeat (UART_TX_FULL_FRAME) @(posedge clk);

  endtask

  task automatic reset_DUT(ref logic clk, ref logic RST_n);
    // Assert reset
    @(negedge clk);
    RST_n = 0;

    // Hold reset for a few cycles
    repeat (5) @(posedge clk);

    // Release reset
    @(negedge clk);
    RST_n = 1;

    // Allow DUT to run for a short time to exit any internal init states
    repeat (10) @(posedge clk);
  endtask

    task automatic wait4sig(ref logic sig, input int clks2wait, ref logic clk);
    fork
      // Timeout branch: aborts if 'sig' never rises
      begin : timeout
        repeat (clks2wait) @(posedge clk);
        $error("[%0t] Timeout waiting for signal to go HIGH.", $time);
        $stop;
      end

      // Wait-for-rise branch: disables timeout on success
      begin : wait_done
        $display("sig is %b at time %0t, waiting for it to go HIGH...", sig, $time);
        if (sig === 1'b1) begin
          // Already high before we start waiting
        end else begin
          @(posedge sig);
        end
        disable timeout;
        $display("[%0t] Signal asserted (transaction finished).", $time);
      end
    join
  endtask


  // ------------------------------------------------------------
  // Task: check_theta_oscillation
  //
  // Purpose:
  //   Observe the pitch angle 'ptch' over a fixed window and confirm:
  //     1) It actually oscillates (i.e., direction reversals occur).
  //     2) The *initial* motion is in a consistent direction relative
  //        to the starting slope (implementation currently checks
  //        for direction changes; target_val is printed for context).
  //
  // Notes:
  //   - Oscillation is detected by counting sign changes of the
  //     step-to-step difference (ptch[n] - ptch[n-1]) that exceed
  //     a small epsilon (DIR_EPS).
  //   - target_val is not used in the math here, but kept in the
  //     interface and prints so you can correlate to the expected
  //     equilibrium pitch in the log.
  // ------------------------------------------------------------
  task automatic check_theta_oscillation(
      ref logic clk, ref logic signed [15:0] ptch,
      input logic signed [15:0] target_val  // for logging / future use
  );

    int                 dir_changes;
    logic signed [15:0] prev_ptch;
    int                 cycle_count;
    int                 OSC_WIN;
    int                 i;

    int                 prev_dir;  // -1 = decreasing, +1 = increasing, 0 = no movement
    int                 curr_dir;
    int                 diff;
    int                 DIR_EPS;  // Minimum |delta| to treat as meaningful movement

    // ----------------------------------------
    // Initialization
    // ----------------------------------------
    dir_changes = 0;
    cycle_count = 0;
    OSC_WIN     = 400_000;  // Number of cycles to watch for oscillation
    prev_dir    = 0;
    curr_dir    = 0;
    DIR_EPS     = 2;  // Ignore tiny step changes (numerical noise)

    $display("\n--- Running check_theta_oscillation ---");
    $display("Target value (for reference) = %0d", target_val);

    prev_ptch = ptch;

    // ----------------------------------------
    // Oscillation detection via direction changes
    // ----------------------------------------
    for (i = 0; i < OSC_WIN; i++) begin
      @(posedge clk);
      cycle_count++;

      // Step-to-step difference in pitch angle
      diff = ptch - prev_ptch;

      // Determine instantaneous direction of motion
      if (diff > DIR_EPS) curr_dir = +1;
      else if (diff < -DIR_EPS) curr_dir = -1;
      else curr_dir = 0;

      // Count direction reversals (e.g., +1 -> -1 or -1 -> +1)
      if ((curr_dir != 0) && (prev_dir != 0) && (curr_dir != prev_dir)) dir_changes++;

      // Only update prev_dir on "real" movement
      if (curr_dir != 0) prev_dir = curr_dir;

      prev_ptch = ptch;
    end

    // Require at least a couple of direction changes to call it oscillatory
    if (dir_changes < 2) begin
      $display(
          "[check_theta_oscillation] FAIL: Expected oscillation but saw only %0d direction changes.",
          dir_changes);
      $stop;
    end else begin
      $display(
          "[check_theta_oscillation] PASS: Detected %0d direction changes -> oscillation confirmed.",
          dir_changes);
    end

    $display("--- check_theta_oscillation completed ---\n");

  endtask



  // ------------------------------------------------------------
  // Task: check_theta_steady_state
  //
  // Purpose:
  //   After transients die out, verify that 'ptch' spends the
  //   vast majority of a long observation window within:
  //       [target_val - tol, target_val + tol]
  //
  // Behavior:
  //   - Samples ptch each cycle for SETTLE_WIN cycles.
  //   - Counts how many samples fall inside the tolerance band.
  //   - Requires >92% of the samples to be "in band" to pass.
  // ------------------------------------------------------------
  task automatic check_theta_steady_state(ref logic clk, ref logic signed [15:0] ptch,
                                          input logic signed [15:0] target_val, input int tol);

    int SETTLE_WIN;
    int i;
    int stable_count;

    // Length of steady-state observation window (cycles)
    SETTLE_WIN   = 600_000;
    stable_count = 0;

    $display("\n--- Running check_theta_steady_state ---");

    // Observe ptch over the entire settle window and count "in tolerance" samples
    for (i = 0; i < SETTLE_WIN; i++) begin
      @(posedge clk);
      if ((ptch >= target_val - tol) && (ptch <= target_val + tol)) stable_count++;
    end

    // Require more than 92% of samples to be in the +/- tol band
    if (stable_count > SETTLE_WIN * 92 / 100) begin
      $display(
          "[check_theta_steady_state] PASS: ptch reached steady state around %0d (within +/- %0d).",
          target_val, tol);
    end else begin
      $display(
          "[check_theta_steady_state] FAIL: ptch did not settle near %0d (within +/- %0d). Stable samples: %0d/%0d",
          target_val, tol, stable_count, SETTLE_WIN);
      $display("Final ptch value: %0d", ptch);
      $display("Total stable count observed: %0d", stable_count);
      $stop;
    end

    $display("--- check_theta_steady_state completed ---\n");

  endtask



  // ------------------------------------------------------------
  // Task: check_theta_zero
  //
  // Purpose:
  //   High-level wrapper that performs BOTH:
  //     1) Oscillation check
  //     2) Steady-state check
  //
  // Notes:
  //   Kept as a convenience to preserve the original task name
  //   while splitting functionality into two reusable subtasks.
  // ------------------------------------------------------------
  task automatic check_theta_zero(ref logic clk, ref logic signed [15:0] ptch,
                                  input logic signed [15:0] target_val, input int tol);
    $display("\n=== check_theta_zero: oscillation + steady-state ===");
    check_theta_oscillation(clk, ptch, target_val);
    check_theta_steady_state(clk, ptch, target_val, tol);
    $display("=== check_theta_zero completed ===\n");
  endtask

  // -------------------------------------------------------------------
  // Task: check_glitch_free_transitions
  //
  // Purpose:
  //   Apply a series of abrupt steps to an input stimulus signal and
  //   verify that a monitored signal *responds smoothly*:
  //     - It must never go to X/Z.
  //     - It must not exhibit large, single-cycle "jumps" (glitches)
  //       beyond a specified tolerance.
  //
  // Arguments:
  //   clk         : Testbench clock.
  //   signal      : Stimulus signal to drive (e.g., rider_lean).
  //   mon_sig     : Monitored signal (e.g., physical theta) whose
  //                 response should be smooth.
  //   values[]    : OPEN ARRAY of stimulus values to apply to 'signal'.
  //   wait_cycles : Number of cycles to watch 'mon_sig' after each
  //                 applied stimulus value.
  //   tolerance   : Maximum allowed delta between consecutive samples
  //                 of 'mon_sig'. Anything larger is considered a glitch.
  //
  // Usage example:
  //   logic signed [15:0] lean_steps[3] = '{-2000, 0, 2000};
  //   check_glitch_free_transitions(clk, rider_lean, theta_platform,
  //                                 lean_steps, 1000, 50);
  // -------------------------------------------------------------------
  task automatic check_glitch_free_transitions(
      ref logic clk, ref logic signed [15:0] signal,  // Stimulus, e.g. rider_lean
      input logic signed [15:0] mon_sig,  // Monitored, e.g. iPHYS.theta_platform
      input logic signed [15:0] values[],  // Sequence of stimulus steps
      input int wait_cycles, input int tolerance);

    int i, j;
    int                 n;
    logic signed [15:0] prev_mon;
    logic signed [15:0] diff;

    $display("\n--- check_glitch_free_transitions starting at time %0t ---", $time);

    prev_mon = mon_sig;
    n        = $size(values);  // Valid for open-array formals

    // Loop over each stimulus value and check the monitored response
    for (i = 0; i < n; i++) begin
      // Drive new stimulus value
      signal = values[i];
      $display("[Stimulus] signal <= 0x%0h at time %0t", values[i], $time);

      // After each step, observe mon_sig for 'wait_cycles' cycles
      for (j = 0; j < wait_cycles; j++) begin
        @(posedge clk);

        // Check for X/Z on the monitored signal
        if ($isunknown(mon_sig)) begin
          $error("[GLITCH] mon_sig went X/Z at time %0t!", $time);
          $stop;
        end

        // Check for excessively large single-cycle jumps
        diff = mon_sig - prev_mon;
        if ((diff < 0 ? -diff : diff) > tolerance) begin
          $error("[GLITCH] mon_sig jump too large: %0d -> %0d (tol=%0d) at %0t", prev_mon, mon_sig,
                 tolerance, $time);
          $stop;
        end

        prev_mon = mon_sig;
      end
    end

    $display("--- check_glitch_free_transitions PASSED at time %0t ---\n", $time);

  endtask



  // ------------------------------------------------------------------
  // Function: check_equal_with_tolerance
  //
  // Purpose:
  //   Compare two integer values and return 1 if they are within
  //   a given absolute tolerance, else return 0.
  //
  // Formula:
  //   |signal_a - signal_b| <= tolerance
  // ------------------------------------------------------------------
  function automatic bit check_equal_with_tolerance(input int signal_a, input int signal_b,
                                                    input int tolerance);
    int diff;

    diff = signal_a - signal_b;
    if (diff < 0) diff = -diff;

    return (diff <= tolerance);

  endfunction

  // ------------------------------------------------------------------
  // Class: rand_lean
  //
  // Purpose:
  //   Randomization helper for generating rider lean values that stay
  //   outside the "dead zone" around zero. This focuses tests on
  //   clearly forward or clearly backward lean conditions.
  //
  // Constraint:
  //   lean_val is constrained to:
  //     - Large negative lean: [0xF001 : 0xF700]
  //     - Large positive lean: [0x0900 : 0x0FFF]
  //   (Signed 16-bit representation: roughly -4095..-2304 and
  //   +2304..+4095.)
  // ------------------------------------------------------------------
  class rand_lean;
    rand logic signed [15:0] lean_val;

    // Randomly generate values between -4095 to -2304 and +2304 to +4095
    constraint lean_c {
      lean_val inside {[16'shF001 : 16'shF700],  // Strong backward lean
      [16'sh0900 : 16'sh0FFF]  // Strong forward lean
      };
    }
  endclass

  // ------------------------------------------------------------------
  // Task: compute_average
  //
  // Purpose:
  //   Compute the arithmetic mean of a 12-bit signed signal over
  //   'num_samples' clock cycles.
  //
  // Behavior:
  //   - On each posedge of clk, sample 'sig' and accumulate into 'sum'.
  //   - At the end, avg_out = sum / num_samples (integer division).
  //
  // Common uses:
  //   - Smoothing noisy sensor readings for debug checks.
  //   - Estimating DC offset or steady-state level of a signal.
  // ------------------------------------------------------------------
  task automatic compute_average(input logic signed [31:0] sig,  // Signal to sample
                                 input int num_samples, ref logic clk, output int avg_out);
    int sum;
    sum = 0;

    // Take num_samples samples on posedge clk
    for (int i = 0; i < num_samples; i++) begin
      @(posedge clk);
      sum += sig;
    end

    avg_out = sum / num_samples;
  endtask

endpackage
