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

    // Local variables for self-checking
    int lft_avg, rght_avg;
    int diff;

    initial begin

        /// Your magic goes here /// // use connect by name here 
        init_DUT(.clk(clk), .RST_n(RST_n), .send_cmd(send_cmd), .cmd(cmd), .rider_lean(rider_lean),
                         .ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .steerPot(steerPot),
                         .batt(batt), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght));

        // Send 'G' command
        SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));

        repeat (3000) @(posedge clk);  // initial small delay
        ld_cell_lft  = 12'h300;  // simulate rider getting on
        ld_cell_rght = 12'h300;  // simulate rider getting on
        repeat (325000) @(posedge clk);  // stabilization before lean tests

        // Check: System powered up with rider present
        if (!iDUT.pwr_up) begin
            $display("FAIL: pwr_up not asserted after GO with rider on");
            $stop();
        end

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

        // Glitch-free transition check across lean values
        check_glitch_free_transitions(.clk(clk), .signal(rider_lean), .mon_sig(iPHYS.theta_platform),
                                                                    .values('{16'sh0FFF, 16'shF000, 16'sh0000}), .wait_cycles(5000),
                                                                    .tolerance(16'd0400));

        repeat (500000) @(posedge clk);  // observe ring-down behavior
        $display("=== Lean Tests Complete ===");


        // ================== Added Comprehensive System Tests ==================

        $display("=== Starting steering Tests ===");
        // Steering right then left (differential wheel response)
        steerPot = 12'hE00;
        repeat (600000) @(posedge clk);
        steerPot = 12'h200;
        repeat (600000) @(posedge clk);
        steerPot = 12'h800; // center
        repeat (300000) @(posedge clk);

        // Check: Center steering balance
        $display("Checking center steering balance...");
        compute_average(.sig(iPHYS.omega_lft), .num_samples(256), .clk(clk), .avg_out(lft_avg));
        compute_average(.sig(iPHYS.omega_rght), .num_samples(256), .clk(clk), .avg_out(rght_avg));
        diff = lft_avg - rght_avg;
        diff = (diff < 0) ? -diff : diff;
        if (diff > 150) begin
            $display("FAIL: Motors not balanced at center steering (diff=%0d)", diff);
            $stop();
        end
        $display("Center steering balance OK (diff=%0d)", diff);
        $display("=== Steering Tests Complete ===");

        $display("=== Starting rider lean Tests ===");
        // Rider lean forward then backward (speed reversal / braking)
        rider_lean = 16'sh0600;
        repeat (800000) @(posedge clk);
        if (iPHYS.omega_lft <= 0 || iPHYS.omega_rght <= 0) begin
            $display("FAIL: Motors did not achieve expected forward speed");
            $display("omega_lft=%0d, omega_rght=%0d", iPHYS.omega_lft, iPHYS.omega_rght);
            $stop();
        end
        $display("Forward speed test passed.");
        rider_lean = -16'sh0400;
        repeat (800000) @(posedge clk);
        if (iPHYS.omega_lft >= 0 || iPHYS.omega_rght >= 0) begin
            $display("FAIL: Motors did not achieve expected backward speed");
            $display("omega_lft=%0d, omega_rght=%0d", iPHYS.omega_lft, iPHYS.omega_rght);
            $stop();
        end
        $display("Backward speed test passed.");
        rider_lean = 16'sh0000;
        repeat (400000) @(posedge clk);
        if (iPHYS.omega_lft < -50 || iPHYS.omega_rght < -50 || iPHYS.omega_lft > 50 || iPHYS.omega_rght > 50) begin
            $display("FAIL: Motors did not return to zero speed");
            $display("iPHYS.omega_lft < -50 evaluated to %0d", iPHYS.omega_lft < -50);
            $display("iPHYS.omega_rght < -50 evaluated to %0d", iPHYS.omega_rght < -50);
            $display("iPHYS.omega_lft > 50 evaluated to %0d", iPHYS.omega_lft > 50);
            $display("iPHYS.omega_rght > 50 evaluated to %0d", iPHYS.omega_rght > 50);
            $display("omega_lft=%0d, omega_rght=%0d", iPHYS.omega_lft, iPHYS.omega_rght);
            $stop();
        end
        $display("Rider not leaning passed.");
        $display("=== Rider lean Tests Complete ===");

        // Simulate step-off (load cells go low) -> expect internal disable
        $display("=== Starting step-off Tests ===");
        ld_cell_lft  = 12'h020;
        ld_cell_rght = 12'h010;
        repeat (600000) @(posedge clk);

        // Check: Step-off triggers rider_off
        if (!iDUT.rider_off) begin
            $display("FAIL: rider_off not asserted after step-off");
            $stop();
        end
        $display("Step-off detected correctly.");

        // Send STOP command to complete shutdown sequence
        SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
        repeat (100000) @(posedge clk);

        // Check: pwr_up drops after STOP + rider_off
        if (iDUT.pwr_up) begin
            $display("FAIL: pwr_up remained high after STOP and step-off");
            $stop();
        end
        $display("Step-off and STOP command behavior passed.");

        // Re-mount rider and send STOP then GO sequence
        ld_cell_lft  = 12'h300;
        ld_cell_rght = 12'h300;
        repeat (200000) @(posedge clk);
        SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
        repeat (400000) @(posedge clk);
        SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(G));
        repeat (600000) @(posedge clk);

        // Check: GO re-enables power
        if (!iDUT.pwr_up) begin
            $display("FAIL: pwr_up did not reassert after GO command");
            $stop();
        end
        $display("GO command re-enabled power as expected.");

        // Battery voltage ramp down (warning / shutdown behavior)
        batt = 12'hC00;
        repeat (200000) @(posedge clk);
        batt = 12'hA00;
        repeat (200000) @(posedge clk);
        batt = 12'h800;
        repeat (200000) @(posedge clk);
        batt = 12'h650;
        repeat (300000) @(posedge clk);
        batt = 12'h500; // near critical
        repeat (400000) @(posedge clk);
        batt = 12'h3A0; // critical low
        repeat (400000) @(posedge clk);

        // Check: Battery low asserted at critical level
        if (!iDUT.batt_low) begin
            $display("FAIL: batt_low not asserted at critical low battery");
            $stop();
        end

        // Recover battery and verify batt_low clears
        batt = 12'h900;
        repeat (100000) @(posedge clk);
        if (iDUT.batt_low) begin
            $display("FAIL: batt_low did not clear on battery recovery");
            $stop();
        end

        // Final STOP
        SendCmd(.clk(clk), .trmt(send_cmd), .tx_data(cmd), .cmd(S));
        repeat (400000) @(posedge clk);

        // ======================================================================

        $display("Segway system test passed!");
        #100;
        $stop();
    end

    always #10 clk = ~clk;

endmodule