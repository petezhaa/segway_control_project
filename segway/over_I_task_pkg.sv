package over_I_task_pkg;

  // Wait for N PWM periods (using PWM_synch)
  task automatic wait_pwm_cycles(int n, input logic PWM_synch);
    repeat (n) @(posedge PWM_synch);
  endtask

  // Pulse over-current for N consecutive PWM cycles
  task automatic pulse_overcurrent_cycles(int cycles, ref logic clk, ref logic PWM_synch,
                                          ref logic ovr_I_blank, ref logic OVR_I_lft,
                                          ref logic OVR_I_rght, input logic left_or_right);
    int i;

    repeat (cycles)
      @(posedge PWM_synch) begin
        @(posedge clk);
        wait (ovr_I_blank === 1'b1);


        @(posedge clk);
        if (left_or_right) begin
          OVR_I_lft  = 1'b1;
          OVR_I_rght = 1'b0;
        end else begin
          OVR_I_rght = 1'b1;
          OVR_I_lft  = 1'b0;
        end
        @(posedge clk);
        OVR_I_lft  = 1'b0;
        OVR_I_rght = 1'b0;
      end
  endtask

  // Inject overcurrent ONLY when ovr_I_blank == 0 (outside blanking window)
  task automatic inject_overcurrent_outside_blank(int cycles, ref logic clk, ref logic PWM_synch,
                                                  ref logic ovr_I_blank, ref logic OVR_I_lft,
                                                  ref logic OVR_I_rght, input logic left_or_right);
    int i;

    repeat (cycles)
      @(posedge PWM_synch) begin
        @(posedge clk);
        wait (ovr_I_blank === 1'b0);


        @(posedge clk);
        if (left_or_right) begin
          OVR_I_lft  = 1'b1;
          OVR_I_rght = 1'b0;
        end else begin
          OVR_I_rght = 1'b1;
          OVR_I_lft  = 1'b0;
        end
        @(posedge clk);
        OVR_I_lft  = 1'b0;
        OVR_I_rght = 1'b0;
      end
  endtask

endpackage : over_I_task_pkg
