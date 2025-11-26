localparam G = 8'h47;
localparam S = 8'h53;
localparam UART_TX_FULL_FRAME = 52080; // Number of clock cycles for full UART transmission at 19200 baud with 50MHz clock

// this task is used to initialize the DUT and its signals
task automatic init_DUT(ref logic clk, ref logic RST_n, ref logic send_cmd, ref logic [7:0] cmd,
                        ref logic signed [15:0] rider_lean, ref logic [11:0] ld_cell_lft,
                        ref logic [11:0] ld_cell_rght, ref logic [11:0] steerPot,
                        ref logic [11:0] batt, ref logic OVR_I_lft, ref logic OVR_I_rght);

  clk = 0;
  RST_n = 0;
  send_cmd = 0;
  cmd = 8'h00;
  rider_lean = 0;
  ld_cell_lft = 0;
  ld_cell_rght = 0;
  steerPot = 12'h7FF;
  batt = 12'hC00;
  OVR_I_lft = 0;
  OVR_I_rght = 0;

  @(posedge clk);
  @(negedge clk);
  RST_n = 1;

  repeat (10) @(posedge clk);

endtask

task automatic SendCmd(ref logic clk, ref logic trmt, ref logic [7:0] tx_data,
                       input logic [7:0] cmd);
  @(negedge clk);
  tx_data = cmd;
  trmt = 1;
  @(negedge clk);
  trmt = 0;

  repeat (UART_TX_FULL_FRAME) @(posedge clk);

endtask


function automatic bit check_equal_with_tolerance(logic signed signal_a, logic signed signal_b,
                                                  input logic tolerance);
  int diff;

  diff = signal_a - signal_b;
  if (diff < 0) diff = -diff;

  return (diff <= tolerance);

endfunction

class rand_lean;
  rand logic signed [15:0] lean_val;


  // randomly generates values between -4095 to -2304  and +2304 to +4095
  constraint lean_c {lean_val inside {[16'shF001 : 16'shF700], [16'sh0900 : 16'sh0FFF]};}
endclass

// computes the average of a signal over a specified number of samples
task automatic compute_average(ref logic signed [11:0] sig,  // signal to sample
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
