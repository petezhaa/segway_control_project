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
  steerPot = 12'h800;
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
