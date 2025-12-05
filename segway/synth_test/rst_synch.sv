`timescale 1ns/1ps
module rst_synch (
    input  logic clk,    // system clock
    input  logic RST_n,  // asynchronous reset
    output logic rst_n   // synchronized reset output
);

  logic rst_d1;  // internal signals

  // first stage ff
  always_ff @(posedge clk, negedge RST_n) begin
    if (!RST_n) begin
      rst_d1 <= 1'b0;
    end else begin
      rst_d1 <= 1'b1;
    end
  end

  // second stage ff
  always_ff @(posedge clk, negedge RST_n) begin
    if (!RST_n) begin
      rst_n <= 1'b0;
    end else begin
      rst_n <= rst_d1;
    end
  end
endmodule
