module PWM11 (
    input logic clk,
    input logic rst_n,
    input logic [10:0] duty,  // 11-bit duty cycle input
    output logic PWM1,
    output logic PWM2,
    output logic PWM_synch,
    output logic ovr_I_blank
);

  localparam NONOVERLAP = 11'h040;

  // the 11-bit counter
  logic [10:0] cnt;

  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      cnt <= 0;
    end else begin
      cnt <= cnt + 1'b1;
    end
  end

  // PWM2 generation
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      PWM2 <= 1'b0;
    end else begin
      if (&cnt) begin
        PWM2 <= 1'b0;
      end else if (cnt >= (duty + NONOVERLAP)) begin
        PWM2 <= 1'b1;
      end
    end
  end

  // PWM1 generation
  always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
      PWM1 <= 1'b0;
    end else begin
      if (cnt >= duty) begin
        PWM1 <= 1'b0;
      end else if (cnt >= NONOVERLAP) begin
        PWM1 <= 1'b1;
      end
    end
  end

  // PWM_Synch generation
  assign PWM_synch = ~(|cnt);

  // ovr_I_blank generation
  // assign ovr_I_blank = (NONOVERLAP < cnt < (NONOVERLAP + 128)) ||
  //                        ((NONOVERLAP + duty) < cnt < (duty + NONOVERLAP + 128));
  assign ovr_I_blank =
  ((cnt > NONOVERLAP) && (cnt < NONOVERLAP + 11'd128)) ||                 // leading edge blank
      ((cnt > duty + NONOVERLAP) && (cnt < duty + NONOVERLAP + 11'd128));  // trailing edge blank



endmodule
