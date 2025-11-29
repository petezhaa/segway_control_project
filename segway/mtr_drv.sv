module mtr_drv(clk,rst_n,lft_spd,rght_spd,OVR_I_lft,OVR_I_rght,PWM1_lft,
               PWM2_lft,PWM1_rght,PWM2_rght,OVR_I_shtdwn);
			   
  input logic clk,rst_n;
  input logic [11:0] lft_spd, rght_spd;	// 12-bit signed (negative is reverse)
  input logic OVR_I_lft, OVR_I_rght;
  
  output logic PWM1_lft, PWM2_lft, PWM1_rght, PWM2_rght;
  output reg OVR_I_shtdwn;
  
  ////////////////////////////////////////////
  // synchronize duty changes to PWM cycle //
  //////////////////////////////////////////
  reg [11:0] lft_spd_synch,rght_spd_synch;
  ///////////////////////////////////////////////////////
  // Meta-stability flops and latching flop for OVR_I //
  /////////////////////////////////////////////////////
  reg OVR_I_ff1,OVR_I_ff2,OVR_I_ff3;

  /////////////////////////////////////////////////////////
  // Flops for handling shut down if OVR_I too frequent //
  ///////////////////////////////////////////////////////
  reg [4:0] OVR_I_cnt;
  reg [3:0] PWM_cycle_cnt;
  reg OVR_I_inc_blank_n;
  
  wire [10:0] lft_mag, rght_mag;	// absolute value of lft/rght_spd
  wire PWM_lft,PWM_rght,PWM_lft_n,PWM_rght_n;
  wire PWM_synch;						// only need from one of the PWMs (choosing lft)
  wire OVR_I_rise;
  wire ovr_I_blank;
  
  logic [7:0] LED;
  assign LED = {OVR_I_shtdwn,7'h00};
  
  ////////////////////////////////////////////////////
  // synchronize lft_spd & rght_spd with PWM cycle //
  // so changes can only occur at "safe" time     //
  /////////////////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
	   lft_spd_synch <= 12'h800;
	   rght_spd_synch <= 12'h800;
	 end else if (PWM_synch) begin
	   lft_spd_synch <= lft_spd + 12'h800;      // convert to unsigned
	   rght_spd_synch <= rght_spd + 12'h800;    // convert to unsigned
	 end
  
  //////////////////////////////////////
  // Find ABS of speeds to drive PWM //
  ////////////////////////////////////
  //assign lft_mag = (lft_spd_synch[11]) ? ~lft_spd_synch : lft_spd_synch[10:0];		// Note deliberate 1's comp so -2048 does not overflow
  //assign rght_mag = (rght_spd_synch[11]) ? ~rght_spd_synch : rght_spd_synch[10:0];	 
  
  //////////////////////
  // Instatiate PWMs //
  ////////////////////
  PWM11 iPWM_lft(.clk(clk), .rst_n(rst_n), .duty(lft_spd_synch[11:1]), .PWM_synch(PWM_synch),
                 .ovr_I_blank(ovr_I_blank), .PWM1(PWM_lft), .PWM2(PWM_lft_n)); 
  PWM11 iPWM_rght(.clk(clk), .rst_n(rst_n), .duty(rght_spd_synch[11:1]), .PWM_synch(),
                  .ovr_I_blank(), .PWM1(PWM_rght), .PWM2(PWM_rght_n));
  
  /////////////////////////////////////////////////////////
  // OVR_I_lft/rght come from opamp and must by synched //
  // It is ignored during blanking period              //
  //////////////////////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n) begin
	  OVR_I_ff1 <= 1'b0;
	  OVR_I_ff2 <= 1'b0;
	  OVR_I_ff3 <= 1'b0;
	end else begin
	  //// OVR_I_ff1 can only be set outside blanking window ////
	  OVR_I_ff1 <= ~ovr_I_blank & (OVR_I_lft | OVR_I_rght);
	  OVR_I_ff2 <= OVR_I_ff1;
	  OVR_I_ff3 <= OVR_I_ff2;
	end
	
  assign OVR_I_rise = OVR_I_ff2 & ~OVR_I_ff3;
  
  ///////////////////////////////////////////
  // Count the instances of OVR_I events  //
  // If they occur many in a row we trip //
  // semi-permanent                     //
  ///////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
      OVR_I_cnt <= 5'h00;
    else if (OVR_I_rise & OVR_I_inc_blank_n)
	  //// If rise of OVR_I occurs outside blanking   ////
	  //// period increment count of OVR_I occurances ////
      OVR_I_cnt <= OVR_I_cnt + 1;
    else if ((&PWM_cycle_cnt) & PWM_synch & (|OVR_I_cnt))
	  //// Decrement OVR_I_cnt once every 16 PWM cycles ////
	  //// (slow decay of count so can "reset" if motor ////
	  //// current returns to normal condition)         ////
	  OVR_I_cnt <= OVR_I_cnt - 1;

  ////////////////////////////////////////////////////
  // To ensure only one increment of OVR_I_cnt can //
  // occur each PWM cycle we inhibit count once a //
  // rise of OVR_I will be counted.              //
  ////////////////////////////////////////////////
  always_ff @(posedge clk)
    if (PWM_synch)
	  OVR_I_inc_blank_n <= 1'b1;		// re-enable inc
	else if (OVR_I_rise)
	  OVR_I_inc_blank_n <= 1'b0;
	  
  ///////////////////////////////////////////////
  // Count PWM cycles.  Every 16 we decrement //
  // OVR_I_cnt. This enables the slow decay  //
  // of OVR_I_cnt to allow "reset"          //
  ///////////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	  PWM_cycle_cnt <= 4'h0;
	else if (PWM_synch)
	  PWM_cycle_cnt <= PWM_cycle_cnt + 1;
	  
  ///////////////////////////////////////////
  // If OVR_I_cnt gets to 31 we shut down //
  /////////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
    if (!rst_n)
	   OVR_I_shtdwn <= 1'b0;
	 else if (OVR_I_cnt==5'h1F)
	   OVR_I_shtdwn <= 1'b1;
	  

  ////////////////////////////////////////////////////
  // Running H-bridge in "fully driven" stiff mode //
  //////////////////////////////////////////////////
  assign PWM1_lft = (OVR_I_ff2 | ~OVR_I_inc_blank_n | OVR_I_shtdwn) ? 1'b0 : PWM_lft;
  assign PWM2_lft = (OVR_I_ff2 | ~OVR_I_inc_blank_n | OVR_I_shtdwn) ? 1'b0 : PWM_lft_n;
  assign PWM1_rght = (OVR_I_ff2 | ~OVR_I_inc_blank_n | OVR_I_shtdwn) ? 1'b0 : PWM_rght;
  assign PWM2_rght = (OVR_I_ff2 | ~OVR_I_inc_blank_n | OVR_I_shtdwn) ? 1'b0 : PWM_rght_n;
  
endmodule
  
  
  