module SegwayMath (
    input logic signed [11:0] PID_cntrl,       // 16-bit signed pitch signal
    input logic [7:0] ss_tmr,    // 16-bit signed pitch rate
    input logic [11:0] steer_pot,
    input logic en_steer,
    input logic pwr_up,
    output logic signed [11:0] lft_spd,
    output logic signed [11:0] rght_spd,
    output logic too_fast,
    input logic clk                       // clock signal
);

    // scaling with soft start
    logic signed [19:0] product;

    assign product = PID_cntrl * ($signed({1'b0, ss_tmr}));
    logic signed [11:0] PID_ss;
    assign PID_ss = product >>> 8;

    // STEERING INPUT //

    // first clip or limit steer_pot to be between 0x200 and 0xE00
    logic [11:0] steer_pot_sat;
    assign steer_pot_sat =
        (steer_pot < 12'h200) ? 12'h200 :
        (steer_pot > 12'hE00) ? 12'hE00 :
        steer_pot;

    logic signed [11:0] sum;
    assign sum = steer_pot_sat - 12'h7ff;

    // now scale the sum as we don't want to use all of it
    logic signed [11:0] steer_scaled;
    // 1/8 + 1/16 = 3/16
    assign steer_scaled = (sum >>> 4) + (sum >>> 3);

    logic signed [12:0] lft_torque;
    logic signed [12:0] right_torque;

    logic signed [12:0] sum_lft_mux;
    assign sum_lft_mux = {steer_scaled[11],steer_scaled} + {PID_ss[11], PID_ss};

    logic signed [12:0] sum_rght_mux;
    assign sum_rght_mux = {PID_ss[11], PID_ss} - {steer_scaled[11],steer_scaled};

    // now mux based on en_steer, if true then make lft_torque[12:0] = sum_lft_mux else PID_ss
    assign lft_torque = (en_steer) ? sum_lft_mux : {PID_ss[11], PID_ss};
    assign right_torque = (en_steer) ? sum_rght_mux : {PID_ss[11], PID_ss};


    // MOTOR DEADZONE SHAPING //

    // first let's declare the localparams we'll use
    localparam MIN_DUTY = 13'h0A8;
    localparam LOW_TORQUE_BAND = 7'h2A;
    localparam GAIN_MULT = 4'h4;

    logic signed [12:0] neg_region;
    assign neg_region =  lft_torque[12:0] - $signed(MIN_DUTY);

    logic signed [12:0] pos_region;
    assign pos_region = lft_torque[12:0] + $signed(MIN_DUTY);

    // now mux, if lft_torque[12] is 1 (negative as MSB is set), then lft_torque_comp = neg_region else pos_region
    logic signed [12:0] lft_torque_comp;
    assign lft_torque_comp = lft_torque[12] ? neg_region : pos_region;

    // now for the middle band
    logic signed [16:0] prod_highgain_seg; // although 17 bits is sufficient since 2^13-1 * E can be represented in 17 bits
    assign prod_highgain_seg = lft_torque[12:0] * $signed(GAIN_MULT);

    logic [12:0] abs_lft_torque;
    assign abs_lft_torque = lft_torque[12] ? -lft_torque : lft_torque;

    // weird stuff cuz we're choosing between 13 and 17 bits, for prod_highgain_seg we actually don't shift down, we just take the lower 13 bits since it's signed?
    logic signed [12:0] mag_mux_output;
    assign mag_mux_output =
        (abs_lft_torque > $signed(LOW_TORQUE_BAND)) ?
        {lft_torque_comp} :
        (prod_highgain_seg[12:0]);


    // now a mux for checking if pwr_up is asserted, if yes then take mag_mux_output else 0
    logic signed [12:0] lft_shaped;
    assign lft_shaped = (pwr_up) ? mag_mux_output : 13'h0000;

    // now we REPEAT for right side

    logic signed [12:0] neg_region_rght;
    assign neg_region_rght =  right_torque[12:0] - $signed(MIN_DUTY);

    logic signed [12:0] pos_region_rght;
    assign pos_region_rght = right_torque[12:0] + $signed(MIN_DUTY);

    // now mux, if lft_torque[12] is 1 (negative as MSB is set), then lft_torque_compensated = neg_region else pos_region
    logic signed [12:0] rght_torque_comp;
    assign rght_torque_comp = right_torque[12] ? neg_region_rght : pos_region_rght;

    // now for the middle band
    logic signed [16:0] prod_highgain_seg_right; // although 17 bits is sufficient since 2^13-1 * E can be represented in 17 bits
    assign prod_highgain_seg_right = right_torque[12:0] * $signed(GAIN_MULT);

    logic [12:0] abs_right_torque;
    assign abs_right_torque = right_torque[12] ? -right_torque : right_torque;

    // now we mux, if abs(lft_torque_comp) < LOW_TORQUE_BAND then we use prod_highgain_seg else lft_torque_comp
    // weird stuff cuz we're choosing between 12 and 19 bits, so we need to shift prod_highgain_seg down by 6 to get it to 13 bits?
    logic signed [12:0] mag_mux_output_right;
    assign mag_mux_output_right =
        (abs_right_torque > $signed(LOW_TORQUE_BAND)) ?
        {rght_torque_comp} :
        (prod_highgain_seg_right[12:0]);

    // now a mux for checking if pwr_up is asserted, if yes then take mag_mux_output else 0
    logic signed [12:0] right_shaped;
    assign right_shaped = pwr_up ? mag_mux_output_right : 13'h0000;

    // now FINAL SATURATION and OVERSPEED DETECT

    // we start off with signed saturation to 12 bits for both left and right shaped signals

    assign lft_spd =
        (!lft_shaped[12] && lft_shaped[11]) ? 12'h7FF : // too positive
        (lft_shaped[12] && ~lft_shaped[11]) ? 12'h800 : // too negative
        lft_shaped[11:0]; // in range

    assign rght_spd =
        (!right_shaped[12] && right_shaped[11]) ? 12'h7FF : // too positive
        (right_shaped[12] && ~right_shaped[11]) ? 12'h800 : // too negative
        right_shaped[11:0]; // in range

    // now implementing too_fast signal generation
    // too_fast is true if either lft_spd > 1536 or rght_spd > 1536 (SIGNED)
    assign too_fast = (lft_spd > $signed(12'd1536)) || (rght_spd > $signed(12'd1536)) ? 1'b1 : 1'b0;

endmodule