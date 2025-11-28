module PID (
    input logic signed [15:0] ptch,       // 16-bit signed pitch signal
    input logic signed [15:0] ptch_rt,    // 16-bit signed pitch rate
    output logic signed [11:0] PID_cntrl,
    input logic clk,
    input logic rst_n,
    input logic vld,
    output logic [7:0] ss_tmr,
    input logic pwr_up,
    input logic rider_off
);

    parameter fast_sim = 1'b1;

    // define the local param or multiplier
    localparam P_COEFF = 5'h09; // coefficient for P term

    // raw (pre-extended) terms
    logic signed [14:0] P_term_raw;
    logic signed [14:0] I_term_raw;
    logic signed [12:0] D_term_raw;
    // sign-extended to 16 bits for summing
    logic signed [15:0] P_term_ext;
    logic signed [15:0] I_term_ext;
    logic signed [15:0] D_term_ext;
    logic signed [9:0] ptch_err_sat;

    // Let's start with the P term

    // Signed saturation for signed_err
    assign ptch_err_sat =
	(!ptch[15] && |ptch[14:9]) ? 10'h1FF : // too positive
    (ptch[15] && ~&ptch[14:9]) ? 10'h200 : // too negative
    ptch[9:0]; // in range

    assign P_term_raw = $signed(P_COEFF) * ptch_err_sat;

    // Now the I term

    logic signed [17:0] integrator;

    // I term logic

    // sign extend 18 bits ptch_err_sat
    logic signed [17:0] sign_ext_ptch_err_sat;
    assign sign_ext_ptch_err_sat = {{8{ptch_err_sat[9]}},ptch_err_sat};

    logic signed [17:0] integrator_addition;
    assign integrator_addition = sign_ext_ptch_err_sat + integrator;

    logic ov;
    // inspect MSBs of the 2 numbers being added. if they match but don't match the result of the addition then overflow occurred
    assign ov = ((sign_ext_ptch_err_sat[17] == integrator[17]) && (integrator[17] != integrator_addition[17]));

    logic mux_SEL;
    assign mux_SEL = (vld && ~ov);

    // mux to choose between integrator and addition
    logic signed [17:0] ov_mux_output;

    assign ov_mux_output = (mux_SEL) ? integrator_addition : integrator;

    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            integrator <= '0;
        else if (rider_off)
                integrator <= '0; // clear integrator when rider is off
            else
                integrator <= ov_mux_output;
    end


    // Finally the D term
    assign D_term_raw = -({{3{ptch_rt[15]}} , ptch_rt[15:6]});

    // sign extend each of the 3 raw terms to 16-bit sign-extended values
    assign P_term_ext = { {1{P_term_raw[14]}}, P_term_raw[14:0] };
    assign I_term_ext = { {1{I_term_raw[14]}}, I_term_raw[14:0] };
    assign D_term_ext = { {3{D_term_raw[12]}}, D_term_raw[12:0] };

    // sum all 3 terms into a 16 bit signed value THEN saturate to 12 bits
    logic signed [15:0] sum;
    assign sum = P_term_ext + I_term_ext + D_term_ext;


    // Now we saturate the sum and assign to PID_cntrl

    assign PID_cntrl =
    (!sum[15] && |sum[14:11]) ? 12'h7FF : // too positive
    (sum[15] && ~&sum[14:11]) ? 12'h800 : // too negative
    sum[11:0]; // in range;

    // ss_tmr implementation
    logic [26:0] long_tmr;

    logic [8:0] increment; // default increment value

    // mux to choose between long timer frozen near full value and incremented timer
    logic [26:0] long_tmr_mux_output;

    assign long_tmr_mux_output = (&long_tmr[26:19]) ? long_tmr[26:0] : (long_tmr+increment);

    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            long_tmr <= 0;
        else if (!pwr_up)
                long_tmr <= 0;
            else
                long_tmr <= long_tmr_mux_output;
    end

    assign ss_tmr = long_tmr[26:19];

    // Ex. 15

    generate
        if (fast_sim) begin
                assign increment = 9'd256; // faster increment for fast sim

                // saturation logic as we dont use bits [17:16] directly, ternary statement
                assign I_term_raw =
	                    (!integrator[17] && |integrator[16:15]) ? 15'h3FFF : // too positive
                        (integrator[17] && ~&integrator[16:15]) ? 15'h4000 : // too negative
                        integrator[15:1]; // in range
        end
        else begin
                assign increment = 9'd1; // normal increment for synthesis
                assign I_term_raw = {{3{integrator[17]}},integrator[17:6]};
        end
    endgenerate


endmodule