module balance_cntrl #(
    parameter logic fast_sim = 1'b1    // Default to fast simulation mode
) (
    // Inputs from sensors/controls
    input logic signed [15:0] ptch,
    input logic signed [15:0] ptch_rt,
    input logic [11:0] steer_pot,
    input logic clk,
    input logic rst_n,
    input logic vld,
    input logic pwr_up,
    input logic rider_off,
    input logic en_steer,
    // Outputs to motors/system
    output logic signed [11:0] lft_spd,
    output logic signed [11:0] rght_spd,
    output logic too_fast
);

    // Internal signals to connect PID to SegwayMath
    logic signed [11:0] PID_cntrl;
    logic [7:0] ss_tmr;

    // Instantiate PID controller with fast_sim parameter
    PID #(.fast_sim(fast_sim)) iPID(
        .ptch(ptch),
        .ptch_rt(ptch_rt),
        .PID_cntrl(PID_cntrl),
        .clk(clk),
        .rst_n(rst_n),
        .vld(vld),
        .ss_tmr(ss_tmr),
        .pwr_up(pwr_up),
        .rider_off(rider_off)
    );

    // Instantiate SegwayMath
    SegwayMath iSegwayMath(
        .PID_cntrl(PID_cntrl),
        .ss_tmr(ss_tmr),
        .steer_pot(steer_pot),
        .en_steer(en_steer),
        .pwr_up(pwr_up),
        .lft_spd(lft_spd),
        .rght_spd(rght_spd),
        .too_fast(too_fast)
    );

endmodule