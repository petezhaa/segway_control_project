`default_nettype none
module A2D_intf(clk, rst_n, nxt, batt, rght_ld, lft_ld, steer_pot, SS_n, SCLK, MOSI, MISO);

    // declare input and output signals
    input logic clk, rst_n, nxt, MISO;
    output logic SS_n, SCLK, MOSI;
    output logic [11:0] batt, rght_ld, lft_ld, steer_pot;

    // declare intermediate signals
    logic [2:0] channel; // there are channels 0,4,5,6

    //TODO: to check; we perform 2 transactions back to back, first one tells A2DC what channel we want to convert, & 2nd tran. used to read converted results back

    // SPI transaction signals
    logic update, wrt, done;
    logic [15:0] wt_data, rd_data;

    // enables for each channel
    logic en_batt, en_rght_ld, en_lft_ld, en_steer_pot;

    // create enum for conversion states
	typedef enum reg [1:0] {IDLE, WRT, WAIT, RECEIVE} state_t;
    state_t state, nxt_state;

    // instantiate the SPI instance
    SPI_mnrch iSPI(.clk(clk), .rst_n(rst_n), .SS_n(SS_n), .SCLK(SCLK), .MOSI(MOSI), .MISO(MISO), .wrt(wrt), .wt_data(wt_data), .done(done), .rd_data(rd_data));

    // state machine reset is IDLE state
	always_ff @(posedge clk, negedge rst_n) begin
		if (!rst_n) state <= IDLE;
		else state <= nxt_state;
	end

    // state machine transition logic
    always_comb begin
        // default outputs
        wrt = 0;
        en_batt = 0;
        en_rght_ld = 0;
        en_lft_ld = 0;
        en_steer_pot = 0;
        update = 0;
        nxt_state = state;

        case(state)
            // once the transaction is complete, move to wait state
            WRT:
                if (done) begin
                    nxt_state = WAIT;
                end
            // wait for one clock
            WAIT: begin
                //TODO: he says there's one clock cycle of dead time between the 2 transactions so i think that happens implicitly but unsure
                wrt = 1;
                nxt_state = RECEIVE;
            end
            // conversion is complete, set the enable signals and move to idle
            RECEIVE:
                if (done) begin
                    // set the enable signals
                    en_lft_ld = (channel == 3'b000);
                    en_rght_ld = (channel == 3'b100);
                    en_steer_pot = (channel == 3'b101);
                    en_batt = (channel == 3'b110);

                    update = 1;
                    nxt_state = IDLE;

                end
            // this is the idle state
            default:
                if (nxt) begin
                    // start the conversion
                    wrt = 1;
                    nxt_state = WRT;
                end
        endcase
    end

    // 2 bit counter
    logic [1:0] cntr;
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            cntr <= 0; // reload
        else if(update)
            cntr <= cntr + 1'b1;
    end

    // channel mapping: cntr 00 -> channel 0, 01 -> channel 4, 10 -> channel 5, 11 -> channel 6
    // Registered implementation: update `channel` on the rising edge of `clk` when `update` is asserted.
    // Otherwise hold the previous value. Reset initializes channel to 0.
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            channel <= 3'b000;
        else if (update) begin
            unique case (cntr)
                2'b00: channel <= 3'b100; // channel 0
                2'b01: channel <= 3'b101; // channel 4
                2'b10: channel <= 3'b110; // channel 5
                2'b11: channel <= 3'b000; // channel 6
                //default: channel <= 3'b000;
            endcase
        end
        // else: keep previous channel value
    end

    // assign the wt_data to the concatenation of channel and padding
    assign wt_data = {2'b00, channel, 11'h000};

    // flip flop for the battery
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            batt <= 0;
        else if (en_batt)
            batt <= rd_data[11:0];
    end

    // flip flop for the rght_ld
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            rght_ld <= 0;
        else if (en_rght_ld)
            rght_ld <= rd_data[11:0];
    end

    // flip flop for the lft_ld
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            lft_ld <= 0;
        else if (en_lft_ld)
            lft_ld <= rd_data[11:0];
    end

     // flip flop for the steer_pot
    always_ff @(posedge clk, negedge rst_n) begin
        if (!rst_n)
            steer_pot <= 0;
        else if (en_steer_pot)
            steer_pot <= rd_data[11:0];
    end

endmodule
