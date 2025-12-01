module Auth_blk (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        RX,
    input logic         rider_off,
    output logic        pwr_up
);


    // instantiate UART_rx module
    logic [7:0] rx_data;
    logic        rx_rdy;
    logic        clr_rx_rdy;
    UART_rx iUART_rx (
        .clk    (clk),
        .rst_n  (rst_n),
        .rx_data(rx_data),
        .RX     (RX),
        .clr_rdy(clr_rx_rdy),
        .rdy    (rx_rdy)
    );

    // now for the auth SM
    typedef enum logic [1:0] {IDLE, CONNECTED, DISCONNECTED} state_t;
    state_t current_state, next_state;

    // flop to store SM state
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            current_state <= IDLE; // reset to idle state
        else
            current_state <= next_state; // move to next state
    end

    // watch out only 3 states so we absorb the unused state into the default case
    always_comb begin

        //default the outputs and next_state
        clr_rx_rdy = 1'b0;
        pwr_up = 1'b0;
        next_state = current_state; // defaulted to current state to optimize

        case(current_state)

            CONNECTED: begin
                pwr_up = 1'b1; // keep pwr_up asserted
                if(rider_off) begin
                    if(rx_rdy && rx_data==8'h53) begin
                        clr_rx_rdy = 1'b1; // clear the rdy signal to indicate that we read the data
                        next_state = IDLE;
                        pwr_up = 1'b0;
                    end
                end
                else if(rx_rdy && rx_data==8'h53) begin
                    clr_rx_rdy = 1'b1; // clear the rdy signal to indicate that we read the data
                    next_state = DISCONNECTED;
                    pwr_up = 1'b1;
                end
                // implicit else stay in state: nxt_state = state
            end

            DISCONNECTED: begin
                pwr_up = 1'b1; // keep pwr_up asserted
                if(rider_off) begin
                    //pwr_up = 1'b0;
                    next_state = IDLE;
                end
                else if(rx_rdy && rx_data==8'h47) begin
                    clr_rx_rdy = 1'b1; // clear the rdy signal to indicate that we read the data
                    next_state = CONNECTED;
                    pwr_up = 1'b1;
                end
                // implied else stay in state: nxt_state = state
            end

            default: begin // absorb IDLE state into default state
                if(rx_rdy && rx_data==8'h47) begin
                    clr_rx_rdy = 1'b1; // clear the rdy signal to indicate that we read the data
                    pwr_up = 1'b1; // assert pwr_up
                    next_state = CONNECTED; // go to connected state
                end
                // implicit else stay in IDLE: nxt_state = state
            end

        endcase
    end

endmodule