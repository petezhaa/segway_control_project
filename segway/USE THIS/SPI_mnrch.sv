module SPI_mnrch(

input logic clk,
input logic rst_n,
output logic SS_n,
output logic SCLK,
output logic MOSI,
input logic MISO,
input logic wrt,
input logic [15:0] wt_data,
output logic done,
output logic [15:0] rd_data

);

    typedef enum reg [1:0] {IDLE, FRONT_PORCH, WORK_HORSE, BACK_PORCH} state_t;
    state_t state, nxt_state;

        // state flop
        always_ff @(posedge clk or negedge rst_n) begin
            if(!rst_n)
                state <= IDLE;
            else
                state <= nxt_state;
        end

    // signals for state machine
    logic shft_imm;
    logic done15;
    logic init;
    logic shft;
    logic ld_SCLK;
    logic set_done;

    // SM block
    always_comb begin

        // default outputs and nxt state
        init = 1'b0;
        shft = 1'b0;
        ld_SCLK = 1'b0;
        set_done = 1'b0;
        nxt_state = state;

        case(state)
            FRONT_PORCH: begin
                if(shft_imm)
                    nxt_state = WORK_HORSE;
                // implicit else stay/wait in state
            end

            WORK_HORSE: begin
                if(done15)
                    nxt_state = BACK_PORCH;
                else if(shft_imm)
                    shft = 1'b1;
                    // implicit else stay/wait in state
            end

            BACK_PORCH: begin
            if(shft_imm) begin
                ld_SCLK = 1'b1;
                set_done = 1'b1;
                shft = 1'b1;
                nxt_state = IDLE;
                end
                // implicit else stay/wait in state
            end

            default: begin // absorbs default state and IDLE state
            ld_SCLK = 1'b1; // Moore output
                if(wrt) begin
                    init = 1'b1;
                    nxt_state = FRONT_PORCH;
                end
                // else waits in IDLE
            end
        endcase
    end


    // BIT COUNTER block
    logic [3:0] bit_cnt;
    always_ff @(posedge clk) begin
        if(init)
            bit_cnt <= '0;
        else if(shft)
            bit_cnt <= bit_cnt + 1;
        // implicit else: hold value
    end

    assign done15 = (&bit_cnt);


    // SCLK_divider logic

    logic [3:0] SCLK_div;
    logic smpl;

    always_ff @(posedge clk) begin
        if(ld_SCLK)
            SCLK_div <= 4'b1011;
        else
            SCLK_div <= SCLK_div + 1;
    end

    assign SCLK = SCLK_div[3];
    // decoding cloud in schematic
    assign shft_imm = (SCLK_div == 4'b1111); // next thing it'll do is roll over i.e. fall of SCLK, that's when we shift
    assign smpl = (SCLK_div == 4'b0111); // as we sample MISO on rise of SCLK or the MSB (next clock cycle)

    // separate flop for sampling MISO
    logic MISO_smpl;
    always_ff @(posedge clk) begin
        if(smpl)
            MISO_smpl <= MISO;
        // implicit else: hold value
    end

    // shift register block
    logic [15:0] shift_reg;
    always_ff @(posedge clk) begin
        if(init)
            shift_reg <= wt_data;
        else if(shft)
            shift_reg <= {shift_reg[14:0], MISO_smpl};
        // implicit else: hold value
    end

    assign MOSI = shift_reg[15];
    assign rd_data = shift_reg;


    // Below we have 2 separate blocks for done and SS_n just because of the difference in async preset/reset requirements for each

    // SR flop for done
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            done <= 1'b0;
        else begin
            if(init)
                done <= 1'b0;
            else if(set_done)
                done <= 1'b1;
            // implicit else: hold value
        end
    end

    // SR flop for SS_n
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            SS_n <= 1'b1;
        else begin
            if(init)
                SS_n <= 1'b0;
            else if(set_done)
                SS_n <= 1'b1;
        end
    end


endmodule