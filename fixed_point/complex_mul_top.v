// Top-level: press S1 -> run complex multiply test vectors, send results over UART.
//
// Output format per test (ASCII hex, human-readable):
//   "T<n>: (<a_re>,<a_im>)*(<b_re>,<b_im>)=(<p_re>,<p_im>)\n"
// All values printed as signed decimal Q1.15 raw integers.

module complex_mul_top (
    input  wire       clk,
    input  wire       btn_s1,
    input  wire       btn_s2,
    output reg  [5:0] led,
    output wire       uart_tx
);

    // --- Test vector ROM ---
    // Each entry: {a_re, a_im, b_re, b_im} in Q1.15
    // Q1.15: 0.5 = 16384, -0.5 = -16384, 0.7 ~= 22938
    localparam N_TESTS = 4;
    localparam signed [15:0]
        // 0.5 + 0i  *  0.5 + 0i  = 0.25 + 0i
        T0_AR = 16'd16384, T0_AI = 16'd0,     T0_BR = 16'd16384, T0_BI = 16'd0,
        // 0.5 + 0.5i * 0.5 - 0.5i = 0.5 + 0i
        T1_AR = 16'd16384, T1_AI = 16'd16384, T1_BR = 16'd16384, T1_BI = -16'd16384,
        // 0 + 0.5i  *  0 + 0.5i  = -0.25 + 0i
        T2_AR = 16'd0,     T2_AI = 16'd16384, T2_BR = 16'd0,     T2_BI = 16'd16384,
        // 0.7 + 0.3i * 0.7 - 0.3i = 0.58 + 0i
        T3_AR = 16'd22938, T3_AI = 16'd9830,  T3_BR = 16'd22938, T3_BI = -16'd9830;

    // Mux test vectors
    reg [1:0] test_idx;
    reg signed [15:0] tv_ar, tv_ai, tv_br, tv_bi;

    always @(*) begin
        case (test_idx)
            2'd0: begin tv_ar = T0_AR; tv_ai = T0_AI; tv_br = T0_BR; tv_bi = T0_BI; end
            2'd1: begin tv_ar = T1_AR; tv_ai = T1_AI; tv_br = T1_BR; tv_bi = T1_BI; end
            2'd2: begin tv_ar = T2_AR; tv_ai = T2_AI; tv_br = T2_BR; tv_bi = T2_BI; end
            2'd3: begin tv_ar = T3_AR; tv_ai = T3_AI; tv_br = T3_BR; tv_bi = T3_BI; end
        endcase
    end

    // --- Debounce ---
    wire s1_pulse;
    debounce deb (
        .clk   (clk),
        .btn_n (btn_s1),
        .pulse (s1_pulse)
    );

    // --- Complex multiplier ---
    reg               mul_valid_in;
    wire signed [15:0] mul_p_re, mul_p_im;
    wire              mul_valid_out;

    complex_mul cmul (
        .clk      (clk),
        .a_re     (tv_ar),
        .a_im     (tv_ai),
        .b_re     (tv_br),
        .b_im     (tv_bi),
        .valid_in (mul_valid_in),
        .p_re     (mul_p_re),
        .p_im     (mul_p_im),
        .valid_out(mul_valid_out)
    );

    // --- UART TX ---
    reg  [7:0] tx_data;
    reg        tx_start;
    wire       tx_busy;

    uart_tx u_tx (
        .clk     (clk),
        .data_in (tx_data),
        .start   (tx_start),
        .busy    (tx_busy),
        .tx      (uart_tx)
    );

    // --- Hex formatting ---
    // Convert a 4-bit nibble to ASCII hex character
    function [7:0] hex_char;
        input [3:0] nibble;
        hex_char = (nibble < 4'd10) ? (8'd48 + {4'd0, nibble})   // '0'-'9'
                                    : (8'd65 + {4'd0, nibble} - 8'd10); // 'A'-'F'
    endfunction

    // --- Output message buffer ---
    // Format: "T0:XXXX,XXXX*XXXX,XXXX=XXXX,XXXX\n" = 35 chars max
    // Use hex for 16-bit values (4 hex chars each)
    localparam MSG_MAX = 36;
    reg [7:0] msg_buf [0:MSG_MAX-1];
    reg [5:0] msg_len;
    reg [5:0] msg_idx;

    // Build message from current test result
    task build_msg;
        input [1:0] idx;
        input signed [15:0] ar, ai, br, bi, pr, pi;
        reg [15:0] u_ar, u_ai, u_br, u_bi, u_pr, u_pi;
        begin
            u_ar = ar; u_ai = ai; u_br = br; u_bi = bi; u_pr = pr; u_pi = pi;
            msg_buf[0]  = "T";
            msg_buf[1]  = 8'd48 + {6'd0, idx}; // '0'-'3'
            msg_buf[2]  = ":";
            // a_re (4 hex)
            msg_buf[3]  = hex_char(u_ar[15:12]);
            msg_buf[4]  = hex_char(u_ar[11:8]);
            msg_buf[5]  = hex_char(u_ar[7:4]);
            msg_buf[6]  = hex_char(u_ar[3:0]);
            msg_buf[7]  = ",";
            // a_im
            msg_buf[8]  = hex_char(u_ai[15:12]);
            msg_buf[9]  = hex_char(u_ai[11:8]);
            msg_buf[10] = hex_char(u_ai[7:4]);
            msg_buf[11] = hex_char(u_ai[3:0]);
            msg_buf[12] = "*";
            // b_re
            msg_buf[13] = hex_char(u_br[15:12]);
            msg_buf[14] = hex_char(u_br[11:8]);
            msg_buf[15] = hex_char(u_br[7:4]);
            msg_buf[16] = hex_char(u_br[3:0]);
            msg_buf[17] = ",";
            // b_im
            msg_buf[18] = hex_char(u_bi[15:12]);
            msg_buf[19] = hex_char(u_bi[11:8]);
            msg_buf[20] = hex_char(u_bi[7:4]);
            msg_buf[21] = hex_char(u_bi[3:0]);
            msg_buf[22] = "=";
            // p_re
            msg_buf[23] = hex_char(u_pr[15:12]);
            msg_buf[24] = hex_char(u_pr[11:8]);
            msg_buf[25] = hex_char(u_pr[7:4]);
            msg_buf[26] = hex_char(u_pr[3:0]);
            msg_buf[27] = ",";
            // p_im
            msg_buf[28] = hex_char(u_pi[15:12]);
            msg_buf[29] = hex_char(u_pi[11:8]);
            msg_buf[30] = hex_char(u_pi[7:4]);
            msg_buf[31] = hex_char(u_pi[3:0]);
            msg_buf[32] = "\n";
            msg_len = 6'd33;
        end
    endtask

    // --- Main FSM ---
    localparam S_IDLE     = 3'd0;
    localparam S_COMPUTE  = 3'd1;  // feed test vector to multiplier
    localparam S_WAIT_MUL = 3'd2;  // wait for pipeline result
    localparam S_BUILD    = 3'd3;  // build output message
    localparam S_SEND     = 3'd4;  // send message bytes over UART
    localparam S_NEXT     = 3'd5;  // advance to next test

    reg [2:0] state;
    reg [1:0] wait_ctr;
    // Latch multiply results
    reg signed [15:0] res_re, res_im;

    initial begin
        state      = S_IDLE;
        test_idx   = 0;
        msg_idx    = 0;
        msg_len    = 0;
        tx_start   = 0;
        tx_data    = 0;
        mul_valid_in = 0;
        led        = 6'b111111;
        wait_ctr   = 0;
        res_re     = 0;
        res_im     = 0;
    end

    always @(posedge clk) begin
        tx_start     <= 1'b0;
        mul_valid_in <= 1'b0;

        case (state)
            S_IDLE: begin
                if (s1_pulse) begin
                    test_idx <= 0;
                    state    <= S_COMPUTE;
                    led[0]   <= ~led[0];
                end
            end

            S_COMPUTE: begin
                mul_valid_in <= 1'b1;
                wait_ctr     <= 0;
                state        <= S_WAIT_MUL;
            end

            S_WAIT_MUL: begin
                // 2-cycle pipeline latency
                wait_ctr <= wait_ctr + 1;
                if (mul_valid_out) begin
                    res_re <= mul_p_re;
                    res_im <= mul_p_im;
                    state  <= S_BUILD;
                end
            end

            S_BUILD: begin
                build_msg(test_idx, tv_ar, tv_ai, tv_br, tv_bi, res_re, res_im);
                msg_idx <= 0;
                state   <= S_SEND;
            end

            S_SEND: begin
                if (!tx_busy && !tx_start) begin
                    if (msg_idx == msg_len) begin
                        state <= S_NEXT;
                    end else begin
                        tx_data  <= msg_buf[msg_idx];
                        tx_start <= 1'b1;
                        msg_idx  <= msg_idx + 1;
                    end
                end
            end

            S_NEXT: begin
                if (test_idx == N_TESTS[1:0] - 1) begin
                    state <= S_IDLE;
                end else begin
                    test_idx <= test_idx + 1;
                    state    <= S_COMPUTE;
                end
            end
        endcase
    end

endmodule
