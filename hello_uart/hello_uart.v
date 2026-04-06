// Top-level: press S1 -> send "Hello\n" over UART TX.
// LED[0] toggles on each button press as visual feedback.

module hello_uart (
    input  wire       clk,
    input  wire       btn_s1,
    input  wire       btn_s2,
    output reg  [5:0] led,
    output wire       uart_tx
);

    // Message ROM: "Hello\n"
    localparam MSG_LEN = 6;
    reg [7:0] msg [0:MSG_LEN-1];
    initial begin
        msg[0] = "H";
        msg[1] = "e";
        msg[2] = "l";
        msg[3] = "l";
        msg[4] = "o";
        msg[5] = 8'h0A;  // '\n'
    end

    // Debounce S1
    wire s1_pulse;
    debounce deb (
        .clk   (clk),
        .btn_n (btn_s1),
        .pulse (s1_pulse)
    );

    // UART TX
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

    // FSM: idle -> sending bytes
    localparam S_IDLE = 1'b0;
    localparam S_SEND = 1'b1;

    reg        state;
    reg  [2:0] char_idx;

    initial begin
        state    = S_IDLE;
        char_idx = 0;
        tx_data  = 0;
        tx_start = 0;
        led      = 6'b111111;  // all off
    end

    always @(posedge clk) begin
        tx_start <= 1'b0;

        case (state)
            S_IDLE: begin
                if (s1_pulse) begin
                    char_idx <= 0;
                    state    <= S_SEND;
                    led[0]   <= ~led[0];  // toggle LED as feedback
                end
            end

            S_SEND: begin
                if (!tx_busy && !tx_start) begin
                    if (char_idx == MSG_LEN[2:0]) begin
                        state <= S_IDLE;
                    end else begin
                        tx_data  <= msg[char_idx];
                        tx_start <= 1'b1;
                        char_idx <= char_idx + 1;
                    end
                end
            end
        endcase
    end

endmodule
