// UART transmitter: 8N1, configurable baud rate.
//
// Interface:
//   data_in[7:0] - byte to send (latched on start)
//   start        - pulse high for one cycle to begin transmission
//   busy         - high while transmitting (do not assert start)
//   tx           - serial output line (active high, idle high)
//
// At 27 MHz / 115200 baud = 234.375 -> use 234 cycles per bit.
// Actual baud = 27e6/234 = 115384 (0.16% error, well within spec).

module uart_tx #(
    parameter CLK_HZ  = 27_000_000,
    parameter BAUD    = 115_200
) (
    input  wire       clk,
    input  wire [7:0] data_in,
    input  wire       start,
    output reg        busy,
    output reg        tx
);

    localparam CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam CTR_W        = $clog2(CLKS_PER_BIT);

    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    reg [1:0]        state;
    reg [CTR_W-1:0]  baud_ctr;
    reg [2:0]        bit_idx;
    reg [7:0]        shift_reg;

    initial begin
        state     = S_IDLE;
        baud_ctr  = 0;
        bit_idx   = 0;
        shift_reg = 0;
        busy      = 1'b0;
        tx        = 1'b1;  // idle high
    end

    always @(posedge clk) begin
        case (state)
            S_IDLE: begin
                tx   <= 1'b1;
                busy <= 1'b0;
                if (start) begin
                    shift_reg <= data_in;
                    busy      <= 1'b1;
                    baud_ctr  <= 0;
                    state     <= S_START;
                end
            end

            S_START: begin
                tx <= 1'b0;  // start bit
                if (baud_ctr == CLKS_PER_BIT[CTR_W-1:0] - 1) begin
                    baud_ctr <= 0;
                    bit_idx  <= 0;
                    state    <= S_DATA;
                end else begin
                    baud_ctr <= baud_ctr + 1;
                end
            end

            S_DATA: begin
                tx <= shift_reg[0];  // LSB first
                if (baud_ctr == CLKS_PER_BIT[CTR_W-1:0] - 1) begin
                    baud_ctr  <= 0;
                    shift_reg <= {1'b0, shift_reg[7:1]};
                    if (bit_idx == 3'd7) begin
                        state <= S_STOP;
                    end else begin
                        bit_idx <= bit_idx + 1;
                    end
                end else begin
                    baud_ctr <= baud_ctr + 1;
                end
            end

            S_STOP: begin
                tx <= 1'b1;  // stop bit
                if (baud_ctr == CLKS_PER_BIT[CTR_W-1:0] - 1) begin
                    state <= S_IDLE;
                end else begin
                    baud_ctr <= baud_ctr + 1;
                end
            end
        endcase
    end

endmodule
