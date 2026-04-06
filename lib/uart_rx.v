// UART receiver: 8N1, configurable baud rate.
//
// Interface:
//   rx       - serial input (active high, idle high)
//   data_out - received byte (valid when 'valid' pulses)
//   valid    - single-cycle pulse when a byte has been received
//
// 2-FF synchronizer on rx to avoid metastability.

module uart_rx #(
    parameter CLK_HZ = 27_000_000,
    parameter BAUD   = 115_200
) (
    input  wire       clk,
    input  wire       rx,
    output reg  [7:0] data_out,
    output reg        valid
);

    localparam CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam HALF_BIT     = CLKS_PER_BIT / 2;
    localparam CTR_W        = $clog2(CLKS_PER_BIT);

    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    // 2-FF synchronizer
    reg rx_s1, rx_s2;
    always @(posedge clk) begin
        rx_s1 <= rx;
        rx_s2 <= rx_s1;
    end
    wire rx_sync = rx_s2;

    reg [1:0]       state;
    reg [CTR_W-1:0] baud_ctr;
    reg [2:0]       bit_idx;
    reg [7:0]       shift_reg;

    initial begin
        state     = S_IDLE;
        baud_ctr  = 0;
        bit_idx   = 0;
        shift_reg = 0;
        data_out  = 0;
        valid     = 0;
        rx_s1     = 1;
        rx_s2     = 1;
    end

    always @(posedge clk) begin
        valid <= 1'b0;

        case (state)
            S_IDLE: begin
                if (rx_sync == 1'b0) begin
                    baud_ctr <= 0;
                    state    <= S_START;
                end
            end

            S_START: begin
                // Sample at middle of start bit
                if (baud_ctr == HALF_BIT[CTR_W-1:0]) begin
                    if (rx_sync == 1'b0) begin
                        baud_ctr <= 0;
                        bit_idx  <= 0;
                        state    <= S_DATA;
                    end else begin
                        state <= S_IDLE;  // glitch, not a real start bit
                    end
                end else begin
                    baud_ctr <= baud_ctr + 1;
                end
            end

            S_DATA: begin
                if (baud_ctr == CLKS_PER_BIT[CTR_W-1:0] - 1) begin
                    baud_ctr  <= 0;
                    shift_reg <= {rx_sync, shift_reg[7:1]};  // LSB first
                    if (bit_idx == 3'd7)
                        state <= S_STOP;
                    else
                        bit_idx <= bit_idx + 1;
                end else begin
                    baud_ctr <= baud_ctr + 1;
                end
            end

            S_STOP: begin
                if (baud_ctr == CLKS_PER_BIT[CTR_W-1:0] - 1) begin
                    if (rx_sync == 1'b1) begin
                        data_out <= shift_reg;
                        valid    <= 1'b1;
                    end
                    // Return to idle regardless (bad stop bit = drop byte)
                    state <= S_IDLE;
                end else begin
                    baud_ctr <= baud_ctr + 1;
                end
            end
        endcase
    end

endmodule
