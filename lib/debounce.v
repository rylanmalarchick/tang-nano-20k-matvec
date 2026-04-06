// Button debouncer: outputs a single-cycle pulse on the falling edge
// of an active-low button input, after the signal has been stable for
// DEBOUNCE_MS milliseconds.
//
// At 27 MHz, 10 ms = 270,000 cycles -> needs 18-bit counter.

module debounce #(
    parameter CLK_HZ    = 27_000_000,
    parameter DEBOUNCE_MS = 10
) (
    input  wire clk,
    input  wire btn_n,    // active-low raw button input
    output reg  pulse     // single-cycle high pulse on press
);

    localparam THRESHOLD = CLK_HZ / 1000 * DEBOUNCE_MS;
    localparam CTR_W     = $clog2(THRESHOLD + 1);

    reg [CTR_W-1:0] counter;
    reg              btn_stable;

    initial begin
        counter    = 0;
        btn_stable = 1'b1;  // not pressed
        pulse      = 1'b0;
    end

    always @(posedge clk) begin
        pulse <= 1'b0;

        if (btn_n != btn_stable) begin
            counter <= counter + 1;
            if (counter == THRESHOLD[CTR_W-1:0]) begin
                btn_stable <= btn_n;
                counter    <= 0;
                // Emit pulse on press (high -> low transition)
                if (btn_n == 1'b0)
                    pulse <= 1'b1;
            end
        end else begin
            counter <= 0;
        end
    end

endmodule
