// Blink all 6 LEDs at ~1 Hz on the Tang Nano 20K
// 27 MHz clock -> toggle every 13,500,000 cycles

module blink (
    input  wire clk,
    input  wire btn_s1,
    input  wire btn_s2,
    output reg  [5:0] led
);

    // 27_000_000 / 2 = 13_500_000 -> need 24 bits
    reg [23:0] counter;

    initial begin
        counter = 0;
        led = 6'b111111; // all off (active low)
    end

    always @(posedge clk) begin
        counter <= counter + 1;

        if (counter == 0)
            led <= ~led;
    end

endmodule
