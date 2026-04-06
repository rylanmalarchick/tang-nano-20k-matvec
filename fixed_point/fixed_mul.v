// Q1.15 signed fixed-point multiplier.
//
// Format: 1 sign bit + 15 fractional bits = 16-bit signed.
//   Value = integer_repr / 2^15.  Range: [-1.0, +0.99997].
//
// Arithmetic:
//   product_32 = a * b                (32-bit signed)
//   result     = product_32 >> 15     (back to Q1.15)
//   i.e., result = product_32[30:15]
//
// DSP18 inference: Yosys maps the 16x16 signed multiply to a single
// MULT18X18 block (inputs are sign-extended to 18 bits internally).
// At 27 MHz this is purely combinational through the DSP; we register
// the output for timing closure at higher frequencies.

module fixed_mul (
    input  wire        clk,
    input  wire signed [15:0] a,
    input  wire signed [15:0] b,
    input  wire        valid_in,
    output reg  signed [15:0] result,
    output reg         valid_out
);

    // Full-precision product: Q2.30 in 32 bits
    wire signed [31:0] product = a * b;

    // Register the Q1.15 extraction for one-cycle latency
    always @(posedge clk) begin
        result    <= product[30:15];
        valid_out <= valid_in;
    end

endmodule
