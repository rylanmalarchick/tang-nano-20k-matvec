// Pipelined complex multiplier in Q1.15 fixed-point.
//
// Computes (a_re + a_im*i) * (b_re + b_im*i):
//   p_re = a_re*b_re - a_im*b_im
//   p_im = a_re*b_im + a_im*b_re
//
// Pipeline (2-cycle latency, 1 result/cycle throughput):
//   Stage 1: 4 parallel multiplies (4 DSP18 blocks)
//   Stage 2: subtract and add
//
// The add/subtract can overflow Q1.15 when both products are near +1.
// For CPTP propagator elements (|P_ij| <= 1), this won't happen in
// practice. We don't saturate here -- the caller (matvec accumulator)
// will use wider accumulators.

module complex_mul (
    input  wire        clk,
    input  wire signed [15:0] a_re,
    input  wire signed [15:0] a_im,
    input  wire signed [15:0] b_re,
    input  wire signed [15:0] b_im,
    input  wire        valid_in,
    output reg  signed [15:0] p_re,
    output reg  signed [15:0] p_im,
    output reg         valid_out
);

    // Stage 1: four parallel multiplies (each infers one DSP18)
    // Full 32-bit products in Q2.30
    reg signed [31:0] ac, bd, ad, bc;
    reg               valid_s1;

    always @(posedge clk) begin
        ac       <= a_re * b_re;
        bd       <= a_im * b_im;
        ad       <= a_re * b_im;
        bc       <= a_im * b_re;
        valid_s1 <= valid_in;
    end

    // Stage 2: combine and extract Q1.15
    // Subtract/add in Q2.30, then take bits [30:15] for Q1.15
    wire signed [31:0] sum_re = ac - bd;
    wire signed [31:0] sum_im = ad + bc;

    always @(posedge clk) begin
        p_re      <= sum_re[30:15];
        p_im      <= sum_im[30:15];
        valid_out <= valid_s1;
    end

endmodule
