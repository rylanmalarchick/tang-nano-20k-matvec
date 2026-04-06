// TMDS 8b/10b encoder for one DVI/HDMI channel.
//
// Implements the encoding algorithm from DVI 1.0 spec section 3.2.
// During active video (de=1): encodes 8-bit pixel data to 10-bit TMDS.
// During blanking (de=0): sends control tokens based on c0/c1.
//
// Control tokens:
//   c1 c0  ->  10-bit token
//    0  0  ->  10'b1101010100
//    0  1  ->  10'b0010101011
//    1  0  ->  10'b0101010100
//    1  1  ->  10'b1010101011

module tmds_encoder (
    input  wire       clk,       // pixel clock
    input  wire       rst_n,
    input  wire [7:0] din,       // pixel data
    input  wire       de,        // data enable (active video)
    input  wire       c0,        // control bit 0 (hsync for blue)
    input  wire       c1,        // control bit 1 (vsync for blue)
    output reg  [9:0] dout       // 10-bit TMDS encoded
);

    // Count number of 1s in input
    wire [3:0] n_ones = din[0] + din[1] + din[2] + din[3]
                      + din[4] + din[5] + din[6] + din[7];

    // Step 1: Transition-minimized encoding (XOR or XNOR)
    wire use_xnor = (n_ones > 4) || (n_ones == 4 && din[0] == 0);
    wire [8:0] q_m;

    assign q_m[0] = din[0];
    assign q_m[1] = use_xnor ? ~(q_m[0] ^ din[1]) : (q_m[0] ^ din[1]);
    assign q_m[2] = use_xnor ? ~(q_m[1] ^ din[2]) : (q_m[1] ^ din[2]);
    assign q_m[3] = use_xnor ? ~(q_m[2] ^ din[3]) : (q_m[2] ^ din[3]);
    assign q_m[4] = use_xnor ? ~(q_m[3] ^ din[4]) : (q_m[3] ^ din[4]);
    assign q_m[5] = use_xnor ? ~(q_m[4] ^ din[5]) : (q_m[4] ^ din[5]);
    assign q_m[6] = use_xnor ? ~(q_m[5] ^ din[6]) : (q_m[5] ^ din[6]);
    assign q_m[7] = use_xnor ? ~(q_m[6] ^ din[7]) : (q_m[6] ^ din[7]);
    assign q_m[8] = use_xnor ? 1'b0 : 1'b1;

    // Count 1s and 0s in q_m[7:0]
    wire [3:0] n_q1 = q_m[0] + q_m[1] + q_m[2] + q_m[3]
                    + q_m[4] + q_m[5] + q_m[6] + q_m[7];
    wire [3:0] n_q0 = 4'd8 - n_q1;

    // DC balance counter (signed)
    reg signed [4:0] disparity;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dout      <= 10'b1101010100;
            disparity <= 5'sd0;
        end else if (!de) begin
            // Control period: send control tokens, reset disparity
            disparity <= 5'sd0;
            case ({c1, c0})
                2'b00: dout <= 10'b1101010100;
                2'b01: dout <= 10'b0010101011;
                2'b10: dout <= 10'b0101010100;
                2'b11: dout <= 10'b1010101011;
            endcase
        end else begin
            // Data period: DC-balanced encoding
            if (disparity == 5'sd0 || n_q1 == 4'd4) begin
                // No disparity or balanced q_m
                dout[9]   <= ~q_m[8];
                dout[8]   <= q_m[8];
                dout[7:0] <= q_m[8] ? q_m[7:0] : ~q_m[7:0];
                if (q_m[8] == 1'b0)
                    disparity <= disparity + ($signed({1'b0, n_q0}) - $signed({1'b0, n_q1}));
                else
                    disparity <= disparity + ($signed({1'b0, n_q1}) - $signed({1'b0, n_q0}));
            end else begin
                if ((disparity > 0 && n_q1 > 4'd4) ||
                    (disparity < 0 && n_q1 < 4'd4)) begin
                    // Need to invert to reduce disparity
                    dout[9]   <= 1'b1;
                    dout[8]   <= q_m[8];
                    dout[7:0] <= ~q_m[7:0];
                    disparity <= disparity + {q_m[8], 1'b0}
                               + ($signed({1'b0, n_q0}) - $signed({1'b0, n_q1}));
                end else begin
                    // Keep as-is
                    dout[9]   <= 1'b0;
                    dout[8]   <= q_m[8];
                    dout[7:0] <= q_m[7:0];
                    disparity <= disparity - {~q_m[8], 1'b0}
                               + ($signed({1'b0, n_q1}) - $signed({1'b0, n_q0}));
                end
            end
        end
    end

endmodule
