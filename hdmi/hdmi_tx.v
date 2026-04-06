// Minimal HDMI/DVI transmitter using Gowin OSER10 + TLVDS_OBUF.
//
// Takes RGB + sync signals at pixel clock rate, TMDS-encodes them,
// and serializes to 4 LVDS differential pairs (3 data + 1 clock).
//
// Uses Gowin OSER10 primitive for 10:1 serialization.
// serial_clk must be exactly 5x pix_clk (provided by CLKDIV).

module hdmi_tx (
    input  wire        pix_clk,     // pixel clock (25.2 MHz)
    input  wire        serial_clk,  // 5x pixel clock (126 MHz)
    input  wire        rst_n,
    input  wire [7:0]  r,
    input  wire [7:0]  g,
    input  wire [7:0]  b,
    input  wire        hsync,
    input  wire        vsync,
    input  wire        de,
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_d_p,
    output wire [2:0]  tmds_d_n
    // Constraints use pair format: IO_LOC "tmds_clk_p" 33,34;
    // Only P names appear in .cst; TLVDS_OBUF handles both pins.
);

    // TMDS encode each channel
    // Blue channel carries hsync/vsync as control signals
    wire [9:0] tmds_b, tmds_g, tmds_r;

    tmds_encoder enc_b (
        .clk   (pix_clk),
        .rst_n (rst_n),
        .din   (b),
        .de    (de),
        .c0    (~hsync),  // active-low sync -> active-high control
        .c1    (~vsync),
        .dout  (tmds_b)
    );

    tmds_encoder enc_g (
        .clk   (pix_clk),
        .rst_n (rst_n),
        .din   (g),
        .de    (de),
        .c0    (1'b0),
        .c1    (1'b0),
        .dout  (tmds_g)
    );

    tmds_encoder enc_r (
        .clk   (pix_clk),
        .rst_n (rst_n),
        .din   (r),
        .de    (de),
        .c0    (1'b0),
        .c1    (1'b0),
        .dout  (tmds_r)
    );

    // TMDS clock channel: constant 10'b0000011111 (5 low, 5 high = clock pattern)
    wire [9:0] tmds_clk = 10'b0000011111;

    // Serialize each channel using OSER10
    wire [3:0] ser_out;  // single-ended serialized outputs

    // Channel 0: Blue
    OSER10 ser_b (
        .Q     (ser_out[0]),
        .D0    (tmds_b[0]),
        .D1    (tmds_b[1]),
        .D2    (tmds_b[2]),
        .D3    (tmds_b[3]),
        .D4    (tmds_b[4]),
        .D5    (tmds_b[5]),
        .D6    (tmds_b[6]),
        .D7    (tmds_b[7]),
        .D8    (tmds_b[8]),
        .D9    (tmds_b[9]),
        .PCLK  (pix_clk),
        .FCLK  (serial_clk),
        .RESET (~rst_n)
    );

    // Channel 1: Green
    OSER10 ser_g (
        .Q     (ser_out[1]),
        .D0    (tmds_g[0]),
        .D1    (tmds_g[1]),
        .D2    (tmds_g[2]),
        .D3    (tmds_g[3]),
        .D4    (tmds_g[4]),
        .D5    (tmds_g[5]),
        .D6    (tmds_g[6]),
        .D7    (tmds_g[7]),
        .D8    (tmds_g[8]),
        .D9    (tmds_g[9]),
        .PCLK  (pix_clk),
        .FCLK  (serial_clk),
        .RESET (~rst_n)
    );

    // Channel 2: Red
    OSER10 ser_r (
        .Q     (ser_out[2]),
        .D0    (tmds_r[0]),
        .D1    (tmds_r[1]),
        .D2    (tmds_r[2]),
        .D3    (tmds_r[3]),
        .D4    (tmds_r[4]),
        .D5    (tmds_r[5]),
        .D6    (tmds_r[6]),
        .D7    (tmds_r[7]),
        .D8    (tmds_r[8]),
        .D9    (tmds_r[9]),
        .PCLK  (pix_clk),
        .FCLK  (serial_clk),
        .RESET (~rst_n)
    );

    // Clock channel
    OSER10 ser_clk (
        .Q     (ser_out[3]),
        .D0    (tmds_clk[0]),
        .D1    (tmds_clk[1]),
        .D2    (tmds_clk[2]),
        .D3    (tmds_clk[3]),
        .D4    (tmds_clk[4]),
        .D5    (tmds_clk[5]),
        .D6    (tmds_clk[6]),
        .D7    (tmds_clk[7]),
        .D8    (tmds_clk[8]),
        .D9    (tmds_clk[9]),
        .PCLK  (pix_clk),
        .FCLK  (serial_clk),
        .RESET (~rst_n)
    );

    // LVDS output buffers (True LVDS): both O and OB must be top-level ports
    TLVDS_OBUF obuf_b   (.I(ser_out[0]), .O(tmds_d_p[0]),   .OB(tmds_d_n[0]));
    TLVDS_OBUF obuf_g   (.I(ser_out[1]), .O(tmds_d_p[1]),   .OB(tmds_d_n[1]));
    TLVDS_OBUF obuf_r   (.I(ser_out[2]), .O(tmds_d_p[2]),   .OB(tmds_d_n[2]));
    TLVDS_OBUF obuf_clk (.I(ser_out[3]), .O(tmds_clk_p),    .OB(tmds_clk_n));

endmodule
