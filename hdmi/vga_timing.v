// VGA timing generator for 640x480 @ ~60Hz.
//
// Pixel clock: 25.2 MHz (25.175 MHz nominal, 25.2 is within spec)
// Horizontal: 640 visible + 16 front porch + 96 sync + 48 back porch = 800
// Vertical:   480 visible + 10 front porch + 2 sync + 33 back porch = 525

module vga_timing (
    input  wire        clk,       // pixel clock (25.2 MHz)
    input  wire        rst_n,
    output reg         hsync,
    output reg         vsync,
    output reg         de,        // data enable (active video)
    output reg  [9:0]  pixel_x,   // current pixel X (0-639)
    output reg  [9:0]  pixel_y    // current pixel Y (0-479)
);

    // 640x480 timings (all active-low sync polarity)
    localparam H_VISIBLE = 10'd640;
    localparam H_FRONT   = 10'd16;
    localparam H_SYNC    = 10'd96;
    localparam H_BACK    = 10'd48;
    localparam H_TOTAL   = 10'd800;

    localparam V_VISIBLE = 10'd480;
    localparam V_FRONT   = 10'd10;
    localparam V_SYNC    = 10'd2;
    localparam V_BACK    = 10'd33;
    localparam V_TOTAL   = 10'd525;

    reg [9:0] h_cnt;
    reg [9:0] v_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt   <= 10'd0;
            v_cnt   <= 10'd0;
            hsync   <= 1'b1;
            vsync   <= 1'b1;
            de      <= 1'b0;
            pixel_x <= 10'd0;
            pixel_y <= 10'd0;
        end else begin
            // Horizontal counter
            if (h_cnt == H_TOTAL - 1) begin
                h_cnt <= 10'd0;
                // Vertical counter
                if (v_cnt == V_TOTAL - 1)
                    v_cnt <= 10'd0;
                else
                    v_cnt <= v_cnt + 1;
            end else begin
                h_cnt <= h_cnt + 1;
            end

            // Sync signals (active low for 640x480)
            hsync <= ~(h_cnt >= H_VISIBLE + H_FRONT &&
                       h_cnt <  H_VISIBLE + H_FRONT + H_SYNC);
            vsync <= ~(v_cnt >= V_VISIBLE + V_FRONT &&
                       v_cnt <  V_VISIBLE + V_FRONT + V_SYNC);

            // Data enable
            de <= (h_cnt < H_VISIBLE) && (v_cnt < V_VISIBLE);

            // Pixel coordinates
            pixel_x <= h_cnt;
            pixel_y <= v_cnt[9:0];
        end
    end

endmodule
