// Simple Bloch sphere renderer.
//
// Draws a circle (sphere outline) and a dot (state point) on a
// 640x480 framebuffer. Uses only integer arithmetic.
//
// The density matrix state is projected to Bloch coordinates:
//   x = 2*Re(rho[01]) -> maps to horizontal position
//   y = 2*Im(rho[01]) -> maps to depth (used for dot size/color)
//   z = rho[00] - rho[11] -> maps to vertical position
//
// For d=3 vectorized rho: rho[00]=vec[0], rho[01]=vec[1], rho[11]=vec[4]
// All in Q1.15 format.
//
// Display layout:
//   - Sphere circle centered at (320, 240), radius 180 pixels
//   - Dot position: x_dot = 320 + bloch_x * 180, y_dot = 240 - bloch_z * 180
//   - Background: dark blue (#101830)
//   - Circle: white outline, 2px thick
//   - Dot: bright green, radius 6px
//   - Axes: dim gray crosshairs

module bloch_renderer (
    input  wire        clk,       // pixel clock
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,
    input  wire        de,

    // Bloch state from matvec (Q1.15, updated once per frame)
    input  wire signed [15:0] bloch_x,  // 2*Re(rho[01])
    input  wire signed [15:0] bloch_z,  // rho[00] - rho[11]
    input  wire signed [15:0] bloch_y,  // 2*Im(rho[01]) -- depth cue

    output reg  [7:0]  r,
    output reg  [7:0]  g,
    output reg  [7:0]  b
);

    // Sphere center and radius
    localparam CX = 320;
    localparam CY = 240;
    localparam R  = 180;
    localparam R2 = R * R;  // 32400

    // Dot radius
    localparam DOT_R = 6;
    localparam DOT_R2 = DOT_R * DOT_R;  // 36

    // Compute dot position from Bloch coordinates
    // bloch_x, bloch_z are Q1.15: range [-1, 1)
    // Multiply by R and shift: pos = center + (bloch * R) >> 15
    wire signed [31:0] dot_x_offset = $signed(bloch_x) * $signed(R);
    wire signed [31:0] dot_z_offset = $signed(bloch_z) * $signed(R);

    wire [9:0] dot_x = CX + dot_x_offset[30:15];  // center + offset
    wire [9:0] dot_y = CY - dot_z_offset[30:15];   // inverted: +z = up

    // Distance calculations (relative to current pixel, pipelined)
    wire signed [10:0] dx_c = $signed({1'b0, pixel_x}) - $signed(11'd320);
    wire signed [10:0] dy_c = $signed({1'b0, pixel_y}) - $signed(11'd240);
    wire [21:0] dist2_center = dx_c * dx_c + dy_c * dy_c;

    wire signed [10:0] dx_d = $signed({1'b0, pixel_x}) - $signed({1'b0, dot_x});
    wire signed [10:0] dy_d = $signed({1'b0, pixel_y}) - $signed({1'b0, dot_y});
    wire [21:0] dist2_dot = dx_d * dx_d + dy_d * dy_d;

    // Circle band: |dist - R| < 2 pixels
    // Equivalent: (R-2)^2 < dist2 < (R+2)^2
    localparam R_INNER2 = (R - 2) * (R - 2);  // 31684
    localparam R_OUTER2 = (R + 2) * (R + 2);  // 33124

    // Axis lines (1px wide crosshairs through center)
    wire on_h_axis = (pixel_y >= CY - 0 && pixel_y <= CY + 0) &&
                     (pixel_x >= CX - R && pixel_x <= CX + R);
    wire on_v_axis = (pixel_x >= CX - 0 && pixel_x <= CX + 0) &&
                     (pixel_y >= CY - R && pixel_y <= CY + R);

    // Pixel classification
    wire on_circle = (dist2_center > R_INNER2) && (dist2_center < R_OUTER2);
    wire on_dot    = (dist2_dot < DOT_R2);
    wire on_axis   = on_h_axis || on_v_axis;
    wire inside    = (dist2_center < R2);

    // Label rendering: |0> at top, |1> at bottom
    // Simple: bright pixels at specific locations
    wire at_label_0 = (pixel_x >= 310 && pixel_x <= 330 &&
                       pixel_y >= CY - R - 20 && pixel_y <= CY - R - 8);
    wire at_label_1 = (pixel_x >= 310 && pixel_x <= 330 &&
                       pixel_y >= CY + R + 8 && pixel_y <= CY + R + 20);

    // Color output
    always @(posedge clk) begin
        if (!de) begin
            r <= 8'd0; g <= 8'd0; b <= 8'd0;
        end else if (on_dot) begin
            // State dot: bright green
            r <= 8'd50; g <= 8'd255; b <= 8'd50;
        end else if (on_circle) begin
            // Sphere outline: white
            r <= 8'd200; g <= 8'd200; b <= 8'd220;
        end else if (on_axis && inside) begin
            // Axis crosshairs: dim gray
            r <= 8'd60; g <= 8'd60; b <= 8'd80;
        end else if (at_label_0 || at_label_1) begin
            // Labels: dim yellow
            r <= 8'd120; g <= 8'd120; b <= 8'd60;
        end else if (inside) begin
            // Inside sphere: very dark blue
            r <= 8'd12; g <= 8'd16; b <= 8'd32;
        end else begin
            // Background: dark
            r <= 8'd8; g <= 8'd10; b <= 8'd20;
        end
    end

endmodule
