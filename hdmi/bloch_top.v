// Bloch sphere HDMI visualization top-level.
//
// Integrates:
//   - Matvec core (9x9 complex, Q1.15)
//   - UART host interface (load P, load rho, step)
//   - HDMI output with Bloch sphere rendering
//
// Clock architecture:
//   27 MHz crystal -> rPLL -> 126 MHz serial_clk
//                          -> CLKDIV /5 -> 25.2 MHz pix_clk
//   Matvec + UART run on pix_clk (25.2 MHz)
//   HDMI serialization on serial_clk (126 MHz)
//
// PLL: 27 * 14 / 3 = 126 MHz (IDIV=2, FBDIV=13, ODIV=4, VCO=504)

module bloch_top (
    input  wire       clk,         // 27 MHz crystal
    input  wire       btn_s1,
    input  wire       btn_s2,
    output wire [5:0] led,
    output wire       uart_tx,
    input  wire       uart_rx,
    output wire       tmds_clk_p,
    output wire       tmds_clk_n,
    output wire [2:0] tmds_d_p,
    output wire [2:0] tmds_d_n
);

    // =================================================================
    // PLL: 27 MHz -> 126 MHz
    // =================================================================
    wire serial_clk, pll_lock;

    rPLL #(
        .FCLKIN       ("27"),
        .IDIV_SEL     (2),        // ÷3
        .FBDIV_SEL    (13),       // ×14 -> 27*14/3 = 126 MHz
        .ODIV_SEL     (4),        // VCO = 504 MHz
        .DEVICE       ("GW2AR-18C"),
        .DYN_IDIV_SEL ("false"),
        .DYN_FBDIV_SEL("false"),
        .DYN_ODIV_SEL ("false"),
        .CLKFB_SEL    ("internal"),
        .CLKOUT_BYPASS("false"),
        .CLKOUTP_BYPASS("false"),
        .CLKOUTD_BYPASS("false"),
        .CLKOUTD_SRC  ("CLKOUT"),
        .CLKOUTD3_SRC ("CLKOUT"),
        .DYN_SDIV_SEL (2)
    ) u_pll (
        .CLKIN    (clk),
        .CLKOUT   (serial_clk),
        .CLKOUTP  (),
        .CLKOUTD  (),
        .CLKOUTD3 (),
        .LOCK     (pll_lock),
        .RESET    (1'b0),
        .RESET_P  (1'b0),
        .CLKFB    (1'b0),
        .FBDSEL   (6'b0),
        .IDSEL    (6'b0),
        .ODSEL    (6'b0),
        .PSDA     (4'b0),
        .DUTYDA   (4'b0),
        .FDLY     (4'b0)
    );

    // =================================================================
    // CLKDIV: 126 / 5 = 25.2 MHz pixel clock
    // =================================================================
    wire pix_clk;

    // Synchronize PLL lock deassertion to pix_clk (2-FF synchronizer).
    // Assert reset asynchronously (pll_lock low), deassert synchronously.
    reg rst_sync0, rst_sync1;
    always @(posedge pix_clk or negedge pll_lock) begin
        if (!pll_lock) begin
            rst_sync0 <= 1'b0;
            rst_sync1 <= 1'b0;
        end else begin
            rst_sync0 <= 1'b1;
            rst_sync1 <= rst_sync0;
        end
    end
    wire rst_n = rst_sync1;

    CLKDIV u_clkdiv (
        .RESETN (rst_n),
        .HCLKIN (serial_clk),
        .CLKOUT (pix_clk),
        .CALIB  (1'b1)
    );
    defparam u_clkdiv.DIV_MODE = "5";
    defparam u_clkdiv.GSREN    = "false";

    // =================================================================
    // Matvec + UART (runs on pixel clock)
    // =================================================================
    wire [5:0] inner_led;

    // Bloch-relevant rho elements from matvec core
    wire signed [15:0] rho00_re, rho01_re, rho01_im, rho11_re;

    // Demo sideband signals
    reg        demo_p_wr_en;
    reg [6:0]  demo_p_wr_addr;
    reg signed [15:0] demo_p_wr_re, demo_p_wr_im;
    reg        demo_rho_wr_en;
    reg [3:0]  demo_rho_wr_addr;
    reg signed [15:0] demo_rho_wr_re, demo_rho_wr_im;
    reg        demo_start;
    reg [15:0] demo_n_steps;
    wire       core_done;

    matvec_top #(
        .CLK_HZ(25_200_000)
    ) u_matvec (
        .clk     (pix_clk),
        .btn_s1  (btn_s1),
        .btn_s2  (btn_s2),
        .led     (inner_led),
        .uart_tx (uart_tx),
        .uart_rx (uart_rx),
        .bloch_rho00_re (rho00_re),
        .bloch_rho01_re (rho01_re),
        .bloch_rho01_im (rho01_im),
        .bloch_rho11_re (rho11_re),
        .demo_p_wr_en    (demo_p_wr_en),
        .demo_p_wr_addr  (demo_p_wr_addr),
        .demo_p_wr_re    (demo_p_wr_re),
        .demo_p_wr_im    (demo_p_wr_im),
        .demo_rho_wr_en  (demo_rho_wr_en),
        .demo_rho_wr_addr(demo_rho_wr_addr),
        .demo_rho_wr_re  (demo_rho_wr_re),
        .demo_rho_wr_im  (demo_rho_wr_im),
        .demo_start      (demo_start),
        .demo_n_steps    (demo_n_steps),
        .core_done       (core_done)
    );

    assign led = {~pll_lock, inner_led[4:0]};

    // =================================================================
    // Button debounce
    // =================================================================
    wire s1_pulse, s2_pulse;

    debounce #(.CLK_HZ(25_200_000)) db_s1 (
        .clk   (pix_clk),
        .btn_n (btn_s1),
        .pulse (s1_pulse)
    );

    debounce #(.CLK_HZ(25_200_000)) db_s2 (
        .clk   (pix_clk),
        .btn_n (btn_s2),
        .pulse (s2_pulse)
    );

    // =================================================================
    // Demo controller: Y-axis Rabi oscillation
    // =================================================================
    // S1: load propagator + rho, start auto-stepping
    // S2: pause/resume
    //
    // Propagator: U = Ry(pi/30) per step, full cycle in 60 frames (1s)
    //   U = [[c, -s], [s, c]]  c=cos(pi/60), s=sin(pi/60)
    //   Superoperator P = U*⊗U (all real since U is real)
    //   Embedded in 9x9 with identity for level 3
    //
    // Initial rho: |0⟩⟨0| = diag(1,0,...,0)
    // Bloch: starts at north pole, traces great circle in x-z plane

    localparam DEMO_IDLE     = 3'd0;
    localparam DEMO_LOAD_P   = 3'd1;
    localparam DEMO_LOAD_RHO = 3'd2;
    localparam DEMO_RUNNING  = 3'd3;
    localparam DEMO_STEPPING = 3'd4;

    reg [2:0] demo_state;
    reg [6:0] demo_addr;
    reg       demo_paused;

    // Q1.15 constants for Ry(pi/30)
    // c = cos(pi/60) ≈ 0.99863,  s = sin(pi/60) ≈ 0.05234
    // c^2 = 32678, s^2 = 89, cs = 1713  (c^2+s^2 = 32767)
    localparam signed [15:0] C2   =  16'sd32678;  // c^2
    localparam signed [15:0] S2   =  16'sd89;     // s^2
    localparam signed [15:0] CS   =  16'sd1713;   // c*s
    localparam signed [15:0] NCS  = -16'sd1713;   // -c*s
    localparam signed [15:0] NS2  = -16'sd89;     // -s^2
    localparam signed [15:0] ONE  =  16'sd32767;  // ~1.0
    localparam signed [15:0] ZERO =  16'sd0;

    // Propagator ROM: returns (re, im) for address 0..80
    // P = U*⊗U for qubit block + identity for level 3
    reg signed [15:0] rom_re, rom_im;

    always @(*) begin
        rom_im = ZERO;  // all entries are real
        case (demo_addr)
            7'd0:  rom_re = C2;     // P[0,0] = c^2
            7'd1:  rom_re = NCS;    // P[0,1] = -cs
            7'd3:  rom_re = NCS;    // P[0,3] = -cs
            7'd4:  rom_re = S2;     // P[0,4] = s^2
            7'd9:  rom_re = CS;     // P[1,0] = cs
            7'd10: rom_re = C2;     // P[1,1] = c^2
            7'd12: rom_re = NS2;    // P[1,3] = -s^2
            7'd13: rom_re = NCS;    // P[1,4] = -cs
            7'd20: rom_re = ONE;    // P[2,2] = 1 (level 3)
            7'd27: rom_re = CS;     // P[3,0] = cs
            7'd28: rom_re = NS2;    // P[3,1] = -s^2
            7'd30: rom_re = C2;     // P[3,3] = c^2
            7'd31: rom_re = NCS;    // P[3,4] = -cs
            7'd36: rom_re = S2;     // P[4,0] = s^2
            7'd37: rom_re = CS;     // P[4,1] = cs
            7'd39: rom_re = CS;     // P[4,3] = cs
            7'd40: rom_re = C2;     // P[4,4] = c^2
            7'd50: rom_re = ONE;    // P[5,5] = 1
            7'd60: rom_re = ONE;    // P[6,6] = 1
            7'd70: rom_re = ONE;    // P[7,7] = 1
            7'd80: rom_re = ONE;    // P[8,8] = 1
            default: rom_re = ZERO;
        endcase
    end

    initial begin
        demo_state     = DEMO_IDLE;
        demo_p_wr_en   = 0;
        demo_rho_wr_en = 0;
        demo_start     = 0;
        demo_n_steps   = 16'd1;
        demo_addr      = 0;
        demo_paused    = 0;
    end

    always @(posedge pix_clk) begin
        demo_p_wr_en   <= 1'b0;
        demo_rho_wr_en <= 1'b0;
        demo_start     <= 1'b0;

        case (demo_state)
            DEMO_IDLE: begin
                if (s1_pulse) begin
                    demo_addr  <= 7'd0;
                    demo_paused <= 1'b0;
                    demo_state <= DEMO_LOAD_P;
                end
            end

            DEMO_LOAD_P: begin
                demo_p_wr_en   <= 1'b1;
                demo_p_wr_addr <= demo_addr;
                demo_p_wr_re   <= rom_re;
                demo_p_wr_im   <= rom_im;
                if (demo_addr == 7'd80) begin
                    demo_addr  <= 7'd0;
                    demo_state <= DEMO_LOAD_RHO;
                end else begin
                    demo_addr <= demo_addr + 1;
                end
            end

            DEMO_LOAD_RHO: begin
                demo_rho_wr_en   <= 1'b1;
                demo_rho_wr_addr <= demo_addr[3:0];
                demo_rho_wr_re   <= (demo_addr == 7'd0) ? ONE : ZERO;
                demo_rho_wr_im   <= ZERO;
                if (demo_addr[3:0] == 4'd8) begin
                    demo_state <= DEMO_RUNNING;
                end else begin
                    demo_addr <= demo_addr + 1;
                end
            end

            DEMO_RUNNING: begin
                if (s1_pulse) begin
                    // Restart demo
                    demo_addr  <= 7'd0;
                    demo_paused <= 1'b0;
                    demo_state <= DEMO_LOAD_P;
                end else if (s2_pulse) begin
                    demo_paused <= ~demo_paused;
                end else if (vsync_falling && !demo_paused && core_done) begin
                    demo_start  <= 1'b1;
                    demo_n_steps <= 16'd1;
                    demo_state  <= DEMO_STEPPING;
                end
            end

            DEMO_STEPPING: begin
                if (core_done) begin
                    demo_state <= DEMO_RUNNING;
                end
                // Also handle button during stepping
                if (s1_pulse) begin
                    demo_addr  <= 7'd0;
                    demo_paused <= 1'b0;
                    demo_state <= DEMO_LOAD_P;
                end
            end

            default: demo_state <= DEMO_IDLE;
        endcase
    end

    // =================================================================
    // Bloch coordinates from density matrix (qubit subspace)
    // =================================================================
    // x = 2 * Re(rho_01),  y = 2 * Im(rho_01),  z = Re(rho_00) - Re(rho_11)
    // All values already Q1.15. Latch once per frame for stable display.
    reg signed [15:0] bloch_x, bloch_y, bloch_z;

    // Detect vsync falling edge
    reg vsync_prev;
    wire vsync_falling = vsync_prev && !vga_vsync;
    always @(posedge pix_clk) begin
        vsync_prev <= vga_vsync;
    end

    always @(posedge pix_clk) begin
        if (vsync_falling) begin
            bloch_x <= {rho01_re[14:0], 1'b0};  // 2 * Re(rho_01)
            bloch_y <= {rho01_im[14:0], 1'b0};  // 2 * Im(rho_01)
            bloch_z <= rho00_re - rho11_re;      // Re(rho_00) - Re(rho_11)
        end
    end

    initial begin
        bloch_x    = 16'sd0;
        bloch_y    = 16'sd0;
        bloch_z    = 16'sd0;
        vsync_prev = 1'b0;
    end

    // =================================================================
    // VGA timing
    // =================================================================
    wire       vga_hsync, vga_vsync, vga_de;
    wire [9:0] pixel_x, pixel_y;

    vga_timing u_vga (
        .clk     (pix_clk),
        .rst_n   (rst_n),
        .hsync   (vga_hsync),
        .vsync   (vga_vsync),
        .de      (vga_de),
        .pixel_x (pixel_x),
        .pixel_y (pixel_y)
    );

    // =================================================================
    // Bloch sphere renderer
    // =================================================================
    wire [7:0] vid_r, vid_g, vid_b;

    bloch_renderer u_render (
        .clk     (pix_clk),
        .pixel_x (pixel_x),
        .pixel_y (pixel_y),
        .de      (vga_de),
        .bloch_x (bloch_x),
        .bloch_z (bloch_z),
        .bloch_y (bloch_y),
        .r       (vid_r),
        .g       (vid_g),
        .b       (vid_b)
    );

    // =================================================================
    // Sync/DE pipeline delay: match bloch_renderer's 1-cycle latency
    // so that hsync/vsync/de arrive at TMDS encoder aligned with RGB.
    // =================================================================
    reg hsync_d, vsync_d, de_d;
    always @(posedge pix_clk) begin
        hsync_d <= vga_hsync;
        vsync_d <= vga_vsync;
        de_d    <= vga_de;
    end

    // =================================================================
    // HDMI transmitter
    // =================================================================
    hdmi_tx u_hdmi (
        .pix_clk    (pix_clk),
        .serial_clk (serial_clk),
        .rst_n      (rst_n),
        .r          (vid_r),
        .g          (vid_g),
        .b          (vid_b),
        .hsync      (hsync_d),
        .vsync      (vsync_d),
        .de         (de_d),
        .tmds_clk_p (tmds_clk_p),
        .tmds_clk_n (tmds_clk_n),
        .tmds_d_p   (tmds_d_p),
        .tmds_d_n   (tmds_d_n)
    );

endmodule
