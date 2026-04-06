// PLL-clocked top-level for 9x9 complex matvec.
//
// Wraps matvec_top with a Gowin rPLL to run at 135 MHz instead of 27 MHz.
//
// PLL config: 27 MHz * 5 = 135 MHz
//   IDIV_SEL  = 0  (input divide by 1)
//   FBDIV_SEL = 4  (feedback multiply by 5)
//   ODIV_SEL  = 4  (VCO = 135 * 4 = 540 MHz, in 500-1250 range)

module matvec_pll_top (
    input  wire       clk,       // 27 MHz from crystal
    input  wire       btn_s1,
    input  wire       btn_s2,
    output wire [5:0] led,
    output wire       uart_tx,
    input  wire       uart_rx
);

    wire clk_fast;
    wire pll_lock;

    rPLL #(
        .FCLKIN       ("27"),
        .IDIV_SEL     (0),       // ÷1
        .FBDIV_SEL    (4),       // ×5 -> 27*5 = 135 MHz
        .ODIV_SEL     (4),       // VCO = 540 MHz
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
        .CLKOUT   (clk_fast),
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

    // LED[5] shows PLL lock status (active-low LEDs)
    // matvec_top controls LED[0:4]
    wire [5:0] inner_led;

    matvec_top #(
        .CLK_HZ(135_000_000)
    ) u_top (
        .clk     (clk_fast),
        .btn_s1  (btn_s1),
        .btn_s2  (btn_s2),
        .led     (inner_led),
        .uart_tx (uart_tx),
        .uart_rx (uart_rx),
        .bloch_rho00_re (),
        .bloch_rho01_re (),
        .bloch_rho01_im (),
        .bloch_rho11_re (),
        .demo_p_wr_en   (1'b0),
        .demo_p_wr_addr (7'd0),
        .demo_p_wr_re   (16'sd0),
        .demo_p_wr_im   (16'sd0),
        .demo_rho_wr_en  (1'b0),
        .demo_rho_wr_addr(4'd0),
        .demo_rho_wr_re  (16'sd0),
        .demo_rho_wr_im  (16'sd0),
        .demo_start      (1'b0),
        .demo_n_steps    (16'd0),
        .core_done       ()
    );

    // Show PLL lock on LED[5], pass through the rest
    assign led = {~pll_lock, inner_led[4:0]};

endmodule
