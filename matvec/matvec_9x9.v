// 9x9 complex matrix-vector multiply in Q1.15 fixed-point.
//
// Computes rho_out = P * rho_in, where P is 9x9 complex and rho is
// a 9-element complex vector (vectorized density matrix).
//
// Architecture:
//   - P stored in inferred BSRAM (81 complex entries, SoA layout)
//   - rho in registers (9 complex entries)
//   - 4-stage pipeline: addr -> rho_delay+BSRAM -> multiply(4 DSP18) -> accumulate+store
//   - ~94 cycles per step (81 elements + 3 drain + 9 copy-back + 1 overhead)
//   - Supports multi-step: runs N steps, copying output back to input between steps
//
// Pipeline timing (element E at address A, column C, row R):
//   Cycle T+0 (S0): present address A to BSRAM, latch rho[C]
//   Cycle T+1 (S1): BSRAM data scheduled, delay rho one cycle to match
//   Cycle T+2 (S2): BSRAM data + matched rho available, 4 DSP multiplies
//   Cycle T+3 (S3): DSP results ready, accumulate; store out[R] when C==8

module matvec_9x9 (
    input  wire        clk,

    // P matrix write port (for host loading)
    input  wire        p_wr_en,
    input  wire [6:0]  p_wr_addr,     // 0..80
    input  wire signed [15:0] p_wr_re,
    input  wire signed [15:0] p_wr_im,

    // rho write port (for host loading)
    input  wire        rho_wr_en,
    input  wire [3:0]  rho_wr_addr,   // 0..8
    input  wire signed [15:0] rho_wr_re,
    input  wire signed [15:0] rho_wr_im,

    // Control
    input  wire        start,         // single-cycle pulse
    input  wire [15:0] n_steps,
    output reg         done,

    // rho read port (for host readback)
    input  wire [3:0]  rho_rd_addr,   // 0..8
    output wire signed [15:0] rho_rd_re,
    output wire signed [15:0] rho_rd_im,

    // Cycle counter (for benchmarking)
    output reg  [31:0] cycle_count,

    // Direct rho outputs for Bloch sphere (qubit subspace)
    output wire signed [15:0] bloch_rho00_re,  // rho_re[0]
    output wire signed [15:0] bloch_rho01_re,  // rho_re[1]
    output wire signed [15:0] bloch_rho01_im,  // rho_im[1]
    output wire signed [15:0] bloch_rho11_re   // rho_re[4]
);

    // =================================================================
    // P matrix storage (inferred BSRAM, synchronous read)
    // =================================================================
    reg signed [15:0] p_re [0:80];
    reg signed [15:0] p_im [0:80];
    reg signed [15:0] p_rd_re, p_rd_im;
    reg [6:0] p_rd_addr;

    always @(posedge clk) begin
        if (p_wr_en) begin
            p_re[p_wr_addr] <= p_wr_re;
            p_im[p_wr_addr] <= p_wr_im;
        end
        p_rd_re <= p_re[p_rd_addr];
        p_rd_im <= p_im[p_rd_addr];
    end

    // =================================================================
    // rho registers (current state)
    // =================================================================
    reg signed [15:0] rho_re [0:8];
    reg signed [15:0] rho_im [0:8];

    assign rho_rd_re = rho_re[rho_rd_addr];
    assign rho_rd_im = rho_im[rho_rd_addr];

    assign bloch_rho00_re = rho_re[0];
    assign bloch_rho01_re = rho_re[1];
    assign bloch_rho01_im = rho_im[1];
    assign bloch_rho11_re = rho_re[4];

    // =================================================================
    // Output buffer (written by pipeline)
    // =================================================================
    reg signed [15:0] out_re [0:8];
    reg signed [15:0] out_im [0:8];

    // =================================================================
    // Pipeline: rho select delay (matches BSRAM 1-cycle read latency)
    // =================================================================
    reg signed [15:0] rho_s0_re, rho_s0_im;  // latched in S0
    reg signed [15:0] rho_s1_re, rho_s1_im;  // delayed to match BSRAM in S1

    // =================================================================
    // Pipeline: 4 DSP18 multipliers (registered)
    // =================================================================
    reg signed [31:0] mul_ac, mul_bd, mul_ad, mul_bc;

    // Complex products (combinational from DSP outputs)
    wire signed [32:0] prod_re = {mul_ac[31], mul_ac} - {mul_bd[31], mul_bd};
    wire signed [32:0] prod_im = {mul_ad[31], mul_ad} + {mul_bc[31], mul_bc};

    // Sign-extended to accumulator width
    wire signed [41:0] prod_re_ext = {{9{prod_re[32]}}, prod_re};
    wire signed [41:0] prod_im_ext = {{9{prod_im[32]}}, prod_im};

    // =================================================================
    // Accumulator (42 bits: 33-bit products * 9 terms = 37 bits needed)
    // =================================================================
    reg signed [41:0] acc_re, acc_im;

    // Next accumulator value (combinational): used for both acc update and store
    wire signed [41:0] next_acc_re = (s2_col == 4'd0) ? prod_re_ext : (acc_re + prod_re_ext);
    wire signed [41:0] next_acc_im = (s2_col == 4'd0) ? prod_im_ext : (acc_im + prod_im_ext);

    // =================================================================
    // Pipeline valid/tracking signals
    // =================================================================
    // S0 -> S1
    reg       s0_valid;
    reg [3:0] s0_col, s0_row;

    // S1 -> S2
    reg       s1_valid;
    reg [3:0] s1_col, s1_row;

    // S2 -> S3 (multiply stage -> accumulate+store)
    reg       s2_valid;
    reg [3:0] s2_col, s2_row;

    // =================================================================
    // State machine
    // =================================================================
    localparam ST_IDLE  = 3'd0;
    localparam ST_RUN   = 3'd1;
    localparam ST_DRAIN = 3'd2;
    localparam ST_COPY  = 3'd3;

    reg [2:0]  state;
    reg [6:0]  addr_ctr;
    reg [3:0]  col_ctr, row_ctr;
    reg [15:0] steps_left;
    reg [3:0]  copy_idx;
    reg [2:0]  drain_ctr;

    // =================================================================
    // Cycle counter: counts clock edges from start to done
    // =================================================================
    always @(posedge clk) begin
        if (start && state == ST_IDLE)
            cycle_count <= 32'd1;
        else if (state != ST_IDLE)
            cycle_count <= cycle_count + 1;
    end

    integer k;
    initial begin
        state     = ST_IDLE;
        done      = 0;
        cycle_count = 0;
        s0_valid  = 0;
        s1_valid  = 0;
        s2_valid  = 0;
        addr_ctr  = 0;
        col_ctr   = 0;
        row_ctr   = 0;
        drain_ctr = 0;
        copy_idx  = 0;
        steps_left = 0;
        for (k = 0; k < 9; k = k + 1) begin
            rho_re[k] = 0; rho_im[k] = 0;
            out_re[k] = 0; out_im[k] = 0;
        end
    end

    // =================================================================
    // S0: Address generation + rho latch (in state machine)
    // =================================================================
    always @(posedge clk) begin
        s0_valid <= 1'b0;  // default

        case (state)
            ST_IDLE: begin
                done <= 1'b0;
                if (start) begin
                    addr_ctr   <= 7'd0;
                    col_ctr    <= 4'd0;
                    row_ctr    <= 4'd0;
                    steps_left <= n_steps;
                    state      <= ST_RUN;
                end
            end

            ST_RUN: begin
                // Present address to BSRAM
                p_rd_addr <= addr_ctr;

                // Latch rho for this column (will be delayed in S1 to match BSRAM)
                rho_s0_re <= rho_re[col_ctr];
                rho_s0_im <= rho_im[col_ctr];

                // Pipeline tracking
                s0_valid <= 1'b1;
                s0_col   <= col_ctr;
                s0_row   <= row_ctr;

                // Advance counters
                if (col_ctr == 4'd8) begin
                    col_ctr <= 4'd0;
                    row_ctr <= row_ctr + 1;
                end else begin
                    col_ctr <= col_ctr + 1;
                end

                if (addr_ctr == 7'd80) begin
                    drain_ctr <= 3'd3;
                    state     <= ST_DRAIN;
                end
                addr_ctr <= addr_ctr + 1;
            end

            ST_DRAIN: begin
                if (drain_ctr > 0)
                    drain_ctr <= drain_ctr - 1;
                else begin
                    steps_left <= steps_left - 1;
                    copy_idx   <= 4'd0;
                    state      <= ST_COPY;
                end
            end

            ST_COPY: begin
                rho_re[copy_idx] <= out_re[copy_idx];
                rho_im[copy_idx] <= out_im[copy_idx];
                if (copy_idx == 4'd8) begin
                    if (steps_left == 16'd0) begin
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        addr_ctr <= 7'd0;
                        col_ctr  <= 4'd0;
                        row_ctr  <= 4'd0;
                        state    <= ST_RUN;
                    end
                end else begin
                    copy_idx <= copy_idx + 1;
                end
            end

            default: state <= ST_IDLE;
        endcase

        // External rho write (only when idle)
        if (rho_wr_en && state == ST_IDLE) begin
            rho_re[rho_wr_addr] <= rho_wr_re;
            rho_im[rho_wr_addr] <= rho_wr_im;
        end
    end

    // =================================================================
    // S1: Delay rho by 1 cycle to match BSRAM read latency
    // =================================================================
    always @(posedge clk) begin
        rho_s1_re <= rho_s0_re;
        rho_s1_im <= rho_s0_im;
        s1_valid  <= s0_valid;
        s1_col    <= s0_col;
        s1_row    <= s0_row;
    end

    // =================================================================
    // S2: Multiply (4 DSP18 blocks, registered)
    //     p_rd_re/im now holds data from 2 cycles ago (matched with rho_s1)
    // =================================================================
    always @(posedge clk) begin
        mul_ac   <= p_rd_re * rho_s1_re;
        mul_bd   <= p_rd_im * rho_s1_im;
        mul_ad   <= p_rd_re * rho_s1_im;
        mul_bc   <= p_rd_im * rho_s1_re;
        s2_valid <= s1_valid;
        s2_col   <= s1_col;
        s2_row   <= s1_row;
    end

    // =================================================================
    // S3: Accumulate + Store
    //     Uses combinational next_acc to both update acc and store result
    //     on the same cycle the last column (col==8) arrives.
    // =================================================================
    always @(posedge clk) begin
        if (s2_valid) begin
            acc_re <= next_acc_re;
            acc_im <= next_acc_im;

            // Store result when row is complete
            if (s2_col == 4'd8) begin
                out_re[s2_row] <= next_acc_re[30:15];
                out_im[s2_row] <= next_acc_im[30:15];
            end
        end
    end

endmodule
