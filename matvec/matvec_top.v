// Top-level: UART protocol handler for 9x9 complex matvec.
//
// Protocol (all multi-byte values are little-endian):
//   'P' + 324 bytes  -> Load propagator (81 entries * 4 bytes). Ack: 'K'.
//   'V' + 36 bytes   -> Load rho vector (9 entries * 4 bytes). Ack: 'K'.
//   'S'              -> Single step. Returns 36 bytes (result rho).
//   'N' + 2 bytes    -> N steps (uint16 LE). Returns 36 bytes (result rho).
//
// Each complex Q1.15 entry is 4 bytes: re_lo, re_hi, im_lo, im_hi.

module matvec_top #(
    parameter CLK_HZ = 27_000_000
) (
    input  wire       clk,
    input  wire       btn_s1,
    input  wire       btn_s2,
    output reg  [5:0] led,
    output wire       uart_tx,
    input  wire       uart_rx,

    // Bloch sphere outputs (directly from rho registers)
    output wire signed [15:0] bloch_rho00_re,
    output wire signed [15:0] bloch_rho01_re,
    output wire signed [15:0] bloch_rho01_im,
    output wire signed [15:0] bloch_rho11_re,

    // Demo sideband: direct core access (active when demo_*_en is high)
    input  wire        demo_p_wr_en,
    input  wire [6:0]  demo_p_wr_addr,
    input  wire signed [15:0] demo_p_wr_re,
    input  wire signed [15:0] demo_p_wr_im,
    input  wire        demo_rho_wr_en,
    input  wire [3:0]  demo_rho_wr_addr,
    input  wire signed [15:0] demo_rho_wr_re,
    input  wire signed [15:0] demo_rho_wr_im,
    input  wire        demo_start,
    input  wire [15:0] demo_n_steps,
    output wire        core_done
);

    // =================================================================
    // UART TX
    // =================================================================
    reg  [7:0] tx_data;
    reg        tx_start;
    wire       tx_busy;

    uart_tx #(.CLK_HZ(CLK_HZ)) u_tx (
        .clk     (clk),
        .data_in (tx_data),
        .start   (tx_start),
        .busy    (tx_busy),
        .tx      (uart_tx)
    );

    // =================================================================
    // UART RX
    // =================================================================
    wire [7:0] rx_data;
    wire       rx_valid;

    uart_rx #(.CLK_HZ(CLK_HZ)) u_rx (
        .clk      (clk),
        .rx       (uart_rx),
        .data_out (rx_data),
        .valid    (rx_valid)
    );

    // =================================================================
    // Matvec core
    // =================================================================
    reg        mv_p_wr_en;
    reg  [6:0] mv_p_wr_addr;
    reg signed [15:0] mv_p_wr_re, mv_p_wr_im;

    reg        mv_rho_wr_en;
    reg  [3:0] mv_rho_wr_addr;
    reg signed [15:0] mv_rho_wr_re, mv_rho_wr_im;

    reg        mv_start;
    reg [15:0] mv_n_steps;
    wire       mv_done;

    reg  [3:0] mv_rho_rd_addr;
    wire signed [15:0] mv_rho_rd_re, mv_rho_rd_im;
    wire [31:0] mv_cycle_count;

    assign core_done = mv_done;

    // Mux: UART vs demo sideband (demo takes priority when its enables are active)
    wire        mux_p_wr_en     = demo_p_wr_en   | mv_p_wr_en;
    wire [6:0]  mux_p_wr_addr   = demo_p_wr_en   ? demo_p_wr_addr   : mv_p_wr_addr;
    wire signed [15:0] mux_p_wr_re = demo_p_wr_en ? demo_p_wr_re : mv_p_wr_re;
    wire signed [15:0] mux_p_wr_im = demo_p_wr_en ? demo_p_wr_im : mv_p_wr_im;

    wire        mux_rho_wr_en     = demo_rho_wr_en   | mv_rho_wr_en;
    wire [3:0]  mux_rho_wr_addr   = demo_rho_wr_en   ? demo_rho_wr_addr   : mv_rho_wr_addr;
    wire signed [15:0] mux_rho_wr_re = demo_rho_wr_en ? demo_rho_wr_re : mv_rho_wr_re;
    wire signed [15:0] mux_rho_wr_im = demo_rho_wr_en ? demo_rho_wr_im : mv_rho_wr_im;

    wire        mux_start   = demo_start | mv_start;
    wire [15:0] mux_n_steps = demo_start ? demo_n_steps : mv_n_steps;

    matvec_9x9 core (
        .clk         (clk),
        .p_wr_en     (mux_p_wr_en),
        .p_wr_addr   (mux_p_wr_addr),
        .p_wr_re     (mux_p_wr_re),
        .p_wr_im     (mux_p_wr_im),
        .rho_wr_en   (mux_rho_wr_en),
        .rho_wr_addr (mux_rho_wr_addr),
        .rho_wr_re   (mux_rho_wr_re),
        .rho_wr_im   (mux_rho_wr_im),
        .start       (mux_start),
        .n_steps     (mux_n_steps),
        .done        (mv_done),
        .rho_rd_addr (mv_rho_rd_addr),
        .rho_rd_re   (mv_rho_rd_re),
        .rho_rd_im   (mv_rho_rd_im),
        .cycle_count (mv_cycle_count),
        .bloch_rho00_re (bloch_rho00_re),
        .bloch_rho01_re (bloch_rho01_re),
        .bloch_rho01_im (bloch_rho01_im),
        .bloch_rho11_re (bloch_rho11_re)
    );

    // =================================================================
    // Protocol state machine
    // =================================================================
    localparam CMD_WAIT    = 4'd0;
    localparam LOAD_P      = 4'd1;
    localparam LOAD_RHO    = 4'd2;
    localparam LOAD_N_LO   = 4'd3;
    localparam LOAD_N_HI   = 4'd4;
    localparam RUN_START   = 4'd5;
    localparam WAIT_DONE   = 4'd6;
    localparam SEND_LATCH  = 4'd7;
    localparam SEND_RHO    = 4'd8;
    localparam SEND_ACK    = 4'd9;
    localparam SEND_TIMER  = 4'd10;

    reg [3:0]  proto_state;

    // Byte assembly for 4-byte complex entries
    reg [1:0]  byte_idx;      // 0-3 within each entry
    reg [7:0]  byte_buf [0:2]; // hold first 3 bytes
    reg [6:0]  entry_idx;     // 0-80 for P, 0-8 for rho
    reg [6:0]  entry_max;     // 80 for P, 8 for rho

    // Send state
    reg [3:0]  send_entry;    // 0-8
    reg [1:0]  send_byte;     // 0-3
    reg signed [15:0] send_re, send_im;  // latched for current entry

    // N steps temp
    reg [7:0]  n_lo;

    // Timer send state
    reg [1:0]  timer_byte;
    reg [31:0] timer_latch;

    initial begin
        proto_state = CMD_WAIT;
        led         = 6'b111111;
        tx_start    = 0;
        tx_data     = 0;
        mv_p_wr_en  = 0;
        mv_rho_wr_en = 0;
        mv_start    = 0;
        byte_idx    = 0;
        entry_idx   = 0;
        send_entry  = 0;
        send_byte   = 0;
    end

    always @(posedge clk) begin
        tx_start     <= 1'b0;
        mv_p_wr_en   <= 1'b0;
        mv_rho_wr_en <= 1'b0;
        mv_start     <= 1'b0;

        case (proto_state)
            // ---------------------------------------------------------
            CMD_WAIT: begin
                if (rx_valid) begin
                    case (rx_data)
                        8'h50: begin // 'P' - load propagator
                            byte_idx    <= 0;
                            entry_idx   <= 0;
                            entry_max   <= 7'd80;
                            proto_state <= LOAD_P;
                            led[0]      <= ~led[0];
                        end
                        8'h56: begin // 'V' - load rho vector
                            byte_idx    <= 0;
                            entry_idx   <= 0;
                            entry_max   <= 7'd8;
                            proto_state <= LOAD_RHO;
                            led[1]      <= ~led[1];
                        end
                        8'h53: begin // 'S' - single step
                            mv_n_steps  <= 16'd1;
                            proto_state <= RUN_START;
                            led[2]      <= ~led[2];
                        end
                        8'h4E: begin // 'N' - multi-step
                            proto_state <= LOAD_N_LO;
                        end
                        8'h54: begin // 'T' - read cycle counter (4 bytes LE)
                            timer_latch <= mv_cycle_count;
                            timer_byte  <= 2'd0;
                            proto_state <= SEND_TIMER;
                        end
                        default: ; // ignore unknown commands
                    endcase
                end
            end

            // ---------------------------------------------------------
            // Load P matrix: 81 entries * 4 bytes = 324 bytes
            // ---------------------------------------------------------
            LOAD_P: begin
                if (rx_valid) begin
                    case (byte_idx)
                        2'd0: begin byte_buf[0] <= rx_data; byte_idx <= 2'd1; end
                        2'd1: begin byte_buf[1] <= rx_data; byte_idx <= 2'd2; end
                        2'd2: begin byte_buf[2] <= rx_data; byte_idx <= 2'd3; end
                        2'd3: begin
                            // All 4 bytes received: re_lo, re_hi, im_lo, im_hi
                            mv_p_wr_en   <= 1'b1;
                            mv_p_wr_addr <= entry_idx;
                            mv_p_wr_re   <= {byte_buf[1], byte_buf[0]};
                            mv_p_wr_im   <= {rx_data, byte_buf[2]};
                            byte_idx     <= 2'd0;

                            if (entry_idx == entry_max) begin
                                proto_state <= SEND_ACK;
                            end else begin
                                entry_idx <= entry_idx + 1;
                            end
                        end
                    endcase
                end
            end

            // ---------------------------------------------------------
            // Load rho vector: 9 entries * 4 bytes = 36 bytes
            // ---------------------------------------------------------
            LOAD_RHO: begin
                if (rx_valid) begin
                    case (byte_idx)
                        2'd0: begin byte_buf[0] <= rx_data; byte_idx <= 2'd1; end
                        2'd1: begin byte_buf[1] <= rx_data; byte_idx <= 2'd2; end
                        2'd2: begin byte_buf[2] <= rx_data; byte_idx <= 2'd3; end
                        2'd3: begin
                            mv_rho_wr_en   <= 1'b1;
                            mv_rho_wr_addr <= entry_idx[3:0];
                            mv_rho_wr_re   <= {byte_buf[1], byte_buf[0]};
                            mv_rho_wr_im   <= {rx_data, byte_buf[2]};
                            byte_idx       <= 2'd0;

                            if (entry_idx == entry_max) begin
                                proto_state <= SEND_ACK;
                            end else begin
                                entry_idx <= entry_idx + 1;
                            end
                        end
                    endcase
                end
            end

            // ---------------------------------------------------------
            // Load N (step count, little-endian uint16)
            // ---------------------------------------------------------
            LOAD_N_LO: begin
                if (rx_valid) begin
                    n_lo        <= rx_data;
                    proto_state <= LOAD_N_HI;
                end
            end

            LOAD_N_HI: begin
                if (rx_valid) begin
                    mv_n_steps  <= {rx_data, n_lo};
                    proto_state <= RUN_START;
                    led[3]      <= ~led[3];
                end
            end

            // ---------------------------------------------------------
            // Start computation
            // ---------------------------------------------------------
            RUN_START: begin
                mv_start    <= 1'b1;
                proto_state <= WAIT_DONE;
            end

            WAIT_DONE: begin
                if (mv_done) begin
                    send_entry  <= 4'd0;
                    send_byte   <= 2'd0;
                    mv_rho_rd_addr <= 4'd0;
                    proto_state <= SEND_LATCH;
                end
            end

            // ---------------------------------------------------------
            // Latch rho entry data before sending
            // ---------------------------------------------------------
            SEND_LATCH: begin
                send_re     <= mv_rho_rd_re;
                send_im     <= mv_rho_rd_im;
                send_byte   <= 2'd0;
                proto_state <= SEND_RHO;
            end

            // ---------------------------------------------------------
            // Send rho: 9 entries * 4 bytes = 36 bytes
            // ---------------------------------------------------------
            SEND_RHO: begin
                if (!tx_busy && !tx_start) begin
                    case (send_byte)
                        2'd0: begin tx_data <= send_re[7:0];  tx_start <= 1'b1; send_byte <= 2'd1; end
                        2'd1: begin tx_data <= send_re[15:8]; tx_start <= 1'b1; send_byte <= 2'd2; end
                        2'd2: begin tx_data <= send_im[7:0];  tx_start <= 1'b1; send_byte <= 2'd3; end
                        2'd3: begin
                            tx_data  <= send_im[15:8];
                            tx_start <= 1'b1;
                            if (send_entry == 4'd8) begin
                                proto_state <= CMD_WAIT;
                            end else begin
                                send_entry     <= send_entry + 1;
                                mv_rho_rd_addr <= send_entry + 1;
                                proto_state    <= SEND_LATCH;
                            end
                        end
                    endcase
                end
            end

            // ---------------------------------------------------------
            // Send acknowledgement byte 'K'
            // ---------------------------------------------------------
            SEND_ACK: begin
                if (!tx_busy && !tx_start) begin
                    tx_data     <= 8'h4B; // 'K'
                    tx_start    <= 1'b1;
                    proto_state <= CMD_WAIT;
                end
            end

            // ---------------------------------------------------------
            // Send cycle counter: 4 bytes, little-endian uint32
            // ---------------------------------------------------------
            SEND_TIMER: begin
                if (!tx_busy && !tx_start) begin
                    case (timer_byte)
                        2'd0: begin tx_data <= timer_latch[7:0];   tx_start <= 1'b1; timer_byte <= 2'd1; end
                        2'd1: begin tx_data <= timer_latch[15:8];  tx_start <= 1'b1; timer_byte <= 2'd2; end
                        2'd2: begin tx_data <= timer_latch[23:16]; tx_start <= 1'b1; timer_byte <= 2'd3; end
                        2'd3: begin tx_data <= timer_latch[31:24]; tx_start <= 1'b1; proto_state <= CMD_WAIT; end
                    endcase
                end
            end

        endcase
    end

endmodule
