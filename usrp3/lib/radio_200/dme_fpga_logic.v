`timescale 1ns / 1ps

module dme_transponder #(
    parameter CLK_RATE_HZ = 100_000_000,
    parameter THRESHOLD   = 16'd800      // ! Adjust based on noise floor. This is based on lab measurements with 50 dB of gain and 
)(
    input  wire         clk,
    input  wire         rst,
    
    // RX Stream (From Radio ADC)
    input  wire signed [15:0] rx_i,
    input  wire signed [15:0] rx_q,
    input  wire         rx_strobe,
    
    // TX Stream (To Radio DAC)
    output reg signed [15:0] tx_i,
    output reg signed [15:0] tx_q,
    output reg          tx_strobe,

    // Counter for number of detected interrogations that lead to a transmission
    output reg [31:0]   tx_start_count,

    // Dynamic number of cicles for the delay
    input wire [31:0] reply_delay_cycles
);

    // =========================================================================
    // 1. TIMING PARAMETERS
    // =========================================================================
    // Mode X Interrogation (RX): 12 us spacing
    localparam RX_SPACING_CYCLES   = (12 * CLK_RATE_HZ) / 1_000_000;
    localparam RX_TOLERANCE_CYCLES = (3  * CLK_RATE_HZ) / 1_000_000; // +/- 3us window
    
    // Total Turnaround Delay: 56 us (User Spec)
    // We subtract fixed processing overhead if necessary, but using raw 56us here.
    // To simulate 10 NM range, set to x us
    // localparam REPLY_DELAY_CYCLES  = (250 * CLK_RATE_HZ) / 1_000_000; // ! Now it's dynamic
    
    // Mode X Reply (TX): 12 us spacing
    localparam TX_SPACING_CYCLES   = (12 * CLK_RATE_HZ) / 1_000_000;
    localparam ROM_DEPTH           = 512;
    localparam ROM_LAST_ADDR       = ROM_DEPTH - 1;
    // DME pulse width is 3.5us nominal; limit playback to that active part.
    localparam TX_PULSE_CYCLES     = ((35 * CLK_RATE_HZ) + 5_000_000) / 10_000_000;
    localparam TX_PULSE_SAMPLES    = (TX_PULSE_CYCLES < ROM_DEPTH) ? TX_PULSE_CYCLES : ROM_DEPTH;
    localparam TX_PULSE_LAST_ADDR  = TX_PULSE_SAMPLES - 1;
    localparam TX_GAP_CYCLES       = (TX_SPACING_CYCLES > TX_PULSE_LAST_ADDR) ? (TX_SPACING_CYCLES - TX_PULSE_LAST_ADDR) : 1;
    localparam COOLDOWN_CYCLES     = (20 * CLK_RATE_HZ) / 1_000_000;
    // =========================================================================
    // 2. GAUSSIAN PULSE ROM (The "Real Signal")
    // =========================================================================
    
    // 512 entries deep, 16 bits wide
    (* RAM_STYLE="BLOCK" *) // Force Xilinx to use BRAM, not logic slices
    reg signed [15:0] rom_memory [0:511];
    
    reg [8:0] rom_addr;
    reg signed [15:0] rom_data;

    // Load the file during Synthesis (and Simulation)
    initial begin
        $readmemh("dme_pulse_x_channel.txt", rom_memory);
    end

    // Synchronous Read (Required for BRAM inference on Spartan-6)
    always @(posedge clk) begin
        rom_data <= rom_memory[rom_addr];
    end

    // =========================================================================
    // 3. ENVELOPE DETECTOR (Alpha Max + Beta Min)
    // =========================================================================
    reg [15:0] abs_i, abs_q;
    reg [16:0] mag_estimate;
    reg pulse_detected;

    always @(posedge clk) begin
        // Absolute value
        abs_i <= (rx_i[15]) ? -rx_i : rx_i;
        abs_q <= (rx_q[15]) ? -rx_q : rx_q;
        
        // Magnitude Estimate: Max + 0.5*Min
        if (abs_i > abs_q)
            mag_estimate <= abs_i + {1'b0, abs_q[15:1]};
        else
            mag_estimate <= abs_q + {1'b0, abs_i[15:1]};
            
        pulse_detected <= (mag_estimate > THRESHOLD);
    end

    // =========================================================================
    // 4. STATE MACHINE
    // =========================================================================
    localparam S_IDLE           = 0;
    localparam S_WAIT_P2_WINDOW = 1;
    localparam S_VERIFY_P2      = 2;
    localparam S_TURNAROUND     = 3;
    localparam S_TX_PULSE_1     = 4;
    localparam S_TX_GAP         = 5;
    localparam S_TX_PULSE_2     = 6;
    localparam S_COOLDOWN       = 7;

    reg [3:0]  state;
    reg [31:0] timer; // General purpose cycle counter

    // =========================================================================
    // SQUITTER GENERATOR (16-bit LFSR)
    // =========================================================================
    localparam US_HZ = 1_000_000;

    reg [31:0] us_accum = 0;
    reg tick_1us = 0;
    
    reg [15:0] lfsr = 16'hACE1; // Seed MUST be non-zero!
    // Standard 16-bit Galois LFSR polynomial taps at 16, 14, 13, 11
    wire lfsr_feedback = lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10];

    always @(posedge clk) begin
        if (rst) begin
            us_accum <= 0;
            lfsr <= 16'hACE1;
            tick_1us <= 0;
        end else begin
            // Generate an average-accurate 1 us strobe, also for non-integer clk/us.
            if ((us_accum + US_HZ) >= CLK_RATE_HZ) begin
                us_accum <= us_accum + US_HZ - CLK_RATE_HZ;
                tick_1us <= 1'b1;
                // Shift the LFSR to get a new random number every 1us
                lfsr <= {lfsr[14:0], lfsr_feedback};
            end else begin
                us_accum <= us_accum + US_HZ;
                tick_1us <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            timer <= 0;
            tx_i <= 0;
            tx_q <= 0;
            tx_strobe <= 1;
            rom_addr <= 0;
            tx_start_count <= 0;
        end else begin
            // Default Strobe low unless transmitting
            tx_strobe <= 1;
            tx_i <= 0;
            tx_q <= 0;

            case (state)
                // --- 1. SEARCH FOR FIRST PULSE OR FIRE SQUITTER ---
                S_IDLE: begin
                    tx_i <= 0;
                    tx_q <= 0; // Assuming tx_strobe is continuously assigned 1 elsewhere
                    
                    if (rx_strobe && pulse_detected) begin
                        // Real interrogation takes priority!
                        timer <= 0;
                        state <= S_WAIT_P2_WINDOW;
                    end 
                    // Squitter logic: 2700 Hz average rate = ~0.27% chance per 1us => 177 out of 65536
                    //                  800 Hz average rate = ~0.08% chance per 1us =>  52 out of 65536
                    else if (tick_1us && (lfsr < 52)) begin
                        timer <= 0;
                        rom_addr <= 0;
                        state <= S_TX_PULSE_1; // Re-use the reply logic to send a random pair!
                    end
                end

                // --- 2. WAIT FOR SPECIFIC TIMING (12us) ---
                S_WAIT_P2_WINDOW: begin
                    // Keep RX timing in sample domain, not raw radio_clk domain.
                    if (rx_strobe) begin
                        timer <= timer + 1;
                        // Open detection window slightly before 12us
                        if (timer >= (RX_SPACING_CYCLES - RX_TOLERANCE_CYCLES)) begin
                            state <= S_VERIFY_P2;
                        end
                        // Timeout
                        else if (timer > (RX_SPACING_CYCLES + RX_TOLERANCE_CYCLES)) begin
                            state <= S_IDLE;
                        end
                    end
                end

                // --- 3. CHECK FOR SECOND PULSE ---
                S_VERIFY_P2: begin
                    if (rx_strobe) begin
                        timer <= timer + 1;
                        if (pulse_detected) begin
                            // DOUBLE PULSE CONFIRMED
                            state <= S_TURNAROUND;
                            // Increment counter each time a valid interrogation leads to a transmission
                            tx_start_count <= tx_start_count + 1;
                        end
                        // Close window if time passes 12us + tolerance
                        else if (timer > (RX_SPACING_CYCLES + RX_TOLERANCE_CYCLES)) begin
                            state <= S_IDLE;
                        end
                    end
                end

                // --- 4. 56us DELAY (TURNAROUND) ---
                S_TURNAROUND: begin
                    timer <= timer + 1;
                    if (timer >= reply_delay_cycles) begin
                        timer <= 0;
                        rom_addr <= 0; // Reset ROM pointer
                        state <= S_TX_PULSE_1;
                    end
                end

                // --- 5. TRANSMIT PULSE 1 (Gaussian) ---
                S_TX_PULSE_1: begin                    
                    // Drive I/Q with ROM data (Real signal only on I, Q=0)
                    tx_i <= rom_data;
                    tx_q <= 0;
                    
                    // Advance ROM
                    rom_addr <= rom_addr + 1;
                    
                    // If ROM finished
                    if (rom_addr >= TX_PULSE_LAST_ADDR) begin
                        timer <= 0;
                        state <= S_TX_GAP;
                    end
                end

                // --- 6. INTER-PULSE GAP (Wait 12us) ---
                S_TX_GAP: begin
                    
                    timer <= timer + 1;
                    // Note: We subtract pulse duration if timing is measured Leading-to-Leading edge
                    if (timer >= TX_GAP_CYCLES) begin
                        timer <= 0;
                        rom_addr <= 0;
                        state <= S_TX_PULSE_2;
                    end
                end

                // --- 7. TRANSMIT PULSE 2 (Gaussian) ---
                S_TX_PULSE_2: begin
                    tx_strobe <= 1;
                    tx_i <= rom_data;
                    tx_q <= 0;
                    rom_addr <= rom_addr + 1;
                    
                    if (rom_addr >= TX_PULSE_LAST_ADDR) begin
                        timer <= 0;
                        state <= S_COOLDOWN;
                    end
                end

                // --- 8. COOLDOWN / DEAD TIME ---
                S_COOLDOWN: begin
                    tx_strobe <= 1;
                    tx_i <= 0; tx_q <= 0;
                    timer <= timer + 1;
                    if (timer >= COOLDOWN_CYCLES) state <= S_IDLE;
                end
            endcase
        end
    end

endmodule