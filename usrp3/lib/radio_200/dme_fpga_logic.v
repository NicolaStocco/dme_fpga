`timescale 1ns / 1ps

module dme_transponder #(
    parameter CLK_RATE_HZ = 100_000_000,
    parameter THRESHOLD   = 16'd5000      // ! Adjust based on noise floor. This is based on lab measurements with 50 dB of gain and 
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
    output reg          tx_strobe
);

    // =========================================================================
    // 1. TIMING PARAMETERS
    // =========================================================================
    // localparam CYCLES_PER_US = CLK_RATE_HZ / 1_000_000; // ! Might lead to truncation issues if not an integer

    // Mode Y Interrogation (RX): 36 us spacing
    localparam RX_SPACING_CYCLES   = (36 * CLK_RATE_HZ) / 1_000_000;
    localparam RX_TOLERANCE_CYCLES = (1 * CLK_RATE_HZ) / 1_000_000; // +/- 1us window
    
    // Total Turnaround Delay: 56 us (User Spec)
    // We subtract fixed processing overhead if necessary, but using raw 56us here.
    localparam REPLY_DELAY_CYCLES  = (56 * CLK_RATE_HZ) / 1_000_000;
    
    // Mode Y Reply (TX): 30 us spacing
    localparam TX_SPACING_CYCLES   = (30 * CLK_RATE_HZ) / 1_000_000;

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
        $readmemh("dme_pulse.mem", rom_memory);
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

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            timer <= 0;
            tx_i <= 0;
            tx_q <= 0;
            tx_strobe <= 0;
            rom_addr <= 0;
        end else begin
            // Default Strobe low unless transmitting
            tx_strobe <= 0;

            case (state)
                // --- 1. SEARCH FOR FIRST PULSE ---
                S_IDLE: begin
                    tx_i <= 0; tx_q <= 0;
                    if (rx_strobe && pulse_detected) begin
                        timer <= 0;
                        state <= S_WAIT_P2_WINDOW;
                    end
                end

                // --- 2. WAIT FOR SPECIFIC TIMING (36us) ---
                S_WAIT_P2_WINDOW: begin
                    timer <= timer + 1;
                    // Open detection window slightly before 36us
                    if (timer >= (RX_SPACING_CYCLES - RX_TOLERANCE_CYCLES)) begin
                        state <= S_VERIFY_P2;
                    end
                    // Timeout
                    else if (timer > (RX_SPACING_CYCLES + RX_TOLERANCE_CYCLES)) begin
                        state <= S_IDLE;
                    end
                end

                // --- 3. CHECK FOR SECOND PULSE ---
                S_VERIFY_P2: begin
                    timer <= timer + 1;
                    if (rx_strobe && pulse_detected) begin
                        // DOUBLE PULSE CONFIRMED
                        timer <= 0;
                        state <= S_TURNAROUND;
                    end
                    // Close window if time passes 36us + tolerance
                    else if (timer > (RX_SPACING_CYCLES + RX_TOLERANCE_CYCLES)) begin
                        state <= S_IDLE;
                    end
                end

                // --- 4. 56us DELAY (TURNAROUND) ---
                S_TURNAROUND: begin
                    timer <= timer + 1;
                    if (timer >= REPLY_DELAY_CYCLES) begin
                        timer <= 0;
                        rom_addr <= 0; // Reset ROM pointer
                        state <= S_TX_PULSE_1;
                    end
                end

                // --- 5. TRANSMIT PULSE 1 (Gaussian) ---
                S_TX_PULSE_1: begin
                    tx_strobe <= 1;
                    
                    // Drive I/Q with ROM data (Real signal only on I, Q=0)
                    tx_i <= rom_data;
                    tx_q <= 0;
                    
                    // Advance ROM
                    rom_addr <= rom_addr + 1;
                    
                    // If ROM finished (using 350 as end of pulse width)
                    if (rom_addr >= 215) begin // ! Hardcoded pulse width in samples (3.5us at 30.72 MHz)
                        timer <= 0;
                        state <= S_TX_GAP;
                    end
                end

                // --- 6. INTER-PULSE GAP (Wait 30us) ---
                S_TX_GAP: begin
                    tx_i <= 0; tx_q <= 0; tx_strobe <= 1; // Send zeros to keep DAC active
                    
                    timer <= timer + 1;
                    // Note: We subtract pulse duration if timing is measured Leading-to-Leading edge
                    if (timer >= (TX_SPACING_CYCLES - 215)) begin // ! Hardcoded as well
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
                    
                    if (rom_addr >= 215) begin // ! Hardcoded pulse width in samples (3.5us at 30.72 MHz)
                        timer <= 0;
                        state <= S_COOLDOWN;
                    end
                end

                // --- 8. COOLDOWN / DEAD TIME ---
                S_COOLDOWN: begin
                    tx_strobe <= 0;
                    tx_i <= 0; tx_q <= 0;
                    timer <= timer + 1;
                    if (timer > 2000) state <= S_IDLE; // Wait 20us before listening again
                end
            endcase
        end
    end

endmodule