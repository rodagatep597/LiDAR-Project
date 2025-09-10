`timescale 1ns / 1ps

module uart_tx (
    input wire clk,           // 100 MHz system clock
    input wire reset,         // Active-high reset
    input wire [7:0] tx_data, // Byte to transmit
    input wire tx_start,      // Start transmission
    output reg tx_serial,     // UART TX line
    output wire tx_busy       // High during transmission
);

// =============================================
// Parameters
// =============================================
parameter CLK_FREQ = 100_000_000;
parameter BAUD_RATE = 115200;
localparam BAUD_TICKS = CLK_FREQ / BAUD_RATE;  // 868 ticks at 100 MHz

// =============================================
// State Machine Definitions
// =============================================
localparam [2:0] 
    STATE_IDLE  = 3'b000,
    STATE_START = 3'b001,
    STATE_DATA  = 3'b010,
    STATE_STOP  = 3'b011;

reg [2:0] state = STATE_IDLE;
reg [15:0] bit_counter = 0;     // Counts clock cycles for baud timing
reg [2:0] bit_index = 0;        // Index of the current bit being sent
reg [7:0] tx_shift_reg = 8'b0;  // Holds the byte being transmitted

// =============================================
// UART Transmission FSM
// =============================================
always @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= STATE_IDLE;
        bit_counter <= 0;
        bit_index <= 0;
        tx_serial <= 1'b1;       // Idle line high
        tx_shift_reg <= 8'b0;
    end else begin
        case (state)
            // ------------------------
            STATE_IDLE: begin
                tx_serial <= 1'b1;
                if (tx_start) begin
                    tx_shift_reg <= tx_data;
                    bit_counter <= 0;
                    bit_index <= 0;
                    tx_serial <= 1'b0;  // Start bit
                    state <= STATE_START;
                end
            end

            // ------------------------
            STATE_START: begin
                if (bit_counter == BAUD_TICKS - 1) begin
                    bit_counter <= 0;
                    tx_serial <= tx_shift_reg[0];  // First data bit
                    bit_index <= 1;
                    state <= STATE_DATA;
                end else begin
                    bit_counter <= bit_counter + 1;
                end
            end

            // ------------------------
            STATE_DATA: begin
                if (bit_counter == BAUD_TICKS - 1) begin
                    bit_counter <= 0;
                    if (bit_index == 7) begin
                        tx_serial <= 1'b1;  // Stop bit
                        state <= STATE_STOP;
                    end else begin
                        tx_serial <= tx_shift_reg[bit_index];
                        bit_index <= bit_index + 1;
                    end
                end else begin
                    bit_counter <= bit_counter + 1;
                end
            end

            // ------------------------
            STATE_STOP: begin
                if (bit_counter == BAUD_TICKS - 1) begin
                    state <= STATE_IDLE;
                    bit_counter <= 0;
                end else begin
                    bit_counter <= bit_counter + 1;
                end
                tx_serial <= 1'b1;  // Keep line high during stop bit
            end
        endcase
    end
end

// =============================================
// Output
// =============================================
assign tx_busy = (state != STATE_IDLE);

endmodule
