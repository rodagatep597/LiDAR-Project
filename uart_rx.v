`timescale 1ns / 1ps

module uart_rx (
    input clk,                // 100 MHz system clock
    input reset,              // Active-high reset
    input rx_serial,          // UART RX line
    output reg [7:0] rx_data, // Received byte
    output reg rx_valid,      // High for one clk when data is valid
    output reg rx_error       // Framing error signal
);

parameter CLK_FREQ = 100_000_000;
parameter BAUD_RATE = 115200;

localparam BAUD_TICKS = CLK_FREQ / BAUD_RATE;    // ~868
localparam SAMPLE_POINT = BAUD_TICKS / 2;        // ~434

// Synchronize input to avoid metastability
reg [2:0] rx_sync;
always @(posedge clk or posedge reset) begin
    if (reset)
        rx_sync <= 3'b111;
    else
        rx_sync <= {rx_sync[1:0], rx_serial};
end

wire rx = rx_sync[2];

// State machine states
localparam [2:0]
    IDLE  = 3'd0,
    START = 3'd1,
    DATA  = 3'd2,
    STOP  = 3'd3,
    ERROR = 3'd4;

reg [2:0] state = IDLE;
reg [15:0] bit_timer = 0;
reg [3:0] bit_index = 0;  // 4 bits to count up to 8

reg [7:0] data_buffer = 0;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= IDLE;
        bit_timer <= 0;
        bit_index <= 0;
        data_buffer <= 0;
        rx_data <= 0;
        rx_valid <= 0;
        rx_error <= 0;
    end else begin
        rx_valid <= 0;
        rx_error <= 0;

        case (state)
            IDLE: begin
                bit_timer <= 0;
                bit_index <= 0;
                data_buffer <= 0;
                if (rx == 0)  // Start bit detected
                    state <= START;
            end

            START: begin
                if (bit_timer == SAMPLE_POINT) begin
                    if (rx == 0) begin
                        bit_timer <= 0;
                        bit_index <= 0;
                        state <= DATA;
                    end else begin
                        state <= ERROR;  // Not a valid start bit
                    end
                end else begin
                    bit_timer <= bit_timer + 1;
                end
            end

            DATA: begin
                if (bit_timer == BAUD_TICKS - 1) begin
                    bit_timer <= 0;
                    data_buffer[bit_index] <= rx;
                    if (bit_index == 7) begin
                        state <= STOP;
                    end else begin
                        bit_index <= bit_index + 1;
                    end
                end else begin
                    bit_timer <= bit_timer + 1;
                end
            end

            STOP: begin
                if (bit_timer == SAMPLE_POINT) begin
                    if (rx == 1) begin
                        rx_data <= data_buffer;
                        rx_valid <= 1'b1;
                        state <= IDLE;
                    end else begin
                        rx_error <= 1'b1;  // Framing error
                        state <= ERROR;
                    end
                    bit_timer <= 0;
                end else begin
                    bit_timer <= bit_timer + 1;
                end
            end

            ERROR: begin
                // On error, reset everything and go back to IDLE
                state <= IDLE;
                bit_timer <= 0;
                bit_index <= 0;
                data_buffer <= 0;
            end

            default: state <= IDLE;
        endcase
    end
end

always @(posedge clk) begin
    if (rx_valid)
        $display("UART_RX: Data 0x%02h received at time %0t", rx_data, $time);
    if (rx_error)
        $display("UART_RX: Framing error at time %0t", $time);
end

endmodule
