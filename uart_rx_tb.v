`timescale 1ns / 1ps

module uart_rx_tb();

// Testbench Parameters
parameter CLK_PERIOD = 10;       // 100 MHz clock (10 ns period)
parameter CLK_FREQ = 100_000_000;
parameter BAUD_RATE = 115200;
parameter BAUD_TICKS = CLK_FREQ / BAUD_RATE;  // 868 clock cycles per bit
parameter SIM_DURATION = 10_000_000; // 10 ms simulation timeout

// Testbench Signals
reg clk;
reg reset;
reg rx_serial;
wire [7:0] rx_data;
wire rx_valid;
wire rx_error;

reg [7:0] captured_data;

// Instantiate UART Receiver
uart_rx uut (
    .clk(clk),
    .reset(reset),
    .rx_serial(rx_serial),
    .rx_data(rx_data),
    .rx_valid(rx_valid),
    .rx_error(rx_error)
);

// Clock Generation
initial begin
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// Task to send one UART byte
task uart_send_byte;
    input [7:0] data;
    integer i;
    begin
        // Start bit
        rx_serial = 1'b0;
        repeat(BAUD_TICKS) @(posedge clk);
        
        // Data bits (LSB first)
        for (i = 0; i < 8; i = i + 1) begin
            rx_serial = data[i];
            repeat(BAUD_TICKS) @(posedge clk);
        end
        
        // Stop bit
        rx_serial = 1'b1;
        repeat(BAUD_TICKS) @(posedge clk);
        
        // Line remains idle after stop bit
        rx_serial = 1'b1;
    end
endtask

// Wait for rx_valid with timeout
task wait_for_rx_valid;
    integer timeout_count;
    reg give_up;
    begin
        give_up = 0;
        timeout_count = 0;

        // Wait for rx_valid to go low
        while (rx_valid == 1 && give_up == 0) begin
            @(posedge clk);
            timeout_count = timeout_count + 1;
            if (timeout_count > 2 * BAUD_TICKS * 10) begin
                $display("[%0t] ERROR: Timeout waiting for rx_valid to go low", $time);
                give_up = 1;
            end
        end

        timeout_count = 0;

        // Wait for rx_valid to go high
        while (rx_valid == 0 && give_up == 0) begin
            @(posedge clk);
            timeout_count = timeout_count + 1;
            if (timeout_count > 2 * BAUD_TICKS * 10) begin
                $display("[%0t] ERROR: Timeout waiting for rx_valid", $time);
                give_up = 1;
            end
        end
    end
endtask

integer i, j;

// Main Test Sequence
initial begin
    // Initialize inputs
    reset = 1'b1;
    rx_serial = 1'b1;  // Idle state high

    // Apply reset
    #100;
    reset = 1'b0;
    #100;
    
    $display("[%0t] Starting UART RX testbench", $time);

    // Test 1: Send 0x55
    $display("[%0t] Test 1: Sending 0x55", $time);
    uart_send_byte(8'h55);
    wait_for_rx_valid();
    captured_data = rx_data;
    if (captured_data !== 8'h55)
        $display("[%0t] ERROR: Expected 0x55, received 0x%02X", $time, captured_data);
    else
        $display("[%0t] PASS: Correctly received 0x55", $time);
    #(BAUD_TICKS*5);

    // Test 2: Send 0xAA
    $display("[%0t] Test 2: Sending 0xAA", $time);
    uart_send_byte(8'hAA);
    wait_for_rx_valid();
    captured_data = rx_data;
    if (captured_data !== 8'hAA)
        $display("[%0t] ERROR: Expected 0xAA, received 0x%02X", $time, captured_data);
    else
        $display("[%0t] PASS: Correctly received 0xAA", $time);
    #(BAUD_TICKS*5);

    // Test 3: Sequential bytes 0x01 to 0x03
    $display("[%0t] Test 3: Sending sequence 0x01 to 0x03", $time);
    for (i = 1; i <= 3; i = i + 1) begin
        uart_send_byte(i[7:0]);
        wait_for_rx_valid();
        captured_data = rx_data;
        if (captured_data !== i[7:0])
            $display("[%0t] ERROR: Expected 0x%02X, received 0x%02X", $time, i, captured_data);
        else
            $display("[%0t] PASS: Correctly received 0x%02X", $time, i);
        #(BAUD_TICKS*2);
    end

    // Test 4: False start bit (line low briefly then back high)
    $display("[%0t] Test 4: False start bit", $time);
    rx_serial = 1'b0;
    #(BAUD_TICKS/2 * CLK_PERIOD); // Brief glitch
    rx_serial = 1'b1;
    #(BAUD_TICKS*5 * CLK_PERIOD);

    // Test 5: Framing error (stop bit low)
    $display("[%0t] Test 5: Framing error test (stop bit low)", $time);
    // Start bit
    rx_serial = 1'b0;
    repeat(BAUD_TICKS) @(posedge clk);
    // Data bits (0x55)
    for (j = 0; j < 8; j = j + 1) begin
        rx_serial = (8'h55 >> j) & 1'b1;
        repeat(BAUD_TICKS) @(posedge clk);
    end
    // Stop bit forced low (should cause framing error)
    rx_serial = 1'b0;
    repeat(BAUD_TICKS) @(posedge clk);
    #(BAUD_TICKS*5 * CLK_PERIOD);

    $display("[%0t] All tests completed", $time);
    $finish;
end

// Monitor rx_valid events
always @(posedge clk) begin
    if (rx_valid)
        $display("[%0t] rx_valid asserted. Received byte: 0x%02X", $time, rx_data);
    if (rx_error)
        $display("[%0t] UART_RX: Framing error at time %0t", $time, $time);
end

// Simulation timeout
initial begin
    #(SIM_DURATION);
    $display("[%0t] Simulation timeout reached", $time);
    $finish;
end

endmodule
