`timescale 1ns / 1ps

module rplidar_simple_tb;

    reg clk = 0;
    reg reset = 1;
    wire pwm_motor;
    wire [7:0] led_out;

    wire rplidar_tx;  // UART TX from DUT
    reg rplidar_rx_reg = 1;  // UART RX input to DUT
    wire rplidar_rx = rplidar_rx_reg;

    // Clock generation: 100 MHz (10 ns period)
    always #5 clk = ~clk;

    // Instantiate DUT
    rplidar_simple dut (
        .clk_100mhz(clk),
        .reset(reset),
        .rplidar_rx(rplidar_rx),
        .rplidar_tx(rplidar_tx),
        .pwm_motor(pwm_motor),
        .led_out(led_out)
    );

    // UART configuration
    parameter BAUD_RATE = 115200;
    parameter CLK_FREQ = 100_000_000;
    localparam integer BIT_PERIOD = 1_000_000_000 / BAUD_RATE;  // in ns

    // Valid RPLIDAR scan data packet (5 bytes)
    reg [7:0] test_data [0:4];
    initial begin
        // Format: [sync][distance_L][distance_H][angle_L][angle_H]
        // Distance = 4000 (0x0FA0), Angle = 90.00° = 9000 = 0x2328
        test_data[0] = 8'hC1; // Valid sync byte: MSB = 1, bit 0 != bit 1
        test_data[1] = 8'hA0; // Distance LSB
        test_data[2] = 8'h0F; // Distance MSB
        test_data[3] = 8'h28; // Angle LSB
        test_data[4] = 8'h23; // Angle MSB
    end

    integer byte_idx;

    initial begin
        $display("[%0t] Testbench start", $time);

        // Initial reset pulse
        reset = 1;
        rplidar_rx_reg = 1;
        #100;
        reset = 0;
        $display("[%0t] Released reset", $time);

        // Wait for DUT to initialize and send commands
        #(BIT_PERIOD * 200);

        // Send 5-byte RPLIDAR scan frame to DUT
        for (byte_idx = 0; byte_idx < 5; byte_idx = byte_idx + 1) begin
            $display("[%0t] Sending UART byte %0d = 0x%02X", $time, byte_idx, test_data[byte_idx]);
            send_uart_byte(test_data[byte_idx]);
            #10000; // Short delay between bytes
        end

        // Wait for DUT to process and update LEDs
        #10_000_000;

        // Display final LED state
        $display("[%0t] Final LED states: %b", $time, led_out);

        $display("[%0t] Testbench finished", $time);
        $finish;
    end

    // UART byte sender
    task send_uart_byte(input [7:0] data);
        integer i;
        begin
            // Start bit
            rplidar_rx_reg = 0;
            #(BIT_PERIOD);

            // Data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                rplidar_rx_reg = data[i];
                #(BIT_PERIOD);
            end

            // Stop bit
            rplidar_rx_reg = 1;
            #(BIT_PERIOD);
        end
    endtask

    // Monitor UART TX
    initial begin
        forever @(posedge rplidar_tx or negedge rplidar_tx) begin
            $display("[%0t] DUT UART TX changed to %b", $time, rplidar_tx);
        end
    end

    // Monitor LED changes
    reg [7:0] led_out_prev = 0;
    always @(posedge clk) begin
        if (led_out !== led_out_prev) begin
            $display("[%0t] LED changed to %b", $time, led_out);
            led_out_prev <= led_out;
        end
    end

endmodule
