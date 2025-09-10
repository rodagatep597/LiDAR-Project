`timescale 1ns / 1ps

module uart_tx_tb();

// Parameters
parameter CLK_PERIOD = 10;       // 100 MHz
parameter BAUD_PERIOD = 8680;    // 115200 baud

// Signals
reg clk;
reg reset;
reg [7:0] tx_data;
reg tx_start;
wire tx_serial;
wire tx_busy;

// Instantiate DUT
uart_tx uut (
    .clk(clk),
    .reset(reset),
    .tx_data(tx_data),
    .tx_start(tx_start),
    .tx_serial(tx_serial),
    .tx_busy(tx_busy)
);

// Clock generation
initial begin
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// Task to verify UART byte on tx_serial
task check_uart_byte;
    input [7:0] expected;
    integer i;
    reg [7:0] received;
    begin
        received = 8'h00;

        // Wait for start bit
        wait (tx_serial == 0);
        #(BAUD_PERIOD / 2);
        if (tx_serial !== 0) begin
            $display("[%0t] ERROR: Start bit not low", $time);
            $finish;
        end

        #(BAUD_PERIOD);
        for (i = 0; i < 8; i = i + 1) begin
            received[i] = tx_serial;
            #(BAUD_PERIOD);
        end

        // Stop bit
        if (tx_serial !== 1) begin
            $display("[%0t] ERROR: Stop bit not high", $time);
            $finish;
        end

        if (received !== expected) begin
            $display("[%0t] ERROR: Expected 0x%02h, got 0x%02h", $time, expected, received);
            $finish;
        end else begin
            $display("[%0t] PASS: Transmitted 0x%02h correctly", $time, expected);
        end
    end
endtask

// Test sequence
initial begin
    // Reset
    reset = 1;
    tx_data = 0;
    tx_start = 0;
    #100;
    reset = 0;
    #100;

    // Test 1: Transmit 0x55
    $display("\n[%0t] Test 1: Send 0x55", $time);
    tx_data = 8'h55;
    tx_start = 1;
    @(posedge clk);
    tx_start = 0;
    wait (tx_busy == 1);
    wait (tx_busy == 0);
    check_uart_byte(8'h55);

    // Test 2: Transmit 3 random bytes
    $display("\n[%0t] Test 2: Random bytes", $time);
    repeat (3) begin
        tx_data = $random;
        tx_start = 1;
        @(posedge clk);
        tx_start = 0;
        wait (tx_busy == 1);
        wait (tx_busy == 0);
        check_uart_byte(tx_data);
    end

    // Test 3: Back-to-back manual control
    $display("\n[%0t] Test 3: Back-to-back transmission", $time);
    
    tx_data = 8'hAA;
    tx_start = 1;
    @(posedge clk);
    tx_start = 0;
    wait (tx_busy == 1);
    wait (tx_busy == 0);
    check_uart_byte(8'hAA);

    tx_data = 8'hBB;
    tx_start = 1;
    @(posedge clk);
    tx_start = 0;
    wait (tx_busy == 1);
    wait (tx_busy == 0);
    check_uart_byte(8'hBB);

    // Test 4: Line idle
    $display("\n[%0t] Test 4: Check idle", $time);
    #(BAUD_PERIOD * 3);
    if (tx_serial !== 1) begin
        $display("[%0t] ERROR: TX line not idle (should be HIGH)", $time);
        $finish;
    end else begin
        $display("[%0t] PASS: Line is idle (HIGH)", $time);
    end

    $display("\n[%0t] All tests passed", $time);
    $finish;
end

// FSM state monitor
always @(uut.state) begin
    case (uut.state)
        3'b000: $display("[%0t] FSM: IDLE", $time);
        3'b001: $display("[%0t] FSM: START", $time);
        3'b010: $display("[%0t] FSM: DATA", $time);
        3'b011: $display("[%0t] FSM: STOP", $time);
    endcase
end

endmodule
