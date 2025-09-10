`timescale 1ns / 1ps

module rplidar_simple (
    input wire clk_100mhz,
    input wire reset,
    input wire rplidar_rx,
    output wire rplidar_tx,
    output wire pwm_motor,
    output reg [7:0] led_out
);

    // UART signals
    wire [7:0] rx_data;
    wire rx_valid;
    wire tx_busy;
    reg [7:0] tx_data = 0;
    reg tx_start = 0;

    // Motor control
    reg motor_enable = 0;
    reg [23:0] init_counter = 0;

    // Command sequence state machine
    localparam [2:0]
        IDLE     = 0,
        SEND_A5  = 1,
        WAIT_TX  = 2,
        SEND_20  = 3,
        RUNNING  = 4;

    reg [2:0] state = IDLE;

    always @(posedge clk_100mhz or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            init_counter <= 0;
            tx_start <= 0;
            motor_enable <= 0;
        end else begin
            tx_start <= 0; // Default no start

            case (state)
                IDLE: begin
                    if (init_counter < 24'd10_000_000)  // ~0.1s delay at 100MHz
                        init_counter <= init_counter + 1;
                    else begin
                        motor_enable <= 1;
                        state <= SEND_A5;
                    end
                end
                SEND_A5: begin
                    if (!tx_busy) begin
                        tx_data <= 8'hA5;    // Command byte 1
                        tx_start <= 1;
                        state <= WAIT_TX;
                    end
                end
                WAIT_TX: begin
                    if (!tx_busy)
                        state <= SEND_20;
                end
                SEND_20: begin
                    if (!tx_busy) begin
                        tx_data <= 8'h20;    // Command byte 2 - Normal Scan
                        tx_start <= 1;
                        state <= RUNNING;
                    end
                end
                RUNNING: begin
                    // Stay here, receiving data
                end
            endcase
        end
    end

    // Distance packet parser (expect 5 bytes per data point)
    reg [2:0] byte_count = 0;
    reg [7:0] distance_lsb = 0;
    reg [15:0] distance_mm = 0;
    reg [15:0] display_distance = 0;
    reg distance_valid = 0;

    always @(posedge clk_100mhz or posedge reset) begin
        if (reset) begin
            byte_count <= 0;
            distance_mm <= 0;
            display_distance <= 0;
            distance_valid <= 0;
            distance_lsb <= 0;
        end else if (rx_valid) begin
            case (byte_count)
                0: begin
                    // Sync byte check: bit0=1 and bit1=0 expected for valid frame start
                    if (rx_data[0] == 1'b1 && rx_data[1] == 1'b0)
                        byte_count <= 1;
                    else
                        byte_count <= 0;
                end
                1: byte_count <= 2; // Angle LSB - skip for now
                2: byte_count <= 3; // Angle MSB - skip for now
                3: begin
                    distance_lsb <= rx_data;
                    byte_count <= 4;
                end
                4: begin
                    distance_mm <= {rx_data, distance_lsb};
                    display_distance <= {rx_data, distance_lsb};
                    distance_valid <= (rx_data != 8'h00 || distance_lsb != 8'h00);
                    byte_count <= 0;
                end
            endcase
        end else begin
            distance_valid <= 0; // clear flag if no data this cycle
        end
    end

    // Debug print for distance and threshold
    always @(posedge clk_100mhz) begin
        if (!reset && distance_valid) begin
            $display("[%0t] Distance = %0d mm, Threshold(6000) = %b, LED3 = %b", 
                     $time, display_distance, (display_distance > 6000), (display_distance > 6000));
        end
    end

    // Blink heartbeat on LED[0]
    reg [23:0] blink_counter = 0;
    reg blink = 0;

    always @(posedge clk_100mhz or posedge reset) begin
        if (reset) begin
            blink_counter <= 0;
            blink <= 0;
        end else begin
            if (blink_counter >= 24'd12_500_000) begin // ~0.125s toggle
                blink <= ~blink;
                blink_counter <= 0;
            end else
                blink_counter <= blink_counter + 1;
        end
    end

    // LED Output logic
    always @(posedge clk_100mhz or posedge reset) begin
        if (reset) begin
            led_out <= 8'b00000000;
        end else begin
            led_out[0] <= blink;                     // Heartbeat
            led_out[1] <= rx_valid;                  // UART data received (activity)
            led_out[2] <= distance_valid;            // Valid distance frame received
            led_out[3] <= (display_distance > 6000); // Distance threshold exceeded
            led_out[7:4] <= 4'b0000;                 // Reserved / unused
        end
    end

    // UART RX instance (115200 baud @ 100 MHz)
    uart_rx #(
        .CLK_FREQ(100_000_000),
        .BAUD_RATE(115200)
    ) uart_rx_inst (
        .clk(clk_100mhz),
        .reset(reset),
        .rx_serial(rplidar_rx),
        .rx_data(rx_data),
        .rx_valid(rx_valid),
        .rx_error()
    );

    // UART TX instance (115200 baud @ 100 MHz)
    uart_tx #(
        .CLK_FREQ(100_000_000),
        .BAUD_RATE(115200)
    ) uart_tx_inst (
        .clk(clk_100mhz),
        .reset(reset),
        .tx_data(tx_data),
        .tx_start(tx_start),
        .tx_serial(rplidar_tx),
        .tx_busy(tx_busy)
    );

    // PWM Motor control (20 kHz PWM)
    pwm_generator #(
        .CLK_FREQ(100_000_000),
        .PWM_FREQ(20000)
    ) pwm_inst (
        .clk(clk_100mhz),
        .reset(reset || !motor_enable),
        .duty_cycle(8'd230), // ~90% duty cycle motor speed
        .pwm_out(pwm_motor)
    );

endmodule
