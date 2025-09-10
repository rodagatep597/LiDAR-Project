module pwm_generator (
    input wire clk,
    input wire reset,             // Active-high reset
    input wire [7:0] duty_cycle,  // 0-255 (255 = 100% duty)
    output reg pwm_out            // PWM output
);

parameter CLK_FREQ = 100_000_000; // 100 MHz
parameter PWM_FREQ = 20_000;      // 20 kHz (inaudible)
localparam PWM_MAX = CLK_FREQ / PWM_FREQ; // 5000 counts
localparam COUNTER_WIDTH = $clog2(PWM_MAX); // 13 bits

reg [COUNTER_WIDTH-1:0] counter = 0;
wire [COUNTER_WIDTH-1:0] threshold = (duty_cycle * PWM_MAX + 128) >> 8; // Rounded

always @(posedge clk or posedge reset) begin
    if (reset) begin
        counter <= 0;
        pwm_out <= 0;
    end else begin
        counter <= (counter >= PWM_MAX-1) ? 0 : counter + 1;
        pwm_out <= (counter < threshold) && (duty_cycle != 0); // Safe zero-duty
    end
end

endmodule