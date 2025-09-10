#include <systemc.h>
#include <queue>
#include <iostream>
using namespace sc_core;
using namespace sc_dt;

// Structure representing a simplified scan packet from RPLIDAR
struct RPLIDAR_packet {
    uint8_t quality;     // Measurement quality
    uint16_t angle;      // Angle in 0.01 degrees
    uint16_t distance;   // Distance in millimeters
    bool valid;          // Indicates if the packet is valid

    RPLIDAR_packet() : quality(0), angle(0), distance(0), valid(false) {}
};

// Module that emulates the behavior of the RPLIDAR A1M8
SC_MODULE(RPLIDAR_Emulator) {
    sc_in<bool> pwm_motor;       // Motor enable input (PWM-like)
    sc_out<uint8_t> tx_data;     // Serial TX data output (to controller RX)
    sc_in<uint8_t> rx_data;      // Serial RX data input (from controller TX)
    sc_in<bool> tx_start;        // Indicates controller is sending a byte
    sc_in<bool> clk;             // Clock input

    // Process to generate and send LIDAR scan packets over UART
    void data_generator() {
        while (true) {
            wait(10, SC_MS); // Emit a new packet every 10ms

            if (!pwm_motor.read()) {
                // Skip sending data if the motor is not running
                continue;
            }

            // Generate random valid scan packet
            RPLIDAR_packet pkt;
            pkt.quality = rand() % 64;
            pkt.angle = rand() % 36000;         // 0.00° to 359.99°
            pkt.distance = 500 + rand() % 6000; // 500mm to 6500mm
            pkt.valid = true;

            // Compose 5-byte scan packet in normal scan mode
            uint8_t sync_quality = 0x01 | ((pkt.quality & 0x3F) << 2);
            uint8_t bytes[5] = {
                sync_quality,
                static_cast<uint8_t>(pkt.angle & 0xFF),
                static_cast<uint8_t>((pkt.angle >> 8) & 0xFF),
                static_cast<uint8_t>(pkt.distance & 0xFF),
                static_cast<uint8_t>((pkt.distance >> 8) & 0xFF)
            };

            // Send each byte on a clock edge
            for (int i = 0; i < 5; i++) {
                wait(clk.posedge_event());
                tx_data.write(bytes[i]);
            }
        }
    }

    // Constructor: Register the data generator thread
    SC_CTOR(RPLIDAR_Emulator) {
        SC_THREAD(data_generator);
        sensitive << clk.pos();
    }
};

// Module that simulates a controller interfacing with the RPLIDAR
SC_MODULE(RPLIDAR_Controller) {
    sc_in<bool> clk;           // System clock
    sc_in<bool> reset;         // Active-high reset
    sc_in<uint8_t> rx_data;    // Serial input from emulator
    sc_out<uint8_t> tx_data;   // Serial output to emulator
    sc_out<bool> tx_start;     // Command to start byte transmission
    sc_out<bool> pwm_motor;    // Motor control output
    sc_out<uint8_t> led_out;   // LED output for status visualization

    std::queue<uint8_t> cmd_queue;  // Queue for outgoing commands
    RPLIDAR_packet current_packet;  // Holds the currently parsed scan packet
    int byte_counter = 0;           // Byte index while parsing packets
    int heartbeat_counter = 0;      // Heartbeat counter
    bool led_heartbeat = false;     // Toggles LED for heartbeat

    // FSM that sends command bytes to the LIDAR (e.g., start scan)
    void command_fsm() {
        // Push initial scan command
        cmd_queue.push(0xA5);
        cmd_queue.push(0x20); // Normal Scan mode

        while (true) {
            wait(clk.posedge_event());

            if (reset.read()) {
                // Reset state
                while (!cmd_queue.empty()) cmd_queue.pop();
                cmd_queue.push(0xA5);
                cmd_queue.push(0x20);
                tx_start.write(false);
                pwm_motor.write(false); // Stop motor during reset
                continue;
            }

            // Turn on motor after reset
            pwm_motor.write(true);

            // If ready to send next command
            if (!cmd_queue.empty() && !tx_start.read()) {
                tx_data.write(cmd_queue.front());
                tx_start.write(true);
                cmd_queue.pop();
                wait(clk.posedge_event());
                tx_start.write(false);
                wait(100, SC_US); // Wait before next command
            }
        }
    }

    // Process to parse incoming 5-byte scan packets
    void packet_parser() {
        while (true) {
            wait(clk.posedge_event());

            if (reset.read()) {
                // Reset parser state
                byte_counter = 0;
                current_packet.valid = false;
                led_out.write(0);
                heartbeat_counter = 0;
                led_heartbeat = false;
                continue;
            }

            uint8_t data = rx_data.read();

            // Packet decoding state machine
            switch (byte_counter) {
                case 0: // Sync + Quality byte
                    if ((data & 0x03) == 0x01) { // Valid sync check
                        current_packet.quality = data >> 2;
                        byte_counter++;
                    }
                    break;
                case 1:
                    current_packet.angle = data;
                    byte_counter++;
                    break;
                case 2:
                    current_packet.angle |= (data << 8);
                    byte_counter++;
                    break;
                case 3:
                    current_packet.distance = data;
                    byte_counter++;
                    break;
                case 4:
                    current_packet.distance |= (data << 8);
                    current_packet.valid = true;
                    byte_counter = 0;
                    update_leds();   // Reflect data on LEDs
                    print_packet(); // Print to console
                    break;
            }
        }
    }

    // Update LED output based on received packet and heartbeat
    void update_leds() {
        uint8_t leds = led_out.read();

        // Toggle LED0 every 500ms (simulate 100MHz clock)
        if (++heartbeat_counter >= 50000000) {
            led_heartbeat = !led_heartbeat;
            heartbeat_counter = 0;
        }
        leds = (led_heartbeat ? 0x01 : 0x00); // LED0

        // LED1: Packet received flag
        if (current_packet.valid) leds |= 0x02;

        // LED3: Set if distance exceeds 6m
        if (current_packet.valid && current_packet.distance > 6000)
            leds |= 0x08;

        led_out.write(leds);
    }

    // Print received packet details to the console
    void print_packet() {
        if (current_packet.valid) {
            double angle_deg = current_packet.angle / 100.0;
            std::cout << sc_time_stamp() << " Packet: Quality=" << (int)current_packet.quality
                      << " Angle=" << angle_deg << "°"
                      << " Distance=" << current_packet.distance << "mm"
                      << std::endl;
        }
    }

    // Constructor: Register all processes
    SC_CTOR(RPLIDAR_Controller) {
        SC_THREAD(command_fsm);
        sensitive << clk.pos();
        async_reset_signal_is(reset, true);

        SC_THREAD(packet_parser);
        sensitive << clk.pos();
        async_reset_signal_is(reset, true);
    }
};

// Top-level simulation entry point
int sc_main(int argc, char* argv[]) {
    sc_clock clk("clk", 10, SC_NS); // 100 MHz clock
    sc_signal<bool> reset;
    sc_signal<uint8_t> rx_data, tx_data;
    sc_signal<bool> tx_start, pwm_motor;
    sc_signal<uint8_t> led_out;

    // Instantiate emulator and connect signals
    RPLIDAR_Emulator emulator("emulator");
    emulator.pwm_motor(pwm_motor);
    emulator.tx_data(rx_data);  // TX from emulator -> RX to controller
    emulator.rx_data(tx_data);  // RX to emulator <- TX from controller
    emulator.tx_start(tx_start);
    emulator.clk(clk);

    // Instantiate controller and connect signals
    RPLIDAR_Controller controller("controller");
    controller.clk(clk);
    controller.reset(reset);
    controller.rx_data(rx_data);
    controller.tx_data(tx_data);
    controller.tx_start(tx_start);
    controller.pwm_motor(pwm_motor);
    controller.led_out(led_out);

    // Set up waveform tracing
    sc_trace_file *tf = sc_create_vcd_trace_file("rplidar_sim");
    sc_trace(tf, clk, "clk");
    sc_trace(tf, reset, "reset");
    sc_trace(tf, rx_data, "rx_data");
    sc_trace(tf, tx_data, "tx_data");
    sc_trace(tf, tx_start, "tx_start");
    sc_trace(tf, pwm_motor, "pwm_motor");
    sc_trace(tf, led_out, "led_out");

    // Issue reset pulse
    reset.write(true);
    sc_start(100, SC_NS);
    reset.write(false);

    // Run simulation for 1 second
    sc_start(1, SC_SEC);

    // Close waveform file
    sc_close_vcd_trace_file(tf);
    return 0;
}
