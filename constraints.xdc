# Clock signal
set_property -dict { PACKAGE_PIN K17   IOSTANDARD LVCMOS33 } [get_ports { clk_100mhz }]; #IO_L12P_T1_MRCC_35 Sch=sysclk
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports { clk_100mhz }];

# Reset (using button 0)
set_property -dict { PACKAGE_PIN K18   IOSTANDARD LVCMOS33 } [get_ports { reset }]; #IO_L12N_T1_MRCC_35 Sch=btn[0]

# UART Interface (using PMOD JE pins)
set_property -dict { PACKAGE_PIN V12   IOSTANDARD LVCMOS33 } [get_ports { rplidar_rx }]; #IO_L4P_T0_34 Sch=je[1]
set_property -dict { PACKAGE_PIN W16   IOSTANDARD LVCMOS33 } [get_ports { rplidar_tx }]; #IO_L18N_T2_34 Sch=je[2]

# Motor PWM (using PMOD JE pin 3)
set_property -dict { PACKAGE_PIN J15   IOSTANDARD LVCMOS33 } [get_ports { pwm_motor }]; #IO_25_35 Sch=je[3]

# Debug LEDs
set_property -dict { PACKAGE_PIN M14   IOSTANDARD LVCMOS33 } [get_ports { led_out[0] }]; #IO_L23P_T3_35 Sch=led[0]
set_property -dict { PACKAGE_PIN M15   IOSTANDARD LVCMOS33 } [get_ports { led_out[1] }]; #IO_L23N_T3_35 Sch=led[1]
set_property -dict { PACKAGE_PIN G14   IOSTANDARD LVCMOS33 } [get_ports { led_out[2] }]; #IO_0_35 Sch=led[2]
set_property -dict { PACKAGE_PIN D18   IOSTANDARD LVCMOS33 } [get_ports { led_out[3] }]; #IO_L3N_T0_DQS_AD1N_35 Sch=led[3]
#set_property -dict { PACKAGE_PIN M14   IOSTANDARD LVCMOS33 } [get_ports { debug_leds[0] }]; #IO_L23P_T3_35 Sch=led[0]
#set_property -dict { PACKAGE_PIN M15   IOSTANDARD LVCMOS33 } [get_ports { debug_leds[1] }]; #IO_L23N_T3_35 Sch=led[1]
#set_property -dict { PACKAGE_PIN G14   IOSTANDARD LVCMOS33 } [get_ports { debug_leds[2] }]; #IO_0_35 Sch=led[2]
#set_property -dict { PACKAGE_PIN D18   IOSTANDARD LVCMOS33 } [get_ports { debug_leds[3] }]; #IO_L3N_T0_DQS_AD1N_35 Sch=led[3]
set_property -dict { PACKAGE_PIN V16   IOSTANDARD LVCMOS33 } [get_ports { led_out[4] }]; #IO_L18P_T2_34 Sch=led6_r
set_property -dict { PACKAGE_PIN F17   IOSTANDARD LVCMOS33 } [get_ports { led_out[5] }]; #IO_L6N_T0_VREF_35 Sch=led6_g
set_property -dict { PACKAGE_PIN M17   IOSTANDARD LVCMOS33 } [get_ports { led_out[6] }]; #IO_L8P_T1_AD10P_35 Sch=led6_b
set_property -dict { PACKAGE_PIN U14   IOSTANDARD LVCMOS33 } [get_ports { led_out[7] }]; #IO_L11P_T1_SRCC_34 Sch=jd_p[3]

# Debug signals (using remaining LEDs)
#set_property -dict { PACKAGE_PIN U15   IOSTANDARD LVCMOS33 } [get_ports { debug[0] }]; #IO_L11N_T1_SRCC_34 Sch=jd_n[3]
#set_property -dict { PACKAGE_PIN V17   IOSTANDARD LVCMOS33 } [get_ports { debug[1] }]; #IO_L21P_T3_DQS_34 Sch=jd_p[4]
#set_property -dict { PACKAGE_PIN V18   IOSTANDARD LVCMOS33 } [get_ports { debug[2] }]; #IO_L21N_T3_DQS_34 Sch=jd_n[4]
#set_property -dict { PACKAGE_PIN H15   IOSTANDARD LVCMOS33 } [get_ports { debug[3] }]; #IO_L19P_T3_35 Sch=je[4]
#set_property -dict { PACKAGE_PIN U15   IOSTANDARD LVCMOS33 } [get_ports { led_out[0] }]; #IO_L11N_T1_SRCC_34 Sch=jd_n[3]
#set_property -dict { PACKAGE_PIN V17   IOSTANDARD LVCMOS33 } [get_ports { led_out[1] }]; #IO_L21P_T3_DQS_34 Sch=jd_p[4]
#set_property -dict { PACKAGE_PIN V18   IOSTANDARD LVCMOS33 } [get_ports { led_out[2] }]; #IO_L21N_T3_DQS_34 Sch=jd_n[4]
#set_property -dict { PACKAGE_PIN H15   IOSTANDARD LVCMOS33 } [get_ports { led_out[3] }]