# Target clock for the first synthesis: 100 MHz. Reset is asynchronous.
create_clock -name clk -period 10.000 [get_ports clk]
derive_clock_uncertainty
set_false_path -from [get_ports rst_n]
