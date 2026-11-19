onbreak resume
onerror resume
vsim -voptargs=+acc work.BallBeamCore_tb

add wave sim:/BallBeamCore_tb/u_BallBeamCore/clk
add wave sim:/BallBeamCore_tb/u_BallBeamCore/reset
add wave sim:/BallBeamCore_tb/u_BallBeamCore/echo_in
add wave sim:/BallBeamCore_tb/u_BallBeamCore/uart_rxd
add wave sim:/BallBeamCore_tb/u_BallBeamCore/trig_out
add wave sim:/BallBeamCore_tb/trig_out_ref
add wave sim:/BallBeamCore_tb/u_BallBeamCore/servo_out
add wave sim:/BallBeamCore_tb/servo_out_ref
add wave sim:/BallBeamCore_tb/u_BallBeamCore/uart_txd
add wave sim:/BallBeamCore_tb/uart_txd_ref
add wave sim:/BallBeamCore_tb/u_BallBeamCore/led0
add wave sim:/BallBeamCore_tb/led0_ref
add wave sim:/BallBeamCore_tb/u_BallBeamCore/led1
add wave sim:/BallBeamCore_tb/led1_ref
add wave sim:/BallBeamCore_tb/u_BallBeamCore/led2
add wave sim:/BallBeamCore_tb/led2_ref
add wave sim:/BallBeamCore_tb/u_BallBeamCore/led3
add wave sim:/BallBeamCore_tb/led3_ref
run -all
