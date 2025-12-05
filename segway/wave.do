onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -format Analog-Step -height 74 -max 24648.0 /Segway_tb/iPHYS/theta_platform
add wave -noupdate -divider PID
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/ptch
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/ptch_rt
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/clk
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/rst_n
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/vld
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/pwr_up
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/rider_off
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/PID_cntrl
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/ptch_err_sat
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/P_term
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/integrator
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/accum_val
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/intergrator_MUX_1
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/I_term
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/D_term
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/PID_sum
add wave -noupdate -radix binary /Segway_tb/iDUT/iBAL/iPID/long_tmr
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/ss_tmr
add wave -noupdate /Segway_tb/iDUT/iBAL/iPID/overflowed
add wave -noupdate -divider tb
add wave -noupdate /Segway_tb/rider_lean
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {18401629 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 292
configure wave -valuecolwidth 172
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {51907744 ps}
