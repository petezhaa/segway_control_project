onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -format Analog-Step -height 84 -max 32753.000000000004 -min -32768.0 -radix decimal -childformat {{{/steering_response_tb/iPHYS/omega_lft[15]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[14]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[13]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[12]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[11]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[10]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[9]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[8]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[7]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[6]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[5]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[4]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[3]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[2]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[1]} -radix decimal} {{/steering_response_tb/iPHYS/omega_lft[0]} -radix decimal}} -subitemconfig {{/steering_response_tb/iPHYS/omega_lft[15]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[14]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[13]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[12]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[11]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[10]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[9]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[8]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[7]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[6]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[5]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[4]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[3]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[2]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[1]} {-height 17 -radix decimal} {/steering_response_tb/iPHYS/omega_lft[0]} {-height 17 -radix decimal}} /steering_response_tb/iPHYS/omega_lft
add wave -noupdate -format Analog-Step -height 84 -max 30105.0 -min -11988.0 -radix decimal /steering_response_tb/iPHYS/omega_rght
add wave -noupdate -format Analog-Step -height 84 -max 4094.9999999999995 -min -2048.0 -radix decimal /steering_response_tb/rider_lean
add wave -noupdate -radix hexadecimal /steering_response_tb/steerPot
add wave -noupdate /steering_response_tb/prev_lft_spd
add wave -noupdate /steering_response_tb/prev_rght_spd
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {620877977 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 225
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 2
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {81516355 ns} {718385277 ns}
