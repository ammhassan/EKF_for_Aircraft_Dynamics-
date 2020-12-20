set datafile separator ","
set autoscale fix
set key outside right center
set grid

# First plot
set terminal qt 0
set xlabel 'Time (s)' font "Times-Roman,14"
set ylabel 'Pitch Rate (deg/s)' font "Times-Roman,14"

plot 'outputs.csv' using 1:5 with lines lc rgb "red" lw 2 title "True",\
     'outputs.csv' using 1:6 with lines lc rgb "black" lw 0.5 title "Measured",\
     'outputs.csv' using 1:7 with lines lc rgb "green" lw 2 title "Estimated"


# Second plot
set terminal qt 1
set xlabel 'Time (s)' font "Times-Roman,14"
set ylabel 'Angle of Attack (deg)' font "Times-Roman,14"

plot 'outputs.csv' using 1:2 with lines lc rgb "red" lw 2 title "True",\
     'outputs.csv' using 1:3 with lines lc rgb "black" lw 0.5 title "Measured",\
     'outputs.csv' using 1:4 with lines lc rgb "green" lw 2 title "Estimated"


pause -1
