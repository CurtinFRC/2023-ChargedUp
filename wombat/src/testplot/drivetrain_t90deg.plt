set output "drivetrain_t90deg.png"
set multiplot layout 2, 2 title "Drivetrain Turn 90 Deg"

set yrange [-180:180]

set title "Heading"
plot "drivetrain_t90deg.csv" using 1:4 w l

set xrange[*:*]
set yrange [-3:3]

set title "Forward Speed"
plot "drivetrain_t90deg.csv" using 1:7 w l

set title "Left Speed"
plot "drivetrain_t90deg.csv" using 1:5 w l

set title "Right Speed"
plot "drivetrain_t90deg.csv" using 1:6 w l