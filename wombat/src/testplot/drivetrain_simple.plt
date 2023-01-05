set output "drivetrain_simple.png"
set multiplot layout 2, 2 title "Drivetrain Simple"

set title "Position"
plot "drivetrain_simple.csv" using 2:3 w l

set yrange [-3:3]

set title "Forward Speed"
plot "drivetrain_simple.csv" using 1:7 w l

set title "Left"
plot "drivetrain_simple.csv" using 1:5 w l

set title "Right"
plot "drivetrain_simple.csv" using 1:6 w l