set output "drivetrain_fwd1m.png"
set multiplot layout 3, 2 title "Drivetrain Forward 1m"

set xrange [-2:2]
set yrange [-2:2]

set title "Position"
plot "drivetrain_fwd1m.csv" using 2:3 w l

set xrange[*:*]
set yrange [-3:3]

set title "Forward Speed"
plot "drivetrain_fwd1m.csv" using 1:7 w l

set title "Left Speed"
plot "drivetrain_fwd1m.csv" using 1:5 w l

set title "Right Speed"
plot "drivetrain_fwd1m.csv" using 1:6 w l

set yrange[-1:2]
set title "X Position"
plot "drivetrain_fwd1m.csv" using 1:2 w l

set title "Y Position"
plot "drivetrain_fwd1m.csv" using 1:3 w l