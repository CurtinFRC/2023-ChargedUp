set output "swerve_module.png"
set multiplot layout 2, 1 title "Swerve Module"

set title "Position"
plot "swerve_module.csv" using 1:2 w l

set title "Forward Speed"
plot "swerve_module.csv" using 1:3 w l
