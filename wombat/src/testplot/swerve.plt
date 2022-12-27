set term png size 2400,1200 enhanced
set output "swerve.png"
set multiplot layout 2, 4 title "Swerve"

set xrange[-2:2]
set yrange[-2:2]

set title "XY"
plot "swerve.csv" using 2:3 w l

set xrange[*:*]

set title "X"
plot "swerve.csv" using 1:2 w l

set title "Y"
plot "swerve.csv" using 1:3 w l

set yrange[-180:180]

set title "Heading"
plot "swerve.csv" using 1:4 w l 

set title "Module 1"
plot "swerve.csv" using 1:5 w l
set title "Module 2"
plot "swerve.csv" using 1:6 w l
set title "Module 3"
plot "swerve.csv" using 1:7 w l
set title "Module 4"
plot "swerve.csv" using 1:8 w l
