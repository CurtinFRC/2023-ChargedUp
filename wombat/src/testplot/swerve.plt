set term png size 2400,1800 enhanced
set output "swerve.png"
set multiplot layout 3, 4 title "Swerve"

set xrange[-2:2]
set yrange[-2:2]

set title "XY"
plot "swerve.csv" using 2:3 w l

set xrange[*:*]

set title "X"
plot "swerve.csv" using 1:2 w l, \
    "" using 1:13 w l

set title "Y"
plot "swerve.csv" using 1:3 w l, \
    "" using 1:14 w l

set yrange[-180:180]

set title "Heading"
plot "swerve.csv" using 1:4 w l, \
    "" using 1:15 w l

set title "Module 1 Angle"
plot "swerve.csv" using 1:5 w l
set title "Module 2 Angle"
plot "swerve.csv" using 1:6 w l
set title "Module 3 Angle"
plot "swerve.csv" using 1:7 w l
set title "Module 4 Angle"
plot "swerve.csv" using 1:8 w l

set yrange[*:*]
set title "Module 1 Speed"
plot "swerve.csv" using 1:9 w l
set title "Module 2 Speed"
plot "swerve.csv" using 1:10 w l
set title "Module 3 Speed"
plot "swerve.csv" using 1:11 w l
set title "Module 4 Speed"
plot "swerve.csv" using 1:12 w l