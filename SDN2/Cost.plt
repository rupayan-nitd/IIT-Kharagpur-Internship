set terminal postscript eps enhanced color solid colortext font "Times-Roman, 27"

set output 'Cost.eps' 
set size 1.0,0.75
#set key bottom right
#set key box bottom center
set key box center

set grid ytics lc rgb "#bbbbbb" lw 1 lt 0
set grid xtics lc rgb "#bbbbbb" lw 1 lt 0

set xlabel "Time (sec)"
set ylabel "Max. cost of 1 SS"

#set xtics ("30" 30, "60" 60, "90" 90, "120" 120, "150" 150, "180" 180, "210" 210, "240" 240, "270" 270, "300" 300)  #point in the x axis

set style line 1 lt 1 pt 2 lw 2 linecolor rgb "red"
set style line 2 lt 6 pt 1 lw 2 linecolor rgb "blue"
set style line 3 lt 3 pt 5 lw 2 linecolor rgb "#006400" #darkgreen
set style line 4 lt 1 pt 4 lw 2 linecolor rgb "blue" #"magenta"
set style line 5 lt 5 pt 6 lw 2 linecolor rgb "#9400D3" #darkviolet
set style line 6 lt 4 pt 8 lw 2 linecolor rgb "brown"
set style line 7 lt 1 pt 3 lw 2 linecolor rgb "black"



plot "Cost.txt" using 1:2:3:4 index 0 with errorlines ls 2 title "Proposed-Scheme", \
     "" using 1:2:3:4 index 1 with errorlines ls 1 title "Random-Scheme",\
     "" using 1:2:3:4 index 2 with errorlines ls 3 title "Centralized-Scheme"


