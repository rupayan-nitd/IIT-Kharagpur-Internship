set terminal postscript eps enhanced color solid colortext font "Times-Roman, 27"
set output 'Cost_SS.eps'
set size 1.0,0.82


set style line 1 lt 1 lw 1.0 pt 1 linecolor rgb "dark-green"
set style line 2 lt 1 lw 0.99 pt 2 linecolor rgb "red"
set style line 3 lt 1 lw 0.99 pt 6 linecolor rgb "blue"
set style line 4 lt 1 lw 0.99 pt 9 linecolor rgb "gray"
set style line 5 lt 1 lw 0.99 pt 10 linecolor rgb "brown"
set style line 6 lt 1 lw 0.99 pt 11 linecolor rgb "navy"
set style line 7 lt 1 lw 0.99 pt 9 linecolor rgb "#696969"

set grid ytics lc rgb "#bbbbbb" lw 1 lt 0
set grid xtics lc rgb "#bbbbbb" lw 1 lt 0

set style fill pattern border
set style histogram errorbars gap 1 lw 3.5
set style data histograms
set boxwidth 0.8
set bars 1

set yr[2000:35000]

#set ytics 0,400,1400
set key box top right
set xlabel "Number of SS"
set ylabel "Max. Cost of a SS" offset 2


plot 'Cost_SS.txt' using 2:3:4:xtic(1) 	index 0  title "Proposed" 	fs pattern 5 ls 1, \
	 '' using 2:3:4 			index 1  title "Random" 		fs pattern 2 ls 3, \
	 '' using 2:3:4 			index 2  title "Centralized" 		fs pattern 4 ls 6
