# p [0:300][0:300]"test.txt" u 2:3, "" u 4:5 w lp lt 7, "" u 6:7 w p, "" u 6:7:($8*sqrt(5.991)):($9*sqrt(5.991)):10 w ellipses
getValue(row,col,filename) = system('awk ''{if (NR == '.row.') print $'.col.'}'' '.filename.'')

set terminal gif animate delay 5 optimize size 640,480 font "Courier, 12"
set output 'ekf.gif'
do for [i=1: 1800] {
  cx = getValue(i, 2, "../build/test.txt")
  cy = getValue(i, 3, "../build/test.txt")
  plot [cx-5:cx+5][cy-5:cy+5]"../build/test.txt" every ::::i using 2:3 w lp pt 7 ps 1 ti "True Position", \
       "" every ::::i u 4:5 w p pt 7 ti "Measured Position", \
       "" every ::::i u 6:7 w lp pt 7 ti "Estimated Position", \
       "" every ::i::i u 6:7:($8*sqrt(5.991)):($9*sqrt(5.991)):10 w ellipses lw 2.0 ti "Confidence Region"
}
unset output