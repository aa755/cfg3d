numLines=`wc -l $1 | cut -f 1 -d ' '  `
echo $numLines
newNumLines=$((numLines-3))
tail -n $newNumLines $1 
