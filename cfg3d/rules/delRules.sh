#ls rule_* | xargs wc -l | sort | grep "     $1" | cut -f 1 
ls rule_* | xargs wc -l | sort | grep "     $1" | cut -f 1 | cut -f 7 -d ' ' | xargs rm
