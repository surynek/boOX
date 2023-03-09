PREFIX="maze-32-32-4"

SCENARIOS_FILE="scenarios_"$PREFIX
ROBOTS_FILE="robots_"$PREFIX

ROBOTS_LIST=`cat $ROBOTS_FILE`
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	#echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
	FILE_OUT='out_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.smtcbs++.txt'
	FILE_SOL='solution_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.smtcbs++.txt'
	if test -f "$FILE_SOL"; then
	  grep "machine TIME" $FILE_OUT
	else
	  echo "missing file for $ROBOTS robots"
	  echo "    missing file for $ROBOTS robots"
	fi
    done
done
