PREFIX="ost003d"
AGENTS="16"

SCENARIOS_FILE="scenarios_"$PREFIX
UNASSIGNED_FILE="unassigned_"$PREFIX

UNASSIGNED_LIST=`cat $UNASSIGNED_FILE`
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for UNASSIGNED in $UNASSIGNED_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	#echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF instance with '$UNASSIGNED' agents of '$AGENTS' agents ...'
	FILE_OUT='out_'$PREFIX'-'$SCENARIO'_a'$AGENTS'_u'$UNASSIGNED'-smtcbs.txt'
	FILE_SOL='solution_'$PREFIX'-random-'$SCENARIO'_a'$AGENTS'_u'$UNASSIGNED'-smtcbs.txt'
	if test -f "$FILE_SOL"; then
	  grep "machine TIME" $FILE_OUT
	else
	  echo "missing file for $UNASSIGNED unassigned agents"
	  echo "    missing file for $UNASSIGNED unassigned agents"
	fi
    done
done
