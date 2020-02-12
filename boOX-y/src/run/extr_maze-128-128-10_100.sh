PREFIX="maze-128-128-10"

SCENARIOS_FILE="scenarios_"$PREFIX
ROBOTS_FILE="robots_"$PREFIX

ROBOTS_LIST=`cat $ROBOTS_FILE`
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
	grep "machine TIME" 'out-100_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.txt'
    done
done
