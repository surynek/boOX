PREFIX="empty-16-16"

SCENARIOS_FILE="scenarios_"$PREFIX
ROBOTS_FILE="robotsH_"$PREFIX
TASK_FILE="task_"$PREFIX

ROBOTS_LIST=`cat $ROBOTS_FILE`
SCENARIOS_LIST=`cat $SCENARIOS_FILE`
TASKS=`cat $TASK_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
	grep "machine TIME" 'out-SMT_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'-t'$TASKS'.txt'
    done
done
