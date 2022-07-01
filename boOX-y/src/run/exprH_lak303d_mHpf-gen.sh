PREFIX="lak303d"
SCENARIO_PREFIX=$PREFIX"-random"

ROBOTS_FILE='robotsH_'$PREFIX
SCENARIOS_FILE='scenarios_'$PREFIX
TASK_FILE='task_'$PREFIX

TASKS=`cat $TASK_FILE`
ROBOTS_LIST=`cat $ROBOTS_FILE`
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Generating '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
	../main/moviscen_convert_boOX '--input-movi-map-file='$PREFIX'.map' '--input-movi-scen-file='$SCENARIO_PREFIX'-'$SCENARIO'.scen' '--output-mHpf-file='$PREFIX'-'$SCENARIO'_a'$ROBOTS'-t'$TASKS'.mHpf' '--N-agents='$ROBOTS '--N-tasks='$TASKS
    done
done
