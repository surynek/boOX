PREFIX="random-32-32-20"
AGENTS="32"
SCENARIO_PREFIX=$PREFIX"-random"

UNASSIGNED_FILE='unassigned_'$PREFIX
SCENARIOS_FILE='scenarios_'$PREFIX

UNASSIGNED_LIST=`cat $UNASSIGNED_FILE`
SCENARIOS_LIST=`cat $SCENARIOS_FILE`


for UNASSIGNED in $UNASSIGNED_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Generating '$PREFIX' scenario '$SCENARIO' MAPF instance with '$UNASSIGNED' unassigned agents of '$AGENTS ' agents ...'
	../../../main/moviscen_convert_boOX '--input-movi-map-file='$PREFIX'.map' '--input-movi-scen-file='$SCENARIO_PREFIX'-'$SCENARIO'.scen' '--output-mpf-file='$SCENARIO_PREFIX'-'$SCENARIO'_a'$AGENTS'_u'$UNASSIGNED'.npf' '--N-unassigned='$UNASSIGNED '--N-agents='$AGENTS
    done
done
