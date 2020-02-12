PREFIX="maze-128-128-10"
SCENARIO_PREFIX=$PREFIX"-random"

ROBOTS_FILE='robots_'$PREFIX
SCENARIOS_FILE='scenarios_'$PREFIX

ROBOTS_LIST=`cat $ROBOTS_FILE`
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Generating '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
	../main/moviscen_convert_boOX '--input-movi-map-file='$PREFIX'.map' '--input-movi-scen-file='$SCENARIO_PREFIX'-'$SCENARIO'.scen' '--output-mpf-file='$PREFIX'-'$SCENARIO'_a'$ROBOTS'.mpf' '--output-bgu-file='$PREFIX'-'$SCENARIO'_a'$ROBOTS'.bgu' '--N-agents='$ROBOTS
    done
done
