PREFIX="road-small"

KRUHOBOTS_LIST=`cat kruhobots_road-small`
SCENARIOS_LIST=`cat scenarios_road-small`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
	grep "machine TIME" 'out_'$PREFIX'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
    done
done
