PREFIX="road2-small"

KRUHOBOTS_LIST=`cat kruhobots_road2-small`
SCENARIOS_LIST=`cat scenarios_road2-small`

for SCENARIO in $SCENARIOS_LIST;
do
    for KRUHOBOTS in $KRUHOBOTS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
	grep "cost     =" 'out_'$PREFIX'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
    done
done
