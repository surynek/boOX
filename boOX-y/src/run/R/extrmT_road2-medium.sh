PREFIX="road-medium"

KRUHOBOTS_LIST=`cat kruhobots_road-medium`
SCENARIOS_LIST=`cat scenarios_road-medium`

for SCENARIO in $SCENARIOS_LIST;
do
    for KRUHOBOTS in $KRUHOBOTS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
	grep "makespan =" 'out_'$PREFIX'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
    done
done
