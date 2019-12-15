NEIGHBOR=$1
PREFIX="den520d-random"

KRUHOBOTS_LIST=`cat kruhobots_den520d`
SCENARIOS_LIST=`cat scenarios_den520d`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
	grep "machine TIME" 'out_'$PREFIX'_n'$NEIGHBOR'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
    done
done
