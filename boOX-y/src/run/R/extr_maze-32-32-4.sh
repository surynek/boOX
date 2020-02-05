NEIGHBOR=$1
PREFIX="maze-32-32-4-random"

KRUHOBOTS_LIST=`cat kruhobots_maze`
SCENARIOS_LIST=`cat scenarios_maze`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
	grep "machine TIME" 'out_'$PREFIX'_n'$NEIGHBOR'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
    done
done
