NEIGHBOR=$1
PREFIX="room-64-64-8-random"

KRUHOBOTS_LIST=`cat kruhobots_room-64-64-8`
SCENARIOS_LIST=`cat scenarios_room-64-64-8`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
	grep "machine TIME" 'out_'$PREFIX'_n'$NEIGHBOR'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
    done
done
