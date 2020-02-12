NEIGHBOR=$1
PREFIX="lak303d"
SCENARIO_PREFIX=$PREFIX'-random'

ROBOTS_LIST=`cat $ROBOTS_FILE`
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
	grep "machine TIME" 'out-mtx_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.txt'
    done
done
