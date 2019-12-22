NEIGHBOR=$1
PREFIX="warehouse-10-20-random"

KRUHOBOTS_LIST=`cat kruhobots_warehouse-10-20`
SCENARIOS_LIST=`cat scenarios_warehouse-10-20`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
	grep "machine TIME" 'out_'$PREFIX'_n'$NEIGHBOR'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
    done
done
