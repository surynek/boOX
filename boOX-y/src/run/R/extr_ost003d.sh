NEIGHBOR=$1
PREFIX="ost003d-random"

KRUHOBOTS_LIST=`cat kruhobots_ost003d`
SCENARIOS_LIST=`cat scenarios_ost003d`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
	grep "machine TIME" 'out_'$PREFIX'_n'$NEIGHBOR'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
    done
done
