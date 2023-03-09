SCENARIO=$1

TIMEOUT=`cat timeout`
PREFIX="w_wundedcoast"
SCENARIO_PREFIX=$PREFIX"-random"

ROBOTS_FILE='robots_'$PREFIX
ROBOTS_LIST=`cat $ROBOTS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
    timeout $TIMEOUT ../lazy-cbs --map $PREFIX'.map.ecbs' --agents $PREFIX'-'$SCENARIO'_a'$ROBOTS'.lcbs' --focal_w 1 --highway_w 1 --highway HONG > 'out_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.lcbs.txt'
done
