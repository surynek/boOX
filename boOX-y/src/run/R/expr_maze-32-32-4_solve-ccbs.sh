NEIGHBOR=$1
SCENARIO=$2

TIMEOUT=`cat timeout`
PREFIX="maze-32-32-4-random"

KRUHOBOTS_LIST=`cat kruhobots_maze`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
    ../../../../ccbs/CCBS-$NEIGHBOR 'map_'$PREFIX'.xml' $PREFIX'-'$SCENARIO'_k'$KRUHOBOTS'.xml' > 'out-ccbs_'$PREFIX'_n'$NEIGHBOR'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
done
