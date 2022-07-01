NEIGHBOR=$1
SCENARIO=$2

TIMEOUT=`cat timeout`
PREFIX="ost003d-random"

KRUHOBOTS_LIST=`cat kruhobots_ost003d`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
    ../../../../ccbs/CCBS-$NEIGHBOR 'map_'$PREFIX'.xml' $PREFIX'-'$SCENARIO'_k'$KRUHOBOTS'.xml' > 'out-ccbs_'$PREFIX'_n'$NEIGHBOR'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
done
