NEIGHBOR=$1
SCENARIO=$2

TIMEOUT=`cat timeout`
PREFIX="den520d-random"

KRUHOBOTS_LIST=`cat kruhobots_den520d`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
    ../../main/mapfR_solver_boOX '--timeout='$TIMEOUT '--input-mapR-file=map_'$PREFIX'_n'$NEIGHBOR'.mapR' '--input-kruhoR-file='$PREFIX'-'$SCENARIO'_k'$KRUHOBOTS'.kruR' '--algorithm=smtcbsR*' '--output-file=solution.txt' > 'out_'$PREFIX'_n'$NEIGHBOR'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
done
