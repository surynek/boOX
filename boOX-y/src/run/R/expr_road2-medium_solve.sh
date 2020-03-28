SCENARIO=$1

TIMEOUT=`cat timeout`
PREFIX="road2-medium"

KRUHOBOTS_LIST=`cat kruhobots_road2-medium`

for KRUHOBOTS in $KRUHOBOTS_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF-R instance with '$KRUHOBOTS' kruhobots ...'
    ../../main/mapfR_solver_boOX '--timeout='$TIMEOUT '--input-mapR-file=map_'$PREFIX'.mapR' '--input-kruhoR-file='$PREFIX'-'$SCENARIO'_k'$KRUHOBOTS'.kruR' '--algorithm=smtcbsR*' '--output-file=solution.txt' > 'out_'$PREFIX'-'$SCENARIO'_k'$KRUHOBOTS'.txt'
done
