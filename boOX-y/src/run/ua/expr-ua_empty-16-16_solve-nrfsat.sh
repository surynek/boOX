SCENARIO=$1
AGENTS="64"

TIMEOUT=`cat timeout`
PREFIX="empty-16-16"
SCENARIO_PREFIX=$PREFIX"-random"

UNASSIGNED_FILE='unassigned_'$PREFIX
UNASSIGNED_LIST=`cat $UNASSIGNED_FILE`

for UNASSIGNED in $UNASSIGNED_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF instance with '$UNASSIGNED' unassigned agents of '$AGENTS' agents...'
    ../../main/rota_solver_boOX '--timeout='$TIMEOUT  '--input-file='$SCENARIO_PREFIX'-'$SCENARIO'_a'$AGENTS'_u'$UNASSIGNED'.npf' '--algorithm=nrfsat' '--output-file=solution_'$SCENARIO_PREFIX'-'$SCENARIO'_a'$AGENTS'_u'$UNASSIGNED'.txt' > 'out_'$PREFIX'-'$SCENARIO'_a'$AGENTS'_u'$UNASSIGNED'.txt'
done
