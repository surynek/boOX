SCENARIO=$1
AGENTS="32"

TIMEOUT=`cat timeout`
PREFIX="random-32-32-20"
SCENARIO_PREFIX=$PREFIX"-random"

UNASSIGNED_FILE='UNASSIGNED_'$PREFIX
UNASSIGNED_LIST=`cat $UNASSIGNED_FILE`

for UNASSIGNED in $UNASSIGNED_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF instance with '$UNASSIGNED' unassigned agents of '$AGENTS' agents...'
    ../../../main/rota_solver_boOX '--timeout='$TIMEOUT  '--input-file='$PREFIX'-'$SCENARIO'_a'$AGENTS'_u'$UNASSIGNED'.npf' '--algorithm=smtcbs++' '--output-file=solution_'$PREFIX'-'$SCENARIO'_a'$AGENTS'_u'$UNASSIGNED'.txt' > 'out_'$PREFIX'-'$SCENARIO'_a'$AGENTS'_u'$UNASSIGNED'.txt'
done
