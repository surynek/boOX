ALGO=$1
PREFIX='empty-16-16'

SCENARIOS_FILE="scenarios_"$PREFIX
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for SCENARIO in $SCENARIOS_LIST;
do
    './exprH_'$PREFIX'_solve-'$ALGO'.sh' $SCENARIO &
done