ALGO=$1
PREFIX='warehouse-10-20-10-2-1'

SCENARIOS_FILE="scenarios_"$PREFIX
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for SCENARIO in $SCENARIOS_LIST;
do
    './exprH_'$PREFIX'_solve-'$ALGO'.sh' $SCENARIO &
done
