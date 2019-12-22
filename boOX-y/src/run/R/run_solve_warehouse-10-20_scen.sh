NEIGHBORHOOD=$1
SCENARIOS_LIST=`cat scenarios_warehouse-10-20`

for SCENARIO in $SCENARIOS_LIST;
do
    ./expr_warehouse-10-20_solve.sh $NEIGHBORHOOD $SCENARIO &
done
