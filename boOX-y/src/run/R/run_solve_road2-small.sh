SCENARIOS_LIST=`cat scenarios_road2-small`

for SCENARIO in $SCENARIOS_LIST;
do
    ./expr_road2-small_solve.sh $SCENARIO &
done
