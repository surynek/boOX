SCENARIOS_LIST=`cat scenarios_road-small`

for SCENARIO in $SCENARIOS_LIST;
do
    ./expr_road-small_solve.sh $SCENARIO &
done
