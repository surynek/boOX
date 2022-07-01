NEIGHBORHOOD=$1
SCENARIOS_LIST=`cat scenarios_room-64-64-8`

for SCENARIO in $SCENARIOS_LIST;
do
    ./expr_room-64-64-8_solve.sh $NEIGHBORHOOD $SCENARIO &
done
