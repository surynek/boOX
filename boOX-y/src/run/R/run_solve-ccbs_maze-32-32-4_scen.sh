NEIGHBORHOOD=$1
SCENARIOS_LIST=`cat scenarios_maze`

for SCENARIO in $SCENARIOS_LIST;
do
    ./expr_maze-32-32-4_solve-ccbs.sh $NEIGHBORHOOD $SCENARIO &
done
