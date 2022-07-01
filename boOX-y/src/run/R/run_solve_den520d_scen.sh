NEIGHBORHOOD=$1
SCENARIOS_LIST=`cat scenarios_den520d`

for SCENARIO in $SCENARIOS_LIST;
do
    ./expr_den520d_solve.sh $NEIGHBORHOOD $SCENARIO &
done
