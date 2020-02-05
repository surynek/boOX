NEIGHBORHOOD=$1
SCENARIOS_LIST=`cat scenarios_ost003d`

for SCENARIO in $SCENARIOS_LIST;
do
    ./expr_ost003d_solve-ccbs.sh $NEIGHBORHOOD $SCENARIO &
done
