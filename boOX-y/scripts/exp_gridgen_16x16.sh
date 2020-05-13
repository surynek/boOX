ROBOT_LIST=`cat robots_16x16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  echo 'Generating grid '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
  ../main/gridgen_boOX --walk '--x-size=16' '--y-size=16' '--N-agents='$ROBOTS '--mpf-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.mpf' '--N-agents='$ROBOTS '--cpf-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.cpf'
done
