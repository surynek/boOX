ROBOT_LIST=`cat robots_08x08`
SIZE=8

for ROBOTS in $ROBOT_LIST;
do
  echo 'Generating grid '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
  ../main/gridgen_boOX --walk '--x-size=8' '--y-size=8' '--N-agents='$ROBOTS '--mpf-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.mpf' '--N-agents='$ROBOTS '--cpf-file=grid_'$SIZE'x'$SIZE'_a'$ROBOTS'.cpf'
done
