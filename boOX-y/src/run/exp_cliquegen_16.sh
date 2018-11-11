ROBOT_LIST=`cat robots_16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  echo 'Generating grid '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
  ../main/stargen_boOX '--N-vertices='$SIZE '--N-agents='$ROBOTS '--mpf-file=clique_'$SIZE'_a'$ROBOTS'.mpf' '--cpf-file=clique_'$SIZE'_a'$ROBOTS'.cpf'
done
