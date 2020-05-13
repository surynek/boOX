ROBOT_LIST=`cat robots_16`
SIZE_LIST=`cat sizes_16`

for SIZE in $SIZE_LIST;
do
  ROBOTS=$SIZE
  echo 'Generating grid '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
  ../main/stargen_boOX '--N-vertices='$SIZE '--N-agents='$ROBOTS '--mpf-file=clique_'$SIZE'_a'$ROBOTS'.mpf' '--cpf-file=clique_'$SIZE'_a'$ROBOTS'.cpf'
done
