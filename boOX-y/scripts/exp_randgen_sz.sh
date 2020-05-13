ROBOT_LIST=`cat robots_16`
SIZE_LIST=`cat sizes_16`

for SIZE in $SIZE_LIST;
do
  ROBOTS=$SIZE
  echo 'Generating random '$SIZE' with '$ROBOTS' agents ...'
  ../main/randgen_boOX '--walk' '--edge-probability=0.2' '--N-vertices='$SIZE '--N-agents='$ROBOTS '--mpf-file=rand_'$SIZE'_a'$ROBOTS'.mpf' '--cpf-file=rand_'$SIZE'_a'$ROBOTS'.cpf'
done
