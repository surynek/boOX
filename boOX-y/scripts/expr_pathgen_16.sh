ROBOT_LIST=`cat robots_16`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
  do
  for SEED in $SEED_LIST;
  do
    echo 'Generating path '$SIZE' with '$ROBOTS' agents ...'
    ../main/pathgen_boOX '--N-vertices='$SIZE '--N-agents='$ROBOTS '--seed='$SEED '--mpf-file=path_'$SIZE'_a'$ROBOTS'_'$SEED'.mpf' '--cpf-file=path_'$SIZE'_a'$ROBOTS'_'$SEED'.cpf'
  done
done
