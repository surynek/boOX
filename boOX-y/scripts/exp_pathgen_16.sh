ROBOT_LIST=`cat robots_16`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  echo 'Generating path '$SIZE'x'$SIZE' with '$ROBOTS' agents ...'
  ../main/pathgen_boOX '--N-vertices='$SIZE '--N-agents='$ROBOTS '--mpf-file=path_'$SIZE'_a'$ROBOTS'.mpf' '--cpf-file=path_'$SIZE'_a'$ROBOTS'.cpf'
done
