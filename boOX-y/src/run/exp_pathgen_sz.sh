ROBOT_LIST=`cat robots_16`
SIZE_LIST=`cat sizes_16`

for SIZE in $SIZE_LIST;
do
  ROBOTS=$SIZE
  echo 'Generating path '$SIZE' with '$ROBOTS' agents ...'
  ../main/pathgen_boOX '--N-vertices='$SIZE '--N-agents='$ROBOTS '--mpf-file=path_'$SIZE'_a'$ROBOTS'.mpf' '--cpf-file=path_'$SIZE'_a'$ROBOTS'.cpf'
done
