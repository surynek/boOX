ROBOT_LIST=`cat robots_16`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
  do	      	      	      
  for SEED in $SEED_LIST;	
  do
      echo 'Generating star '$SIZE' with '$ROBOTS' agents ...'
      ../main/stargen_boOX '--N-vertices='$SIZE '--N-agents='$ROBOTS '--seed='$SEED '--N-agents='$ROBOTS '--mpf-file=star_'$SIZE'_a'$ROBOTS'_'$SEED'.mpf' '--cpf-file=star_'$SIZE'_a'$ROBOTS'_'$SEED'.cpf'
  done
done
