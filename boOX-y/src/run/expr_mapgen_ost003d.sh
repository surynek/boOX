ROBOT_LIST=`cat robots_maps`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do                
    echo 'Generating map ost003d with '$ROBOTS' agents ...'
   ../main/gridgen_boOX --map-file=../../maps/ost003d.map '--N-agents='$ROBOTS '--mpf-file=ost003d_a'$ROBOTS'_'$SEED'.mpf' '--cpf-file=ost003d_a'$ROBOTS'_'$SEED'.cpf'
  done
done
