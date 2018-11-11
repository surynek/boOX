ROBOT_LIST=`cat robots_maps`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do                
    echo 'Generating map brc202d with '$ROBOTS' agents ...'
   ../main/gridgen_boOX --map-file=../../maps/brc202d.map '--N-agents='$ROBOTS '--mpf-file=brc202d_a'$ROBOTS'_'$SEED'.mpf' '--cpf-file=brc202d_a'$ROBOTS'_'$SEED'.cpf'
  done
done
