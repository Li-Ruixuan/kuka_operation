import rosgraph

if rosgraph.is_master_online():
    exit( 0 )
else:
    exit( 1 )