# Represents the occupancy of grids in a map
string name     # name of map image
uint16 width    # no. grids along width of map
uint16 height   # no. grids along height of map
uint8 gridsize   # length of each square grid in pixels
bool[] occupancy   # whether a grid is occupied or not
