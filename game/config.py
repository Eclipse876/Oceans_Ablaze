# Simulation & world constants (no logic)
from dataclasses import dataclass

import arcade.color
from arcade.color import BLACK

DT = 1.0 / 60.0  # fixed simulation timestep (seconds)
WORLD_UNITS = "meters"
WORLD_WIDTH = 8000  # meters (game world space)
WORLD_HEIGHT = 8000  # meters
VIEWPORT_WIDTH = 1920  # initial window size (pixels)
VIEWPORT_HEIGHT = 1080
CAMERA_PIXEL_PER_METER_AT_ZOOM1 = 0.5

# Map Colors
OCEAN_COLOR = BLACK  # ui-only color tuple; not “logic”
LAND_OUTLINE_COLOR = arcade.color.WHITE
LAND_FILL_COLOR = (40, 40, 40)  # Dark Grey

# Unit Colors
FRIENDLY_COLOR = arcade.color.GREEN
HOSTILE_COLOR = arcade.color.RED
NEUTRAL_COLOR = arcade.color.MAGENTA
UNKNOWN_COLOR = arcade.color.ORANGE


@dataclass
class ShipSpec:
    name: str
    length_m: float
    beam_m: float
    radius_m: float  # physical hull radius (clearance)
    safety_m: float  # buffer beyond
    max_speed_mps: float
    max_accel_mps2: float
    max_lat_accel_mps2: float #Turning "grip", effects the turning radius (v^2/R = max_lat)
                                #Higher = tighter turns


# Example specs ("Name", length, beam, radius of the ship circle, extra safety zone,
#                                                       max speed, max accel, turning rate)
FRIGATE   = ShipSpec("Frigate",   150.0, 20.0, 20.0, 15.0, 18.0,  12.0, 3.0)
SUBMARINE = ShipSpec("Submarine", 120.0, 20.0, 25.0, 40.0, 15.0,  10.0, 4.0)
DESTROYER = ShipSpec("Destroyer", 170.0, 22.0, 22.0, 20.0, 20.0,  12.0, 3.5)
CRUISER   = ShipSpec("Cruiser",   200.0, 25.0, 25.0, 20.0, 22.0,  12.0, 4.0)
CARRIER   = ShipSpec("Carrier",   300.0, 40.0, 40.0, 30.0, 16.0,  10.0, 2.5)
PATROL    = ShipSpec("Patrol",    50.0, 10.0, 10.0, 10.0, 28.0,  14.0, 5.0)


