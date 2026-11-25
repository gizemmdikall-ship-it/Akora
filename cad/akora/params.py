"""AKORA master dimensions (mm).

Source drawing: cad/reference/teknik_cizim.jpg (PDF poster says "Ölçek 1/40").
Drawing values are in CENTIMETRES - verified three ways:
  * the poster compares the folded robot with a 180 cm person (robot ~160 cm tall),
  * 600 / 170 / 200 / 165 give a 6.0 m bridge on a 1.7 m x 2.0 m vehicle,
  * deck height read from the side view (~90 cm) matches a person stepping onto it.
Values tagged [D] are read directly from a dimension on the drawing.
Values tagged [S] are scaled off the drawing geometry (no dimension given).
Values tagged [A] are engineering assumptions added to complete the design.
"""

# ------------------------------------------------------------------ overall
OPEN_LENGTH = 6000          # [D] 600  - fully deployed, rear stair foot -> front stair foot
BODY_LENGTH = 1700          # [D] 170  - track / hull length
TOTAL_WIDTH = 2000          # [D] 200  - front view overall width
BODY_WIDTH = 1300           # [D] 130  - hull between the tracks
TRACK_WIDTH = 350           # [D] 35 / 34.5 - each track module
DEPLOYED_HEIGHT = 1900      # user: railing 100 cm above the deck (drawing showed 165 total = 75 cm railing)
DECK_Z = 900                # [S] walkway height above ground (side view)
PLATFORM_WIDTH = 1000       # [D] 100  - platform outer width (top view)
PLATFORM_CLEAR = 900        # [D] 90   - between side profiles (top view)
RAIL_SPACING = 800          # [D] 80   - railing centre lines (front view)
RAIL_HEIGHT = DEPLOYED_HEIGHT - DECK_Z   # 1000 above deck
P1_LEN = 1100               # [D] 110  - platform 1, fixed on the hull
P2_LEN = 1105               # [D] 110.50 - platform 2
P3_LEN = 1200               # [D] 120  - intermediate / end platform
FRONT_STAIR_LEN = 1200      # [D] 120  - front ramp-stair length (slope)
FRONT_STAIR_W = 850         # [D] 85
REAR_STAIR_W = 900          # [D] 90 (inner), 100 outer
REAR_STEP_DEPTH = 350       # [D] 35
REAR_STEP_RISE = 180        # [D] 20 nominal (rounded to 5 equal rises of 180 to reach 900)
SUPPORT_LEG_H = 650         # [D] 65  - leg column below the platform frame
SUPPORT_LEG_W = 100         # [S]
HINGE_GEAR_PD = 200         # [D] Ø20.00 - hinge gear pitch diameter
HINGE_GEAR_M = 8            # [A] module 8 x 25 teeth = Ø200 pitch
HINGE_GEAR_Z = 25
HINGE_CD = HINGE_GEAR_M * HINGE_GEAR_Z   # 200 centre distance (equal gears)

# ------------------------------------------------------------------ hull
HULL_X0, HULL_X1 = 40, 1660
HULL_Y = BODY_WIDTH / 2 - 20             # 630, 20 mm running gap to the track
HULL_Z0 = 150                            # [A] ground clearance under hull
HULL_TOP = 700                           # tub rim (cover parting line)
HULL_R = 120                             # plan corner radius
HULL_WALL = 8                            # [A] rotomoulded PE / PA wall
COVER_TOP = 790
RING_DEPTH = 30                          # interlock ring height (z 670..700)

# cover interlock (offsets measured inward from the outer skin)
GROOVE_OFF = (16, 28)                    # 12 mm wide groove
GROOVE_DEPTH = 18
GASKET_OFF = (17, 27)                    # 10 x 8 EPDM cord
GASKET_H = 8
LIP_OFF = (17.5, 26.5)                   # 9 mm male lip -> 1.5 mm clearance per side
LIP_LEN = 12                             # lip bottom 2 mm into an 8 mm gasket = 25 % squeeze
COVER_WALL = 6

# ------------------------------------------------------------------ tracks
TRACK_Y_IN = BODY_WIDTH / 2              # 650
TRACK_Y_OUT = TOTAL_WIDTH / 2            # 1000
BELT_Y = (665, 985)
SPROCKET = (220, 380, 200)               # [D] Ø40 end wheels  (x, z, r)
IDLER = (1480, 380, 200)
ROAD_WHEELS = [(520, 190, 150), (850, 190, 150), (1180, 190, 150)]   # [S] Ø30 road wheels
SUPPORT_ROLLERS = [(690, 533, 45), (1010, 533, 45)]   # top at 578, just under the belt
BELT_T = 40
LINK_PITCH = 110

# ------------------------------------------------------------------ platform modules
PROFILE_W = 50                           # side profile 50 x 100
FRAME_H = 100                            # deck top -> frame underside
DECK_T = 8
P1_X0 = 440                              # P1 rear end on the hull
P1_X1 = P1_X0 + P1_LEN                   # 1540 = J1 axis a
GEAR_W = 25
GEAR_Y = PLATFORM_WIDTH / 2 + 10         # gear inner face (outboard of side profile)
LINK_Y = GEAR_Y + GEAR_W + 5             # link plate inner face

# railings
POST_R_LOW, POST_R_UP = 21, 17           # Ø42 lower post, Ø34 telescopic upper post
RAIL_R = 21
POST_LOW_LEN = 540                       # lower post (fixed), upper post telescopes out of it
MID_RAIL_Z = 505                         # mid rail centre ~480 above the hinge
RAIL_TOP_LOCAL = POST_LOW_LEN + 20       # top-rail centre in the collapsed state (panel frame)
RAIL_HINGE_Z = (28, 82)                  # staggered fold heights (left, right) -> folded pack <= 106 mm
J1_AXIS_DZ = 10                          # J1 hinge axes 10 mm above deck: stacked deck gap = 200 + 2*10 = 220

# rear stair (5 nested step frames)
REAR_N = 5
REAR_LANDING_X0 = -20

# front stair
FRONT_PIVOT_DZ = 115                     # [A] pivot raised above the deck so the folded stair clears the railings
FRONT_STAIR_ANGLE_DEPLOYED = 41.0        # [A] 1200 stair + 150 stringer depth reaches the ground from 900
FRONT_STAIR_ANGLE_STOWED = -180.0        # folded back, upside-down, on top of P3

# export
EXPORT_UNIT = "cm"                       # user request: model delivered in centimetres
EXPORT_SCALE = 0.1                       # internal mm -> cm
