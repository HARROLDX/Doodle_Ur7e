"""Shared simulation geometry in metres (base frame; wrist-local attachments).

The two-finger gripper and bracket approximate the reference photo, not vendor CAD.
The optical transform retains Lab5's configured mount; it is not a hand-eye calibration.
"""
GROUND_BOX = dict(id='doodle_ground',center=[0.,0.,-.027],size=[3.,3.,.05])
TABLE_TOP = .12
PAPER_SURFACE = TABLE_TOP + .0003
MARKER_SURFACE = TABLE_TOP + .00026
OBSERVATION_WRIST = [0., .5, TABLE_TOP + .395]
TABLE_BOX = dict(id='doodle_table', center=[0., .5, TABLE_TOP-.015], size=[.48,.40,.03])
TABLE_LEGS = [dict(id=f'doodle_table_leg_{i}', center=[x,y,(TABLE_TOP-.03)/2],
                   size=[.035,.035,TABLE_TOP-.03])
              for i,(x,y) in enumerate([(-.21,.33),(.21,.33),(-.21,.67),(.21,.67)])]
# Box enclosures also used by MoveIt; visual detail fits within them.
BRACKET_BOXES = [
    dict(id='doodle_mount_plate',center=[0.,.055,-.009],size=[.065,.14,.012],touch_links=['wrist_3_link','tool0']),
    dict(id='doodle_mount_riser',center=[-.025,.119,-.021],size=[.09,.008,.036],touch_links=['wrist_3_link','tool0']),
]
