from ursina import *

app = Ursina()

# Base
base = Entity(model='cube', color=color.gray, scale=(2, 0.2, 2))

# First segment
segment1 = Entity(parent=base, model='cube', color=color.red, scale=(1, 1, 1), position=(1, 0, 0))

# Second segment
segment2 = Entity(parent=segment1, model='cube', color=color.green, scale=(1, 1, 1), position=(1, 0, 0))

# End effector
end_effector = Entity(parent=segment2, model='cube', color=color.blue, scale=(0.5, 0.2, 0.2), position=(0.5, 0, 0))

EditorCamera()

def update():
    segment2.rotation_x += 1


app.run()