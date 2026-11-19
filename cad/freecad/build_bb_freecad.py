# Ball & Beam - FreeCAD 1.1 assembly with working kinematics.
#
# Run inside FreeCAD (Macro > Execute, or exec(open(path).read()) in the console).
# Printed parts come from cad/asm_stl/*.stl (OpenSCAD, assembled pose, see
# cad/bb_export_asm.scad); bought parts are simple native solids. Geometry is
# read from cad/bb_params.json (generated from simscape/bb_mech_params.m).
#
# Components (App::Part)        Joints (Assembly workbench)
#   Frame   (grounded)            Pivot    Revolute  Frame-Beam  (axis = world y)
#   Beam                          Servo    Revolute  Frame-Horn  (servo shaft)
#   Horn                          LinkBot  Ball      Horn-Pushrod
#   Pushrod                       LinkTop  Ball      Beam-Pushrod (closes the 4-bar)
#   Ball                          Track    Slider    Beam-Ball   (along the rails)
#
# Simulation "Sweep": servo +/-45 deg at 0.25 Hz drives the beam through the
# linkage; the ball slides along the track. Open it from the Assembly tree
# (double-click Sweep, press Generate, then Play).
import json, math, os
import FreeCAD as App
import Part, Mesh
import JointObject, UtilsAssembly
import CommandCreateSimulation as CS

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() else BB_FREECAD_DIR  # noqa: F821
CAD = os.path.dirname(HERE)
P = json.load(open(os.path.join(CAD, "bb_params.json")))
V = App.Vector

# ---- values from bb_design.scad (keep in sync) ----
horn_t, horn_disc_t, spline_h = 3.5, 2.5, 4.0
link_y = P["link_y"]
horn_y = link_y + 6.0
servo_top_y = horn_y + horn_t / 2 + horn_disc_t + spline_h
servo_tip_y = servo_top_y - spline_h
servo_ear_h, servo_ear_t, shaft_off = 29.6, 2.5, 10.0
hub_w = P["tower_gap"] - 2.0
rail_y = P["rail_spacing"] / 2

# ------------------------------------------------------------------ helpers
def stl_solid(name):
    m = Mesh.Mesh(os.path.join(CAD, "asm_stl", name + ".stl"))
    s = Part.Shape()
    s.makeShapeFromMesh(m.Topology, 0.05)
    try:
        sol = Part.Solid(Part.Shell(s.Faces)).removeSplitter()
    except Exception:
        sol = s
    return sol


def add_feature(container, name, shape, color, transparency=0):
    f = container.newObject("Part::Feature", name)
    f.Shape = shape
    if App.GuiUp:
        f.ViewObject.ShapeColor = color
        f.ViewObject.Transparency = transparency
    return f


def cyl(r, h, base, axis):
    return Part.makeCylinder(r, h, base, axis)


def tube(ro, ri, h, base, axis):
    return cyl(ro, h, base, axis).cut(cyl(ri, h, base, axis))


PLA, PLA2, STEEL = (0.25, 0.5, 0.85), (0.15, 0.4, 0.8), (0.78, 0.78, 0.8)
BLACK, WOOD = (0.1, 0.1, 0.1), (0.87, 0.72, 0.53)

# ------------------------------------------------------------------ document
for d in list(App.listDocuments().values()):
    if d.Name == "BallBeam":
        App.closeDocument("BallBeam")
doc = App.newDocument("BallBeam")
asm = doc.addObject("Assembly::AssemblyObject", "Assembly")
jg = asm.newObject("Assembly::JointGroup", "Joints")

# ---- Frame (world frame) ----
frame = asm.newObject("App::Part", "Frame")
add_feature(frame, "BasePlate", stl_solid("base"), WOOD)
add_feature(frame, "TowerLeft", stl_solid("tower_left"), PLA)
add_feature(frame, "TowerRight", stl_solid("tower_right"), PLA)
add_feature(frame, "ServoBracket", stl_solid("servo_bracket"), PLA)
add_feature(frame, "Axle_M5x90", cyl(P["axle_d"] / 2, 90, V(0, -45, P["pivot_h"]), V(0, 1, 0)), STEEL)
L, W, H = P["mg996r_body"]
bx1 = P["servo_x"] + shaft_off
body = Part.makeBox(L, H, W, V(bx1 - L, servo_top_y, P["servo_z"] - W / 2))
ear_y1 = servo_top_y + (H - servo_ear_h) + servo_ear_t
ears = Part.makeBox(P["mg996r_ear_span"], servo_ear_t, W,
                    V(bx1 - L / 2 - P["mg996r_ear_span"] / 2, ear_y1 - servo_ear_t, P["servo_z"] - W / 2))
spline = cyl(2.9, spline_h, V(P["servo_x"], servo_tip_y, P["servo_z"]), V(0, 1, 0))
add_feature(frame, "MG996R", body.fuse(ears).fuse(spline), BLACK)

# ---- Beam (beam frame: origin on the pivot axis) ----
beam = asm.newObject("App::Part", "Beam")
for i in range(3):
    add_feature(beam, "BeamSegment%d" % i, stl_solid("beam_seg%d" % i), PLA2 if i == 1 else PLA)
add_feature(beam, "PivotHub", stl_solid("pivot_hub"), PLA)
add_feature(beam, "LinkBracket", stl_solid("link_bracket"), PLA)
for s in (-1, 1):
    add_feature(beam, "Rail%s" % ("L" if s > 0 else "R"),
                cyl(P["rail_d"] / 2, P["rail_len"], V(-P["rail_len"] / 2, s * rail_y, P["rail_z"]), V(1, 0, 0)), STEEL)
    add_feature(beam, "F625_%s" % ("L" if s > 0 else "R"),
                tube(8, 2.5, 5, V(0, s * (hub_w / 2 - 5) if s > 0 else -hub_w / 2, 0), V(0, 1, 0)), STEEL)
bw, bh = P["hcsr04_board"]
xf, zc = P["sensor_x_face"], P["ball_zc"]
can_h, can_d, pitch = P["hcsr04_can_h"], P["hcsr04_can_d"], P["hcsr04_can_pitch"]
pcb = Part.makeBox(1.6, bw, bh, V(xf - can_h - 1.6, -bw / 2, zc - bh / 2))
for s in (-1, 1):
    pcb = pcb.fuse(cyl(can_d / 2, can_h, V(xf - can_h, s * pitch / 2, zc), V(1, 0, 0)))
add_feature(beam, "HC_SR04", pcb, (0.2, 0.35, 0.8))

# ---- Horn (horn frame: origin on the spline tip, z along the shaft away from the servo) ----
horn = asm.newObject("App::Part", "Horn")
add_feature(horn, "DiscHorn", cyl(12, horn_disc_t, V(0, 0, 0), V(0, 0, 1)), (0.95, 0.95, 0.95))
arm = stl_solid("horn_arm")
arm.translate(V(0, 0, horn_disc_t))
add_feature(horn, "HornArm", arm, (1.0, 0.55, 0.1))
z_link_horn = servo_tip_y - link_y          # ball-link centre, horn frame z

# ---- Pushrod (origin at the lower ball centre, z along the rod) ----
rod_len = P["rod_len"]
pr = asm.newObject("App::Part", "Pushrod")
link = Part.makeSphere(3.2, V(0, 0, 0)).fuse(cyl(4, 8, V(0, 0, 2), V(0, 0, 1)))
top = Part.makeSphere(3.2, V(0, 0, rod_len)).fuse(cyl(4, 8, V(0, 0, rod_len - 10), V(0, 0, 1)))
add_feature(pr, "BallLinks", link.fuse(top), (0.25, 0.25, 0.25))
add_feature(pr, "M3Rod", cyl(1.5, rod_len - 20, V(0, 0, 10), V(0, 0, 1)), STEEL)

# ---- Ball ----
ball = asm.newObject("App::Part", "Ball")
add_feature(ball, "PingPong40", Part.makeSphere(P["ball_d"] / 2), (1.0, 0.5, 0.0))

doc.recompute()

# ------------------------------------------------------------------ initial placements
def rot_x(deg):
    return App.Rotation(V(1, 0, 0), deg)

beam.Placement = App.Placement(V(0, 0, P["pivot_h"]), App.Rotation())
horn.Placement = App.Placement(V(P["servo_x"], servo_tip_y, P["servo_z"]), rot_x(90))
pr.Placement = App.Placement(V(P["servo_x"] - P["horn_r"], link_y, P["servo_z"]), App.Rotation())
s0 = (P["track_s_min"] + P["track_s_max"]) / 2
ball.Placement = App.Placement(V(xf + s0, 0, P["pivot_h"] + zc), App.Rotation())

# ------------------------------------------------------------------ joints
def first_face(container):
    f = container.Group[0]
    return [f.Name + ".Face1", f.Name + ".Vertex1"]


def joint(name, type_index, c1, plc1, c2, plc2):
    j = jg.newObject("App::FeaturePython", name)
    JointObject.Joint(j, type_index)
    # set references directly (setJointConnectors would solve with face-derived frames)
    j.Detach1 = True
    j.Detach2 = True
    j.Reference1 = [c1, first_face(c1)]
    j.Reference2 = [c2, first_face(c2)]
    j.Placement1 = plc1
    j.Placement2 = plc2
    return j

gj = jg.newObject("App::FeaturePython", "GroundedFrame")
JointObject.GroundedJoint(gj, frame)

Y_AXIS = rot_x(-90)          # JCS z -> +y
ALONG_X = App.Rotation(V(0, 1, 0), 90)   # JCS z -> +x
pivot = joint("Pivot", 1, frame, App.Placement(V(0, 0, P["pivot_h"]), Y_AXIS),
              beam, App.Placement(V(0, 0, 0), Y_AXIS))
servo = joint("Servo", 1, frame, App.Placement(V(P["servo_x"], servo_tip_y, P["servo_z"]), rot_x(90)),
              horn, App.Placement(V(0, 0, 0), App.Rotation()))
linkbot = joint("LinkBottom", 4, horn, App.Placement(V(-P["horn_r"], 0, z_link_horn), App.Rotation()),
                pr, App.Placement(V(0, 0, 0), App.Rotation()))
linktop = joint("LinkTop", 4, beam, App.Placement(V(P["link_attach_x"], link_y, P["link_attach_z"]), App.Rotation()),
                pr, App.Placement(V(0, 0, rod_len), App.Rotation()))
track = joint("Track", 3, beam, App.Placement(V(xf, 0, zc), ALONG_X),
              ball, App.Placement(V(0, 0, 0), ALONG_X))
# joint limits (FreeCAD applies them when dragging in the GUI; scripted solves
# and Simulation motions do not enforce them)
track.EnableLengthMin = track.EnableLengthMax = True
track.LengthMin, track.LengthMax = 20.0, P["track_s_max"]   # ball touches sensor face / end stop
servo.EnableAngleMin = servo.EnableAngleMax = True
servo.AngleMin, servo.AngleMax = -67.5, 67.5                 # controller range 1000..2500 us
pivot.EnableAngleMin = pivot.EnableAngleMax = True
pivot.AngleMin, pivot.AngleMax = -20.0, 20.0                 # beam clears the frame to about +/-25 deg
doc.recompute()
res = asm.solve()
print("initial solve:", res)

# ------------------------------------------------------------------ simulation
sg = UtilsAssembly.getSimulationGroup(asm) or asm.newObject("Assembly::SimulationGroup", "Simulations")
sim = sg.newObject("App::FeaturePython", "Sweep")
CS.Simulation(sim)
if App.GuiUp:
    CS.ViewProviderSimulation(sim.ViewObject)
sim.aTimeStart = 0.0
sim.bTimeEnd = 8.0
sim.cTimeStepOutput = 1.0 / 30
sim.jFramesPerSecond = 30
# servo joint z points along -y, so +theta (horn tip up) is a negative joint angle
m1 = asm.newObject("App::FeaturePython", "ServoMotion")
CS.Motion(m1, "Angular", [servo, [""]], "-pi/4*sin(2*pi*0.25*time)")
# steady periodic rolling: s = s_mid + A*sin(wt), A = 0.522*g*sin(alpha_max)/w^2 ~ 190 mm
m2 = asm.newObject("App::FeaturePython", "BallMotion")
CS.Motion(m2, "Linear", [track, [""]], "%.1f + 190*sin(2*pi*0.25*time)" % s0)
if App.GuiUp:
    CS.ViewProviderMotion(m1.ViewObject)
    CS.ViewProviderMotion(m2.ViewObject)
sim.Group = [m1, m2]
doc.recompute()
