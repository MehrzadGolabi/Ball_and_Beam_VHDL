# Replay the Simscape closed-loop run (bb_motion_simscape.csv) on the FreeCAD
# assembly. For every frame the servo horn is set to the Simscape servo angle
# and held (temporary grounded joint); FreeCAD's assembly solver then closes the
# four-bar linkage and positions the beam, so the CAD kinematics are checked
# against Simscape frame by frame. The ball is placed at the simulated distance.
#
# Run in FreeCAD with the BallBeam document open (Macro > Execute). Frames are
# solved first (about 30 s), then played back at 30 fps; call bb_play() to replay.
import csv, math, os, time
import FreeCAD as App
import JointObject

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() else BB_FREECAD_DIR  # noqa: F821
doc = App.getDocument("BallBeam")
asm = doc.Assembly
V = App.Vector
P = __import__("json").load(open(os.path.join(os.path.dirname(HERE), "bb_params.json")))
servo_j = doc.getObject("Servo")
xf, zc = P["sensor_x_face"], P["ball_zc"]


def beam_angle(plc):
    v = plc.Rotation.multVec(V(1, 0, 0))
    return math.degrees(math.atan2(-v.z, v.x))


def solve_frames(rows):
    shaft = servo_j.Placement1                     # servo JCS in the frame (world) coordinates
    horn0 = App.Placement(shaft.Base, App.Rotation(V(1, 0, 0), 90))
    g = doc.Joints.newObject("App::FeaturePython", "ReplayHoldHorn")
    JointObject.GroundedJoint(g, doc.Horn)
    frames, err = [], []
    try:
        for r in rows:
            th = float(r["theta_deg"])
            # +theta = horn tip up = rotation about world +y through the shaft
            doc.Horn.Placement = App.Placement(V(0, 0, 0), App.Rotation(V(0, 1, 0), th), shaft.Base).multiply(horn0)
            s = float(r["s_mm"])
            doc.Ball.Placement = doc.Beam.Placement.multiply(App.Placement(V(xf + s, 0, zc), App.Rotation()))
            asm.solve()
            a = beam_angle(doc.Beam.Placement)
            err.append(a - float(r["alpha_deg"]))
            frames.append([o.Placement for o in (doc.Beam, doc.Horn, doc.Pushrod, doc.Ball)])
    finally:
        doc.removeObject(g.Name)
    return frames, err


rows = list(csv.DictReader(open(os.path.join(HERE, "bb_motion_simscape.csv"))))
t0 = time.time()
BB_FRAMES, err = solve_frames(rows)
print("solved %d frames in %.1f s; beam angle FreeCAD - Simscape: max |err| = %.4f deg, rms = %.4f deg"
      % (len(BB_FRAMES), time.time() - t0, max(abs(e) for e in err), math.sqrt(sum(e * e for e in err) / len(err))))


def bb_play(fps=30):
    if not App.GuiUp:
        return
    from PySide import QtCore
    import FreeCADGui as Gui
    state = {"i": 0}
    timer = QtCore.QTimer()

    def step():
        i = state["i"]
        if i >= len(BB_FRAMES):
            timer.stop()
            return
        for o, p in zip((doc.Beam, doc.Horn, doc.Pushrod, doc.Ball), BB_FRAMES[i]):
            o.Placement = p
        Gui.updateGui()
        state["i"] = i + 1

    timer.timeout.connect(step)
    timer.start(int(1000 / fps))
    globals()["_bb_timer"] = timer          # keep a reference while playing


bb_play()
