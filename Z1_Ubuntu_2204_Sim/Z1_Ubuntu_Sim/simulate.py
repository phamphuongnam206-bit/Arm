#!/usr/bin/env python3
"""Z1 V3 articulated CAD demonstration, SI units throughout."""
import argparse
import json
import math
from pathlib import Path
import time

import pybullet as p

ROOT = Path(__file__).resolve().parent
DT = 1 / 240
ARM = [0, 1, 2, 3]
FINGERS = [4, 5]
TOOL = 6
BASE_Z = 0.052
TABLE_TOP = 0.060


def box(half, position, mass, color):
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
    return p.createMultiBody(mass, collision, visual, position)


class Simulation:
    def __init__(self, gui):
        self.gui = gui
        p.connect(p.GUI if gui else p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(DT)
        p.setPhysicsEngineParameter(numSolverIterations=150)
        box([.5, .5, .005], [0, 0, -.005], 0, [.12, .16, .21, 1])
        self.table = box([.028, .028, TABLE_TOP / 2],
                         [-.2, 0, TABLE_TOP / 2], 0, [.4, .45, .5, 1])
        self.robot = p.loadURDF(str(ROOT / 'z1.urdf'), [0, 0, BASE_Z],
                               useFixedBase=True)
        self.cube = box([.0075] * 3, [-.2, 0, TABLE_TOP + .0075],
                        .050, [1, .58, .12, 1])
        p.changeDynamics(self.cube, -1, lateralFriction=.8, restitution=0)
        for link in FINGERS:
            p.changeDynamics(self.robot, link, lateralFriction=.8,
                             spinningFriction=.001, restitution=0)
        self.arm = [0.0] * 4
        self.closure = 0.0
        self.time = 0.0
        self.max_height = TABLE_TOP + .0075
        if gui:
            p.resetDebugVisualizerCamera(.5, 125, -25, [-.10, 0, .10])
            p.addUserDebugText('Robot Z1 V3 | Tam voi 200 mm | Vat 50 g', [-.25, .07, .20],
                              [1, 1, 1], 1.1)
            p.addUserDebugLine([0, -.07, .005], [-.2, -.07, .005],
                              [1, .7, .1], 2)
            p.addUserDebugText('200 mm', [-.13, -.075, .008], [1, .7, .1])

    def step(self):
        for index, target in zip(ARM, self.arm):
            p.setJointMotorControl2(self.robot, index, p.POSITION_CONTROL,
                                   targetPosition=target,
                                   force=[1, 1, 1, .15][index],
                                   maxVelocity=1.0, positionGain=.12)
        for index in FINGERS:
            p.setJointMotorControl2(self.robot, index, p.POSITION_CONTROL,
                                   targetPosition=self.closure, force=5,
                                   maxVelocity=.015, positionGain=.1)
        p.stepSimulation()
        self.time += DT
        self.max_height = max(self.max_height,
                              p.getBasePositionAndOrientation(self.cube)[0][2])
        if self.gui:
            time.sleep(DT)

    def run_for(self, duration):
        for _ in range(round(duration / DT)):
            self.step()

    def move(self, xyz, duration=2.0):
        start = p.getLinkState(self.robot, TOOL, computeForwardKinematics=True)[4]
        for frame in range(1, round(duration / DT) + 1):
            fraction = frame / round(duration / DT)
            weight = fraction * fraction * (3 - 2 * fraction)
            target = [a + weight * (b - a) for a, b in zip(start, xyz)]
            # Hold the wrist's reference orientation to keep both contact pads level.
            solution = p.calculateInverseKinematics(
                self.robot, TOOL, target, [0, 0, 0, 1],
                maxNumIterations=100, residualThreshold=1e-6)
            self.arm = list(solution[:4])
            self.step()

    def demo(self):
        initial = list(p.getBasePositionAndOrientation(self.cube)[0])
        grasp_z = TABLE_TOP + .0075
        self.run_for(.5)
        self.move([-.2, 0, grasp_z + .040])
        self.move([-.2, 0, grasp_z], 2)
        for frame in range(240):
            self.closure = .006 * (frame + 1) / 240
            self.step()
        contacts = {
            str(i): bool(p.getContactPoints(self.robot, self.cube, linkIndexA=i))
            for i in FINGERS
        }
        self.move([-.2, 0, grasp_z + .04], 3)
        held = []
        grip_contacts = []
        lateral_errors = []
        for frame in range(720):
            self.step()
            if frame % 24 == 0:
                position = p.getBasePositionAndOrientation(self.cube)[0]
                held.append(position[2])
                lateral_errors.append(math.hypot(position[0] + .2, position[1]))
                grip_contacts.append(all(
                    p.getContactPoints(self.robot, self.cube, linkIndexA=i)
                    for i in FINGERS))
        minimum_height = min(held)
        result = {
            'mode': 'approximate_contact_physics',
            'payload_kg': .050,
            'target_radius_m': .200,
            'cube_side_m': .015,
            'table_top_m': TABLE_TOP,
            'initial_cube_center_m': initial,
            'bilateral_contact_before_lift': all(contacts.values()),
            'maximum_cube_center_height_m': self.max_height,
            'minimum_cube_center_during_3s_hold_m': minimum_height,
            'bilateral_contact_throughout_hold': all(grip_contacts),
            'maximum_lateral_error_during_hold_m': max(lateral_errors),
            'lift_held_3s': (all(contacts.values()) and all(grip_contacts)
                             and minimum_height > grasp_z + .015
                             and max(lateral_errors) < .015),
            'assumptions': [
                'Link masses and motor torque limits are estimates.',
                'Finger translation and flat pads approximate the CAD linkage.',
                'Self collision is disabled. Collision geometry is simplified.',
                'No adhesive or fixed attachment is used to hold the payload.',
                'Joint limits need calibration against actual servos.'
            ]
        }
        print(json.dumps(result, indent=2))
        (ROOT / 'last_result.json').write_text(json.dumps(result, indent=2))
        if self.gui:
            status = 'GIU VAT THANH CONG (mo hinh gan dung)' if result['lift_held_3s'] else 'NANG VAT THAT BAI'
            p.addUserDebugText(status, [-.25, .07, .18], [1, 1, 1], 1)
        return result

    def manual(self):
        try:
            import tkinter as tk
        except ImportError:
            raise SystemExit(
                "Thieu Tkinter. Ubuntu: sudo apt install python3-tk\n"
                "Neu dung Python Conda: conda install tk"
            )

        for flag in (p.COV_ENABLE_RGB_BUFFER_PREVIEW,
                     p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                     p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW):
            p.configureDebugVisualizer(flag, 0)

        root = tk.Tk()
        root.option_add("*Font", "{Noto Sans} 11")
        root.title("Tay May — Dieu khien robot Z1")
        root.geometry("680x730")
        root.minsize(680, 730)
        bg, card, yellow = "#101827", "#1d2b40", "#ffc857"
        root.configure(bg=bg)

        axes = [0.0] * 4
        grip_direction = [0]
        closed = [False]
        sticks = []
        self.arm = [p.getJointState(self.robot, i)[0] for i in ARM]

        limits = []
        for i in ARM:
            info = p.getJointInfo(self.robot, i)
            lo, hi = info[8], info[9]
            limits.append((lo, hi) if lo < hi else (-math.pi, math.pi))

        finger_limits = [p.getJointInfo(self.robot, i)[8:10]
                         for i in FINGERS]
        grip_low = max([0.0] + [a for a, b in finger_limits if a < b])
        grip_high = min([.008] + [b for a, b in finger_limits if a < b])
        if grip_low > grip_high:
            root.destroy()
            raise ValueError("Gioi han hai ngon kep khong tuong thich.")

        def label(parent, text, size=12, color="white", background=bg):
            return tk.Label(parent, text=text, bg=background, fg=color,
                            font=("Noto Sans", size))

        def button(parent, text, command=None, color="#304762"):
            return tk.Button(
                parent, text=text, command=command, bg=color, fg="white",
                activebackground="#476587", activeforeground="white",
                relief="flat", padx=13, pady=10,
                font=("Noto Sans", 10)
            )

        def camera():
            p.resetDebugVisualizerCamera(.45, 125, -25, [-.10, 0, .10])

        def stop(event=None):
            if closed[0]:
                return
            if p.isConnected():
                self.arm = [p.getJointState(self.robot, i)[0] for i in ARM]
                if grip_direction[0]:
                    actual = sum(p.getJointState(self.robot, i)[0]
                                 for i in FINGERS) / len(FINGERS)
                    self.closure = max(grip_low, min(grip_high, actual))
            axes[:] = [0.0] * 4
            grip_direction[0] = 0
            for canvas, knob in sticks:
                canvas.coords(knob, 120, 85, 160, 125)
            grabbed = root.grab_current()
            if grabbed:
                grabbed.grab_release()

        def release(event):
            if any(axes) or grip_direction[0] or root.grab_current():
                stop()

        def close():
            if not closed[0]:
                stop()
                closed[0] = True
                root.destroy()

        def reset_cube():
            stop()
            p.resetBasePositionAndOrientation(
                self.cube, [-.2, 0, TABLE_TOP + .0075], [0, 0, 0, 1]
            )
            p.resetBaseVelocity(self.cube, [0, 0, 0], [0, 0, 0])

        label(root, "TAY MAY  /  Z1", 23, yellow).pack(pady=(18, 3))
        label(root, "THONG SO TRUC TIEP", 10, "#a4b6cb").pack()

        info_panel = tk.Frame(root, bg=card, padx=20, pady=10)
        info_panel.pack(fill="x", padx=22, pady=12)
        readouts = []
        for row, name in enumerate(
                ["Xoay de", "Khop vai", "Khop khuyu",
                 "Co tay", "Do dong kep"]):
            label(info_panel, name, background=card).grid(
                row=row, column=0, sticky="w", pady=4)
            value = label(info_panel, "—", 13, "#6ee7c7", card)
            value.grid(row=row, column=1, sticky="e")
            readouts.append(value)
        info_panel.columnconfigure(1, weight=1)

        status = label(root, "● Dang giu goc", 11, "#6ee7c7")
        status.pack()
        label(root, "Giu va keo can de chay • Tha chuot de giu goc",
              11, "#c8d4e3").pack(pady=(10, 4))

        joystick_panel = tk.Frame(root, bg=bg)
        joystick_panel.pack()

        def make_stick(column, title, horizontal, vertical, slots):
            frame = tk.Frame(joystick_panel, bg=bg)
            frame.grid(row=0, column=column, padx=15)
            label(frame, title, 11, yellow).pack()
            canvas = tk.Canvas(frame, width=280, height=210,
                               bg=bg, highlightthickness=0)
            canvas.pack()
            canvas.create_oval(70, 35, 210, 175,
                               fill=card, outline="#516780", width=2)
            canvas.create_line(70, 105, 210, 105, fill="#516780")
            canvas.create_line(140, 35, 140, 175, fill="#516780")
            for x, y, text in [
                (140, 15, vertical + " +"),
                (140, 195, vertical + " −"),
                (34, 105, horizontal + " −"),
                (246, 105, horizontal + " +"),
            ]:
                canvas.create_text(x, y, text=text, fill="#c8d4e3",
                                   font=("Noto Sans", 9))
            knob = canvas.create_oval(
                120, 85, 160, 125, fill=yellow, outline="#ffe5a0", width=2)
            sticks.append((canvas, knob))

            def drag(event):
                x, y = (event.x - 140) / 70, (105 - event.y) / 70
                length = math.hypot(x, y)
                if length > 1:
                    x, y = x / length, y / length
                if length < .10:
                    x = y = 0.0
                for slot, value in zip(slots, (x, y)):
                    if axes[slot] and not value:
                        self.arm[slot] = p.getJointState(
                            self.robot, ARM[slot])[0]
                    axes[slot] = value
                cx, cy = 140 + 70*x, 105 - 70*y
                canvas.coords(knob, cx-20, cy-20, cx+20, cy+20)

            def press(event):
                stop()
                canvas.focus_set()
                canvas.grab_set()
                drag(event)

            canvas.bind("<ButtonPress-1>", press)
            canvas.bind("<B1-Motion>", drag)

        make_stick(0, "CAN TRAI", "De", "Vai", (0, 1))
        make_stick(1, "CAN PHAI", "Co tay", "Khuyu", (3, 2))

        speed_row = tk.Frame(root, bg=bg)
        speed_row.pack(fill="x", padx=25)
        label(speed_row, "Toc do (do/giay)", 11).pack(side="left")
        speed = tk.DoubleVar(value=25)
        tk.Scale(
            speed_row, from_=5, to=60, orient="horizontal",
            variable=speed, length=350, bg=bg, fg="white",
            troughcolor="#304762", highlightthickness=0
        ).pack(side="right")

        actions = tk.Frame(root, bg=bg)
        actions.pack(pady=12)

        def start_grip(event, direction):
            stop()
            event.widget.focus_set()
            event.widget.grab_set()
            grip_direction[0] = direction

        for title, direction in [("MO KEP", -1), ("DONG KEP", 1)]:
            b = button(actions, title)
            b.pack(side="left", padx=6)
            b.bind("<ButtonPress-1>",
                   lambda e, d=direction: start_grip(e, d))

        button(actions, "DUNG / GIU GOC", stop, "#a63545").pack(
            side="left", padx=6)

        footer = tk.Frame(root, bg=bg)
        footer.pack()
        button(footer, "Dat lai vat", reset_cube).pack(side="left", padx=6)
        button(footer, "Dat lai goc nhin", camera).pack(side="left", padx=6)
        label(root, "Space: giu goc  •  Esc: thoat", 10, "#a4b6cb").pack(
            pady=12)

        def check_focus():
            if not closed[0] and root.focus_displayof() is None:
                stop()

        root.bind_all("<ButtonRelease-1>", release)
        root.bind("<FocusOut>", lambda e: root.after_idle(check_focus))
        root.bind("<Unmap>",
                  lambda e: stop() if e.widget == root else None)
        root.bind("<space>", stop)
        root.bind("<Escape>", lambda e: close())
        root.protocol("WM_DELETE_WINDOW", close)
        last_display = [0.0]

        def tick():
            if closed[0]:
                return
            if not p.isConnected():
                close()
                return
            try:
                for _ in range(4):
                    rate = math.radians(speed.get())
                    for slot, value in enumerate(axes):
                        if value:
                            actual = p.getJointState(
                                self.robot, ARM[slot])[0]
                            target = self.arm[slot] + value * rate * DT
                            target = max(actual-.06, min(actual+.06, target))
                            lo, hi = limits[slot]
                            self.arm[slot] = max(lo, min(hi, target))

                    if grip_direction[0]:
                        self.closure = max(grip_low, min(
                            grip_high,
                            self.closure + grip_direction[0] * .004 * DT
                        ))
                    self.step()

                if time.monotonic() - last_display[0] > .08:
                    for value_label, index in zip(readouts, ARM):
                        angle = p.getJointState(self.robot, index)[0]
                        value_label.configure(
                            text=f"{math.degrees(angle):+.1f}°")
                    closure = sum(p.getJointState(self.robot, i)[0]
                                  for i in FINGERS) / len(FINGERS)
                    readouts[4].configure(text=f"{closure*1000:.2f} mm")
                    moving = any(axes) or grip_direction[0]
                    status.configure(text="● Dang dieu khien" if moving
                                     else "● Dang giu goc")
                    last_display[0] = time.monotonic()
            except p.error as exc:
                print("Mo phong da dung:", exc)
                close()
                return
            root.after(1, tick)

        camera()
        tick()
        try:
            root.mainloop()
        finally:
            close()




def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--headless', action='store_true',
                        help='Run the automatic trial without opening a window')
    parser.add_argument('--manual', action='store_true',
                        help='Start with manual joint controls')
    args = parser.parse_args()
    if args.headless and args.manual:
        parser.error('--manual requires the graphical window')
    sim = Simulation(gui=not args.headless)
    try:
        if not args.manual:
            sim.demo()
        if not args.headless:
            sim.manual()
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == '__main__':
    main()
