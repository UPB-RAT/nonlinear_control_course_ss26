import time


class LiveSMCDashboard:
    def __init__(
        self,
        enabled=True,
        update_period=0.2,
        initial_params=None,
        on_param_change=None,
        on_reset_controller_memory=None,
    ):
        self.enabled = enabled
        self.update_period = update_period
        self.last_update = 0.0
        self.initial_params = initial_params or {}
        self.on_param_change = on_param_change
        self.on_reset_controller_memory = on_reset_controller_memory

        self.plt = None
        self.fig = None
        self.axes = {}
        self.lines = {}
        self.sliders = {}
        self.buttons = {}
        self.interactive_available = False

        if not self.enabled:
            return

        try:
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt

            self.plt = plt
            self.plt.ion()
            self.fig, axs = self.plt.subplots(3, 2, figsize=(13, 9))
            try:
                self.fig.canvas.manager.set_window_title("SMC Live Dashboard")
            except Exception:
                pass
            self.interactive_available = True
        except Exception as exc:
            print(f"[LiveSMCDashboard] Interactive dashboard disabled: {exc}")
            self.enabled = False
            self.interactive_available = False
            return

        self.axes = {
            "trajectory": axs[0, 0],
            "pos_error": axs[0, 1],
            "sliding_surfaces": axs[1, 0],
            "attitude": axs[1, 1],
            "control_outputs": axs[2, 0],
            "motors": axs[2, 1],
        }

        self._configure_axes()
        self._configure_controls()

        self.fig.subplots_adjust(
            left=0.06, right=0.98, top=0.95, bottom=0.28, hspace=0.42, wspace=0.28
        )
        self.plt.show(block=False)

    def _configure_axes(self):
        ax = self.axes["trajectory"]
        ax.set_title("XY Trajectory")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(True)
        ax.set_aspect("equal", adjustable="box")
        self.lines["desired_path"], = ax.plot([], [], "k--", label="desired")
        self.lines["actual_path"], = ax.plot([], [], "b-", label="actual")
        self.lines["cur_des"], = ax.plot([], [], "ko")
        self.lines["cur_act"], = ax.plot([], [], "bo")
        ax.legend(loc="upper right")

        ax = self.axes["pos_error"]
        ax.set_title("Position errors")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("error [m]")
        ax.grid(True)
        self.lines["ex"], = ax.plot([], [], label="ex")
        self.lines["ey"], = ax.plot([], [], label="ey")
        self.lines["ez"], = ax.plot([], [], label="ez")
        self.lines["err"], = ax.plot([], [], "k-", label="||e||")
        ax.legend(loc="upper right")

        ax = self.axes["sliding_surfaces"]
        ax.set_title("Sliding surfaces")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("s")
        ax.grid(True)
        self.lines["sx"], = ax.plot([], [], label="sx")
        self.lines["sy"], = ax.plot([], [], label="sy")
        self.lines["sz"], = ax.plot([], [], label="sz")
        ax.legend(loc="upper right")

        ax = self.axes["attitude"]
        ax.set_title("Attitude tracking [deg]")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("deg")
        ax.grid(True)
        self.lines["roll"], = ax.plot([], [], label="roll")
        self.lines["pitch"], = ax.plot([], [], label="pitch")
        self.lines["roll_des"], = ax.plot([], [], "--", label="roll_des")
        self.lines["pitch_des"], = ax.plot([], [], "--", label="pitch_des")
        ax.legend(loc="upper right")

        ax = self.axes["control_outputs"]
        ax.set_title("SMC outputs")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("command")
        ax.grid(True)
        self.lines["u_roll"], = ax.plot([], [], label="u_roll")
        self.lines["u_pitch"], = ax.plot([], [], label="u_pitch")
        self.lines["u_z"], = ax.plot([], [], label="u_z")
        self.lines["u_yaw"], = ax.plot([], [], label="u_yaw")
        ax.legend(loc="upper right")

        ax = self.axes["motors"]
        ax.set_title("Motor speeds")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("rad/s cmd")
        ax.grid(True)
        self.lines["m0"], = ax.plot([], [], label="m0")
        self.lines["m1"], = ax.plot([], [], label="m1")
        self.lines["m2"], = ax.plot([], [], label="m2")
        self.lines["m3"], = ax.plot([], [], label="m3")
        ax.legend(loc="upper right")

    def _configure_controls(self):
        try:
            from matplotlib.widgets import Slider, Button
        except Exception as exc:
            print(f"[LiveSMCDashboard] Sliders disabled: {exc}")
            return

        slider_specs = [
            ("k_x", "k_x", 0.0, 15.0, 0.1, [0.07, 0.20, 0.24, 0.02]),
            ("k_y", "k_y", 0.0, 15.0, 0.1, [0.37, 0.20, 0.24, 0.02]),
            ("k_z", "k_z", 0.0, 120.0, 1.0, [0.67, 0.20, 0.24, 0.02]),

            ("k_roll", "k_roll", 0.0, 40.0, 0.1, [0.07, 0.16, 0.24, 0.02]),
            ("k_pitch", "k_pitch", 0.0, 40.0, 0.1, [0.37, 0.16, 0.24, 0.02]),
            ("k_yaw", "k_yaw", 0.0, 20.0, 0.1, [0.67, 0.16, 0.24, 0.02]),

            ("lambda_x", "lambda_x", 0.0, 4.0, 0.05, [0.07, 0.12, 0.24, 0.02]),
            ("lambda_y", "lambda_y", 0.0, 4.0, 0.05, [0.37, 0.12, 0.24, 0.02]),
            ("lambda_z", "lambda_z", 0.0, 4.0, 0.05, [0.67, 0.12, 0.24, 0.02]),
        ]

        for key, label, vmin, vmax, step, rect in slider_specs:
            ax = self.fig.add_axes(rect, facecolor="0.92")
            slider = Slider(
                ax=ax,
                label=label,
                valmin=vmin,
                valmax=vmax,
                valinit=self.initial_params.get(key, 0.0),
                valstep=step,
            )
            slider.on_changed(self._on_slider_changed)
            self.sliders[key] = slider

        reset_ax = self.fig.add_axes([0.40, 0.04, 0.20, 0.04])
        btn = Button(reset_ax, "Reset SMC memory")
        btn.on_clicked(self._on_reset_clicked)
        self.buttons["reset"] = btn

    def _on_slider_changed(self, _):
        if self.on_param_change is not None:
            self.on_param_change(self.current_slider_params())

    def _on_reset_clicked(self, _):
        if self.on_reset_controller_memory is not None:
            self.on_reset_controller_memory()

    def current_slider_params(self):
        return {k: s.val for k, s in self.sliders.items()}

    def update(self, history, force=False):
        if not self.enabled or not history or not self.interactive_available:
            return

        now = time.monotonic()
        if not force and (now - self.last_update) < self.update_period:
            return
        self.last_update = now

        t = [h["time"] for h in history]
        tx = [h["tx"] for h in history]
        ty = [h["ty"] for h in history]
        x = [h["x"] for h in history]
        y = [h["y"] for h in history]

        self.lines["desired_path"].set_data(tx, ty)
        self.lines["actual_path"].set_data(x, y)
        self.lines["cur_des"].set_data([tx[-1]], [ty[-1]])
        self.lines["cur_act"].set_data([x[-1]], [y[-1]])

        self.lines["ex"].set_data(t, [h["ex"] for h in history])
        self.lines["ey"].set_data(t, [h["ey"] for h in history])
        self.lines["ez"].set_data(t, [h["ez"] for h in history])
        self.lines["err"].set_data(t, [h["err"] for h in history])

        self.lines["sx"].set_data(t, [h["sx"] for h in history])
        self.lines["sy"].set_data(t, [h["sy"] for h in history])
        self.lines["sz"].set_data(t, [h["sz"] for h in history])

        self.lines["roll"].set_data(t, [h["roll"] for h in history])
        self.lines["pitch"].set_data(t, [h["pitch"] for h in history])
        self.lines["roll_des"].set_data(t, [h["roll_des"] for h in history])
        self.lines["pitch_des"].set_data(t, [h["pitch_des"] for h in history])

        self.lines["u_roll"].set_data(t, [h["u_roll"] for h in history])
        self.lines["u_pitch"].set_data(t, [h["u_pitch"] for h in history])
        self.lines["u_z"].set_data(t, [h["u_z"] for h in history])
        self.lines["u_yaw"].set_data(t, [h["u_yaw"] for h in history])

        self.lines["m0"].set_data(t, [h["m0"] for h in history])
        self.lines["m1"].set_data(t, [h["m1"] for h in history])
        self.lines["m2"].set_data(t, [h["m2"] for h in history])
        self.lines["m3"].set_data(t, [h["m3"] for h in history])

        for name, ax in self.axes.items():
            if name == "trajectory":
                continue
            ax.relim()
            ax.autoscale_view()

        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)