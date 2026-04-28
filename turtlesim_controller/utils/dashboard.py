import time

class LivePIDDashboard:
    def __init__(self, enabled=True, update_period=0.2, initial_gains=None,
                 on_gain_change=None, on_reset_pid_memory=None):
        self.enabled = enabled
        self.update_period = update_period
        self.last_update = 0.0
        self.initial_gains = initial_gains or {}
        self.on_gain_change = on_gain_change
        self.on_reset_pid_memory = on_reset_pid_memory
        self.plt = None
        self.fig = None
        self.axes = {}
        self.lines = {}
        self.sliders = {}
        self.buttons = {}

        if not self.enabled:
            return

        try:
            import matplotlib.pyplot as plt
            self.plt = plt
            self.plt.ion()
            self.fig, axs = self.plt.subplots(3, 2, figsize=(13, 9))
            self.fig.canvas.manager.set_window_title(
                "Controller Dashboard"
            )
        except Exception as exc:
            self.enabled = False
            self.plt = None
            self.fig = None
            print(f"Live plots are disabled: {exc}")
            return

        self.axes = {
            "trajectory": axs[0, 0],
            "xy_error": axs[0, 1],
            "tracking_error": axs[1, 0],
            "commands": axs[1, 1],
            "distance_terms": axs[2, 0],
            "heading_terms": axs[2, 1],
        }

        self._configure_axes()
        self._configure_controls()
        self.fig.subplots_adjust(
            left=0.07,
            right=0.98,
            top=0.95,
            bottom=0.24,
            hspace=0.48,
            wspace=0.28,
        )
        self.plt.show(block=False)

    def _configure_axes(self):
        ax = self.axes["trajectory"]
        ax.set_title("Reference vs actual path")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_xlim(0.0, 11.0)
        ax.set_ylim(0.0, 11.0)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True)
        self.lines["desired_path"], = ax.plot([], [], "k--", label="desired")
        self.lines["actual_path"], = ax.plot([], [], "b-", label="actual")
        self.lines["current_desired"], = ax.plot([], [], "ko", label="current desired")
        self.lines["current_actual"], = ax.plot([], [], "bo", label="current turtle")
        ax.legend(loc="upper right")

        ax = self.axes["xy_error"]
        ax.set_title("Position error components")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("error [turtlesim units]")
        ax.grid(True)
        self.lines["error_x"], = ax.plot([], [], label="x error")
        self.lines["error_y"], = ax.plot([], [], label="y error")
        ax.legend(loc="upper right")

        ax = self.axes["tracking_error"]
        ax.set_title("Tracking errors")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("error")
        ax.grid(True)
        self.lines["distance_error"], = ax.plot([], [], label="distance error")
        self.lines["heading_error"], = ax.plot([], [], label="heading error [rad]")
        ax.legend(loc="upper right")

        ax = self.axes["commands"]
        ax.set_title("Controller output commands")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("velocity command")
        ax.grid(True)
        self.lines["linear_cmd"], = ax.plot([], [], label="linear.x")
        self.lines["angular_cmd"], = ax.plot([], [], label="angular.z")
        ax.legend(loc="upper right")

        ax = self.axes["distance_terms"]
        ax.set_title("Distance PID terms")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("linear contribution")
        ax.grid(True)
        self.lines["distance_p"], = ax.plot([], [], label="P")
        self.lines["distance_i"], = ax.plot([], [], label="I")
        self.lines["distance_d"], = ax.plot([], [], label="D")
        self.lines["distance_output"], = ax.plot([], [], "k-", label="limited output")
        ax.legend(loc="upper right")

        ax = self.axes["heading_terms"]
        ax.set_title("Heading PID terms")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("angular contribution")
        ax.grid(True)
        self.lines["heading_p"], = ax.plot([], [], label="P")
        self.lines["heading_i"], = ax.plot([], [], label="I")
        self.lines["heading_d"], = ax.plot([], [], label="D")
        self.lines["heading_output"], = ax.plot([], [], "k-", label="limited output")
        ax.legend(loc="upper right")

    def _configure_controls(self):
        try:
            from matplotlib.widgets import Button, Slider
        except Exception as exc:
            print(f"PID sliders are disabled: {exc}")
            return

        slider_color = "0.92"
        slider_specs = [
            ("distance_kp", "Dist Kp", 0.0, 5.0, 0.01, [0.10, 0.17, 0.34, 0.02]),
            ("heading_kp", "Head Kp", 0.0, 12.0, 0.01, [0.58, 0.17, 0.34, 0.02]),
            ("distance_ki", "Dist Ki", 0.0, 1.0, 0.001, [0.10, 0.13, 0.34, 0.02]),
            ("heading_ki", "Head Ki", 0.0, 2.0, 0.001, [0.58, 0.13, 0.34, 0.02]),
            ("distance_kd", "Dist Kd", 0.0, 2.0, 0.001, [0.10, 0.09, 0.34, 0.02]),
            ("heading_kd", "Head Kd", 0.0, 3.0, 0.001, [0.58, 0.09, 0.34, 0.02]),
        ]

        for key, label, value_min, value_max, step, rect in slider_specs:
            ax = self.fig.add_axes(rect, facecolor=slider_color)
            slider = Slider(
                ax=ax,
                label=label,
                valmin=value_min,
                valmax=value_max,
                valinit=self.initial_gains.get(key, 0.0),
                valstep=step,
            )
            slider.on_changed(self._on_slider_changed)
            self.sliders[key] = slider

        reset_ax = self.fig.add_axes([0.41, 0.025, 0.18, 0.035])
        reset_button = Button(reset_ax, "Reset PID memory")
        reset_button.on_clicked(self._on_reset_clicked)
        self.buttons["reset_pid_memory"] = reset_button

    def _on_slider_changed(self, _value):
        if self.on_gain_change is None:
            return
        self.on_gain_change(self.current_slider_gains())

    def _on_reset_clicked(self, _event):
        if self.on_reset_pid_memory is not None:
            self.on_reset_pid_memory()

    def current_slider_gains(self):
        return {
            key: slider.val
            for key, slider in self.sliders.items()
        }

    def update(self, history, force=False):
        if not self.enabled or not history:
            return

        now = time.monotonic()
        if not force and now - self.last_update < self.update_period:
            return
        self.last_update = now

        t = [row["time"] for row in history]
        desired_x = [row["desired_x"] for row in history]
        desired_y = [row["desired_y"] for row in history]
        actual_x = [row["actual_x"] for row in history]
        actual_y = [row["actual_y"] for row in history]

        self.lines["desired_path"].set_data(desired_x, desired_y)
        self.lines["actual_path"].set_data(actual_x, actual_y)
        self.lines["current_desired"].set_data([desired_x[-1]], [desired_y[-1]])
        self.lines["current_actual"].set_data([actual_x[-1]], [actual_y[-1]])

        self.lines["error_x"].set_data(t, [row["error_x"] for row in history])
        self.lines["error_y"].set_data(t, [row["error_y"] for row in history])
        self.lines["distance_error"].set_data(t, [row["distance_error"] for row in history])
        self.lines["heading_error"].set_data(t, [row["heading_error"] for row in history])
        self.lines["linear_cmd"].set_data(t, [row["linear_cmd"] for row in history])
        self.lines["angular_cmd"].set_data(t, [row["angular_cmd"] for row in history])

        self.lines["distance_p"].set_data(t, [row["distance_p"] for row in history])
        self.lines["distance_i"].set_data(t, [row["distance_i"] for row in history])
        self.lines["distance_d"].set_data(t, [row["distance_d"] for row in history])
        self.lines["distance_output"].set_data(t, [row["distance_output"] for row in history])

        self.lines["heading_p"].set_data(t, [row["heading_p"] for row in history])
        self.lines["heading_i"].set_data(t, [row["heading_i"] for row in history])
        self.lines["heading_d"].set_data(t, [row["heading_d"] for row in history])
        self.lines["heading_output"].set_data(t, [row["heading_output"] for row in history])

        for name, ax in self.axes.items():
            if name == "trajectory":
                continue
            ax.relim()
            ax.autoscale_view()

        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def save_png(self, path, history):
        if not history:
            return

        if self.plt is None:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            self.plt = plt

        if self.fig is None or not self.axes:
            self.enabled = True
            self.fig, axs = self.plt.subplots(3, 2, figsize=(13, 9))
            self.axes = {
                "trajectory": axs[0, 0],
                "xy_error": axs[0, 1],
                "tracking_error": axs[1, 0],
                "commands": axs[1, 1],
                "distance_terms": axs[2, 0],
                "heading_terms": axs[2, 1],
            }
            self._configure_axes()
            self._configure_controls()
            self.fig.subplots_adjust(
                left=0.07,
                right=0.98,
                top=0.95,
                bottom=0.24,
                hspace=0.48,
                wspace=0.28,
            )

        self.update(history, force=True)
        self.fig.savefig(path, dpi=150)