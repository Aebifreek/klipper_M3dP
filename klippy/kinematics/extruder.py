# Code for handling printer nozzle extruders
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, threading, time, re
import stepper, chelper

######################################################################
# Asynchronous Raspberry Pi GPIO controller
######################################################################

def _parse_gpio_bcm(pin_desc):
    """Extract the BCM GPIO number from a pin description such as
    'gpio17', 'rpi:gpio17', or '!gpio17'.  Returns None if the
    description is not in gpio<N> format."""
    m = re.match(r'^[!^~]*(?:[^:]+:)?gpio(\d+)$', pin_desc.strip())
    if m:
        return int(m.group(1))
    return None

class AsyncGPIOController:
    """
    Background-thread GPIO manager for Raspberry Pi (BCM) pins.

    The extruder calls schedule(gpio_num, value, print_time) to request
    a pin state change.  The worker thread converts print_time to a
    wall-clock time and fires the GPIO change at approximately that
    moment via RPi.GPIO, independently of the MCU command pipeline.

    Debounce rule: a 1->0 event followed by a 0->1 event on the same
    pin within DEBOUNCE_TIME seconds are BOTH cancelled – the pin keeps
    its previous state and the brief off-glitch is suppressed.

    Minimum toggle interval: any two consecutive state changes on the same
    pin are guaranteed to be at least MIN_TOGGLE_INTERVAL seconds apart.
    If a new event would fire sooner, its wall time is pushed out to
    prev_wall + MIN_TOGGLE_INTERVAL.  This prevents "Timer too close"
    errors on single-core hosts (e.g. Pi Zero) where the worker thread
    and the MCU command pipeline compete for the same CPU.
    """
    DEBOUNCE_TIME = 0.020       # 20 ms – brief LOW-glitch suppression
    MIN_TOGGLE_INTERVAL = 0.050 # 50 ms – minimum gap between any two transitions

    def __init__(self, printer):
        self.printer = printer
        self._mcu = None
        self._GPIO = None
        # Per-pin sorted queue:  gpio_num -> [(print_time, value), ...]
        self._pending = {}
        self._cond = threading.Condition()
        self._running = True
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            self._GPIO = GPIO
        except ImportError:
            logging.warning(
                "AsyncGPIOController: RPi.GPIO not available;"
                " GPIO output will be logged only")
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        logging.info("AsyncGPIOController: init complete, GPIO=%s, worker thread started",
                     "RPi.GPIO" if self._GPIO is not None else "SIMULATED")
        printer.register_event_handler("klippy:connect",
                                       self._handle_connect)
        printer.register_event_handler("klippy:disconnect",
                                       self._handle_disconnect)

    # --- event handlers ---------------------------------------------------

    def _handle_connect(self):
        self._mcu = self.printer.lookup_object('mcu')
        logging.info("AsyncGPIOController: klippy:connect fired, MCU reference acquired")

    def _handle_disconnect(self):
        logging.info("AsyncGPIOController: klippy:disconnect fired, shutting down")
        self.shutdown()

    # --- public interface -------------------------------------------------

    def setup_pin(self, gpio_num):
        """Configure a BCM GPIO pin as a digital output (initial LOW)."""
        logging.info("AsyncGPIOController: setup_pin gpio%d", gpio_num)
        if self._GPIO is not None:
            try:
                self._GPIO.setup(gpio_num, self._GPIO.OUT,
                                 initial=self._GPIO.LOW)
                logging.info("AsyncGPIOController: gpio%d configured as OUTPUT LOW",
                             gpio_num)
            except Exception as e:
                logging.error("AsyncGPIOController: setup_pin gpio%d failed: %s",
                              gpio_num, e)
        with self._cond:
            self._pending.setdefault(gpio_num, [])

    def schedule(self, gpio_num, value, print_time):
        """
        Queue a pin state change to fire at approximately print_time.
        Must be called from the reactor thread (process_move context)
        so that estimated_print_time is safe to use here.
        Returns immediately without blocking.

        Debounce: if the last queued event for this pin set it LOW (0)
        and the new event sets it HIGH (1) within DEBOUNCE_TIME, both
        events are cancelled – suppressing brief LOW glitches.

        Minimum toggle interval: if the new event would fire sooner than
        MIN_TOGGLE_INTERVAL after the previous queued event, its wall
        time is delayed to prev_wall + MIN_TOGGLE_INTERVAL.
        """
        # Convert to wall time here, on the reactor thread, so the
        # worker thread never has to call any MCU/reactor methods.
        wall_time = self._print_time_to_wall_time(print_time)
        logging.debug(
            "AsyncGPIOController: schedule gpio%d=%d pt=%.4f wall=%.4f"
            " (now=%.4f wait=%.3fms)",
            gpio_num, value, print_time, wall_time,
            time.monotonic(), (wall_time - time.monotonic()) * 1000)
        with self._cond:
            pending = self._pending.setdefault(gpio_num, [])
            if pending:
                prev_wall, prev_val = pending[-1]
                if (prev_val == 0 and value == 1
                        and (wall_time - prev_wall) < self.DEBOUNCE_TIME):
                    pending.pop()
                    logging.info(
                        "AsyncGPIOController: debounce cancelled gpio%d"
                        " (1->0->1 within %.0fms)",
                        gpio_num, self.DEBOUNCE_TIME * 1000)
                    return
                # Enforce minimum gap between consecutive transitions so
                # a single-core host (Pi Zero) never services two GPIO
                # edges faster than MIN_TOGGLE_INTERVAL.
                min_wall = prev_wall + self.MIN_TOGGLE_INTERVAL
                if wall_time < min_wall:
                    logging.debug(
                        "AsyncGPIOController: gpio%d=%d delayed +%.0fms"
                        " (MIN_TOGGLE_INTERVAL)",
                        gpio_num, value, (min_wall - wall_time) * 1000)
                    wall_time = min_wall
            pending.append((wall_time, value))
            self._cond.notify()
        logging.debug("AsyncGPIOController: gpio%d=%d queued, pending=%d",
                      gpio_num, value, len(self._pending.get(gpio_num, [])))

    def shutdown(self):
        """Stop the worker thread and release GPIO resources."""
        with self._cond:
            self._running = False
            self._cond.notify_all()
        self._thread.join(timeout=1.0)
        if self._GPIO is not None:
            try:
                self._GPIO.cleanup()
            except Exception:
                pass

    # --- internal helpers -------------------------------------------------

    def _print_time_to_wall_time(self, print_time):
        """Estimate the monotonic wall time for a given print_time."""
        if self._mcu is None:
            logging.debug("AsyncGPIOController: _print_time_to_wall_time"
                          " called before MCU available, using now")
            return time.monotonic()
        try:
            now = time.monotonic()
            est_pt = self._mcu.estimated_print_time(now)
            return now + (print_time - est_pt)
        except Exception as e:
            logging.error("AsyncGPIOController: estimated_print_time failed: %s", e)
            return time.monotonic()

    def _worker(self):
        """Background thread: fires GPIO events at their scheduled times.
        Uses only wall-clock time (time.monotonic) – never calls MCU
        methods, avoiding any reactor lock contention."""
        logging.info("AsyncGPIOController: worker thread started")
        while True:
            gpio_num = None
            val = None
            with self._cond:
                while self._running:
                    # Find the earliest pending event across all pins
                    next_wall = next_gpio = next_val = None
                    for gnum, pending in self._pending.items():
                        if pending:
                            wt, v = pending[0]
                            if next_wall is None or wt < next_wall:
                                next_wall = wt
                                next_gpio = gnum
                                next_val = v
                    if next_wall is None:
                        self._cond.wait(timeout=0.050)
                        continue
                    wait = next_wall - time.monotonic()
                    if wait > 0.001:
                        self._cond.wait(timeout=min(wait, 0.050))
                        continue
                    # Due – dequeue and prepare to execute
                    self._pending[next_gpio].pop(0)
                    gpio_num = next_gpio
                    val = next_val
                    logging.debug(
                        "AsyncGPIOController: firing gpio%d=%d"
                        " (late by %.3fms)",
                        gpio_num, val, -wait * 1000)
                    break
                else:
                    break  # _running became False; exit outer loop
            if gpio_num is None:
                logging.info("AsyncGPIOController: worker thread stopping")
                break  # shutting down
            self._apply(gpio_num, val)

    def _apply(self, gpio_num, value):
        """Write the GPIO state to hardware."""
        if self._GPIO is not None:
            try:
                self._GPIO.output(gpio_num, value)
                logging.debug("AsyncGPIOController: gpio%d -> %d OK",
                              gpio_num, value)
            except Exception as e:
                logging.error(
                    "AsyncGPIOController: error setting gpio%d=%d: %s",
                    gpio_num, value, e)
        else:
            logging.info("AsyncGPIOController: gpio%d -> %d (simulated)",
                         gpio_num, value)

class ExtruderStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.pressure_advance = self.pressure_advance_smooth_time = 0.
        self.config_pa = config.getfloat('pressure_advance', 0., minval=0.)
        self.config_smooth_time = config.getfloat(
                'pressure_advance_smooth_time', 0.040, above=0., maxval=.200)
        # Setup stepper
        self.stepper = stepper.PrinterStepper(config)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.sk_extruder = ffi_main.gc(ffi_lib.extruder_stepper_alloc(),
                                       ffi_lib.extruder_stepper_free)
        self.stepper.set_stepper_kinematics(self.sk_extruder)
        self.motion_queue = None
        # Register commands
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')
        if self.name == 'extruder':
            gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER", None,
                                       self.cmd_default_SET_PRESSURE_ADVANCE,
                                       desc=self.cmd_SET_PRESSURE_ADVANCE_help)
        gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_PRESSURE_ADVANCE,
                                   desc=self.cmd_SET_PRESSURE_ADVANCE_help)
        gcode.register_mux_command("SET_EXTRUDER_ROTATION_DISTANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_E_ROTATION_DISTANCE,
                                   desc=self.cmd_SET_E_ROTATION_DISTANCE_help)
        gcode.register_mux_command("SYNC_EXTRUDER_MOTION", "EXTRUDER",
                                   self.name, self.cmd_SYNC_EXTRUDER_MOTION,
                                   desc=self.cmd_SYNC_EXTRUDER_MOTION_help)
    def _handle_connect(self):
        self._set_pressure_advance(self.config_pa, self.config_smooth_time)
    def get_status(self, eventtime):
        return {'pressure_advance': self.pressure_advance,
                'smooth_time': self.pressure_advance_smooth_time,
                'motion_queue': self.motion_queue}
    def find_past_position(self, print_time):
        mcu_pos = self.stepper.get_past_mcu_position(print_time)
        return self.stepper.mcu_to_commanded_position(mcu_pos)
    def sync_to_extruder(self, extruder_name):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        motion_queuing = self.printer.lookup_object('motion_queuing')
        if not extruder_name:
            self.stepper.set_trapq(None)
            self.motion_queue = None
            motion_queuing.check_step_generation_scan_windows()
            return
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None or not isinstance(extruder, PrinterExtruder):
            raise self.printer.command_error("'%s' is not a valid extruder."
                                             % (extruder_name,))
        self.stepper.set_position([extruder.last_position, 0., 0.])
        self.stepper.set_trapq(extruder.get_trapq())
        self.motion_queue = extruder_name
        motion_queuing.check_step_generation_scan_windows()
    def _set_pressure_advance(self, pressure_advance, smooth_time):
        old_smooth_time = self.pressure_advance_smooth_time
        if not self.pressure_advance:
            old_smooth_time = 0.
        new_smooth_time = smooth_time
        if not pressure_advance:
            new_smooth_time = 0.
        toolhead = self.printer.lookup_object("toolhead")
        ffi_main, ffi_lib = chelper.get_ffi()
        espa = ffi_lib.extruder_set_pressure_advance
        if new_smooth_time != old_smooth_time:
            # Need full kinematic flush to change the smooth time
            toolhead.flush_step_generation()
            espa(self.sk_extruder, 0., pressure_advance, new_smooth_time)
            motion_queuing = self.printer.lookup_object('motion_queuing')
            motion_queuing.check_step_generation_scan_windows()
        else:
            toolhead.register_lookahead_callback(
                lambda print_time: espa(self.sk_extruder, print_time,
                                        pressure_advance, new_smooth_time))
        self.pressure_advance = pressure_advance
        self.pressure_advance_smooth_time = smooth_time
    cmd_SET_PRESSURE_ADVANCE_help = "Set pressure advance parameters"
    def cmd_default_SET_PRESSURE_ADVANCE(self, gcmd):
        extruder = self.printer.lookup_object('toolhead').get_extruder()
        if extruder.extruder_stepper is None:
            raise gcmd.error("Active extruder does not have a stepper")
        strapq = extruder.extruder_stepper.stepper.get_trapq()
        if strapq is not extruder.get_trapq():
            raise gcmd.error("Unable to infer active extruder stepper")
        extruder.extruder_stepper.cmd_SET_PRESSURE_ADVANCE(gcmd)
    def cmd_SET_PRESSURE_ADVANCE(self, gcmd):
        pressure_advance = gcmd.get_float('ADVANCE', self.pressure_advance,
                                          minval=0.)
        smooth_time = gcmd.get_float('SMOOTH_TIME',
                                     self.pressure_advance_smooth_time,
                                     minval=0., maxval=.200)
        self._set_pressure_advance(pressure_advance, smooth_time)
        msg = ("pressure_advance: %.6f\n"
               "pressure_advance_smooth_time: %.6f"
               % (pressure_advance, smooth_time))
        self.printer.set_rollover_info(self.name, "%s: %s" % (self.name, msg))
        gcmd.respond_info(msg, log=False)
    cmd_SET_E_ROTATION_DISTANCE_help = "Set extruder rotation distance"
    def cmd_SET_E_ROTATION_DISTANCE(self, gcmd):
        rotation_dist = gcmd.get_float('DISTANCE', None)
        if rotation_dist is not None:
            if not rotation_dist:
                raise gcmd.error("Rotation distance can not be zero")
            invert_dir, orig_invert_dir = self.stepper.get_dir_inverted()
            next_invert_dir = orig_invert_dir
            if rotation_dist < 0.:
                next_invert_dir = not orig_invert_dir
                rotation_dist = -rotation_dist
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()
            self.stepper.set_rotation_distance(rotation_dist)
            self.stepper.set_dir_inverted(next_invert_dir)
        else:
            rotation_dist, spr = self.stepper.get_rotation_distance()
        invert_dir, orig_invert_dir = self.stepper.get_dir_inverted()
        if invert_dir != orig_invert_dir:
            rotation_dist = -rotation_dist
        gcmd.respond_info("Extruder '%s' rotation distance set to %0.6f"
                          % (self.name, rotation_dist))
    cmd_SYNC_EXTRUDER_MOTION_help = "Set extruder stepper motion queue"
    def cmd_SYNC_EXTRUDER_MOTION(self, gcmd):
        ename = gcmd.get('MOTION_QUEUE')
        self.sync_to_extruder(ename)
        gcmd.respond_info("Extruder '%s' now syncing with '%s'"
                          % (self.name, ename))

# Tracking for hotend heater, extrusion motion queue, and extruder stepper
class PrinterExtruder:
    def __init__(self, config, extruder_num):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.last_position = 0.
        self.can_extrude = True
        # Setup kinematic checks
        self.nozzle_diameter = config.getfloat('nozzle_diameter', above=0.)
        filament_diameter = config.getfloat(
            'filament_diameter', minval=self.nozzle_diameter)
        self.filament_area = math.pi * (filament_diameter * .5)**2
        def_max_cross_section = 4. * self.nozzle_diameter**2
        def_max_extrude_ratio = def_max_cross_section / self.filament_area
        max_cross_section = config.getfloat(
            'max_extrude_cross_section', def_max_cross_section, above=0.)
        self.max_extrude_ratio = max_cross_section / self.filament_area
        logging.info("Extruder max_extrude_ratio=%.6f", self.max_extrude_ratio)
        toolhead = self.printer.lookup_object('toolhead')
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_e_velocity = config.getfloat(
            'max_extrude_only_velocity', max_velocity * def_max_extrude_ratio
            , above=0.)
        self.max_e_accel = config.getfloat(
            'max_extrude_only_accel', max_accel * def_max_extrude_ratio
            , above=0.)
        self.max_e_dist = config.getfloat(
            'max_extrude_only_distance', 50., minval=0.)
        self.instant_corner_v = config.getfloat(
            'instantaneous_corner_velocity', 1., minval=0.)
        # Setup extruder trapq (trapezoidal motion queue)
        self.motion_queuing = self.printer.load_object(config, 'motion_queuing')
        self.trapq = self.motion_queuing.allocate_trapq()
        self.trapq_append = self.motion_queuing.lookup_trapq_append()
        # Optional: Setup digital output pin for extrude motion signaling
        self.motion_pin = None
        self.motion_pin_active = False
        self.motion_pin_off_delay = config.getfloat(
            'extrude_motion_pin_off_delay',
            self.printer.lookup_object('mcu').min_schedule_time(),
            minval=0.)
        # Reactor timer for detecting end-of-extrusion (pin LOW)
        self._reactor = None
        self._pin_low_timer = None
        self._last_extrude_wall_time = 0.
        # How long (wall seconds) of no positive-extrusion process_move calls
        # before the pin is driven LOW via the idle timer.
        self._pin_idle_threshold = config.getfloat(
            'extrude_motion_pin_idle_delay', 0.050, minval=0.005)
        # Async RPi GPIO controller (used when pin is a BCM gpio<N> pin)
        self._async_gpio = None
        self._async_gpio_num = None
        motion_pin_name = config.get('extrude_motion_pin', None)
        if motion_pin_name is not None:
            gpio_num = _parse_gpio_bcm(motion_pin_name)
            logging.info("extruder: extrude_motion_pin='%s' parsed gpio_num=%s",
                         motion_pin_name, gpio_num)
            if gpio_num is not None:
                # BCM GPIO on the host (Raspberry Pi): use the async controller
                logging.info("extruder: using AsyncGPIOController for gpio%d",
                             gpio_num)
                self._async_gpio = AsyncGPIOController(self.printer)
                self._async_gpio.setup_pin(gpio_num)
                self._async_gpio_num = gpio_num
            else:
                # Non-RPi pin: fall back to MCU-scheduled digital output
                logging.info("extruder: using MCU digital_out for '%s'",
                             motion_pin_name)
                ppins = self.printer.lookup_object('pins')
                self.motion_pin = ppins.setup_pin('digital_out',
                                                  motion_pin_name)
                self.motion_pin.setup_max_duration(0.)
                self.motion_pin.setup_start_value(0., 0.)
        # Register connect handler to set up the idle-check reactor timer
        if motion_pin_name is not None:
            self.printer.register_event_handler(
                "klippy:connect", self._handle_motion_pin_connect)
        # Setup extruder stepper
        self.extruder_stepper = None
        if (config.get('step_pin', None) is not None
            or config.get('dir_pin', None) is not None
            or config.get('rotation_distance', None) is not None):
            self.extruder_stepper = ExtruderStepper(config)
            self.extruder_stepper.stepper.set_trapq(self.trapq)
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        if self.name == 'extruder':
            toolhead.set_extruder(self, 0.)
            gcode.register_command("M104", self.cmd_M104)
            gcode.register_command("M109", self.cmd_M109)
        gcode.register_mux_command("ACTIVATE_EXTRUDER", "EXTRUDER",
                                   self.name, self.cmd_ACTIVATE_EXTRUDER,
                                   desc=self.cmd_ACTIVATE_EXTRUDER_help)
    def _handle_motion_pin_connect(self):
        self._reactor = self.printer.get_reactor()
        self._pin_low_timer = self._reactor.register_timer(
            self._handle_pin_low_timer, self._reactor.NEVER)
        logging.info("extruder: motion pin idle timer registered")
    def _handle_pin_low_timer(self, eventtime):
        """Reactor timer: fires when no positive extrusion has been
        processed for _pin_idle_threshold wall seconds.  Sets the pin
        LOW using the toolhead's current scheduled print time."""
        if not self.motion_pin_active:
            return self._reactor.NEVER
        elapsed = eventtime - self._last_extrude_wall_time
        if elapsed < self._pin_idle_threshold:
            # Not yet idle long enough – reschedule
            return self._last_extrude_wall_time + self._pin_idle_threshold
        # Extrusion has been idle – set pin LOW
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        low_time = print_time + 0.010
        logging.debug("extruder: motion pin LOW (idle timer) at pt=%.4f",
                      low_time)
        if self.motion_pin is not None:
            self.motion_pin.set_digital(low_time, 0)
        elif self._async_gpio is not None:
            self._async_gpio.schedule(self._async_gpio_num, 0, low_time)
        self.motion_pin_active = False
        return self._reactor.NEVER
    def get_status(self, eventtime):
        sts = {
            'temperature': 0.,
            'target': 0.,
            'power': 0.,
            'can_extrude': self.can_extrude,
        }
        if self.extruder_stepper is not None:
            sts.update(self.extruder_stepper.get_status(eventtime))
        return sts
    def get_name(self):
        return self.name
    def get_heater(self):
        raise self.printer.command_error("Extruder heater disabled")
    def get_trapq(self):
        return self.trapq
    def get_axis_gcode_id(self):
        return 'E'
    def stats(self, eventtime):
        return False, '%s: target=0 temp=0.0 pwm=0.000' % (self.name,)
    def check_move(self, move, ea_index):
        axis_r = move.axes_r[ea_index]
        axis_d = move.axes_d[ea_index]
        if (not move.axes_d[0] and not move.axes_d[1]) or axis_r < 0.:
            # Extrude only move (or retraction move) - limit accel and velocity
            if abs(axis_d) > self.max_e_dist:
                raise self.printer.command_error(
                    "Extrude only move too long (%.3fmm vs %.3fmm)\n"
                    "See the 'max_extrude_only_distance' config"
                    " option for details" % (axis_d, self.max_e_dist))
            inv_extrude_r = 1. / abs(axis_r)
            move.limit_speed(self.max_e_velocity * inv_extrude_r,
                             self.max_e_accel * inv_extrude_r)
        elif axis_r > self.max_extrude_ratio:
            if axis_d <= self.nozzle_diameter * self.max_extrude_ratio:
                # Permit extrusion if amount extruded is tiny
                return
            area = axis_r * self.filament_area
            logging.debug("Overextrude: %s vs %s (area=%.3f dist=%.3f)",
                          axis_r, self.max_extrude_ratio, area, move.move_d)
            raise self.printer.command_error(
                "Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\n"
                "See the 'max_extrude_cross_section' config option for details"
                % (area, self.max_extrude_ratio * self.filament_area))
    def calc_junction(self, prev_move, move, ea_index):
        diff_r = move.axes_r[ea_index] - prev_move.axes_r[ea_index]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2
    def process_move(self, print_time, move, ea_index):
        axis_r = move.axes_r[ea_index]
        accel = move.accel * axis_r
        start_v = move.start_v * axis_r
        cruise_v = move.cruise_v * axis_r
        can_pressure_advance = False
        if axis_r > 0. and (move.axes_d[0] or move.axes_d[1]):
            can_pressure_advance = True
        # Queue movement (x is extruder movement, y is pressure advance flag)
        self.trapq_append(self.trapq, print_time,
                          move.accel_t, move.cruise_t, move.decel_t,
                          move.start_pos[ea_index], 0., 0.,
                          1., can_pressure_advance, 0.,
                          start_v, cruise_v, accel)
        self.last_position = move.end_pos[ea_index]
        # Handle motion signaling pin (if configured)
        if self._async_gpio is not None or self.motion_pin is not None:
            extrude_d = move.axes_d[ea_index]
            if extrude_d > 0.:
                # Positive extrusion: set pin HIGH at start and keep it there.
                # Do NOT schedule a LOW at the end of each move — consecutive
                # extrusion moves overlap in scheduled time, so HIGH/LOW pairs
                # from adjacent moves would arrive at the MCU out of clock order
                # causing "Timer too close".  The pin stays HIGH until the
                # idle timer detects that no new extrusion has arrived, or until
                # a retract/non-extrusion move explicitly brings it LOW.
                if not self.motion_pin_active:
                    logging.debug("extruder: motion pin HIGH at pt=%.4f",
                                  print_time)
                    if self._async_gpio is not None:
                        self._async_gpio.schedule(
                            self._async_gpio_num, 1, print_time)
                    else:
                        self.motion_pin.set_digital(
                            print_time - self.motion_pin_off_delay, 1)
                    self.motion_pin_active = True
                # Update last-extrude wall time and arm the idle-check timer
                if self._reactor is not None and self._pin_low_timer is not None:
                    now = self._reactor.monotonic()
                    self._last_extrude_wall_time = now
                    self._reactor.update_timer(
                        self._pin_low_timer,
                        now + self._pin_idle_threshold)
            else:
                # Retract or zero extrusion: set pin LOW immediately
                if self.motion_pin_active:
                    logging.debug("extruder: motion pin LOW (retract) at pt=%.4f",
                                  print_time)
                    if self._async_gpio is not None:
                        self._async_gpio.schedule(
                            self._async_gpio_num, 0, print_time)
                    else:
                        self.motion_pin.set_digital(print_time, 0)
                    self.motion_pin_active = False
                    # Disarm the idle timer — pin is already LOW
                    if self._reactor is not None and self._pin_low_timer is not None:
                        self._reactor.update_timer(
                            self._pin_low_timer, self._reactor.NEVER)
    def find_past_position(self, print_time):
        if self.extruder_stepper is None:
            return 0.
        return self.extruder_stepper.find_past_position(print_time)
    def cmd_M104(self, gcmd, wait=False):
        # Heater support is intentionally disabled in this extruder module.
        gcmd.get_float('S', 0.)
        gcmd.get_int('T', None, minval=0)
    def cmd_M109(self, gcmd):
        self.cmd_M104(gcmd, wait=True)
    cmd_ACTIVATE_EXTRUDER_help = "Change the active extruder"
    def cmd_ACTIVATE_EXTRUDER(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is self:
            gcmd.respond_info("Extruder %s already active" % (self.name,))
            return
        gcmd.respond_info("Activating extruder %s" % (self.name,))
        toolhead.flush_step_generation()
        toolhead.set_extruder(self, self.last_position)
        self.printer.send_event("extruder:activate_extruder")

# Dummy extruder class used when a printer has no extruder at all
class DummyExtruder:
    def __init__(self, printer):
        self.printer = printer
    def check_move(self, move, ea_index):
        raise move.move_error("Extrude when no extruder present")
    def find_past_position(self, print_time):
        return 0.
    def calc_junction(self, prev_move, move, ea_index):
        return move.max_cruise_v2
    def get_name(self):
        return ""
    def get_heater(self):
        raise self.printer.command_error("Extruder not configured")
    def get_trapq(self):
        return None
    def get_axis_gcode_id(self):
        return 'E'

def add_printer_objects(config):
    printer = config.get_printer()
    for i in range(99):
        section = 'extruder'
        if i:
            section = 'extruder%d' % (i,)
        if not config.has_section(section):
            break
        pe = PrinterExtruder(config.getsection(section), i)
        printer.add_object(section, pe)
