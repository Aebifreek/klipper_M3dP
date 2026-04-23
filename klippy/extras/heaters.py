# Dummy heater system - all heater functionality disabled
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging

KELVIN_TO_CELSIUS = -273.15


######################################################################
# Dummy Heater
######################################################################

class Heater:
    """Dummy heater that accepts config but does nothing - no temperature sensor"""
    def __init__(self, config, sensor=None):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.short_name = self.name.split()[-1]
        # Consume common heater options so config validation accepts
        # standard heater/extruder sections in dummy mode.
        self.heater_pin = config.get('heater_pin', None)
        self.sensor_type = config.get('sensor_type', None)
        self.sensor_pin = config.get('sensor_pin', None)
        self.sensor_mcu = config.get('sensor_mcu', None)
        self.pullup_resistor = config.get('pullup_resistor', None)
        self.inline_resistor = config.get('inline_resistor', None)
        self.adc_voltage = config.get('adc_voltage', None)
        self.pwm_cycle_time = config.get('pwm_cycle_time', None)
        self.control = config.get('control', None)
        self.pid_kp = config.get('pid_Kp', None)
        self.pid_ki = config.get('pid_Ki', None)
        self.pid_kd = config.get('pid_Kd', None)
        self.max_delta = config.get('max_delta', None)
        # Store minimal config needed by dependents
        self.min_temp = config.getfloat('min_temp', minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.min_extrude_temp = config.getfloat(
            'min_extrude_temp', 170.,
            minval=self.min_temp, maxval=self.max_temp)
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        self.smooth_time = config.getfloat('smooth_time', 1., above=0.)
        # Dummy state - sensor is ignored
        self.target_temp = 0.
        self.last_temp = 0.
        self.smoothed_temp = 0.
        self.last_pwm_value = 0.
        # Always allow extrusion since we have no temperature sensor
        self.can_extrude = True
        logging.info("Dummy heater '%s' initialized (no sensor)", self.name)
    
    def get_name(self):
        return self.name
    
    def get_pwm_delay(self):
        return 0.015  # Dummy value
    
    def get_max_power(self):
        return self.max_power
    
    def get_smooth_time(self):
        return self.smooth_time
    
    def set_temp(self, degrees):
        """Accept temperature setting but do nothing"""
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        self.target_temp = degrees
    
    def get_temp(self, eventtime):
        """Return dummy temperature readings"""
        return self.smoothed_temp, self.target_temp
    
    def set_pwm(self, read_time, value):
        """Accept PWM commands but do nothing"""
        self.last_pwm_value = value
    
    def check_busy(self, eventtime):
        """Always report not busy since we have no sensor to check"""
        return False
    
    def set_control(self, control):
        """Accept control changes but do nothing"""
        return None
    
    def alter_target(self, target_temp):
        """Accept target alterations but do nothing"""
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp
    
    def stats(self, eventtime):
        """Return dummy stats"""
        return False, '%s: target=%.0f temp=%.1f pwm=%.3f' % (
            self.short_name, self.target_temp, self.last_temp, 
            self.last_pwm_value)
    
    def get_status(self, eventtime):
        """Return dummy status"""
        return {'temperature': round(self.smoothed_temp, 2), 
                'target': self.target_temp,
                'power': self.last_pwm_value}
    
    cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"
    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float('TARGET', 0.)
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(self, temp)


######################################################################
# Dummy PrinterHeaters
######################################################################

class PrinterHeaters:
    """Dummy heaters manager - maintains API but no actual heating"""
    def __init__(self, config):
        self.printer = config.get_printer()
        self.heaters = {}
        self.available_heaters = []
        self.available_sensors = []
        self.available_monitors = []
        self.gcode_id_to_sensor = {}
        # Register gcode commands (these will do nothing)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("TURN_OFF_HEATERS", self.cmd_TURN_OFF_HEATERS,
                               desc=self.cmd_TURN_OFF_HEATERS_help)
        gcode.register_command("M105", self.cmd_M105, when_not_ready=True)
        logging.info("Dummy heaters system initialized")
    
    def setup_heater(self, config, gcode_id=None):
        """Create a dummy heater"""
        heater_name = config.get_name().split()[-1]
        if heater_name in self.heaters:
            raise config.error("Heater %s already registered" % (heater_name,))
        # Create dummy heater
        heater = Heater(config, None)
        self.heaters[heater_name] = heater
        self.available_heaters.append(config.get_name())
        # Register gcode ID if provided
        if gcode_id is not None:
            self.gcode_id_to_sensor[gcode_id] = heater
        return heater
    
    def get_all_heaters(self):
        """Return list of available heaters"""
        return self.available_heaters
    
    def lookup_heater(self, heater_name):
        """Look up a heater by name"""
        if heater_name not in self.heaters:
            raise self.printer.config_error(
                "Unknown heater '%s'" % (heater_name,))
        return self.heaters[heater_name]
    
    def set_temperature(self, heater, temp, wait=False):
        """Accept temperature setting but do nothing"""
        heater.set_temp(temp)
        # No waiting needed - no real sensor to check
    
    def turn_off_all_heaters(self, print_time=0.):
        """Turn off all heaters (no-op)"""
        for heater in self.heaters.values():
            heater.set_temp(0.)
    
    def get_status(self, eventtime):
        """Return dummy status"""
        return {'available_heaters': self.available_heaters,
                'available_sensors': self.available_sensors,
                'available_monitors': self.available_monitors}
    
    cmd_TURN_OFF_HEATERS_help = "Turn off all heaters"
    def cmd_TURN_OFF_HEATERS(self, gcmd):
        """Turn off all heaters command"""
        self.turn_off_all_heaters()
    
    def cmd_M105(self, gcmd):
        """M105 - Report temperatures (dummy response)"""
        out = []
        for gcode_id, sensor in sorted(self.gcode_id_to_sensor.items()):
            cur, target = sensor.get_temp(0.)
            out.append("%s:%.1f /%.1f" % (gcode_id, cur, target))
        msg = " ".join(out) if out else "T:0"
        did_ack = gcmd.ack(msg)
        if not did_ack:
            gcmd.respond_raw(msg)


def load_config(config):
    return PrinterHeaters(config)
