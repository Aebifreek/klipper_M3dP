# Weld control for G0/G1 commands with weld on/off
#
# Copyright (C) 2024  Weld Control System
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

class PrinterWeldControl:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.weld_state = False  # Track current weld state
        
        # Rename existing G0 and G1 to T0 and T1
        self.gcode.register_command("G0", self.cmd_G0, when_not_ready=False)
        self.gcode.register_command("G1", self.cmd_G1, when_not_ready=False)
        self.gcode.rename_command("G0", "T0")
        self.gcode.rename_command("G1", "T1")
        
        logging.info("Weld control initialized: G0->T0, G1->T1 with weld control")
    
    def _execute_weld_command(self, command_name):
        """Execute Weld_ON or Weld_OFF gcode macro"""
        try:
            self.gcode.run_script(command_name)
        except Exception as e:
            logging.warning("Failed to execute %s: %s", command_name, str(e))
    
    def cmd_G0(self, gcmd):
        """G0 - Rapid Move (no extrusion, weld off)"""
        # Extract parameters
        x = gcmd.get_float('X', default=None)
        y = gcmd.get_float('Y', default=None)
        z = gcmd.get_float('Z', default=None)
        f = gcmd.get_float('F', default=None)
        
        # Turn weld off for rapid move
        self._execute_weld_command("Weld_OFF")
        self.weld_state = False
        
        # Build movement command
        res = ""
        if x is not None:
            res += "X%.3f " % x
        if y is not None:
            res += "Y%.3f " % y
        if z is not None:
            res += "Z%.3f " % z
        if f is not None:
            res += "F%.1f " % f
        
        # Execute original T0 (renamed G0) with parameters
        if res:
            self.gcode.run_script("T0 " + res)
        else:
            self.gcode.run_script("T0")
    
    def cmd_G1(self, gcmd):
        """G1 - Linear Move (may include extrusion, controls weld)"""
        # Extract parameters
        x = gcmd.get_float('X', default=None)
        y = gcmd.get_float('Y', default=None)
        z = gcmd.get_float('Z', default=None)
        e = gcmd.get_float('E', default=0.0)
        f = gcmd.get_float('F', default=None)
        
        # Control weld based on extrusion
        if e > 0.0:
            # Turn weld on for extrusion
            if not self.weld_state:
                self._execute_weld_command("Weld_ON")
                self.weld_state = True
        else:
            # Turn weld off when not extruding
            if self.weld_state:
                self._execute_weld_command("Weld_OFF")
                self.weld_state = False
        
        # Build movement command (without E parameter)
        res = ""
        if x is not None:
            res += "X%.3f " % x
        if y is not None:
            res += "Y%.3f " % y
        if z is not None:
            res += "Z%.3f " % z
        if f is not None:
            res += "F%.1f " % f
        
        # Execute original T1 (renamed G1) with parameters
        if res:
            self.gcode.run_script("T1 " + res)
        else:
            self.gcode.run_script("T1")

def load_config(config):
    return PrinterWeldControl(config)
