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

        # Capture existing handlers, register them as T0/T1, then override G0/G1.
        self.prev_G0 = self.gcode.register_command("G0", None)
        self.prev_G1 = self.gcode.register_command("G1", None)
        if self.prev_G0 is None or self.prev_G1 is None:
            raise config.error("Unable to override G0/G1 weld control handlers")
        self.gcode.register_command("T0", self.prev_G0, when_not_ready=False)
        self.gcode.register_command("T1", self.prev_G1, when_not_ready=False)
        self.gcode.register_command("G0", self.cmd_G0, when_not_ready=False)
        self.gcode.register_command("G1", self.cmd_G1, when_not_ready=False)
        
        logging.info("Weld control initialized: G0->T0, G1->T1 with weld control")
    
    def _execute_weld_command(self, command_name):
        """Execute Weld_ON or Weld_OFF gcode macro"""
        try:
            self.gcode.run_script(command_name)
        except Exception as e:
            logging.warning("Failed to execute %s: %s", command_name, str(e))
    
    def cmd_G0(self, gcmd):
        """G0 - Rapid Move (no extrusion, weld off)"""
        # Turn weld off for rapid move
        logging.info("Executing G0: turning weld off for rapid move")
        self._execute_weld_command("Weld_OFF")
        self.weld_state = False
        logging.info("Executing G0 with weld control: weld_state=OFF")
        # Forward the original rapid move command handler.
        self.gcode.run_command("T0", gcmd.get_commandline())
    
    def cmd_G1(self, gcmd):
        """G1 - Linear Move (may include extrusion, controls weld)"""
        # Extract parameters
        e = gcmd.get_float('E', default=0.0)

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

        # Forward to original G1 handler while dropping E for weld-only motion.
        params = dict(gcmd.get_command_parameters())
        params.pop('E', None)
        logging.info("Executing G1 with weld control: E=%.3f, weld_state=%s", e, self.weld_state)
        self.gcode.run_command("T1", params)
        

def load_config(config):
    return PrinterWeldControl(config)
