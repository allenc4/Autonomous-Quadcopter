/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  ArduPlane parameter definitions

*/

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, { def_value:def } }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, { group_info: class::var_info } }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v,  { group_info: class::var_info } }

const AP_Param::Info var_info[] PROGMEM = {
	GSCALAR(format_version,         "FORMAT_VERSION", 0),
	GSCALAR(software_type,          "SYSID_SW_TYPE",  Parameters::k_software_type),

	GSCALAR(battery_monitoring,     "BATT_MONITOR",   DISABLED),
	GSCALAR(volt_div_ratio,         "VOLT_DIVIDER",   VOLT_DIV_RATIO),
	GSCALAR(curr_amp_per_volt,      "AMP_PER_VOLT",   CURR_AMP_PER_VOLT),
	GSCALAR(input_voltage,          "INPUT_VOLTS",    INPUT_VOLTAGE),
	GSCALAR(pack_capacity,          "BATT_CAPACITY",  HIGH_DISCHARGE),

	// @Group: INS_
	// @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
	GOBJECT(ins,            "INS_", AP_InertialSensor),

	AP_VAREND
};


void Parameters::load_parameters()
{
	if (!g.format_version.load() ||
	     g.format_version != Parameters::k_format_version) {

		// erase all parameters
		hal.console->printf_P(PSTR("Firmware change (%u -> %u): erasing EEPROM...\n"),
						g.format_version.get(), Parameters::k_format_version);
		AP_Param::erase_all();

		// save the current format version
		g.format_version.set_and_save(Parameters::k_format_version);
		hal.console->println_P(PSTR("done."));
    } else {
	    unsigned long before = hal.scheduler->micros();
	    // Load all auto-loaded EEPROM variables
	    AP_Param::load_all();

	    hal.console->printf_P(PSTR("load_all took %luus\n"), hal.scheduler->micros() - before);
	}
}
