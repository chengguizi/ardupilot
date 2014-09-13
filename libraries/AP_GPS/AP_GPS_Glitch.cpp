// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-



#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_Notify.h>
#include "AP_GPS_Glitch.h"

#include <GCS.h> //added
// CHM - debugging
#include "../ArduCopter/Debug_CHM.h"
extern debug_s mydebug;
void debug_send_message(enum ap_message id);

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo GPS_Glitch::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: GPS Glitch protection enable/disable
    // @Description: Allows you to enable (1) or disable (0) gps glitch protection
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",      0,  GPS_Glitch,   _enabled,   1),

    // @Param: RADIUS
    // @DisplayName: GPS glitch protection radius within which all new positions are accepted
    // @Description: GPS glitch protection radius within which all new positions are accepted
    // @Units: cm
    // @Range: 100 2000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("RADIUS",      1,  GPS_Glitch,   _radius_cm,   GPS_GLITCH_RADIUS_CM),

    // @Param: ACCEL
    // @DisplayName: GPS glitch protection's max vehicle acceleration assumption
    // @Description: GPS glitch protection's max vehicle acceleration assumption
    // @Units: cm/s/s
    // @Range: 100 2000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("ACCEL",   2,  GPS_Glitch,   _accel_max_cmss,   GPS_GLITCH_ACCEL_MAX_CMSS),

    AP_GROUPEND
};

// constuctor
GPS_Glitch::GPS_Glitch(const AP_GPS &gps) :
    _gps(gps),
    _last_good_update(0),
    _last_good_lat(0),
    _last_good_lon(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// check_position - returns true if gps position is acceptable, false if not
void GPS_Glitch::check_position()
{
    uint32_t now = hal.scheduler->millis(); // current system time
    float sane_dt;                  // time since last sane gps reading
    float accel_based_distance;     // movement based on max acceleration
    Location curr_pos;              // our current position estimate
    Location gps_pos;               // gps reported position
    float distance_cm;              // distance from gps to current position estimate in cm
    bool all_ok;                    // true if the new gps position passes sanity checks

    // exit immediately if we don't have gps lock
    if (_gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        _flags.glitching = true;
        return;
    }

    // if not initialised or disabled update last good position and exit
    if (!_flags.initialised || !_enabled) {
		// CHM .location etc is updated in gps.update()
        const Location &loc = _gps.location();
        const Vector3f &vel = _gps.velocity();
        _last_good_update = now;
        _last_good_lat = loc.lat;
        _last_good_lon = loc.lng;
		//CHM - add altitude, in centimeters
		_last_good_alt = loc.alt;
        _last_good_vel.x = vel.x;
        _last_good_vel.y = vel.y;
		//CHM - add vertical velocity
		_last_good_vel.z = vel.z;
        _flags.initialised = true;
        _flags.glitching = false;
        return;
    }

    // calculate time since last sane gps reading in ms
    sane_dt = (now - _last_good_update) / 1000.0f;

    // project forward our position from last known velocity
    curr_pos.lat = _last_good_lat;
    curr_pos.lng = _last_good_lon;
	// CHM - add altitude verification
	curr_pos.alt = _last_good_alt;

	if (_gps.have_vertical_velocity())
		location_offset(curr_pos, _last_good_vel.x * sane_dt, _last_good_vel.y * sane_dt, _last_good_vel.z * 100.0f * sane_dt);
	else
		location_offset(curr_pos, _last_good_vel.x * sane_dt, _last_good_vel.y * sane_dt);

    // calculate distance from recent gps position to current estimate
    const Location &loc = _gps.location();
    gps_pos.lat = loc.lat;
    gps_pos.lng = loc.lng;
	gps_pos.alt = loc.alt; //add
	// CHM - this distance is in xy plane only
    distance_cm = get_distance_cm(curr_pos, gps_pos);

    // all ok if within a given hardcoded radius
	
	///////////////////////////
	//////DEBUG Message////////
	
	/*
	strcpy(mydebug.name,"Gl_cm_x");
	mydebug.valuef = locdiff.x*100.0;
	debug_send_message(MSG_NAMED_VALUE_FLOAT);

	strcpy(mydebug.name, "Gl_cm_y");
	mydebug.valuef = locdiff.y*100.0;
	debug_send_message(MSG_NAMED_VALUE_FLOAT);

	strcpy(mydebug.name, "Gl_cm");
	mydebug.valuef = distance_cm;
	debug_send_message(MSG_NAMED_VALUE_FLOAT);
	///////////////////////////
	*/
	if (distance_cm > 90)
	{
		Log_Write_Data(DATA_GPS_LOCATION_DIFFERENCE_CM, (float)distance_cm);
		Vector2f locdiff = location_diff(curr_pos, gps_pos);
		Log_Write_Data(DATA_GPS_LOCATION_DIFFERENCE_X, (float)locdiff.x*100.0f);
		Log_Write_Data(DATA_GPS_LOCATION_DIFFERENCE_Y, (float)locdiff.y*100.0f);
	}
	

	if (distance_cm <= _radius_cm) {
        all_ok = true;
    }else{
		hal.uartC->printf_P(PSTR("GPS difference_xy:%.0f !!!\n"), distance_cm);
        // or if within the maximum distance we could have moved based on our acceleration
        accel_based_distance = 0.5f * _accel_max_cmss * sane_dt * sane_dt;
        all_ok = (distance_cm <= accel_based_distance);
		/*//////////////////////////
		//////DEBUG Message////////
		strcpy(mydebug.name, "Glitch_R");
		mydebug.valuef = accel_based_distance;
		debug_send_message(MSG_NAMED_VALUE_FLOAT);
		//////////////////////////*/
		Log_Write_Data(DATA_GPS_GROW_RADIUS, accel_based_distance);
    }

	//CHM - check for altitude change
	if (all_ok && _gps.have_vertical_velocity())
	{
		distance_cm = gps_pos.alt - curr_pos.alt;

		/*//////////////////////////
		//////DEBUG Message////////
		strcpy(mydebug.name, "Gl_alt_cm");
		mydebug.valuef = gps_pos.alt - curr_pos.alt;
		debug_send_message(MSG_NAMED_VALUE_FLOAT);
		//////////////////////////*/
		if (distance_cm > 90.0f || distance_cm < -100.0f)
			Log_Write_Data(DATA_GPS_ALTITUDE_DIFFERENCE, (float)(gps_pos.alt - curr_pos.alt));

		if ( (float)fabs(distance_cm) > _radius_cm ) {
			hal.uartC->printf_P(PSTR("GPS difference_z:%.0f !!!\n"), distance_cm);
			// or if within the maximum distance we could have moved based on our acceleration
			if (distance_cm > 0.0f)
			{
				accel_based_distance = 0.5f * _accel_max_cmss * 1.5f * sane_dt * sane_dt;
				all_ok = distance_cm <= accel_based_distance;
			}
			else
			{
				accel_based_distance = -0.5f * 1200.0f * sane_dt * sane_dt;
				all_ok = distance_cm >= accel_based_distance;
			}
			

			
			/*//////////////////////////
			//////DEBUG Message////////
			strcpy(mydebug.name, "Gl_alt_R");
			mydebug.valuef = accel_based_distance;
			debug_send_message(MSG_NAMED_VALUE_FLOAT);
			//////////////////////////*/
			Log_Write_Data(DATA_GPS_GROW_ALT, accel_based_distance);
		}

	}

	// CHM - code added
	// once failsafe occured, wait until reasonable hdop and satellite number is obtained
	// perhaps the altitude info must be reasonable, based on barometer
	if (_flags.glitching)
	{
		if (_gps.get_hdop() > 200 || _gps.num_sats() <= 8 /*|| abs(baro_alt - _gps.location().alt)*/)
			all_ok = false;
	}
		

    // store updates to gps position
    if (all_ok) {
        // position is acceptable
        _last_good_update = now;
        _last_good_lat = loc.lat;
        _last_good_lon = loc.lng;
		_last_good_alt = loc.alt; // CHM - code added
        const Vector3f &vel = _gps.velocity();
        _last_good_vel.x = vel.x;
        _last_good_vel.y = vel.y;
		_last_good_vel.z = vel.z; // CHM - code added
    }
    
    // update glitching flag
    _flags.glitching = !all_ok;
	if (_flags.glitching)
		hal.uartC->printf_P(PSTR("GPS GLITCH !!!\n"));
}

