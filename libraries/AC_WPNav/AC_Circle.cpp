/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_Circle.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Circle::var_info[] PROGMEM = {
    // @Param: RADIUS
    // @DisplayName: Circle Radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: cm
    // @Range: 0 10000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("RADIUS",  0,  AC_Circle, _radius, AC_CIRCLE_RADIUS_DEFAULT),

    // @Param: RATE
    // @DisplayName: Circle rate
    // @Description: Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
    // @Units: deg/s
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE",    1, AC_Circle, _rate,    AC_CIRCLE_RATE_DEFAULT),

	// @Param: Max angle difference
	// @DisplayName: Max angle difference
	// @Description:
	// @Units: 
	// @Range: 
	// @Increment:
	// @User: Advanced
	AP_GROUPINFO("MAXDIFF", 2, AC_Circle, _max_angle_diff, 0.2f),

	// @Param: Radius Offset Gain
	// @DisplayName: Radius Offset Gain
	// @Description:
	// @Units: 
	// @Range: 
	// @Increment:
	// @User: Advanced
	AP_GROUPINFO("OFS_P", 3, AC_Circle, _radius_offset_p, 0.0f),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Circle::AC_Circle(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _last_update(0),
    _yaw(0.0f),
    _angle(0.0f),
    _angle_total(0.0f),
    _angular_vel(0.0f),
    _angular_vel_max(0.0f),
    _angular_accel(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/// init - initialise circle controller setting center specifically
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Circle::init(const Vector3f& center)
{
    _center = center;

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.init_xy_controller();

	//CHM - extract from later commented functions
	_pos_control.calc_leash_length_xy();
	_pos_control.calc_leash_length_z();
    // set initial position target to reasonable stopping point
    //_pos_control.set_target_to_stopping_point_xy();
    //_pos_control.set_target_to_stopping_point_z();

    // calculate velocities
    calc_velocities();

    // set start angle from position
    init_start_angle(false);

	_max_angle_diff = constrain_float(_max_angle_diff, 0.0f, PI);

	update();
}

/// init - initialise circle controller setting center using stopping point and projecting out based on the copter's heading
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Circle::init()
{
    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.init_xy_controller();

    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    // get stopping point
    const Vector3f& stopping_point = _pos_control.get_pos_target();

    // set circle center to circle_radius ahead of stopping point
    _center.x = stopping_point.x + _radius * _ahrs.cos_yaw();
    _center.y = stopping_point.y + _radius * _ahrs.sin_yaw();
    _center.z = stopping_point.z;

    // calculate velocities
    calc_velocities();

    // set starting angle from vehicle heading
    init_start_angle(true);

	_max_angle_diff = constrain_float(_max_angle_diff, 0.0f, PI);

	update();
}

/// update - update circle controller
void AC_Circle::update()
{
    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _last_update) / 1000.0f;

    // update circle position at 10hz
    if (dt > 0.095f) {

        // double check dt is reasonable
        if (dt >= 1.0f) {
            dt = 0.0;
        }
        // capture time since last iteration
        _last_update = now;

        // ramp up angular velocity to maximum
        if (_rate >= 0) {
			// CHM - _angular_vel_max is actually limited _rate
            if (_angular_vel < _angular_vel_max) {
				// CHM - there should be a fake wind introduced in angular velocity
                //_angular_vel += _angular_accel * dt;
				_angular_vel += _angular_accel * dt * (1 - 0.99f * _angular_vel / _angular_vel_max);
                _angular_vel = constrain_float(_angular_vel, 0, _angular_vel_max);
            }
        }else{
            if (_angular_vel > _angular_vel_max) {
                //_angular_vel += _angular_accel * dt;
				_angular_vel += _angular_accel * dt * (1 - 0.99f * _angular_vel / _angular_vel_max);
                _angular_vel = constrain_float(_angular_vel, _angular_vel_max, 0);
            }
        }

        // update the target angle and total angle traveled
        float angle_change = _angular_vel * dt;
        _angle += angle_change;
        _angle = wrap_PI(_angle);
        _angle_total += angle_change;

        // if the circle_radius is zero we are doing panorama so no need to update loiter target
        if (_radius != 0.0f) {
			const Vector3f &curr_pos = _inav.get_position();
			float uav_angle = wrap_PI(ToRad(90) + fast_atan2(-(curr_pos.x - _center.x), curr_pos.y - _center.y));
			float tangent;
			
			if (_rate >= 0.0f)
				tangent = wrap_PI(uav_angle + PI / 4);
			else
				tangent = wrap_PI(uav_angle - PI / 4);

			float tangential_speed = cosf(tangent)*_inav.get_velocity().x + sinf(tangent)*_inav.get_velocity().y;

			tangential_speed = constrain_float(tangential_speed, 0.0f, _inav.get_velocity_xy());

			float ofs_radius = constrain_float(_radius - tangential_speed * tangential_speed / _radius * _radius_offset_p, 10.0f, _radius);

			static int i = 0;
			if (i >= 10)
			{
				hal.uartC->printf_P(PSTR("Circle TanSpeed:%5.1f m/s  Offset Radius:%5.1f m\n"), tangential_speed / 100.0f, ofs_radius/100.0f);
				i = 0;
			}
			else
				i++;

            // calculate target position
            Vector3f target;
			target.x = _center.x + ofs_radius * cosf(-_angle);
			target.y = _center.y - ofs_radius * sinf(-_angle);
            target.z = _pos_control.get_alt_target();

			
			Vector3f pos_diff = target - curr_pos;

			if (pos_diff.length() > _pos_control.get_leash_xy()*1.5) // make it larger than _leash, so that the pos_diff won't jerk around boundary
			{
				// The position difference is too long, stop updating.
				_angle -= angle_change;
				_angle = wrap_PI(_angle);
				_angle_total -= angle_change;
				return;
			}

			
			// CHM - all in cm

			Vector2f target_on_circle;
			Vector2f inter_unit_vector;

			if (_rate >= 0.0f && (wrap_PI(_angle - uav_angle) > _max_angle_diff)) // 11.48 degree difference // rotating clockwise
			{
				target_on_circle.x = _center.x + cosf(wrap_PI(uav_angle + _max_angle_diff)) * ofs_radius;
				target_on_circle.y = _center.y + sinf(wrap_PI(uav_angle + _max_angle_diff)) * ofs_radius;
				// pos diff hasn't reach max, find intermediate angle for target
				inter_unit_vector.x = target_on_circle.x - curr_pos.x;
				inter_unit_vector.y = target_on_circle.y - curr_pos.y;
				inter_unit_vector.normalize();

				target.x = curr_pos.x + inter_unit_vector.x * pos_diff.length();
				target.y = curr_pos.y + inter_unit_vector.y * pos_diff.length();
				// _radius confirm != 0
				
			}
			else if (_rate < 0.0f && (wrap_PI(_angle - uav_angle) < -_max_angle_diff))
			{
				target_on_circle.x = _center.x + cosf(wrap_PI(uav_angle - _max_angle_diff)) * ofs_radius;
				target_on_circle.y = _center.y + sinf(wrap_PI(uav_angle - _max_angle_diff)) * ofs_radius;

				inter_unit_vector.x = target_on_circle.x - curr_pos.x;
				inter_unit_vector.y = target_on_circle.y - curr_pos.y;
				inter_unit_vector.normalize();

				target.x = curr_pos.x + inter_unit_vector.x * pos_diff.length();
				target.y = curr_pos.y + inter_unit_vector.y * pos_diff.length();
			}

			// update position controller target
			_pos_control.set_pos_target(target);
            

            // heading is 180 deg from vehicles target position around circle
			// CHM - this can be changed to make UAV facing front
            //_yaw = wrap_PI(_angle-PI) * AC_CIRCLE_DEGX100;
			if (_rate>= 0.0f) // clockwise
				_yaw = wrap_PI(_angle + PI/2.0f) * AC_CIRCLE_DEGX100;
			else
				_yaw = wrap_PI(_angle - PI / 2.0f) * AC_CIRCLE_DEGX100;
        }else{
            // set target position to center
            Vector3f target;
            target.x = _center.x;
            target.y = _center.y;
            target.z = _pos_control.get_alt_target();

            // update position controller target
            _pos_control.set_pos_target(target);

            // heading is same as _angle but converted to centi-degrees
            _yaw = _angle * AC_CIRCLE_DEGX100;
        }

        // trigger position controller on next update
        _pos_control.trigger_xy();
    }

    // run loiter's position to velocity step
    _pos_control.update_xy_controller(false);
}

// get_closest_point_on_circle - returns closest point on the circle
//  circle's center should already have been set
//  closest point on the circle will be placed in result
//  result's altitude (i.e. z) will be set to the circle_center's altitude
//  if vehicle is at the center of the circle, the edge directly behind vehicle will be returned
void AC_Circle::get_closest_point_on_circle(Vector3f &result)
{
    // return center if radius is zero
    if (_radius <= 0) {
        result = _center;
        return;
    }

    // get current position
    const Vector3f &curr_pos = _inav.get_position();

    // calc vector from current location to circle center
    Vector2f vec;   // vector from circle center to current location
    vec.x = (curr_pos.x - _center.x);
    vec.y = (curr_pos.y - _center.y);
    float dist = pythagorous2(vec.x, vec.y);

    // if current location is exactly at the center of the circle return edge directly behind vehicle
    if (dist == 0) {
        result.x = _center.x - _radius * _ahrs.cos_yaw();
        result.y = _center.y - _radius * _ahrs.sin_yaw();
        result.z = _center.z;
        return;
    }

    // calculate closest point on edge of circle
    result.x = _center.x + vec.x / dist * _radius;
    result.y = _center.y + vec.y / dist * _radius;
    result.z = _center.z;
}

// calc_velocities - calculate angular velocity max and acceleration based on radius and rate
//      this should be called whenever the radius or rate are changed
//      initialises the yaw and current position around the circle
void AC_Circle::calc_velocities()
{
    // if we are doing a panorama set the circle_angle to the current heading
    if (_radius <= 0) {
        _angular_vel_max = ToRad(_rate);
        _angular_accel = _angular_vel_max;  // reach maximum yaw velocity in 1 second
    }else{
        // set starting angle to current heading - 180 degrees
		// CHM - need to be modified?
		// Looks like this is overriden by init_start_angle()
        _angle = wrap_PI(_ahrs.yaw-PI);

        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        float velocity_max = min(_pos_control.get_speed_xy(), safe_sqrt(0.707f*_pos_control.get_accel_xy()*_radius));

        // angular_velocity in radians per second
        _angular_vel_max = velocity_max/_radius;
		// CHM - _angular_vel_max is eventually defined as limited _rate
        _angular_vel_max = constrain_float(ToRad(_rate),-_angular_vel_max,_angular_vel_max);

        // angular_velocity in radians per second
		// CHM - This is tangential acceleration
		// CHM - we should give room for acceleration towards center
        _angular_accel = _pos_control.get_accel_xy()/_radius;
        if (_rate < 0.0f) {
			// CHM - positive is clockwise, negetive is CCW
            _angular_accel = -_angular_accel;
        }
    }

    // initialise angular velocity
    _angular_vel = 0;
}

// init_start_angle - sets the starting angle around the circle and initialises the angle_total
//  if use_heading is true the vehicle's heading will be used to init the angle causing minimum yaw movement
//  if use_heading is false the vehicle's position from the center will be used to initialise the angle
void AC_Circle::init_start_angle(bool use_heading)
{
    // initialise angle total
	// CHM - this can be set, to give a bit overshoot to make sure the circle is completed
    _angle_total = 0.0f;

    // if the radius is zero we are doing panorama so init angle to the current heading
    if (_radius <= 0.0f) {
        _angle = _ahrs.yaw;
        return;
    }

    // if use_heading is true
    if (use_heading) {
        _angle = wrap_PI(_ahrs.yaw-PI);
    } else {
        // if we are exactly at the center of the circle, init angle to directly behind vehicle (so vehicle will backup but not change heading)
        const Vector3f &curr_pos = _inav.get_position();
        if (curr_pos.x == _center.x && curr_pos.y == _center.y) {
            _angle = wrap_PI(_ahrs.yaw-PI);
        } else {
            // get bearing from circle center to vehicle in radians
			// CHM - starting angle of LOITER_TURNS is defined here
			float bearing_rad = ToRad(90) + fast_atan2(-(curr_pos.x - _center.x), curr_pos.y - _center.y);
			if (_rate>=0)
				_angle = wrap_PI(bearing_rad + 0.15f);
			else
				_angle = wrap_PI(bearing_rad - 0.15f);
			hal.uartC->printf_P(PSTR("circle start bearing: %f dgree\n"), _angle/PI*180.0f);
			hal.console->printf_P(PSTR("circle start bearing: %f dgree\n"), _angle / PI*180.0f);
        }

		// _radius confirm > 0
		const Vector3f &curr_vel = _inav.get_velocity();
		Vector3f tan_unit_vector;
		tan_unit_vector.z = 0.0f;
		if (_rate >= 0.0f) //clockwise
		{
			tan_unit_vector.x = cosf(wrap_PI(_angle + ToRad(90)));
			tan_unit_vector.y = sinf(wrap_PI(_angle + ToRad(90)));
		}
		else
		{
			tan_unit_vector.x = cosf(wrap_PI(_angle - ToRad(90)));
			tan_unit_vector.y = sinf(wrap_PI(_angle - ToRad(90)));
		}
		float angular_tan_vel = (curr_vel * tan_unit_vector) / _radius;
		hal.uartC->printf_P(PSTR("circle starting angluar velocity: %f rad/s\n"),angular_tan_vel);
		_angular_vel = angular_tan_vel;
    }
}
