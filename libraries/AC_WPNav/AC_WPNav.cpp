/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_WPNav.h>

/////////
#include <GCS.h> //added
// CHM - debugging
#include "../ArduCopter/Debug_CHM.h"
extern debug_s mydebug;
void debug_send_message(enum ap_message id);
////////

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_WPNav::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",       0, AC_WPNav, _wp_speed_cms, WPNAV_WP_SPEED),

    // @Param: RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: cm
    // @Range: 100 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS",      1, AC_WPNav, _wp_radius_cm, WPNAV_WP_RADIUS),

    // @Param: SPEED_UP
    // @DisplayName: Waypoint Climb Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
    // @Units: cm/s
    // @Range: 0 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_UP",    2, AC_WPNav, _wp_speed_up_cms, WPNAV_WP_SPEED_UP),

    // @Param: SPEED_DN
    // @DisplayName: Waypoint Descent Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
    // @Units: cm/s
    // @Range: 0 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_DN",    3, AC_WPNav, _wp_speed_down_cms, WPNAV_WP_SPEED_DOWN),

    // @Param: LOIT_SPEED
    // @DisplayName: Loiter Horizontal Maximum Speed
    // @Description: Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("LOIT_SPEED",  4, AC_WPNav, _loiter_speed_cms, WPNAV_LOITER_SPEED),

    // @Param: ACCEL
    // @DisplayName: Waypoint Acceleration 
    // @Description: Defines the horizontal acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL",       5, AC_WPNav, _wp_accel_cms, WPNAV_ACCELERATION),

    // @Param: ACCEL_Z
    // @DisplayName: Waypoint Vertical Acceleration
    // @Description: Defines the vertical acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",     6, AC_WPNav, _wp_accel_z_cms, WPNAV_WP_ACCEL_Z_DEFAULT),

    // @Param: LOIT_JERK
    // @DisplayName: Loiter maximum jerk
    // @Description: Loiter maximum jerk in cm/s/s/s
    // @Units: cm/s/s/s
    // @Range: 500 2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LOIT_JERK",   7, AC_WPNav, _loiter_jerk_max_cmsss, WPNAV_LOITER_JERK_MAX_DEFAULT),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_WPNav::AC_WPNav(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _loiter_last_update(0),
    _loiter_step(0),
    _pilot_accel_fwd_cms(0),
    _pilot_accel_rgt_cms(0),
    _loiter_accel_cms(WPNAV_LOITER_ACCEL),
    _wp_last_update(0),
    _wp_step(0),
    _track_length(0.0f),
    _track_desired(0.0f),
    _limited_speed_xy_cms(0.0f),
    _track_accel(0.0f),
    _track_speed(0.0f),
    _track_leash_length(0.0f),
    _slow_down_dist(0.0f),
    _spline_time(0.0f),
    _spline_time_scale(0.0f),
    _spline_vel_scaler(0.0f),
    _yaw(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);
}

///
/// loiter controller
///

/// init_loiter_target in cm from home
void AC_WPNav::init_loiter_target(const Vector3f& position, bool reset_I)
{
    // if reset_I is false we warn position controller not to reset I terms
    if (!reset_I) {
        _pos_control.keep_xy_I_terms();
    }
    
    // initialise position controller
    _pos_control.init_xy_controller();

    // initialise pos controller speed and acceleration
	// CHM - make _speed_cms = _loiter_speed_cms = LOIT_SPEED
	// This initialisation may enable the cabability of having different speed for each target??
	// But in this case, it is always LOIT_SPEED
    _pos_control.set_speed_xy(_loiter_speed_cms);
    _loiter_accel_cms = _loiter_speed_cms/2.0f;
    _pos_control.set_accel_xy(_loiter_accel_cms);

    // set target position
    _pos_control.set_xy_target(position.x, position.y);

    // initialise feed forward velocity to zero
    _pos_control.set_desired_velocity_xy(0,0);

    // initialise desired accel and add fake wind
    _loiter_desired_accel.x = 0;
    _loiter_desired_accel.y = 0;

    // initialise pilot input
    _pilot_accel_fwd_cms = 0;
    _pilot_accel_rgt_cms = 0;
}

/// init_loiter_target - initialize's loiter position and feed-forward velocity from current pos and velocity
// CHM - this is called when motors are armed, and throttle is above 0; OR inertial position is bad
void AC_WPNav::init_loiter_target()
{
    const Vector3f& curr_pos = _inav.get_position();
    const Vector3f& curr_vel = _inav.get_velocity();

    // initialise position controller
    _pos_control.init_xy_controller();

    // initialise pos controller speed and acceleration
	// CHM - this is to set default max speed and max acceleration
    _pos_control.set_speed_xy(_loiter_speed_cms);
    _loiter_accel_cms = _loiter_speed_cms/2.0f;
    _pos_control.set_accel_xy(_loiter_accel_cms);

    // set target position
    _pos_control.set_xy_target(curr_pos.x, curr_pos.y);

    // move current vehicle velocity into feed forward velocity
    _pos_control.set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // initialise desired accel and add fake wind
	// CHM - this is to say, the acceleration increase with current velocity, probably to COMPENSATE the wind resistance
	// but why this is needed in initialisation????
	// one way to explain it that , when pilot is commanding the uav to move, and in the meanwhile he change mode to loiter.
	// in this situation, the jerk feeling will be less
    _loiter_desired_accel.x = (_loiter_accel_cms)*curr_vel.x/_loiter_speed_cms;
    _loiter_desired_accel.y = (_loiter_accel_cms)*curr_vel.y/_loiter_speed_cms;

    // initialise pilot input
    _pilot_accel_fwd_cms = 0;
    _pilot_accel_rgt_cms = 0;
}

/// set_loiter_velocity - allows main code to pass the maximum velocity for loiter
void AC_WPNav::set_loiter_velocity(float velocity_cms)
{
    // range check velocity and update position controller
	// CHM - here is the place to change the _loiter_speed_cms, MAX loiter speed value
    if (velocity_cms >= WPNAV_LOITER_SPEED_MIN) {
        _loiter_speed_cms = velocity_cms;

        // initialise pos controller speed
        _pos_control.set_speed_xy(_loiter_speed_cms);

        // initialise pos controller acceleration
        _loiter_accel_cms = _loiter_speed_cms/2.0f;
        _pos_control.set_accel_xy(_loiter_accel_cms);
    }
}

/// set_pilot_desired_acceleration - sets pilot desired acceleration from roll and pitch stick input
void AC_WPNav::set_pilot_desired_acceleration(float control_roll, float control_pitch)
{
	// CHM - added code, ensure when UAV travel diagonally, the acceleration is not too fast
	// To be verified
	float net_accel_chm = pythagorous2(control_pitch, control_roll);
	if (net_accel_chm > 4500.0f)
	{
		control_pitch = 4500.0f * control_pitch / net_accel_chm;
		control_roll = 4500.0f * control_roll / net_accel_chm;
	}
    // convert pilot input to desired acceleration in cm/s/s
    _pilot_accel_fwd_cms = -control_pitch * _loiter_accel_cms / 4500.0f;
    _pilot_accel_rgt_cms = control_roll * _loiter_accel_cms / 4500.0f;
}

/// get_loiter_stopping_point_xy - returns vector to stopping point based on a horizontal position and velocity
void AC_WPNav::get_loiter_stopping_point_xy(Vector3f& stopping_point) const
{
	_pos_control.get_stopping_point_xy(stopping_point);
}

/// calc_loiter_desired_velocity - updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
///		updated velocity sent directly to position controller
void AC_WPNav::calc_loiter_desired_velocity(float nav_dt)
{
    // range check nav_dt
    if( nav_dt < 0 ) {
        return;
    }

    // check loiter speed and avoid divide by zero
    if( _loiter_speed_cms < WPNAV_LOITER_SPEED_MIN) {
        _loiter_speed_cms = WPNAV_LOITER_SPEED_MIN;
        _loiter_accel_cms = _loiter_speed_cms/2.0f;
    }

    // rotate pilot input to lat/lon frame
    Vector2f desired_accel;
	// CHM - acceleration along Lat? cos_yaw is cos of angle of heading from the North
	// This is important, showing that LOITER uses Lat/lon position as reference
    desired_accel.x = (_pilot_accel_fwd_cms*_ahrs.cos_yaw() - _pilot_accel_rgt_cms*_ahrs.sin_yaw());
    desired_accel.y = (_pilot_accel_fwd_cms*_ahrs.sin_yaw() + _pilot_accel_rgt_cms*_ahrs.cos_yaw());

    // calculate the difference
	// CHM - _loiter_desired_accel, initialised as half of the initial velocity, this is to prevent jerk
    Vector2f des_accel_diff = (desired_accel - _loiter_desired_accel);

    // constrain and scale the desired acceleration
    float des_accel_change_total = pythagorous2(des_accel_diff.x, des_accel_diff.y);
	// CHM - _loiter_jerk_max_cmsss == WPNAV_LOIT_JERK, defualt 1000 cm/s/s/s
    float accel_change_max = _loiter_jerk_max_cmsss * nav_dt;
    if (des_accel_change_total > accel_change_max && des_accel_change_total > 0.0f) {
        des_accel_diff.x = accel_change_max * des_accel_diff.x/des_accel_change_total;
        des_accel_diff.y = accel_change_max * des_accel_diff.y/des_accel_change_total;
    }
    // adjust the desired acceleration
    _loiter_desired_accel += des_accel_diff;

    // get pos_control's feed forward velocity
	// CHM - return _vel_desired , it is initialised as initial speed
    Vector3f desired_vel = _pos_control.get_desired_velocity();

    // reduce velocity with fake wind resistance
	// CHM - the fake wind should always exist no matter what is the pilot input
	// DELETE if condition here

	// CHM - in the following code, a WPNAV_LOITER_ACCEL_MIN deceleration always applies.
	// This will make sure the UAV decelerate to true 0 speed in time, in max. 5~6 sec
	//
	// Another feature to note is that, the desired velocity will never hit _loiter_speed_cms,
	// due to the damping.
	// In max. acceleration, it can achieve 90% of max. velocity in 5 sec

	desired_vel.x += _loiter_desired_accel.x * nav_dt
		- 0.85*(_loiter_accel_cms - WPNAV_LOITER_ACCEL_MIN)*nav_dt*desired_vel.x / _loiter_speed_cms;
        
	if(desired_vel.x > 0 ) {
		// CHM - when the desired velocity is too small, make it 0
    desired_vel.x = max(desired_vel.x - WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
    }else if(desired_vel.x < 0) {
        desired_vel.x = min(desired_vel.x + WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
    }

	desired_vel.y += _loiter_desired_accel.y * nav_dt
		- 0.85*(_loiter_accel_cms - WPNAV_LOITER_ACCEL_MIN)*nav_dt*desired_vel.y / _loiter_speed_cms;
        
	if(desired_vel.y > 0 ) {
        desired_vel.y = max(desired_vel.y - WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
    }else if(desired_vel.y < 0) {
        desired_vel.y = min(desired_vel.y + WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
    }

    // send adjusted feed forward velocity back to position controller
    _pos_control.set_desired_velocity_xy(desired_vel.x,desired_vel.y);
}

/// get_bearing_to_target - get bearing to loiter target in centi-degrees
int32_t AC_WPNav::get_loiter_bearing_to_target() const
{
    return get_bearing_cd(_inav.get_position(), _pos_control.get_pos_target());
}

/// update_loiter - run the loiter controller - should be called at 100hz
void AC_WPNav::update_loiter()
{
    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _loiter_last_update) / 1000.0f;

    // reset step back to 0 if 0.1 seconds has passed and we completed the last full cycle
	// CHM -
	// Looks like even though pilot command accel update at 400hz, but the _vel_desired is update only at 50hz
	// what's the point?
    if (dt >= WPNAV_LOITER_UPDATE_TIME) {
        // double check dt is reasonable
        if (dt >= 1.0f) {
            dt = 0.0;
        }
        // capture time since last iteration
        _loiter_last_update = now;
        // translate any adjustments from pilot to loiter target
		// CHM - this will set _vel_desired, in lat/lon, based on pilot commanded acceleration
        calc_loiter_desired_velocity(dt);
        // trigger position controller on next update
		// CHM - set _flags.force_recalc_xy = true
        _pos_control.trigger_xy();
    }else{
        // run horizontal position controller
		// CHM - _flags.recalc_leash_xy
        _pos_control.update_xy_controller(true);
    }
}


///
/// waypoint navigation
///

/// wp_and_spline_init - initialise straight line and spline waypoint controllers
///     updates target roll, pitch targets and I terms based on vehicle lean angles
///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination

// CHM - used in AUTO, GUIDED, RTL

void AC_WPNav::wp_and_spline_init()
{
    // check _wp_accel_cms is reasonable
    if (_wp_accel_cms <= 0) {
        _wp_accel_cms.set_and_save(WPNAV_ACCELERATION);
    }

    // initialise position controller
    _pos_control.init_xy_controller();

	// CHM - clear desired velocity, so it does not interefer with the auto mode
	// Need to be verified
	const Vector3f& curr_vel = _inav.get_velocity();
	_pos_control.set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // initialise position controller speed and acceleration
	// CHM - WPNAV_SPEED
    _pos_control.set_speed_xy(_wp_speed_cms); // _speed_cms
    _pos_control.set_accel_xy(_wp_accel_cms); // _accel_cms
    _pos_control.set_speed_z(-_wp_speed_down_cms, _wp_speed_up_cms); // _speed_down_cms _speed_up_cms
    _pos_control.set_accel_z(_wp_accel_z_cms); //_accel_z_cms
    _pos_control.calc_leash_length_xy(); //_leash
    _pos_control.calc_leash_length_z(); //_leash_up_z _leash_down_z
}

/// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
void AC_WPNav::set_speed_xy(float speed_cms)
{
    // range check new target speed and update position controller
    if (_wp_speed_cms >= WPNAV_WP_SPEED_MIN) {
        _wp_speed_cms = speed_cms;
        _pos_control.set_speed_xy(_wp_speed_cms);
        // flag that wp leash must be recalculated
        _flags.recalc_wp_leash = true;
    }
}

/// set_destination - set destination using cm from home
void AC_WPNav::set_wp_destination(const Vector3f& destination)
{
	Vector3f origin;

    // if waypoint controller is active use the existing position target as the origin
    if ((hal.scheduler->millis() - _wp_last_update) < 1000) {
        origin = _pos_control.get_pos_target();
    } else {
        // if waypoint controller is not active, set origin to reasonable stopping point (using curr pos and velocity)
        _pos_control.get_stopping_point_xy(origin);
        _pos_control.get_stopping_point_z(origin);
    }

    // set origin and destination
    set_wp_origin_and_destination(origin, destination);
}

/// set_origin_and_destination - set origin and destination using lat/lon coordinates
void AC_WPNav::set_wp_origin_and_destination(const Vector3f& origin, const Vector3f& destination)
{
    // store origin and destination locations
    _origin = origin;
    _destination = destination;
    Vector3f pos_delta = _destination - _origin;

    _track_length = pos_delta.length(); // get track length

    // calculate each axis' percentage of the total distance to the destination
    if (_track_length == 0.0f) {
        // avoid possible divide by zero
        _pos_delta_unit.x = 0;
        _pos_delta_unit.y = 0;
        _pos_delta_unit.z = 0;
    }else{
		// CHM - the component of the vector should all <= 1.0
        _pos_delta_unit = pos_delta/_track_length;
    }

    // calculate leash lengths
	// CHM - initialising leash for WP
    calculate_wp_leash_length();

    // initialise yaw heading
    if (_track_length >= WPNAV_YAW_DIST_MIN) {
        _yaw = get_bearing_cd(_origin, _destination);
    } else {
        // set target yaw to current heading.  Alternatively we could pull this from the attitude controller if we had access to it
        _yaw = _ahrs.yaw_sensor;
    }

    // initialise intermediate point to the origin
    _pos_control.set_pos_target(origin);
    _track_desired = 0;             // target is at beginning of track
    _flags.reached_destination = false;
    _flags.fast_waypoint = false;   // default waypoint back to slow // CHM - default fast_waypoint mode
    _flags.slowing_down = false;    // target is not slowing down yet
	// CHM - defualt segment_type
    _flags.segment_type = SEGMENT_STRAIGHT;
    _flags.new_wp_destination = true;   // flag new waypoint so we can freeze the pos controller's feed forward and smooth the transition

    // initialise the limited speed to current speed along the track
    const Vector3f &curr_vel = _inav.get_velocity();
    // get speed along track (note: we convert vertical speed into horizontal speed equivalent)
    float speed_along_track = curr_vel.x * _pos_delta_unit.x + curr_vel.y * _pos_delta_unit.y + curr_vel.z * _pos_delta_unit.z;
    _limited_speed_xy_cms = constrain_float(speed_along_track,0,_wp_speed_cms);
}

/// get_wp_stopping_point_xy - returns vector to stopping point based on a horizontal position and velocity
void AC_WPNav::get_wp_stopping_point_xy(Vector3f& stopping_point) const
{
	_pos_control.get_stopping_point_xy(stopping_point);
}

/// advance_wp_target_along_track - move target location along track from origin to destination
void AC_WPNav::advance_wp_target_along_track(float dt)
{
    float track_covered;        // distance (in cm) along the track that the vehicle has traveled.  Measured by drawing a perpendicular line from the track to the vehicle.
    Vector3f track_error;       // distance error (in cm) from the track_covered position (i.e. closest point on the line to the vehicle) and the vehicle
    float track_desired_max;    // the farthest distance (in cm) along the track that the leash will allow
    float track_leash_slack;    // additional distance (in cm) along the track from our track_covered position that our leash will allow
    bool reached_leash_limit = false;   // true when track has reached leash limit and we need to slow down the target point

    // get current location
    Vector3f curr_pos = _inav.get_position();
	// CHM - Relative position of UAV to the last waypoint
    Vector3f curr_delta = curr_pos - _origin;

    // calculate how far along the track we are
	// CHM - Scalar projection,length of track consider covered
    track_covered = curr_delta.x * _pos_delta_unit.x + curr_delta.y * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;

    Vector3f track_covered_pos = _pos_delta_unit * track_covered;
	// CHM - vector error, from UAV to the point on the track
    track_error = curr_delta - track_covered_pos;

    // calculate the horizontal error
    float track_error_xy = pythagorous2(track_error.x, track_error.y);

    // calculate the vertical error
    float track_error_z = fabsf(track_error.z);

    // get position control leash lengths
	// CHM - _pos_control._leash
    float leash_xy = _pos_control.get_leash_xy();
    float leash_z;
    if (track_error.z >= 0) {
        leash_z = _pos_control.get_leash_up_z();
    }else{
        leash_z = _pos_control.get_leash_down_z();
    }

    // calculate how far along the track we could move the intermediate target before reaching the end of the leash
	// CHM - _track_leash_length
	// CHM - change the way to finde track_leash_slack
    //track_leash_slack = min(_track_leash_length*(leash_z-track_error_z)/leash_z, _track_leash_length*(leash_xy-track_error_xy)/leash_xy);
	track_leash_slack = _track_leash_length - track_error_z - track_error_xy;
	if (track_leash_slack < 0) {
        track_desired_max = track_covered;
    }else{
        track_desired_max = track_covered + track_leash_slack;
    }

    // check if target is already beyond the leash
    if (_track_desired > track_desired_max) {
        reached_leash_limit = true;
    }

    // get current velocity
    const Vector3f &curr_vel = _inav.get_velocity();
    // get speed along track
    float speed_along_track = curr_vel.x * _pos_delta_unit.x + curr_vel.y * _pos_delta_unit.y + curr_vel.z * _pos_delta_unit.z;

    // calculate point at which velocity switches from linear to sqrt
    float linear_velocity = _wp_speed_cms;
	// CHM - HLD_LAT_P
    float kP = _pos_control.get_pos_xy_kP();
	// CHM - bug fix: replace >= to >
    if (kP > 0.0f) {   // avoid divide by zero
        linear_velocity = _track_accel/kP;
    }

    // let the limited_speed_xy_cms be some range above or below current velocity along track
    if (speed_along_track < -linear_velocity) {
        // we are traveling fast in the opposite direction of travel to the waypoint so do not move the intermediate point
        // CHM - the term xy is misleading. It is actually speed along the track
		_limited_speed_xy_cms = 0;
    }else{
        // increase intermediate target point's velocity if not yet at the leash limit
        if(dt > 0 && !reached_leash_limit) {
			// CHM - WHY there is a coefficient 2.0 ???
			// CHM - edit: delete 2.0f
			// CHM - add fake wind
			_limited_speed_xy_cms += _track_accel * dt * (1 - 0.85f * _limited_speed_xy_cms / _track_speed);
        }
        // do not allow speed to be below zero or over top speed
		// CHM - should it be _track_speed * pos_delta_unit_xy ? no
        _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms, 0.0f, _track_speed);

        // check if we should begin slowing down
		// CHM - by default fast_waypoint is ture, as long as there is no delay
        if (!_flags.fast_waypoint) {
            float dist_to_dest = _track_length - _track_desired;
            if (!_flags.slowing_down && dist_to_dest <= _slow_down_dist) {
                _flags.slowing_down = true;
            }
            // if target is slowing down, limit the speed
            if (_flags.slowing_down) {
                _limited_speed_xy_cms = min(_limited_speed_xy_cms, get_slow_down_speed(dist_to_dest, _track_accel));
            }
        }

        // if our current velocity is within the linear velocity range limit the intermediate point's velocity to be no more than the linear_velocity above or below our current velocity
        // CHM - this make sure the difference of _limited_speed_xy_cms and actual speed difference is within response range?
		if (fabsf(speed_along_track) < linear_velocity) {
            _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms,speed_along_track-linear_velocity,speed_along_track+linear_velocity);
        }
    }
    // advance the current target
    if (!reached_leash_limit) {
		// CHM - desired travelled distance, in scalar
    	_track_desired += _limited_speed_xy_cms * dt;

    	// reduce speed if we reach end of leash
        if (_track_desired > track_desired_max) {
        	_track_desired = track_desired_max;
			// CHM - edit: delete 2.0f
        	//_limited_speed_xy_cms -= _track_accel * dt;
			_limited_speed_xy_cms -= _track_accel * dt * (1 - 0.85f * _limited_speed_xy_cms / _track_speed);
        	if (_limited_speed_xy_cms < 0.0f) {
        	    _limited_speed_xy_cms = 0.0f;
        	}
    	}
    }

    // do not let desired point go past the end of the track unless it's a fast waypoint
    if (!_flags.fast_waypoint) {
        _track_desired = constrain_float(_track_desired, 0, _track_length);
    } else {
        _track_desired = constrain_float(_track_desired, 0, _track_length + WPNAV_WP_FAST_OVERSHOOT_MAX);
    }

	/// DEBUG _track_desired
	static int i;
	if (i < 0 || i >= 25)
	{
		i = 0;
		hal.uartC->printf_P(PSTR("_track_desired = %.0f, _track_length = %.0f\n" ), _track_desired, _track_length);
	}
	else
		i++;
	/// end debug _track_desired


    // recalculate the desired position
	// CHM - set "_pos_target" here
    _pos_control.set_pos_target(_origin + _pos_delta_unit * _track_desired);

    // check if we've reached the waypoint
    if( !_flags.reached_destination ) {
        if( _track_desired >= _track_length ) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            if (_flags.fast_waypoint) {
                _flags.reached_destination = true;
            }else{
                // regular waypoints also require the copter to be within the waypoint radius
                Vector3f dist_to_dest = curr_pos - _destination;
                if( dist_to_dest.length() <= _wp_radius_cm ) {
                    _flags.reached_destination = true;
                }
            }
        }
    }
}

/// get_wp_distance_to_destination - get horizontal distance to destination in cm
float AC_WPNav::get_wp_distance_to_destination() const
{
    // get current location
    Vector3f curr = _inav.get_position();
    return pythagorous2(_destination.x-curr.x,_destination.y-curr.y);
}

/// get_wp_bearing_to_destination - get bearing to next waypoint in centi-degrees
int32_t AC_WPNav::get_wp_bearing_to_destination() const
{
    return get_bearing_cd(_inav.get_position(), _destination);
}

/// update_wpnav - run the wp controller - should be called at 100hz or higher
void AC_WPNav::update_wpnav()
{
    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _wp_last_update) / 1000.0f;

    // reset step back to 0 if 0.1 seconds has passed and we completed the last full cycle
    if (dt >= WPNAV_WP_UPDATE_TIME) {
        // double check dt is reasonable
        if (dt >= 1.0f) {
            dt = 0.0;
        }
        // capture time since last iteration
        _wp_last_update = now;

        // advance the target if necessary
        advance_wp_target_along_track(dt);
        _pos_control.trigger_xy();
        if (_flags.new_wp_destination) {
            _flags.new_wp_destination = false;
            _pos_control.freeze_ff_xy();
        }
        _pos_control.freeze_ff_z();
    }else{
        // run horizontal position controller
        _pos_control.update_xy_controller(false);

        // check if leash lengths need updating
        check_wp_leash_length();
    }
}

// check_wp_leash_length - check if waypoint leash lengths need to be recalculated
//  should be called after _pos_control.update_xy_controller which may have changed the position controller leash lengths
void AC_WPNav::check_wp_leash_length()
{
    // exit immediately if recalc is not required
    if (_flags.recalc_wp_leash) {
        calculate_wp_leash_length();
    }
}

/// calculate_wp_leash_length - calculates horizontal and vertical leash lengths for waypoint controller
void AC_WPNav::calculate_wp_leash_length()
{
    // length of the unit direction vector in the horizontal
    float pos_delta_unit_xy = pythagorous2(_pos_delta_unit.x, _pos_delta_unit.y);
    float pos_delta_unit_z = fabsf(_pos_delta_unit.z);

    float speed_z;
    float leash_z;
    if (_pos_delta_unit.z >= 0.0f) {
        speed_z = _wp_speed_up_cms;
        leash_z = _pos_control.get_leash_up_z();
    }else{
        speed_z = _wp_speed_down_cms;
        leash_z = _pos_control.get_leash_down_z();
    }

    // calculate the maximum acceleration, maximum velocity, and leash length in the direction of travel
    if(pos_delta_unit_z == 0.0f && pos_delta_unit_xy == 0.0f){
        _track_accel = 0.0f;
        _track_speed = 0.0f;
        _track_leash_length = WPNAV_LEASH_LENGTH_MIN;
    }else if(_pos_delta_unit.z == 0.0f){
		// CHM - pos_delta_unit_xy is the length of unit vector in the direction of track on XY plane
		// interesting to note that it is / not * .
		// CHM - clean the code
        _track_accel = _wp_accel_cms;
        _track_speed = _wp_speed_cms;
        _track_leash_length = _pos_control.get_leash_xy();
    }else if(pos_delta_unit_xy == 0.0f){
        _track_accel = _wp_accel_z_cms;
        _track_speed = speed_z;
        _track_leash_length = leash_z;
    }else{

		// CHM - add
		float elevation_angle = degrees(fast_atan2(pos_delta_unit_z, pos_delta_unit_xy));
		elevation_angle = constrain_float(elevation_angle, 0.0f, 90.0f);
		float z_fraction = elevation_angle / 90.0f;
		float xy_fraction = 1.0f - z_fraction;

		// CHM - this algorithm may create too much acceleration during flying at an upward angle
        //_track_accel = min(_wp_accel_z_cms/pos_delta_unit_z, _wp_accel_cms/pos_delta_unit_xy);
        //_track_speed = min(speed_z/pos_delta_unit_z, _wp_speed_cms/pos_delta_unit_xy);
        //_track_leash_length = min(leash_z/pos_delta_unit_z, _pos_control.get_leash_xy()/pos_delta_unit_xy);

		_track_accel = _wp_accel_z_cms * z_fraction + _wp_accel_cms * xy_fraction;
		_track_speed = speed_z * z_fraction + _wp_speed_cms * xy_fraction;
		_track_leash_length = leash_z * z_fraction + _pos_control.get_leash_xy() * xy_fraction;

		// CHM - debug message
		hal.uartC->printf_P(PSTR("New Track: a=%f, v=%f, l=%f\n"), _track_accel, _track_speed, _track_leash_length);
		Log_Write_Data(DATA_WP_TRACK, _track_accel);
		Log_Write_Data(DATA_WP_TRACK, _track_speed);
		Log_Write_Data(DATA_WP_TRACK, _track_leash_length);

    }

    // calculate slow down distance (the distance from the destination when the target point should begin to slow down)
    calc_slow_down_distance(_track_speed, _track_accel);

    // set recalc leash flag to false
    _flags.recalc_wp_leash = false;
}

///
/// spline methods
///

/// set_spline_destination waypoint using position vector (distance from home in cm)
///     stopped_at_start should be set to true if vehicle is stopped at the origin
///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
void AC_WPNav::set_spline_destination(const Vector3f& destination, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination)
{
    Vector3f origin;

    // if waypoint controller is active and copter has reached the previous waypoint use current pos target as the origin
    if ((hal.scheduler->millis() - _wp_last_update) < 1000) {
        origin = _pos_control.get_pos_target();
    }else{
        // otherwise calculate origin from the current position and velocity
        _pos_control.get_stopping_point_xy(origin);
        _pos_control.get_stopping_point_z(origin);
    }

    // set origin and destination
    set_spline_origin_and_destination(origin, destination, stopped_at_start, seg_end_type, next_destination);
}

/// set_spline_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
///     seg_type should be calculated by calling function based on the mission
void AC_WPNav::set_spline_origin_and_destination(const Vector3f& origin, const Vector3f& destination, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination)
{
    // mission is "active" if wpnav has been called recently and vehicle reached the previous waypoint
    // CHM - _flags.reached_destination can be used to detect if there is a previous nav cmd
	bool prev_segment_exists = (_flags.reached_destination && ((hal.scheduler->millis() - _wp_last_update) < 1000));

    // check _wp_accel_cms is reasonable to avoid divide by zero
    if (_wp_accel_cms <= 0) {
        _wp_accel_cms.set_and_save(WPNAV_ACCELERATION);
    }

    // segment start types
    // stop - vehicle is not moving at origin
    // straight-fast - vehicle is moving, previous segment is straight.  vehicle will fly straight through the waypoint before beginning it's spline path to the next wp
    //     _flag.segment_type holds whether prev segment is straight vs spline but we don't know if it has a delay
    // spline-fast - vehicle is moving, previous segment is splined, vehicle will fly through waypoint but previous segment should have it flying in the correct direction (i.e. exactly parallel to position difference vector from previous segment's origin to this segment's destination)

    // calculate spline velocity at origin
    if (stopped_at_start || !prev_segment_exists) {
    	// if vehicle is stopped at the origin, set origin velocity to 0.1 * distance vector from origin to destination
		// CHM - modified
		const Vector3f &curr_vel = _inav.get_velocity();
		_spline_origin_vel = curr_vel * 2.5f; 
    	_spline_time = 0.0f;
    	_spline_vel_scaler = curr_vel.length();
    }else{
    	// look at previous segment to determine velocity at origin
        if (_flags.segment_type == SEGMENT_STRAIGHT) {
            // previous segment is straight, vehicle is moving so vehicle should fly straight through the origin
            // before beginning it's spline path to the next waypoint. Note: we are using the previous segment's origin and destination
			_spline_origin_vel = (_destination - _origin)*2.0f; // CHM do we need to modify this to match current velocity? no
			
			_spline_time = 0.0f;	// To-Do: this should be set based on how much overrun there was from straight segment?
            _spline_vel_scaler = _pos_control.get_vel_target().length();    // start velocity target from current target velocity
        }else{
            // previous segment is splined, vehicle will fly through origin
            // we can use the previous segment's destination velocity as this segment's origin velocity
            // Note: previous segment will leave destination velocity parallel to position difference vector
            //       from previous segment's origin to this segment's destination)
            _spline_origin_vel = _spline_destination_vel;
            /*if (_spline_time > 1.0f && _spline_time < 1.1f) {    // To-Do: remove hard coded 1.1f
                _spline_time -= 1.0f;
            }else{
                _spline_time = 0.0f;
            }*/
			_spline_time = 0.0f;
            // Note: we leave _spline_vel_scaler as it was from end of previous segment
        }
    }

    // calculate spline velocity at destination
    switch (seg_end_type) {

    case SEGMENT_END_STOP:
        // if vehicle stops at the destination set destination velocity to 0.1 * distance vector from origin to destination
        _spline_destination_vel = (destination - origin).normalized(); // change to 0.1m/s in magenitude
        _flags.fast_waypoint = false;
        break;

    case SEGMENT_END_STRAIGHT:
        // if next segment is straight, vehicle's final velocity should face along the next segment's position
		_spline_destination_vel = (next_destination - destination)*2.0f;
        _flags.fast_waypoint = true;
        break;

    case SEGMENT_END_SPLINE:
        // if next segment is splined, vehicle's final velocity should face parallel to the line from the origin to the next destination
		//_spline_destination_vel = (next_destination - origin);
		float l1 = (destination - origin).length();
		float l2 = (next_destination - destination).length();

		_spline_destination_vel = (destination - origin).normalized()*(l2 / (l1 + l2)) + \
			(next_destination - destination).normalized()*(l1 / (l1 + l2));
		_spline_destination_vel.normalize();
		_spline_destination_vel *= (l1 + l2) / 2.0f;

        _flags.fast_waypoint = true;
        break;
    }

    // code below ensures we don't get too much overshoot when the next segment is short
    float vel_len = (_spline_origin_vel - _spline_destination_vel).length(); // CHM - changed to minus sign
    float pos_len = (destination - origin).length() * 4.0f;
    if (vel_len > pos_len) {
        // if total start+stop velocity is more than twice position difference
        // use a scaled down start and stop velocityscale the  start and stop velocities down
        float vel_scaling = pos_len / vel_len;
		hal.uartC->printf_P(PSTR("Spline: Being rescaled, vel_scaling=%f!!!!!!!!!!\n"), vel_scaling);
        // update spline calculator
        update_spline_solution(origin, destination, _spline_origin_vel * vel_scaling, _spline_destination_vel * vel_scaling);
    }else{
        // update spline calculator
        update_spline_solution(origin, destination, _spline_origin_vel, _spline_destination_vel);
    }

    // initialise yaw heading to current heading
    _yaw = _ahrs.yaw_sensor;

    // store origin and destination locations
    _origin = origin;
    _destination = destination;

    // calculate slow down distance
    calc_slow_down_distance(_wp_speed_cms, _wp_accel_cms);

    // initialise intermediate point to the origin
    _pos_control.set_pos_target(origin);
    _flags.reached_destination = false;
    _flags.segment_type = SEGMENT_SPLINE;
	if (stopped_at_start || !prev_segment_exists)
		_flags.new_wp_destination = true;   // flag new waypoint so we can freeze the pos controller's feed forward and smooth the transition
}

/// update_spline - update spline controller
void AC_WPNav::update_spline()
{
    // exit immediately if this is not a spline segment
    if (_flags.segment_type != SEGMENT_SPLINE) {
        return;
    }

    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _wp_last_update) / 1000.0f;

    // reset step back to 0 if 0.1 seconds has passed and we completed the last full cycle
    if (dt >= WPNAV_WP_UPDATE_TIME) {
        // double check dt is reasonable
        if (dt >= 1.0f) {
            dt = 0.0;
        }
        // capture time since last iteration
        _wp_last_update = now;

        // advance the target if necessary
        advance_spline_target_along_track(dt);
        _pos_control.trigger_xy();
        if (_flags.new_wp_destination) {
            _flags.new_wp_destination = false;
            _pos_control.freeze_ff_xy();
        }
        _pos_control.freeze_ff_z();
    }else{
        // run horizontal position controller
        _pos_control.update_xy_controller(false);
    }
}

/// update_spline_solution - recalculates hermite_spline_solution grid
///		relies on _spline_origin_vel, _spline_destination_vel and _origin and _destination
void AC_WPNav::update_spline_solution(const Vector3f& origin, const Vector3f& dest, const Vector3f& origin_vel, const Vector3f& dest_vel)
{
    _hermite_spline_solution[0] = origin;
    _hermite_spline_solution[1] = origin_vel;
    _hermite_spline_solution[2] = -origin*3.0f -origin_vel*2.0f + dest*3.0f - dest_vel;
    _hermite_spline_solution[3] = origin*2.0f + origin_vel -dest*2.0f + dest_vel;
 }

/// advance_spline_target_along_track - move target location along track from origin to destination
// CHM - called at 50 hz, dt ~= 0.02
void AC_WPNav::advance_spline_target_along_track(float dt)
{
	static int i = 0;

	if (_spline_time == 0.0f)
		hal.uartC->printf_P(PSTR("Spline new: %u\n"), (unsigned)hal.scheduler->millis());



	if (_spline_time <= 1.0f) {
        Vector3f target_pos, target_vel, target_accel;


        // update target position and velocity from spline calculator
		calc_spline_pos_vel(_spline_time, target_pos, target_vel, target_accel);

        // update velocity
        float spline_dist_to_wp = (_destination - target_pos).length();

        // if within the stopping distance from destination, set target velocity to sqrt of distance * 2 * acceleration
        if (!_flags.fast_waypoint && spline_dist_to_wp < _slow_down_dist) {
            _spline_vel_scaler = safe_sqrt(spline_dist_to_wp * 2.0f * _wp_accel_cms);
        }else if(_spline_vel_scaler < _wp_speed_cms) {
            // increase velocity using acceleration
        	// To-Do: replace 0.1f below with update frequency passed in from main program
			// CHM - can add fake wind to this
			_spline_vel_scaler += _wp_accel_cms* dt * (1 - 0.85f * _spline_vel_scaler / _wp_speed_cms);
        }

        // constrain target velocity
        if (_spline_vel_scaler > _wp_speed_cms) {
            _spline_vel_scaler = _wp_speed_cms;
        }

        // scale the spline_time by the velocity we've calculated vs the velocity that came out of the spline calculator
		// CHM - target_vel is from the result of equation
        float target_vel_length = target_vel.length();
        if (target_vel_length != 0.0f) {
            _spline_time_scale = _spline_vel_scaler/target_vel_length;

			// CHM - now limit the _spline_time_scale furthur down, if acceleration exceed the _wp_accel_cms
			float equivalent_accel = _spline_time_scale * _spline_time_scale * target_accel.length();
			if (equivalent_accel > _wp_accel_cms && equivalent_accel != 0.0f)
			{
				_spline_time_scale *= safe_sqrt(_wp_accel_cms / equivalent_accel);
				if (i % 20 == 0)
				{
					hal.uartC->printf_P(PSTR("Spline: [Accel limit] from %f to %f\n"), equivalent_accel, _spline_time_scale * _spline_time_scale * target_accel.length());
				}
				i++;
			}
			else
				i = 0;
        }

        // update target position
        _pos_control.set_pos_target(target_pos);

        // update the yaw
        _yaw = RadiansToCentiDegrees(fast_atan2(target_vel.y,target_vel.x));





		// CHM - add feedback from actual positon
		Vector3f pos_error = target_pos - _inav.get_position();

		if (_spline_time >= 1.0f || (_spline_time + _spline_time_scale*dt * 6 >= 1.0f)) {
			hal.uartC->printf_P(PSTR("Spline Reached - time:%u\n"), (unsigned)hal.scheduler->millis());
			hal.uartC->printf_P(PSTR("pos_error=%f m\n"), pos_error.length()/100.0f);
			_flags.reached_destination = true;
		}

        // advance spline time to next step
		// CHM - ADD constraint to update
		if (pos_error.length() < _pos_control.get_leash_xy()*1.1f)
			_spline_time += _spline_time_scale*dt;

		if (_spline_time > 1.0f)
			_spline_time = 1.0f;

        // we will reach the next waypoint in the next step so set reached_destination flag
        // To-Do: is this one step too early?
		//pos_error = _destination - target_pos;
    }
}

// calc_spline_pos_vel_accel - calculates target position, velocity and acceleration for the given "spline_time"
/// 	relies on update_spline_solution being called when the segment's origin and destination were set
void AC_WPNav::calc_spline_pos_vel(float spline_time, Vector3f& position, Vector3f& velocity, Vector3f& acceleration)
{
    float spline_time_sqrd = spline_time * spline_time;
    float spline_time_cubed = spline_time_sqrd * spline_time;

    position = _hermite_spline_solution[0] + \
               _hermite_spline_solution[1] * spline_time + \
               _hermite_spline_solution[2] * spline_time_sqrd + \
               _hermite_spline_solution[3] * spline_time_cubed;

    velocity = _hermite_spline_solution[1] + \
               _hermite_spline_solution[2] * 2.0f * spline_time + \
               _hermite_spline_solution[3] * 3.0f * spline_time_sqrd;

	// CHM - add acceleration in path planning, used to limit the max acceleration of the planned path later on
	acceleration =	_hermite_spline_solution[2] * 2.0f + \
					_hermite_spline_solution[3] * 6.0f * spline_time;
}


///
/// shared methods
///

// get_bearing_cd - return bearing in centi-degrees between two positions
// To-Do: move this to math library
float AC_WPNav::get_bearing_cd(const Vector3f &origin, const Vector3f &destination) const
{
    float bearing = 9000 + fast_atan2(-(destination.x-origin.x), destination.y-origin.y) * 5729.57795f;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

/// calc_slow_down_distance - calculates distance before waypoint that target point should begin to slow-down assuming it is travelling at full speed
void AC_WPNav::calc_slow_down_distance(float speed_cms, float accel_cmss)
{
	// protect against divide by zero
	if (accel_cmss <= 0.0f) {
		_slow_down_dist = 0.0f;
		return;
	}
    // To-Do: should we use a combination of horizontal and vertical speeds?
    // To-Do: update this automatically when speed or acceleration is changed
	// CHM - may be change to 2.0f?
    _slow_down_dist = speed_cms * speed_cms / (4.0f*accel_cmss);
}

/// get_slow_down_speed - returns target speed of target point based on distance from the destination (in cm)
float AC_WPNav::get_slow_down_speed(float dist_from_dest_cm, float accel_cmss)
{
    // return immediately if distance is zero (or less)
    if (dist_from_dest_cm <= 0) {
        return WPNAV_WP_TRACK_SPEED_MIN;
    }

    // calculate desired speed near destination
    float target_speed = safe_sqrt(dist_from_dest_cm * 4.0f * accel_cmss);

    // ensure desired speed never becomes too low
    if (target_speed < WPNAV_WP_TRACK_SPEED_MIN) {
        return WPNAV_WP_TRACK_SPEED_MIN;
    } else {
        return target_speed;
    }
}
