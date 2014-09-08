/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
	//strcpy(mydebug.name, "decoupl_k");
	//mydebug.valuef = attitude_control.pitch_decouple_factor;
	//debug_send_message(MSG_NAMED_VALUE_FLOAT);
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
	
	
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{

	float v_xy = inertial_nav.get_velocity_xy() / 100;

	float v_z = inertial_nav.get_velocity_z() / 100;

	float accel_z = ahrs.get_accel_ef().z + GRAVITY_MSS;
	Vector2f accel_xy;
	accel_xy.x = ahrs.get_accel_ef().x;
	accel_xy.y = ahrs.get_accel_ef().y;

	if (fabs(v_xy)>9.0 || fabs(v_z)>2.0 || fabs(accel_xy.length())>4.0 || fabs(accel_z)>3.0)
		hal.uartC->printf_P(PSTR("Vxy=%5.1f  Vz=%5.1f   Axy=%5.1f  Az=%5.1f\n"), v_xy, v_z, accel_xy.length(), accel_z);


	static int i;
	if (circle_started)
	{
		Vector3f pos_diff = inertial_nav.get_position() - circle_nav.get_center();
		float pos_diff_length = pos_diff.length()/100.0f; // in meter

		hal.uartC->printf_P(PSTR("%d %.9f %.9f 1 %.2f\n"), i, inertial_nav.get_latitude() * 1.0e-7f, inertial_nav.get_longitude() *1.0e-7f,pos_diff_length);
		i++;
	}
	else
		i = 0;

}
#endif