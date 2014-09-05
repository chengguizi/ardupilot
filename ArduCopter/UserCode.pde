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
	float v_xy = inertial_nav.get_velocity_xy()/100;
	
	float v_z = inertial_nav.get_velocity_z()/100;

	float accel_z = ahrs.get_accel_ef().z+ GRAVITY_MSS;
	Vector2f accel_xy;
	accel_xy.x = ahrs.get_accel_ef().x;
	accel_xy.y = ahrs.get_accel_ef().y;

	if( fabs(v_xy)>9.0 || fabs(v_z)>2.0 || fabs(accel_xy.length())>4.0 || fabs(accel_z)>3.0)
		hal.uartC->printf_P(PSTR("Vxy=%5.1f  Vz=%5.1f   Axy=%5.1f  Az=%5.1f\n"),v_xy,v_z,accel_xy.length(),accel_z);
	
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
	//gcs_send_message(MSG_NAMED_VALUE_FLOAT);
	//gcs_send_text_P(SEVERITY_MEDIUM, PSTR("message test"));
	//hal.console->print_P(PSTR("CHM test message - hal.console \n"));
	//hal.uartC->print_P(PSTR("CHM test message - uartC \n"));

}
#endif