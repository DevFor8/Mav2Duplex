/*
	 Mav2DupEx Version 0.1
	 2014, by DevFor8.com, info@devfor8.com
	 
	 part of code is based on ArduCAM OSD
*/
	 
	 /*Panels variables*/
//Will come from APM telem port

// static float	    nav_roll = 0; // Current desired roll in degrees
// static float        nav_pitch = 0; // Current desired pitch in degrees
// static int16_t	    nav_bearing = 0; // Current desired heading in degrees
// static float	    alt_error = 0; // Current altitude error in meters
// static float        aspd_error = 0; // Current airspeed error in meters/second
// static float	    xtrack_error = 0; // Current crosstrack error on x-y plane in meters
// static float        eff = 0; //Efficiency

#define LCDMaxPos 32

 static short      base_mode=0;
 static short         motor_armed = 0;

// static boolean      switch_mode = 0;

// static int8_t       setup_menu = 0;
// static float        converts = 0;
// static float        converth = 0;
// static uint8_t      battv = 0;                //Battery warning voltage - units Volt *10 
//static int        battp = 0;

// static uint8_t      spe = 0;
// static uint8_t      high = 0;
 static float        osd_vbat_A = 0;                 // Battery A voltage in milivolt
 static int16_t      osd_curr_A = 0;                 // Battery A current
 static short       osd_battery_remaining_A = 0;    // 0 to 100 <=> 0 to 1000
 static short        osd_capacity_mA = 0;
// static uint8_t      batt_warn_level = 0;

// static float        start_Time = -1.0;
 static uint8_t      osd_mode = 0;                   // Navigation mode from RC AC2 = CH5, APM = CH8
// static uint8_t      osd_nav_mode = 0;               // Navigation mode from RC AC2 = CH5, APM = CH8
// static unsigned long text_timer = 0;
// static unsigned long warning_timer =0;

// static uint8_t      warning_type = 0;
// static uint8_t      last_warning = 0;
// s/tatic uint8_t      warning = 0;
// static uint8_t      osd_off_switch = 0;
// static uint8_t      osd_switch_last = 100;
// static unsigned long         osd_switch_time = 0;
// static unsigned long         descendt = 0;
// static unsigned long         palt = 0;
// static float        osd_climb = 0;
// static float        descend = 0;

 static float        osd_lat = 0;                    // latidude
 static float        osd_lon = 0;                    // longitude
 static short      osd_satellites_visible = 0;     // number of satelites
 static uint8_t      osd_fix_type = 0;               // GPS lock 0-1=no fix, 2=2D, 3=3D
 static short      osd_fix_type_jeti = 0;               // GPS lock 0-1=no fix, 2=2D, 3=3D

 static uint8_t      osd_got_home = 0;               // tels if got home position or not
 static float        osd_home_lat = 0;               // home latidude
 static float        osd_home_lon = 0;               // home longitude
 static float        osd_home_alt = -9; 
 static short      osd_home_distance = 0;          // distance from home
 static short      osd_home_altdif = -9;          // distance from home
// static uint8_t      osd_home_direction = 0;             // Arrow direction pointing to home (1-16 to CW loop)
 static short      osd_home_heading = 0;             // Arrow direction pointing to home (1-16 to CW loop)
// static float        glide = 0;

 static short       osd_pitch = 0;                  // pitch from DCM
 static short       osd_roll = 0;                   // roll from DCM
 static short       osd_yaw = 0;                    // relative heading form DCM
 static float        osd_heading = 0;                // ground course heading from GPS

 static float        osd_alt = 0;                    // altitude
 static float        osd_baro_alt = 0;                    // altitude
// static float        osd_airspeed = -1;              // airspeed
// static float        osd_windspeed = 0;
// static float        osd_windspeedz = 0;
// static float        osd_winddirection = 0;
// static int8_t       osd_wind_arrow_rotate_int;

 static uint8_t      osd_alt_cnt = 0;              // counter for stable osd_alt
 static float        osd_alt_prev = 0;             // previous altitude

// static float        osd_groundspeed = 0;            // ground speed
// static uint16_t     osd_throttle = 0;               // throtle

//MAVLink session control
 static boolean      mavbeat = 0;
 static float        lastMAVBeat = 0;
 static boolean      waitingMAVBeats = 1;
 static uint8_t      apm_mav_type;
 static int8_t      apm_mav_system; 
 static uint8_t      apm_mav_component;
 static boolean      enable_mav_request = 0;

static char LastMessage[LCDMaxPos];

 static short      debug1 = 0;
 static short      debug2 = 0;
 static float        debug3 = 0;


