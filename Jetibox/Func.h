/*
	 Mav2DupEx Version 0.1
	 2014, by DevFor8.com, info@devfor8.com
	 
	 part of code is based on ArduCAM OSD
*/

//------------------ Home Distance and Direction Calculation ----------------------------------

unsigned long lastmillis = 0;

void setHomeVars()
{
  float dstlon, dstlat;
  long bearing;
  
  if (lastmillis < millis())
  {
     lastmillis = millis()+1000; //next second
    
     osd_capacity_mA = osd_capacity_mA + (osd_curr_A / 3.6);
    
  }
  
  if (osd_fix_type >1) //0-1 = no fix, confusing. 0 = nofix, 2=2D , 3=3D
  { osd_fix_type_jeti = osd_fix_type;}
  else
  { osd_fix_type_jeti = 0;}
  
  
  if(osd_got_home == 0 && osd_fix_type > 1){
    osd_home_lat = osd_lat;
    osd_home_lon = osd_lon;
    //osd_home_alt = osd_alt;
    osd_got_home = 1;
  }
  else if(osd_got_home == 1){
    // JRChange: osd_home_alt: check for stable osd_alt (must be stable for 25*120ms = 3s)
    if(osd_alt_cnt < 25){
      if(fabs(osd_alt_prev - osd_alt) > 0.5){
        osd_alt_cnt = 0;
        osd_alt_prev = osd_alt;
      }
      else
      {
        if(++osd_alt_cnt >= 25){
          osd_home_alt = osd_alt;  // take this stable osd_alt as osd_home_alt
        }
      }
    }
    
    if (osd_home_alt != -9) // we have it
      {
        osd_alt = osd_baro_alt - osd_home_alt;
      }
    
    // shrinking factor for longitude going to poles direction
    float rads = fabs(osd_home_lat) * 0.0174532925;
    double scaleLongDown = cos(rads);
    double scaleLongUp   = 1.0f/cos(rads);

    //DST to Home
    dstlat = fabs(osd_home_lat - osd_lat) * 111319.5;
    dstlon = fabs(osd_home_lon - osd_lon) * 111319.5 * scaleLongDown;
    osd_home_distance = sqrt(sq(dstlat) + sq(dstlon));

    //DIR to Home
    dstlon = (osd_home_lon - osd_lon); //OffSet_X
    dstlat = (osd_home_lat - osd_lat) * scaleLongUp; //OffSet Y
    bearing = 90 + (atan2(dstlat, -dstlon) * 57.295775); //absolut home direction
    if(bearing < 0) bearing += 360;//normalization
    bearing = bearing - 180;//absolut return direction
    if(bearing < 0) bearing += 360;//normalization
    bearing = bearing - osd_heading;//relative home direction
    if(bearing < 0) bearing += 360; //normalization
    
    osd_home_heading = bearing;
        
//    osd_home_direction = round((float)(bearing/360.0f) * 16.0f) + 1;//array of arrows =)
//    if(osd_home_direction > 16) osd_home_direction = 0;

  }

}

