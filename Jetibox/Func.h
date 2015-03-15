/*
	 Mav2DupEx Version 0.1
	 2014, by DevFor8.com, info@devfor8.com
	 fixes by MTBSTEVE
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
    if (osd_curr_A >= 0)
     {
         osd_capacity_mA = osd_capacity_mA + (osd_curr_A / 0.36);
     }
    
  }
  
 //0 = no GPS, 1 = nofix, 2=2D , 3=3D
  { osd_fix_type_jeti = osd_fix_type;}
  
  
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
// fix by rosewhite
void calcGPS() {
  uint16_t gps_lat_gr, gps_lon_gr, gps_lat_ms, gps_lon_ms;
  long lat_tmp, lon_tmp;
  long osd_lat_org_tmp, osd_lon_org_tmp;
  float osd_lat_tmp, osd_lon_tmp;
  
  if(osd_lat_org >= 0) {
    gps_lat[0] = 78; // North
    osd_lat_org_tmp = osd_lat_org;
    osd_lat_tmp = osd_lat;
  } 
  else {
    gps_lat[0] = 83; // South
    osd_lat_org_tmp = -osd_lat_org;
    osd_lat_tmp = -osd_lat;
  }
  gps_lat_gr = (uint16_t)osd_lat_tmp;
  lat_tmp = osd_lat_org_tmp-(gps_lat_gr*10000000.0f);
  gps_lat_ms = lat_tmp*6/1000;
  gps_lat[1] = gps_lat_gr;
  gps_lat[2] = (uint8_t)(gps_lat_ms>>8);
  gps_lat[3] = (uint8_t)(gps_lat_ms&0xFF);

  if(osd_lon_org >= 0) {
    gps_lon[0] = 69; // East
    osd_lon_org_tmp = osd_lon_org;
    osd_lon_tmp = osd_lon;
  } 
  else {
    gps_lon[0] = 100; //West
    osd_lon_org_tmp = -osd_lon_org;
    osd_lon_tmp = -osd_lon;
  }
  gps_lon_gr = (uint16_t)osd_lon_tmp;
  lon_tmp = osd_lon_org_tmp-(gps_lon_gr*10000000.0f);
  gps_lon_ms = lon_tmp*6/1000;
  gps_lon[1] = gps_lon_gr;
  gps_lon[2] = (uint8_t)(gps_lon_ms>>8);
  gps_lon[3] = (uint8_t)(gps_lon_ms&0xFF);
}
