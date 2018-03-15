
int changeLane(int lane, vector<double> gap, vector<vector<double>> sensor_fusion, int prev_size, double car_s )
{
  if (lane == 1) // if we are in middle lane
  {
    int check_lane = 0;  // check if the left lane is free
    bool is_free = isLaneFree(check_lane, gap, sensor_fusion, prev_size, car_s);
    if (is_free == true)
    {
      lane = check_lane;
    }

    if (is_free == false) // if the left lane is not free, check the right lane
    {
      int check_lane = 2;
      bool is_free = isLaneFree(check_lane, gap, sensor_fusion, prev_size, car_s);
      if (is_free == true)
      {
        lane = check_lane;
      }
    }
  }
  

  else // if we are initially in the left or right lanes
  {
    int check_lane = 1; // check middle lane
    bool is_free = isLaneFree(check_lane, gap, sensor_fusion, prev_size, car_s);
    if (is_free == true)
    {
      lane = check_lane;
    }
  }

  return lane;
}
