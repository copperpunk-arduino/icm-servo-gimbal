boolean imuTasks()
{
  if (new_icm_data_)
  {
    new_icm_data_ = false;
    convertQuatToEuler();
    return true;
  }
  return false;
}

void convertQuatToEuler()
{
  attitude_rad_[0] = atan2(2.0 * (quat_[0] * quat_[1] + quat_[2] * quat_[3]), (1.0 - 2.0 * (quat_[1] * quat_[1] + quat_[2] * quat_[2])));
  attitude_rad_[1] = asin(2 * (quat_[0] * quat_[2] - quat_[3] * quat_[1]));
  attitude_rad_[2] = atan2(2.0 * (quat_[0] * quat_[3] + quat_[1] * quat_[2]), (1.0 - 2.0 * (quat_[2] * quat_[2] + quat_[3] * quat_[3])));
}