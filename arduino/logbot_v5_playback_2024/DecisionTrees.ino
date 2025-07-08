/*

Decision Tree

The model was trained to classify 
"flying (active and passive flight)" or "other (non-flying)" behaviours
using tri-axial acceleration data (acc_x, acc_y, and acc_z).

The model was used for Umineko 2024 experiment in Kabushima, Japan.
The model was last updated on 2024-05-01.

*/ 


static uint8_t Run_Umineko_Flying_Binary_2_2024()
{ 
  // Number of features = 4
  float acc_mag_var = Variance(magnitude_buffer_copy, ACC_BUFFER_SIZE);
  float acc_mag_mean = Mean(magnitude_buffer_copy, ACC_BUFFER_SIZE);
  float acc_mag_mc = ACCRollingMeanCross(magnitude_buffer_copy, acc_mag_mean);
  float acc_mag_kurtosis = Kurtosis(magnitude_buffer_copy);

  // Tree
  if (acc_mag_var <= 0.1343757063150406)
  {
    if (acc_mag_var <= 0.004335553850978613)
    {
      if (acc_mag_var <= 0.0017875659395940602)
      {
        if (acc_mag_var <= 0.00115390116116032)
        {
          if (acc_mag_mean <= 0.8158162534236908)
          {
            return 1; // others
          }
          else
          {
            return 1; // others
          }
        }
        else
        {
          if (acc_mag_mean <= 0.6596124768257141)
          {
            return 0; // flying
          }
          else
          {
            return 1; // others
          }
        }
      }
      else
      {
        if (acc_mag_mc <= 7.5)
        {
          if (acc_mag_kurtosis <= 3.5018835067749023)
          {
            return 1; // others
          }
          else
          {
            return 1; // others
          }
        }
        else
        {
          if (acc_mag_mc <= 9.5)
          {
            return 1; // others
          }
          else
          {
            return 1; // others
          }
        }
      }
    }
    else
    {
      if (acc_mag_mc <= 7.193576335906982)
      {
        if (acc_mag_kurtosis <= 5.090475559234619)
        {
          if (acc_mag_mean <= 1.5042860507965088)
          {
            return 1; // others
          }
          else
          {
            return 0; // flying
          }
        }
        else
        {
          if (acc_mag_kurtosis <= 7.469491481781006)
          {
            return 1; // others
          }
          else
          {
            return 1; // others
          }
        }
      }
      else
      {
        if (acc_mag_mean <= 1.5635777711868286)
        {
          if (acc_mag_mc <= 9.5)
          {
            return 1; // others
          }
          else
          {
            return 1; // others
          }
        }
        else
        {
          if (acc_mag_kurtosis <= 5.996149301528931)
          {
            return 0; // flying
          }
          else
          {
            return 1; // others
          }
        }
      }
    }
  }
  else
  {
    if (acc_mag_kurtosis <= 2.385233521461487)
    {
      if (acc_mag_mc <= 9.5)
      {
        if (acc_mag_kurtosis <= 2.0952411890029907)
        {
          if (acc_mag_mc <= 3.5)
          {
            return 0; // flying
          }
          else
          {
            return 0; // flying
          }
        }
        else
        {
          if (acc_mag_mc <= 8.5)
          {
            return 0; // flying
          }
          else
          {
            return 0; // flying
          }
        }
      }
      else
      {
        if (acc_mag_var <= 0.23218406736850739)
        {
          if (acc_mag_kurtosis <= 2.3275035619735718)
          {
            return 1; // others
          }
          else
          {
            return 0; // flying
          }
        }
        else
        {
          if (acc_mag_mc <= 12.5)
          {
            return 0; // flying
          }
          else
          {
            return 1; // others
          }
        }
      }
    }
    else
    {
      if (acc_mag_kurtosis <= 3.6376465559005737)
      {
        if (acc_mag_mc <= 8.5)
        {
          if (acc_mag_var <= 1.1520787477493286)
          {
            return 0; // flying
          }
          else
          {
            return 1; // others
          }
        }
        else
        {
          if (acc_mag_var <= 0.3593991547822952)
          {
            return 1; // others
          }
          else
          {
            return 0; // flying
          }
        }
      }
      else
      {
        if (acc_mag_mean <= 1.4227893948554993)
        {
          if (acc_mag_mean <= 0.7261419892311096)
          {
            return 0; // flying
          }
          else
          {
            return 1; // others
          }
        }
        else
        {
          if (acc_mag_kurtosis <= 5.189692258834839)
          {
            return 1; // others
          }
          else
          {
            return 1; // others
          }
        }
      }
    }
  }
}
