# How a single sensor channel should be corrupted.

uint8 MODE_HEALTHY=0    # pass measurements straight through
uint8 MODE_DROPOUT=1    # publish nothing at all (GNSS also gets STATUS_NO_FIX)
uint8 MODE_DEGRADE=2    # extra noise and inflated reported covariance
uint8 MODE_BIAS_RAMP=3  # slowly growing offset, i.e. a plausible spoof
uint8 MODE_SLIP=4       # report only a fraction of the true motion

# Channel name as configured on the injector, e.g. "gnss", "imu", "wheel".
string channel
uint8 mode

# MODE_DEGRADE
float64 noise_stddev      # extra white noise, in the channel's native units
float64 covariance_scale  # multiplies the covariance the sensor reports

# MODE_BIAS_RAMP
float64 bias_rate         # native units per second, added cumulatively

# MODE_SLIP
float64 slip_ratio        # reported motion / true motion; 1.0 is no slip,
                          # below 1.0 under-reports, above 1.0 is a free-spinning wheel

# Zero means "until explicitly cleared".
builtin_interfaces/Duration duration
