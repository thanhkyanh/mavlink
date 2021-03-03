from pymavlink import mavutil
import time

mavutil.set_dialect("mav_gps")

# create a connection to FMU
hoverGames = mavutil.mavlink_connection("/dev/ttyUSB0", baud=921600)

# wait for the heartbeat message to find the system id
hoverGames.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" %(hoverGames.target_system, hoverGames.target_component))

counter = 0

#send custom mavlink message
while(True) :
    timestamp = int(time.time() * 1e6)
    time_utc_usec = timestamp + 1613692609599954
    lat = 296603018
    lon = -823160500
    alt = 30100
    alt_ellipsoid = 30100
    s_variance_m_s = 0.3740
    c_variance_rad = 0.6737
    eph = 2.1060
    epv = 3.8470
    hdop = 0.8800
    vdop = 1.3300
    noise_per_ms = 101
    jamming_indicator = 35
    vel_m_s = 0.0420
    vel_n_m_s = 0.0370
    vel_e_m_s = 0.0200
    vel_d_m_s = -0.0570
    cog_rad = 0.3988
    timestamp_time_relative = 0
    heading = 0
    heading_offset = 0.0000
    fix_type = 4
    jamming_state = 0
    vel_ned_valid = 1
    satellites_used = 14


    hoverGames.mav.mav_gps_send(timestamp, lat, lon, alt, alt_ellipsoid, s_variance_m_s, c_variance_rad, fix_type, eph, epv, hdop, vdop, noise_per_ms, jamming_indicator, jamming_state, vel_m_s, vel_n_m_s, vel_e_m_s, vel_d_m_s, cog_rad, vel_ned_valid, timestamp_time_relative, time_utc_usec, satellites_used, heading, heading_offset)
    
    counter += 1
    print ("The custom mesage with the number %u was sent it!!!!" %(counter))

    time.sleep(1.0)
