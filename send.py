from pymavlink import mavutil
import time

mavutil.set_dialect("video_monitor")

# create a connection to FMU
hoverGames = mavutil.mavlink_connection("/dev/ttyUSB0", baud=921600)

# wait for the heartbeat message to find the system id
hoverGames.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" %(hoverGames.target_system, hoverGames.target_component))

counter = 0

#send custom mavlink message
while(True) :
    hoverGames.mav.video_monitor_send(
    timestamp = int(time.time() * 1e6), # time in microseconds
    info = b'Salut!',
    lat = counter,
    lon = 231234567,
    no_people = counter,
    confidence = 0.357)
    
    counter += 1
    print ("The custom mesage with the number %u was sent it!!!!" %(counter))

    time.sleep(1.0)
