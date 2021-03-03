from pymavlink import mavutil
import time

# create a connection to FMU
hoverGames = mavutil.mavlink_connection("/dev/ttyUSB0", baud=921600)
# Once connected, use 'hoverGames' to get and send messages

# wait for the heartbeat message to find the system id
hoverGames.wait_heartbeat()

print("Received heartbeat message from FMUK66...")

# Get some basic information!
while True:
    try:
        print(hoverGames.recv_match().to_dict())
    except:
        pass
    time.sleep(1.0)
