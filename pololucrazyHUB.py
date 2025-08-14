import time
import sys
import paho.mqtt.client as mqtt

'''
TODOs:

'''

# Crazyflie imports\import cflib.crtp
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander
cflib.crtp.init_drivers()
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
import cflib.crtp

#Dictionary initionation
pololu_status_dict = {}
pololu_coord_dict = {'00':(0,0),'01':(0,0),'02':(0,0),'03':(0,0)}
crazy_coord_dict = {}
alert_dict = {} #put type as key (object or jewel) and coordinate as value ex: 02/bingo:(x,y)
crazy_batt_dict = {}
crazy_coord_dict = {}

# MQTT topics
BROKER_IP = "192.168.1.10"
pololu_alert = ['00/alert','01/alert','02/alert','03/alert'] #object or jewel identified "jewel:(x,y)"
pololu_coord = ['00/coord','01/coord','02/coord','03/coord'] #for updating location at each intersection (x,y) in meters
pololu_status = ['00/status','01/status','02/status','03/status'] #moving, stopped, connected, disconnected
pololu_command = ['00/command','01/command','02/command','03/command'] #stop, location to move to, calibrate

# Crazyflie configuration
crazy_URIs = []
active_drone_dict = {}
TAKEOFF_HEIGHT = 1
TAKEOFF_TIME = 3.0
LAND_TIME = 3.0
POINT_TO_POINT_TIME = 5.0
WAYPOINTS = [
    (.5, 1, .75),
    (.75, .25, .75),
]

def write_log_info(info, filename):
    '''
    takes a list of strings for info and txt file for filename
    '''
    f = open(filename, "a")
    f.write(info + "\n")
    f.close()

def crazyflie_URI_scan():
    ''' returns list of crazyflie URIs in the area'''
    available = cflib.crtp.scan_interfaces()
    uri_list = []
    for uri in available:
        uri_list += uri
    print('Found crazies: ' + str(uri_list))
    return uri_list
    

# Enable high level commands on Crazyflie
def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')

# Function to handle incoming Pololu status messages
def on_pololu_msg(client, userdata, msg):
    payload = msg.payload.decode('utf-8')
    #split MQTT topic into ID and message type
    robot_id,msg_type = msg.topic.split('/')
    if 'alert' in msg_type:
        alert, alert_coord = payload.split(':')
        if 'jewel' in alert:
            #if jewel found robot continues on but location is logged
            alert_dict[str(robot_id)+'/jewel'] = alert_coord
            pololu_coord_dict[robot_id] = alert_coord
            write_log_info(str(robot_id)+','+str(alert)+','+str(alert_coord), 'pololu_log.txt')
        elif 'bingo' in alert:
            #if object found all bots stop and location is logged
            for topic in pololu_command:
                hub.publish(topic,'stop')
            alert_dict[str(robot_id)+'/bingo'] = alert_coord
            write_log_info(str(robot_id)+','+str(alert)+','+str(alert_coord), 'pololu_log.txt')
            #TS
            print('alert dict updated with Bingo')
    elif 'coord' in msg_type:
        #if coordinate recieved, robots location is updated in dict
        pololu_coord_dict[robot_id] = payload
        write_log_info(str(robot_id)+','+str(payload), 'pololu_log.txt')
        #TS
        print('dict updated: '+ str(robot_id) + str(payload))
    elif 'status' in msg_type:
        pololu_status_dict[robot_id] = payload
        write_log_info(str(robot_id)+','+str(payload), 'pololu_log.txt')
        #TS
        print('dict updated: '+ str(robot_id) + str(payload))

# Set up MQTT client
def setup_mqtt():
    hub = mqtt.Client()
    hub.on_message = on_pololu_msg
    hub.connect(BROKER_IP, 1883, 60)
    for topic in pololu_coord:
        hub.subscribe(topic)
    for topic in pololu_status:
        hub.subscribe(topic)
    for topic in pololu_alert:
        hub.subscribe(topic)
    hub.loop_start()
    return hub

def crazy_battery_callback(uri):
    def callback(timestamp, data, logconf):
        crazy_batt_dict[uri] = (data['pm.vbat'], timestamp)
        write_log_info(str(uri)+', '+str(data)+', '+str(timestamp), 'crazy_battery.txt')
    return callback

def crazy_position_callback(uri):
    def callback(timestamp, data, logconf):
        crazy_coord_dict[uri] = (
            data['kalman.stateX'],
            data['kalman.stateY'],
            data['kalman.stateZ'],
            timestamp
        )
        write_log_info(str(uri)+', '+str(data['kalman.stateX'])+', '+str(data['kalman.stateY'])+', '+str(data['kalman.stateZ'])+', '+str(timestamp), 'crazy_position.txt')
    return callback

if __name__ == '__main__':

    # Start MQTT
    hub = setup_mqtt()

    # Command all Pololus to calibrate
    for topic in pololu_command:
        hub.publish(topic, "calibrate")
        write_log_info('All calibrating', 'pololu_log.txt')
        #wait until all done
        #TS temporary patch

    while True:
        time.sleep(1)
        if all(status == 'stopped' for status in pololu_status_dict.values()):
            break

    #setup crazies 
    while not crazy_URIs:
        print('No drones found')
        time.sleep(2)
        crazy_URIs = crazyflie_URI_scan()
    for drone_uri in crazy_URIs:
        if len(drone_uri)>0:
            scf = SyncCrazyflie(drone_uri, cf=Crazyflie(rw_cache='./cache'))
            scf.open_link()
            activate_high_level_commander(scf.cf)
            commander = scf.cf.high_level_commander
            active_drone_dict[drone_uri] = (scf, commander)
            #TS
            print('Drone activaed')
            write_log_info(str(drone_uri) + ' activated', 'crazy_position.txt')

        #logging set up
        # Battery logger
        log_battery = LogConfig(name= drone_uri, period_in_ms=1000)
        log_battery.add_variable('pm.vbat', 'float')
        log_battery.data_received_cb.add_callback(crazy_battery_callback(drone_uri))

        # Position logger
        log_pos = LogConfig(name= drone_uri, period_in_ms=500)
        log_pos.add_variable('kalman.stateX', 'float')
        log_pos.add_variable('kalman.stateY', 'float')
        log_pos.add_variable('kalman.stateZ', 'float')
        log_pos.data_received_cb.add_callback(crazy_position_callback(drone_uri))

        # Register and start
        scf.cf.log.add_config(log_battery)
        log_battery.start()

        scf.cf.log.add_config(log_pos)
        log_pos.start()
        
        # Takeoff
        commander.takeoff(TAKEOFF_HEIGHT, TAKEOFF_TIME)
        #TS
        print('Takeoff message sent')

    # For each waypoint, instruct Pololu and fly
    for x, y, z in WAYPOINTS:
        # Send coordinates to Pololus
        for topic in pololu_command:
            hub.publish(topic, f"{x}/{y}")

        #troubelshooting
        print('pololu direction sent')

        # Fly to waypoint
        for uri, (scf, commander) in active_drone_dict.items():
            commander.go_to(x, y, z, 0, 3)

        #troubelshooting
        print('crazy direction sent')
        
        #troubelshooting
        print('WAITING ON POLOLU MOVEMENT')
        
        #wait for pololus to get there
        while True:
            time.sleep(1)
            if x == pololu_coord_dict[00][0] and y == pololu_coord_dict[00][2] and all(status == 'stopped' for status in pololu_status_dict.values()):
                break

    # Land
    for uri, (scf, commander) in active_drone_dict.items():
        commander.land(0.0, LAND_TIME)
        
        #troubleshooting
        print('landing crazy')

        time.sleep(LAND_TIME + 1)
        
        log_battery.stop()
        log_pos.stop()
        commander.stop()
        write_log_info(str(uri) + ' landed', 'crazy_position.txt')

#shutdown pololus
for bot in pololu_command:
    hub.publish(topic, "exit")

# Shutdown MQTT
print('shutting down')
hub.loop_stop()
hub.disconnect()
exit()