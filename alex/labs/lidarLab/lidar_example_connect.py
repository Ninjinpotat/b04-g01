import serial.tools
import serial.tools.list_ports
from lidar.alex_lidar import  BAUDRATE, lidarConnect, lidarDisconnect, process_scan, lidarStatus


PORT = "/dev/ttyUSB0"   # Please change this to the correct port for your device if necessary
BAUDRATE = 115200
def TestLidarConnection():
    """
    Main function to demonstrate the usage of the LiDAR device.
    """
    # Establish a connection to the LiDAR device
    # print all serial ports
    print("==========Starting LiDAR Connection Test==========")
    serial_ports = serial.tools.list_ports.comports()
    print("Printing available serial ports:")
    for port, desc, hwid in serial_ports:
        print(f"  {port}: {desc} [{hwid}]")
    print("...")
    print("Connecting to LiDAR device on port", PORT)

    lidar = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
    print("Connected to LiDAR device on port", PORT)
    print("...")

    # Retrieve and print the LiDAR device information
    print("Retrieving LiDAR device information...")
    status = lidarStatus(lidar, verbose=True)
    print("...")

    # Disconnect from the LiDAR device
    print("Disconnecting from LiDAR device...")
    lidarDisconnect(lidar)
    print("Disconnected!")

    print("==========LiDAR Connection Test Complete==========")

if __name__ == "__main__":
    TestLidarConnection()

