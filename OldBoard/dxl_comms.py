from src.dynamixel_sdk import * 

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

# Data Byte Length
LEN_GOAL_POSITION       = 4
LEN_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
# DXL3_ID                     = 3                 # Dynamixel#1 ID : 3
# DXL4_ID                     = 4                 # Dynamixel#1 ID : 4
# DXL5_ID                     = 5                 # Dynamixel#1 ID : 5
# DXL6_ID                     = 6                 # Dynamixel#1 ID : 6
BAUDRATE                    = 2000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM11' #'/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 2048-150           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 2048+150            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold

def initComms():
    # import sys, tty, termios
    # fd = sys.stdin.fileno()
    # old_settings = termios.tcgetattr(fd)
    # def getch():
    #     try:
    #         tty.setraw(sys.stdin.fileno())
    #         ch = sys.stdin.read(1)
    #     finally:
    #         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    #     return ch
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupSyncWrite instance
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

    # Initialize GroupSyncRead instace for Present Position
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
        
    return portHandler, packetHandler, groupSyncWrite, groupSyncRead