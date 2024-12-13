import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nicklas/Documents/DAS_kandidate/IDT_SilvanDrone/module10_mission_upload/install/idt'
