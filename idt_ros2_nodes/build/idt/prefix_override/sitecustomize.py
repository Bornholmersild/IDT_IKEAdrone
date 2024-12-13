import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nicklas/Documents/DAS_kandidate/IDT_SilvanDrone/idt_ros2_nodes/install/idt'
