import sys

from roboclaw_movemotor import test_connection


if __name__ == "__main__":
    
    address = int(sys.argv[1]) 
    
    rc = test_connection(address)

    # for RC1 (128): 0x80a3
    # rc.SetConfig(address, 32931)
    # for RC2 (129): 0x81a3
    # rc.SetConfig(address, 33187)
    # for RC3 (130): 0x82a3
    # rc.SetConfig(address, 33443)

    config = rc.GetConfig(address)[1]
    print(f"Config (int): {config}, (hex) {hex(config)}")

    # rc.WriteNVM(address)

    