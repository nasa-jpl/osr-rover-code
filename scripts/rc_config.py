import sys

from roboclaw_movemotor import test_connection


if __name__ == "__main__":
    
    address = int(sys.argv[1]) 
    
    rc = test_connection(address)

    # rc.SetConfig(address, 32931)

    config = rc.GetConfig(address)[1]
    print(f"Config (int): {config}, (hex) {hex(config)}")

    rc.WriteNVM(address)

    