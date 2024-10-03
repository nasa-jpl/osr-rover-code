import logging


class Roboclaw:
    def __init__(self, comport, rate, timeout=0.01, retries=3):
        self.comport = comport
        self.rate = rate
        self.timeout = timeout
        self.retries = retries
        self.logger = logging.getLogger("MockRoboClaw")
        self.logger.setLevel(logging.DEBUG)

    def Open(self):
        self.logger.info(f"Opening connection to {self.comport} at {self.rate} baud")
        return True

    def ReadVersion(self, address):
        self.logger.info(f"Reading version for address {address}")
        return (b"Mock RoboClaw v1.0", True)

    def ForwardM1(self, address, speed):
        self.logger.info(f"ForwardM1: address={address}, speed={speed}")
        return True

    def BackwardM1(self, address, speed):
        self.logger.info(f"BackwardM1: address={address}, speed={speed}")
        return True

    def ForwardM2(self, address, speed):
        self.logger.info(f"ForwardM2: address={address}, speed={speed}")
        return True

    def BackwardM2(self, address, speed):
        self.logger.info(f"BackwardM2: address={address}, speed={speed}")
        return True

    def ForwardBackwardM1(self, address, speed):
        self.logger.info(f"ForwardBackwardM1: address={address}, speed={speed}")
        return True

    def ForwardBackwardM2(self, address, speed):
        self.logger.info(f"ForwardBackwardM2: address={address}, speed={speed}")
        return True

    def ForwardMixed(self, address, speed):
        self.logger.info(f"ForwardMixed: address={address}, speed={speed}")
        return True

    def BackwardMixed(self, address, speed):
        self.logger.info(f"BackwardMixed: address={address}, speed={speed}")
        return True

    def TurnRightMixed(self, address, speed):
        self.logger.info(f"TurnRightMixed: address={address}, speed={speed}")
        return True

    def TurnLeftMixed(self, address, speed):
        self.logger.info(f"TurnLeftMixed: address={address}, speed={speed}")
        return True

    def ReadEncM1(self, address):
        self.logger.info(f"ReadEncM1: address={address}")
        return (1, 0, 0)  # Mocked encoder value

    def ReadEncM2(self, address):
        self.logger.info(f"ReadEncM2: address={address}")
        return (1, 0, 0)  # Mocked encoder value

# we can add more as needed I think this works for the inital tests