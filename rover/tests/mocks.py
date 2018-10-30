class MockBlueTooth():
    def __init__(self, *args, **kwargs):
            pass

    def advertise_service(*args, **kwargs):
            pass

    class RFCOMM:
            pass

    class PORT_ANY:
            pass

    class SERIAL_PORT_CLASS:
            pass

    class SERIAL_PORT_PROFILE:
            pass

    class BluetoothSocket():
            def __init__(self, *args, **kwargs):
                    pass

            class ClientSocket:
                def setblocking(*args):
                        pass
                def settimeout(*args):
                        pass

            class ClientInfo:
                pass
        
            def bind(self, *args):
                    pass
            
            def listen(self, *args):
                    pass
        
            def getsockname(self, *args):
                    return [0, 0]
        
            def accept(self, *args):
                    return  self.ClientSocket, self.ClientInfo
        

class MockXbox():
    def __init__(self, *args, **kwargs):
            pass

    class Joystick():
        pass