class MockBlueTooth:
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

    class BluetoothSocket:
        def __init__(self, *args, **kwargs):
                pass

        class ClientSocket:
            def __init__(self, *args, **kwargs):
                pass

            def setblocking(self, *args):
                pass

            def settimeout(self, *args):
                pass
                
            def recv(self, *args):
                return 'areallylongstring'
                
            def send(self, *args):
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


class MockXbox:
    def __init__(self, *args, **kwargs):
        pass

    class Joystick:
        pass

        def dpadUp(*args):
            return True
        
        def dpadDown(*args):
            return True
        
        def dpadLeft(*args):
            return True
        
        def dpadRight(*args):
            return True


class MockOs:
    def __init__(self, *args, **kwargs):
            pass

    def _get_exports_list(self, *args, **kwargs):
        return []

    @property
    def name(self):
            pass

    class path:
        def __init__(self, *args, **kwargs):
            pass

        def exists(self, *args):
            return True


class MockSocket:
        def __init__(self, *args, **kwargs):
            pass
        
        class AF_UNIX:
            pass

        class SOCK_DGRAM:
            pass
        
        class socket:
            def __init__(self, *args, **kwargs):
                    pass