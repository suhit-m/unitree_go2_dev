import platform
import ctypes

def LoadNDIRestServerLib():
    OS = platform.system()
    LIB_BIN_PATH = "../NDIRestServer/bin/"

    if OS == "Windows":
        libFile = LIB_BIN_PATH + "NDIRestServer.dll"
    elif OS == "Linux":
        libFile = LIB_BIN_PATH + "libNDIRestServer.so"
    elif OS == "Darwin":
        libFile = LIB_BIN_PATH + "libNDIRestServer.dylib"
    else:
        print("Unsupported OS!")
        return []

    return ctypes.cdll.LoadLibrary(libFile)

class NDIImageSender:
    def __init__(self, sender_name_bytes, sender_ms):
        # load the library
        self.libNDIRestServer = LoadNDIRestServerLib()
        if self.libNDIRestServer == []:
            print("Failed to load the lib OS!")
            return

        # create an NDISender object
        self.libNDIRestServer.NDIImageSender_create.restype = ctypes.c_void_p
        self.NDISender = self.libNDIRestServer.NDIImageSender_create(sender_name_bytes, ctypes.c_int(sender_ms))

    def send_image(self, img_bytes, xres, yres):
        self.libNDIRestServer.NDIImageSender_setImage(ctypes.c_void_p(self.NDISender), ctypes.c_char_p(img_bytes), ctypes.c_int(xres), ctypes.c_int(yres))

    def __del__(self):
        # delete the sender object
        self.libNDIRestServer.NDIImageSender_delete(ctypes.c_void_p(self.NDISender))