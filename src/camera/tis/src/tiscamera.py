import os
import subprocess
from collections import namedtuple
import gi
from random import Random

gi.require_version("Gst", "1.0")
gi.require_version("Tcam", "0.1")

from gi.repository import Tcam, Gst, GLib, GObject

DeviceInfo = namedtuple("DeviceInfo", "status name identifier connection_type")
CameraProperty = namedtuple("CameraProperty", "status value min max default step type flags category group")

# Disable pylint false positives
# pylint:disable=E0712

def random_str(randomlength=8):
    str=''
    chars='AaBbCcDdEeFfGgHhIiJjKkLlMmNnOoPpQqRrSsTtUuVvWwXxYyZz0123456789'
    length=len(chars)-1
    random=Random()
    for i in range(randomlength):
        str+=chars[random.randint(0, length)]
    return str


class Camera:
    """"""
    def __init__(self, serial, width, height, framerate, cam_name, camera_info_url):
        """ Constructor.
        Creates the sink pipeline and the source pipeline.

        :param serial: Serial number of the camera to use.
        :param width: Width of the video format, e.g. 640, 1920 etc,
        :param height: Height of the video format, e.g. 480, 1080
        :param framerate: Numerator of the frame rate, e.g. 15, 30, 60 etc
        :param color: If True, color is used, else gray scale
        :param liveview: If True an own live window is opened.
        """
        Gst.init([])
        self.height = height
        self.width = width
        self.sample = None
        self.samplelocked = False
        self.newsample = False
        self.pid = -1
        self.camera_name = cam_name
        self.camera_info_url = camera_info_url
        self.randshm = random_str(8)


        self.__remove_tmp_file()

        pixelformat = "BGRx"
        color = True
        liveview = False
        if not color:
            pixelformat = "GRAY8"

        if liveview:
            p = 'tcambin serial="%s" name=source ! video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1' % (serial, pixelformat, width, height, framerate,)
            p += ' ! tee name=t'
            p += ' t. ! queue ! videoconvert ! video/x-raw,format=RGB ,width=%d,height=%d,framerate=%d/1! shmsink socket-path=/tmp/tiscamera_%s'  % (width, height, framerate, self.randshm)
            p += ' t. ! queue ! videoconvert ! ximagesink'
        else:
            p = 'tcambin serial="%s" name=source ! video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1' % (
            serial, pixelformat, width, height, framerate,)
            p += ' ! videoconvert ! video/x-raw,format=RGB ,width=%d,height=%d,framerate=%d/1! shmsink socket-path=/tmp/tiscamera_%s'  % (width, height, framerate, self.randshm)

        # print(p)

        try:
            self.pipeline = Gst.parse_launch(p)
        except GLib.Error as error:
            raise RuntimeError("Error creating pipeline: {0}".format(error))


        self.pipeline.set_state(Gst.State.READY)
        if self.pipeline.get_state(10 * Gst.SECOND)[0] != Gst.StateChangeReturn.SUCCESS:
            raise RuntimeError("Failed to start video stream.")
        # Query a pointer to our source, so we can set properties.
        self.source = self.pipeline.get_by_name("source")

        # Create gscam_config variable with content
        gscam = 'shmsrc socket-path=/tmp/tiscamera_%s ! video/x-raw-rgb, width=%d,height=%d,framerate=%d/1' % (self.randshm, width, height, framerate, )
        gscam += ',bpp=24,depth=24,blue_mask=16711680, green_mask=65280, red_mask=255 ! ffmpegcolorspace'
        os.environ["GSCAM_CONFIG"] = gscam
        # print (gscam)

    def start_pipeline(self):
        """ Starts the camera sink pipeline and the rosrun process

        :return:
        """
        try:
            self.pipeline.set_state(Gst.State.PLAYING)
            # self.pid = subprocess.Popen(["rosrun", "gscam", "gscam", "_camera_info_url:=file://%s" % (self.camera_info_url), "camera_name:=%s" % (self.camera_name), "camera/image_raw:=%s/image_raw" % (self.camera_name), "_frame_id:=/%s_frame" % (self.camera_name), "__name:=%s_node" % (self.camera_name)])
            self.pid = subprocess.Popen(["rosrun", "gscam", "gscam", "_camera_info_url:=file://%s" % (self.camera_info_url), "camera_name:=", "camera/image_raw:=image_raw", "_frame_id:=/%s_frame" % (self.camera_name), "__name:=%s_node" % (self.camera_name)])

        except GLib.Error as error:
            print("Error starting pipeline: {0}".format(error))
            raise

    def stop_pipeline(self):
        """ Stops the camera pipeline. Should also kill the rosrun process, but is not implemented

        :return:
        """
        self.pipeline.set_state(Gst.State.PAUSED)
        self.pipeline.set_state(Gst.State.READY)
        self.pipeline.set_state(Gst.State.NULL)
        self.pid.kill()

    def list_properties(self):
        """ Helper function. List available properties

        :return:
        """
        for name in self.source.get_tcam_property_names():
            print(name)

    def get_property(self, property_name):
        """ Return the value of the passed property.

        Use list_properties for querying names of available properties.

        :param property_name: Name of the property, e.g. Gain, Exposure, Gain Auto.
        :return: Current value of the property.
        """
        try:
            return CameraProperty(*self.source.get_tcam_property(property_name))
        except GLib.Error as error:
            raise RuntimeError("Error get Property {0}: {1}", property_name, format(error))

    def set_property(self, property_name, value):
        """ Set a property. Use list_properties for querying names of available properties.

        :param property_name: Name of the property, e.g. Gain, Exposure, Gain Auto.
        :param value: Value to be set.
        :return:
        """
        try:
            self.source.set_tcam_property(property_name, value)
        except GLib.Error as error:
            raise RuntimeError("Error set Property {0}: {1}", property_name, format(error))

    def push_property(self, property_name):
        """ Simplify push properties, like Auto Focus one push

        :param property_name: Name of the property to be pushed
        :return:
        """
        try:
            self.source.set_tcam_property(property_name, True)

        except GLib.Error as error:
            raise RuntimeError("Error set Property {0}: {1}", property_name, format(error))

    def __remove_tmp_file(self):
        """ Delete the memory file used by the pipelines to share memory

        :return:
        """
        try:
            os.remove('/tmp/tiscamera_%s' % (self.randshm))
        except OSError:
            pass
