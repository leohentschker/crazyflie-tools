from cflib.crazyflie.log import LogConfig
from crazyflie_logger import CrazyflieLogger
import cflib
from cflib import crazyflie
from cflib import crtp

from threading import Thread
import time

# setup logging as it is a massive hassle
CrazyflieLogger.set_logging()


class DrakeFly(crazyflie.Crazyflie):

    def __init__(self, link_uri):
        """
        Override the init method to open the link
        by default
        """
        # instantiate the super class
        crazyflie.Crazyflie.__init__(self)

        # open the connection
        self.open_link(link_uri)

        # add callback on connect
        self.connected.add_callback(self.start_control)

        # add a callback for disconnect
        self.disconnected.add_callback(self.disconnect)

    @staticmethod
    def get_fly_uri():
        """
        Initialize the drivers to gain access
        to the crazyflie
        """
        cflib.crtp.init_drivers()
        
        # return the first fly we find
        return cflib.crtp.scan_interfaces()[0][0]

    @classmethod
    def construct_fly(cls):
        """
        Helper method to instantiate a fly
        with the correct uri that we find
        """
        fly_uri = cls.get_fly_uri()

        fly = cls(fly_uri)
        return fly

    def disconnect(self, *args, **kwargs):
        print "WE DISCONNECTED"

    def start_control(self, link_uri):

        self.set_data_logging()

        Thread(target=self.ramp).start()

    def ramp(self):

        # unlock startup thrust protection
        self.commander.send_setpoint(0, 0, 0, 0)

        time.sleep(1)
        end = time.time() + 10
        while time.time() < end:
            self.commander.send_setpoint(0, 0, 0, 1000)

        print "AND WE DID IT"

    def set_data_logging(self):
        """
        Method to setup logging of crazyflie data following
        the example in their docs
        master/examples/basiclog.py
        """

		# The definition of the logconfig can be made before connecting
        self.data_log = LogConfig(name='Stabilizer', period_in_ms=100)
        self.data_log.add_variable('stabilizer.roll', 'float')
        self.data_log.add_variable('stabilizer.pitch', 'float')
        self.data_log.add_variable('stabilizer.yaw', 'float')
        self.data_log.add_variable('stabilizer.yaw', 'float')
        self.data_log.add_variable('stabilizer.thrust', 'float')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        self.log.add_config(self.data_log)

        # This callback will receive the data
        self.data_log.data_received_cb.add_callback(self._stab_log_data)
        # This callback will be called on errors
        self.data_log.error_cb.add_callback(self._stab_log_error)
        # Start the logging
        self.data_log.start()

    def _stab_log_data(self, timestamp, data, logconf):
        print "WE HAVE THE DATA", data

    def _stab_log_error(self, *args, **kwargs):
        print "ERROR"


if __name__ == "__main__":
    fly = DrakeFly.construct_fly()
    time.sleep(6)
    fly.close_link()
