class RadioSim(object):
    """simulates a radio"""

    def __init__(self, station_conn):
        super(RadioSim, self).__init__()
        self.station_conn = station_conn

    def receive(self):
        if self.station_conn.poll():
            return self.station_conn.recv()
        return None
