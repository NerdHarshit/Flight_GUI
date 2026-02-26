class FlightBuffer:

    def __init__(self):
        self.data = []

        self.last_counter = None
        self.lost_packets = 0

    def add_packet(self, packet):

        # Packet loss logic
        if self.last_counter is not None:
            expected = self.last_counter + 1
            if packet["Counter"] != expected:
                lost = packet["Counter"] - expected
                if lost > 0:
                    self.lost_packets += lost

        self.last_counter = packet["Counter"]

        self.data.append(packet)

    def get_packet_loss(self):
        return self.lost_packets

    def reset(self):
        self.__init__()