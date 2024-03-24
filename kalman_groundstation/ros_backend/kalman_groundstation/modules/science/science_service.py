from .services.science import ScienceService
from .services.smart_probe import SmartProbeService
from .services.weight import WeightService


class Science:
    def __init__(self):
        self.sub_services = [
            ScienceService(),
            SmartProbeService(),
            WeightService()
        ]
