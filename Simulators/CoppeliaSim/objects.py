from settings import Settings

class CoppeliaObj:
    def __init__(self, sim, obj: str) -> None:
        Settings.Log(f'Init Coppelia {obj}...')
        self.sim = sim