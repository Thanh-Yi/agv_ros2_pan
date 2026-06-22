import yaml

class Config:
    def __init__(self, path):
        with open(path, "r") as f:
            self.cfg = yaml.safe_load(f)

    def reg(self, name):
        r = self.cfg["canopen"]["registers"][name]
        return r["index"], r["sub"]

    def get(self, *keys):
        val = self.cfg
        for k in keys:
            val = val[k]
        return val