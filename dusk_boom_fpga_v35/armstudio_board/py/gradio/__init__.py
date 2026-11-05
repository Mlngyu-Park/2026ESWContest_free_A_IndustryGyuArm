"""gradio stand-in for the DUSK_BOOM board (the real gradio and its dependencies do not exist for
32-bit ARM and would not fit in 512 MB).  arm_studio_web.py builds its old gradio screen while it is
imported; the Pro server (arm_studio_pro.py) only uses the handler functions and the dicts that
gr.update() returns.  So every component here is an inert object that accepts any call."""

__version__ = "0-dusk-stub"


def update(**kw):
    d = {"__type__": "update"}
    d.update(kw)
    return d


class _Inert:
    def __init__(self, *a, **k):
        self.value = k.get("value")

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def __call__(self, *a, **k):
        return self

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return lambda *a, **k: self


class SelectData:
    index = None
    value = None
    selected = True

    def __init__(self, *a, **k):
        pass


class EventData(SelectData):
    pass


class Error(Exception):
    pass


def Warning(*a, **k):
    return None


def Info(*a, **k):
    return None


def __getattr__(name):          # gr.Blocks, gr.Button, gr.Row, gr.Plot, ...
    if name.startswith("__"):
        raise AttributeError(name)
    return _Inert
