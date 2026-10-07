"""PNG renders of Thenar_Walker.SLDASM in every configuration (Z-up camera)."""
import os
import sys

import swlib
from swlib import cast, sw
import design as D
import build_robot as BR

VIEWS = {'iso': (1.0, -1.25, 0.9), 'iso_rear': (-1.0, 1.1, 0.8), 'side': (0.0, -1.0, 0.0), 'front': (1.0, 0.0, 0.0), 'top': (0.0001, 0.0, 1.0)}


def open_asm(path=BR.ASM_PATH):
    r = sw().OpenDoc6(path, 2, 1, '', 0, 0)
    m = cast(r[0] if isinstance(r, tuple) else r, 'IModelDoc2')
    sw().ActivateDoc3(m.GetTitle(), False, 0, 0)
    return m


def shot(m, fname, eye):
    swlib.hide_refs(m)
    swlib.zup_view(m, eye)
    m.ViewZoomtofit2()
    m.GraphicsRedraw2()
    os.makedirs(D.RENDERS, exist_ok=True)
    m.SaveAs3(os.path.join(D.RENDERS, fname), 0, 3)


if __name__ == '__main__':
    m = open_asm()
    configs = sys.argv[1:] or list(D.POSES)
    for c in configs:
        m.ShowConfiguration2(c)
        m.ForceRebuild3(False)
        for v in (['iso', 'side', 'front', 'top', 'iso_rear'] if c == 'STOW' else ['iso', 'side']):
            shot(m, f'robot_{c}_{v}.png', VIEWS[v])
    m.ShowConfiguration2('STOW')
    print('renders in', D.RENDERS)
