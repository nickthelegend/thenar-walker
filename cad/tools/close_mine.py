"""Close only Thenar Walker documents in the running SolidWorks (other projects stay open)."""
import swlib
from swlib import cast, sw
import design as D

def close_mine(verbose=True):
    s = sw()
    root = D.ROOT.lower()
    closed = 0
    for _ in range(3):   # assemblies first, then their now-unreferenced parts
        for d in list(s.GetDocuments() or []):
            try:
                m = cast(d, 'IModelDoc2')
                p = (m.GetPathName() or '').lower()
            except Exception:
                continue
            if p.startswith(root) or p == '':
                if p == '' and 'thenar' not in (m.GetTitle() or '').lower() and not (m.GetTitle() or '').startswith(('Assem', 'Part')):
                    continue
                s.CloseDoc(m.GetTitle()); closed += 1
    if verbose:
        print('closed', closed, 'Thenar Walker docs')

if __name__ == '__main__':
    close_mine()
