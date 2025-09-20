import numpy as np

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def apply_calibration(vec, bias=None, scale=None):
    v = np.array(vec, dtype=float)
    if bias is not None:
        v = v - np.array(bias, dtype=float)
    if scale is not None:
        S = np.array(scale, dtype=float)
        if S.shape == (3,):
            v = v * S
        else:
            v = S.dot(v)
    return v
