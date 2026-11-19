"""Manufacturer's planar pixel-to-arm mapping, guarded by explicit bounds."""
from dataclasses import dataclass
from math import isfinite
from typing import Sequence, Tuple


@dataclass(frozen=True)
class PlanarCalibration:
    kx: float
    bx: float
    ky: float
    by: float

    @classmethod
    def from_values(cls, values: Sequence[float]):
        if len(values) != 4:
            raise ValueError("calibration requires exactly four coefficients")
        values = tuple(float(v) for v in values)
        if not all(isfinite(v) for v in values) or abs(values[0]) < 1e-9 or abs(values[2]) < 1e-9:
            raise ValueError("invalid or degenerate calibration coefficients")
        return cls(*values)

    def project(self, pixel_x: float, pixel_y: float,
                bounds: Tuple[float, float, float, float] = None) -> Tuple[float, float]:
        if not all(isfinite(v) for v in (pixel_x, pixel_y) + (tuple(bounds) if bounds is not None else ())):
            raise ValueError("non-finite pixel or arm bounds")
        if bounds is not None and (bounds[0] >= bounds[1] or bounds[2] >= bounds[3]):
            raise ValueError("invalid arm bounds")
        x = self.kx * pixel_y + self.bx
        y = self.ky * pixel_x + self.by
        if not isfinite(x) or not isfinite(y):
            raise ValueError('non-finite projected arm point')
        if bounds is not None and not (bounds[0] <= x <= bounds[1] and bounds[2] <= y <= bounds[3]):
            raise ValueError("projected point is outside configured arm bounds")
        return x, y
