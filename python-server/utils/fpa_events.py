# fpa_events.py
# Host-side Foot-Flat detection (sections 3.1–3.4) for events.

G_DEFAULT = 9.81  # m/s²

class FootFlatDetector:
    """
    Stub foot-flat detector for compatibility.
    Real FPA detection is now in fpa_algorithm.py
    """
    
    def __init__(self, **kwargs):
        """Initialize with any parameters (ignored)."""
        pass
    
    def update(self, t_ms, ax, ay, az, gx, gy, gz):
        """
        Process one sample.
        
        Returns:
            Empty list (no events detected in stub)
        """
        return []


__all__ = ['FootFlatDetector', 'G_DEFAULT']
