def _axes_dimensions(ax):
    """
    Dimensions of axes

    :param ax: axes
    :type ax: Axes3DSubplot or AxesSubplot
    :return: dimensionality of axes, either 2 or 3
    :rtype: int
    """
    classname = ax.__class__.__name__

    if classname in ("Axes3D", "Axes3DSubplot", "Animate"):
        return 3
    elif classname in ("AxesSubplot", "Animate2"):
        return 2
    
        if hasattr(self, "ikine_a"):
            ik = self.ikine_a
        else:
            ik = self.ikine_LMS