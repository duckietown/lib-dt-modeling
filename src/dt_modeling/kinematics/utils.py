def trim_value(value, low, high):
    """
    Trims a value to be between some bounds.

    Args:
        value: the value to be trimmed
        low: the minimum bound
        high: the maximum bound

    Returns:
        the trimmed value
    """
    return max(min(value, high), low)
