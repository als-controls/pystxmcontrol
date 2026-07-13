"""EPICS IOC layer for pystxmcontrol devices (caproto-based).

Leaf subpackage: nothing here is imported by the base pystxmcontrol package.
Install the extra to use it:  pip install pystxmcontrol[iocs]
"""


def require_caproto():
    """Import and return caproto, with an actionable error if missing."""
    try:
        import caproto
    except ImportError as exc:  # pragma: no cover
        raise ImportError(
            "pystxmcontrol.iocs requires caproto. "
            "Install with: pip install pystxmcontrol[iocs]"
        ) from exc
    return caproto
