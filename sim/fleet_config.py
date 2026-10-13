"""Persistent fleet selection shared by the viewer and episode runner."""
import json
from pathlib import Path

COUNTS = (1, 2, 4)


def validate_config(value):
    if not isinstance(value, dict) or type(value.get('count')) is not int or value['count'] not in COUNTS:
        raise ValueError('count must be 1, 2 or 4')
    focus = value.get('focus', 0)
    if type(focus) is not int or not 0 <= focus < value['count']:
        raise ValueError('invalid focused drone')
    return {'count': value['count'], 'focus': focus}


def read_config(live_dir):
    try:
        return validate_config(json.loads((Path(live_dir)/'fleet.json').read_text()))
    except (OSError, ValueError, TypeError):
        return {'count': 1, 'focus': 0}
