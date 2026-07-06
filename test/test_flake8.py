# Copyright 2024 Edge Impulse
#
# Licensed under the MIT License; you may not use this file except in
# compliance with the License. See the LICENSE file for details.
"""Ament flake8 style check for the whole package."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Run flake8 across the package and fail on any reported error."""
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
