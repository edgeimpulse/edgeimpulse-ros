# Copyright 2024 Edge Impulse
#
# Licensed under the MIT License; you may not use this file except in
# compliance with the License. See the LICENSE file for details.
"""Ament pep257 docstring style check for the whole package."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Run pep257 across the package sources and tests."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
