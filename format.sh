#!/bin/bash
find . -maxdepth 1 -type f \( -name "*.cpp" -o -name "*.hpp" \) | xargs clang-format -i -style=Chromium
find ./python -name "*py"|xargs python3 -m autoflake -i --remove-all-unused-imports --remove-unused-variables --ignore-init-module-imports
python3 -m isort ./python
python3 -m black --required-version 22.6.0 ./python
python3 -m flake8 ./python
