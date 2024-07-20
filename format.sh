#!/bin/bash
find . -maxdepth 1 -type f \( -name "*.cpp" -o -name "*.hpp" \) | xargs clang-format -i -style=Chromium
