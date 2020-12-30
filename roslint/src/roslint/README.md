For overall stability and to avoid a packaging headache, the linters in use are
vendored into this packages.

python
======

The python linter is `pycodestyle` (nee `pep8`), and can be downloaded from here:

    wget https://raw.githubusercontent.com/PyCQA/pycodestyle/master/pycodestyle.py -O pycodestyle.py

C++
===

The C++ linter is `cpplint.py` which is downloaded from here:

    wget https://raw.githubusercontent.com/google/styleguide/gh-pages/cpplint/cpplint.py -O cpplint.py

However, the current master branch version of cpplint does not run cleanly under Python 3, so
a small patch has been applied to resolve this based on this pull request:

https://github.com/google/styleguide/pull/349
