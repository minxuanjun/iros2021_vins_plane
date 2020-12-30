from . import pycodestyle

if __name__ == '__main__':
    pycodestyle.MAX_LINE_LENGTH = 120
    pycodestyle._main()
