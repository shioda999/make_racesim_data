from glob import glob
try:
    from msvcrt import getch
    win = True
except ImportError:
    import sys
    import tty
    import termios
    def getch():
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                return sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
    win = False

def get_key():
    EOT = 3
    TAB = 9
    ESC = 27
    DEL = 8 if win else 46
    RETURN = 13
    UP = 72 if win else ord('A')
    DOWN = 80 if win else ord('B')
    LEFT = 75 if win else ord('C')
    RIGHT = 77 if win else ord('D')
    key = ord(getch())
    if key == EOT:
        return 'EOT'
    elif key == TAB:
        return 'TAB'
    elif key == RETURN:
        return 'RETURN'
    elif key == DEL:
        return 'DEL'
    elif key == ESC:
        if not win:
            key = ord(getch())
            if key is None: return ESC
            if key == ord('['):
                key = "CURSOR_KEY"
    elif key == 224 and win: key = "CURSOR_KEY"
    else:
        return chr(key)
    if key == "CURSOR_KEY":
        key = ord(getch())
        if key == UP:
            return 'UP'
        elif key == DOWN:
            return 'DOWN'
        elif key == LEFT:
            return 'LEFT'
        elif key == RIGHT:
            return 'RIGHT'

def select_file(fname, n=1, disp=9):
    files = glob(fname)
    files.sort()
    files = files[::-1]
    print('Please select file.')
    p = 0
    file_n = len(files)
    disp = min(disp, file_n)
    selected = []
    while True:
        s = max(0, min(p - disp // 2, file_n - disp))
        for i in range(s, s + disp):
            print('\033[K', end='')
            print(('⇨' if p == i else ('+  ' if i in selected else '   ')) + files[i] + '   ')
        key = get_key()
        if key == 'UP':
            p -= 1
        elif key == 'DOWN':
            p += 1
        elif key == 'RETURN':
            if p not in selected:
                selected.append(p)
                if len(selected) == n: break
        elif key == 'DEL':
            selected.pop()
        elif key == 'EOT':
            exit()
        p = (p + file_n) % file_n
        print('\033[%dA' % disp, end='')
    if n == 1: return files[selected[0]]
    return [files[i] for i in selected]