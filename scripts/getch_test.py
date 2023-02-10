import time
import signal

from readchar import readkey, key

def handler(signum, frame):
    raise Exception()

signal.signal(signal.SIGALRM, handler)

i = 0
while(True):

    char = " "
    try:
        signal.setitimer(signal.ITIMER_REAL, 0.2)
        time.sleep(0.05)
        char = readkey()
    except KeyboardInterrupt:
        raise KeyboardInterrupt
    except Exception as err:
        print(err)
        print("timeout")


    if(char == '['):
        i -= 1
    elif(char == ']'):
        i += 1

    print(i)