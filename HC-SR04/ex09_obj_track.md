### OpenCV를 이용한 객체 추적

#### `getchar.py` 작성

표준 입력장치(키보드)로 부터 입력된 한 문자를 반환하는 Python 코드 `getchar.py`를 다음과 같이 작성한다.

```
import sys, select, os, msvcrt, time
  
# class for checking keyboard input
class Getchar:
    def __init__(self):
        pass
  
    def getch(self):
        if os.name == 'nt':
            timeout = 0.1
            startTime = time.time()
            while(1):
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                elif time.time() - startTime > timeout:
                    return ''

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


```



### 

#### `getchar.py` 테스트

앞서 작성한 `getchar.py` 를 테스트 하기위한 코드 `test_getchar.py`를 다음과 같이 작성한다. 

```
from getchar import Getchar

def main(args=None):
    kb = Getchar()
    key = ''
    
    while key!='Q':
    
        key = kb.getch()
        if key != '':
            print(key)
        else:
            pass
        

if __name__ == '__main__':
    main()



```



[**목차**](../README.md) 