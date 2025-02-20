# READ ME

## Importing modules
This solution uses two libraries from the standard Python libraries.  
The VEX VR platform doesn't save these imports when you save the .vrpython project file, so if you try to open it, the program will error and crash.  
To prevent this, you will need to add the following code to the top of maze_solver.vrpython :  
```
from collections import deque
import heapq
```

The top of the file should look like:

```
# region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vr import *
from collections import deque
import heapq

```