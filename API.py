import sys
import io

# Fix for Windows encoding issue - add this at the top
if sys.platform == "win32":
    # Force stdout to use UTF-8
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

class MouseCrashedError(Exception):
    pass

def command(args, return_type=None):
    line = " ".join([str(x) for x in args]) + "\n"
    sys.stdout.write(line)
    sys.stdout.flush()
    if return_type:
        response = sys.stdin.readline().strip()
        if return_type == bool:
            return response == "true"
        return return_type(response)

def mazeWidth():
    return command(args=["mazeWidth"], return_type=int)

def togglePause():
    return command(args=["togglePause"], return_type=str)

def mazeHeight():
    return command(args=["mazeHeight"], return_type=int)

def checkWall(wallCommand, half_steps_away=None):
    args = [wallCommand]
    if half_steps_away is not None:
        args.append(half_steps_away)
    return command(args, return_type=bool)

def wallFront(half_steps_away=None):
    return checkWall("wallFront", half_steps_away)

def wallBack(half_steps_away=None):
    return checkWall("wallBack", half_steps_away)

def wallLeft(half_steps_away=None):
    return checkWall("wallLeft", half_steps_away)

def wallRight(half_steps_away=None):
    return checkWall("wallRight", half_steps_away)

def wallFrontLeft(half_steps_away=None):
    return checkWall("wallFrontLeft", half_steps_away)

def wallFrontRight(half_steps_away=None):
    return checkWall("wallFrontRight", half_steps_away)

def wallBackLeft(half_steps_away=None):
    return checkWall("wallBackLeft", half_steps_away)

def wallBackRight(half_steps_away=None):
    return checkWall("wallBackRight", half_steps_away)

def moveForward(distance=None):
    args = ["moveForward"]
    # Don't append distance argument unless explicitly specified, for
    # backwards compatibility with older versions of the simulator
    if distance is not None:
        args.append(distance)
    response = command(args=args, return_type=str)
    if response == "crash":
        raise MouseCrashedError()

def moveForwardHalf(num_half_steps=None):
    args = ["moveForwardHalf"]
    if num_half_steps is not None:
        args.append(num_half_steps)
    response = command(args=args, return_type=str)
    if response == "crash":
        raise MouseCrashedError()

def turnRight():
    command(args=["turnRight"], return_type=str)

def turnLeft():
    command(args=["turnLeft"], return_type=str)

def turnRight90():
    turnRight()

def turnLeft90():
    turnLeft()

def turnRight45():
    command(args=["turnRight45"], return_type=str)

def turnLeft45():
    command(args=["turnLeft45"], return_type=str)

def setWall(x, y, direction):
    command(args=["setWall", x, y, direction])

def clearWall(x, y, direction):
    command(args=["clearWall", x, y, direction])

def setColor(x, y, color):
    command(args=["setColor", x, y, color])

def clearColor(x, y):
    command(args=["clearColor", x, y])

def clearAllColor():
    command(args=["clearAllColor"])

def setText(x, y, text):
    # Ensure text is properly encoded for Windows
    if isinstance(text, str) and sys.platform == "win32":
        # Try to encode as UTF-8, fallback to ASCII if needed
        try:
            text.encode('utf-8')
        except UnicodeEncodeError:
            # Replace unsupported characters
            text = text.encode('ascii', errors='replace').decode('ascii')
    command(args=["setText", x, y, text])

def clearText(x, y):
    command(args=["clearText", x, y])

def clearAllText():
    command(args=["clearAllText"])

def wasReset():
    return command(args=["wasReset"], return_type=bool)

def ackReset():
    command(args=["ackReset"], return_type=str)