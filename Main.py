import API
import sys
import heapq

# ---------- Logging ----------
def log(s):
    sys.stderr.write(s + "\n")
    sys.stderr.flush()

# ---------- Directions ----------
E, N, W, S = 0, 1, 2, 3
DIRS = [E, N, W, S]

DX = {
    E: 1,
    N: 0,
    W: -1,
    S: 0
}

DY = {
    E: 0,
    N: 1,
    W: 0,
    S: -1
}

LEFT = {
    E: N,
    N: W,
    W: S,
    S: E
}

RIGHT = {
    E: S,
    S: W,
    W: N,
    N: E
}

BACK = {
    E: W,
    W: E,
    N: S,
    S: N
}
# ---------- Maze State ----------
WIDTH = API.mazeWidth()
HEIGHT = API.mazeHeight()

# walls[x][y][dir] = True / False / None (unknown)
walls = [[[None for _ in range(4)] for _ in range(HEIGHT)] for _ in range(WIDTH)]




full_path = [(0, 0)]




# ---------- Target ----------
def target_cells(x=None, y=None):
    if x is not None and y is not None:
        return {(x, y)}

    # maze center
    cx = WIDTH // 2
    cy = HEIGHT // 2

    if WIDTH % 2 == 0:
        return {
            (cx-1, cy-1),
            (cx,   cy-1),
            (cx-1, cy),
            (cx,   cy)
        }

    return {(cx, cy)}


TARGETS = target_cells()

# ---------- Heuristic ----------
def heuristic(x, y):
    return min(abs(x-tx) + abs(y-ty) for tx, ty in TARGETS)

# ---------- Wall sensing ----------
def sense_walls(x, y, heading):
    for rel_dir, api_call in [
        (heading, API.wallFront),
        (LEFT[heading], API.wallLeft),
        (RIGHT[heading], API.wallRight),
        (BACK[heading], API.wallBack),
    ]:
        has_wall = api_call()
        walls[x][y][rel_dir] = has_wall

        # Also write the wall to the neighboring cell
        nx = x + DX[rel_dir]
        ny = y + DY[rel_dir]

        if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
            walls[nx][ny][BACK[rel_dir]] = has_wall
# ---------- A* ----------
def astar(start):
    sx, sy = start
    pq = []
    heapq.heappush(pq, (heuristic(sx, sy), 0, sx, sy))
    came_from = {}
    cost = {(sx, sy): 0}

    while pq:
        _, g, x, y = heapq.heappop(pq)

        if (x, y) in TARGETS:
            return reconstruct_path(came_from, (x, y))

        for d in DIRS:
            if walls[x][y][d] is True:
                continue

            nx, ny = x + DX[d], y + DY[d]
            if not (0 <= nx < WIDTH and 0 <= ny < HEIGHT): #checks if the next grid is inside the maze
                continue

            ng = g + 1
            if (nx, ny) not in cost or ng < cost[(nx, ny)]:
                cost[(nx, ny)] = ng
                f = ng + heuristic(nx, ny)
                heapq.heappush(pq, (f, ng, nx, ny))
                came_from[(nx, ny)] = (x, y)

    return None

# none / (or any another color) 
'''{
      {'k', BLACK},       {'b', BLUE},
      {'a', GRAY},        {'c', CYAN},
      {'g', GREEN},       {'o', ORANGE},
      {'r', RED},         {'w', WHITE},
      {'y', YELLOW},      {'B', DARK_BLUE},
      {'C', DARK_CYAN},   {'A', DARK_GRAY},
      {'G', DARK_GREEN},  {'O', DARK_ORANGE},
      {'R', DARK_RED},    {'V', DARK_VIOLET},
      {'Y', DARK_YELLOW},
  }
'''
fullColorMap = [[ 'none' for _ in range(HEIGHT)] for _ in range(WIDTH)]
previousePath = []

def reconstruct_path(came_from, end):
    global previousePath

    path = [end]
    while end in came_from:
        end = came_from[end]
        path.append(end)
    path.reverse()

    if len(previousePath) !=0:
        for cell in previousePath:
            x,y = cell
            if(fullColorMap[cell[0]][cell[1]] == 'none'):
                API.clearColor(x,y)
            else:
                API.setColor(x,y,fullColorMap[cell[0]][cell[1]])

    for cell in path:
        x,y = cell
        API.setColor(x,y,'c' )

    previousePath = path

    log("planned Path: " + str(path))
    return path

def reconstruct_path_from_start(came_from, start, end):
    path = [end]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


# ---------- Movement ----------
def turn_to(current, target):
    diff = (target - current) % 4

    if diff == 0:
        return current

    # Turn right
    if diff == 1:
        API.turnLeft()
        current = LEFT[current]

    # Turn left
    elif diff == 3:
        API.turnRight()
        current = RIGHT[current]

    # Opposite direction (2 turns either way)
    elif diff == 2:
        API.turnRight()
        API.turnRight()
        current = BACK[current]

    return current

def move_step(x, y, heading, nx, ny):
    for d in DIRS:
        if x + DX[d] == nx and y + DY[d] == ny:
            heading = turn_to(heading, d)
            API.moveForward()
            return nx, ny, heading

    raise RuntimeError("Invalid move")

# ---------- Main ----------
def main():

    for x in range(WIDTH):
        for y in range(HEIGHT):
            API.setText(x, y, f"{x},{y}")


    log("A* Micromouse running")
	
  
    API.setText(0,0,'😁')

    
    x, y = 0, 0
    heading = N

    API.setColor(x, y, "g")
    fullColorMap[x][y] = 'g'
    
    startedBackTracing = False

    while (x, y) not in TARGETS:
        sense_walls(x, y, heading)

        path = astar((x, y))
        if not path or len(path) < 2:
            log("No path found")
            return

        nx, ny = path[1]
        x, y, heading = move_step(x, y, heading, nx, ny)
        
    
        if (nx, ny) not in full_path:
                startedBackTracing = False
                full_path.append((nx, ny))
                API.setColor(nx, ny, "g")
                fullColorMap[nx][ny] = 'g'
        else:
            if not startedBackTracing:
                startedBackTracing = True
                API.togglePause()
            # Backtracking
            while full_path[-1] != (nx, ny):
                bx, by = full_path.pop()



    log("Target reached!")
    log("Final full path: " + str(full_path))
  
    # Paint the actual path yellow
    for px, py in full_path:
        API.setColor(px, py, "Y")

if __name__ == "__main__":
    main()
