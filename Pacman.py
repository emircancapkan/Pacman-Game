import pygame
import heapq
import time
import math
import random


maze = [
    [0,1,0,0,0,1,0,1,0,0,0,1,0,0,1,0,1,0,0,0],
    [0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,1,1,1,0],
    [0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0],
    [1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1,0],
    [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0],
    [0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,0],
    [0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0],
    [0,1,0,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0],
    [0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0],
    [1,0,1,1,0,1,1,1,0,1,1,0,1,1,1,1,1,0,1,0],
    [0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0],
    [0,1,1,1,1,1,0,1,1,0,1,1,1,1,1,0,1,1,1,0],
    [0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0],
    [0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0],
    [0,1,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,0],
    [0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0],
    [0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0]
]
# --- Config ---
TILE_SIZE = 40
ROWS = 20
COLS = 20
SCREEN_WIDTH = COLS * TILE_SIZE
SCREEN_HEIGHT = ROWS * TILE_SIZE

start = (0, 0)
goal = (19, 19)

# Starting position of the ghosts
ghosts = [
    {"pos": (1, 8), "color": (255, 0, 0)},      # red ghost
    {"pos": (5, 15), "color": (0, 255, 255)},   # turquoise ghost
    {"pos": (8, 13), "color": (255, 182, 255)}, # pink ghost
    {"pos": (14, 9), "color": (255, 182, 85)},  # orange ghost
    {"pos": (15, 17), "color": (0, 162, 232)},  # blue ghost
    {"pos": (18, 6), "color": (128, 0, 128)}    # purple ghost
]

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def is_ghost_nearby(pos, ghosts, distance=2):
    for ghost in ghosts:
        ghost_pos = ghost["pos"]
        if abs(pos[0] - ghost_pos[0]) + abs(pos[1] - ghost_pos[1]) < distance:
            return True
    return False

def astar(maze, start, goal, ghosts, blocked_positions=None):
    if blocked_positions is None:
        blocked_positions = set()
    
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < ROWS and 0 <= neighbor[1] < COLS:
                if (maze[neighbor[0]][neighbor[1]] == 1 or 
                    any(neighbor == ghost["pos"] for ghost in ghosts) or
                    neighbor in blocked_positions):
                    continue
                
                # Avoid roads close to ghosts
                ghost_penalty = 100 if is_ghost_nearby(neighbor, ghosts) else 0
                
                tentative_g = g_score[current] + 1 + ghost_penalty
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

    return []

def find_safe_path(maze, start, goal, ghosts):
    blocked_positions = set()
    while True:
        path = astar(maze, start, goal, ghosts, blocked_positions)
        if not path:
            return None
        
        # Check if the road is safe
        unsafe_positions = []
        for pos in path:
            if check_collision(pos, ghosts):
                unsafe_positions.append(pos)
        
        if not unsafe_positions:
            return path
        
        # Block unsafe positions
        for pos in unsafe_positions:
            blocked_positions.add(pos)

def move_ghosts():
    for ghost in ghosts:
        # Get current position
        current_pos = ghost["pos"]
        possible_moves = []
        
        # Check possible moves
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            new_pos = (current_pos[0] + dx, current_pos[1] + dy)
            if (0 <= new_pos[0] < ROWS and 
                0 <= new_pos[1] < COLS and 
                maze[new_pos[0]][new_pos[1]] == 0):
                possible_moves.append(new_pos)
        
        # If there is a valid move, choose one at random
        if possible_moves:
            ghost["pos"] = random.choice(possible_moves)

def draw_ghost(screen, x, y, color):
    # Ghost drawing
    center = (x * TILE_SIZE + TILE_SIZE // 2, y * TILE_SIZE + TILE_SIZE // 2)
    radius = TILE_SIZE // 2 - 4
    
    # the body of ghost
    pygame.draw.circle(screen, color, center, radius)  # upper part of ghost
    
    # bottom part of ghost
    ghost_bottom = [
        (center[0] - radius, center[1]),
        (center[0] - radius, center[1] + radius),
        (center[0] - radius + radius//2, center[1] + radius - 5),
        (center[0], center[1] + radius),
        (center[0] + radius - radius//2, center[1] + radius - 5),
        (center[0] + radius, center[1] + radius),
        (center[0] + radius, center[1])
    ]
    pygame.draw.polygon(screen, color, ghost_bottom)
    
    # Eyes
    eye_color = (255, 255, 255)
    pupil_color = (0, 0, 0)
    eye_radius = radius // 4
    pupil_radius = eye_radius // 2
    
    # left eye
    left_eye_pos = (center[0] - radius//2, center[1] - radius//4)
    pygame.draw.circle(screen, eye_color, left_eye_pos, eye_radius)
    pygame.draw.circle(screen, pupil_color, left_eye_pos, pupil_radius)
    
    # right eye
    right_eye_pos = (center[0] + radius//2, center[1] - radius//4)
    pygame.draw.circle(screen, eye_color, right_eye_pos, eye_radius)
    pygame.draw.circle(screen, pupil_color, right_eye_pos, pupil_radius)

def draw_maze(path_up_to=None):
    for row in range(ROWS):
        for col in range(COLS):
            rect = pygame.Rect(col*TILE_SIZE, row*TILE_SIZE, TILE_SIZE, TILE_SIZE)
            if maze[row][col] == 1:
                pygame.draw.rect(screen, (0, 0, 0), rect)
            else:
                pygame.draw.rect(screen, (255, 255, 255), rect)
                pygame.draw.rect(screen, (200, 200, 200), rect, 1)

    # draw ghosts
    for ghost in ghosts:
        draw_ghost(screen, ghost["pos"][1], ghost["pos"][0], ghost["color"])

    # Start and goal points
    pygame.draw.rect(screen, (0, 0, 255), pygame.Rect(start[1]*TILE_SIZE, start[0]*TILE_SIZE, TILE_SIZE, TILE_SIZE))
    pygame.draw.rect(screen, (255, 0, 0), pygame.Rect(goal[1]*TILE_SIZE, goal[0]*TILE_SIZE, TILE_SIZE, TILE_SIZE))

def draw_pacman(screen, x, y, direction):
    # centre of pac-man
    center = (x * TILE_SIZE + TILE_SIZE // 2, y * TILE_SIZE + TILE_SIZE // 2)
    radius = TILE_SIZE // 2 - 2
    
    # Calculate mouth angle for animation
    mouth_angle = abs(math.sin(time.time() * 10)) * 45  # Mouth opening and closing animation
    
    # Calculating angles according to direction
    base_angle = 0
    if direction == 'right':
        base_angle = 0
    elif direction == 'left':
        base_angle = 180
    elif direction == 'up':
        base_angle = 90
    elif direction == 'down':
        base_angle = 270
    
    # Pac-Man drawing
    pygame.draw.circle(screen, (255, 255, 0), center, radius)
    
    # mouth drawing
    pygame.draw.polygon(screen, (0, 0, 0), [center,
        (center[0] + radius * math.cos(math.radians(base_angle - mouth_angle)),
         center[1] - radius * math.sin(math.radians(base_angle - mouth_angle))),
        (center[0] + radius * math.cos(math.radians(base_angle + mouth_angle)),
         center[1] - radius * math.sin(math.radians(base_angle + mouth_angle)))])

def get_direction(current, next_pos):
    if next_pos[1] > current[1]:
        return 'right'
    elif next_pos[1] < current[1]:
        return 'left'
    elif next_pos[0] > current[0]:
        return 'down'
    else:
        return 'up'

def check_collision(pacman_pos, ghosts):
    for ghost in ghosts:
        if pacman_pos == ghost["pos"]:
            return True
    return False

def check_path_safety(path, ghosts):
    # Check for ghosts at every point on the path
    for pos in path:
        if check_collision(pos, ghosts):
            return False
    return True

def draw_game_over(screen):
    font = pygame.font.Font(None, 74)
    text = font.render('Lose', True, (255, 0, 0))
    text_rect = text.get_rect(center=(SCREEN_WIDTH/2, SCREEN_HEIGHT/2))
    screen.blit(text, text_rect)

# Pygame setup
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Pac-Man A*")

# A* is used to find the path
path = find_safe_path(maze, start, goal, ghosts)
current_index = 0

clock = pygame.time.Clock()

running = True
current_index = 0
current_direction = 'right'
ghost_move_delay = 0
GHOST_MOVE_INTERVAL = 5  # Ghosts will move every 5 frames

while running:
    screen.fill((255, 255, 255))
    
    # Move ghosts at regular intervals
    ghost_move_delay += 1
    if ghost_move_delay >= GHOST_MOVE_INTERVAL:
        move_ghosts()
        ghost_move_delay = 0
        
        # Calculate new path when ghosts move
        if path and current_index < len(path):
            current_pos = path[current_index]
        else:
            current_pos = start
            
        new_path = find_safe_path(maze, current_pos, goal, ghosts)
        if new_path:  # If a new safe path is found
            path = new_path
            current_index = 0
    
    if path and current_index < len(path):
        draw_maze()
        
        # Determine Pac-Man's position and direction
        current_pos = path[current_index]
        next_pos = path[current_index + 1] if current_index + 1 < len(path) else current_pos
        current_direction = get_direction(current_pos, next_pos)
        
        # draw Pac-man
        draw_pacman(screen, current_pos[1], current_pos[0], current_direction)
        
        current_index += 1
        pygame.time.delay(150)
    else:
        draw_maze()
        if path:
            draw_pacman(screen, path[-1][1], path[-1][0], current_direction)

    pygame.display.flip()
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

pygame.quit()
