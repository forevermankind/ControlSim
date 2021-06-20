import os
from numpy.random import PCG64
import pygame as pg
import numpy as np
# from modules.classes import Point


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"Point at position (x,y)=({self.x},{self.y})"

    @property
    def coords(self):
        return np.array([self.x, self.y])

# define simulation window
WINDOW_WIDTH, WINDOW_HEIGHT = 1200, 600

pg.font.init()

# colours
WHITE = (255,255,255)
BROWN = (200, 125, 15)
GREY = (180, 180, 180)
DARK_GREY = (100, 100, 100)
RED = (255, 0, 0)
LIGHT_GREEN = (30,215,90)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)

# graphics
CART_WIDTH, CART_HEIGHT = 100, 60
CART_INIT_POS = Point((WINDOW_WIDTH - CART_WIDTH)//2, 350)
WHEEL_RADIUS = 18
PIVOT_RADIUS = 16
ARM_WIDTH = 16
TABLE_POS = Point(0, CART_INIT_POS.y + CART_HEIGHT + WHEEL_RADIUS)

ICON_SIZE = 64
CG_SIZE = 16
CG_SYMBOL_OFFSET = np.array([CG_SIZE//2, CG_SIZE//2])# distance from top left to centre of CG symbol
MENU_FONT = pg.font.SysFont('Calibri', 28)
MENU_SUB_FONT = pg.font.SysFont('Calibri', 18)
EDIT_TEXT = MENU_FONT.render("Toggle Edit Mode", 1, BLACK)
PAUSE_TEXT = MENU_FONT.render("Pause Simulation", 1, BLACK)
CONTROLLER_TEXT = MENU_FONT.render("Toggle Controller", 1, BLACK)
RESET_TEXT = MENU_FONT.render("Reset", 1, BLACK)
E_KEY = pg.transform.scale(pg.image.load(os.path.join(os.getcwd(), 'assets', 'e_key.png')), (ICON_SIZE, ICON_SIZE))
C_KEY = pg.transform.scale(pg.image.load(os.path.join(os.getcwd(), 'assets', 'c_key.png')), (ICON_SIZE, ICON_SIZE))
P_KEY = pg.transform.scale(pg.image.load(os.path.join(os.getcwd(), 'assets', 'p_key.png')), (ICON_SIZE, ICON_SIZE))
R_KEY = pg.transform.scale(pg.image.load(os.path.join(os.getcwd(), 'assets', 'r_key.png')), (ICON_SIZE, ICON_SIZE))
CG_SYMBOL = pg.transform.scale(pg.image.load(os.path.join(os.getcwd(), 'assets', 'centre_of_gravity.png')), (CG_SIZE, CG_SIZE))
    
def create_window():
    WINDOW_FLAGS = pg.RESIZABLE
    WINDOW = pg.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), WINDOW_FLAGS)
    pg.display.set_caption("Control Sim")
    return WINDOW

def get_window_dimensions(WINDOW):
    width, height = WINDOW.get_size()
    return width, height

def create_cart(WINDOW):
    cart_obj = pg.Rect(*CART_INIT_POS.coords, CART_WIDTH, CART_HEIGHT)
    cart_init_pos_physical = Point(*CART_INIT_POS.coords * 1e-3)
    return cart_obj, cart_init_pos_physical

def create_table(WINDOW):
    window_width, window_height = get_window_dimensions(WINDOW)
    table = pg.Rect(*TABLE_POS.coords, window_width, window_height)
    return table

def draw_menu(WINDOW, is_paused, use_control):
    window_width, _ = get_window_dimensions(WINDOW)

    WINDOW.blit(E_KEY, (window_width-1.25*ICON_SIZE, 0.5*ICON_SIZE))
    WINDOW.blit(EDIT_TEXT, (window_width - 1.25*ICON_SIZE - EDIT_TEXT.get_width(), 0.75*ICON_SIZE))

    WINDOW.blit(P_KEY, (window_width -1.25*ICON_SIZE, 1.5*ICON_SIZE))
    WINDOW.blit(PAUSE_TEXT, (window_width - 1.25*ICON_SIZE - PAUSE_TEXT.get_width(), 1.75*ICON_SIZE))
    if is_paused:
        pause_state_text = MENU_SUB_FONT.render("paused", 1, RED)
    else:
        pause_state_text = MENU_SUB_FONT.render("running", 1, LIGHT_GREEN)
    WINDOW.blit(pause_state_text, (window_width - 1.25*ICON_SIZE - pause_state_text.get_width(), 2.1*ICON_SIZE))

    WINDOW.blit(C_KEY, (window_width -1.25*ICON_SIZE, 2.5*ICON_SIZE))
    WINDOW.blit(CONTROLLER_TEXT, (window_width - 1.25*ICON_SIZE - CONTROLLER_TEXT.get_width(), 2.75*ICON_SIZE))
    if use_control:
        control_state_text = MENU_SUB_FONT.render("controlled", 1, LIGHT_GREEN)
    else:
        control_state_text = MENU_SUB_FONT.render("uncontrolled", 1, RED)
    WINDOW.blit(control_state_text, (window_width - 1.25*ICON_SIZE - control_state_text.get_width(), 3.1*ICON_SIZE))

    WINDOW.blit(R_KEY, (window_width -1.25*ICON_SIZE, 3.5*ICON_SIZE))
    WINDOW.blit(RESET_TEXT, (window_width - 1.25*ICON_SIZE - RESET_TEXT.get_width(), 3.75*ICON_SIZE))

def draw_cart_configuration(WINDOW, state):
    # draw table, cart & pendulum(s)
    pg.draw.rect(WINDOW, GREY, state.cart.display_obj)
    cart_cg_position = np.array([state.cart.display_obj.centerx, state.cart.display_obj.centery])
    pg.draw.circle(WINDOW, WHITE, cart_cg_position, CG_SIZE/2 - 1)
    WINDOW.blit(CG_SYMBOL, cart_cg_position - CG_SYMBOL_OFFSET)
    pg.draw.circle(WINDOW, DARK_GREY, (state.cart.display_obj.centerx - state.cart.display_obj.width//4, state.cart.display_obj.y + state.cart.display_obj.height), WHEEL_RADIUS) # left wheel
    pg.draw.circle(WINDOW, DARK_GREY, (state.cart.display_obj.centerx + state.cart.display_obj.width//4, state.cart.display_obj.y + state.cart.display_obj.height), WHEEL_RADIUS) # right wheel

    pivots = state.get_pivot_points(WINDOW)
    for pivot_num in range(len(pivots)):
        pivot = pivots[pivot_num]
        pivot_coords = pivot.coords

        if pivot_num > 0: # pivot is not at cart
            base_pivot_coords = base_pivot.coords
            pg.draw.line(WINDOW, BLUE, base_pivot_coords, pivot_coords, ARM_WIDTH)
            pg.draw.circle(WINDOW, RED, base_pivot_coords, PIVOT_RADIUS)
            # draw CG position at centre of each bar (can be adjusted later)
            cg_position = (base_pivot_coords + pivot_coords)/2
            pg.draw.circle(WINDOW, WHITE, cg_position, CG_SIZE/2 - 1)
            WINDOW.blit(CG_SYMBOL, cg_position - CG_SYMBOL_OFFSET)
        pg.draw.circle(WINDOW, RED, pivot_coords, PIVOT_RADIUS)
        base_pivot = pivot # current pivot is base for next linkage

def update_scene(WINDOW, state, table, is_paused=False, use_control=False):
    WINDOW.fill(WHITE)
    pg.draw.rect(WINDOW, BROWN, table)
    draw_cart_configuration(WINDOW, state)
    draw_menu(WINDOW, is_paused, use_control)
    pg.display.update()

def pixel_to_physical_coords(window, pix_x, pix_y):
    # In: pixel location (top left of screen is origin, x increases to the right (units: pix ~ mm), y increases down (units: pix ~ mm))
    _, window_height = get_window_dimensions(window)
    phys_x = pix_x * 1e-3
    phys_y = (window_height - pix_y) * 1e-3
    return np.array([phys_x, phys_y])

def physical_to_pixel_coords(window, phys_x, phys_y):
    # In: physical location (bottom left of screen is origin, x increases to the right, y increases up)
    _, window_height = get_window_dimensions(window)
    pix_x = phys_x * 1e3
    pix_y = window_height - phys_y * 1e3
    return np.array([pix_x, pix_y])