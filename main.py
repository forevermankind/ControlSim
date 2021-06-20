import pygame as pg
import numpy as np

from modules.state import State
from modules.graphics import create_window, create_cart, create_table, update_scene
from modules.control import FullStateFeedback, PID

## MASTER TODO LIST:
# - Pendulum dynamics
# - Controller implementation
# - Main menu
# - Edit mode
#   - click to select pivots/cart (& highlight them)
#   - scroll to change their velocity
#   - show velocity with arrow
#   - change number of arms
#   - change arm lengths
#   - change arm properties
# - Settings
#   - save/load settings from file
#   - save states (incl. control inputs)
#   - show trajectory history
# - Resize window correctly
#   - Store point information in nondimensional form?

# simulation properties
FPS = 60

def run_sim():
    clock = pg.time.Clock()

    WINDOW = create_window()
    cart_obj, cart_init_pos_physical = create_cart(WINDOW)
    table = create_table(WINDOW)

    state = State(cart_mass=1, cart_init_pos=cart_init_pos_physical, cart_display_obj=cart_obj)
    state.cart.add_linkage(linkage_mass=0.2, linkage_length=200e-3, linkage_angle=0.05)
    
    controller_setpoint = np.array([cart_init_pos_physical.x, 0, 0, 0]).T
    state.assign_controller(FullStateFeedback(K=np.array([0.000000000000425, 0.1, 30.290107277097590, 0.1]), setpoint=controller_setpoint))

    continue_sim = True
    paused = True
    apply_control = True

    while continue_sim:
        clock.tick(FPS)

        # simulate dynamics
        if not paused:
            dt = clock.get_time() # ms
            state.time += dt # time since last frame
            state.advance_step(dt, apply_control)

        # draw the current configuration
        update_scene(WINDOW, state, table, is_paused=paused, use_control=apply_control)

        # handle events
        for event in pg.event.get():

            if event.type == pg.QUIT: # user quits the windos
                continue_sim = False
                pg.quit()

            if event.type == pg.KEYDOWN: # user presses a key
                if event.key == pg.K_p: # pause game
                    print('toggled pause')
                    paused = not paused
                    # TODO: deal with pause/unpause while in edit mode

                if event.key == pg.K_c: # toggle controller action
                    print('toggled control')
                    apply_control = not apply_control

                if event.key == pg.K_e: # toggle edit mode
                    print('toggled edit')
                    paused = not paused
                    # TODO: IMPLEMENT EDITING

                if event.key == pg.K_r: # reset simulation
                    print('reset')
                    run_sim()




if __name__ == "__main__":
    run_sim()