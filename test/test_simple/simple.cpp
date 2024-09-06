#include "../../src/nav_parameters.h"
#include "simulator.h"
#include "vector.h"
#include <debug.h>
#include <navigation.h>

int main()
{
    debug_init();
    // TODO: Migrate to new navigation
    /*
    Navigation nav(ERROR_COEFFICENT, ANGLE_CONSTRAINT, MIN_AFTER_DISTANCE, BLOCK_FIXED_OFFSET);

    Vector initial_pos = Vector();
    obstacle_t outer_wall = { .start = Vector(-1500, -1500), .end = Vector(1500, 1500) };
    obstacle_t inner_wall = { .start = Vector(-500, -500), .end = Vector(500, 500) };
    obstacle_t default_mat[] = { outer_wall, inner_wall };

    Simulator sim(initial_pos, 0, 10, default_mat, sizeof(default_mat) / sizeof(obstacle_t));

    for (int i = 0; i < 100000; i++) {
        sim.tick();
        if (nav.orientation == 0) {
            if (sim.distances.left > 1500) {
                debug_msg("setting orientation to 1");
                nav.orientation = 1;
            } else if (sim.distances.right > 1500) {
                debug_msg("setting orientation to -1");
                nav.orientation = -1;
            }
            sim.rotation = nav.angleToAxis(sim.position.y, 0);
        } else {
            // Standard Navigation
            nav.update(&sim.position, false, false);
            sim.rotation = nav.angle;
        }
        debug_position(sim.position);
        debug_current_direction(sim.rotation);
        if (nav.turn_count > 12) {
            return 0;
        }
    }
    return -1;
    */
}
