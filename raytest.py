import math
import pyray as pr

def draw_tire(r):
    pr.draw_circle_lines(0, 0, float(r), pr.WHITE)
    pr.draw_line(0, 0, int(r), 0, pr.WHITE)

pr.init_window(800, 600, "Tire test")
pr.init_physics()
pr.set_target_fps(60)

tick_rate = 1.0/50.0

# init physics object?
floor_obj = pr.create_physics_body_rectangle(pr.Vector2(400, 560), 600, 20, 10)
floor_obj.enabled = False
floor_obj.staticFriction = 1.0
floor_obj.dynamicFriction = 0.9

floor_obj2 = pr.create_physics_body_rectangle(pr.Vector2(700, 500), 600, 20, 10)
floor_obj2.enabled = False
floor_obj2.staticFriction = 1.0
floor_obj2.dynamicFriction = 0.9
pr.set_physics_body_rotation(floor_obj2, math.radians(-45.0))

floor_obj3 = pr.create_physics_body_rectangle(pr.Vector2(100, 500), 600, 20, 10)
floor_obj3.enabled = False
floor_obj3.staticFriction = 1.0
floor_obj3.dynamicFriction = 0.9
pr.set_physics_body_rotation(floor_obj3, math.radians(45.0))

square_obj = pr.create_physics_body_rectangle(pr.Vector2(100, 20), 30, 80, 20)
circle_obj = None

current_scenario = ""
tire_radius = 20.0
tire_density = 0.01
tire_spawn_pos = pr.Vector2(120, 200)
tire_initial_vel = pr.Vector2(10 * tick_rate, 10 * tick_rate)

dummy_counter = 0

def scenario1():
    global current_scenario, circle_obj, tire_radius, tire_density, tire_spawn_pos, tire_initial_vel

    current_scenario = "Sticky Wheel"
    if circle_obj is None:
        circle_obj = pr.create_physics_body_circle(tire_spawn_pos, tire_radius, tire_density)
    circle_obj.position = tire_spawn_pos
    circle_obj.orient = 0.0
    circle_obj.velocity = tire_initial_vel
    circle_obj.angularVelocity = 0.0
    circle_obj.staticFriction = 0.8
    circle_obj.dynamicFriction = 0.6

def scenario2():
    global current_scenario, circle_obj, tire_radius, tire_density, tire_spawn_pos, tire_initial_vel

    current_scenario = "Slippery Wheel"
    if circle_obj is None:
        circle_obj = pr.create_physics_body_circle(tire_spawn_pos, tire_radius, tire_density)
    circle_obj.position = tire_spawn_pos
    circle_obj.orient = 0.0
    circle_obj.velocity = tire_initial_vel
    circle_obj.angularVelocity = 0.0
    circle_obj.staticFriction = 0.0
    circle_obj.dynamicFriction = 0.0

# init camera
cam = pr.Camera2D(pr.Vector2(0,0), pr.Vector2(0, 0), 0.0, 1.0)

pr.set_physics_gravity(0.0, 9.81 * tick_rate)
pr.set_physics_time_step(tick_rate)

scenario1()

while not pr.window_should_close():
    """ elapsed += pr.get_frame_time()
    while elapsed >= tick_rate:
        pr.update_physics()
        elapsed -= tick_rate """

    if pr.is_mouse_button_released(pr.MouseButton.MOUSE_BUTTON_LEFT):
        # print("Click")
        if dummy_counter % 2 == 0:
            scenario1()
        else:
            scenario2()
    
    if pr.is_mouse_button_released(pr.MouseButton.MOUSE_BUTTON_RIGHT):
        dummy_counter += 1
        if dummy_counter % 2 == 0:
            scenario1()
        else:
            scenario2()
    
    pr.update_physics()

    pr.begin_drawing()

    mpos = pr.get_mouse_position()

    pr.clear_background(pr.Color(20, 10, 10))

    view_mat = pr.get_camera_matrix_2d(cam)

    # print(f"{view_mat.m0} {view_mat.m4} {view_mat.m8} {view_mat.m12}")
    # print(f"{view_mat.m1} {view_mat.m5} {view_mat.m9} {view_mat.m13}")
    # print(f"{view_mat.m2} {view_mat.m6} {view_mat.m10} {view_mat.m14}")
    # print(f"{view_mat.m3} {view_mat.m7} {view_mat.m11} {view_mat.m15}")

    # print(f"frame_time: {pr.get_frame_time()}")

    # draw physics objects
    bodies_count = pr.get_physics_bodies_count()
    for i in range(bodies_count):
        body = pr.get_physics_body(i)
        
        # if body.shape.vertexData.vertexCount > 4:
            # print(f"{body.position.x}, {body.position.y}, {math.degrees(body.orient)}, {body.shape.radius}")

        vcount = pr.get_physics_shape_vertices_count(i)
        for v in range(vcount):
            vA = pr.get_physics_shape_vertex(body, v)
            next_idx = (v+1) if (v+1) < vcount else 0
            vB = pr.get_physics_shape_vertex(body, next_idx)
            pr.draw_line_v(vA, vB, pr.WHITE)
        
        if body.shape.vertexData.vertexCount > 4:
            # a circle, add line
            vA = body.position
            vB = pr.get_physics_shape_vertex(body, 0)
            pr.draw_line_v(vA, vB, pr.WHITE)

    pr.draw_text(f"Mouse Pos: {mpos.x}, {mpos.y}", 24, 24, 24, pr.WHITE)
    pr.draw_text(f"Scenario: {current_scenario}", 24, 48, 24, pr.VIOLET)

    if circle_obj != None:
        vel = circle_obj.velocity
        pr.draw_text(f"Vel({vel.x:.2f}, {vel.y:.2f}), mass: {circle_obj.mass:.2f}, inertia: {circle_obj.inertia:.2f}", 24, 72, 24, pr.WHITE)
    pr.end_drawing()

pr.close_physics()
pr.close_window()