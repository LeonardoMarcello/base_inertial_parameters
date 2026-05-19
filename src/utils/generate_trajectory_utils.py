import numpy as np
from scipy.optimize import minimize
from yaml import SafeLoader
try:
    import hppfcl as coal # Compatible with python 3.10
except ImportError:
    import coal # Compatible with python 3.12

def fourier_decomposition(t, params):
    """
    params: a flattened array of [a_coeffs, b_coeffs, q0]
    Returns q, dq, ddq at time t
    """
    # Reshape params to (num_joints, num_harmonics)
    # Calculate q(t) using sin/cos sums
    # ... (Implementation of the math above)
    Nj = params["Nj"]                 # Number of joints
    q = params["q0"].copy()                  # poition offset [rad]
    dq = np.zeros_like(q)
    ddq = np.zeros_like(q)

    wf = 2*np.pi*params["omega"]    # Frequency in [rad/s]
    if np.abs(wf) < 1e-6:
        wf = 1e-6  # Tiny epsilon to avoid division by zero

    Nh = params["Nh"]               # Number of harmonics

    for i in range(Nj):  # For each joint
        for k in range(1, Nh+1):    # For each harmonic
            ai_k = params["a"][i, k-1]
            bi_k = params["b"][i, k-1]

            q[i] += ai_k/(wf*k) * np.sin(k * wf * t) + bi_k/(wf*k) * np.cos(k * wf * t)
            dq[i] += ai_k * np.cos(k * wf * t) + bi_k * np.sin(k * wf * t)
            ddq[i] += -k*wf * ai_k * np.sin(k * wf * t) + k*wf * bi_k * np.cos(k * wf * t)

    return q, dq, ddq

def objective(params, robot, trajectory, T_period, resolution=15, k2 = 1):
    """
    Minimize the condition number of the integrated regressor
    """
    times = np.linspace(0, T_period, resolution)
    # Build regressor matrix
    Y_log = []
    for t in times:
        q, dq, ddq = trajectory(t, params)

        robot.set_q(q)
        robot.set_dq(dq)
        robot.set_ddq(ddq)
        robot.set_dqr(dq)
        robot.set_ddqr(ddq)

        # Full inverse-dynamics regressor (per-joint rows)
        Y_red = robot.get_Yr_red()
        Y_dl = robot.get_reg_dl()
        Y = np.hstack([Y_red, Y_dl])
        Y_log.append(Y)

    YY = np.concatenate([np.asarray(Y) for Y in Y_log], axis=0)  # (M*n) x p

    # Get metrics bsed on regressor
    s = np.linalg.svd(YY, compute_uv=False)

    sigma_max = np.max(s)
    sigma_min = np.min(s)
    C1 = sigma_max / (sigma_min+1e-16)

    C2 = C1 + k2/(sigma_min+1e-16)


    return C2

def speed_constraint(bounds, trajectory, params, T_period, resolution=15, verbose=False):
    check_times = np.linspace(0, T_period, resolution) # Discretize trajectory
    for t in check_times:
        _, dq, _ = trajectory(t, params)
        if np.any(np.abs(dq) > bounds):
            if verbose:
                print(f"Speed violation detected at time {t}.")
            return -np.linalg.norm(np.abs(dq) - bounds)
    return np.linalg.norm(np.abs(dq) - bounds)  # No violation

def fourier_kynematic_constraint(q_max, dq_max, ddq_max, params, verbose=False):
    """
    ref from work 'Parameter identification for industrial robots with a fast and robust
    trajectory design approach'.
    """
    wf = 2*np.pi*params["omega"]    # Frequency in [rad/s]

    for i in range(params["Nj"]):
        qi = params["q0"][i]
        dqi = 0
        ddqi = 0
        for k in range(1, params["Nh"]+1):
            ai_k = params["a"][i, k-1]
            bi_k = params["b"][i, k-1]
            qi += 1/(wf*k) * np.sqrt(ai_k**2 + bi_k**2)
            dqi += np.sqrt(ai_k**2 + bi_k**2)
            ddqi += wf*k * np.sqrt(ai_k**2 + bi_k**2)
        if qi > q_max[i]:
            if verbose:
                print(f"Position violation detected.")
            return -1
        elif dqi > dq_max[i]:
            if verbose:
                print(f"Velocity violation detected.")
            return -1
        elif ddqi > ddq_max[i]:
            if verbose:
                print(f"Acceleration violation detected.")
            return -1
    return 1  # No violation


def collision_compute(robot, trajectory, params, T_period, resolution=15, verbose=False):
    """
    Returns the minimum distance across the whole trajectory.
    SciPy 'ineq' means: result >= 0 is SAFE.
    """
    check_times = np.linspace(0, T_period, resolution) # Discretize trajectory
    min_dist = float('inf')

    transformation = map_franka_pin2thunder()

    # Load meshes
    mesh_dir = "/home/leo/Desktop/Base Inertial Parameter/src/assets/franka_description/meshes/collision/"

    # 1. Define the Ground Plane
    # coal.Plane(Normal_Vector, Offset) -> Plane equation: n.x = d
    # For a horizontal plane at Z = 0.1: normal is [0, 0, 1], d is 0.1
    ground_plane = coal.Plane(np.array([0, 0, 1]), 0.1)
    ground_tf = coal.Transform3f() # Identity transform for static ground

    non_adjacent_pairs = [(i, j) for i in range(robot.numJoints) for j in range(i + 2, robot.numJoints)]

    for t in check_times:
        q, _, _ = trajectory(t, params)
        robot.set_q(q)

        for i in range(robot.numJoints + 1):
            # 2. Get Transform for Link i
            T_i = getattr(robot, f"get_T_w_{i}")() @ transformation[f"Link {i}"]
            tf_i = coal.Transform3f(T_i[:3, :3], T_i[:3, 3])

            # Load mesh (Note: PRE-LOAD THESE OUTSIDE THE LOOP TO SAVE SPEED)
            mesh_i = loadConvexMesh(mesh_dir + f"link{i}.stl")

            # 3. Check Collision/Distance with Ground Plane
            dist_req = coal.DistanceRequest()
            dist_res = coal.DistanceResult()

            # Distance from Link i to Ground
            d_ground = coal.distance(mesh_i, tf_i, ground_plane, ground_tf, dist_req, dist_res)

            if d_ground < min_dist:
                min_dist = d_ground

            # 4. Check Self-Collisions (Non-adjacent pairs)
            # This logic remains, but now also updates 'min_dist'
            for j in [pair[1] for pair in non_adjacent_pairs if pair[0] == i]:
                T_j = getattr(robot, f"get_T_w_{j}")() @ transformation[f"Link {j}"]
                tf_j = coal.Transform3f(T_j[:3, :3], T_j[:3, 3])
                mesh_j = loadConvexMesh(mesh_dir + f"link{j}.stl")

                d_self = coal.distance(mesh_i, tf_i, mesh_j, tf_j, dist_req, dist_res)
                if d_self < min_dist:
                    min_dist = d_self

    # SciPy ineq: return value must be > 0 for feasibility
    # If min_dist < 0.05, we are getting too close or are in collision
    return min_dist - 0.05

def collision_constraint(robot, trajectory, params, T_period, resolution=15, verbose=False):
    """
    Returns the minimum distance across the whole trajectory.
    SciPy 'ineq' means: result >= 0 is SAFE.
    """
    check_times = np.linspace(0, T_period, resolution) # Discretize trajectory
    min_dist = float('inf')

    transformation = map_franka_pin2thunder()

    # Load meshes
    mesh_dir = "/home/leo/Desktop/Base Inertial Parameter/src/assets/franka_description/meshes/collision/"

    # 1. Pre-define the Ground Plane at Z = 0.1
    ground_plane = coal.Plane(np.array([0, 0, 1]), 0.05)
    ground_tf = coal.Transform3f()

    non_adjacent_pairs = [(i, j) for i in range(robot.numJoints) for j in range(i + 2, robot.numJoints)]

    # Recommendation: Move mesh loading OUTSIDE this function to a global dict
    # If not, this function will still be slow due to Disk I/O.
    for t in check_times:
        q, _, _ = trajectory(t, params)
        robot.set_q(q)

        # Iterate through links for ground and self-collision
        for i in range(robot.numJoints + 1):
            T_i = getattr(robot, f"get_T_w_{i}")() @ transformation[f"Link {i}"]
            tf_i = coal.Transform3f(T_i[:3, :3], T_i[:3, 3])
            mesh_i = loadConvexMesh(mesh_dir + f"link{i}.stl")

            # --- Check Ground Collision ---
            if i != 0:  # Base link might be below ground, so we check distance instead of collision
                req_g = coal.CollisionRequest()
                res_g = coal.CollisionResult()
                coal.collide(mesh_i, tf_i, ground_plane, ground_tf, req_g, res_g)

                if res_g.isCollision():
                    if verbose: print(f"Ground collision: Link {i} at t={t}")
                    return -1.0

            # --- Check Self-Collision ---
            for j in [pair[1] for pair in non_adjacent_pairs if pair[0] == i]:
                T_j = getattr(robot, f"get_T_w_{j}")() @ transformation[f"Link {j}"]
                tf_j = coal.Transform3f(T_j[:3, :3], T_j[:3, 3])
                mesh_j = loadConvexMesh(mesh_dir + f"link{j}.stl")

                req_s = coal.CollisionRequest()
                res_s = coal.CollisionResult()
                coal.collide(mesh_i, tf_i, mesh_j, tf_j, req_s, res_s)

                if res_s.isCollision():
                    if verbose: print(f"Self-collision: Link {i} & {j} at t={t}")
                    return -1.0

    return 1.0  # Return positive value if entire trajectory is safe

# def loadConvexMesh(file_name: str):
#     loader = coal.MeshLoader()
#     bvh: coal.BVHModelBase = loader.load(file_name)
#     bvh.buildConvexHull(True, "Qt")
#     return bvh.convex
def loadConvexMesh(path):
    import trimesh
    import numpy as np
    import hppfcl as coal

    t_mesh = trimesh.load(path)
    mesh = coal.BVHModelOBBRSS()

    num_vertices = len(t_mesh.vertices)
    num_faces = len(t_mesh.faces)

    mesh.beginModel(num_faces, num_vertices)

    # 1. Add Vertices
    for i in range(num_vertices):
        v = t_mesh.vertices[i].astype(np.float64)
        mesh.addVertex(v)

    # 2. Add Triangles using the vertex coordinates directly
    for face in t_mesh.faces:
        # Get the actual [x, y, z] for each corner of the triangle
        p1 = t_mesh.vertices[face[0]].astype(np.float64)
        p2 = t_mesh.vertices[face[1]].astype(np.float64)
        p3 = t_mesh.vertices[face[2]].astype(np.float64)

        # Pass the three points to addTriangle as required by your C++ signature
        mesh.addTriangle(p1, p2, p3)

    mesh.endModel()

    return mesh

def solve(N_sample, params, robot, q_upper,q_lower,speed_bounds, trajectory, T_period, resolution=15):
    # Initial coefficients (usually start small or at zero)
    # params = [wf,
    #           q00, q01, ..., q0n,
    #           a0_0, a0_1, ..., a0_Nh,
    #
    #           aNj_0, aNj_1, ..., aNj_Nh,
    #           b0_0, b0_1, ..., b0_Nh,
    #
    #           bNj_0, bNj_1, ..., bNj_Nh]
    Nj = params["Nj"]
    Nh = params["Nh"]

    Np = 1 + Nj + 2*Nj*Nh  # Total number of parameters to optimize

    # Bounds: prevent the robot from requesting infinite speed/acceleration
    Wf_bounds = (0.05, .15)                                 # Frequency in [Hz]
    q0_bounds = [(l,u) for l,u in zip(q_lower, q_upper)]    # Joint positions in rad
    #q0_bounds = [(-0.5851,-0.5850), (-0.1745, -0.1744), (-0.3374, -0.3373), (-1.8768, -1.8767), (-1.0632, -1.0631), (1.7916, 1.7917), (-0.7285, -0.7284)]  # [rad]
  

    a_bounds = [(-.5, .5)] * (Nj * Nh)    # Fourier Coefficients amplitude
    b_bounds = [(-.5, .5)] * (Nj * Nh)    # Fourier Coefficients amplitude


    lower_bounds = [Wf_bounds[0]] + [q[0] for q in q0_bounds] + [a[0] for a in a_bounds] + [b[0] for b in b_bounds]
    upper_bounds = [Wf_bounds[1]] + [q[1] for q in q0_bounds] + [a[1] for a in a_bounds] + [b[1] for b in b_bounds]


    from scipy.stats import qmc # Quasi-Monte Carlo for smart grids
    sampler = qmc.Sobol(d=Np, scramble=True)
    sample = sampler.random(n=N_sample) 
    grid_points = qmc.scale(sample, lower_bounds, upper_bounds)

    best_score = float('inf')
    founded_configs = []

    print("Starting Grid Search...")
    for i,point in enumerate(grid_points):
        print(f"Testing grid point {i+1}/{N_sample}", end="\r")
        search_params = params.copy()
        search_params["omega"] = point[0]
        search_params["q0"] = point[1:1+Nj]

        a_ampli = point[1+Nj:1+Nj*(1+Nh)].reshape(Nj, Nh)
        b_ampli = point[1+Nj*(1+Nh):].reshape(Nj, Nh)

        #a_sign = point[1+Nj:1+Nj*(1+Nh)].reshape(Nj, Nh)
        #b_sign = point[1+Nj*(1+Nh):].reshape(Nj, Nh)


        search_params["a"] = a_ampli #* np.sign(a_sign)
        search_params["b"] = b_ampli #* np.sign(b_sign)
        # search_params["omega"] = 0.075
        # search_params["q0"] = [-0.5850, -0.1744, -0.3373, -1.8767, -1.0631, 1.7917, -0.7284]
        # search_params["b"] = np.array([[-0.1136, 0.66, -0.1858, 0.1867, 0.2357] ,       # Link 1
        #                                 [-0.2437, 0.182, -0.239, -0.2647, 0.4252],      # Link 2
        #                                 [0.3898, -0.0268, 0.0179, 0.4767, 0.2726],      # Link 3
        #                                 [0.0401, -0.1503, 0.2359, -0.2022, 0.3693],     # Link 4
        #                                 [0.5491, 0.1046, 0.0407, -0.0432, 0.2448],      # Link 5
        #                                 [0.0692, -0.6946, -0.4442, -0.5749, -0.0698],   # Link 6
        #                                 [0.2023, -0.257, -0.501, 0.0774, -0.4833]] )    # Link 7
        # search_params["a"] =   np.array([   [-0.2031, 0.1295, -0.009, 0.2319, -0.7598],    # Link 1
        #                                     [-0.0699, -0.538, -0.2015, -0.5535, 0.2352],   # Link 2
        #                                     [0.3076, 0.5864, -0.4813, -0.1228, -0.5273] ,  # Link 3
        #                                     [0.1269, -0.0253, -0.2405, 0.2178, 0.6984]  ,  # Link 4
        #                                     [-0.2773, -0.0857, 0.481, 0.5311, -0.1639]  ,  # Link 5
        #                                     [0.21, -0.1194, -0.095, -0.0964, -0.0399]   ,  # Link 6
        #                                     [-0.2273, -0.5636, -0.2099, 0.5725, -0.3748]]) # Link 7



        # Quick check: Is it collision free at the start?
        if fourier_kynematic_constraint(q_max=(q_upper-q_lower)/2, dq_max=speed_bounds, ddq_max=5*speed_bounds, params=search_params) < 0:
            continue
        if collision_constraint(robot, trajectory, search_params, T_period, resolution) < 0:
            #plot_3Dviz(robot, None, None, None, block=True)
            continue
        if speed_constraint(speed_bounds, trajectory, search_params, T_period, resolution) < 0:
            continue


        score = objective(search_params, robot, trajectory, T_period, resolution)
        founded_configs.append((search_params, score))

        if score < best_score:
            print(f"Grid point {i+1}/{N_sample} score: {score:.4f}")
            best_score = score
            result = search_params.copy()

    founded_configs.sort(key=lambda x: x[1])
    print(f"Best Grid Score: {best_score}                   ")
    return result, best_score, founded_configs


def export_fourier_trajectory(path, result, score):
    import datetime
    import os
    if path is None:
        date_str = datetime.datetime.now().strftime("%d_%m_%Y")
        results_dir = f"results/data_{date_str}"
    else:
        results_dir = path

    if not os.path.exists(results_dir):
        os.makedirs(results_dir)

    # 2. Create identification_par.yaml
    traject_path = os.path.join(results_dir, f"optimal_trajectory_score_{score}.yaml")

    with open(traject_path, "w") as f:
        f.write(f"# This file were autogenerated on {datetime.datetime.now()} from the optima trajectory routine\n")
        f.write(f"# The search grid with fourier decomposition has provides a regressor conditioning of {score:.4f}\n")
        f.write(f"# TO DO: print report of search used\n")
        # Add other parameters
        f.write(f"fourier: \n")
        f.write(f"  n: {params['Nh']} # Number of harmonics \n")
        f.write(f"  wf: {np.array(result['omega']).repeat(params['Nj']).tolist()} # Frequency in [Hz]\n")
        f.write(f"  a:  # Fourier coefficients\n")
        for i in range(params['Nj']):
            a_coeffs = ", ".join([f"{coef:.6f}" for coef in result['a'][i]])
            f.write(f"    - [{a_coeffs}]  # Link {i+1}\n")
        f.write(f"  b:  # Fourier coefficients\n")
        for i in range(params['Nj']):
            b_coeffs = ", ".join([f"{coef:.6f}" for coef in result['b'][i]])
            f.write(f"    - [{b_coeffs}]  # Link {i+1}\n")

        f.write(f"  q0: {result['q0'].tolist()}  # Initial joint positions in radians\n")
        f.write(f"  duration: 21.0  # Set how long should trajectory be performed in seconds\n")

    print(f"Exiting Trajectory Parameters written in '{traject_path}'")

# def solve_SLSQP(params, robot, speed_bounds, trajectory, T_period, resolution=15):
#     # Initial coefficients (usually start small or at zero)
#     # params = [wf,
#     #           q00, q01, ..., q0n,
#     #           a0_0, a0_1, ..., a0_Nh,
#     #
#     #           aNj_0, aNj_1, ..., aNj_Nh,
#     #           b0_0, b0_1, ..., b0_Nh,
#     #
#     #           bNj_0, bNj_1, ..., bNj_Nh]
#     # Flatten your paper_params into a single vector x0
#     Nj = params["Nj"]
#     Nh = params["Nh"]

#     Np = 1 + Nj + 2*Nj*Nh  # Total number of parameters to optimize
#     x0 = np.zeros(Np)
#     x0[0] = 0.1
#     x0[Nj] = np.pi
#     x0[2+Nj:] = 0.01


#     def objective_wrapper(x, robot, trajectory, T_period, resolution):
#         # Unflatten x back into params dict
#         Nj = int(params["Nj"])
#         Nh = int(params["Nh"])

#         wrapped_params = params.copy()
#         wrapped_params["omega"] = x[0]
#         wrapped_params["q0"] = x[1:1+Nj]
#         wrapped_params["a"] = x[1+Nj:1+Nj*(1+Nh)].reshape(Nj, Nh)
#         wrapped_params["b"] = x[1+Nj*(1+Nh):].reshape(Nj, Nh)

#         return objective(wrapped_params, robot, trajectory, T_period, resolution)

#     # Define Constraints
#     def collision_constraint_wrapper(x,robot, trajectory, params, T_period, resolution):
#         # Unflatten x back into params dict
#         Nj = int(params["Nj"])
#         Nh = int(params["Nh"])

#         wrapped_params = params.copy()
#         wrapped_params["omega"] = x[0]
#         wrapped_params["q0"] = x[1:1+Nj]
#         wrapped_params["a"] = x[1+Nj:1+Nj*(1+Nh)].reshape(Nj, Nh)
#         wrapped_params["b"] = x[1+Nj*(1+Nh):].reshape(Nj, Nh)

#         return collision_constraint(robot, trajectory, wrapped_params, T_period, resolution)

#     cons = [
#         {'type': 'ineq', 'fun': collision_constraint_wrapper, 'args': (franka, trajectory, params, T_period, resolution)},
#         # {'type': 'ineq', 'fun': speed_constraint_smooth, 'args': (speed_bounds, fourier_decomposition, 21.0, 15)}
#     ]


#     bounds = [(-5.0, 5.0) for _ in range(Np)]
#     # Run Minimize
#     res = minimize(
#         objective_wrapper,
#         x0,
#         args=(robot, trajectory, T_period, resolution),
#         method='SLSQP',
#         bounds=bounds,
#         constraints=cons,
#         options={'ftol': 1e-3, 'disp': True, 'maxiter': 50}
#     )

#     if res.success:
#         print("Optimal Trajectory Found!")
#         res.x
#     else:        print("Optimization Failed:", res.message)


def plot_3Dviz(robot, p1,p2,dist, block=False):
    # --- 3D Visualization ---
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import trimesh
    import numpy as np

    higlight = (5,7)


    remap = map_franka_pin2thunder()
    Tranforms = []
    meshes = []
    for i in range(0,robot.numJoints+1):
        Ti = getattr(robot, f"get_T_w_{i}")() 
        meshi = trimesh.load(mesh_dir + f"link{i}.stl")
        Tranforms.append(Ti)
        meshes.append(meshi)


    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 1. Print meshes
    for i in range(robot.numJoints+1):
        vis_mesh = meshes[i]
        vis_Tranforms = Tranforms[i] @ remap[f"Link {i}"]
        vis_mesh.apply_transform(vis_Tranforms)

        if i in higlight:
            ax.plot_trisurf(vis_mesh.vertices[:, 0], vis_mesh.vertices[:, 1], vis_mesh.vertices[:, 2], 
                            triangles=vis_mesh.faces, color='forestgreen', alpha=0.6, 
                            edgecolor='k', linewidth=0.1, label=f'Link {i}')
        else:
            ax.plot_trisurf(vis_mesh.vertices[:, 0], vis_mesh.vertices[:, 1], vis_mesh.vertices[:, 2], 
                            triangles=vis_mesh.faces, color='royalblue', alpha=0.3, 
                            edgecolor='k', linewidth=0.1, label=f'Link {i}')
    # 2. Plot collision data
    if dist is not None:
        # 4. Plot the Witness Points (Coal/hpp-fcl results)
        # ONLY do this if the points appear near [0,0,0] instead of near the robot
        # p6_world = T_EE[:3, :3] @ p_ee + T_EE[:3, 3]
        # p7_world = T_EE[:3, :3] @ p_link + T_EE[:3, 3]
        #p7_world = T_link[:3, :3] @ p_link + T_link[:3, 3]
        p6_world =  p1
        p7_world =  p2

        # 2. Calculate the World-Frame Centers of the Meshes
        # This is the average of the vertices after apply_transform(T)
        center6 = Tranforms[higlight[0]][:3, 3]
        center7 = Tranforms[higlight[1]][:3, 3]
        
        # Then plot p1_world and p2_world
        ax.scatter([p6_world[0]], [p6_world[1]], [p6_world[2]], color='red', s=150, 
                edgecolors='white', depthshade=False, label='Closest Pt on L6')
        ax.scatter([p7_world[0]], [p7_world[1]], [p7_world[2]], color='orange', s=150, 
                edgecolors='white', depthshade=False, label='Closest Pt on L7')

        # 3. Plot the Mesh Centers (Big Blue/Green Dots)
        ax.scatter([center6[0]], [center6[1]], [center6[2]], 
                color='blue', s=300, marker='o', edgecolors='white', 
                linewidth=2, label='Center Link 6')
        ax.scatter([center7[0]], [center7[1]], [center7[2]], 
                color='green', s=300, marker='o', edgecolors='white', 
                linewidth=2, label='Center Link 7')
        
        # 5. Connect them with a line
        ax.plot([p6_world[0], p7_world[0]], [p6_world[1], p7_world[1]], [p6_world[2], p7_world[2]], 
                color='black', linestyle='--', linewidth=2.5, alpha=0.8)

    # 6. Normalize the View (Prevent squashing)
    # This keeps the x, y, z axes at the same scale
    all_v = np.vstack([vis_mesh.vertices for vis_mesh in meshes])
    center = all_v.mean(axis=0)
    scale = all_v.flatten()
    ax.set_box_aspect((np.ptp(scale), np.ptp(scale), np.ptp(scale))) # Equal aspect ratio

    # Set limits based on the mesh bounds
    mins = all_v.min(axis=0)
    maxs = all_v.max(axis=0)
    # ax.set_xlim(mins[0]-0.05, maxs[0]+0.05)
    # ax.set_ylim(mins[1]-0.05, maxs[1]+0.05)
    # ax.set_zlim(mins[2]-0.05, maxs[2]+0.05)
    ax.set_aspect('equal')
    if dist is not None:
        ax.set_title(f"Franka Collision Scene\nDistance: {dist:.5f} m")
    else:
        ax.set_title(f"Franka Collision Scene\nDistance: N/A")
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')


    # Custom legend logic (trisurf doesn't always play nice with legends)
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='royalblue', lw=4, label='Link 6 (STL)'),
        Line2D([0], [0], color='forestgreen', lw=4, label='Link 7 (STL)'),
        Line2D([0], [0], marker='o', color='w', label='Witness Points',
               markerfacecolor='red', markersize=10)
    ]
    ax.legend(handles=legend_elements, loc='upper left')

    plt.tight_layout()
    plt.show(block=block)


def map_franka_pin2thunder():
    # Load Thunder
    config_path = '/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/identif_config.yaml'
    with open(config_path, 'r') as f:
        config = yaml.load(f, Loader=SafeLoader)
    franka = thunder.thunder_franka()
    thunder.load_params(franka, config['robot']['path'])

    # Load Pinocchio
    import pinocchio as pin
    model = pin.buildModelFromUrdf("/home/leo/Desktop/Base Inertial Parameter/src/assets/franka_description/meshes/urdf/panda.urdf")
    data = model.createData()

    q_pin = pin.neutral(model)
    franka.set_q(q_pin)
    pin.forwardKinematics(model, data, q_pin)

    Transformations = {}
    for i in range(0, franka.numJoints+1):
        T_thunder = getattr(franka, f"get_T_w_{i}")()
        T_pin = data.oMi[i].homogeneous
        Transformations[f"Link {i}"] = T_thunder@np.linalg.inv(T_pin)

    return Transformations


if __name__ == "__main__":

    import utils.import_thunder as thunder
    import yaml

    mesh_dir = "/home/leo/Desktop/Base Inertial Parameter/src/assets/franka_description/meshes/collision/"
    mesh_ee = loadConvexMesh(mesh_dir + "link7.stl")
    mesh_link6 = loadConvexMesh(mesh_dir + "link5.stl")

    config_path = '/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/identif_config.yaml'
    with open(config_path, 'r') as f:
        config = yaml.load(f, Loader=SafeLoader)
    franka = thunder.thunder_franka()
    thunder.load_params(franka, config['robot']['path'])

    # q = [-0.41019758, -1.13292484, -2.19103828, -2.08644876,  0.04045621,  1.74171272, -0.62473228]
    # franka.set_q(q)
    # plot_3Dviz(franka, None,None,None, block=True)


    paper_params = {"Nh":5, "Nj":7}
    paper_params["omega"] = 0.075
    paper_params["q0"] = [-0.5850, -0.1744, -0.3373, -1.8767, -1.0631, 1.7917, -0.7284]
    paper_params["b"] = np.array([[-0.1136, 0.66, -0.1858, 0.1867, 0.2357] ,       # Link 1
                                    [-0.2437, 0.182, -0.239, -0.2647, 0.4252],      # Link 2
                                    [0.3898, -0.0268, 0.0179, 0.4767, 0.2726],      # Link 3
                                    [0.0401, -0.1503, 0.2359, -0.2022, 0.3693],     # Link 4
                                    [0.5491, 0.1046, 0.0407, -0.0432, 0.2448],      # Link 5
                                    [0.0692, -0.6946, -0.4442, -0.5749, -0.0698],   # Link 6
                                    [0.2023, -0.257, -0.501, 0.0774, -0.4833]] )    # Link 7
    paper_params["a"] =   np.array([   [-0.2031, 0.1295, -0.009, 0.2319, -0.7598],    # Link 1
                                        [-0.0699, -0.538, -0.2015, -0.5535, 0.2352],   # Link 2
                                        [0.3076, 0.5864, -0.4813, -0.1228, -0.5273] ,  # Link 3
                                        [0.1269, -0.0253, -0.2405, 0.2178, 0.6984]  ,  # Link 4
                                        [-0.2773, -0.0857, 0.481, 0.5311, -0.1639]  ,  # Link 5
                                        [0.21, -0.1194, -0.095, -0.0964, -0.0399]   ,  # Link 6
                                        [-0.2273, -0.5636, -0.2099, 0.5725, -0.3748]]) # Link 7

    # q_col,_,_ =fourier_decomposition(0.42424242424242425,paper_params)
    # franka.set_q(q_col)
    # print(q_col)
    # plot_3Dviz(franka, None,None,None, block=True)

    # paper_params["omega"] = 0.0893187559209764 # Frequency in [Hz]
    # paper_params["a"] =   np.array(  # Fourier coefficients
    #     [[0.037990, -0.046651, -0.132481, 0.402229, -0.260996],  # Link 1
    #     [0.350611, -0.274119, -0.406349, 0.253730, 0.024772],  # Link 2
    #     [0.362076, -0.487712, -0.296093, -0.135831, -0.450565],  # Link 3
    #     [-0.121160, 0.186634, -0.466576, 0.276735, 0.449062],  # Link 4
    #     [0.128430, 0.172185, 0.207366, -0.484483, 0.097615],  # Link 5
    #     [0.127528, -0.343084, 0.094688, 0.110154, 0.033521],  # Link 6
    #     [-0.463566, -0.461097, 0.206034, -0.454893, -0.388043]])  # Link 7
    # paper_params["b"] = np.array([  # Fourier coefficients
    #     [-0.111301, -0.287124, 0.305348, -0.185885, -0.243007],  # Link 1
    #     [0.113111, 0.173080, -0.126301, 0.387160, 0.483779],  # Link 2
    #     [-0.415453, 0.352556, -0.192394, -0.191735, 0.143572],  # Link 3
    #     [0.087162, 0.326257, 0.417937, -0.252944, -0.007059],  # Link 4
    #     [-0.392872, 0.085927, 0.496421, -0.095969, 0.411640],  # Link 5
    #     [-0.064587, 0.059342, -0.382597, -0.392241, -0.336571],  # Link 6
    #     [-0.315751, 0.435470, 0.269812, 0.198037, -0.260547]])  # Link 7
    # paper_params["q0"] = [-0.585082657000795, -0.17442114158226177, -0.3373499395822175, -1.8767831435745583, -1.0631895937715656, 1.791618825138081, -0.7284639814339579]  # Initial joint positions in radians

    # if collision_constraint(franka, fourier_decomposition, paper_params, T_period=21.0, resolution=100, verbose=True) < 0:
    #     print("Warning: Checking Collision at finer resolution has found a collision!")
    # exit(0)

    params = {
                "Nh": 5, # Number of harmonics
                "Nj": franka.numJoints, # Number of joints
            }
    
    joint_bounds = np.array([[-2.8973, 2.8973],  # joint1
                            [-1.7628, 1.7628],  # joint2
                            [-2.8973, 2.8973],  # joint3
                            [-3.0718, -0.0698],  # joint4
                            [-2.8973, 2.8973],  # joint5
                            [-0.0175, 3.7525],  # joint6
                            [-2.8973, 2.8973]]) # joint7

    speed_bounds = np.array([2.1750,2.1750,2.1750,2.1750,2.6100,2.6100,2.6100])         # Max joint speeds in rad/s
    q_lower_bounds = np.array([-2.8973,-1.7628,-2.8973,-3.0718, -2.8973, -0.0175, -2.8973])       # Min joint positions in radians
    q_upper_bounds = np.array([2.8973,1.7628,2.8973, -0.0698, 2.8973, 3.7525, 2.8973])     # Max joint positions in radians

    #solve(1024, params, franka, speed_bounds, fourier_decomposition, T_period=10.0, resolution=15)
    result, score, all = solve(2**12, params, franka, q_upper_bounds, q_lower_bounds, speed_bounds, fourier_decomposition, T_period=21.0, resolution=50 )


    import datetime
    date_str = datetime.datetime.now().strftime("%d_%m_%Y")
    for i, res in enumerate(all[:5]):
        if collision_constraint(franka, fourier_decomposition, result, T_period=21.0, resolution=1000, verbose=True) < 0:
            print("Warning: Checking Collision at finer resolution has found a collision!")
        else:
            print(f"Score: {res[1]:.4f} is collision free at finer resolution check.")
            path = f"/home/leo/Desktop/Base Inertial Parameter/src/examples/franka_testing/results/data_{date_str}/attempt_{i}"
            export_fourier_trajectory(path, res[0], res[1])



    #solve_SLSQP(params, franka, speed_bounds, fourier_decomposition, T_period=21.0, resolution=10)












    # # Set transformation
    # q = np.random.uniform(-1.0, 1.0, (7,))
    # #q = np.array([0.12834504, -0.92389165,  0.66622907,  0.18338204,  0.48880064, -0.11523212, 0.70985779])
    # franka.set_q(q)

    # T_EE = franka.get_T_w_7()
    # T_link = franka.get_T_w_5()

    # # Convert numpy matrices to Coal Transform objects
    # tf_ee = coal.Transform3f(T_EE[:3, :3], T_EE[:3, 3])
    # tf_link = coal.Transform3f(T_link[:3, :3], T_link[:3, 3])

    # # Query collision/distance
    # req = coal.CollisionRequest()
    # res = coal.CollisionResult()

    # coal.collide(mesh_ee, tf_ee, mesh_link6, tf_link, req, res)

    # # Accessing the collision result once it has been populated
    # print("Is collision? ", {res.isCollision()})
    # if res.isCollision():
    #     contact: coal.Contact = res.getContact(0)
    #     print("Penetration depth: ", contact.penetration_depth)
    #     print("Distance between the shapes including the security margin: ", contact.penetration_depth + req.security_margin)
    #     # print("Witness point shape1: ", contact.getNearestPoint1())
    #     # print("Witness point shape2: ", contact.getNearestPoint2())
    #     print("Position: ", contact.pos)
    #     print("Normal: ", contact.normal)
    
    # # Calculate distance
    # req = coal.DistanceRequest()
    # res = coal.DistanceResult()
    # dist = coal.distance(mesh_ee, tf_ee, mesh_link6, tf_link, req, res)


    # # --- Results ---
    # print(f"--- Collision Query ---")
    # print(f"Joint Position: {franka.get_q()}")
    # print(f"Minimum Distance: {dist:.4f} meters")
    # # Access nearest points
    # p_ee = res.getNearestPoint1()
    # p_link = res.getNearestPoint2()
    # print(f"Nearest point on Link 7: {p_ee}")
    # print(f"Nearest point on Link 6: {p_link}")
    # print(f"manual dist: {np.linalg.norm(p_ee - p_link)}")
    
    # # --- 3D Visualization ---
    # plot_3Dviz(franka, p_ee, p_link, dist, block=True)