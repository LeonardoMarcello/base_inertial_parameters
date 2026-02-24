from utils.loader_utils import *
from utils.identification_utils import *


class Identifier():
    def __init__(self, robot, config_path):
        self.robot = robot
        self.config_path = config_path

        self.config = {}
        self._load_config(config_path)

        self.trajectory = None
        self.metrics = None

    def init(self):
        """initialize trajectory"""
        # Load config
        config_traject = self.config['trajectory']
        config_processing = self.config['processing']

        # Load trajectory manager and data
        self.trajectory = TrajectoryManager(self.config_path)
        if self.config['trajectory']['type'] == 'ros2':
            self.trajectory.df = TrajectoryManager.read_bag(config_traject['bag'], config_traject['topic'], config_traject['joints'])
        elif self.config['trajectory']['type'] == 'csv':
            self.trajectory.df = TrajectoryManager.read_csv(config_traject['path'])

        #  process data
        config_traject_interval = config_traject.get('interval', (5,100))
        apply_decimation = config_processing.get('apply_decimation', False)

        self.trajectory.process(interval_min=config_traject_interval[0], interval_max=config_traject_interval[1], apply_decimation=apply_decimation)  


    def solve_base_parameter(self):
        """solve and store the identfied base dynamic parameters of the robot in the following fields:
             - robot.get_par_REG_red()
             - robot.get_par_Dl()
        """
        # set solver config
        method = self.config['identification'].get('method','OLS')
        conditioning_ratio = self.config['identification'].get('conditioning_ratio', None)
        # Solve identification of base parameters
        print(">> Base inertial Parameters Estimation")
        if method == 'OLS':
            hat_par_REG_red, metrics = solve_OLS(self.robot, self.trajectory, conditioning_ratio = conditioning_ratio)
        elif method == 'WLS':
            hat_par_REG_red, metrics = solve_WLS(self.robot, self.trajectory, conditioning_ratio = conditioning_ratio)
        elif method == 'OLS|prior':
            hat_par_REG_red, metrics = solve_OLS_with_prior(self.robot, self.trajectory, conditioning_ratio = conditioning_ratio)
        elif method == 'WLS|prior':
            hat_par_REG_red, metrics = solve_WLS_with_prior(self.robot, self.trajectory, conditioning_ratio = conditioning_ratio)
        # Set estimation
        n = self.robot.numJoints
        hat_pi_REG_red = hat_par_REG_red[:-n*(self.robot.Dl_order)]
        hat_pi_dl = hat_par_REG_red[-n*(self.robot.Dl_order):]
        self.robot.set_par_REG_red(hat_pi_REG_red)
        self.robot.set_par_Dl(hat_pi_dl)

        self.metrics = metrics

    def solve_full_dynamics(self):
        """solve and store the full identfied robot dynamics model in the following fields:
             - robot.get_par_REG()
             - robot.get_par_DYN()
             - robot.get_par_REG_red()
             [- robot.get_par_Ia()]
        """
        # Define and Solve Full dynamics
        print(">> Full inertial Parameters Estimation")
        sol = solve_dynamics(self.robot, self.config, sigma_pi = self.metrics['parameters covariance matrix'])
        results = sol[-1]
        # Assign estimation
        self.robot.set_par_REG(sol[0])
        if hasattr(self.robot,"set_par_Ia"):
            self.robot.set_par_Ia(sol[1])


        # Convert into dynamic and regressor parameterss
        self.robot.set_par_DYN(self.robot.get_reg2dyn())
        self.robot.set_par_REG_red(self.robot.get_reg2red())

        return results

    def export(self, path = None):
        """export results in thunder dynamics config yaml file"""
        # 1. Define and create the results directory
        if path is None:
            date_str = datetime.datetime.now().strftime("%d_%m_%Y")
            results_dir = f"results/data_{date_str}"
        else:
            results_dir = path

        if not os.path.exists(results_dir):
            os.makedirs(results_dir)

        # 2. Create identification_par.yaml
        config_path = os.path.join(results_dir, "identification_par.yaml")

        with open(config_path, "w") as f:
            # Add other parameters
            f.write(f"d3q: {[0 for _ in range(self.robot.numJoints)]}\n")
            f.write(f"d4q: {[0 for _ in range(self.robot.numJoints)]}\n")
            f.write(f"ddq: {[0 for _ in range(self.robot.numJoints)]}\n")
            f.write(f"ddqr: {[0 for _ in range(self.robot.numJoints)]}\n")
            f.write(f"dq: {[0 for _ in range(self.robot.numJoints)]}\n")
            f.write(f"dqr: {[0 for _ in range(self.robot.numJoints)]}\n")
            f.write(f"q: {[0 for _ in range(self.robot.numJoints)]}\n")
            f.write(f"w: {[0 for _ in range(6)]}\n")
            if hasattr(self.robot, "get_par_Ln2EE"):
                f.write(f"par_Ln2EE: {self.robot.get_par_Ln2EE().tolist()}\n")
            if hasattr(self.robot, "get_world2L0"):
                f.write(f"par_world2L0: {self.robot.get_world2L0().tolist()}\n")
            if hasattr(self.robot, "get_par_gravity"):
                f.write(f"par_gravity: {self.robot.get_par_gravity().tolist()}\n")

            # Add identified parameters
            f.write(f"par_DYN: {self.robot.get_par_DYN().tolist()}\n")
            f.write(f"par_REG: {self.robot.get_par_REG().tolist()}\n")
            f.write(f"par_Dl: {self.robot.get_par_Dl().tolist()}\n")
            f.write(f"par_REG_red: {self.robot.get_par_REG_red().tolist()}\n")
            if hasattr(self.robot, "get_par_Ia"):
                # Check for actuator inertia or additional parameters
                f.write(f"par_Ia: {self.robot.get_par_Ia().tolist()}\n")

        print(f"Estimated Dynamic Parameters written in '{config_path}'")


    def help():
        """print object usage"""
        # TO DO: Print usage decription
        return

    def _load_config(self, filename):
        """ load identification configuration """
        with open(filename, 'r') as f:
            try:
                self.config = yaml.load(f, Loader=SafeLoader)
            except yaml.YAMLError as e:
                raise ValueError(f"Error parsing YAML file: {e}")

        # --- 0. Top-Level Checks ---
        required_sections = ['robot', 'trajectory', 'processing', 'identification']
        for section in required_sections:
            if section not in self.config:
                raise KeyError(f"Missing required top-level section: '{section}'")

        # --- 1. Robot Checks ---
        config_robot = self.config['robot']
        if 'path' not in config_robot:
            raise KeyError("Missing 'path' in the 'robot' section.")
        if not os.path.isfile(config_robot['path']):
            # Using a warning instead of an error here just in case you are 
            # generating this file dynamically later in the pipeline.
            print(f"[WARNING] Robot parameter file not found at: {config_robot['path']}")

        # --- 2. Trajectory Checks ---
        config_traj = self.config['trajectory']
        if 'type' not in config_traj:
            config_traj['type'] = 'csv'
            print(f"[WARNING] Trajectory type not found. default type is csv.")

        if config_traj['type'] == 'ros2' and ('topic' not in config_traj or 'bag' not in config_traj):
            raise ValueError("'topic or bag name must be defined for a ros2 trajectory.")

        joints = config_traj.get('joints', [])
        if not isinstance(joints, list) or len(joints) == 0:
            raise ValueError("'trajectory.joints' must be a non-empty list of joint name strings.")

        # --- 3. Processing Checks ---
        config_proc = self.config['processing']
        if config_proc.get('cut_off_frequency', -1) <= 0:
            raise ValueError("'processing.cut_off_frequency' must be a positive number.")
        if not isinstance(config_proc.get('apply_decimation'), bool):
            raise TypeError("'processing.apply_decimation' must be a boolean (true/false).")
        if config_proc.get('apply_decimation') and config_proc.get('decimate_factor', 0) <= 1:
            raise ValueError("If 'apply_decimation' is true, 'decimate_factor' must be an integer > 1.")

        # --- 4. Identification Checks ---
        config_ident = self.config['identification']
        valid_methods = ['OLS', 'WLS', 'OLS|prior', 'WLS|prior']
        if config_ident.get('method') not in valid_methods:
            raise ValueError(f"'identification.method' must be one of {valid_methods}. Got: {config_ident.get('method')}")

        weight = config_ident.get('weight', {})
        links = weight.get('link', [])
        if not isinstance(links, list):
            raise TypeError("'identification.weight.link' must be a list of numbers.")

        # Cross-validation: Ensure we have a weight for every joint specified
        if len(links) != len(joints):
            raise ValueError(f"Mismatch: Found {len(joints)} joints but {len(links)} link weights. They must match.")

        print("[INFO] Configuration loaded and validated successfully!")
