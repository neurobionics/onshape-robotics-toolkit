import json
import os
from functools import partial

import mujoco
import mujoco.include
import mujoco.viewer
import numpy as np
import optuna
import plotly
from controllers import PIDController
from loguru import logger
from mods import modify_ballbot
from mujoco.usd import exporter
from scipy.spatial.transform import Rotation
from transformations import compute_motor_torques

from onshape_robotics_toolkit.connect import Client
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.document import Document
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot

N_DESIGN_TRAILS = 100
N_PID_TRAILS = 50

USE_MUJOCO_VIEWER = False

HEIGHT = 480
WIDTH = 640

FREQUENCY = 200
USD_FRAME_RATE = 25
dt = 1 / FREQUENCY

# Variable bounds (in mm and degrees)
WHEEL_DIAMETER_BOUNDS = (100, 150)
SPACER_HEIGHT_BOUNDS = (75, 150)
ALPHA_BOUNDS = (35, 50)
PLATE_BOUNDS = (1, 30)

LAMBDA_WEIGHT = 100.0
BETA_WEIGHT = 25.0

SIMULATION_DURATION = 120  # seconds to run each trial
VIBRATION_PENALTY = 1e-3

TARGET_VALUE = 50.0  # Exit control optimation if balanced for this long
PERTURBATION_INCREASE = 0.125  # Amount of Newtons to increase perturbation by each time
PERTURBATION_START = 5  # Time delay before perturbations begin
PERTURBATION_REST = 7  # Time delay between perturbations

MAX_ANGLE = np.deg2rad(60)
MAX_DISTANCE_FROM_BALL = 0.3  # meters

# PID parameters for roll, pitch, and yaw
KP = 13.4
KI = 5.4
KD = 2.4
FF = 0.2

DERIVATIVE_FILTER_ALPHA = 0.1

TORQUE_LIMIT_HIGH = 10.0
TORQUE_LIMIT_LOW = -10.0


def get_theta(data, degrees=False):
    rot = Rotation.from_quat(data.qpos[3:7], scalar_first=True)
    theta = rot.as_euler("XYZ", degrees=degrees)
    return theta[0], theta[1], theta[2]


def get_psi(data):
    return data.qpos[7], data.qpos[8], data.qpos[9]


def get_phi(data, degrees=False):
    rot = Rotation.from_quat(data.qpos[13:17], scalar_first=True)
    phi = rot.as_euler("XYZ", degrees=degrees)
    return phi[0], phi[1], phi[2]


def get_bot_pos(data):
    return data.qpos[0], data.qpos[1], data.qpos[2]


def get_ball_pos(data):
    return data.qpos[10], data.qpos[11], data.qpos[12]


def get_bot_vel(data):
    return data.qvel[0], data.qvel[1], data.qvel[2]


def get_dtheta(data):
    return data.qvel[3], data.qvel[4], data.qvel[5]


def get_dpsi(data):
    return data.qvel[6], data.qvel[7], data.qvel[8]


def get_ball_vel(data):
    return data.qvel[9], data.qvel[10], data.qvel[11]


def get_dphi(data):
    return data.qvel[12], data.qvel[13], data.qvel[14]


def get_states(data):
    theta = get_theta(data)
    phi = get_phi(data)

    dtheta = get_dtheta(data)
    dphi = get_dphi(data)

    return np.array([
        phi[0],
        theta[0],
        dphi[0],
        dtheta[0],
    ]), np.array([
        phi[1],
        theta[1],
        dphi[1],
        dtheta[1],
    ])


def apply_ball_torque(data, torque):
    data.qfrc_applied[12:15] = torque


def apply_ball_force(data, force):
    data.qfrc_applied[9:12] = force


def apply_wheel_torque(data, torque):
    data.qfrc_applied[6:9] = torque


def get_wheel_torque(data):
    return data.qfrc_smooth[6], data.qfrc_smooth[7], data.qfrc_smooth[8]


def exit_condition(data):
    _roll, _pitch, _yaw = get_theta(data)

    angle_condition = _roll > MAX_ANGLE or _pitch > MAX_ANGLE

    ball_pos = get_ball_pos(data)
    bot_pos = get_bot_pos(data)
    distance_between_ball_and_bot = np.linalg.norm(np.array(ball_pos) - np.array(bot_pos))

    distance_condition = distance_between_ball_and_bot > MAX_DISTANCE_FROM_BALL

    return angle_condition or distance_condition


def control(data, roll_pid, pitch_pid, variables: dict[str, float]):
    # Get current orientation
    theta = get_theta(data)

    # Calculate errors
    error_x = 0.0 - theta[0]
    error_y = 0.0 - theta[1]

    tx = roll_pid.update(error_x)
    ty = pitch_pid.update(error_y)
    tz = 0.0

    # Compute motor torques
    t1, t2, t3 = compute_motor_torques(np.deg2rad(variables["alpha"]), tx, ty, tz, theta[2])

    data.ctrl[0] = t1
    data.ctrl[1] = t2
    data.ctrl[2] = t3


def apply_perturbation(data, count):
    direction = np.random.rand(3)
    direction[2] = 0  # Only apply force in the x-y plane

    force = direction * count * PERTURBATION_INCREASE
    logger.info(f"Applying perturbation {count}: {force}")
    apply_ball_force(data, force)


def find_best_pid_params(trial, model, data, viewer, variables, usd_output_dir):
    # Suggest values for the PID gains
    kp = trial.suggest_float("kp", low=5, high=25.0, step=0.1)
    ki = trial.suggest_float("ki", low=0.0, high=15.0, step=0.1)
    kd = trial.suggest_float("kd", low=0.0, high=1.0, step=0.01)
    ff = trial.suggest_float("ff", low=0.01, high=1.01, step=0.05)

    logger.info(f"KP: {kp}, KI: {ki}, KD: {kd}, FF: {ff}")

    usd_exporter = exporter.USDExporter(
        model=model,
        output_directory=os.path.join(usd_output_dir, f"pid_{trial.number}"),
    )

    # Initialize the PID controllers with the suggested gains
    roll_pid = PIDController(
        kp=kp,
        ki=ki,
        kd=kd,
        dt=1.0 / FREQUENCY,
        min_output=TORQUE_LIMIT_LOW,
        max_output=TORQUE_LIMIT_HIGH,
        feed_forward_offset=ff,
        derivative_filter_alpha=DERIVATIVE_FILTER_ALPHA,
    )
    pitch_pid = PIDController(
        kp=kp,
        ki=ki,
        kd=kd,
        dt=1.0 / FREQUENCY,
        min_output=TORQUE_LIMIT_LOW,
        max_output=TORQUE_LIMIT_HIGH,
        feed_forward_offset=ff,
        derivative_filter_alpha=DERIVATIVE_FILTER_ALPHA,
    )

    # Reset the simulation
    mujoco.mj_resetData(model, data)
    roll_pid.reset()
    pitch_pid.reset()

    # Initialize variables to track distance
    cumulative_distance = 0.0
    cumulative_vibration = 0.0
    steps = 0

    # Run the simulation
    j = 0

    while data.time < SIMULATION_DURATION:
        mujoco.mj_step(model, data)

        if usd_exporter.frame_count < data.time * USD_FRAME_RATE:
            usd_exporter.update_scene(data=data)

        if data.time > 0.3:
            control(data, roll_pid, pitch_pid, variables)

        if data.time > PERTURBATION_START + j * PERTURBATION_REST:
            j += 1
            apply_perturbation(data, j)

        if exit_condition(data):
            break

        ball_pos = get_ball_pos(data)
        distance = np.linalg.norm(np.array(ball_pos))
        cumulative_distance += distance

        dtheta = get_dtheta(data)
        cumulative_vibration += np.linalg.norm(dtheta)
        steps += 1

        if USE_MUJOCO_VIEWER:
            viewer.sync()
        elif viewer.is_running():
            viewer.close()

    # Combine performance metric with vibration penalty and distance
    time_on_ball = data.time  # Time the ball stayed on top
    average_distance = cumulative_distance / steps if steps > 0 else 0.0
    average_vibration = cumulative_vibration / steps if steps > 0 else 0.0
    objective_value = time_on_ball - LAMBDA_WEIGHT * average_distance - BETA_WEIGHT * average_vibration

    logger.info(
        f"Time on Ball: {time_on_ball}, "
        f"Average Distance: {average_distance}, "
        f"Average Vibration: {average_vibration}, "
        f"Objective: {objective_value}"
    )

    usd_exporter.save_scene(filetype="usd")

    return objective_value


def stop_when_target_reached(study, trial):
    if trial.value is not None and trial.value >= TARGET_VALUE:
        study.stop()


def find_best_design_variables(trial):
    # reset global PID error values
    wheel_diameter = trial.suggest_float("wheel_diameter", WHEEL_DIAMETER_BOUNDS[0], WHEEL_DIAMETER_BOUNDS[1])
    spacer_height = trial.suggest_float("spacer_height", SPACER_HEIGHT_BOUNDS[0], SPACER_HEIGHT_BOUNDS[1])
    alpha = trial.suggest_float("alpha", ALPHA_BOUNDS[0], ALPHA_BOUNDS[1])
    plate_thickness = trial.suggest_float("plate_thickness", PLATE_BOUNDS[0], PLATE_BOUNDS[1])

    variables["wheel_diameter"].expression = f"{wheel_diameter:.1f} mm"
    variables["alpha"].expression = f"{alpha:.1f} deg"
    variables["spacer_height"].expression = f"{spacer_height:.1f} mm"
    variables["plate_thickness"].expression = f"{plate_thickness:.1f} mm"

    variables_to_set = {
        "wheel_diameter": variables["wheel_diameter"].expression,
        "alpha": variables["alpha"].expression,
        "spacer_height": variables["spacer_height"].expression,
        "plate_thickness": variables["plate_thickness"].expression,
    }

    client.set_variables(doc.did, doc.wid, elements["variables"].id, variables_to_set)
    assembly = client.get_assembly(doc.did, doc.wtype, doc.wid, doc.eid)

    cad = CAD.from_assembly(assembly, max_depth=1, client=client)
    graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
    ballbot = Robot.from_graph(kinematic_graph=graph, client=client, name="ballbot")

    ballbot.set_robot_position(pos=(0, 0, 0.35))
    ballbot = modify_ballbot(ballbot)
    ballbot.save("ballbot.xml")

    model = mujoco.MjModel.from_xml_path(filename="ballbot.xml")
    data = mujoco.MjData(model)
    viewer = mujoco.viewer.launch_passive(model, data)
    mujoco.mj_resetData(model, data)

    if not USE_MUJOCO_VIEWER:
        viewer.close()

    # find the best PID parameters
    this_pid_study = partial(
        find_best_pid_params,
        model=model,
        data=data,
        viewer=viewer,
        variables={
            "wheel_diameter": wheel_diameter,
            "spacer_height": spacer_height,
            "alpha": alpha,
            "plate_thickness": plate_thickness,
        },
        usd_output_dir=os.path.join(output_dir, "scenes", f"trial_{trial.number}"),
    )
    pid_study = optuna.create_study(directions=["maximize"])
    pid_study.optimize(this_pid_study, n_trials=N_PID_TRAILS, callbacks=[stop_when_target_reached])
    viewer.close()

    if pid_study.best_params is None:
        logger.error("No best trial found")
        kp = KP
        ki = KI
        kd = KD
        ff = FF
    else:
        logger.info(f"Best PID params: {pid_study.best_params}")
        kp = pid_study.best_params["kp"]
        ki = pid_study.best_params["ki"]
        kd = pid_study.best_params["kd"]
        ff = pid_study.best_params["ff"]

    # Store PID values in trial user attributes
    trial.set_user_attr("kp", kp)
    trial.set_user_attr("ki", ki)
    trial.set_user_attr("kd", kd)
    trial.set_user_attr("ff", ff)

    best_roll_pid = PIDController(
        kp=kp,
        ki=ki,
        kd=kd,
        dt=1.0 / FREQUENCY,
        min_output=TORQUE_LIMIT_LOW,
        max_output=TORQUE_LIMIT_HIGH,
        feed_forward_offset=ff,
        derivative_filter_alpha=DERIVATIVE_FILTER_ALPHA,
    )
    best_pitch_pid = PIDController(
        kp=kp,
        ki=ki,
        kd=kd,
        dt=1.0 / FREQUENCY,
        min_output=TORQUE_LIMIT_LOW,
        max_output=TORQUE_LIMIT_HIGH,
        feed_forward_offset=ff,
        derivative_filter_alpha=DERIVATIVE_FILTER_ALPHA,
    )

    mujoco.mj_resetData(model, data)

    usd_exporter = exporter.USDExporter(
        model=model,
        output_directory=os.path.join(output_dir, "scenes", f"trial_{trial.number}"),
    )

    j = 0
    viewer = mujoco.viewer.launch_passive(model, data)
    # Reset data for a new trial
    mujoco.mj_resetData(model, data)

    if not USE_MUJOCO_VIEWER:
        viewer.close()

    # Initialize variables to track distance
    cumulative_distance = 0.0
    cumulative_vibration = 0.0
    steps = 0

    # while data.time < SIMULATION_DURATION and viewer.is_running():
    while data.time < SIMULATION_DURATION:
        mujoco.mj_step(model, data)

        if usd_exporter.frame_count < data.time * USD_FRAME_RATE:
            usd_exporter.update_scene(data=data)

        if data.time > 0.3:
            control(
                data,
                best_roll_pid,
                best_pitch_pid,
                {
                    "alpha": alpha,
                    "wheel_diameter": wheel_diameter,
                    "spacer_height": spacer_height,
                    "plate_thickness": plate_thickness,
                },
            )

        if data.time > PERTURBATION_START + j * PERTURBATION_REST:
            j += 1
            apply_perturbation(data, j)

        if exit_condition(data):
            break

        ball_pos = get_ball_pos(data)
        distance = np.linalg.norm(np.array(ball_pos))
        cumulative_distance += distance

        dtheta = get_dtheta(data)
        cumulative_vibration += np.linalg.norm(dtheta)
        steps += 1

        if USE_MUJOCO_VIEWER:
            viewer.sync()
        elif viewer.is_running():
            viewer.close()

    time_on_ball = data.time  # Time the ball stayed on top
    average_distance = cumulative_distance / steps if steps > 0 else 0.0
    average_vibration = cumulative_vibration / steps if steps > 0 else 0.0

    objective_value = time_on_ball - LAMBDA_WEIGHT * average_distance - BETA_WEIGHT * average_vibration

    logger.info(
        f"Time on Ball: {time_on_ball}, "
        f"Average Distance: {average_distance}, "
        f"Average Vibration: {average_vibration}, "
        f"Objective: {objective_value}"
    )

    if viewer.is_running():
        viewer.close()

    usd_exporter.save_scene(filetype="usd")

    return objective_value


if __name__ == "__main__":
    import sys

    run_name = input("Enter run name: ")
    # Create output directory for this run
    output_dir = run_name
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(os.path.join(output_dir, "scenes"), exist_ok=True)  # Create scenes subdirectory
    # Update log file path
    logger.remove()
    logger.add(os.path.join(output_dir, f"{run_name}.log"), rotation="10 MB", level="DEBUG")
    logger.add(sys.stderr, level="INFO")

    client = Client(env=".env")
    doc = Document.from_url(
        url="https://cad.onshape.com/documents/01d73bbd0f243938a11fbb7c/w/20c6ecfe7711055ba2420fdc/e/833959fcd6ba649195a1e94c"
    )

    elements = client.get_elements(doc.did, doc.wtype, doc.wid)
    variables = client.get_variables(doc.did, doc.wid, elements["variables"].id)

    study = optuna.create_study(direction="maximize")
    study.optimize(find_best_design_variables, n_trials=N_DESIGN_TRAILS)

    logger.info("\nOptimization finished!")
    logger.info("Best trial:")
    logger.info(f"  Value: {study.best_trial.value}")
    logger.info("  Params:")
    for key, value in study.best_trial.params.items():
        logger.info(f"    {key}: {value}")
    logger.info("  PID values:")
    logger.info(f"    kp: {study.best_trial.user_attrs['kp']}")
    logger.info(f"    ki: {study.best_trial.user_attrs['ki']}")
    logger.info(f"    kd: {study.best_trial.user_attrs['kd']}")
    logger.info(f"    ff: {study.best_trial.user_attrs['ff']}")

    # Save outputs in the run directory
    with open(os.path.join(output_dir, "best_params.json"), "w") as f:
        # Combine design parameters and PID values
        all_params = {
            **study.best_trial.params,
            "pid": {
                "kp": study.best_trial.user_attrs["kp"],
                "ki": study.best_trial.user_attrs["ki"],
                "kd": study.best_trial.user_attrs["kd"],
                "ff": study.best_trial.user_attrs["ff"],
            },
        }
        json.dump(all_params, f)

    study.trials_dataframe().to_csv(os.path.join(output_dir, "data.csv"))

    # Save visualization plots
    contour_plot = optuna.visualization.plot_contour(study)
    plotly.io.write_html(contour_plot, os.path.join(output_dir, "contour.html"))

    optimization_history_plot = optuna.visualization.plot_optimization_history(study)
    plotly.io.write_html(optimization_history_plot, os.path.join(output_dir, "optimization_history.html"))

    hyperparameter_importance_plot = optuna.visualization.plot_param_importances(study)
    plotly.io.write_html(hyperparameter_importance_plot, os.path.join(output_dir, "hyperparameter_importance.html"))

    timeline_plot = optuna.visualization.plot_timeline(study)
    plotly.io.write_html(timeline_plot, os.path.join(output_dir, "timeline.html"))

    parallel_coordinate_plot = optuna.visualization.plot_parallel_coordinate(study)
    plotly.io.write_html(parallel_coordinate_plot, os.path.join(output_dir, "parallel_coordinate.html"))
