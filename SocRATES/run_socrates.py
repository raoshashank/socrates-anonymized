import os, yaml,json
from automation_utils import *
from termcolor import cprint
from argparse import ArgumentParser
MAX_RETRIES_PER_SCENARIO = 3
def main(mode):
    # assert os.getcwd() == SocRATES_DIRECTORY, f"Expected working directory to be {SocRATES_DIRECTORY}, but got {os.getcwd()}
    with open('inputs.yaml', 'r') as f:
        inputs = yaml.safe_load(f)
    with open('location_descriptions.yaml', 'r') as f:
        location_descriptions = yaml.safe_load(f)
    COLCON_WS_DIRECTORY = inputs['colcon_ws_directory']    
    experiment_name = inputs['experiment_name']
    world = inputs['location']['name']
    inputs['experiment_name'] = experiment_name
    inputs['context'] = inputs['context']
    inputs['location'] = location_descriptions[world]
    experiment_dir = os.path.join(os.getcwd(), 'scenarios','experiments',experiment_name)
    inputs['paths']['save_dir'] = experiment_dir
    os.makedirs(experiment_dir, exist_ok=True)
    LOAD = [inputs['load_scenario_response'], inputs['load_trajectory_response'], inputs['load_bt_response']]
    # print(mode in ['gen','both'])
    success, outputs = run_socrates(
        inputs = inputs,
        colcon_ws_directory=COLCON_WS_DIRECTORY,
        experiment_dir=experiment_dir,
        skip_generation= mode == 'sim',
        skip_simulation= mode == 'gen',
        load= all(LOAD) or mode == 'sim',
        max_retries=MAX_RETRIES_PER_SCENARIO
    )
    if not LOAD:
        if outputs:
            print("Saved Scenario outputs")
            with open(os.path.join(experiment_dir, 'scenario_generation_outputs.json'), 'w') as f:
                json.dump(outputs, f, indent=4)

if __name__ == "__main__":
    args = ArgumentParser()
    args.add_argument('--mode', type=str, required=True, help='gen/sim/both')
    parsed_args = args.parse_args()
    if parsed_args.mode not in ['gen','sim','both']:
        raise ValueError("Invalid mode. Choose from 'gen', 'sim', 'both'")
    main(parsed_args.mode)
