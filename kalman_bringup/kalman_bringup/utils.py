import importlib.util

from launch import LaunchDescription

# Extract arguments declared in a launch file
def get_arg_decls(launch_file: str):
    # Import generate_launch_description() from the specified launch file
    import_name = launch_file.split("/share/")[-1].replace("/", ".")[:-10]
    spec = importlib.util.spec_from_file_location(f"{import_name}", launch_file)
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)

    # Generate the launch description
    launch_desc: LaunchDescription = launch_module.generate_launch_description()

    # Extract the arguments from the launch description
    args: list[tuple[str, str]] = []  # name: desc
    for action in launch_desc.get_launch_arguments():
        args.append((action.name, action.description, action.choices))

    return args


# See if a kalman_ launch file is composable
def is_kalman_composable(launch_file: str):
    args = get_arg_decls(launch_file)

    # Check if there's a component_container arg
    for arg_name, _, _ in args:
        if arg_name == "component_container":
            return True
