import os
from ament_index_python import (
    get_package_share_path,
    PackageNotFoundError,
    get_search_paths,
)

from kalman_bringup.gen_launch import get_arg_decls


if __name__ == "__main__":
    modules = [(x.split("/")[-1]) for x in get_search_paths()]

    launches_args = {}

    for module in modules:
        name = module if not module.startswith("kalman_") else module[7:]
        try:
            launch_name = f"{name}.launch.py"
            launch_path = str(
                get_package_share_path(f"{module}") / "launch" / f"{launch_name}"
            )
        except PackageNotFoundError:
            print(f"{module} is not a package")
            continue

        if os.path.exists(launch_path):
            launches_args[name] = [name for name, _ in get_arg_decls(launch_path)]
            if len(launches_args[name]) == 0:
                launches_args[name] = [""]
            # print(launch_path)
            # print(get_arg_decls(launch_path))
            # print(is_kalman_composable(launch_path))
            # print()
        # else:
        #     print(f"No such file: {launch_path}")
        # print()

    # print(launches_args)

    print()
    keys = list(launches_args.keys())

    print(f"Keys = Literal{keys}")

    second_keys = [
        f"{str(key).capitalize()}Arguments = dict[Literal{value}, object]"
        for key, value in launches_args.items()
    ]

    print("\n".join(second_keys))

    arguments = []
    for key in keys:
        arguments.append(f"'{key}': {key.capitalize()}Arguments")

    print(f"BringupType = TypedDict('BringupType', {{{', '.join(arguments)}}})")
