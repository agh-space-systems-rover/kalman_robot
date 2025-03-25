import os


def find_colors_in_file(file_path):
    """
    Opens a file and looks for specific color names (red, green, blue, white),
    preserving their order of appearance.

    Args:
        file_path (str): Path to the file to analyze

    Returns:
        tuple: A tuple containing the color names in order of appearance
    """
    target_colors = ["red", "green", "blue", "white"]
    found_colors = []

    try:
        with open(file_path, "r") as file:
            content = file.read().lower()

            # Track position of each found color
            color_positions = {}

            for color in target_colors:
                pos = content.find(color)
                if pos != -1:
                    color_positions[color] = pos

            # Sort colors by their position in the file
            found_colors = sorted(
                color_positions.keys(), key=lambda c: color_positions[c]
            )

        return tuple(found_colors)

    except Exception as e:
        print(f"Error reading file {file_path}: {e}")
        return tuple()


def get_color_emoji_cube(file_path):
    cube_colors = find_colors_in_file(file_path)

    color_str_cubes = ""
    for color in cube_colors:
        if color == "red":
            color_str_cubes += "🟥"
        elif color == "green":
            color_str_cubes += "🟩"
        elif color == "blue":
            color_str_cubes += "🟦"
        elif color == "white":
            color_str_cubes += "⬜"
    return color_str_cubes


def color_lens(file_path, color_filter):
    """
    Checks if a file contains at least one of the specified colors that are enabled in the filter.

    Args:
        file_path (str): Path to the file to analyze
        color_filter (dict): Dictionary with color names as keys and boolean values indicating if they're enabled

    Returns:
        bool: True if the file contains at least one enabled color, False otherwise
    """
    # Get all colors found in the file
    found_colors = find_colors_in_file(file_path)

    # Check if any enabled color in the filter exists in the file
    for color, is_enabled in color_filter.items():
        if is_enabled and color in found_colors:
            return True

    return False


def get_image_array(folder_path, color_filter, sort_method, ascending):

    supported_extensions = (".jpg", ".jpeg", ".png", ".gif")
    images = [
        os.path.join(folder_path, f)
        for f in os.listdir(folder_path)
        if f.lower().endswith(supported_extensions)
        and (
            color_lens(
                os.path.join(folder_path, f).replace(".jpg", ".txt"),
                color_filter,
            )
            or not (
                color_filter["red"]
                or color_filter["green"]
                or color_filter["blue"]
                or color_filter["white"]
            )
        )
    ]
    print(sort_method)
    # Sort images based on selected color if only one color filter is active
    active_colors = [color for color, is_active in color_filter.items() if is_active]

    def get_match_value(img_path):
        if sort_method == "timestamp":
            # Extract timestamp from filename
            # Format: cube-<epoch>-<...>.jpg
            try:
                return int(img_path.split("-")[1])
            except (ValueError, IndexError):
                return 0

        txt_path = img_path.replace(".jpg", ".txt")
        try:
            with open(txt_path, "r") as file:
                for line in file:
                    parts = line.strip().split()
                    if len(parts) >= 2 and parts[0].lower() in [
                        "red",
                        "green",
                        "blue",
                        "white",
                    ]:
                        if (
                            len(active_colors) == 1
                            and parts[0].lower() != active_colors[0]
                        ):
                            continue

                        # Return the first numerical value after color name
                        try:
                            if sort_method == "accuracy":
                                return float(parts[1])
                            elif sort_method == "distance":
                                return (
                                    float(parts[2]) ** 2
                                    + float(parts[3]) ** 2
                                    + float(parts[4]) ** 2
                                )
                            else:
                                return 0.0
                        except (ValueError, IndexError):
                            pass
            return 0.0  # Default value if no valid match value found
        except Exception:
            return 0.0  # Default if file can't be read

    images.sort(key=get_match_value, reverse=not ascending)

    return images
