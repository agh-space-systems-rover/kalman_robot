def load_images(folder_path):
    import os
    from PIL import Image

    valid_extensions = ('.jpg', '.jpeg', '.png', '.gif', '.bmp')
    image_paths = []

    for filename in os.listdir(folder_path):
        if filename.lower().endswith(valid_extensions):
            image_paths.append(os.path.join(folder_path, filename))

    return image_paths