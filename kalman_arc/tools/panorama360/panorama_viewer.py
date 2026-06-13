import os
import sys
import webbrowser
from threading import Timer

from flask import Flask, render_template, send_file

app = Flask(__name__)
IMAGE_PATH = ""


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/image")
def get_image():
    return send_file(IMAGE_PATH)


def open_browser():
    webbrowser.open_new("http://127.0.0.1:5000/")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python panorama_viewer.py <path_to_panorama.jpg>")
        sys.exit(1)

    IMAGE_PATH = os.path.abspath(sys.argv[1])

    if not os.path.exists(IMAGE_PATH):
        print(f"Error: File '{IMAGE_PATH}' not exists.")
        sys.exit(1)

    print(f"Opening: {IMAGE_PATH}")
    print("Opening browser...")

    Timer(1, open_browser).start()

    app.run(host="127.0.0.1", port=5000)
