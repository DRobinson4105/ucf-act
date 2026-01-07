import os
import argparse

import cv2
import numpy as np

NUM_CLASSES = 10
CLASS_COLORS = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
    (255, 255, 0),
    (255, 0, 255),
    (0, 255, 255),
    (255, 128, 0),
    (255, 0, 128),
    (128, 255, 0),
    (0, 255, 128)
]
LABELS = [
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "bus",
    "truck",
    "golf_cart",
    "scooter",
    "bicycle_rack",
    "scooter_rack"
]
ZOOM_STEP = 1.2
MIN_SCALE = 1.0
MAX_SCALE = 8.0

config = {
    "current_class": 0,
    "img": None,
    "label": [],
    "overlay_vis": None,
    "show": True,
    "scale": 1.0,
    "offset_x": 0.0,
    "offset_y": 0.0,
    "width": 0,
    "height": 0,
    "curr_box": [],
    "active_pixel": (0, 0),
}

def display_to_image_coords(xd, yd):
    return xd / config["scale"] + config["offset_x"], yd / config["scale"] + config["offset_y"]

def image_to_display_coords(xi, yi):
    return int((xi - config["offset_x"]) * config["scale"]), int((yi - config["offset_y"]) * config["scale"])

def render_overlay():
    h, w = config["height"], config["width"]

    base = config["img"].copy()

    if config["show"]:
        for box in config["label"]:
            c, bx, by, bw, bh = box

            x1 = round(bx - bw / 2)
            x2 = round(bx + bw / 2)
            y1 = round(by - bh / 2)
            y2 = round(by + bh / 2)

            cv2.rectangle(base, (x1, y1), (x2, y2), CLASS_COLORS[c], thickness=1)

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 1
            _, baseline = cv2.getTextSize(LABELS[c], font, font_scale, thickness)

            cv2.putText(base, LABELS[c], (x1, y1 - baseline), font, font_scale, CLASS_COLORS[c], thickness, cv2.LINE_AA)

    base[np.clip(config["active_pixel"][1], 0, h-1), np.clip(config["active_pixel"][0], 0, w-1)] = (255, 255, 255)

    view_w = w / config["scale"]
    view_h = h / config["scale"]

    offset_x = config["offset_x"] = max(0, min(config["offset_x"], w - view_w))
    offset_y = config["offset_y"] = max(0, min(config["offset_y"], h - view_h))

    x0 = int(round(offset_x))
    y0 = int(round(offset_y))
    x1 = int(round(offset_x + view_w))
    y1 = int(round(offset_y + view_h))

    x0 = max(0, min(x0, w - 1))
    y0 = max(0, min(y0, h - 1))
    x1 = max(x0 + 1, min(x1, w))
    y1 = max(y0 + 1, min(y1, h))

    crop = base[y0:y1, x0:x1]

    config["overlay_vis"] = cv2.resize(
        crop,
        (w, h),
        interpolation=cv2.INTER_NEAREST
    )

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEWHEEL:
        if flags > 0:
            new_scale = min(MAX_SCALE, config["scale"] * ZOOM_STEP)
        else:
            new_scale = max(MIN_SCALE, config["scale"] / ZOOM_STEP)

        if new_scale == config["scale"]:
            return

        xi, yi = display_to_image_coords(x, y)

        config["scale"] = new_scale

        config["offset_x"] = xi - x / config["scale"]
        config["offset_y"] = yi - y / config["scale"]

        render_overlay()
    else:
        xi, yi = display_to_image_coords(x, y)
        xi = int(round(xi))
        yi = int(round(yi))
        config["active_pixel"] = (xi, yi)
        render_overlay()

        if event == cv2.EVENT_LBUTTONDOWN:
            if len(config["curr_box"]) <= 1:
                config["curr_box"].append((xi, yi))
            else:
                config["curr_box"] = [(xi, yi)]
            render_overlay()

def main(args):
    image = args.image
    init_label = args.init_label
    save_label = args.save_label
    
    if not init_label and not save_label:
        raise RuntimeError("init_label_path and save_label_path cannot both be None.")

    config["img"] = cv2.imread(image)[..., ::-1]
    if config["img"] is None:
        raise RuntimeError(f"Failed to read {image}")
    
    h, w, _ = config["height"], config["width"], _  = config["img"].shape
    
    label = []
    if init_label:
        with open(init_label) as fp:
            for line in fp:
                parts = line.split()
                label.append([int(parts[0]), float(parts[1]) * w, float(parts[2]) * h, float(parts[3]) * w, float(parts[4]) * h])

    if not save_label:
        save_label = init_label

    config["label"] = label

    render_overlay()

    win_name = "Detection Editor"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, config["width"], config["height"])
    cv2.setMouseCallback(win_name, on_mouse)

    print("Controls:")
    print("Left mouse: paint current class")
    print("1-7: Select class")
    print("s: save")
    print("c: add current bounding box")
    print("z: undo last bounding box")
    print("f: toggle bounding box overlay")

    while True:
        vis = config["overlay_vis"].copy()

        cv2.putText(vis, f"class={LABELS[config['current_class']]}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis, f"scale={config['scale']:.2f}", (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow(win_name, vis[:, :, ::-1])
        key = cv2.waitKey(10) & 0xFF

        if key == ord('s'):
            os.makedirs(os.path.dirname(save_label), exist_ok=True)
            with open(save_label, 'w', newline="\n") as fp:
                for r in config["label"]:
                    fp.write(f"{int(r[0])} {r[1] / w:.6f} {r[2] / h:.6f} {r[3] / w:.6f} {r[4] / h:.6f}\n")
            break
        elif key in [ord(str(d)) for d in range(10)]:
            current_class = (int(chr(key)) + 9) % 10
            config["current_class"] = min(max(current_class, 0), NUM_CLASSES - 1)
        elif key == ord('c') or key == ord('C'):
            if len(config["curr_box"]) == 2:
                (x1, y1), (x2, y2) = config["curr_box"]
                config["label"].append([
                    config["current_class"],
                    (x1 + x2) / 2,
                    (y1 + y2) / 2,
                    abs(x1 - x2),
                    abs(y1 - y2)
                ])
                render_overlay()
        elif key == ord('z') or key == ord('Z'):
            if len(config["label"]) > 0:
                config["label"].pop()
                render_overlay()
        elif key == ord('f') or key == ord('F'):
            config["show"] = not config["show"]
            render_overlay()

    cv2.destroyWindow(win_name)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--image", type=str)
    parser.add_argument("--init-label", type=str)
    parser.add_argument("--save-label", type=str)

    args = parser.parse_args()

    main(args)
