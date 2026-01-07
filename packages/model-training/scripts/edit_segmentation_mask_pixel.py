import os
import argparse

import cv2
import numpy as np

NUM_CLASSES = 2
CLASS_COLORS = [
    (50, 209, 50), # drivable
    (255, 0, 0) # non-drivable
]
LABELS = [
    "drivable",
    "non-drivable"
]
ZOOM_STEP = 1.2
MIN_SCALE = 1.0
MAX_SCALE = 8.0

config = {
    "drawing": False,
    "current_class": 0,
    "img": None,
    "label": None,
    "overlay_vis": None,
    "scale": 1.0,
    "offset_x": 0.0,
    "offset_y": 0.0,
    "show_seg": True,
    "width": 0,
    "height": 0,
    "brush_radius": 5
}

def display_to_image_coords(xd, yd):
    return xd / config["scale"] + config["offset_x"], yd / config["scale"] + config["offset_y"]

def image_to_display_coords(xi, yi):
    return int((xi - config["offset_x"]) * config["scale"]), int((yi - config["offset_y"]) * config["scale"])

def render_overlay():
    h, w = config["height"], config["width"]

    color_mask = np.zeros((h, w, 3), dtype=np.uint8)
    for cid in range(NUM_CLASSES):
        color_mask[config["label"] == cid] = CLASS_COLORS[cid % len(CLASS_COLORS)]
    base = cv2.addWeighted(config["img"], 0.5, color_mask[:, :, ::-1], 0.5, 0)

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

        if event == cv2.EVENT_LBUTTONDOWN:
            config["drawing"] = True
            cv2.circle(config["label"], (xi, yi), config["brush_radius"], config["current_class"], -1)
            render_overlay()
        elif event == cv2.EVENT_MOUSEMOVE and config["drawing"]:
            cv2.circle(config["label"], (xi, yi), config["brush_radius"], config["current_class"], -1)
            render_overlay()
        elif event == cv2.EVENT_LBUTTONUP:
            config["drawing"] = False
            cv2.circle(config["label"], (xi, yi), config["brush_radius"], config["current_class"], -1)
            render_overlay()

def main(args):
    image = args.image
    init_label = args.init_label
    save_label = args.save_label
    
    if not init_label and not save_label:
        raise RuntimeError("init_label_path and save_label_path cannot both be None.")
    
    show_seg = True

    config["img"] = cv2.imread(image)
    if config["img"] is None:
        raise RuntimeError(f"Failed to read {image}")
    
    h, w, _ = config["height"], config["width"], _  = config["img"].shape
    
    if init_label:
        label_img = cv2.imread(init_label)
        if label_img is None:
            raise RuntimeError(f"Failed to read {init_label}")
    else:
        label_img = np.zeros((config["height"], config["width"]))

    if not save_label:
        save_label = init_label

    if label_img.ndim == 3:
        label_img = label_img[:, :, 0]
    label_img = label_img.astype(np.uint8)

    label_img[label_img > 0] = 1

    config["label"] = label_img.copy()

    render_overlay()

    win_name = "Segmentation Editor"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, config["width"], config["height"])
    cv2.setMouseCallback(win_name, on_mouse)

    print("Controls:")
    print("Left mouse: paint current class")
    print("1: Select drivable class")
    print("2: Select non-drivable class")
    print("+/-: Change brush size")
    print("s: save")

    while True:
        if show_seg:
            vis = config["overlay_vis"].copy()
        else:
            view_w = w / config["scale"]
            view_h = h / config["scale"]
            offset_x = config["offset_x"] = max(0, min(config["offset_x"], w - view_w))
            offset_y = config["offset_y"] = max(0, min(config["offset_y"], h - view_h))
            x0 = int(round(offset_x))
            y0 = int(round(offset_y))
            x1 = int(round(offset_x + view_w))
            y1 = int(round(offset_y + view_h))
            crop = config["img"][y0:y1, x0:x1]
            vis = cv2.resize(crop, (w, h), interpolation=cv2.INTER_NEAREST)

        cv2.putText(vis, f"class={LABELS[config['current_class']]}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis, f"brush={config['brush_radius']}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis, f"scale={config['scale']:.2f}", (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow(win_name, vis)
        key = cv2.waitKey(10) & 0xFF

        if key == ord('s') or key == ord('S'):
            os.makedirs(os.path.dirname(save_label), exist_ok=True)
            label = config["label"]
            label = cv2.normalize(label, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            cv2.imwrite(save_label, label)
            break
        elif key in [ord(str(d)) for d in range(10)]:
            current_class = int(chr(key)) - 1
            config["current_class"] = min(max(current_class, 0), NUM_CLASSES - 1)
        elif key == ord('+') or key == ord('='):
            config["brush_radius"] = min(100, config["brush_radius"] + 1)
        elif key == ord('-') or key == ord('_'):
            config["brush_radius"] = max(0, config["brush_radius"] - 1)
        elif key == ord('f') or key == ord('F'):
            show_seg = not show_seg

    cv2.destroyWindow(win_name)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--image", type=str)
    parser.add_argument("--init-label", type=str)
    parser.add_argument("--save-label", type=str)

    args = parser.parse_args()

    main(args)
