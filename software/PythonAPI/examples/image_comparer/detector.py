import cv2
import numpy as np
from typer import Typer
from pathlib import Path

APP = Typer()

DEFAULT_MATCH_LIMIT = 0.8
DEFAULT_RANSAC_THRESHOLD = 5
MAX_WIDTH = 1600
MAX_HEIGHT = 900
DEFAULT_PIXEL_UM = 1.0

def compare_images(img1_path: Path, img2_path: Path, match_limit: float):

    img1 = cv2.imread(img1_path, cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread(img2_path, cv2.IMREAD_GRAYSCALE)

    sift = cv2.SIFT_create(
        nfeatures=15000,
        contrastThreshold=0.01,
        edgeThreshold=5
    )

    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)

    # FLANN matcher
    index_params = dict(algorithm=1, trees=5)
    search_params = dict(checks=50)

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1, des2, k=2)

    good = []

    # Lowe ratio test
    for m, n in matches:
        if m.distance < match_limit * n.distance:
            good.append(m)

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good])

    # Estimate transform
    M, inliers = cv2.estimateAffinePartial2D(
        pts1,
        pts2,
        method=cv2.RANSAC,
        ransacReprojThreshold=DEFAULT_RANSAC_THRESHOLD
    )

    # Keep only inlier matches
    inlier_matches = []
    for i, m in enumerate(good):
        if inliers[i]:
            inlier_matches.append(m)

    inlier_matches = sorted(inlier_matches, key=lambda m: m.distance)

    return img1, img2, kp1, kp2, inlier_matches, M

def visualize_matches(img1, img2, kp1, kp2, matches):

    img1_color = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
    img2_color = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)


    h1, w1 = img1_color.shape[:2]
    h2, w2 = img2_color.shape[:2]

    max_h = max(h1, h2)
    max_w = max(w1, w2)

    # pad image 1 if needed
    if h1 < max_h:
        pad = max_h - h1
        img1_color = cv2.copyMakeBorder(img1_color, 0, pad, 0, 0, cv2.BORDER_CONSTANT)
    if w1 < max_w:
        pad = max_w - w1
        img1_color = cv2.copyMakeBorder(img1_color, 0, 0, pad, 0, cv2.BORDER_CONSTANT)

    # pad image 2 if needed
    if h2 < max_h:
        pad = max_h - h2
        img2_color = cv2.copyMakeBorder(img2_color, 0, pad, 0, 0, cv2.BORDER_CONSTANT)
    if w2 < max_h:
        pad = max_w - w2
        img2_color = cv2.copyMakeBorder(img2_color, 0, 0, pad, 0, cv2.BORDER_CONSTANT)

    combined = np.hstack((img1_color, img2_color))
    # Scale combined image if too large
    h, w = combined.shape[:2]
    scale = min(MAX_WIDTH / w, MAX_HEIGHT / h, 1.0)
    if scale < 1.0:
        combined = cv2.resize(combined, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        base = combined.copy()  # scaled base


    base = combined.copy()


    show_matches = True
    updated_matches = False


    window_name = "Feature Matches (m=toggle, q=quit)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    match_to_color = {}

    while True:

        # Get current window size
        if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
            break
        
        _, _, win_w, win_h = cv2.getWindowImageRect(window_name)

        # Scale base to fit window
        h, w = base.shape[:2]
        h_scale = win_h / h
        w_scale = win_w / w
        display = cv2.resize(base, (int(w * w_scale), int(h * h_scale)), interpolation=cv2.INTER_AREA)

        if show_matches and not updated_matches:
            left_img_width = win_w/2
            for m in matches[:100]:
                x1, y1 = kp1[m.queryIdx].pt
                x2, y2 = kp2[m.trainIdx].pt

                h, w = display.shape[:2]
                real_w = w-left_img_width

                y1_scale = h/h1
                y2_scale = h/h2

                x1_scale = real_w/w1
                x2_scale = real_w/w2

                pt1 = (int(x1 * x1_scale), int(y1 * y1_scale))
                pt2 = (int(x2 * x2_scale), int(y2 * y2_scale))


                pt2_shifted = (int(pt2[0] + left_img_width), int(pt2[1]))

                if m.queryIdx not in match_to_color:
                    match_to_color[m.queryIdx] = tuple(np.random.randint(0,255,3).tolist())
                color = match_to_color[m.queryIdx]

                cv2.circle(display, pt1, 4, color, -1)
                cv2.circle(display, pt2_shifted, 4, color, -1)
                cv2.line(display, pt1, pt2_shifted, color, 1)
            updated_matches = True

        cv2.imshow(window_name, display)

        key = cv2.waitKey(-1) & 0xFF

        if key == ord('m'):
            show_matches = not show_matches
            updated_matches = False

        if key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()


@APP.command()
def compare_images_script(img1_path: Path, img2_path: Path, visualize: bool = True, match_limit: float = DEFAULT_MATCH_LIMIT, pixel_to_um: float= DEFAULT_PIXEL_UM):
    img1, img2, kp1, kp2, matches, transform = compare_images(img1_path, img2_path, match_limit)

    # Translation in real-world units
    tx_um = transform[0, 2] * pixel_to_um
    ty_um = transform[1, 2] * pixel_to_um

    # Rotation (in degrees)
    # For partial affine (no shear, uniform scale), rotation angle is:
    theta_rad = np.arctan2(transform[1,0], transform[0,0])
    theta_deg = np.degrees(theta_rad)

    print("Matched features:", len(matches))
    print(f"Translation: x = {tx_um:.2f} µm, y = {ty_um:.2f} µm")
    print(f"Rotation: {theta_deg:.2f}°")

    if visualize:
        visualize_matches(img1, img2, kp1, kp2, matches)

if __name__ == "__main__":
    APP()

