import cv2
import numpy as np
import math
import copy
import time
import serial

# =========================================================
# UART SETUP
# =========================================================
ser = serial.Serial('/dev/serial0', 115200, timeout=1)
time.sleep(2)

# =========================================================
# ---------------- STRAIGHT LANE FUNCTIONS ----------------
# =========================================================

def preprocessing_straight(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gblur = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(gblur, 150, 255, cv2.THRESH_BINARY)[1]
    return thresh

def regionOfInterest(img, polygon):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, [np.array(polygon)], 255)
    return cv2.bitwise_and(img, mask)

def slopeIntercept(line):
    x1, y1 = line[0]
    x2, y2 = line[1]

    if (x2 - x1) == 0:
        return 999, 0

    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    return m, b

def removeCloseLines(linelist, m):
    linelist_copy = copy.deepcopy(linelist)

    for line in linelist:
        m1, _ = slopeIntercept(line)

        if abs(m - m1) <= 0.5:
            if line in linelist_copy:
                linelist_copy.remove(line)

    return linelist_copy

def calculate_angle_from_line(line, frame_width):

    if line is None:
        return 90

    x1, y1 = line[0]
    x2, y2 = line[1]

    mid_x = (x1 + x2) / 2
    frame_center = frame_width / 2

    deviation = mid_x - frame_center

    angle = deviation * 0.1

    servo_angle = 90 + angle

    servo_angle = max(0, min(180, servo_angle))

    return servo_angle

def lineDetection(img, masked_img, solid_prev, dashed_prev):

    img_copy = copy.deepcopy(img)
    height, width = masked_img.shape

    linesP = cv2.HoughLinesP(
        masked_img,
        1,
        np.pi/180,
        50,
        None,
        30,
        20
    )

    if linesP is None:
        return img_copy, solid_prev, dashed_prev

    linelist = linesP.tolist()
    linelist = [tuple((line[0][:2], line[0][2:])) for line in linelist]

    lengths = [math.dist(l[0], l[1]) for l in linelist]

    try:
        solid_line = linelist[lengths.index(max(lengths))]
        linelist.remove(solid_line)
    except:
        solid_line = solid_prev

    if solid_line is not None:

        m, b = slopeIntercept(solid_line)

        linelist = removeCloseLines(linelist, m)

        initial = (int((height*0.6 - b)/m), int(height*0.6))
        final = (int((height - b)/m), height)

        img_copy = cv2.line(
            img_copy,
            initial,
            final,
            (0,255,0),
            5
        )

    lengths = [math.dist(l[0], l[1]) for l in linelist]

    try:
        dashed_line = linelist[lengths.index(max(lengths))]
    except:
        dashed_line = dashed_prev

    if dashed_line is not None:

        m, b = slopeIntercept(dashed_line)

        initial = (int((height*0.6 - b)/m), int(height*0.6))
        final = (int((height - b)/m), height)

        img_copy = cv2.line(
            img_copy,
            initial,
            final,
            (0,0,255),
            5
        )

    return img_copy, solid_line, dashed_line

# =========================================================
# ---------------- CURVED LANE FUNCTIONS ----------------
# =========================================================

def preprocessing_curved(img):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    gblur = cv2.GaussianBlur(gray,(5,5),0)

    white_mask = cv2.threshold(
        gblur,
        200,
        255,
        cv2.THRESH_BINARY
    )[1]

    lower_yellow = np.array([0,100,100])
    upper_yellow = np.array([210,255,255])

    yellow_mask = cv2.inRange(
        hsv,
        lower_yellow,
        upper_yellow
    )

    mask = cv2.bitwise_or(
        white_mask,
        yellow_mask
    )

    return mask

def warp(img, src, dst, size):

    matrix = cv2.getPerspectiveTransform(src, dst)

    return cv2.warpPerspective(
        img,
        matrix,
        size
    )

def unwarp(img, src, dst, size):

    matrix = cv2.getPerspectiveTransform(dst, src)

    return cv2.warpPerspective(
        img,
        matrix,
        size
    )

def fitCurve(img):

    histogram = np.sum(
        img[img.shape[0]//2:,:],
        axis=0
    )

    midpoint = int(histogram.shape[0]/2)

    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    nwindows = 50
    margin = 100
    minpix = 50

    window_height = int(img.shape[0]/nwindows)

    y, x = img.nonzero()

    leftx_current = leftx_base
    rightx_current = rightx_base

    left_lane_indices = []
    right_lane_indices = []

    for window in range(nwindows):

        win_y_low = img.shape[0] - (window+1)*window_height
        win_y_high = img.shape[0] - window*window_height

        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin

        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        good_left = (
            (y >= win_y_low) &
            (y < win_y_high) &
            (x >= win_xleft_low) &
            (x < win_xleft_high)
        ).nonzero()[0]

        good_right = (
            (y >= win_y_low) &
            (y < win_y_high) &
            (x >= win_xright_low) &
            (x < win_xright_high)
        ).nonzero()[0]

        left_lane_indices.append(good_left)
        right_lane_indices.append(good_right)

        if len(good_left) > minpix:
            leftx_current = int(np.mean(x[good_left]))

        if len(good_right) > minpix:
            rightx_current = int(np.mean(x[good_right]))

    left_lane_indices = np.concatenate(left_lane_indices)
    right_lane_indices = np.concatenate(right_lane_indices)

    leftx = x[left_lane_indices]
    lefty = y[left_lane_indices]

    rightx = x[right_lane_indices]
    righty = y[right_lane_indices]

    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    return left_fit, right_fit

def findPoints(shape, left_fit, right_fit):

    ploty = np.linspace(
        0,
        shape[0]-1,
        shape[0]
    )

    left_fitx = (
        left_fit[0]*ploty**2 +
        left_fit[1]*ploty +
        left_fit[2]
    )

    right_fitx = (
        right_fit[0]*ploty**2 +
        right_fit[1]*ploty +
        right_fit[2]
    )

    pts_left = np.array([
        np.transpose(np.vstack([left_fitx, ploty]))
    ])

    pts_right = np.array([
        np.flipud(np.transpose(np.vstack([right_fitx, ploty])))
    ])

    return pts_left, pts_right

def fillCurves(shape, pts_left, pts_right):

    pts = np.hstack((pts_left, pts_right))

    img = np.zeros(
        (shape[0], shape[1], 3),
        dtype='uint8'
    )

    cv2.fillPoly(
        img,
        np.int_([pts]),
        (0,0,255)
    )

    return img

def calculate_steering_angle(
    left_fit,
    right_fit,
    img_width,
    img_height
):

    y = img_height - 1

    left_x = (
        left_fit[0]*y**2 +
        left_fit[1]*y +
        left_fit[2]
    )

    right_x = (
        right_fit[0]*y**2 +
        right_fit[1]*y +
        right_fit[2]
    )

    lane_center = (left_x + right_x) / 2
    car_center = img_width / 2

    offset = lane_center - car_center

    angle = np.arctan(offset / img_height) * (180 / np.pi)

    servo_angle = 100 - angle

    servo_angle = max(0, min(180, servo_angle))

    return round(servo_angle, 2)

# =========================================================
# ---------------- VIDEO SETUP ----------------
# =========================================================

straight_video = cv2.VideoCapture("straight_lane.mp4")
curved_video = cv2.VideoCapture("curved_lane.mp4")

solid_line_previous = None
dashed_line_previous = None

FRAME_TIME = 0.04

mode = "STRAIGHT"

print("Running Combined Lane Detection")

# =========================================================
# ---------------- MAIN LOOP ----------------
# =========================================================

while True:

    start = time.time()

    # =====================================================
    # ---------------- STRAIGHT LANE FIRST ----------------
    # =====================================================

    if mode == "STRAIGHT":

        ret, frame = straight_video.read()

        # Straight video finished -> switch to curved
        if not ret:
            mode = "CURVED"
            continue

        processed = preprocessing_straight(frame)

        h, w = processed.shape

        polygon = [
            (int(w*0.1), h),
            (int(w*0.45), int(h*0.6)),
            (int(w*0.55), int(h*0.6)),
            (int(0.95*w), h)
        ]

        masked = regionOfInterest(processed, polygon)

        detected, solid_line, dashed_line = lineDetection(
            frame,
            masked,
            solid_line_previous,
            dashed_line_previous
        )

        solid_line_previous = solid_line
        dashed_line_previous = dashed_line

        steering_angle = calculate_angle_from_line(
            solid_line,
            w
        )

        cv2.putText(
            detected,
            f"STRAIGHT MODE : {int(steering_angle)}",
            (40,50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255,255,255),
            2
        )

        # SEND UART
        try:
            ser.write(f"{int(steering_angle)}\n".encode())
        except:
            pass

        cv2.imshow("Lane Detection", detected)

    # =====================================================
    # ---------------- CURVED LANE LOOP -------------------
    # =====================================================

    elif mode == "CURVED":

        ret, frame = curved_video.read()

        # LOOP CURVED VIDEO FOREVER
        if not ret:
            curved_video.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        processed = preprocessing_curved(frame)

        h, w = processed.shape

        polygon = [
            (int(w*0.15), int(h*0.94)),
            (int(w*0.45), int(h*0.62)),
            (int(w*0.58), int(h*0.62)),
            (int(0.95*w), int(0.94*h))
        ]

        masked = regionOfInterest(processed, polygon)

        src = np.float32([
            [int(w*0.49), int(h*0.62)],
            [int(w*0.58), int(h*0.62)],
            [int(w*0.15), int(h*0.94)],
            [int(0.95*w), int(0.94*h)]
        ])

        dst = np.float32([
            [0,0],
            [400,0],
            [0,960],
            [400,960]
        ])

        warped = warp(masked, src, dst, (400,960))

        opening = cv2.morphologyEx(
            warped,
            cv2.MORPH_CLOSE,
            np.ones((11,11), np.uint8)
        )

        left_fit, right_fit = fitCurve(opening)

        pts_l, pts_r = findPoints(
            (960,400),
            left_fit,
            right_fit
        )

        filled = fillCurves(
            (960,400),
            pts_l,
            pts_r
        )

        unwarped = unwarp(
            filled,
            src,
            dst,
            (w,h)
        )

        overlay = cv2.addWeighted(
            frame,
            1,
            unwarped,
            1,
            0
        )

        steering_angle = calculate_steering_angle(
            left_fit,
            right_fit,
            w,
            h
        )

        cv2.putText(
            overlay,
            f"CURVED MODE : {int(steering_angle)}",
            (40,50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255,255,255),
            2
        )

        # SEND UART
        try:
            ser.write(f"{int(steering_angle)}\n".encode())
        except:
            pass

        cv2.imshow("Lane Detection", overlay)

    # =====================================================
    # FRAME TIMING
    # =====================================================

    elapsed = time.time() - start

    if elapsed < FRAME_TIME:
        time.sleep(FRAME_TIME - elapsed)

    # EXIT
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# =========================================================
# CLEANUP
# =========================================================

straight_video.release()
curved_video.release()

cv2.destroyAllWindows()