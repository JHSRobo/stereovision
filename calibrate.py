import cv2, sys, os
import glob

def main():
    path = "/home/jhsrobo/stereovision/img/calibration"

    # To add a flag, just add the flag and its description to the following dictionary 
    # To make the flag actually do something, you'll also need to add a conditional to
    # the end of main.
    flags = {"-g": "get_photos", "-c": "calibirate", "-gc": "get_photos and calibrate"}

    # Some iteration to print out all the flags in the instructions
    instructions = "1. Please enter a flag:"
    for flag in flags.keys():
        instructions += f"\n    {flag}: {flags[flag]}"
    instructions += "\n2. (Optional) Enter a port to receive the camera stream (defaults to 1111)"

    cur_flag = None
    port = "5000"

    length = len(sys.argv)
    if length < 2:
        exit_on_msg("Missing arguments", instructions)

    if length == 2 or length == 3:
        cur_flag = sys.argv[1] 
        if cur_flag not in flags.keys():
            exit_on_msg("Invalid Flag", instructions)

    if length == 3:
        port = sys.argv[2] 
        if not port.isdigit():
            exit_on_msg("Invalid Port", instructions)

    if length > 3:
        exit_on_msg("Too many arguments", instructions)

    if cur_flag == "-g":
        get_photos(path, port) 
    elif cur_flag == "-c":
        calibrate_on_photos(path)
    elif cur_flag == "-gc":
        get_photos(path, port)
        calibrate_on_photos(path)
    else:
        exit_on_msg("Unfinished Flag Implementation")

def exit_on_msg(msg, instructions):
    print(msg)
    sys.exit(instructions)

# Store images when space bar is pressed
def get_photos(path, port):
    # Remove old images
    os.system("rm -f /home/jhsrobo/stereovision/img/calibration/L/*.png")
    os.system("rm -f /home/jhsrobo/stereovision/img/calibration/R/*.png")

    # Continue requesting an image from the camera at the specified ip and port until received or one second passed
    cv2.namedWindow('calibration', cv2.WINDOW_AUTOSIZE)

    i = 1
    key = None

    while True:
        if key == 32: # Space bar
            vid_capture = cv2.VideoCapture(f"http://192.168.88.111:{port}/snapshot")
            ret, frame = vid_capture.read() 
            if not ret or frame is None:
                continue 

            cv2.imshow("calibration", frame)

            print(f"Received Frame {i}")

            height, width, _ = frame.shape 

            divider = int(width/2)

            # Left Frame
            cv2.imwrite(f"{path}/L/{i}.png", frame[0:height, 0:divider])
            # Right Frame
            frame2 = frame[0:height, divider:width]
            cv2.imwrite(f"{path}/R/{i}.png", frame[0:height, divider:width])

            vid_capture.release()

            i += 1
        elif key == 13 or key == 10: # Enter/return 
            return


        # Least readable code ever lol
        # Bascially waitKey returns a ton of bytes with the last byte being
        # a relevant keystroke. By using a bitwise-and operator here, we're 
        # just copying the last byte (the keystroke) and ignoring the rest.
        key = cv2.waitKey(1) & 0xff

# Find more info on the calibration process and functions at the following link 
# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
def calibrate_on_photos(path):
    # Inside corners of chessboard in rows by cols format
    board_dim = (7, 9) 

    # Get existing left and right images 
    left_imgs = list(sorted(glob.glob(f"{path}/L/*.png")))
    right_imgs = list(sorted(glob.glob(f"{path}/R/*.png")))
    # Make sure that each image has a matching copy 
    assert len(left_imgs) == len(right_imgs)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    left_points = []
    right_points = []

    # Get size of images 
    img = left_imgs[0]
    img_size = (img.shape[1], img.shape[0])

    for lpath, rpath in zip(left_imgs, right_imgs):
        left_img = cv2.imread(lpath, cv2.IMREAD_GRAYSCALE)
        right_img = cv2.imread(rpath, cv2.IMREAD_GRAYSCALE)

        left_ret, left_corners = cv2.findChessboardCorners(left_img, board_dim)
        right_ret, right_corners = cv2.findChessboardCorners(right_img, board_dim)
        if left_ret and right_ret:
            left_corners = cv2.cornerSubPix(left_img, left_corners, (11, 11), (-1, -1), criteria)
            right_corners = cv2.cornerSubPix(right_img, right_corners, (11, 11), (-1, -1), criteria)
        else:
            print(f"Failed to find chesboard corners at \n{lpath}\n{rpath}")
            continue

        left_points.append(left_corners)
        right_points.append(right_corners)


        # Makes an array of points found and estimates the x and y coordinates (assumes z-coordinate of 0)

        # Note: Due to the assumption of a 0 z-coordinate, the chessboard must be placed on a flat plane.
        # I reccomend just downloading a chessboard, printing it, and putting it flat on a table.
        common_points = np.zeros((np.prod(board_dim), 3), np.float32)
        common_points[:, :2] = np.indices(board_dim).T.reshape(-1, 2)
        common_points = [common_points] * len(left_imgs)

        # Calibrate the camera modules based on the estimated position of the points
        error, Kl, Dl, Kr, Dr, R, T, E, F = cv2.stereoCalibrate(common_points, left_pts, right_pts, None, None, None, None, img_size, flags=0)

        print('Left camera:')
        print(Kl)
        print('Left camera distortion:')
        print(Dl)
        print('Right camera:')
        print(Kr)
        print('Right camera distortion:')
        print(Dr)
        print('Rotation matrix:')
        print(R)
        print('Translation:')
        print(T)
             

if __name__ == "__main__":
    main()
