import cv2
import mediapipe as mp
import time

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
points = [11, 12]


def get_points(image, show=True):
    with mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as pose:
        # Flip the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
        # image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = pose.process(image)

        # Draw the pose annotation on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(
            image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        if show:
            out_win = "output_style_full_screen"
            cv2.namedWindow(out_win, cv2.WINDOW_NORMAL)
            # cv2.setWindowProperty(out_win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.imshow(out_win, image)
            if cv2.waitKey(20) & 0xFF == 27:
                cv2.destroyAllWindows()
                cap.release()

        if results.pose_landmarks is not None:
            key_points = [0, 0, 0, 0]
            key_points[0] += results.pose_landmarks.landmark[0].x
            for i in points:
                key_points[1] += results.pose_landmarks.landmark[i].y
            key_points[1] /= len(points)
        else:
            key_points = None
        return key_points


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            cv2.destroyAllWindows()
            cap.release()
            break
        key_point = get_points(image, show=True)
        # print(key_point)
