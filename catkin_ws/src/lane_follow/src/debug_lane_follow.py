import sys, time
import cv2
# from lane_follower import LaneFollower
from img_helpers import BirdsEye, Treshold, FitPolynomial

def test_birdsEye(image):
    fx = 742.64941
    fy = 742.64941
    cx = 666.99677
    cy = 517.3488

    # Camera position (in metres, degrees)
    H = 0.21
    theta = 0.00

    # Defining desired field-of-view (in metres)
    # Ox and Oy are the position of the projection of the optical center on the 
    # ground plane from (0, 0) of the top-view image
    # Wx and Wy are the width and height of the top-view image
    # x-axis is the direction pointing right in the top-view image
    # y-axis is the direction pointing down in the top-view image
    Wx = 4.8
    Wy = 3.6
    Ox = Wx / 2
    Oy = Wy    

    # Scaling factor (in pixels/m)
    # (Use 80.0 for realtime low-res output but 160.0 for datasets)
    s = 100.0

    birdsEyeImage = BirdsEye((fx, fy), (cx, cy), H, theta, (Ox, Oy), (Wx, Wy), s)    
    birdsEyeImage.warpPerspective(image)

    return birdsEyeImage.birds_image

def test_treshold(image):
    tresholdImage = Treshold()
    tresholdImage.treshold_binary(image)
    return tresholdImage.image

def test_fit_polynomial(image):
    fitPolynomial = FitPolynomial()
    fitPolynomial.fit_polynomial(image)
    return fitPolynomial.left_fit, fitPolynomial.right_fit

def test_get_waypoints(image):
    fitPolynomial = FitPolynomial()
    fitPolynomial.fit_polynomial(image)
    fitPolynomial.get_waypoints()

    return fitPolynomial.center_array

def main(args):
    image_np = cv2.imread('C:/tmp/img/2.jpg')
    # testing binary tresholding
    treshold_image = test_treshold(image_np)
    cv2.imwrite('C:/tmp/img/treshold_image.jpg', treshold_image)
    image_np = treshold_image

    # testing warp functions
    birds_image = test_birdsEye(image_np)
    cv2.imwrite('C:/tmp/img/birds_image.jpg', birds_image)
    image_np = birds_image

    # testing fitting polynomial
    left_fit, right_fit = test_fit_polynomial(image_np)
    print("left fit", str(left_fit))
    print("right fit", str(right_fit))

    # testing getting new waypoints
    waypoints = test_get_waypoints(image_np)
    print("waypoints", waypoints)
    
    
if __name__ == '__main__':
    main(sys.argv) 