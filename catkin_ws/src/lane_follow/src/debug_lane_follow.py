import sys, time
import cv2
import math
# from lane_follower import LaneFollower
from img_helpers import BirdsEye, Treshold, FitPolynomial
from lane_follower import LaneFollower

def test_birdsEye(image):
    Wx = 4.8
    Wy = 3.6
    s = 100
    C = ( 641.408166, 397.025688)
    F = (944.023585, 944.023585)
    H = 0.36
    pitch = 0
    birdsEyeImage = BirdsEye(F, C, H, pitch, (Wx/2, Wy+1.2), (Wx, Wy), s)    
    birdsEyeImage.warpPerspective(image)

    return birdsEyeImage.birds_image

def test_treshold(image):
    tresholdImage = Treshold()
    tresholdImage.treshold_binary(image)
    return tresholdImage.image

def test_fit_polynomial(image):
    fitPolynomial = FitPolynomial()
    fitPolynomial.fit_polynomial(image, True)
    # fitPolynomial.draw_filled_polygon()
    return fitPolynomial.left_fit, fitPolynomial.right_fit, fitPolynomial.poly_img

def test_get_waypoints(image):
    fitPolynomial = FitPolynomial()
    fitPolynomial.fit_polynomial(image)
    fitPolynomial.get_waypoints()

    return fitPolynomial.center_array

def test_lane_follow(image):
    Wx = 4.8
    Wy = 3.6
    s = 100
    C = ( 641.408166, 397.025688)
    F = (944.023585, 944.023585)
    H = 0.36
    pitch = 0
    lane_follow = LaneFollower(Wx, Wy, s)
    lane_follow.setCameraInfo(C, F)
    lane_follow.setCameraPos(H, pitch)
    lane_follow(image)
    return lane_follow.birdsEyeImage.birds_image

def main(args):
    image_np = cv2.imread('C:/tmp/img/1.jpg')    
    # testing binary tresholding
    treshold_image = test_treshold(image_np)
    # cv2.imwrite('C:/tmp/img/treshold_image.jpg', treshold_image)
    
    # testing warp functions
    birds_image = test_birdsEye(treshold_image)
    # cv2.imwrite('C:/tmp/img/birds_image.jpg', birds_image)
    
    # testing fitting polynomial
    left_fit, right_fit, poly_img = test_fit_polynomial(birds_image)
    print("left fit", str(left_fit))
    print("right fit", str(right_fit))
    cv2.imwrite('C:/tmp/img/poly.jpg', poly_img)

    # testing getting new waypoints
    waypoints = test_get_waypoints(birds_image)
    print("waypoints", waypoints)

    # testing whole laneFollower
    # image_test = cv2.imread('C:/tmp/img/1.jpg')
    # lf_img = test_lane_follow(image_test)
    # cv2.imwrite('C:/tmp/img/lf_img.jpg', lf_img)
    
    
if __name__ == '__main__':
    main(sys.argv) 