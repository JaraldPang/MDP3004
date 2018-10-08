import numpy
import cv2 as cv
from matplotlib import pyplot as plt
from timeit import default_timer as timer

def show_image(img):
    cv.imshow('image', img)
    keycode = cv.waitKey(0)
    cv.destroyAllWindows()
#a reminder of contour highlighting
def contour_test():
    img = cv.imread('assets/20cm.jpg', opencv.IMREAD_UNCHANGED)
    pass
#we can use adaptive histogram to increase contrast, but don't think needed
def histogram_equalization():
    pass
#HAAR cascade trial
def cascade_trial():
    pass
#basic numpy fast lookup
def numpy_test():

    #array of 16 numbers, arrange in 2 dimensions consisting of 4 rows with 4 columns
    arr = numpy.arange(16).reshape([4,4])
    '''
        numpy array slicing works by evaluating the dimension to work in first, then by the range to cut
        [1-dimension,2-dimension,...,nth-dimension]
        [1-dimension 1:3 - slices from index 1 to index 3,2-dimension,...,nth-dimension]
    '''
    #access row 1 and 2
    #arr[1:3]
    #access from row 1 and 2, all elements in index 3 and return as an array
    #arr[1:3, 3]

    #OMG WTF
    #(channel_b, channel_g, channel_r) = (img[:,:,0], img[:,:,1], img[:,:,2])

    '''
        ELI5 of colour channels
        channels represent the amount of coloured light required for the picture, in that channel's colour
        in a grayscale image showing only 1 channel, the brightest areas represents the most amount of coloured light required for that area in the current channel
    '''

    '''
        img[:,:,2] = 0
        select the entire height, and for each height row select the entire column, and for each height column select the channel value
        basically, selects the red channel value for all pixes and sets it to 0
    '''

    '''
        cv2.threadhold retval has meaning only if otsu's binarization is used. the retval is the optimal threshold value found by the binarization only if 
        otsu's binarization is used, otherwise it returns the original threshold you passed it
    '''

    pass
#a helper method
def draw_bounding_box(img, x, y, x_plus_w, y_plus_h):

    cv.rectangle(img, (x,y), (x_plus_w,y_plus_h),(0,255,0), 2)

    return img

    #cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def test_sift_matching():
    MIN_MATCH_COUNT = 5
    img1 = cv.imread('assets/arrow.jpg', cv.IMREAD_GRAYSCALE)  # queryImage
    #img1 = cv.Laplacian(img1, cv.CV_8U)
    #img1 = cv.blur(img1, (9, 9))
    #img1 = cv.adaptiveThreshold(img1, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 11, 2)
    #img1 = cv.bilateralFilter(img1,9,150,150)


    img2 = cv.imread('assets/20cm.jpg', cv.IMREAD_GRAYSCALE)  # trainImage
    #img2 = cv.blur(img2, (11, 11))
    #img2 = cv.bilateralFilter(img2,9,75,75)
    #img2 = cv.adaptiveThreshold(img2, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 11, 2)

    # Initiate SIFT detector
    sift = cv.xfeatures2d.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)

    #bf = cv.BFMatcher(cv.NORM_L2, crossCheck=True)

    # Match descriptors.
    #matches = bf.match(des1, des2)
    # Sort them in the order of their distance.
    #matches = sorted(matches, key=lambda x: x.distance)

    ## match descriptors and sort them in the order of their distance
    bf = cv.BFMatcher(cv.NORM_L2, crossCheck=True)
    matches = bf.match(des1, des2)

    dmatches = sorted(matches, key = lambda x:x.distance)

    ## extract the matched keypoints
    src_pts  = numpy.float32([kp1[m.queryIdx].pt for m in dmatches]).reshape(-1,1,2)
    dst_pts  = numpy.float32([kp2[m.trainIdx].pt for m in dmatches]).reshape(-1,1,2)

    ## find homography matrix and do perspective transform
    M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
    h,w = img1.shape[:2]
    pts = numpy.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv.perspectiveTransform(pts,M)

    ## draw found regions
    img2 = cv.polylines(img2, [numpy.int32(dst)], True, 255, 3, cv.LINE_AA)

    img3 = cv.drawMatches(img1, kp1, img2, kp2, dmatches[:20],None,flags=2)

    plt.imshow(img3), plt.show()

def test_orb_matching():
    kernel = numpy.ones((5, 5), numpy.uint8)
    img1 = cv.imread('assets/arrow.jpg', cv.IMREAD_GRAYSCALE)  # queryImage
    img1 = cv.bilateralFilter(img1, 9, 150, 150)
    img1 = cv.adaptiveThreshold(img1, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 11, 2)
    img1 = cv.blur(img1, (9, 9))


    img2 = cv.imread('assets/30cm.jpg', cv.IMREAD_GRAYSCALE)  # queryImage
    img2 = cv.bilateralFilter(img2, 9, 75, 75)
    img2 = cv.adaptiveThreshold(img2, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 11, 2)
    img2 = cv.bilateralFilter(img2, 9, 75, 75)
    ret, img2 = cv.threshold(img2, 127, 255, cv.THRESH_BINARY_INV)
    img2 = cv.dilate(img2, kernel, iterations=1)
    ret, img2 = cv.threshold(img2, 127, 255, cv.THRESH_BINARY_INV)
    img2 = cv.blur(img2,(9,9))

    #use ORB algorithm as the feature descriptora
    #a feature descriptor finds the features or unique points of your target image
    orb = cv.ORB_create()

    #find compute the descriptors with ORB
    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)

    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

    # Match descriptors.
    matches = bf.match(des1, des2)
    # Sort them in the order of their distance.
    matches = sorted(matches, key=lambda x: x.distance)


    # Draw first 10 matches.
    img3 = cv.drawMatches(img1, kp1,img2, kp2,matches[:10],None, flags=2)

    plt.imshow(img3), plt.show()

def test_sift_matching_with_flann():
    MIN_MATCH_COUNT = 10
    img1 = cv.imread('assets/arrow.jpg', cv.IMREAD_GRAYSCALE)  # queryImage
    img1 = cv.Laplacian(img1, cv.CV_8U)
    img1 = cv.blur(img1, (9, 9))
    img1 = cv.adaptiveThreshold(img1, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 11, 2)
    img1 = cv.bilateralFilter(img1,9,150,150)


    img2 = cv.imread('assets/20cm.jpg', cv.IMREAD_GRAYSCALE)  # trainImage
    img2 = cv.blur(img2, (11, 11))
    #img2 = cv.bilateralFilter(img2,9,75,75)
    img2 = cv.adaptiveThreshold(img2, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 11, 2)



    # Initiate SIFT detector
    sift = cv.xfeatures2d.SIFT_create()

    # find the keypoints and descriptors with SIFT
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)

    flann = cv.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1, des2, k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good.append(m)

    if len(good) > MIN_MATCH_COUNT:
        src_pts = numpy.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = numpy.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        h, w = img1.shape
        pts = numpy.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
        dst = cv.perspectiveTransform(pts, M)

        img2 = cv.polylines(img2, [numpy.int32(dst)], True, 255, 3, cv.LINE_AA)

    else:
        print("Not enough matches are found - %d/%d" % (len(good), MIN_MATCH_COUNT))
        matchesMask = None

    draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                       singlePointColor=None,
                       matchesMask=matchesMask,  # draw only inliers
                       flags=2)

    img3 = cv.drawMatches(img1, kp1, img2, kp2, good, None, **draw_params)

    #plt.imshow(img3, 'gray')
    plt.imshow(img3)
    plt.show()


'''
target arrow - sift
1) blur = few nonsensical keypoints at random locations
2) bilateral filter = few nonsensical keypoints at random locations
3) adaptive threshold = majority keypoints on top right edge, insufficient
4) bilateral filter -> adaptive threshold = less keypoints than 3), insufficient
5) bilateral filter -> adaptive threshold -> blur = few keypoints but at corners, seems viable
6) blur -> adaptive threshold = some random keypoints, most at corners but not all corners
7) blur -> adaptive threshold -> bilateral filter = some random keypoitns, but all corners have keypoints, seems viable


8) lapacian -> bilateral filter -> adaptive threshold -> blur = few keypoints but at corners, keypoints seem more randomly distributed. edges were not clearly defined
9) lapacian -> blur -> adaptive threshold -> bilateral filter = edges were clearly defined. many keypoints on top left and right edge, as well as corners, most viable
10) blur -> adaptive threshold -> bilateral filter -> lapacian = image inversion corners wrecked gg don'try
11) blur -> adaptive threshold -> lapacian -> bilateral filter = edges faded gg don't yry
12) blur -> lapacian -> adaptive threshold -> bilateral filter  = edges faded but keypoitns at corners. not as viable

Candidate approach - 9
'''
def test_sift_keypoints_on_target():
    # find the keypoints and descriptors with SIFT
    img1 = cv.imread('assets/arrow.jpg', cv.IMREAD_GRAYSCALE)  # queryImage
    img1 = cv.Laplacian(img1, cv.CV_8U)
    img1 = cv.blur(img1, (9, 9))
    img1 = cv.adaptiveThreshold(img1, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 11, 2)
    img1 = cv.bilateralFilter(img1, 9, 150, 150)

    #img1 = cv.bilateralFilter(img1,9,150,150)
    #img1 = cv.blur(img1, (9, 9))


    # Initiate SIFT detector
    sift = cv.xfeatures2d.SIFT_create()

    kp1, des1 = sift.detectAndCompute(img1, None)
    print(len(kp1))

    img2 = cv.drawKeypoints(img1, kp1,None, flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    plt.imshow(img2), plt.show()

'''
SIFT seems to detect too many keypoints. we may need to cutdown the image.

input image - sift
1) adaptive threshold = heavy noise
2) blur -> adaptive threshold = slight noise, but rough edges, arrow corners eroded, keypoints on arrows but not enough; background has too many
3) blur -> adaptive threshold -> bilateral filter 75,75,= no observable difference from 2)
4) blur -> adaptive threshold -> bilateral filter 140,140,= no observable difference from 2)
5) bilateral filter 75,75 -> adaptive threshold = some keypoints at arrow, arrow corners eroded
5) bilateral filter 140,140-> adaptive threshold = no diff from 5)
6) bilateral filter 75,75 -> adaptive threshold -> bilateral filter 75,75 = no diff from 6)
7) bilateral filter 140,140 -> adaptive threshold -> bilateral filter 140,140 = corners eroded, but more keypoints than 5), viable
8) bilateral filter 75,75 -> adaptive threshold -> blur = smoothed edges, but few key points at arrow and mostly on corners. reduced number of noisy keypoints however.
9) bilateral filter 75,75 -> adaptive threshold -> blur(11,11) = slightly lesser keypoints on arrow from 9)
10) bilateral filter 140,140 -> adaptive threshold -> blur = no differnce from 9)


lapacian is not useful
11) lapacian after adaptive at any point results in inverted image
12) lapacian -> bilateral filter 140,140 -> adaptive threshold -> bilateral filter 140,140 = complete arrow erosin, gg dead
13) lapacian -> bilateral filter 75,75 -> adaptive threshold -> blur(11,11) = complete arrow erosion, GG DEAD
14) lapacian -> blur -> adaptive threshold -> bilateral filter 140,140,= compelte arrow eerrosion

'''
def test_sift_keypoints_on_input():
    # find the keypoints and descriptors with SIFT
    img1 = cv.imread('assets/20cm.jpg', cv.IMREAD_GRAYSCALE)  # queryImage

    img1 = cv.bilateralFilter(img1,9,140,140)
    #img1 = cv.Laplacian(img1, cv.CV_8U)
    #img1 = cv.blur(img1, (9, 9))

    img1 = cv.adaptiveThreshold(img1, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 11, 2)
    img1 = cv.bilateralFilter(img1,9,140,140)
    #img1 = cv.blur(img1, (9, 9))
    # Initiate SIFT detector
    sift = cv.xfeatures2d.SIFT_create()

    kp1, des1 = sift.detectAndCompute(img1, None)
    print(len(kp1))

    img2 = cv.drawKeypoints(img1, kp1,None, flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    plt.imshow(img2), plt.show()

'''
target arrow - orb
1) bilateral filter -> adaptive threshold -> blur = keypoints at corners
2) bilateral filter -> adaptive threshold = keypoints at corner, top left and right edges with left edge heavily clustered
3) blur -> adaptive threshold = keypoints at corner, inner and outer side of top left and right edges with even distribution
4) blur -> adaptive threshold -> bilateral filter = keypoints at corner, and inner side of top left and right edges with even distribution, 312 keypoints. angled corners eroded

5) lapacian -> blur -> adaptive threshold -> bilateral filter = keypoints at corner, and at both inner and outer sides of top left and right edges with even distribuition. angled corners eroded 488 keypoints, no observable difference to 3)
6) blur -> adaptive threshold -> bilateral filter -> lapacian = keypoints at corner, top left and right edges with even distribution, but image inverted colours, gg dead
7) blur -> adaptive threshold -> lapacian -> bilateral filter = keypoints at corner, and inner side of top left and right edges with even distribution, but image colours inverted gg dead
8) blur -> lapacian -> adaptive threshold -> bilateral filter  = keypoints at corners and top edges, but top left and right edges are heavily eroded

dilation
9) lapacian -> blur -> adaptive threshold -> bilateral filter -> dilate = keypoints at corner, edges with uneven distribution; more keypoints on right edge. 484 keypoints. rough edges
10) lapacian -> blur -> adaptive threshold -> bilateral filter -> dilate -> blur = keypoints at corner only, 244 keypoints.
10) lapacian -> blur -> adaptive threshold -> bilateral filter -> dilate -> blur(7,7) = keypoints at corner, with some keypoints on right edge, 278 keypoints.
'''
def test_orb_keypoints_on_target():
    kernel = numpy.ones((5, 5), numpy.uint8)
    img1 = cv.imread('assets/arrow.jpg', cv.IMREAD_GRAYSCALE)  # queryImage

    #img1 = cv.Laplacian(img1, cv.CV_8U)
    #bilateral filter appears to remove noise with the least effect to corners
    img1 = cv.bilateralFilter(img1, 9, 140, 140)
    #img1 = cv.blur(img1, (9, 9))
    # this value is the minimum value that removed the dots on the right, but no effect afterwards  due to blur
    #img1 = cv.adaptiveThreshold(img1, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 13, 9)
    #
    img1 = cv.adaptiveThreshold(img1, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 11, 2)

    #blur is supposed to remove noise
    #higher appears to be better. gaussian and normal no difference
    img1 = cv.blur(img1, (9, 9))
    #img1 = cv.bilateralFilter(img1,9,150,150)

    #erosion, dilation operates on white foreground objects and black background, hence we invert
    #ret, img1 = cv.threshold(img1, 127, 255, cv.THRESH_BINARY_INV)
    #img1 = cv.dilate(img1, kernel, iterations=1)
    #ret, img1 = cv.threshold(img1, 127, 255, cv.THRESH_BINARY_INV)

    orb = cv.ORB_create()
    #find compute the descriptors with ORB
    kp1, des1 = orb.detectAndCompute(img1,None)
    #print num of keypoints
    print(len(kp1))

    # draw only keypoints location,not size and orientation
    img2 = cv.drawKeypoints(img1, kp1,None, color=(0, 255, 0), flags=0)
    plt.imshow(img2,'gray')

    plt.show()
    pass


'''
input image - orb
1) adaptive threshold = heavy noise
2) blur -> adaptive threshold = slight noise, but rough edges, arrow corners eroded
3) blur -> adaptive threshold -> bilateral filter 75,75,= slight noise, but not enough keypoints at arrow , arrow corners eroded
4) bilateral filter 75,75 -> adaptive threshold = slight noise, not enough keypoints at arrow, arrow corners slightly eroded
5) bilateral filter 140,140-> adaptive threshold = slight noise, not enough keypoints at arrow, arrow corners  slitghly eroded
6) bilateral filter 75,75 -> adaptive threshold -> bilateral filter 75,75 = slight noise, not enough keypoints at arrow, arrow corners eroded
7) bilateral filter 140,140 -> adaptive threshold -> bilateral filter 140,140 = slight noise, not enough keypoints at arrow, arrow corners eroded
8) bilateral filter 75,75 -> adaptive threshold -> blur = slight noise, not enough keypoints at arrow
9) bilateral filter 75,75 -> adaptive threshold -> blur(11,11) = slight noise, not enough keypoints at arrow, more keypoints found but not enough
10) lapacian after adaptive at any point results in inverted image
11) bilateral filter 75,75 -> lapacian -> adaptive threshold -> blur(11,11) = slight noise, edges eroded, no keypoints at arrow, ggead
12) lapacian -> bilateral filter 75,75 -> adaptive threshold -> blur(11,11) = complete arrow erosion, GG DEAD
'''
def test_orb_keypoints_on_input():
    kernel = numpy.ones((5, 5), numpy.uint8)
    img1 = cv.imread('assets/30cm.jpg', cv.IMREAD_GRAYSCALE)  # queryImage

    img1 = cv.bilateralFilter(img1, 9, 75, 75)
    #img1 = cv.blur(img1, (9, 9))


    img1 = cv.adaptiveThreshold(img1, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 11, 2)

    #img1 = cv.blur(img1, (11, 11))
    img1 = cv.bilateralFilter(img1, 9, 75, 75)

    #img1 = cv.blur(img1, (11, 11))
    ret, img1 = cv.threshold(img1, 127, 255, cv.THRESH_BINARY_INV)
    img1 = cv.dilate(img1, kernel, iterations=1)
    #img1 = cv.erode(img1, kernel, iterations=1)
    ret, img1 = cv.threshold(img1, 127, 255, cv.THRESH_BINARY_INV)

    #higher appears to be better. gaussian and normal no difference
    img1 = cv.blur(img1, (9, 9))
    #img1 = cv.bilateralFilter(img1,9,150,150)

    orb = cv.ORB_create()

    #find compute the descriptors with ORB
    kp1, des1 = orb.detectAndCompute(img1,None)
    print(len(kp1))

    # draw only keypoints location,not size and orientation
    img2 = cv.drawKeypoints(img1, kp1,None, color=(0, 255, 0), flags=0)
    plt.imshow(img2,'gray')
    plt.show()


#required
if __name__ == '__main__':
    # execute only if run as a script
    #test_orb_matching()
    #test_sift_matching()
    #test_orb_keypoints_on_input()
    test_sift_matching()
    #test_orb_keypoints_on_target()
    #test_sift_keypoints_on_target()
    #test_sift_keypoints_on_input()
    print("Exiting...")



'''
conclusion
1) thresholding makes images easier for opencv to interpret
2) blurring is supposed to remove noise. 

tentative approach is
1) preprocess image by removing noise, thresholding to make it easier, then highlighting edges with highpass
2) extract feature of target. ensure feature extraction is the best there is
3) detect if feature exists in input
4) if feature exists, using the features, classify the features

'''