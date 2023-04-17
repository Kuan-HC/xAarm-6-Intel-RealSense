import cv2
import numpy as np
import os.path

class qrCodeDetect:
    def __init__(self):
        self.status = None
        self.sift = cv2.SIFT_create()

        # fEATURE MATHCING
        self.index_params = dict(algorithm=1, trees=5)
        self.search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)

    def re_size_photo(self,img, scale):
        # percent of original size
        w = int(img.shape[1] / scale)
        h = int(img.shape[0] / scale)
        image = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        return image
    
    def setRefImg(self, imgId = 0):
        imgPath = "./ref_images/" + str(imgId) + ".jpg" 
        print(imgPath)
        refImg = cv2.imread(imgPath, cv2.IMREAD_GRAYSCALE) 
        self.refImg = self.re_size_photo(refImg, 10)      


    def findQRcode(self,rgbImage):
        '''
        creat reference image detrector and descriptor
        '''
        kp_image, desc_image = self.sift.detectAndCompute(self.refImg, None)

        '''
        creat input image detrector and descriptor
        '''
        kp_grayframe, desc_grayframe = self.sift.detectAndCompute(rgbImage, None)

        matches = self.flann.knnMatch(desc_image, desc_grayframe, k=2)
        good_points = []

        for m, n in matches:
            if m.distance < 0.73 * n.distance:
                good_points.append(m)
        
        drawMatches = False
        if drawMatches == True:
            img3 = cv2.drawMatches(self.refImg, kp_image, rgbImage, kp_grayframe, good_points, rgbImage,
                               flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            
            cv2.namedWindow('MatchesPoints', cv2.WINDOW_NORMAL)
            cv2.imshow('MatchesPoints', img3)
            cv2.waitKey(1)  


        if len(good_points) > 15:
            query_pts = np.float32([kp_image[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
            train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)

            matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)

            h, w = self.refImg.shape

            pts = np.float32([[0, 0], [0, h], [w, h], [w, 0],[w/2, h/2]]).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts, matrix)
            
            return  int(dst[4][0][0]), int(dst[4][0][1])
        
        return None, None

