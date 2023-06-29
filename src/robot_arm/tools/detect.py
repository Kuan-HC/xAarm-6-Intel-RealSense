import cv2
import numpy as np
import math

class qrCodeDetect:
    def __init__(self):
        self.status = None
        self.sift = cv2.SIFT_create()

        # fEATURE MATHCING
        self.index_params = dict(algorithm=1, trees=5)
        self.search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(self.index_params, self.search_params)
        self.bfMatcher = cv2.BFMatcher()

    def re_size_photo(self,img, scale):
        # percent of original size
        w = int(img.shape[1] / scale)
        h = int(img.shape[0] / scale)
        image = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        return image
    
    def setRefImg(self, imgId = 0):
        imgPath = "./ref_images/" + str(imgId) + ".jpg" 
        refImg = cv2.imread(imgPath, cv2.IMREAD_GRAYSCALE) 
        self.refImg = self.re_size_photo(refImg, 4) 
        self.kp_image, self.desc_image = self.sift.detectAndCompute(self.refImg, None)

    def get_angle(self, rPos, lPos, real_rpos, real_lpos):
        #vector OC
        img_lr = [rPos[0] - lPos[0], rPos[1] - lPos[1]]
        #vector LR
        real_lr = [real_rpos[0] - real_lpos[0], real_rpos[1] - real_lpos[1]]

        ocLen = (img_lr[0]**2 + img_lr[1]**2)**0.5
        lrLen = (real_lr[0]**2 + real_lr[1]**2)**0.5
        dot = img_lr[0] * real_lr[0] + img_lr[1] * real_lr[1]

        theta = math.acos(dot / (ocLen * lrLen)) * 180 / math.pi
        
        if real_lpos[1]  > real_rpos[1]:
            return -theta
        
        return theta   


    def findQRcode(self,rgbImage):
        '''
        creat reference image detrector and descriptor
        '''
        #kp_image, desc_image = self.sift.detectAndCompute(self.refImg, None)

        '''
        creat input image detrector and descriptor
        '''
        
        kp_grayframe, desc_grayframe = self.sift.detectAndCompute(rgbImage, None)

        #matches = self.flann.knnMatch(self.desc_image, desc_grayframe, k=2)
        matches = self.bfMatcher.knnMatch(self.desc_image, desc_grayframe, k=2)
        good_points = []

        for m, n in matches:
            if m.distance < 0.5 * n.distance:
                good_points.append(m)
        
        drawMatches = False
        if drawMatches == True:
            img3 = cv2.drawMatches(self.refImg, self.kp_image, rgbImage, kp_grayframe, good_points, rgbImage,
                               flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
            
            cv2.namedWindow('MatchesPoints', cv2.WINDOW_NORMAL)
            cv2.imshow('MatchesPoints', img3)
            cv2.waitKey(1)  

        if len(good_points) > 80:
            query_pts = np.float32([self.kp_image[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
            train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)

            matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)

            h, w = self.refImg.shape

            pts = np.float32([[0, 0], [0, h], [w, h], [w, 0],[w/2, h/2]]).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts, matrix)
            theta = self.get_angle([0, 0], [w, 0],dst[0][0], dst[3][0])
            
            
            return  int(dst[4][0][0]), int(dst[4][0][1]), theta
        
        return None, None, None


