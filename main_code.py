# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 23:58:20 2018

@author: Sourav
"""

import cv2
import mouse
import serial
import numpy as np
from multiprocessing import Process
import math
import wx 



try:
    del app;
except:
    pass;                 
                    ###################################################################         
                    #    Press 'r'-to reset the background if it becomes so noisy.    #
                    #    Press 'esc'-to exit the software.                            #
                    ###################################################################
                          
                          
#function to reduce flickering
def isPointClose(x1,y1,x2,y2,scale):
    d=math.sqrt((x1-x2)**2+(y1-y2)**2);
#    print(d)
    if d<=scale:     #length<=scale?
        return True;
    else :
        return False; 

ser = serial.Serial('COM3', 9600, timeout=1)   ##### Arduino & ultrasonic sensor
def clickArduino():
    try:
        text=ser.readline()
        dis = int(text.decode())
        print(dis)
        return dis
    except:
        pass

#if __name__ == "__main__":
cap=cv2.VideoCapture(0);#'http://192.168.0.101:4747/mjpegfeed');
bg=cv2.flip(cap.read()[1],1);
w=np.shape(bg)[1];
h=np.shape(bg)[0];
bg=bg[1:h-199,250:w].copy();
app=wx.App(False);
(sx,sy)=wx.GetDisplaySize();
########################

#font = cv2.FONT_HERSHEY_SIMPLEX;
#low_range=np.array([123,112,4]);      #hand color thresholds
#high_range=np.array( [250,180,124]);

########################

counter=0;
temp_x=temp_y=0;

while True:
    frame=cv2.flip(cap.read()[1],1);
    roi=frame[1:h-199,250:w].copy();
    temp_roi=roi.copy();
    
    fmask=cv2.absdiff(bg,roi,0);
    fmask=cv2.cvtColor(fmask,cv2.COLOR_BGR2GRAY);
    fmask=cv2.threshold(fmask,10,255,0)[1];
     ####### Morphological Processing #########
#    fmask=cv2.erode(fmask,cv2.getStructuringElement(cv2.MORPH_ERODE,(2,2)),iterations=2);
    mask1=cv2.morphologyEx(fmask,cv2.MORPH_CLOSE,\
                           cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2)));
    mask1=cv2.erode(mask1,cv2.getStructuringElement(cv2.MORPH_ERODE,(2,2)),iterations=2);
    cv2.imshow('mask1',mask1);
    fg_frame=cv2.bitwise_and(roi,roi,mask=mask1);
    cv2.imshow('fg_frame',fg_frame);
    
    gr_frame=cv2.cvtColor(fg_frame,cv2.COLOR_BGR2GRAY);
    gr_frame=cv2.blur(gr_frame,(10,10));
    bw_frame=cv2.threshold(gr_frame,50,255,0)[1];
    
    ############ Tracking the hand contour ################
    
    con=cv2.findContours(bw_frame,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0];
    try:
        my_con=max(con,key=cv2.contourArea);
    except:
        my_con=np.array([[[1,0],[1,2],[2,3]]],dtype=np.int32);
        #pass;

    try:
        if cv2.contourArea(my_con)>90:
            
            hull=cv2.convexHull(my_con,True);
            
            leftmost = tuple(hull[hull[:,:,0].argmin()][0]) 
            rightmost = tuple(my_con[my_con[:,:,0].argmax()][0]) 
            topmost = tuple(hull[hull[:,:,1].argmin()][0]) 
            bottommost = tuple(my_con[my_con[:,:,1].argmax()][0])
            
            
            temp=bottommost[0]+30 #getting the bottom middle of the hand
            cv2.line(roi,(topmost[0],topmost[1]+10),(topmost[0],h-280),(0,10,225),2);
#            cv2.line(roi,leftmost,(topmost[0],bottommost[1]-80),(0,242,225),2);
            
            cv2.circle(roi,topmost,10,(255,0,0),2);
#            cv2.circle(roi,leftmost,5,(0,120,255),-1);
            cv2.circle(roi,(temp,bottommost[1]),5,(230,0,255),-1);
            
            ################### get original pixel location #############
            
            x=sx-((topmost[0]-50)*sx/(w-340));
            y=(topmost[1]*sy/(h-281));
            if(not isPointClose(topmost[0],topmost[1],temp_x,temp_y,3)):
                mouse.move(sx-x,y,absolute=True, duration=.03);
            temp_x=topmost[0];temp_y=topmost[1];

            dis = clickArduino()
            if dis<=50:
                #mouse.click(button='left');
                cv2.circle(roi,topmost,20,(0,255,0),-1);
                cv2.line(roi,(topmost[0],topmost[1]+25),(topmost[0],h-280),(0,10,225),2);

            cv2.putText(roi,str('%d,%d'%(sx-x,y)),topmost, cv2.FONT_HERSHEY_SIMPLEX, .5,(255,255,255),1,cv2.LINE_AA)

    except:
        pass;
    frame[1:h-199,250:w]=roi;
    cv2.rectangle(frame,(250,1),(w-1,h-200),(0,255,0),2);
    cv2.rectangle(frame,(300,1),(w-40,h-280),(255,0,0),2);
    cv2.imshow('frame',frame);
    if cv2.waitKey(2)==ord('r'):
        print('Background reset')
        bg=temp_roi;
    elif cv2.waitKey(2)==27:
        break;
#%%############# Releasing the resources ##############
cv2.destroyAllWindows();
cap.release();
ser.close()




