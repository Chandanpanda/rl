import cv2
import numpy as np

def pooling(mat,ksize,method='max',pad=False):
    '''Non-overlapping pooling on 2D or 3D data.

    <mat>: ndarray, input array to pool.
    <ksize>: tuple of 2, kernel size in (ky, kx).
    <method>: str, 'max for max-pooling, 
                   'mean' for mean-pooling.
    <pad>: bool, pad <mat> or not. If no pad, output has size
           n//f, n being <mat> size, f being kernel size.
           if pad, output has size ceil(n/f).

    Return <result>: pooled matrix.
    '''

    m, n = mat.shape[:2]
    ky,kx=ksize

    _ceil=lambda x,y: int(np.ceil(x/float(y)))

    if pad:
        ny=_ceil(m,ky)
        nx=_ceil(n,kx)
        size=(ny*ky, nx*kx)+mat.shape[2:]
        mat_pad=np.full(size,np.nan)
        mat_pad[:m,:n,...]=mat
    else:
        ny=m//ky
        nx=n//kx
        mat_pad=mat[:ny*ky, :nx*kx, ...]

    new_shape=(ny,ky,nx,kx)+mat.shape[2:]

    if method=='max':
        result=np.nanmax(mat_pad.reshape(new_shape),axis=(1,3))
    else:
        result=np.nanmean(mat_pad.reshape(new_shape),axis=(1,3))

    return result
    
image = cv2.imread('mask.png')
mask = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cv2.imshow("Image",mask)
cv2.waitKey(0) # waits until a key is pressed

#Average Pooling
mask = pooling(mask, (2,2),'mean',False)
mask = mask.astype(np.uint8)
cv2.imshow("Image",mask)
cv2.waitKey(0) # waits until a key is pressed

#Convolution
kernel = np.ones((10, 10), np.float32)
kernel = kernel/kernel.size
mask = cv2.filter2D(mask, -1, kernel=kernel)
mask = mask.astype(np.uint8)
cv2.imshow("Image",mask)
cv2.waitKey(0) # waits until a key is pressed

#Thresholding
mask = cv2.threshold(mask, 100, 255, cv2.THRESH_BINARY)[1]
cv2.imshow("Image",mask)
cv2.waitKey(0) # waits until a key is pressed

#Clustering


#Bounding Rectangle
x, y, w, h = cv2.boundingRect(mask)
cv2.rectangle(mask, (x, y), (x + w, y + h), (255, 0, 255), 1)
cv2.imshow("Image",mask)
cv2.waitKey(0) # waits until a key is pressed
cv2.destroyAllWindows() # destroys the window showing image